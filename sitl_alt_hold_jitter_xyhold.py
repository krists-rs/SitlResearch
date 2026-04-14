#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import Optional

from pymavlink import mavutil


# ----------------------------
# RC helpers
# ----------------------------
def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def send_rc_override(master, ch1=0, ch2=0, ch3=0, ch4=0, ch5=0, ch6=0, ch7=0, ch8=0):
    """
    RC_CHANNELS_OVERRIDE:
      - 0 => release / no override on that channel
      - 1000..2000 => PWM override
    """
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        int(ch1), int(ch2), int(ch3), int(ch4), int(ch5), int(ch6), int(ch7), int(ch8)
    )


def release_all_overrides(master):
    send_rc_override(master, 0, 0, 0, 0, 0, 0, 0, 0)


# ----------------------------
# MAV helpers
# ----------------------------
def wait_heartbeat_or_die(master, timeout_s: float = 10.0):
    hb = master.wait_heartbeat(timeout=timeout_s)
    if hb is None:
        raise RuntimeError(f"No HEARTBEAT received in {timeout_s} seconds.")
    return hb


def set_mode_or_die(master, mode: str, timeout_s: float = 8.0):
    if mode not in master.mode_mapping():
        raise RuntimeError(
            f"Mode '{mode}' not supported by this vehicle/firmware. Available: {sorted(master.mode_mapping().keys())}"
        )

    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)

    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg is None:
            continue
        if int(getattr(msg, "custom_mode", -1)) == int(mode_id):
            return
    raise RuntimeError(f"Failed to confirm mode '{mode}' within {timeout_s}s.")


def arm_or_die(master):
    master.arducopter_arm()
    master.motors_armed_wait()


def disarm_or_die(master):
    master.arducopter_disarm()
    master.motors_disarmed_wait()


# ----------------------------
# Telemetry state (non-blocking pump)
# ----------------------------
@dataclass
class TelemetryState:
    # Relative altitude (m)
    alt_m: Optional[float] = None

    # Local position NED (meters): x=north, y=east, z=down
    x_n: Optional[float] = None
    y_e: Optional[float] = None

    # Local velocity NED (m/s): vx=north, vy=east
    vx_n: Optional[float] = None
    vy_e: Optional[float] = None

    # Attitude yaw (rad, [-pi..pi])
    yaw_rad: Optional[float] = None


def pump_telemetry(master, st: TelemetryState, max_msgs: int = 200):
    """
    Drain incoming MAVLink messages quickly and update latest telemetry values.
    """
    for _ in range(max_msgs):
        msg = master.recv_match(blocking=False)
        if msg is None:
            break

        mtype = msg.get_type()
        if mtype == "BAD_DATA":
            continue

        if mtype == "GLOBAL_POSITION_INT":
            st.alt_m = float(msg.relative_alt) / 1000.0

        elif mtype == "LOCAL_POSITION_NED":
            st.x_n = float(msg.x)
            st.y_e = float(msg.y)
            st.vx_n = float(msg.vx)
            st.vy_e = float(msg.vy)

        elif mtype == "ATTITUDE":
            st.yaw_rad = float(msg.yaw)


def wait_for_telemetry(master, st: TelemetryState, rate_hz: float = 30.0, timeout_s: float = 10.0):
    """
    Wait until we have at least altitude + local pos + yaw.
    """
    try:
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            int(max(1, rate_hz)),
            1,
        )
        print(f"[INFO] Requested data stream rate ~{rate_hz} Hz")
    except Exception as e:
        print(f"[WARN] request_data_stream_send failed (non-fatal): {e}")

    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        pump_telemetry(master, st)
        if st.alt_m is not None and st.x_n is not None and st.y_e is not None and st.yaw_rad is not None:
            return
        time.sleep(0.02)
    raise RuntimeError("Telemetry not ready (need GLOBAL_POSITION_INT, LOCAL_POSITION_NED, ATTITUDE).")


# ----------------------------
# Control config
# ----------------------------
@dataclass
class ControlConfig:
    # RC centers
    roll_center: int = 1500
    pitch_center: int = 1500
    yaw_pwm: int = 1500

    # Throttle (ALT_HOLD)
    thr_center: int = 1500
    thr_min: int = 1200
    thr_max: int = 2000
    kp_thr_pwm_per_m: float = 120.0

    # XY hold (local)
    # PD in "PWM per (meter or m/s)" after transforming NED->body
    kp_xy_pwm_per_m: float = 80.0
    kd_xy_pwm_per_mps: float = 50.0

    # Max stick deflection for XY correction (PWM delta from center)
    max_xy_delta_pwm: int = 220

    # Signs to match your RC channel direction
    # Common default: roll right -> PWM+, pitch forward -> PWM-  (so pitch_sign=-1)
    roll_sign: int = +1
    pitch_sign: int = -1

    # Override refresh rate
    rc_hz: float = 50.0


# ----------------------------
# Controllers
# ----------------------------
def compute_throttle_pwm(target_alt_m: float, alt_m: float, cfg: ControlConfig) -> int:
    err = target_alt_m - alt_m
    delta = cfg.kp_thr_pwm_per_m * err
    thr = cfg.thr_center + delta
    return int(clamp(thr, cfg.thr_min, cfg.thr_max))


def compute_xy_hold_pwm(st: TelemetryState, target_x_n: float, target_y_e: float, cfg: ControlConfig):
    """
    Returns (roll_pwm, pitch_pwm) to keep position near target in local NED, compensating drift.
    Error computed in NED then rotated into body frame using yaw.
    """
    if st.x_n is None or st.y_e is None or st.yaw_rad is None:
        return cfg.roll_center, cfg.pitch_center

    # Position error in NED
    n_err = target_x_n - st.x_n
    e_err = target_y_e - st.y_e

    # Velocities (if missing, treat as 0)
    vx_n = st.vx_n if st.vx_n is not None else 0.0
    vy_e = st.vy_e if st.vy_e is not None else 0.0

    # Rotate NED -> body (forward/right) by yaw
    # forward = +North when yaw=0, right = +East when yaw=0
    yaw = st.yaw_rad
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    f_err = cy * n_err + sy * e_err
    r_err = -sy * n_err + cy * e_err

    f_vel = cy * vx_n + sy * vy_e
    r_vel = -sy * vx_n + cy * vy_e

    # PD in body frame (PWM)
    pitch_delta = cfg.kp_xy_pwm_per_m * f_err - cfg.kd_xy_pwm_per_mps * f_vel
    roll_delta = cfg.kp_xy_pwm_per_m * r_err - cfg.kd_xy_pwm_per_mps * r_vel

    pitch_delta = clamp(pitch_delta, -cfg.max_xy_delta_pwm, cfg.max_xy_delta_pwm)
    roll_delta = clamp(roll_delta, -cfg.max_xy_delta_pwm, cfg.max_xy_delta_pwm)

    roll_pwm = cfg.roll_center + cfg.roll_sign * roll_delta
    pitch_pwm = cfg.pitch_center + cfg.pitch_sign * pitch_delta

    roll_pwm = int(clamp(roll_pwm, 1000, 2000))
    pitch_pwm = int(clamp(pitch_pwm, 1000, 2000))
    return roll_pwm, pitch_pwm


def send_controls(master, st: TelemetryState, target_alt_m: float, target_x_n: float, target_y_e: float, cfg: ControlConfig):
    """
    One control tick: pump telemetry, compute XY+Z, send RC override.
    """
    pump_telemetry(master, st)

    if st.alt_m is None:
        # Try to get at least one alt sample quickly
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1.0)
        if msg is not None:
            st.alt_m = float(msg.relative_alt) / 1000.0

    if st.alt_m is None:
        return  # cannot control without altitude

    thr = compute_throttle_pwm(target_alt_m, st.alt_m, cfg)
    roll_pwm, pitch_pwm = compute_xy_hold_pwm(st, target_x_n, target_y_e, cfg)

    send_rc_override(master, roll_pwm, pitch_pwm, thr, cfg.yaw_pwm)


# ----------------------------
# Scenario steps
# ----------------------------
def hold(master, st: TelemetryState, target_alt_m: float, target_x_n: float, target_y_e: float, seconds: float, cfg: ControlConfig, label: str):
    dt = 1.0 / cfg.rc_hz
    t_end = time.monotonic() + seconds
    last_print = 0.0
    while time.monotonic() < t_end:
        send_controls(master, st, target_alt_m, target_x_n, target_y_e, cfg)

        now = time.monotonic()
        if now - last_print > 0.5 and st.alt_m is not None and st.x_n is not None and st.y_e is not None:
            dx = (st.x_n - target_x_n)
            dy = (st.y_e - target_y_e)
            dist = math.sqrt(dx * dx + dy * dy)
            print(f"[{label}] alt={st.alt_m:5.2f}m  drift={dist:5.2f}m  xN={st.x_n:7.2f} yE={st.y_e:7.2f}")
            last_print = now

        time.sleep(dt)


def reach_altitude(master, st: TelemetryState, target_alt_m: float, target_x_n: float, target_y_e: float,
                  settle_band_m: float, settle_time_s: float, cfg: ControlConfig):
    dt = 1.0 / cfg.rc_hz
    stable_t0 = None
    last_print = 0.0

    while True:
        send_controls(master, st, target_alt_m, target_x_n, target_y_e, cfg)

        now = time.monotonic()
        if st.alt_m is None:
            time.sleep(dt)
            continue

        if now - last_print > 0.5 and st.x_n is not None and st.y_e is not None:
            err = target_alt_m - st.alt_m
            dx = st.x_n - target_x_n
            dy = st.y_e - target_y_e
            dist = math.sqrt(dx * dx + dy * dy)
            print(f"[CLIMB] alt={st.alt_m:5.2f} target={target_alt_m:5.2f} err={err:5.2f}  drift={dist:5.2f}m")
            last_print = now

        if abs(st.alt_m - target_alt_m) <= settle_band_m:
            if stable_t0 is None:
                stable_t0 = now
            elif now - stable_t0 >= settle_time_s:
                return st.alt_m
        else:
            stable_t0 = None

        time.sleep(dt)


def jitter_loop(master, st: TelemetryState, base_alt_m: float, amp_m: float, freq_hz: float,
                target_x_n: float, target_y_e: float, cfg: ControlConfig):
    dt = 1.0 / cfg.rc_hz
    t0 = time.monotonic()
    last_print = 0.0

    while True:
        t = time.monotonic() - t0
        target_alt = base_alt_m + amp_m * math.sin(2.0 * math.pi * freq_hz * t)

        send_controls(master, st, target_alt, target_x_n, target_y_e, cfg)

        now = time.monotonic()
        if now - last_print > 0.5 and st.alt_m is not None and st.x_n is not None and st.y_e is not None:
            dx = st.x_n - target_x_n
            dy = st.y_e - target_y_e
            dist = math.sqrt(dx * dx + dy * dy)
            print(f"[JITTER] targetAlt={target_alt:5.2f} alt={st.alt_m:5.2f}  drift={dist:5.2f}m")
            last_print = now

        time.sleep(dt)


# ----------------------------
# Main
# ----------------------------
def main():
    p = argparse.ArgumentParser(description="ArduPilot SITL: ALT_HOLD + RC override + XY hold in local NED + altitude jitter scenario.")
    p.add_argument("--endpoint", default="tcp:127.0.0.1:5762")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--mode", default="ALT_HOLD")
    p.add_argument("--land-mode", default="LAND")

    p.add_argument("--takeoff-alt", type=float, default=10.0)
    p.add_argument("--hold-s", type=float, default=3.0)
    p.add_argument("--jitter-min", type=float, default=8.0)
    p.add_argument("--jitter-max", type=float, default=12.0)
    p.add_argument("--jitter-period-s", type=float, default=6.0)

    # XY-hold tuning
    p.add_argument("--kp-xy", type=float, default=80.0, help="PWM per meter (body frame) for XY hold")
    p.add_argument("--kd-xy", type=float, default=50.0, help="PWM per (m/s) for XY hold")
    p.add_argument("--max-xy-delta", type=int, default=220, help="Max roll/pitch PWM delta from center")
    p.add_argument("--invert-roll", action="store_true", help="Invert roll direction (if correction goes the wrong way)")
    p.add_argument("--invert-pitch", action="store_true", help="Invert pitch direction (if correction goes the wrong way)")

    args = p.parse_args()

    # Connect
    print(f"[INFO] Connecting: {args.endpoint}")
    if args.endpoint.lower().startswith("com:"):
        port = args.endpoint.split(":", 1)[1]
        master = mavutil.mavlink_connection(port, baud=args.baud, timeout=1.0)
    else:
        master = mavutil.mavlink_connection(args.endpoint, timeout=1.0)

    wait_heartbeat_or_die(master)
    print(f"[OK] Heartbeat: sys={master.target_system} comp={master.target_component}")

    print(f"[INFO] Setting mode: {args.mode}")
    set_mode_or_die(master, args.mode)

    print("[INFO] Arming...")
    arm_or_die(master)
    print("[OK] Armed")

    cfg = ControlConfig()
    cfg.kp_xy_pwm_per_m = args.kp_xy
    cfg.kd_xy_pwm_per_mps = args.kd_xy
    cfg.max_xy_delta_pwm = args.max_xy_delta
    if args.invert_roll:
        cfg.roll_sign *= -1
    if args.invert_pitch:
        cfg.pitch_sign *= -1

    st = TelemetryState()

    print("[INFO] Waiting for telemetry (GLOBAL_POSITION_INT + LOCAL_POSITION_NED + ATTITUDE)...")
    wait_for_telemetry(master, st)
    print(f"[INFO] Telemetry ready: alt={st.alt_m:.2f}m xN={st.x_n:.2f} yE={st.y_e:.2f} yaw={st.yaw_rad:.2f}rad")

    # Lock XY target at current local position
    target_x_n = st.x_n
    target_y_e = st.y_e
    print(f"[INFO] XY hold target locked at local NED: xN={target_x_n:.2f}m yE={target_y_e:.2f}m")

    # Takeoff boost
    print("[INFO] Takeoff boost (2s)...")
    dt = 1.0 / cfg.rc_hz
    t_end = time.monotonic() + 2.0
    while time.monotonic() < t_end:
        pump_telemetry(master, st)
        roll_pwm, pitch_pwm = compute_xy_hold_pwm(st, target_x_n, target_y_e, cfg)
        send_rc_override(master, roll_pwm, pitch_pwm, 1750, cfg.yaw_pwm)
        time.sleep(dt)

    # Takeoff to target altitude
    print(f"[INFO] Taking off to {args.takeoff_alt:.1f} m (ALT_HOLD + XY hold)...")
    reached = reach_altitude(master, st, args.takeoff_alt, target_x_n, target_y_e,
                            settle_band_m=0.8, settle_time_s=0.6, cfg=cfg)
    print(f"[OK] Reached ~{reached:.2f} m")

    # Hold
    print(f"[INFO] Holding {args.takeoff_alt:.1f} m for {args.hold_s:.1f} s (with XY hold)")
    hold(master, st, args.takeoff_alt, target_x_n, target_y_e, args.hold_s, cfg, label="HOLD")

    # Jitter
    base = (args.jitter_min + args.jitter_max) / 2.0
    amp = (args.jitter_max - args.jitter_min) / 2.0
    freq = 1.0 / max(0.5, args.jitter_period_s)

    print(f"[INFO] Jitter loop: {args.jitter_min:.1f}..{args.jitter_max:.1f} m, period={args.jitter_period_s:.1f}s. Ctrl+C to stop.")
    try:
        jitter_loop(master, st, base_alt_m=base, amp_m=amp, freq_hz=freq, target_x_n=target_x_n, target_y_e=target_y_e, cfg=cfg)
    except KeyboardInterrupt:
        print("\n[INFO] Cancel requested. Returning to 10m, holding, then landing...")

    # Return to 10m and hold
    reach_altitude(master, st, args.takeoff_alt, target_x_n, target_y_e,
                  settle_band_m=0.8, settle_time_s=0.6, cfg=cfg)
    hold(master, st, args.takeoff_alt, target_x_n, target_y_e, args.hold_s, cfg, label="HOLD2")

    # Land
    print(f"[INFO] Switching to {args.land_mode} and releasing RC overrides")
    set_mode_or_die(master, args.land_mode)
    release_all_overrides(master)

    # Wait until near ground, then disarm
    print("[INFO] Waiting to be near ground (<0.3m) ...")
    while True:
        pump_telemetry(master, st)
        if st.alt_m is not None and st.alt_m < 0.3:
            break
        time.sleep(0.05)

    print("[INFO] Disarming...")
    disarm_or_die(master)
    print("[OK] Done")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[ERROR] {e}")
        sys.exit(1)
