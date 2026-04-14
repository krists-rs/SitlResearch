#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import Optional, Tuple

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


def set_message_interval(master, msg_id: int, rate_hz: float):
    """
    Best-effort: request a specific MAVLink message at given rate using MAV_CMD_SET_MESSAGE_INTERVAL.
    interval_us = 1e6 / rate_hz.
    """
    if rate_hz <= 0:
        interval_us = -1  # disable
    else:
        interval_us = int(1_000_000 / rate_hz)

    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,          # param1: message id
            interval_us,     # param2: interval (us)
            0, 0, 0, 0, 0
        )
    except Exception:
        # Non-fatal
        pass


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


def pump_telemetry(master, st: TelemetryState, max_msgs: int = 300):
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


def request_telemetry(master, rate_hz: float):
    """
    Best-effort request: stream all + set specific message intervals.
    """
    try:
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            int(max(1, rate_hz)),
            1,
        )
        print(f"[INFO] Requested data stream rate ~{rate_hz} Hz (MAV_DATA_STREAM_ALL)")
    except Exception as e:
        print(f"[WARN] request_data_stream_send failed (non-fatal): {e}")

    # Explicit message intervals (IDs are standard MAVLink v1):
    # ATTITUDE=30, LOCAL_POSITION_NED=32, GLOBAL_POSITION_INT=33
    set_message_interval(master, 30, rate_hz)
    set_message_interval(master, 32, rate_hz)
    set_message_interval(master, 33, rate_hz)


def wait_for_telemetry(master, st: TelemetryState, rate_hz: float = 30.0, timeout_s: float = 12.0):
    request_telemetry(master, rate_hz)

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
    kp_xy_pwm_per_m: float = 80.0
    kd_xy_pwm_per_mps: float = 50.0

    # Max stick deflection for XY correction (PWM delta from center)
    max_xy_delta_pwm: int = 220

    # Channel directions (defaults match common SITL: roll right => PWM+, pitch forward => PWM-)
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
    Position hold in local NED, transformed into body-forward/right using current yaw.
    Produces roll/pitch PWM commands.
    """
    if st.x_n is None or st.y_e is None or st.yaw_rad is None:
        return cfg.roll_center, cfg.pitch_center

    n_err = target_x_n - st.x_n
    e_err = target_y_e - st.y_e

    vx_n = st.vx_n if st.vx_n is not None else 0.0
    vy_e = st.vy_e if st.vy_e is not None else 0.0

    yaw = st.yaw_rad
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    # NED -> body
    f_err = cy * n_err + sy * e_err
    r_err = -sy * n_err + cy * e_err

    f_vel = cy * vx_n + sy * vy_e
    r_vel = -sy * vx_n + cy * vy_e

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
    One control tick: update telemetry, compute XY+Z, send RC override.
    """
    pump_telemetry(master, st)

    if st.alt_m is None:
        return

    thr = compute_throttle_pwm(target_alt_m, st.alt_m, cfg)
    roll_pwm, pitch_pwm = compute_xy_hold_pwm(st, target_x_n, target_y_e, cfg)
    send_rc_override(master, roll_pwm, pitch_pwm, thr, cfg.yaw_pwm)


# ----------------------------
# Motion primitives
# ----------------------------
def dist_xy(st: TelemetryState, tx: float, ty: float) -> Optional[float]:
    if st.x_n is None or st.y_e is None:
        return None
    dx = st.x_n - tx
    dy = st.y_e - ty
    return math.sqrt(dx * dx + dy * dy)


def goto_point(master, st: TelemetryState, target_alt_m: float, target_x_n: float, target_y_e: float,
              cfg: ControlConfig, speed_mps: float, pos_tol_m: float, settle_s: float,
              label: str = "GOTO"):
    """
    Move to a point using a "carrot" target that progresses along the segment at speed_mps,
    while low-level controller holds XY and altitude.
    """
    if st.x_n is None or st.y_e is None:
        raise RuntimeError("No local position available (LOCAL_POSITION_NED).")

    sx, sy = st.x_n, st.y_e
    dx = target_x_n - sx
    dy = target_y_e - sy
    seg_len = math.sqrt(dx * dx + dy * dy)

    dt = 1.0 / cfg.rc_hz
    t0 = time.monotonic()
    stable_t0 = None
    last_print = 0.0

    # Degenerate: already there
    if seg_len < 1e-3:
        hold(master, st, target_alt_m, target_x_n, target_y_e, settle_s, cfg, label=f"{label}_HOLD")
        return

    ux = dx / seg_len
    uy = dy / seg_len

    while True:
        now = time.monotonic()
        t = now - t0

        # Carrot distance traveled along segment
        s = min(seg_len, max(0.0, speed_mps * t))
        cx = sx + ux * s
        cy = sy + uy * s

        send_controls(master, st, target_alt_m, cx, cy, cfg)

        d = dist_xy(st, target_x_n, target_y_e)
        if d is not None and now - last_print > 0.5 and st.alt_m is not None:
            print(f"[{label}] alt={st.alt_m:5.2f}m dist={d:5.2f}m  xN={st.x_n:7.2f} yE={st.y_e:7.2f}")
            last_print = now

        # settle near final target
        if d is not None and d <= pos_tol_m:
            if stable_t0 is None:
                stable_t0 = now
            elif now - stable_t0 >= settle_s:
                return
        else:
            stable_t0 = None

        time.sleep(dt)


def hold(master, st: TelemetryState, target_alt_m: float, target_x_n: float, target_y_e: float,
         seconds: float, cfg: ControlConfig, label: str = "HOLD"):
    dt = 1.0 / cfg.rc_hz
    t_end = time.monotonic() + seconds
    last_print = 0.0
    while time.monotonic() < t_end:
        send_controls(master, st, target_alt_m, target_x_n, target_y_e, cfg)

        now = time.monotonic()
        if now - last_print > 0.5 and st.alt_m is not None and st.x_n is not None and st.y_e is not None:
            d = dist_xy(st, target_x_n, target_y_e)
            print(f"[{label}] alt={st.alt_m:5.2f}m drift={d if d is not None else -1:5.2f}m")
            last_print = now

        time.sleep(dt)


def takeoff_to(master, st: TelemetryState, home_x: float, home_y: float, target_alt_m: float, cfg: ControlConfig):
    # Short boost helps liftoff
    print("[INFO] Takeoff boost (2s)...")
    dt = 1.0 / cfg.rc_hz
    t_end = time.monotonic() + 2.0
    while time.monotonic() < t_end:
        pump_telemetry(master, st)
        roll_pwm, pitch_pwm = compute_xy_hold_pwm(st, home_x, home_y, cfg)
        send_rc_override(master, roll_pwm, pitch_pwm, 1750, cfg.yaw_pwm)
        time.sleep(dt)

    # Climb using the same controller to a fixed XY (home)
    print(f"[INFO] Climbing to {target_alt_m:.1f}m ...")
    stable_t0 = None
    last_print = 0.0
    while True:
        send_controls(master, st, target_alt_m, home_x, home_y, cfg)
        now = time.monotonic()

        if st.alt_m is not None and now - last_print > 0.5:
            err = target_alt_m - st.alt_m
            print(f"[CLIMB] alt={st.alt_m:5.2f} target={target_alt_m:5.2f} err={err:5.2f}")
            last_print = now

        if st.alt_m is not None and abs(st.alt_m - target_alt_m) <= 0.8:
            if stable_t0 is None:
                stable_t0 = now
            elif now - stable_t0 >= 0.6:
                return
        else:
            stable_t0 = None

        time.sleep(1.0 / cfg.rc_hz)


# ----------------------------
# Path generation in NED from nose direction
# ----------------------------
def forward_right_vectors_from_yaw(yaw_rad: float) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """
    Returns forward and right unit vectors in NED (north,east).
    yaw=0 => forward=(1,0) (north), right=(0,1) (east)
    """
    cy = math.cos(yaw_rad)
    sy = math.sin(yaw_rad)
    f = (cy, sy)
    r = (-sy, cy)
    return f, r


def add_xy(p: Tuple[float, float], v: Tuple[float, float], scale: float) -> Tuple[float, float]:
    return (p[0] + v[0] * scale, p[1] + v[1] * scale)


# ----------------------------
# Scenario loops
# ----------------------------
def loop_line(master, st: TelemetryState, home: Tuple[float, float], yaw_ref: float,
              alt_m: float, dist_m: float, dwell_s: float, speed_mps: float,
              cfg: ControlConfig, pos_tol_m: float, settle_s: float):
    f, _ = forward_right_vectors_from_yaw(yaw_ref)
    far = add_xy(home, f, dist_m)

    print(f"[INFO] LINE: dist={dist_m}m dwell={dwell_s}s speed={speed_mps}m/s (Ctrl+C to stop)")
    while True:
        goto_point(master, st, alt_m, far[0], far[1], cfg, speed_mps, pos_tol_m, settle_s, label="LINE_FWD")
        hold(master, st, alt_m, far[0], far[1], dwell_s, cfg, label="LINE_DWELL")
        goto_point(master, st, alt_m, home[0], home[1], cfg, speed_mps, pos_tol_m, settle_s, label="LINE_BACK")


def loop_rect(master, st: TelemetryState, home: Tuple[float, float], yaw_ref: float,
              alt_m: float, n_m: float, m_m: float, dwell_s: float, speed_mps: float,
              cfg: ControlConfig, pos_tol_m: float, settle_s: float):
    f, r = forward_right_vectors_from_yaw(yaw_ref)

    p1 = add_xy(home, f, n_m)
    p2 = add_xy(p1, r, m_m)
    p3 = add_xy(p2, f, -n_m)
    p4 = add_xy(p3, r, -m_m)  # should be home

    points = [("RECT_P1", p1), ("RECT_P2", p2), ("RECT_P3", p3), ("RECT_P4", p4)]

    print(f"[INFO] RECT: n={n_m}m m={m_m}m dwell={dwell_s}s speed={speed_mps}m/s (Ctrl+C to stop)")
    while True:
        for name, p in points:
            goto_point(master, st, alt_m, p[0], p[1], cfg, speed_mps, pos_tol_m, settle_s, label=name)
            hold(master, st, alt_m, p[0], p[1], dwell_s, cfg, label=f"{name}_DWELL")


def loop_circle(master, st: TelemetryState, home: Tuple[float, float], yaw_ref: float,
                alt_m: float, r_m: float, dwell_s: float, speed_mps: float, direction: str,
                cfg: ControlConfig, pos_tol_m: float, settle_s: float):
    """
    Circle around home. We first go to the circle point at "forward" direction (angle=0),
    dwell, then move continuously along the circle until Ctrl+C.
    """
    f, r = forward_right_vectors_from_yaw(yaw_ref)
    start = add_xy(home, f, r_m)

    print(f"[INFO] CIRCLE: r={r_m}m dwell={dwell_s}s speed={speed_mps}m/s dir={direction} (Ctrl+C to stop)")

    # Move to start point on the circle
    goto_point(master, st, alt_m, start[0], start[1], cfg, speed_mps, pos_tol_m, settle_s, label="CIRCLE_TO_ARC")
    hold(master, st, alt_m, start[0], start[1], dwell_s, cfg, label="CIRCLE_DWELL")

    # Continuous circle
    # omega = v / r
    omega = speed_mps / max(0.1, r_m)
    sign = +1.0 if direction.lower() == "cw" else -1.0

    dt = 1.0 / cfg.rc_hz
    t0 = time.monotonic()
    last_print = 0.0

    while True:
        t = time.monotonic() - t0
        theta = sign * omega * t

        # pos = home + f*(r*cos) + r*(r*sin)
        cx = home[0] + f[0] * (r_m * math.cos(theta)) + r[0] * (r_m * math.sin(theta))
        cy = home[1] + f[1] * (r_m * math.cos(theta)) + r[1] * (r_m * math.sin(theta))

        send_controls(master, st, alt_m, cx, cy, cfg)

        now = time.monotonic()
        if now - last_print > 0.5 and st.x_n is not None and st.y_e is not None and st.alt_m is not None:
            # How far from ideal circle center radius (for debug)
            dx = st.x_n - home[0]
            dy = st.y_e - home[1]
            rr = math.sqrt(dx * dx + dy * dy)
            print(f"[CIRCLE] alt={st.alt_m:5.2f}m r={rr:5.2f}m theta={theta:6.2f}rad")
            last_print = now

        time.sleep(dt)


# ----------------------------
# Main
# ----------------------------
def main():
    p = argparse.ArgumentParser(
        description="ArduPilot SITL: ALT_HOLD + RC override + local XY control. Patterns: line/rect/circle."
    )
    p.add_argument("--endpoint", default="tcp:127.0.0.1:5762")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--mode", default="ALT_HOLD")
    p.add_argument("--land-mode", default="LAND")

    p.add_argument("--takeoff-alt", type=float, default=10.0)
    p.add_argument("--takeoff-hold-s", type=float, default=3.0)

    p.add_argument("--telemetry-hz", type=float, default=30.0)
    p.add_argument("--xy-speed", type=float, default=1.5, help="speed along segments / circle tangential speed, m/s")
    p.add_argument("--pos-tol", type=float, default=0.4, help="position tolerance for waypoint arrival, m")
    p.add_argument("--settle-s", type=float, default=0.4, help="settle time near waypoint, s")

    p.add_argument("--after-stop", choices=["land", "hold"], default="land",
                   help="what to do after Ctrl+C: return home + (land or hold)")
    p.add_argument("--after-stop-hold-s", type=float, default=3.0, help="hold duration at home before landing or exit")

    # XY controller tuning
    p.add_argument("--kp-xy", type=float, default=80.0)
    p.add_argument("--kd-xy", type=float, default=50.0)
    p.add_argument("--max-xy-delta", type=int, default=220)
    p.add_argument("--invert-roll", action="store_true")
    p.add_argument("--invert-pitch", action="store_true")

    sub = p.add_subparsers(dest="pattern", required=True)

    p_line = sub.add_parser("line", help="shuttle along nose direction")
    p_line.add_argument("--dist", type=float, default=5.0, help="meters forward from home along nose direction")
    p_line.add_argument("--dwell", type=float, default=3.0, help="seconds to dwell at far endpoint")

    p_rect = sub.add_parser("rect", help="rectangle in nose-forward/right frame")
    p_rect.add_argument("--n", type=float, default=5.0, help="forward side length (meters)")
    p_rect.add_argument("--m", type=float, default=5.0, help="right side length (meters)")
    p_rect.add_argument("--dwell", type=float, default=3.0, help="seconds to dwell at each corner")

    p_circle = sub.add_parser("circle", help="circle around takeoff point")
    p_circle.add_argument("--r", type=float, default=5.0, help="radius in meters")
    p_circle.add_argument("--dwell", type=float, default=3.0, help="seconds to dwell at arc entry point")
    p_circle.add_argument("--dir", choices=["cw", "ccw"], default="cw", help="direction along circle")

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
    wait_for_telemetry(master, st, rate_hz=args.telemetry_hz)
    print(f"[INFO] Telemetry ready: alt={st.alt_m:.2f}m xN={st.x_n:.2f} yE={st.y_e:.2f} yaw={st.yaw_rad:.2f}rad")

    # Home is takeoff point in local NED
    if st.x_n is None or st.y_e is None:
        raise RuntimeError("No LOCAL_POSITION_NED received; cannot define home.")
    home = (st.x_n, st.y_e)

    # Takeoff to 10m, hold 3s
    takeoff_to(master, st, home[0], home[1], args.takeoff_alt, cfg)
    print(f"[INFO] Hold at {args.takeoff_alt:.1f}m for {args.takeoff_hold_s:.1f}s ...")
    hold(master, st, args.takeoff_alt, home[0], home[1], args.takeoff_hold_s, cfg, label="TAKEOFF_HOLD")

    # Fix yaw reference (nose direction) after takeoff stabilization
    pump_telemetry(master, st)
    if st.yaw_rad is None:
        raise RuntimeError("No ATTITUDE yaw received; cannot define nose direction.")
    yaw_ref = st.yaw_rad
    print(f"[INFO] Yaw reference locked: {yaw_ref:.3f} rad")

    # Run pattern loop until Ctrl+C
    try:
        if args.pattern == "line":
            loop_line(
                master, st, home, yaw_ref,
                alt_m=args.takeoff_alt,
                dist_m=args.dist,
                dwell_s=args.dwell,
                speed_mps=args.xy_speed,
                cfg=cfg,
                pos_tol_m=args.pos_tol,
                settle_s=args.settle_s,
            )
        elif args.pattern == "rect":
            loop_rect(
                master, st, home, yaw_ref,
                alt_m=args.takeoff_alt,
                n_m=args.n,
                m_m=args.m,
                dwell_s=args.dwell,
                speed_mps=args.xy_speed,
                cfg=cfg,
                pos_tol_m=args.pos_tol,
                settle_s=args.settle_s,
            )
        elif args.pattern == "circle":
            loop_circle(
                master, st, home, yaw_ref,
                alt_m=args.takeoff_alt,
                r_m=args.r,
                dwell_s=args.dwell,
                speed_mps=args.xy_speed,
                direction=args.dir,
                cfg=cfg,
                pos_tol_m=args.pos_tol,
                settle_s=args.settle_s,
            )
        else:
            raise RuntimeError("Unknown pattern")
    except KeyboardInterrupt:
        print("\n[INFO] Stop requested (Ctrl+C). Returning to home...")

    # Return to home
    try:
        goto_point(master, st, args.takeoff_alt, home[0], home[1], cfg, args.xy_speed, args.pos_tol, args.settle_s, label="RETURN_HOME")
        hold(master, st, args.takeoff_alt, home[0], home[1], args.after_stop_hold_s, cfg, label="HOME_HOLD")
    except Exception as e:
        print(f"[WARN] Return-home failed: {e}")

    if args.after_stop == "land":
        print(f"[INFO] Landing: switching mode to {args.land_mode} and releasing RC overrides")
        set_mode_or_die(master, args.land_mode)
        release_all_overrides(master)

        # Wait near ground then disarm
        print("[INFO] Waiting to be near ground (<0.3m) ...")
        while True:
            pump_telemetry(master, st)
            if st.alt_m is not None and st.alt_m < 0.3:
                break
            time.sleep(0.05)

        print("[INFO] Disarming...")
        disarm_or_die(master)
        print("[OK] Done")
    else:
        print("[INFO] after-stop=hold: releasing RC overrides (vehicle remains in current mode/armed state)")
        release_all_overrides(master)
        print("[OK] Done (no landing).")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[ERROR] {e}")
        sys.exit(1)
