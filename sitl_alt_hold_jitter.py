#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import sys
import time
from dataclasses import dataclass

from pymavlink import mavutil


# ----------------------------
# RC helpers
# ----------------------------
def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def pwm_from_stick_norm(x: float) -> int:
    """
    x in [-1..+1] -> PWM [1000..2000], 0 -> 1500
    """
    x = clamp(x, -1.0, 1.0)
    return int(round(1500 + 500 * x))


def send_rc_override(master, ch1=0, ch2=0, ch3=0, ch4=0, ch5=0, ch6=0, ch7=0, ch8=0):
    """
    RC_CHANNELS_OVERRIDE:
      - 0 => release / no override on that channel
      - 1000..2000 => PWM override
    """
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8
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
    """
    Uses MAVLink SET_MODE via pymavlink convenience.
    """
    if mode not in master.mode_mapping():
        raise RuntimeError(f"Mode '{mode}' not supported by this vehicle/firmware. Available: {sorted(master.mode_mapping().keys())}")

    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)

    # Wait until heartbeat reports the mode.
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg is None:
            continue
        # custom_mode is mode id for ArduPilot
        if int(getattr(msg, "custom_mode", -1)) == int(mode_id):
            return
    raise RuntimeError(f"Failed to confirm mode '{mode}' within {timeout_s}s.")


def arm_or_die(master):
    master.arducopter_arm()
    master.motors_armed_wait()


def disarm_or_die(master):
    master.arducopter_disarm()
    master.motors_disarmed_wait()


def get_relative_alt_m(master, timeout_s: float = 1.0):
    """
    Reads GLOBAL_POSITION_INT.relative_alt (mm) -> meters.
    """
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=timeout_s)
    if msg is None:
        return None
    return float(msg.relative_alt) / 1000.0


# ----------------------------
# Scenario control
# ----------------------------
@dataclass
class ControlConfig:
    # RC neutral
    roll_pwm: int = 1500
    pitch_pwm: int = 1500
    yaw_pwm: int = 1500

    # Throttle control parameters (in ALT_HOLD)
    # 1500 is "neutral zone" around which Copter holds altitude. :contentReference[oaicite:2]{index=2}
    thr_center: int = 1500

    # Limits to avoid overly aggressive climb/sink via stick
    thr_min: int = 1200
    thr_max: int = 2000

    # Simple P-controller mapping altitude error -> throttle PWM delta
    # (ALT_HOLD interprets throttle deflection as climb/sink demand; autopilot closes the loop.)
    kp_pwm_per_m: float = 120.0

    # Update rate of RC override (Hz). Must be periodic; overrides time out if not refreshed.
    rc_hz: float = 50.0


def compute_throttle_pwm(target_alt_m: float, alt_m: float, cfg: ControlConfig) -> int:
    err = target_alt_m - alt_m
    delta = cfg.kp_pwm_per_m * err
    thr = cfg.thr_center + delta
    return int(clamp(thr, cfg.thr_min, cfg.thr_max))


def hold_altitude(master, target_alt_m: float, seconds: float, cfg: ControlConfig):
    dt = 1.0 / cfg.rc_hz
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        alt = get_relative_alt_m(master, timeout_s=1.0)
        if alt is None:
            continue
        thr = compute_throttle_pwm(target_alt_m, alt, cfg)
        send_rc_override(master, cfg.roll_pwm, cfg.pitch_pwm, thr, cfg.yaw_pwm)
        time.sleep(dt)


def reach_altitude(master, target_alt_m: float, settle_band_m: float, settle_time_s: float, cfg: ControlConfig):
    """
    Drive to target altitude; require staying within settle_band for settle_time.
    Prints periodic progress so it never looks "stuck".
    """
    dt = 1.0 / cfg.rc_hz
    stable_t0 = None
    last_print = 0.0

    while True:
        alt = get_relative_alt_m(master, timeout_s=1.0)
        if alt is None:
            continue

        thr = compute_throttle_pwm(target_alt_m, alt, cfg)
        send_rc_override(master, cfg.roll_pwm, cfg.pitch_pwm, thr, cfg.yaw_pwm)

        now = time.monotonic()

        # Periodic progress log (always)
        if now - last_print > 0.5:
            err = target_alt_m - alt
            sat = " SAT" if thr >= cfg.thr_max - 1 else ""
            print(f"[CLIMB] alt={alt:5.2f}m target={target_alt_m:5.2f}m err={err:5.2f} thr={thr}{sat}")
            last_print = now

        # "Reached" logic
        if abs(alt - target_alt_m) <= settle_band_m:
            if stable_t0 is None:
                stable_t0 = now
            elif now - stable_t0 >= settle_time_s:
                return alt
        else:
            stable_t0 = None

        time.sleep(dt)


def jitter_loop(master, base_alt_m: float, amp_m: float, freq_hz: float, cfg: ControlConfig):
    """
    Oscillate target altitude: base + amp*sin(2π f t), until Ctrl+C.
    """
    dt = 1.0 / cfg.rc_hz
    t0 = time.monotonic()
    last_print = 0.0

    while True:
        t = time.monotonic() - t0
        target = base_alt_m + amp_m * math.sin(2.0 * math.pi * freq_hz * t)

        alt = get_relative_alt_m(master, timeout_s=1.0)
        if alt is None:
            continue

        thr = compute_throttle_pwm(target, alt, cfg)
        send_rc_override(master, cfg.roll_pwm, cfg.pitch_pwm, thr, cfg.yaw_pwm)

        now = time.monotonic()
        if now - last_print > 0.5:
            print(f"[JITTER] target={target:5.2f}m alt={alt:5.2f}m thr={thr}")
            last_print = now

        time.sleep(dt)


def main():
    p = argparse.ArgumentParser(description="ArduPilot SITL: ALT_HOLD takeoff->hold->jitter->return->land via RC override.")
    p.add_argument("--endpoint", default="tcp:127.0.0.1:5762", help="e.g. tcp:127.0.0.1:5760 (MissionPlanner SITL SERIAL0)")
    p.add_argument("--baud", type=int, default=115200, help="serial baudrate (only for serial endpoints)")
    p.add_argument("--mode", default="ALT_HOLD", help="Recommended: ALT_HOLD")
    p.add_argument("--takeoff-alt", type=float, default=10.0, help="meters")
    p.add_argument("--hold-s", type=float, default=3.0, help="seconds")
    p.add_argument("--jitter-min", type=float, default=8.0, help="meters")
    p.add_argument("--jitter-max", type=float, default=12.0, help="meters")
    p.add_argument("--jitter-period-s", type=float, default=6.0, help="seconds for full cycle (sinusoid)")
    p.add_argument("--land-mode", default="LAND", help="Landing mode to switch to at the end")
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

    # Set mode and arm
    print(f"[INFO] Setting mode: {args.mode}")
    set_mode_or_die(master, args.mode)

    print("[INFO] Arming...")
    arm_or_die(master)
    print("[OK] Armed")

    cfg = ControlConfig()

    # Wait first altitude
    print("[INFO] Waiting for altitude telemetry (GLOBAL_POSITION_INT)...")
    alt0 = None
    t0 = time.monotonic()
    while alt0 is None and time.monotonic() - t0 < 10.0:
        alt0 = get_relative_alt_m(master, timeout_s=1.0)
    if alt0 is None:
        raise RuntimeError("No GLOBAL_POSITION_INT received (relative_alt).")

    print(f"[INFO] Current relative altitude: {alt0:.2f} m")

    print("[INFO] Takeoff boost (2s)...")
    dt = 1.0 / cfg.rc_hz
    t_end = time.monotonic() + 2.0
    while time.monotonic() < t_end:
        send_rc_override(master, cfg.roll_pwm, cfg.pitch_pwm, 1750, cfg.yaw_pwm)
        time.sleep(dt)

    # Takeoff to target altitude
    print(f"[INFO] Taking off to {args.takeoff_alt:.1f} m (ALT_HOLD with RC throttle deflection)...")
    reached = reach_altitude(master, args.takeoff_alt, settle_band_m=0.7, settle_time_s=0.6, cfg=cfg)
    print(f"[OK] Reached ~{reached:.2f} m")

    # Hold
    print(f"[INFO] Holding {args.takeoff_alt:.1f} m for {args.hold_s:.1f} s")
    hold_altitude(master, args.takeoff_alt, args.hold_s, cfg)

    # Jitter
    base = (args.jitter_min + args.jitter_max) / 2.0
    amp = (args.jitter_max - args.jitter_min) / 2.0
    freq = 1.0 / max(0.5, args.jitter_period_s)

    print(f"[INFO] Jitter loop: {args.jitter_min:.1f}..{args.jitter_max:.1f} m (sin), period={args.jitter_period_s:.1f}s. Ctrl+C to stop.")
    try:
        jitter_loop(master, base_alt_m=base, amp_m=amp, freq_hz=freq, cfg=cfg)
    except KeyboardInterrupt:
        print("\n[INFO] Cancel requested. Returning to 10m, holding, then landing...")

    # Return to 10m and hold
    reach_altitude(master, args.takeoff_alt, settle_band_m=0.7, settle_time_s=0.6, cfg=cfg)
    hold_altitude(master, args.takeoff_alt, args.hold_s, cfg)

    # Land
    print(f"[INFO] Switching to {args.land_mode} and releasing RC overrides")
    set_mode_or_die(master, args.land_mode)
    release_all_overrides(master)

    # Wait until near ground, then disarm (optional safety)
    print("[INFO] Waiting to be near ground (<0.3m) ...")
    while True:
        alt = get_relative_alt_m(master, timeout_s=1.0)
        if alt is None:
            continue
        if alt < 0.3:
            break

    print("[INFO] Disarming...")
    disarm_or_die(master)
    print("[OK] Done")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[ERROR] {e}")
        sys.exit(1)
