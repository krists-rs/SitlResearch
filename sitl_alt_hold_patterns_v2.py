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
    if rate_hz <= 0:
        interval_us = -1
    else:
        interval_us = int(1_000_000 / rate_hz)
    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval_us,
            0, 0, 0, 0, 0
        )
    except Exception:
        pass


# ----------------------------
# Telemetry
# ----------------------------
@dataclass
class TelemetryState:
    alt_m: Optional[float] = None

    # Local NED: x=north, y=east
    x_n: Optional[float] = None
    y_e: Optional[float] = None

    vx_n: Optional[float] = None
    vy_e: Optional[float] = None

    yaw_rad: Optional[float] = None


def pump_telemetry(master, st: TelemetryState, max_msgs: int = 400):
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

    # Explicit message intervals:
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
    roll_center: int = 1500
    pitch_center: int = 1500
    yaw_pwm: int = 1500  # (yaw hold not used here)

    thr_center: int = 1500
    thr_min: int = 1200
    thr_max: int = 2000
    kp_thr_pwm_per_m: float = 120.0

    kp_xy_pwm_per_m: float = 80.0
    kd_xy_pwm_per_mps: float = 50.0
    max_xy_delta_pwm: int = 220

    roll_sign: int = +1
    pitch_sign: int = -1

    rc_hz: float = 50.0


# ----------------------------
# Low-level control (same idea)
# ----------------------------
def compute_throttle_pwm(target_alt_m: float, alt_m: float, cfg: ControlConfig) -> int:
    err = target_alt_m - alt_m
    delta = cfg.kp_thr_pwm_per_m * err
    thr = cfg.thr_center + delta
    return int(clamp(thr, cfg.thr_min, cfg.thr_max))


def compute_xy_hold_pwm(st: TelemetryState, target_x_n: float, target_y_e: float, cfg: ControlConfig):
    if st.x_n is None or st.y_e is None or st.yaw_rad is None:
        return cfg.roll_center, cfg.pitch_center

    n_err = target_x_n - st.x_n
    e_err = target_y_e - st.y_e

    vx_n = st.vx_n if st.vx_n is not None else 0.0
    vy_e = st.vy_e if st.vy_e is not None else 0.0

    yaw = st.yaw_rad
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    # NED -> body (forward/right)
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
    pump_telemetry(master, st)
    if st.alt_m is None:
        return
    thr = compute_throttle_pwm(target_alt_m, st.alt_m, cfg)
    roll_pwm, pitch_pwm = compute_xy_hold_pwm(st, target_x_n, target_y_e, cfg)
    send_rc_override(master, roll_pwm, pitch_pwm, thr, cfg.yaw_pwm)


# ----------------------------
# Navigation helpers (NEW)
# ----------------------------
def speed_xy(st: TelemetryState) -> float:
    vx = st.vx_n if st.vx_n is not None else 0.0
    vy = st.vy_e if st.vy_e is not None else 0.0
    return math.sqrt(vx * vx + vy * vy)


def dist_xy(st: TelemetryState, tx: float, ty: float) -> Optional[float]:
    if st.x_n is None or st.y_e is None:
        return None
    dx = st.x_n - tx
    dy = st.y_e - ty
    return math.sqrt(dx * dx + dy * dy)


def goto_point_smooth(
    master,
    st: TelemetryState,
    target_alt_m: float,
    target_x_n: float,
    target_y_e: float,
    cfg: ControlConfig,
    v_max: float,
    slow_radius_m: float,
    lookahead_s: float,
    pos_tol_m: float,
    vel_tol_mps: float,
    settle_s: float,
    min_carrot_m: float = 0.45,
    v_min: float = 0.35,
    label: str = "GOTO",
):
    """
    Smooth waypoint approach:
    - compute remaining distance from CURRENT position each tick
    - desired speed decreases as we approach the waypoint
    - command a carrot point ahead of current position (lookahead), thus we start braking early
    - arrival requires both distance AND low speed, stable for settle_s
    """
    if st.x_n is None or st.y_e is None:
        raise RuntimeError("No local position available (LOCAL_POSITION_NED).")

    dt = 1.0 / cfg.rc_hz
    stable_t0 = None
    last_print = 0.0

    while True:
        pump_telemetry(master, st)
        if st.x_n is None or st.y_e is None or st.alt_m is None:
            time.sleep(dt)
            continue

        dx = target_x_n - st.x_n
        dy = target_y_e - st.y_e
        d = math.sqrt(dx * dx + dy * dy)

        # --- speed profile with a floor to avoid stalling in deadband ---
        if slow_radius_m <= 1e-3:
            v_des = v_max
        else:
            v_des = v_max * (d / slow_radius_m)
            v_des = clamp(v_des, v_min, v_max)  # <-- key change

        # carrot distance: never too small until we are basically at the point
        # if we are already inside pos_tol, allow carrot to shrink to d
        if d <= pos_tol_m:
            carrot_dist = d
        else:
            carrot_dist = min(d, max(min_carrot_m, v_des * lookahead_s))  # <-- key change

        if d > 1e-6:
            ux = dx / d
            uy = dy / d
        else:
            ux, uy = 0.0, 0.0

        cx = st.x_n + ux * carrot_dist
        cy = st.y_e + uy * carrot_dist

        send_controls(master, st, target_alt_m, cx, cy, cfg)

        now = time.monotonic()
        if now - last_print > 0.5:
            v = speed_xy(st)
            print(f"[{label}] d={d:5.2f}m v={v:4.2f}m/s v_des={v_des:4.2f} carrot={carrot_dist:4.2f}  xN={st.x_n:7.2f} yE={st.y_e:7.2f}")
            last_print = now

        v = speed_xy(st)
        arrived = (d <= pos_tol_m) and (v <= vel_tol_mps)

        if arrived:
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
        if now - last_print > 0.5 and st.alt_m is not None:
            d = dist_xy(st, target_x_n, target_y_e)
            v = speed_xy(st)
            print(f"[{label}] alt={st.alt_m:5.2f}m d={d if d is not None else -1:5.2f}m v={v:4.2f}m/s")
            last_print = now
        time.sleep(dt)


def takeoff_to(master, st: TelemetryState, home_x: float, home_y: float, target_alt_m: float, cfg: ControlConfig):
    print("[INFO] Takeoff boost (2s)...")
    dt = 1.0 / cfg.rc_hz
    t_end = time.monotonic() + 2.0
    while time.monotonic() < t_end:
        pump_telemetry(master, st)
        roll_pwm, pitch_pwm = compute_xy_hold_pwm(st, home_x, home_y, cfg)
        send_rc_override(master, roll_pwm, pitch_pwm, 1750, cfg.yaw_pwm)
        time.sleep(dt)

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

        time.sleep(dt)


# ----------------------------
# Path generation (same as before)
# ----------------------------
def forward_right_vectors_from_yaw(yaw_rad: float) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    cy = math.cos(yaw_rad)
    sy = math.sin(yaw_rad)
    f = (cy, sy)
    r = (-sy, cy)
    return f, r


def add_xy(p: Tuple[float, float], v: Tuple[float, float], scale: float) -> Tuple[float, float]:
    return (p[0] + v[0] * scale, p[1] + v[1] * scale)


# ----------------------------
# Scenario loops (updated to use goto_point_smooth + better circle)
# ----------------------------
def loop_line(master, st: TelemetryState, home: Tuple[float, float], yaw_ref: float,
              alt_m: float, dist_m: float, dwell_s: float,
              cfg: ControlConfig, v_max: float, slow_radius: float, lookahead_s: float,
              pos_tol: float, vel_tol: float, settle_s: float, min_carrot_m: float, v_min: float):
    f, _ = forward_right_vectors_from_yaw(yaw_ref)
    far = add_xy(home, f, dist_m)

    print(f"[INFO] LINE: dist={dist_m}m dwell={dwell_s}s v_max={v_max} (Ctrl+C to stop)")
    while True:
        goto_point_smooth(master, st, alt_m, far[0], far[1], cfg, v_max, slow_radius, lookahead_s, pos_tol, vel_tol, settle_s, min_carrot_m=min_carrot_m, v_min=v_min, label="LINE_FWD")
        hold(master, st, alt_m, far[0], far[1], dwell_s, cfg, label="LINE_DWELL")
        goto_point_smooth(master, st, alt_m, home[0], home[1], cfg, v_max, slow_radius, lookahead_s, pos_tol, vel_tol, settle_s, min_carrot_m=min_carrot_m, v_min=v_min, label="LINE_BACK")


def loop_rect(master, st: TelemetryState, home: Tuple[float, float], yaw_ref: float,
              alt_m: float, n_m: float, m_m: float, dwell_s: float,
              cfg: ControlConfig, v_max: float, slow_radius: float, lookahead_s: float,
              pos_tol: float, vel_tol: float, settle_s: float, min_carrot_m: float, v_min: float):
    f, r = forward_right_vectors_from_yaw(yaw_ref)

    p1 = add_xy(home, f, n_m)
    p2 = add_xy(p1, r, m_m)
    p3 = add_xy(p2, f, -n_m)
    p4 = add_xy(p3, r, -m_m)

    points = [("RECT_P1", p1), ("RECT_P2", p2), ("RECT_P3", p3), ("RECT_P4", p4)]

    print(f"[INFO] RECT: n={n_m}m m={m_m}m dwell={dwell_s}s v_max={v_max} (Ctrl+C to stop)")
    while True:
        for name, p in points:
            goto_point_smooth(master, st, alt_m, p[0], p[1], cfg, v_max, slow_radius, lookahead_s, pos_tol, vel_tol, settle_s, min_carrot_m=min_carrot_m, v_min=v_min, label=name)
            if dwell_s > 0:
                hold(master, st, alt_m, p[0], p[1], dwell_s, cfg, label=f"{name}_DWELL")


def loop_circle(master, st: TelemetryState, home: Tuple[float, float], yaw_ref: float,
                alt_m: float, r_m: float, dwell_s: float, direction: str,
                cfg: ControlConfig, v_tan: float, lookahead_s: float, k_rad: float):
    """
    Vector-field circle:
    desired velocity = tangential * v_tan  + radial * (-k_rad * (r_cur - r_target))
    and we command a carrot = current + v_des * lookahead_s
    """
    f, _ = forward_right_vectors_from_yaw(yaw_ref)
    start = add_xy(home, f, r_m)

    print(f"[INFO] CIRCLE: r={r_m}m dwell={dwell_s}s v_tan={v_tan} dir={direction} (Ctrl+C to stop)")

    # go to arc entry smoothly, then dwell
    # For this "goto", reuse the same shaping defaults: slow down inside radius 2m etc.
    # We'll do it in main by passing appropriate args; here we keep circle loop clean.
    # Caller will move to start and dwell before calling this loop.

    sign = +1.0 if direction.lower() == "cw" else -1.0

    dt = 1.0 / cfg.rc_hz
    last_print = 0.0

    while True:
        pump_telemetry(master, st)
        if st.x_n is None or st.y_e is None or st.alt_m is None:
            time.sleep(dt)
            continue

        dx = st.x_n - home[0]
        dy = st.y_e - home[1]
        r_cur = math.sqrt(dx * dx + dy * dy)
        if r_cur < 1e-6:
            # if exactly at center, push outward along forward
            dx, dy = 1.0, 0.0
            r_cur = 1.0

        # radial unit (from center to current)
        rx = dx / r_cur
        ry = dy / r_cur

        # tangential unit (NE plane)
        # CCW: (-ry, rx). CW: (ry, -rx)
        if sign > 0:  # cw
            tx = ry
            ty = -rx
        else:  # ccw
            tx = -ry
            ty = rx

        r_err = r_cur - r_m
        # radial correction towards the circle
        vx = tx * v_tan + rx * (-k_rad * r_err)
        vy = ty * v_tan + ry * (-k_rad * r_err)

        # carrot position ahead
        cx = st.x_n + vx * lookahead_s
        cy = st.y_e + vy * lookahead_s

        send_controls(master, st, alt_m, cx, cy, cfg)

        now = time.monotonic()
        if now - last_print > 0.5:
            v = speed_xy(st)
            print(f"[CIRCLE] alt={st.alt_m:5.2f} r={r_cur:5.2f} r_err={r_err:+5.2f} v={v:4.2f}")
            last_print = now

        time.sleep(dt)


# ----------------------------
# Main
# ----------------------------
def main():
    p = argparse.ArgumentParser(
        description="ArduPilot SITL: ALT_HOLD + RC override + local XY control. Patterns: line/rect/circle (with smooth braking)."
    )
    p.add_argument("--endpoint", default="tcp:127.0.0.1:5762")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--mode", default="ALT_HOLD")
    p.add_argument("--land-mode", default="LAND")

    p.add_argument("--takeoff-alt", type=float, default=10.0)
    p.add_argument("--takeoff-hold-s", type=float, default=3.0)

    p.add_argument("--telemetry-hz", type=float, default=50.0)

    # navigation shaping
    p.add_argument("--v-max", type=float, default=1.5, help="max horizontal speed (m/s) for line/rect/goto")
    p.add_argument("--slow-radius", type=float, default=2.0, help="start slowing down within this distance to waypoint (m)")
    p.add_argument("--lookahead-s", type=float, default=0.6, help="carrot lookahead time (s)")
    p.add_argument("--pos-tol", type=float, default=0.25, help="position tolerance for arrival (m)")
    p.add_argument("--vel-tol", type=float, default=0.35, help="velocity tolerance for arrival (m/s)")
    p.add_argument("--settle-s", type=float, default=0.6, help="must stay within tolerances this long (s)")

    # circle params
    p.add_argument("--circle-v", type=float, default=1.2, help="tangential speed for circle (m/s)")
    p.add_argument("--circle-k-rad", type=float, default=1.5, help="radial correction gain (1/s), higher -> more circular")

    p.add_argument("--after-stop", choices=["land", "hold"], default="land")
    p.add_argument("--after-stop-hold-s", type=float, default=3.0)

    # XY controller tuning
    p.add_argument("--kp-xy", type=float, default=80.0)
    p.add_argument("--kd-xy", type=float, default=50.0)
    p.add_argument("--max-xy-delta", type=int, default=220)
    p.add_argument("--invert-roll", action="store_true")
    p.add_argument("--invert-pitch", action="store_true")

    p.add_argument("--v-min", type=float, default=0.35,
                   help="minimum approach speed used for carrot sizing (m/s), helps overcome deadband")
    p.add_argument("--min-carrot", type=float, default=0.45,
                   help="minimum carrot distance (m). If too small, vehicle may stall before reaching waypoint")

    sub = p.add_subparsers(dest="pattern", required=True)

    p_line = sub.add_parser("line")
    p_line.add_argument("--dist", type=float, default=5.0)
    p_line.add_argument("--dwell", type=float, default=3.0)

    p_rect = sub.add_parser("rect")
    p_rect.add_argument("--n", type=float, default=5.0)
    p_rect.add_argument("--m", type=float, default=5.0)
    p_rect.add_argument("--dwell", type=float, default=3.0)

    p_circle = sub.add_parser("circle")
    p_circle.add_argument("--r", type=float, default=5.0)
    p_circle.add_argument("--dwell", type=float, default=3.0)
    p_circle.add_argument("--dir", choices=["cw", "ccw"], default="cw")

    args = p.parse_args()

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

    if st.x_n is None or st.y_e is None:
        raise RuntimeError("No LOCAL_POSITION_NED received; cannot define home.")
    home = (st.x_n, st.y_e)

    takeoff_to(master, st, home[0], home[1], args.takeoff_alt, cfg)
    print(f"[INFO] Takeoff hold {args.takeoff_hold_s:.1f}s ...")
    hold(master, st, args.takeoff_alt, home[0], home[1], args.takeoff_hold_s, cfg, label="TAKEOFF_HOLD")

    pump_telemetry(master, st)
    if st.yaw_rad is None:
        raise RuntimeError("No ATTITUDE yaw received; cannot lock yaw reference.")
    yaw_ref = st.yaw_rad
    print(f"[INFO] Yaw reference locked: {yaw_ref:.3f} rad")

    try:
        if args.pattern == "line":
            loop_line(
                master, st, home, yaw_ref,
                alt_m=args.takeoff_alt,
                dist_m=args.dist,
                dwell_s=args.dwell,
                cfg=cfg,
                v_max=args.v_max,
                slow_radius=args.slow_radius,
                lookahead_s=args.lookahead_s,
                pos_tol=args.pos_tol,
                vel_tol=args.vel_tol,
                settle_s=args.settle_s,
                min_carrot_m=args.min_carrot,
                v_min=args.v_min,
            )

        elif args.pattern == "rect":
            loop_rect(
                master, st, home, yaw_ref,
                alt_m=args.takeoff_alt,
                n_m=args.n,
                m_m=args.m,
                dwell_s=args.dwell,
                cfg=cfg,
                v_max=args.v_max,
                slow_radius=args.slow_radius,
                lookahead_s=args.lookahead_s,
                pos_tol=args.pos_tol,
                vel_tol=args.vel_tol,
                settle_s=args.settle_s,
                min_carrot_m=args.min_carrot,
                v_min=args.v_min,
            )

        elif args.pattern == "circle":
            # сначала — выйти на дугу по носу (точка старта на окружности)
            f, _ = forward_right_vectors_from_yaw(yaw_ref)
            start = add_xy(home, f, args.r)

            goto_point_smooth(
                master, st,
                target_alt_m=args.takeoff_alt,
                target_x_n=start[0], target_y_e=start[1],
                cfg=cfg,
                v_max=min(args.v_max, args.circle_v),
                slow_radius_m=args.slow_radius,
                lookahead_s=args.lookahead_s,
                pos_tol_m=args.pos_tol,
                vel_tol_mps=args.vel_tol,
                settle_s=args.settle_s,
                min_carrot_m=args.min_carrot,
                v_min=args.v_min,
                label="CIRCLE_TO_ARC",
            )
            if args.dwell > 0:
                hold(master, st, args.takeoff_alt, start[0], start[1], args.dwell, cfg, label="CIRCLE_DWELL")

            loop_circle(
                master, st, home, yaw_ref,
                alt_m=args.takeoff_alt,
                r_m=args.r,
                dwell_s=args.dwell,
                direction=args.dir,
                cfg=cfg,
                v_tan=args.circle_v,
                lookahead_s=args.lookahead_s,
                k_rad=args.circle_k_rad,
            )
        else:
            raise RuntimeError("Unknown pattern")

    except KeyboardInterrupt:
        print("\n[INFO] Stop requested (Ctrl+C). Returning to home...")

    # Return home (with braking)
    try:
        goto_point_smooth(
            master, st,
            target_alt_m=args.takeoff_alt,
            target_x_n=home[0], target_y_e=home[1],
            cfg=cfg,
            v_max=args.v_max,
            slow_radius_m=args.slow_radius,
            lookahead_s=args.lookahead_s,
            pos_tol_m=args.pos_tol,
            vel_tol_mps=args.vel_tol,
            settle_s=args.settle_s,
            min_carrot_m=args.min_carrot,
            v_min=args.v_min,
            label="RETURN_HOME",
        )
        hold(master, st, args.takeoff_alt, home[0], home[1], args.after_stop_hold_s, cfg, label="HOME_HOLD")
    except Exception as e:
        print(f"[WARN] Return-home failed: {e}")

    if args.after_stop == "land":
        print(f"[INFO] Landing: switching mode to {args.land_mode} and releasing RC overrides")
        set_mode_or_die(master, args.land_mode)
        release_all_overrides(master)

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
        print("[INFO] after-stop=hold: releasing RC overrides (no landing)")
        release_all_overrides(master)
        print("[OK] Done")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[ERROR] {e}")
        sys.exit(1)
