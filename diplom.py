#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import pickle
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

import cv2
from pymavlink import mavutil


# ----------------------------
# TCP helpers
# ----------------------------
def send_msg(sock: socket.socket, data):
    payload = pickle.dumps(data, protocol=2)
    sock.sendall(struct.pack(">I", len(payload)) + payload)


def recvall(sock: socket.socket, size: int):
    data = b""
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            return None
        data += chunk
    return data


class TcpDataBridge:
    """
    Sends IMU and image messages over TCP and receives estimated positions back.

    Time is transferred as:
        t_ns = monotonic timestamp relative to stream_start_ns
    """

    def __init__(self, host: str, port: int, connect_timeout: float = 5.0):
        self.host = host
        self.port = port
        self.connect_timeout = connect_timeout

        self.sock: Optional[socket.socket] = None
        self.send_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.receiver = None

        self.enabled = False
        self.stream_start_ns = 0
        self.estimated_data = []

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.connect_timeout)
            self.sock.connect((self.host, self.port))
            self.sock.settimeout(None)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

            self.stream_start_ns = time.monotonic_ns()
            self.enabled = True

            self.receiver = threading.Thread(target=self._receiver_thread, daemon=True)
            self.receiver.start()

            print(f"[OK] TCP bridge connected to {self.host}:{self.port}")

        except Exception as e:
            self.enabled = False
            self.sock = None
            print(f"[WARN] TCP bridge connection failed: {e}")

    def rel_time_ns(self, timestamp_ns: Optional[int] = None) -> int:
        if timestamp_ns is None:
            timestamp_ns = time.monotonic_ns()
        return int(timestamp_ns - self.stream_start_ns)

    def send_data(self, data: dict):
        if not self.enabled or self.sock is None:
            return

        try:
            with self.send_lock:
                send_msg(self.sock, data)
        except Exception as e:
            print(f"[WARN] TCP send failed: {e}")
            self.enabled = False
            self._close_socket()

    def send_imu(self, timestamp_ns: int, gyro, acc):
        data = {
            "type": "imu",
            "t_ns": self.rel_time_ns(timestamp_ns),
            "gyro": [float(gyro[0]), float(gyro[1]), float(gyro[2])],
            "acc": [float(acc[0]), float(acc[1]), float(acc[2])],
        }
        self.send_data(data)

    def send_image(
        self,
        timestamp_ns: int,
        frame,
        cam_id: int = 0,
        transport: str = "jpg",
        quality: int = 90,
    ):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        transport = transport.lower()
        quality = int(max(1, min(100, quality)))

        if transport == "raw":
            payload = bytes(gray.data)
            data = {
                "type": "image",
                "t_ns": self.rel_time_ns(timestamp_ns),
                "encoding": "raw_mono8",
                "payload": payload,
                "shape": (int(gray.shape[0]), int(gray.shape[1])),
                "cam_id": int(cam_id),
            }
            self.send_data(data)
            return

        if transport == "jpg":
            ok, enc = cv2.imencode(
                ".jpg",
                gray,
                [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            )
            encoding = "jpeg"

        elif transport == "webp":
            ok, enc = cv2.imencode(
                ".webp",
                gray,
                [int(cv2.IMWRITE_WEBP_QUALITY), quality]
            )
            encoding = "webp"

        elif transport == "png":
            ok, enc = cv2.imencode(".png", gray)
            encoding = "png"

        else:
            raise ValueError(f"Unsupported image transport: {transport}")

        if not ok:
            raise RuntimeError(f"Failed to encode image with transport={transport}")

        data = {
            "type": "image",
            "t_ns": self.rel_time_ns(timestamp_ns),
            "encoding": encoding,
            "payload": enc.tobytes(),
            "shape": (int(gray.shape[0]), int(gray.shape[1])),
            "cam_id": int(cam_id),
        }
        self.send_data(data)

    def _receiver_thread(self):
        while not self.stop_event.is_set() and self.sock is not None:
            try:
                raw_len = recvall(self.sock, 4)
                if not raw_len:
                    break

                msglen = struct.unpack(">I", raw_len)[0]
                payload = recvall(self.sock, msglen)
                if payload is None:
                    break

                msg = pickle.loads(payload)

                if isinstance(msg, dict) and "pos" in msg:
                    print("RECV:", msg["pos"])
                    try:
                        self.estimated_data.append([
                            msg.get("t_ros", None),
                            float(msg["pos"][0]),
                            float(msg["pos"][1]),
                            float(msg["pos"][2]),
                        ])
                    except Exception:
                        pass
                else:
                    print("RECV:", msg)

            except Exception:
                break

        self.enabled = False
        self._close_socket()

    def _close_socket(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def close(self):
        self.stop_event.set()
        self.enabled = False
        self._close_socket()

        if self.receiver is not None and self.receiver.is_alive():
            self.receiver.join(timeout=1.0)


# ----------------------------
# MAVLink connection helper
# ----------------------------
def connect(endpoint: str, baud: int, timeout: float):
    if endpoint.lower().startswith("com:"):
        port = endpoint.split(":", 1)[1]
        return mavutil.mavlink_connection(port, baud=baud, timeout=timeout)
    return mavutil.mavlink_connection(endpoint, baud=baud, timeout=timeout)


# ----------------------------
# Generic helpers
# ----------------------------
def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


# ----------------------------
# RC helpers
# ----------------------------
def send_rc_override(master, ch1=0, ch2=0, ch3=0, ch4=0, ch5=0, ch6=0, ch7=0, ch8=0):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        int(ch1), int(ch2), int(ch3), int(ch4),
        int(ch5), int(ch6), int(ch7), int(ch8)
    )


def release_all_overrides(master):
    send_rc_override(master, 0, 0, 0, 0, 0, 0, 0, 0)


# ----------------------------
# MAVLink helpers
# ----------------------------
def wait_heartbeat_or_die(master, timeout_s: float = 10.0):
    hb = master.wait_heartbeat(timeout=timeout_s)
    if hb is None:
        raise RuntimeError(f"No HEARTBEAT received in {timeout_s} seconds.")
    return hb


def set_mode_or_die(master, mode: str, timeout_s: float = 8.0):
    mode_map = master.mode_mapping()
    if mode not in mode_map:
        raise RuntimeError(f"Mode '{mode}' not supported. Available: {sorted(mode_map.keys())}")

    mode_id = mode_map[mode]
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


def request_message_interval(master, message_name: str, rate_hz: float):
    try:
        msg_id_attr = f"MAVLINK_MSG_ID_{message_name}"
        msg_id = getattr(mavutil.mavlink, msg_id_attr, None)
        if msg_id is None:
            print(f"[WARN] Unknown MAVLink message id for {message_name}")
            return

        interval_us = -1 if rate_hz <= 0 else int(1_000_000.0 / rate_hz)

        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            float(msg_id),
            float(interval_us),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        print(f"[INFO] Requested {message_name} at ~{rate_hz:.1f} Hz")

    except Exception as e:
        print(f"[WARN] Failed to request {message_name} interval: {e}")


def configure_message_streams(master, imu_rate_hz: float, telemetry_rate_hz: float):
    request_message_interval(master, "HIGHRES_IMU", imu_rate_hz)
    request_message_interval(master, "GLOBAL_POSITION_INT", telemetry_rate_hz)
    request_message_interval(master, "LOCAL_POSITION_NED", telemetry_rate_hz)
    request_message_interval(master, "ATTITUDE", telemetry_rate_hz)

    try:
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            int(max(1, telemetry_rate_hz)),
            1,
        )
        print(f"[INFO] Requested fallback MAV_DATA_STREAM_ALL at ~{telemetry_rate_hz:.1f} Hz")
    except Exception as e:
        print(f"[WARN] request_data_stream_send failed (non-fatal): {e}")


# ----------------------------
# HIGHRES_IMU processor
# ----------------------------
class HighresImuProcessor:
    """
    Converts HIGHRES_IMU FCU timestamps into a local monotonic session timeline.

    DISABLED FILE SAVE:
      IMU CSV logging is intentionally commented out.
    """

    def __init__(self, tcp_bridge: Optional[TcpDataBridge] = None):
        self.tcp_bridge = tcp_bridge

        self.base_fcu_ns: Optional[int] = None
        self.base_local_ns: Optional[int] = None
        self.last_fcu_ns: Optional[int] = None
        self.last_sent_ns: Optional[int] = None

        self.has_data = False
        self.sent_count = 0
        self.dropped_non_monotonic = 0

        # DISABLED FILE SAVE (IMU):
        # import csv
        # self.csv_file = open("imu.csv", "w", newline="", encoding="utf-8")
        # self.csv_writer = csv.writer(self.csv_file)
        # self.csv_writer.writerow([
        #     "%time",
        #     "field.header.seq",
        #     "field.header.stamp",
        #     "field.header.frame_id",
        #     "field.angular_velocity.x",
        #     "field.angular_velocity.y",
        #     "field.angular_velocity.z",
        #     "field.linear_acceleration.x",
        #     "field.linear_acceleration.y",
        #     "field.linear_acceleration.z",
        # ])
        # self.csv_seq = 0

    def handle_highres_imu(self, msg):
        time_usec = int(getattr(msg, "time_usec", 0))
        if time_usec <= 0:
            return

        fcu_ns = time_usec * 1000
        now_ns = time.monotonic_ns()

        if self.base_fcu_ns is None:
            self.base_fcu_ns = fcu_ns
            self.base_local_ns = now_ns
            self.last_fcu_ns = fcu_ns

        if self.last_fcu_ns is not None and fcu_ns < self.last_fcu_ns:
            print("[WARN] HIGHRES_IMU time moved backwards, rebasing local timeline")
            self.base_fcu_ns = fcu_ns
            self.base_local_ns = now_ns
            self.last_sent_ns = None

        self.last_fcu_ns = fcu_ns

        stamp_ns = self.base_local_ns + (fcu_ns - self.base_fcu_ns)

        if self.last_sent_ns is not None and stamp_ns <= self.last_sent_ns:
            self.dropped_non_monotonic += 1
            if self.dropped_non_monotonic % 20 == 1:
                print(f"[WARN] Dropping non-monotonic IMU sample (count={self.dropped_non_monotonic})")
            return

        gyro = [
            float(getattr(msg, "xgyro", 0.0)),
            float(getattr(msg, "ygyro", 0.0)),
            float(getattr(msg, "zgyro", 0.0)),
        ]
        acc = [
            float(getattr(msg, "xacc", 0.0)),
            float(getattr(msg, "yacc", 0.0)),
            float(getattr(msg, "zacc", 0.0)),
        ]

        # DISABLED FILE SAVE (IMU):
        # self.csv_writer.writerow([
        #     int(stamp_ns),
        #     self.csv_seq,
        #     int(stamp_ns),
        #     "imu",
        #     gyro[0], gyro[1], gyro[2],
        #     acc[0], acc[1], acc[2],
        # ])
        # self.csv_seq += 1
        # if self.csv_seq % 20 == 0:
        #     self.csv_file.flush()

        if self.tcp_bridge is not None:
            self.tcp_bridge.send_imu(stamp_ns, gyro, acc)

        self.last_sent_ns = stamp_ns
        self.has_data = True
        self.sent_count += 1

    def close(self):
        # DISABLED FILE SAVE (IMU):
        # try:
        #     self.csv_file.flush()
        #     self.csv_file.close()
        # except Exception:
        #     pass
        pass


# ----------------------------
# Camera streamer
# ----------------------------
class CameraStreamer:
    """
    Reads frames from a video stream and immediately sends them over TCP.

    DISABLED FILE SAVE:
      Frame saving to disk is intentionally commented out.
    """

    def __init__(
        self,
        video_url: str,
        tcp_bridge: Optional[TcpDataBridge] = None,
        cam_id: int = 0,
        image_transport: str = "jpg",
        image_quality: int = 90,
    ):
        self.video_url = video_url
        self.tcp_bridge = tcp_bridge
        self.cam_id = cam_id
        self.image_transport = image_transport
        self.image_quality = int(clamp(image_quality, 1, 100))

        self.stop_event = threading.Event()
        self.thread = None
        self.frame_count = 0

        # DISABLED FILE SAVE (CAMERA):
        # self.output_dir = "frames"
        # os.makedirs(self.output_dir, exist_ok=True)
        # self.timestamps_path = os.path.join(self.output_dir, "camera_timestamps.csv")
        # self.timestamps_file = open(self.timestamps_path, "w", newline="", encoding="utf-8")
        # self.timestamps_writer = csv.writer(self.timestamps_file)
        # self.timestamps_writer.writerow(["frame_id", "timestamp_ns", "filename"])

    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        print(f"[INFO] Camera streamer started: {self.video_url}")

    def _run(self):
        cap = None
        try:
            while not self.stop_event.is_set():
                if cap is None or not cap.isOpened():
                    if cap is not None:
                        try:
                            cap.release()
                        except Exception:
                            pass

                    print(f"[INFO] Connecting to video stream: {self.video_url}")
                    cap = cv2.VideoCapture(self.video_url)

                    if not cap.isOpened():
                        print(f"[WARN] Failed to open video stream: {self.video_url}")
                        time.sleep(1.0)
                        continue

                ret, frame = cap.read()

                if not ret or frame is None:
                    print("[WARN] Video frame not received, reconnecting...")
                    try:
                        cap.release()
                    except Exception:
                        pass
                    cap = None
                    time.sleep(0.2)
                    continue

                timestamp_ns = time.monotonic_ns()

                # DISABLED FILE SAVE (CAMERA):
                # filename = f"frame_{self.frame_count:06d}.jpg"
                # cv2.imwrite(os.path.join(self.output_dir, filename), frame)
                # self.timestamps_writer.writerow([self.frame_count, int(timestamp_ns), filename])
                # if self.frame_count % 30 == 0:
                #     self.timestamps_file.flush()

                if self.tcp_bridge is not None:
                    try:
                        self.tcp_bridge.send_image(
                            timestamp_ns=timestamp_ns,
                            frame=frame,
                            cam_id=self.cam_id,
                            transport=self.image_transport,
                            quality=self.image_quality,
                        )
                    except Exception as e:
                        print(f"[WARN] Failed to encode/send frame: {e}")

                self.frame_count += 1
                if self.frame_count % 30 == 0:
                    print(f"[VIDEO] Sent frames: {self.frame_count}")

        except Exception as e:
            print(f"[ERROR] Camera thread error: {e}")
        finally:
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass

    def stop(self):
        self.stop_event.set()
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=3.0)

        # DISABLED FILE SAVE (CAMERA):
        # try:
        #     self.timestamps_file.flush()
        #     self.timestamps_file.close()
        # except Exception:
        #     pass

        print("[INFO] Camera streamer stopped")


# ----------------------------
# Telemetry state
# ----------------------------
@dataclass
class TelemetryState:
    alt_m: Optional[float] = None
    x_n: Optional[float] = None
    y_e: Optional[float] = None
    vx_n: Optional[float] = None
    vy_e: Optional[float] = None
    yaw_rad: Optional[float] = None


# ----------------------------
# Telemetry pump
# ----------------------------
def pump_telemetry(
    master,
    st: TelemetryState,
    imu_processor: Optional[HighresImuProcessor] = None,
    max_msgs: int = 300
):
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

        elif mtype == "HIGHRES_IMU":
            if imu_processor is not None:
                imu_processor.handle_highres_imu(msg)


def wait_for_telemetry(
    master,
    st: TelemetryState,
    imu_processor: Optional[HighresImuProcessor] = None,
    timeout_s: float = 10.0,
):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        pump_telemetry(master, st, imu_processor=imu_processor)

        state_ready = (
            st.alt_m is not None and
            st.x_n is not None and
            st.y_e is not None and
            st.yaw_rad is not None
        )

        imu_ready = True if imu_processor is None else imu_processor.has_data

        if state_ready and imu_ready:
            return

        time.sleep(0.02)

    missing = []
    if st.alt_m is None:
        missing.append("GLOBAL_POSITION_INT.relative_alt")
    if st.x_n is None or st.y_e is None:
        missing.append("LOCAL_POSITION_NED")
    if st.yaw_rad is None:
        missing.append("ATTITUDE.yaw")
    if imu_processor is not None and not imu_processor.has_data:
        missing.append("HIGHRES_IMU")

    raise RuntimeError("Telemetry not ready. Missing: " + ", ".join(missing))


# ----------------------------
# Control config
# ----------------------------
@dataclass
class ControlConfig:
    roll_center: int = 1500
    pitch_center: int = 1500
    yaw_pwm: int = 1500

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
# Controllers
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


def send_controls(
    master,
    st: TelemetryState,
    target_alt_m: float,
    target_x_n: float,
    target_y_e: float,
    cfg: ControlConfig,
    imu_processor: Optional[HighresImuProcessor] = None
):
    pump_telemetry(master, st, imu_processor=imu_processor)

    if st.alt_m is None:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1.0)
        if msg is not None:
            st.alt_m = float(msg.relative_alt) / 1000.0

    if st.alt_m is None:
        return

    thr = compute_throttle_pwm(target_alt_m, st.alt_m, cfg)
    roll_pwm, pitch_pwm = compute_xy_hold_pwm(st, target_x_n, target_y_e, cfg)

    send_rc_override(master, roll_pwm, pitch_pwm, thr, cfg.yaw_pwm)


# ----------------------------
# Scenario steps
# ----------------------------
def hold(
    master,
    st: TelemetryState,
    target_alt_m: float,
    target_x_n: float,
    target_y_e: float,
    seconds: float,
    cfg: ControlConfig,
    label: str,
    imu_processor: Optional[HighresImuProcessor] = None
):
    dt = 1.0 / cfg.rc_hz
    t_end = time.monotonic() + seconds
    last_print = 0.0

    while time.monotonic() < t_end:
        send_controls(master, st, target_alt_m, target_x_n, target_y_e, cfg, imu_processor=imu_processor)

        now = time.monotonic()
        if now - last_print > 0.5 and st.alt_m is not None and st.x_n is not None and st.y_e is not None:
            dx = st.x_n - target_x_n
            dy = st.y_e - target_y_e
            dist = math.sqrt(dx * dx + dy * dy)
            print(f"[{label}] alt={st.alt_m:5.2f}m drift={dist:5.2f}m xN={st.x_n:7.2f} yE={st.y_e:7.2f}")
            last_print = now

        time.sleep(dt)


def hold_until_ctrl_c(
    master,
    st: TelemetryState,
    target_alt_m: float,
    target_x_n: float,
    target_y_e: float,
    cfg: ControlConfig,
    imu_processor: Optional[HighresImuProcessor] = None
):
    dt = 1.0 / cfg.rc_hz
    last_print = 0.0

    while True:
        send_controls(master, st, target_alt_m, target_x_n, target_y_e, cfg, imu_processor=imu_processor)

        now = time.monotonic()
        if now - last_print > 0.5 and st.alt_m is not None and st.x_n is not None and st.y_e is not None:
            dx = st.x_n - target_x_n
            dy = st.y_e - target_y_e
            dist = math.sqrt(dx * dx + dy * dy)
            print(f"[HOLD_LOOP] alt={st.alt_m:5.2f}m target={target_alt_m:5.2f}m drift={dist:5.2f}m")
            last_print = now

        time.sleep(dt)


def reach_altitude(
    master,
    st: TelemetryState,
    target_alt_m: float,
    target_x_n: float,
    target_y_e: float,
    settle_band_m: float,
    settle_time_s: float,
    cfg: ControlConfig,
    imu_processor: Optional[HighresImuProcessor] = None
):
    dt = 1.0 / cfg.rc_hz
    stable_t0 = None
    last_print = 0.0

    while True:
        send_controls(master, st, target_alt_m, target_x_n, target_y_e, cfg, imu_processor=imu_processor)

        now = time.monotonic()
        if st.alt_m is None:
            time.sleep(dt)
            continue

        if now - last_print > 0.5 and st.x_n is not None and st.y_e is not None:
            err = target_alt_m - st.alt_m
            dx = st.x_n - target_x_n
            dy = st.y_e - target_y_e
            dist = math.sqrt(dx * dx + dy * dy)
            print(f"[CLIMB] alt={st.alt_m:5.2f} target={target_alt_m:5.2f} err={err:5.2f} drift={dist:5.2f}m")
            last_print = now

        if abs(st.alt_m - target_alt_m) <= settle_band_m:
            if stable_t0 is None:
                stable_t0 = now
            elif now - stable_t0 >= settle_time_s:
                return st.alt_m
        else:
            stable_t0 = None

        time.sleep(dt)


def controlled_descent(
    master,
    st: TelemetryState,
    start_alt_m: float,
    target_x_n: float,
    target_y_e: float,
    cfg: ControlConfig,
    imu_processor: Optional[HighresImuProcessor] = None,
    step_m: float = 0.4,
    dwell_s: float = 0.7,
    final_alt_m: float = 0.8
):
    current_target = start_alt_m
    last_print = 0.0

    while current_target > final_alt_m:
        current_target = max(final_alt_m, current_target - step_m)
        t_end = time.monotonic() + dwell_s

        while time.monotonic() < t_end:
            send_controls(master, st, current_target, target_x_n, target_y_e, cfg, imu_processor=imu_processor)

            now = time.monotonic()
            if now - last_print > 0.5 and st.alt_m is not None:
                print(f"[DESCENT] target={current_target:5.2f}m alt={st.alt_m:5.2f}m")
                last_print = now

            time.sleep(1.0 / cfg.rc_hz)

    hold(master, st, final_alt_m, target_x_n, target_y_e, 1.0, cfg, label="LOW_HOLD", imu_processor=imu_processor)


# ----------------------------
# Main
# ----------------------------
def main():
    parser = argparse.ArgumentParser(
        description="ArduPilot SITL -> HIGHRES_IMU + compressed camera -> TCP -> ROS/OpenVINS."
    )

    parser.add_argument("--endpoint", default="tcp:127.0.0.1:5762", help="MAVLink endpoint.")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)

    parser.add_argument("--mode", default="ALT_HOLD")
    parser.add_argument("--takeoff-alt", type=float, default=3.0)
    parser.add_argument("--hold-s", type=float, default=3.0)
    parser.add_argument("--kp-xy", type=float, default=80.0)
    parser.add_argument("--kd-xy", type=float, default=50.0)
    parser.add_argument("--max-xy-delta", type=int, default=220)
    parser.add_argument("--invert-roll", action="store_true")
    parser.add_argument("--invert-pitch", action="store_true")

    parser.add_argument("--video-url", default="http://127.0.0.1:8080/video", help="Video stream URL.")
    parser.add_argument("--cam-id", type=int, default=0)

    parser.add_argument("--image-transport", default="jpg", choices=["raw", "jpg", "webp", "png"])
    parser.add_argument("--image-quality", type=int, default=90)

    parser.add_argument("--tcp-host", default="192.168.111.138", help="TCP receiver host.")
    parser.add_argument("--tcp-port", type=int, default=5000, help="TCP receiver port.")
    parser.add_argument("--tcp-timeout", type=float, default=5.0, help="TCP connect timeout in seconds.")
    parser.add_argument("--disable-tcp", action="store_true", help="Disable TCP streaming.")

    parser.add_argument("--imu-rate-hz", type=float, default=200.0)
    parser.add_argument("--telem-rate-hz", type=float, default=50.0)

    args = parser.parse_args()

    imu_processor = None
    camera_streamer = None
    tcp_bridge = None

    try:
        if not args.disable_tcp:
            tcp_bridge = TcpDataBridge(
                host=args.tcp_host,
                port=args.tcp_port,
                connect_timeout=args.tcp_timeout,
            )
            tcp_bridge.connect()

        camera_streamer = CameraStreamer(
            video_url=args.video_url,
            tcp_bridge=tcp_bridge,
            cam_id=args.cam_id,
            image_transport=args.image_transport,
            image_quality=args.image_quality,
        )
        camera_streamer.start()

        print(f"[INFO] Connecting: {args.endpoint}")
        master = connect(args.endpoint, args.baud, args.timeout)

        wait_heartbeat_or_die(master)
        print(f"[OK] Heartbeat: sys={master.target_system} comp={master.target_component}")

        configure_message_streams(master, args.imu_rate_hz, args.telem_rate_hz)

        imu_processor = HighresImuProcessor(tcp_bridge=tcp_bridge)

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

        print("[INFO] Waiting for telemetry (HIGHRES_IMU + GLOBAL_POSITION_INT + LOCAL_POSITION_NED + ATTITUDE)...")
        wait_for_telemetry(master, st, imu_processor=imu_processor)
        print(
            f"[INFO] Telemetry ready: alt={st.alt_m:.2f}m "
            f"xN={st.x_n:.2f} yE={st.y_e:.2f} yaw={st.yaw_rad:.2f}rad "
            f"imu_samples={imu_processor.sent_count}"
        )

        target_x_n = st.x_n
        target_y_e = st.y_e
        print(f"[INFO] XY hold target locked: xN={target_x_n:.2f}m yE={target_y_e:.2f}m")

        print("[INFO] Takeoff boost (2s)...")
        dt = 1.0 / cfg.rc_hz
        t_end = time.monotonic() + 2.0
        while time.monotonic() < t_end:
            pump_telemetry(master, st, imu_processor=imu_processor)
            roll_pwm, pitch_pwm = compute_xy_hold_pwm(st, target_x_n, target_y_e, cfg)
            send_rc_override(master, roll_pwm, pitch_pwm, 1750, cfg.yaw_pwm)
            time.sleep(dt)

        print(f"[INFO] Taking off to {args.takeoff_alt:.1f} m (ALT_HOLD + XY hold)...")
        reached = reach_altitude(
            master, st, args.takeoff_alt, target_x_n, target_y_e,
            settle_band_m=0.8, settle_time_s=0.6,
            cfg=cfg, imu_processor=imu_processor
        )
        print(f"[OK] Reached ~{reached:.2f} m")

        print(f"[INFO] Holding {args.takeoff_alt:.1f} m for {args.hold_s:.1f} s")
        hold(master, st, args.takeoff_alt, target_x_n, target_y_e, args.hold_s, cfg, "HOLD", imu_processor=imu_processor)

        print(f"[INFO] Continuous hold at {args.takeoff_alt:.1f} m. Ctrl+C to start smooth descent.")
        try:
            hold_until_ctrl_c(master, st, args.takeoff_alt, target_x_n, target_y_e, cfg, imu_processor=imu_processor)
        except KeyboardInterrupt:
            print("\n[INFO] Cancel requested. Starting smooth descent...")

        start_alt = st.alt_m if st.alt_m is not None else args.takeoff_alt
        controlled_descent(
            master, st, start_alt,
            target_x_n, target_y_e,
            cfg,
            imu_processor=imu_processor,
            step_m=0.4,
            dwell_s=0.7,
            final_alt_m=0.8
        )

        print("[INFO] Releasing RC overrides near ground...")
        release_all_overrides(master)
        time.sleep(1.0)
        pump_telemetry(master, st, imu_processor=imu_processor)

        print("[INFO] Disarming...")
        disarm_or_die(master)
        print("[OK] Done")

    finally:
        if camera_streamer is not None:
            camera_streamer.stop()

        if imu_processor is not None:
            imu_processor.close()

        if tcp_bridge is not None:
            tcp_bridge.close()

        if imu_processor is not None:
            print(
                f"[INFO] IMU summary: sent={imu_processor.sent_count}, "
                f"dropped_non_monotonic={imu_processor.dropped_non_monotonic}"
            )


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[ERROR] {e}")
        sys.exit(1)