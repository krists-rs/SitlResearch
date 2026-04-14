#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import sys
import time

from pymavlink import mavutil


def connect(endpoint: str, baud: int, timeout: float):
    """
    endpoint examples:
      - tcp:127.0.0.1:5760        (Mission Planner SITL SERIAL0)
      - udp:127.0.0.1:14550       (часто используется для GCS)
      - com:COM7                  (Windows serial)
      - /dev/ttyUSB0              (Linux serial)
    """
    # pymavlink: serial обычно просто "COM7" или "/dev/ttyUSB0"
    # tcp/udp — строго с префиксом tcp: / udp:
    if endpoint.lower().startswith("com:"):
        port = endpoint.split(":", 1)[1]
        link = mavutil.mavlink_connection(port, baud=baud, timeout=timeout)
    else:
        link = mavutil.mavlink_connection(endpoint, baud=baud, timeout=timeout)

    return link


def main():
    parser = argparse.ArgumentParser(description="Connect to ArduPilot SITL via Mission Planner port and read MAVLink.")
    parser.add_argument(
        "--endpoint",
        default="tcp:127.0.0.1:5762",
        help="MAVLink endpoint. Default: tcp:127.0.0.1:5760 (SITL SERIAL0). "
             "Examples: udp:127.0.0.1:14550, com:COM7, /dev/ttyUSB0",
    )
    parser.add_argument("--baud", type=int, default=115200, help="Serial baudrate (only for serial). Default 115200.")
    parser.add_argument("--timeout", type=float, default=1.0, help="Read timeout seconds. Default 1.0.")
    parser.add_argument("--rate-hz", type=float, default=5.0, help="Requested data stream rate (best-effort). Default 5 Hz.")
    args = parser.parse_args()

    print(f"[INFO] Connecting to: {args.endpoint}")
    link = connect(args.endpoint, args.baud, args.timeout)

    # Ждём heartbeat — это базовая проверка, что канал живой и MAVLink идёт
    print("[INFO] Waiting for HEARTBEAT...")
    hb = link.wait_heartbeat(timeout=10)
    if hb is None:
        print("[ERROR] No HEARTBEAT received (10s). Check endpoint/port and that SITL is running.")
        sys.exit(1)

    print(
        f"[OK] HEARTBEAT from system={link.target_system} component={link.target_component} "
        f"type={hb.type} autopilot={hb.autopilot} base_mode={hb.base_mode} custom_mode={hb.custom_mode}"
    )

    # Пытаемся запросить потоки (в SITL обычно работает; но это best-effort)
    # MAV_DATA_STREAM_ALL = 0, но в MAVLink1 некоторые реализации лучше воспринимают конкретные потоки.
    # Для простоты просим ALL, а дальше просто читаем то, что приходит.
    try:
        link.mav.request_data_stream_send(
            link.target_system,
            link.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            int(max(1, args.rate_hz)),
            1,
        )
        print(f"[INFO] Requested data stream rate ~{args.rate_hz} Hz")
    except Exception as e:
        print(f"[WARN] request_data_stream_send failed (non-fatal): {e}")

    print("[INFO] Reading messages (CTRL+C to stop)...")

    last_print = 0.0
    while True:
        msg = link.recv_match(blocking=True, timeout=args.timeout)
        if msg is None:
            continue

        mtype = msg.get_type()
        if mtype == "BAD_DATA":
            # иногда бывает мусор на линии/неполный пакет
            continue

        now = time.time()

        # Для старта печатаем ключевые сообщения
        if mtype in ("HEARTBEAT", "SYS_STATUS", "GLOBAL_POSITION_INT", "ATTITUDE", "STATUSTEXT"):
            if mtype == "STATUSTEXT":
                print(f"[STATUSTEXT] {msg.text}")
            elif mtype == "GLOBAL_POSITION_INT":
                # lat/lon в 1e7, alt в мм
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt_m = msg.alt / 1000.0
                rel_alt_m = msg.relative_alt / 1000.0
                if now - last_print > 0.2:
                    print(f"[POS] lat={lat:.7f} lon={lon:.7f} alt={alt_m:.1f}m rel={rel_alt_m:.1f}m")
                    last_print = now
            elif mtype == "ATTITUDE":
                # rad -> deg
                roll = msg.roll * 57.2957795
                pitch = msg.pitch * 57.2957795
                yaw = msg.yaw * 57.2957795
                if now - last_print > 0.2:
                    print(f"[ATT] roll={roll:.1f} pitch={pitch:.1f} yaw={yaw:.1f}")
                    last_print = now
            else:
                # Остальные просто одной строкой, чтобы видеть что идёт поток
                if now - last_print > 0.5:
                    print(f"[{mtype}] {msg.to_dict()}")
                    last_print = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
