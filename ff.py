#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import sys
import time
import os
import threading
import cv2
import requests
import numpy as np
from pymavlink import mavutil

def connect(endpoint: str, baud: int, timeout: float):
    if endpoint.lower().startswith("com:"):
        port = endpoint.split(":", 1)[1]
        link = mavutil.mavlink_connection(port, baud=baud, timeout=timeout)
    else:
        link = mavutil.mavlink_connection(endpoint, baud=baud, timeout=timeout)
    return link

def video_capture_thread(url, save_path):
    """Функция для захвата кадров в отдельном потоке"""
    if not os.path.exists(save_path):
        os.makedirs(save_path)
        print(f"[INFO] Created directory: {save_path}")

    print(f"[INFO] Starting video capture from {url}")
    
    frame_count = 0
    while True:
        try:
            # Делаем GET запрос к симулятору
            response = requests.get(url, timeout=2)
            if response.status_code == 200:
                # Превращаем байты в изображение OpenCV
                image_array = np.frombuffer(response.content, dtype=np.uint8)
                frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

                if frame is not None:
                    file_name = os.path.join(save_path, f"frame_{frame_count:05d}.jpg")
                    cv2.imwrite(file_name, frame)
                    frame_count += 1
                    
                    if frame_count % 30 == 0:
                        print(f"[VIDEO] Saved {frame_count} frames...")
                else:
                    print("[WARN] Failed to decode image from GET response")
            else:
                print(f"[WARN] Video server returned status: {response.status_code}")
        
        except Exception as e:
            print(f"[ERROR] Video capture error: {e}")
            time.sleep(1) # Ждем секунду перед повтором при ошибке
        
        # Ограничиваем частоту запросов (например, 20 FPS), чтобы не перегружать сеть
        time.sleep(0.05)

def main():
    parser = argparse.ArgumentParser(description="Connect to ArduPilot SITL and save video frames.")
    parser.add_argument("--endpoint", default="tcp:127.0.0.1:5762", help="MAVLink endpoint.")
    parser.add_argument("--video-url", default="tcp://127.0.0.1:8080/video", help="URL for video GET requests.")
    parser.add_argument("--output", default="frames", help="Folder to save frames.")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--rate-hz", type=float, default=5.0)
    args = parser.parse_args()

    # --- ЗАПУСК ПОТОКА ВИДЕО ---
    video_thread = threading.Thread(
        target=video_capture_thread, 
        args=(args.video_url, args.output),
        daemon=True # Поток умрет сам при выходе из основной программы
    )
    video_thread.start()

    # --- РАБОТА С MAVLINK ---
    print(f"[INFO] Connecting to MAVLink: {args.endpoint}")
    link = connect(args.endpoint, args.baud, args.timeout)

    print("[INFO] Waiting for HEARTBEAT...")
    hb = link.wait_heartbeat(timeout=10)
    if hb is None:
        print("[ERROR] No HEARTBEAT received.")
        sys.exit(1)

    print(f"[OK] HEARTBEAT received. System {link.target_system}")

    try:
        link.mav.request_data_stream_send(
            link.target_system, link.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, int(max(1, args.rate_hz)), 1
        )
    except Exception as e:
        print(f"[WARN] Stream request failed: {e}")

    last_print = 0.0
    while True:
        msg = link.recv_match(blocking=True, timeout=args.timeout)
        if msg is None: continue
        
        mtype = msg.get_type()
        if mtype == "BAD_DATA": continue

        now = time.time()
        # Вывод телеметрии (как в вашем исходном коде)
        if mtype == "GLOBAL_POSITION_INT" and now - last_print > 0.5:
            print(f"[POS] Alt: {msg.relative_alt/1000.0}m")
            last_print = now

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")