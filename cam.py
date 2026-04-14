#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import sys
import time
import os
import threading
import cv2
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
    """Функция для надежного захвата кадров через OpenCV"""
    if not os.path.exists(save_path):
        os.makedirs(save_path)
        print(f"[INFO] Создана папка для кадров: {save_path}")

    print(f"[INFO] Подключение к видеопотоку: {url}")
    
    # Используем OpenCV для захвата потока (работает лучше requests для видео)
    cap = cv2.VideoCapture(url)
    
    if not cap.isOpened():
        print(f"[ERROR] Не удалось открыть поток по адресу {url}")
        return

    frame_count = 0
    try:
        while True:
            ret, frame = cap.read()
            
            if ret and frame is not None:
                # Формируем имя файла с ведущими нулями для правильной сортировки
                file_name = os.path.join(save_path, f"frame_{frame_count:05d}.jpg")
                
                # Сохраняем кадр на диск
                cv2.imwrite(file_name, frame)
                frame_count += 1
                
                if frame_count % 30 == 0:
                    print(f"[VIDEO] Сохранено кадров: {frame_count}")
            else:
                # Если кадр не получен, ждем немного и пробуем переподключиться
                # print("[WARN] Кадр не получен, ожидание...")
                time.sleep(0.1)
                
    except Exception as e:
        print(f"[ERROR] Ошибка в потоке видео: {e}")
    finally:
        cap.release()

def main():
    parser = argparse.ArgumentParser(description="Connect to ArduPilot SITL and save video frames.")
    parser.add_argument("--endpoint", default="tcp:127.0.0.1:5762", help="MAVLink endpoint.")
    # Используем проверенный вами адрес
    parser.add_argument("--video-url", default="http://127.0.0.1:8080/video", help="URL потока.")
    parser.add_argument("--output", default="frames", help="Папка для сохранения.")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--rate-hz", type=float, default=5.0)
    args = parser.parse_args()

    # --- ЗАПУСК ПОТОКА ВИДЕО ---
    video_thread = threading.Thread(
        target=video_capture_thread, 
        args=(args.video_url, args.output),
        daemon=True 
    )
    video_thread.start()

    # --- РАБОТА С MAVLINK ---
    print(f"[INFO] Подключение к MAVLink: {args.endpoint}")
    link = connect(args.endpoint, args.baud, args.timeout)

    print("[INFO] Ожидание HEARTBEAT...")
    hb = link.wait_heartbeat(timeout=10)
    if hb is None:
        print("[ERROR] HEARTBEAT не получен. Проверьте запуск SITL.")
        sys.exit(1)

    print(f"[OK] HEARTBEAT получен. Система: {link.target_system}")

    # Запрос потока данных телеметрии
    link.mav.request_data_stream_send(
        link.target_system, link.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, int(args.rate_hz), 1
    )

    last_print = 0.0
    try:
        while True:
            msg = link.recv_match(blocking=True, timeout=args.timeout)
            if msg is None: continue
            
            mtype = msg.get_type()
            if mtype == "BAD_DATA": continue

            now = time.time()
            # Вывод высоты для контроля связи
            if mtype == "GLOBAL_POSITION_INT" and now - last_print > 1.0:
                print(f"[TELEMETRY] Высота: {msg.relative_alt/1000.0}м")
                last_print = now
                
    except KeyboardInterrupt:
        print("\n[INFO] Завершение работы...")

if __name__ == "__main__":
    main()