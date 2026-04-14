#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import pickle
import socket
import struct
import threading

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image, Imu


HOST = "0.0.0.0"
PORT = 5000

bridge = CvBridge()

conn = None
conn_lock = threading.Lock()
session_start_ros = None


def send_msg(sock, data):
    payload = pickle.dumps(data, protocol=2)
    sock.sendall(struct.pack(">I", len(payload)) + payload)


def recvall(sock, size):
    data = b""
    while len(data) < size and not rospy.is_shutdown():
        try:
            chunk = sock.recv(size - len(data))
        except socket.timeout:
            continue

        if not chunk:
            return None
        data += chunk
    return data


def recv_msg(sock):
    raw_len = recvall(sock, 4)
    if raw_len is None:
        return None

    msglen = struct.unpack(">I", raw_len)[0]
    payload = recvall(sock, msglen)
    return payload


def duration_from_ns(total_ns):
    secs = int(total_ns // 1000000000)
    nsecs = int(total_ns % 1000000000)
    return rospy.Duration(secs=secs, nsecs=nsecs)


def close_current_connection():
    global conn

    with conn_lock:
        if conn is not None:
            try:
                conn.close()
            except Exception:
                pass
            conn = None


def ov_callback(msg):
    global conn

    with conn_lock:
        if conn is None:
            return

        try:
            p = msg.pose.pose.position
            res = {
                "t_ros": msg.header.stamp.to_sec(),
                "pos": [float(p.x), float(p.y), float(p.z)],
            }
            send_msg(conn, res)

        except Exception as e:
            print("SEND BACK ERROR: {}".format(e))


def main():
    global conn
    global session_start_ros

    rospy.init_node("tcp_bridge_server")

    imu_pub = rospy.Publisher("/imu0", Imu, queue_size=400)
    cam0_pub = rospy.Publisher("/cam0/image_raw", Image, queue_size=30)
    cam1_pub = rospy.Publisher("/cam1/image_raw", Image, queue_size=30)

    rospy.Subscriber("/ov_msckf/poseimu", PoseWithCovarianceStamped, ov_callback, queue_size=10)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)
    server.settimeout(1.0)

    print("[INFO] TCP bridge server listening on {}:{}".format(HOST, PORT))

    while not rospy.is_shutdown():
        try:
            print("\n[WAITING] Ожидание подключения Windows...")
            try:
                new_conn, addr = server.accept()
            except socket.timeout:
                continue

            new_conn.settimeout(1.0)
            new_conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

            with conn_lock:
                conn = new_conn

            session_start_ros = rospy.Time.now()

            print("[CONNECTED] Подключено: {}".format(addr))

            last_imu_t_ns = None
            last_cam_t_ns = {
                0: None,
                1: None,
            }

            while not rospy.is_shutdown():
                raw = recv_msg(new_conn)
                if raw is None:
                    print("[INFO] Client disconnected")
                    break

                try:
                    msg = pickle.loads(raw)
                except Exception as e:
                    print("[WARN] Failed to decode message: {}".format(e))
                    continue

                msg_type = msg.get("type")
                t_ns = int(msg.get("t_ns", -1))
                if t_ns < 0:
                    print("[WARN] Message without valid t_ns, skipped")
                    continue

                stamp = session_start_ros + duration_from_ns(t_ns)

                if msg_type == "imu":
                    if last_imu_t_ns is not None and t_ns <= last_imu_t_ns:
                        print("[WARN] Non-monotonic IMU timestamp skipped: {} <= {}".format(t_ns, last_imu_t_ns))
                        continue

                    last_imu_t_ns = t_ns

                    gyro = msg.get("gyro", [0.0, 0.0, 0.0])
                    acc = msg.get("acc", [0.0, 0.0, 0.0])

                    m = Imu()
                    m.header.stamp = stamp
                    m.header.frame_id = "imu"

                    # Orientation unavailable
                    m.orientation_covariance[0] = -1.0

                    m.angular_velocity.x = float(gyro[0])
                    m.angular_velocity.y = float(gyro[1])
                    m.angular_velocity.z = -float(gyro[2])

                    m.linear_acceleration.x = float(acc[0])
                    m.linear_acceleration.y = float(acc[1])
                    m.linear_acceleration.z = -float(acc[2])

                    imu_pub.publish(m)

                elif msg_type == "image":
                    cam_id = int(msg.get("cam_id", 0))

                    last_cam = last_cam_t_ns.get(cam_id)
                    if last_cam is not None and t_ns <= last_cam:
                        print("[WARN] Non-monotonic camera timestamp skipped for cam{}: {} <= {}".format(cam_id, t_ns, last_cam))
                        continue
                    last_cam_t_ns[cam_id] = t_ns

                    encoding = msg.get("encoding", "raw_mono8")
                    payload = msg.get("payload", None)
                    shape = msg.get("shape", None)

                    if payload is None:
                        print("[WARN] Image payload missing, skipped")
                        continue

                    if encoding == "raw_mono8":
                        if shape is None:
                            print("[WARN] raw_mono8 image without shape, skipped")
                            continue

                        h = int(shape[0])
                        w = int(shape[1])

                        if len(payload) != h * w:
                            print("[WARN] raw_mono8 size mismatch: len={} expected={}".format(len(payload), h * w))
                            continue

                        frame = np.frombuffer(payload, dtype=np.uint8).reshape(h, w)

                    elif encoding in ("jpeg", "webp", "png"):
                        arr = np.frombuffer(payload, dtype=np.uint8)
                        frame = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)

                        if frame is None:
                            print("[WARN] Failed to decode compressed image: {}".format(encoding))
                            continue

                    else:
                        print("[WARN] Unsupported image encoding: {}".format(encoding))
                        continue

                    ros_img = bridge.cv2_to_imgmsg(frame, encoding="mono8")
                    ros_img.header.stamp = stamp

                    if cam_id == 1:
                        ros_img.header.frame_id = "cam1"
                        cam1_pub.publish(ros_img)
                    else:
                        ros_img.header.frame_id = "cam0"
                        cam0_pub.publish(ros_img)

                else:
                    print("[WARN] Unknown message type: {}".format(msg_type))

        except Exception as e:
            print("[ERROR] {}".format(e))

        finally:
            close_current_connection()

    try:
        server.close()
    except Exception:
        pass


if __name__ == "__main__":
    main()