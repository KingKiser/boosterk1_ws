#!/usr/bin/env python3

import socket
import time

import cv2
import numpy as np


PORT = 5000

sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM
)

# 수신 버퍼를 너무 크게 잡지 않음
sock.setsockopt(
    socket.SOL_SOCKET,
    socket.SO_RCVBUF,
    1024 * 1024
)

sock.bind(
    ("0.0.0.0", PORT)
)

print(f"Listening UDP :{PORT}")

frames = {}

last_frame_id = None

fps_count = 0
fps_start = time.perf_counter()


while True:

    packet, addr = sock.recvfrom(65535)

    if len(packet) < 8:
        continue

    frame_id = int.from_bytes(
        packet[0:4],
        'big'
    )

    index = int.from_bytes(
        packet[4:6],
        'big'
    )

    total = int.from_bytes(
        packet[6:8],
        'big'
    )

    payload = packet[8:]

    # 새 프레임이 왔는데 이전 프레임이 완성되지 않았다면
    # 이전 프레임을 버린다.
    if (
        last_frame_id is not None
        and frame_id != last_frame_id
    ):
        frames.clear()

    last_frame_id = frame_id

    if frame_id not in frames:
        frames[frame_id] = {
            "total": total,
            "chunks": {}
        }

    frames[frame_id]["chunks"][index] = payload

    info = frames[frame_id]

    # 모든 조각 수신
    if len(info["chunks"]) != info["total"]:
        continue

    try:

        jpeg = b''.join(
            info["chunks"][i]
            for i in range(info["total"])
        )

    except KeyError:

        frames.clear()
        continue

    frames.clear()

    np_arr = np.frombuffer(
        jpeg,
        dtype=np.uint8
    )

    frame = cv2.imdecode(
        np_arr,
        cv2.IMREAD_COLOR
    )

    if frame is None:
        continue

    # =========================
    # FPS
    # =========================

    fps_count += 1

    now = time.perf_counter()

    if now - fps_start >= 1.0:

        fps = fps_count / (
            now - fps_start
        )

        print(
            f"FPS: {fps:.1f} | "
            f"{frame.shape[1]}x{frame.shape[0]}"
        )

        fps_count = 0
        fps_start = now

    # =========================
    # Display
    # =========================

    cv2.imshow(
        "Booster K1 Camera",
        frame
    )

    key = cv2.waitKey(1) & 0xff

    if key == ord('q') or key == 27:
        break


sock.close()
cv2.destroyAllWindows()