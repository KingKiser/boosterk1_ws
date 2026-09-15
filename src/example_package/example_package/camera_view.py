#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np


class ImageViewer(Node):

    def __init__(self):
        super().__init__('booster_image_viewer')

        # =====================================================
        # 최신 프레임만 보관
        # =====================================================
        self.latest_data = None
        self.lock = threading.Lock()

        # =====================================================
        # 저지연용 QoS
        #
        # BEST_EFFORT:
        #   영상에서는 오래된 프레임 재전송보다
        #   최신 프레임을 받는 것이 중요
        #
        # KEEP_LAST + depth=1:
        #   메시지를 쌓아두지 않고 최신 1개만 유지
        # =====================================================
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.sub = self.create_subscription(
            CompressedImage,
            '/booster_video_stream',
            self.image_callback,
            qos
        )

        self.get_logger().info(
            'Low latency subscribe: /booster_video_stream'
        )

    # =========================================================
    # ROS Callback
    # =========================================================
    def image_callback(self, msg):

        # 여기서는 JPEG decode 하지 않음.
        # 최신 JPEG 데이터만 저장하고 즉시 callback 종료.
        #
        # bytes()로 복사해서 msg 객체 수명과 분리
        data = bytes(msg.data)

        with self.lock:
            self.latest_data = data

    # =========================================================
    # 최신 영상 가져오기
    # =========================================================
    def get_latest_frame(self):

        with self.lock:

            if self.latest_data is None:
                return None

            # 최신 데이터를 가져오고 비움
            # 같은 프레임을 반복 decode하지 않기 위함
            data = self.latest_data
            self.latest_data = None

        # JPEG byte array -> NumPy
        np_arr = np.frombuffer(
            data,
            dtype=np.uint8
        )

        # JPEG -> OpenCV BGR
        frame = cv2.imdecode(
            np_arr,
            cv2.IMREAD_COLOR
        )

        return frame


def main(args=None):

    rclpy.init(args=args)

    node = ImageViewer()

    # =========================================================
    # ROS spin은 별도 Thread
    # =========================================================
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True
    )

    spin_thread.start()

    try:

        while rclpy.ok():

            # 항상 가장 최근 프레임만 가져옴
            frame = node.get_latest_frame()

            if frame is not None:

                cv2.imshow(
                    'Booster Camera - Low Latency',
                    frame
                )

            # GUI event 처리
            key = cv2.waitKey(1) & 0xFF

            # q 또는 ESC 종료
            if key == ord('q') or key == 27:
                break

    except KeyboardInterrupt:
        pass

    finally:

        cv2.destroyAllWindows()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()