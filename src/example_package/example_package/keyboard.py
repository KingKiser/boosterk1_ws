#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import sys
import time
import select
import termios
import tty

import rclpy
from rclpy.node import Node

from booster_interface.srv import RpcService
from booster_interface.msg import BoosterApiReqMsg


class KeyboardMoveNode(Node):

    def __init__(self):
        super().__init__('keyboard_move_node')

        self.client = self.create_client(
            RpcService,
            '/booster_rpc_service'
        )

        self.linear_speed = 0.3
        self.lateral_speed = 0.2
        self.angular_speed = 0.5

        self.running = True

        self.get_logger().info("Booster Keyboard Controller initialized")

    # ========================================================
    # RPC 서비스 연결 대기
    # ========================================================
    def wait_for_rpc(self):

        while rclpy.ok():

            if self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Booster RPC connected")
                return True

            self.get_logger().warn(
                "Waiting for booster_rpc_service..."
            )

        return False

    # ========================================================
    # RPC 메시지 생성
    # ========================================================
    def create_msg(self, api_id, param_dict=None):

        msg = BoosterApiReqMsg()
        msg.api_id = api_id

        if param_dict is not None:
            msg.body = json.dumps(param_dict)
        else:
            msg.body = ""

        return msg

    # ========================================================
    # RPC 전송
    # ========================================================
    def send_rpc(self, msg):

        request = RpcService.Request()
        request.msg = msg

        future = self.client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=1.0
        )

        if not future.done():
            self.get_logger().error("RPC timeout")
            return False

        try:
            response = future.result()

            if response is None:
                self.get_logger().error("RPC response is None")
                return False

            if response.msg.status != 0:
                self.get_logger().error(
                    f"RPC rejected: status={response.msg.status}"
                )
                return False

            return True

        except Exception as e:
            self.get_logger().error(f"RPC error: {e}")
            return False

    # ========================================================
    # 이동 명령
    # API ID 2001 : kMove
    # ========================================================
    def send_move(self, vx, vy, vyaw):

        msg = self.create_msg(
            2001,
            {
                "vx": vx,
                "vy": vy,
                "vyaw": vyaw
            }
        )

        success = self.send_rpc(msg)

        if success:
            self.get_logger().info(
                f"Move -> vx={vx:.2f}, "
                f"vy={vy:.2f}, "
                f"vyaw={vyaw:.2f}"
            )

        return success

    # ========================================================
    # 키보드 입력
    # ========================================================
    def get_key(self, timeout=0.02):

        if select.select([sys.stdin], [], [], timeout)[0]:
            return sys.stdin.read(1)

        return None

    # ========================================================
    # 사용 안내
    # ========================================================
    def print_guide(self):

        print("""
================================
 Booster K1 Keyboard Controller
================================

        W
        ↑
   A ←  S  → D
        ↓
        X

W : 전진
X : 후진
A : 왼쪽 횡이동
D : 오른쪽 횡이동
Q : 왼쪽 회전
E : 오른쪽 회전
S : 정지
ESC : 종료

================================
""")

    # ========================================================
    # 키보드 제어
    # ========================================================
    def run(self):

        self.print_guide()

        if not sys.stdin.isatty():
            raise RuntimeError("키보드 입력을 위한 터미널이 필요합니다.")

        old_settings = termios.tcgetattr(sys.stdin)

        try:
            tty.setcbreak(sys.stdin.fileno())

            while rclpy.ok() and self.running:

                key = self.get_key()

                if key is None:
                    continue

                key = key.lower()

                vx = 0.0
                vy = 0.0
                vyaw = 0.0

                if key == 'w':
                    vx = self.linear_speed

                elif key == 'x':
                    vx = -self.linear_speed

                elif key == 'a':
                    vy = self.lateral_speed

                elif key == 'd':
                    vy = -self.lateral_speed

                elif key == 'q':
                    vyaw = self.angular_speed

                elif key == 'e':
                    vyaw = -self.angular_speed

                elif key == 's':
                    pass

                elif key == '\x1b':
                    self.get_logger().info("ESC pressed")
                    self.running = False
                    break

                else:
                    continue

                if not self.send_move(vx, vy, vyaw):
                    self.get_logger().warn(
                        "Move command failed. Requesting stop."
                    )
                    self.send_move(0.0, 0.0, 0.0)

        finally:
            # 종료 시 정지 명령
            if rclpy.ok():
                self.send_move(0.0, 0.0, 0.0)

            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                old_settings
            )


def main(args=None):

    rclpy.init(args=args)

    node = KeyboardMoveNode()

    try:
        if node.wait_for_rpc():
            node.run()

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt")

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
