#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math #파이썬 기본 수학 연산 라이브러리

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy #주파수의 또다른 옵션
from sensor_msgs.msg import JointState #ros2의 기본 관절 측정 인터페이스


class ArmJointStateSub(Node): 

    def __init__(self):
        super().__init__('arm_joint_state_sub')

        sub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT #qos 정의
        )

        self.sub = self.create_subscription(
            JointState,#기본 관절 측정 인터페이스 사요
            '/joint_states', #수신할 토픽 이름
            self.receive, #토픽 받아서 어쩔건데
            sub_qos
        )

        # 관절 인덱스 (제공한 문서 기준)
        self.joint_names = [
            "Left Shoulder Pitch",   # 2
            "Left Shoulder Roll",    # 3
            "Left Elbow Pitch",      # 4
            "Left Elbow Yaw",        # 5
            "Right Shoulder Pitch",  # 6
            "Right Shoulder Roll",   # 7
            "Right Elbow Pitch",     # 8
            "Right Elbow Yaw"        # 9
        ]

        self.get_logger().info("Booster K1 양팔 JointState 수신 시작")

    def receive(self, msg):

        # position, velocity, effort 배열 확인
        if len(msg.position) < 10:
            self.get_logger().warn("Position 데이터 부족")
            return

        if len(msg.velocity) < 10:
            self.get_logger().warn("Velocity 데이터 부족")
            return

        if len(msg.effort) < 10:
            self.get_logger().warn("Effort 데이터 부족")
            return
        #joint_state 내부에서도 토픽이 다양하게 있다
        logs = ["\n========== ARM JOINT STATES =========="]
        #터미널에서 보기 좋게 분기하려고 넣음

        for index in range(2, 10):

            name = self.joint_names[index - 2]

            position = msg.position[index]
            velocity = msg.velocity[index]
            effort = msg.effort[index]

            position_deg = math.degrees(position) #관절 각도 joint_state/position[2]

            logs.append(
                f"[{index}] {name:<22} | "
                f"Pos: {position_deg:7.2f} deg | "
                f"Vel: {velocity:8.3f} rad/s | "
                f"Effort: {effort:8.3f} Nm"
            )

        logs.append("======================================")

        self.get_logger().info("\n".join(logs))


def main(args=None):

    rclpy.init(args=args)

    node = ArmJointStateSub()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
