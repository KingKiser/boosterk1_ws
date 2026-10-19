#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class ArmJointStateSub(Node):

    def __init__(self):
        super().__init__('arm_joint_state_sub')

        sub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.receive,
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

        logs = ["\n========== ARM JOINT STATES =========="]

        for index in range(2, 10):

            name = self.joint_names[index - 2]

            position = msg.position[index]
            velocity = msg.velocity[index]
            effort = msg.effort[index]

            position_deg = math.degrees(position)

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
