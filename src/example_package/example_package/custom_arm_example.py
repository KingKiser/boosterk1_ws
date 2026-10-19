#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import signal
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from booster_interface.msg import LowState, LowCmd, MotorCmd
from booster_robotics_sdk_python import B1LocoClient, ChannelFactory, RobotMode

import change as G

N = 22

class AmrSamplePub(Node):

    def __init__(self):
        super().__init__('amr_sample_pub')
        pub_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE )

        sub_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(LowCmd, "joint_ctrl", pub_qos)
        self.recent = {}
        self.sub = self.create_subscription(LowState, "low_state", self.recieve, sub_qos)

        self.sub
        self.cmd = LowCmd()
        self.cmd.cmd_type = LowCmd.CMD_TYPE_SERIAL
        self.cmd.motor_cmd = [MotorCmd() for _ in range(N)]
    def recieve(self, msg):
        self.recent["q"] = np.array([m.q for m in msg.motor_state_serial], np.float32)

    def send(self, q):

        for i in range(N):
            m = self.cmd.motor_cmd[i]
            m.q = float(q[i])
            m.dq = 0.0
            m.tau = 0.0
            m.kp = float(G.KP[i])
            m.kd = float(G.KD[i])
            m.weight = 0.0
        self.pub.publish(self.cmd)


def main():
    # ── SDK(모드 바꾸기) 와 ROS 2(관절 상태·명령) 준비 ────────────────────
    ChannelFactory.Instance().Init(0, "wlp3s0")
    client = B1LocoClient()
    client.Init()

    rclpy.init()
    amr_sample_pub = AmrSamplePub()
    #threading.Thread(target=rclpy.spin, args=(amr_sample_pub,), daemon=True).start()
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(amr_sample_pub,),
        daemon=False
    )
    spin_thread.start()

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())        # Ctrl+C: 사람이 내린 정지 — 되돌리지 않는다


    t0 = time.time()
    while "q" not in amr_sample_pub.recent:
        if time.time() - t0 > 5:
            raise RuntimeError("/low_state 를 5초 동안 못 받았다 — 기체가 Prepare/Walking 모드인지 확인")
        time.sleep(0.01)
    while amr_sample_pub.pub.get_subscription_count() == 0:
        if stop.is_set():
            return
        time.sleep(0.5)

    amr_sample_pub.start_angle = amr_sample_pub.recent["q"].copy()

    amr_sample_pub.get_logger().info(
        f"지금 자세(도) 어깨롤 "
        f"{np.round(np.degrees(amr_sample_pub.start_angle[[3, 7]]), 1)}"
    )
    custom_mode_entered = False

    try:
        # ② 지금 각도 그대로 한 번 보내 두고 커스텀 모드로
        amr_sample_pub.send(amr_sample_pub.start_angle)
        time.sleep(0.1)
        client.ChangeMode(RobotMode.kCustom)
        amr_sample_pub.get_logger().info("커스텀 모드 — 제조사 균형 제어가 꺼졌다")
        custom_mode_entered = True

        # ③ 1초 동안 준비 자세로 (2ms × 500)
        시작 = time.monotonic()
        for i, q in enumerate(np.linspace(amr_sample_pub.start_angle, G.준비각, num=500)):
            if stop.is_set():
                return
            amr_sample_pub.send(q)
            while time.monotonic() < 시작 + (i + 1) * 0.002:
                time.sleep(0.0002)

        # ④ 팔 동작 50Hz
        _, 궤적 = G.궤적()
        시작 = time.monotonic()
        for k, q in enumerate(궤적):
            if stop.is_set():
                amr_sample_pub.get_logger().info("Ctrl+C — 멈춘다")
                return
            amr_sample_pub.send(q)
            while time.monotonic() < 시작 + (k + 1) * 0.02:
                time.sleep(0.0005)
        amr_sample_pub.get_logger().info("팔 동작 끝")

    finally:
        if custom_mode_entered:
            try:
                client.ChangeMode(RobotMode.kWalking)
                amr_sample_pub.get_logger().info("Walking 모드 복귀 요청 성공")
            except Exception as e:
                amr_sample_pub.get_logger().error(
                    f"Walking 모드 복귀 실패: {e}"
                )

        amr_sample_pub.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()