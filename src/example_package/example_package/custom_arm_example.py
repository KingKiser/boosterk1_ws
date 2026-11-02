import signal #신호 
import threading #멀티 스레드의 의의 : 통신 모드 바꾸는게 아예 스레드 분기
import time #

import numpy as np #
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from booster_interface.msg import LowState, LowCmd, MotorCmd #우리 패키지 인터페이스
from booster_robotics_sdk_python import (
    B1LocoClient,
    ChannelFactory,
    RobotMode,
)

from example_package import change as G #조인트 각도 어떻게 설정 

N = 22
DDS_WAIT = 3.0
RPC_RETRY = 5
RPC_INTERVAL = 1.0


class AmrSamplePub(Node):

    def __init__(self):
        super().__init__("amr_sample_pub")

        pub_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE
        )
        sub_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.pub = self.create_publisher(
            LowCmd, "joint_ctrl", pub_qos #토픽 날려서 팔 각도 조절 0번 부터 21 까지 다보냄 
        )
        self.sub = self.create_subscription(
            LowState, "low_state", self.receive, sub_qos #피드백 루프 서브스크라이버 
        )

        self.recent = {} #피드백 루프 저장
        self.cmd = LowCmd()
        self.cmd.cmd_type = LowCmd.CMD_TYPE_SERIAL #값 1
        self.cmd.motor_cmd = [MotorCmd() for _ in range(N)] #조인트 배열

    def receive(self, msg):
        q = np.array(
            [m.q for m in msg.motor_state_serial],
            dtype=np.float32
        )
        if len(q) >= N:
            self.recent["q"] = q[:N]

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


def get_mode_retry(client, stop, Node, attempts=RPC_RETRY):
    for i in range(attempts):
        if stop.is_set():
            raise RuntimeError("사용자 정지")

        try:
            mode = client.GetMode().mode
            Node.get_logger().info(f"[RPC] GetMode 성공: {mode}")
            return mode
        except RuntimeError as e:
            Node.get_logger().error(
                f"[RPC] GetMode 실패 {i+1}/{attempts}: {e}"
            )
            if i + 1 < attempts:
                time.sleep(RPC_INTERVAL)

    raise RuntimeError("GetMode RPC 통신 확인 실패")


def change_mode_safe(client, target, stop, Node):
    for i in range(5):
        if stop.is_set():
            raise RuntimeError("사용자 정지")

        # 먼저 현재 모드를 확인한다.
        current = get_mode_retry(client, stop, Node)

        if current == target:
            Node.get_logger().info(f"[MODE] 이미 {target} 상태")
            return

        Node.get_logger().info(
            f"[MODE] 전환 요청 {i+1}/{5}: "
            f"{current} -> {target}",
        )

        try:
            client.ChangeMode(target)
        except RuntimeError as e:
            Node.get_logger().info(f"[MODE] ChangeMode 오류: {e}")

        # 타임아웃이 발생해도 전환됐을 수 있으므로 확인한다.
        time.sleep(5)

        try:
            current = get_mode_retry(client, stop, Node)
            if current == target:
                Node.get_logger().info(f"[MODE] 전환 확인 성공: {target}")
                return
        except RuntimeError as e:
            Node.get_logger().error(f"[MODE] 상태 확인 실패: {e}")

        if i + 1 < 5:
            time.sleep(5)

    raise RuntimeError(
        f"모드 전환 실패: 목표 모드 {target} 확인 불가"
    )


def wait_until(stop, deadline):
    while time.monotonic() < deadline:
        if stop.is_set():
            return False
        time.sleep(0.0005)
    return not stop.is_set()


def main():
    stop = threading.Event()

    signal.signal(
        signal.SIGINT,
        lambda *_: stop.set()
    )
    rclpy.init()
    node = AmrSamplePub()
    
    spin_thread = None
    client = None
    custom_mode_entered = False

    try:
        # 1. Booster SDK 초기화
        ChannelFactory.Instance().Init(0, "wlp3s0")
        client = B1LocoClient()
        client.Init()

        node.get_logger().info("[INIT] DDS Discovery 대기")
        time.sleep(DDS_WAIT)

        # 2. RPC 연결 확인

        change_mode_safe(
            client,
            RobotMode.kWalking,
            stop,
            node
        )

        current = get_mode_retry(client, stop, node)
        node.get_logger().info(f"[INIT] 현재 모드: {current}")


        if current != RobotMode.kCustom:
            raise RuntimeError(
                f"시작 모드가 Walking이 아님: {current}"
            )

        spin_thread = threading.Thread( #나중에 ros 스레드로 바꿀것
            target=rclpy.spin,
            args=(node,),
            daemon=False
        )
        spin_thread.start()

        # 4. LowState 수신 대기
        deadline = time.monotonic() + 5.0

        while "q" not in node.recent:
            if stop.is_set():
                return
            if time.monotonic() > deadline:
                raise RuntimeError("/low_state 수신 실패")
            time.sleep(0.01)

        # 5. joint_ctrl 구독자 확인
        deadline = time.monotonic() + 5.0

        while node.pub.get_subscription_count() == 0:
            if stop.is_set():
                return
            if time.monotonic() > deadline:
                raise RuntimeError("/joint_ctrl 구독자 없음")
            time.sleep(0.1)

        start_angle = node.recent["q"].copy()

        node.get_logger().info(
            "지금 자세(도) 어깨롤 "
            f"{np.round(np.degrees(start_angle[[3, 7]]), 1)}"
        )

        change_mode_safe(
                    client,
                    RobotMode.kCustom,
                    stop,
                    node
                )

        # 6. 현재 자세 명령을 미리 발행
        node.send(start_angle)
        time.sleep(0.1)

        custom_mode_entered = True

        # 8. 준비 자세 이동 (2ms x 500)
        start = time.monotonic()

        for i, q in enumerate(
            np.linspace(start_angle, G.pre_angle, num=500)
        ):
            if stop.is_set():
                return

            node.send(q)

            if not wait_until(
                stop,
                start + (i + 1) * 0.002
            ):
                return

        # 9. 팔 궤적 실행 (50Hz)
        _, trajectory = G.moving_way()
        start = time.monotonic()

        for k, q in enumerate(trajectory):
            if stop.is_set():
                node.get_logger().info("사용자 정지")
                return

            node.send(q)

            if not wait_until(
                stop,
                start + (k + 1) * 0.02
            ):
                return

        node.get_logger().info("팔 동작 완료")

    finally:
        # Custom 전환 요청이 타임아웃됐더라도
        # 실제 모드가 바뀌었을 가능성을 확인한다.
        if client is not None:
            try:
                recovery_stop = threading.Event()
                mode = get_mode_retry(
                    client, recovery_stop, node, attempts=3
                )

                if mode == RobotMode.kCustom:
                    custom_mode_entered = True

                if custom_mode_entered:
                    node.get_logger().info("[CLEANUP] Walking 복귀 시도")
                    change_mode_safe(
                        client,
                        RobotMode.kWalking,
                        recovery_stop,
                        node
                    )
                    node.get_logger().info("[CLEANUP] Walking 복귀 확인")

            except Exception as e:
                node.get_logger().fatal(
                    f"[CRITICAL] 모드 복귀 확인 실패: {e}",
                )
                node.get_logger().warn(
                    "[CRITICAL] 로봇 상태를 직접 확인해야 함",
                )

        # ROS 2 종료
        if rclpy.ok():
            rclpy.shutdown()

        if spin_thread is not None:
            spin_thread.join(timeout=3.0)

        if node is not None:
            node.destroy_node()


if __name__ == "__main__":
    main()
