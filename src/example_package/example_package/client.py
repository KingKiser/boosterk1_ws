#!/usr/bin/env python3

import json
import time

import rclpy
from rclpy.node import Node

from booster_interface.srv import RpcService
from booster_interface.msg import BoosterApiReqMsg


def create_msg(api_id, param_dict=None):
    msg = BoosterApiReqMsg()
    msg.api_id = api_id

    if param_dict is not None:
        msg.body = json.dumps(param_dict)
    else:
        msg.body = ""

    return msg


def send_rpc(node, client, msg, name="RPC"):
    """
    Booster RPC 요청을 하나 전송하고 응답까지 기다린다.
    """

    request = RpcService.Request()
    request.msg = msg

    node.get_logger().info(
        f"[SEND] {name}"
    )
    node.get_logger().info(
        f"       api_id={msg.api_id}"
    )
    node.get_logger().info(
        f"       body={msg.body}"
    )

    future = client.call_async(request)

    rclpy.spin_until_future_complete(
        node,
        future
    )

    if future.result() is None:
        node.get_logger().error(
            f"[FAILED] {name}"
        )
        return False

    node.get_logger().info(
        f"[SUCCESS] {name}"
    )

    # 서버 응답 body는 비어 있을 수 있음
    node.get_logger().info(
        f"          response={repr(future.result().msg.body)}"
    )

    return True


def main():

    rclpy.init()

    node = Node('arm_control_node')

    client = node.create_client(
        RpcService,
        'booster_rpc_service'
    )

    # ========================================================
    # Booster RPC service 연결 대기
    # ========================================================
    node.get_logger().info(
        "Waiting for booster_rpc_service..."
    )

    while not client.wait_for_service(timeout_sec=1.0):

        node.get_logger().info(
            "service not available, waiting again..."
        )

        if not rclpy.ok():
            node.get_logger().error(
                "Interrupted while waiting for the service."
            )
            return

    node.get_logger().info(
        "booster_rpc_service connected"
    )


    # ========================================================
    # 1. Hand End Effector Control Mode ON
    #
    # API ID 2012
    # kSwitchHandEndEffectorControlMode
    # ========================================================
    req_hand_mode_on = create_msg(
        2012,
        {
            "switch_on": True
        }
    )


    # ========================================================
    # 2. Hand End Effector Control Mode OFF
    # ========================================================
    req_hand_mode_off = create_msg(
        2012,
        {
            "switch_on": False
        }
    )


    # ========================================================
    # 3. kMove
    #
    # 제조사 샘플에서는:
    #
    # vx = 0.5
    #
    # 를 사용했지만 여기서는 실제로 걷지 않게
    # vx = vy = vyaw = 0 으로 설정
    #
    # 목적:
    # Booster 내부 locomotion / controller state
    # 활성화 여부 확인
    # ========================================================
    req_move_enable = create_msg(
        2001,
        {
            "vx": 0.0,
            "vy": 0.0,
            "vyaw": 0.0
        }
    )


    # ========================================================
    # 4. 왼손 목표 자세
    #
    # 제조사 예제 값 그대로 사용
    #
    # hand_index
    # 0 = Left Hand
    # ========================================================
    target_posture = {

        "position_": {
            "x": 0.35,
            "y": 0.25,
            "z": 0.10
        },

        "orientation_": {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0
        }
    }


    # ========================================================
    # 5. Move Hand End Effector
    #
    # API ID 2009
    # ========================================================
    req_move_hand = create_msg(
        2009,
        {
            "target_posture": target_posture,
            "time_millis": 2000,
            "hand_index": 0
        }
    )


    # ========================================================
    # 메인 동작
    # ========================================================
    try:

        while rclpy.ok():

            node.get_logger().info(
                "========================================"
            )

            node.get_logger().info(
                "Start arm control sequence"
            )


            # ------------------------------------------------
            # STEP 1
            # Hand End Effector Control Mode ON
            # ------------------------------------------------
            if not send_rpc(
                node,
                client,
                req_hand_mode_on,
                "Hand Control Mode ON"
            ):
                break


            time.sleep(2.0)


            # ------------------------------------------------
            # STEP 2
            # kMove(0,0,0)
            #
            # 제조사 샘플의 2001 호출 순서를 유지
            # ------------------------------------------------
            if not send_rpc(
                node,
                client,
                req_move_enable,
                "kMove activation"
            ):
                break


            time.sleep(2.0)


            # ------------------------------------------------
            # STEP 3
            # Hand End Effector 이동
            # ------------------------------------------------
            if not send_rpc(
                node,
                client,
                req_move_hand,
                "Move Left Hand End Effector"
            ):
                break


            # 실제 명령 시간이 2000 ms이므로
            # 조금 여유 있게 대기
            time.sleep(4.0)


            # ------------------------------------------------
            # STEP 4
            # Hand control mode OFF
            # ------------------------------------------------
            if not send_rpc(
                node,
                client,
                req_hand_mode_off,
                "Hand Control Mode OFF"
            ):
                break


            node.get_logger().info(
                "Arm control sequence finished"
            )


            # 일단 반복 실행하지 않고
            # 한 번만 테스트
            break


    except KeyboardInterrupt:

        node.get_logger().info(
            "Keyboard interrupt"
        )


    finally:

        # ----------------------------------------------------
        # 프로그램 종료 전에 가능하면 hand control mode OFF
        # ----------------------------------------------------
        if rclpy.ok():

            try:
                send_rpc(
                    node,
                    client,
                    req_hand_mode_off,
                    "Safety Hand Control Mode OFF"
                )
            except Exception:
                pass


        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
