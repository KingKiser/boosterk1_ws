#include "booster_interface/srv/rpc_service.hpp"
#include "booster_interface/message_utils.hpp"
#include "booster_interface/msg/booster_api_req_msg.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <thread>

using namespace std::chrono_literals;


// ============================================================
// RPC 요청 함수
// ============================================================
bool send_rpc(
    const std::shared_ptr<rclcpp::Node>& node,
    const rclcpp::Client<booster_interface::srv::RpcService>::SharedPtr& client,
    const booster_interface::msg::BoosterApiReqMsg& msg,
    const std::string& name)
{
    auto request =
        std::make_shared<booster_interface::srv::RpcService::Request>();

    request->msg = msg;

    RCLCPP_INFO(
        node->get_logger(),
        "[SEND] %s",
        name.c_str()
    );

    auto future = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(
            node->get_logger(),
            "[SUCCESS] %s",
            name.c_str()
        );

        return true;
    }

    RCLCPP_ERROR(
        node->get_logger(),
        "[FAILED] %s",
        name.c_str()
    );

    return false;
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        rclcpp::Node::make_shared("dual_arm_swing_node");

    auto client =
        node->create_client<booster_interface::srv::RpcService>(
            "booster_rpc_service"
        );


    // ========================================================
    // Hand End Effector Control Mode ON
    // ========================================================
    auto req_hand_mode_on =
        booster_interface::CreateMsg<
            booster::robot::b1::LocoApiId::
                kSwitchHandEndEffectorControlMode,
            booster::robot::b1::
                SwitchHandEndEffectorControlModeParameter
        >(true);


    // ========================================================
    // Hand End Effector Control Mode OFF
    // ========================================================
    auto req_hand_mode_off =
        booster_interface::CreateMsg<
            booster::robot::b1::LocoApiId::
                kSwitchHandEndEffectorControlMode,
            booster::robot::b1::
                SwitchHandEndEffectorControlModeParameter
        >(false);


    // ========================================================
    // 왼팔 위치 A
    //
    // 몸쪽
    // ========================================================
    booster::robot::Posture left_a;

    left_a.position_ =
        booster::robot::Position(
            0.35,
            0.20,
            0.10
        );

    left_a.orientation_ =
        booster::robot::Orientation(
            0.0,
            0.0,
            0.0
        );


    // ========================================================
    // 왼팔 위치 B
    //
    // 바깥쪽
    // ========================================================
    booster::robot::Posture left_b;

    left_b.position_ =
        booster::robot::Position(
            0.35,
            0.30,
            0.10
        );

    left_b.orientation_ =
        booster::robot::Orientation(
            0.0,
            0.0,
            0.0
        );


    // ========================================================
    // 오른팔 위치 A
    //
    // 몸쪽
    // ========================================================
    booster::robot::Posture right_a;

    right_a.position_ =
        booster::robot::Position(
            0.35,
            -0.20,
            0.10
        );

    right_a.orientation_ =
        booster::robot::Orientation(
            0.0,
            0.0,
            0.0
        );


    // ========================================================
    // 오른팔 위치 B
    //
    // 바깥쪽
    // ========================================================
    booster::robot::Posture right_b;

    right_b.position_ =
        booster::robot::Position(
            0.35,
            -0.30,
            0.10
        );

    right_b.orientation_ =
        booster::robot::Orientation(
            0.0,
            0.0,
            0.0
        );


    // ========================================================
    // 왼팔 A
    // ========================================================
    auto req_left_a =
        booster_interface::CreateMsg<
            booster::robot::b1::LocoApiId::kMoveHandEndEffector,
            booster::robot::b1::MoveHandEndEffectorParameter
        >(
            left_a,
            1000,
            booster::robot::b1::HandIndex::kLeftHand
        );


    // ========================================================
    // 왼팔 B
    // ========================================================
    auto req_left_b =
        booster_interface::CreateMsg<
            booster::robot::b1::LocoApiId::kMoveHandEndEffector,
            booster::robot::b1::MoveHandEndEffectorParameter
        >(
            left_b,
            1000,
            booster::robot::b1::HandIndex::kLeftHand
        );


    // ========================================================
    // 오른팔 A
    // ========================================================
    auto req_right_a =
        booster_interface::CreateMsg<
            booster::robot::b1::LocoApiId::kMoveHandEndEffector,
            booster::robot::b1::MoveHandEndEffectorParameter
        >(
            right_a,
            1000,
            booster::robot::b1::HandIndex::kRightHand
        );


    // ========================================================
    // 오른팔 B
    // ========================================================
    auto req_right_b =
        booster_interface::CreateMsg<
            booster::robot::b1::LocoApiId::kMoveHandEndEffector,
            booster::robot::b1::MoveHandEndEffectorParameter
        >(
            right_b,
            1000,
            booster::robot::b1::HandIndex::kRightHand
        );


    // ========================================================
    // Service 대기
    // ========================================================
    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "Interrupted while waiting for service."
            );

            rclcpp::shutdown();
            return 0;
        }

        RCLCPP_INFO(
            node->get_logger(),
            "Waiting for booster_rpc_service..."
        );
    }


    // ========================================================
    // Hand End Effector Mode ON
    // ========================================================
    if (!send_rpc(
            node,
            client,
            req_hand_mode_on,
            "Hand Control Mode ON"))
    {
        rclcpp::shutdown();
        return 1;
    }


    std::this_thread::sleep_for(2s);


    // ========================================================
    // 양팔 좌우 반복
    // ========================================================
    constexpr int SWING_COUNT = 5;


    for (int i = 0;
         i < SWING_COUNT && rclcpp::ok();
         ++i)
    {
        RCLCPP_INFO(
            node->get_logger(),
            "===== Swing %d / %d =====",
            i + 1,
            SWING_COUNT
        );


        // ----------------------------------------------------
        // 양팔 몸쪽
        // ----------------------------------------------------
        send_rpc(
            node,
            client,
            req_left_a,
            "Left Arm -> A"
        );

        send_rpc(
            node,
            client,
            req_right_a,
            "Right Arm -> A"
        );


        std::this_thread::sleep_for(1200ms);


        // ----------------------------------------------------
        // 양팔 바깥쪽
        // ----------------------------------------------------
        send_rpc(
            node,
            client,
            req_left_b,
            "Left Arm -> B"
        );

        send_rpc(
            node,
            client,
            req_right_b,
            "Right Arm -> B"
        );


        std::this_thread::sleep_for(1200ms);
    }


    // ========================================================
    // Hand End Effector Mode OFF
    // ========================================================
    send_rpc(
        node,
        client,
        req_hand_mode_off,
        "Hand Control Mode OFF"
    );


    RCLCPP_INFO(
        node->get_logger(),
        "Dual arm movement finished."
    );


    rclcpp::shutdown();

    return 0;
}