#include "booster_interface/srv/rpc_service.hpp"
#include "booster_interface/message_utils.hpp"
#include "booster_interface/msg/booster_api_req_msg.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("hand_end_effector_client");

    auto client =
        node->create_client<booster_interface::srv::RpcService>(
            "booster_rpc_service");

    // -------------------------------------------------
    // 1. Hand End Effector Control Mode ON
    // -------------------------------------------------
    auto req_mode_on = booster_interface::CreateMsg<
        booster::robot::b1::LocoApiId::kSwitchHandEndEffectorControlMode,
        booster::robot::b1::SwitchHandEndEffectorControlModeParameter>(
        true);

    // -------------------------------------------------
    // 2. 목표 End Effector 자세
    // position : x, y, z
    // orientation : roll, pitch, yaw 추정
    // -------------------------------------------------
    booster::robot::Posture target_posture;

    target_posture.position_ =
        booster::robot::Position(
            0.35,
            0.25,
            0.10);

    target_posture.orientation_ =
        booster::robot::Orientation(
            0.0,
            0.0,
            0.0);

    // -------------------------------------------------
    // 3. 왼손 End Effector 이동 명령
    // 2000 : 동작 시간(ms)
    // -------------------------------------------------
    auto req_move_hand = booster_interface::CreateMsg<
        booster::robot::b1::LocoApiId::kMoveHandEndEffector,
        booster::robot::b1::MoveHandEndEffectorParameter>(
        target_posture,
        2000,
        booster::robot::b1::HandIndex::kLeftHand);

    // -------------------------------------------------
    // Service 대기
    // -------------------------------------------------
    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "Interrupted while waiting for service.");

            rclcpp::shutdown();
            return 0;
        }

        RCLCPP_INFO(
            node->get_logger(),
            "Waiting for booster_rpc_service...");
    }

    auto request =
        std::make_shared<
            booster_interface::srv::RpcService::Request>();

    // =================================================
    // Hand End Effector Control Mode ON
    // =================================================
    request->msg = req_mode_on;

    auto result =
        client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(
            node->get_logger(),
            "Hand control mode response: %s",
            result.get()->msg.body.c_str());
    }
    else
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Failed to enable hand end effector control mode.");

        rclcpp::shutdown();
        return 1;
    }

    // =================================================
    // Hand End Effector 이동
    // =================================================
    request->msg = req_move_hand;

    result =
        client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(
            node->get_logger(),
            "Move hand response: %s",
            result.get()->msg.body.c_str());
    }
    else
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Failed to move hand end effector.");
    }

    rclcpp::shutdown();

    return 0;
}