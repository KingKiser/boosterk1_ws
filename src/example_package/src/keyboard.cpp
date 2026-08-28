#include "booster_interface/srv/rpc_service.hpp"
#include "booster_interface/message_utils.hpp"
#include "booster_interface/msg/booster_api_req_msg.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <thread>
#include <iostream>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std::chrono_literals;


// ================================
// 키보드 입력
// ================================
char getKey()
{
    char ch = 0;

    struct termios oldt;
    struct termios newt;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // canonical mode OFF
    newt.c_lflag &= ~(ICANON | ECHO);

    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // non-blocking
    int oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    read(STDIN_FILENO, &ch, 1);

    // 복구
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    return ch;
}


// ================================
// RPC Move 명령 전송
// ================================
bool sendMove(
    std::shared_ptr<rclcpp::Node> node,
    rclcpp::Client<booster_interface::srv::RpcService>::SharedPtr client,
    double vx,
    double vy,
    double wz)
{
    auto request =
        std::make_shared<booster_interface::srv::RpcService::Request>();

    auto move_msg =
        booster_interface::CreateMsg<
            booster::robot::b1::LocoApiId::kMove,
            booster::robot::b1::MoveParameter>(
                vx,
                vy,
                wz
            );

    request->msg = move_msg;

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(
            node,
            result,
            500ms)
        == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(
            node->get_logger(),
            "Move command -> vx: %.2f  vy: %.2f  wz: %.2f",
            vx,
            vy,
            wz
        );

        return true;
    }

    RCLCPP_ERROR(
        node->get_logger(),
        "Failed to send move command"
    );

    return false;
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        rclcpp::Node::make_shared("keyboard_move_node");

    auto client =
        node->create_client<
            booster_interface::srv::RpcService>(
                "booster_rpc_service"
            );


    // ================================
    // 서비스 대기
    // ================================
    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "Interrupted while waiting for service."
            );

            return 0;
        }

        RCLCPP_INFO(
            node->get_logger(),
            "Waiting for booster_rpc_service..."
        );
    }


    // ================================
    // 속도 설정
    // ================================

    const double LINEAR_SPEED  = 0.3;
    const double LATERAL_SPEED = 0.2;
    const double ANGULAR_SPEED = 0.5;


    std::cout << "\n";
    std::cout << "=============================\n";
    std::cout << " Booster Keyboard Controller\n";
    std::cout << "=============================\n";
    std::cout << "\n";

    std::cout << "        W\n";
    std::cout << "        ↑\n";
    std::cout << "   A ←  X  → D\n";
    std::cout << "        ↓\n";
    std::cout << "        S\n";
    std::cout << "\n";

    std::cout << "Q : 왼쪽 회전\n";
    std::cout << "E : 오른쪽 회전\n";
    std::cout << "X : 정지\n";
    std::cout << "ESC : 종료\n";

    std::cout << "\n";


    // 처음에는 정지
    sendMove(node, client, 0.0, 0.0, 0.0);


    while (rclcpp::ok())
    {
        char key = getKey();

        if (key == 0)
        {
            std::this_thread::sleep_for(20ms);
            continue;
        }


        switch (key)
        {

        // ================================
        // 전진
        // ================================
        case 'w':
        case 'W':

            sendMove(
                node,
                client,
                LINEAR_SPEED,
                0.0,
                0.0
            );

            break;


        // ================================
        // 후진
        // ================================
        case 's':
        case 'S':

            sendMove(
                node,
                client,
                -LINEAR_SPEED,
                0.0,
                0.0
            );

            break;


        // ================================
        // 왼쪽 횡이동
        // ================================
        case 'a':
        case 'A':

            sendMove(
                node,
                client,
                0.0,
                LATERAL_SPEED,
                0.0
            );

            break;


        // ================================
        // 오른쪽 횡이동
        // ================================
        case 'd':
        case 'D':

            sendMove(
                node,
                client,
                0.0,
                -LATERAL_SPEED,
                0.0
            );

            break;


        // ================================
        // 왼쪽 회전
        // ================================
        case 'q':
        case 'Q':

            sendMove(
                node,
                client,
                0.0,
                0.0,
                ANGULAR_SPEED
            );

            break;


        // ================================
        // 오른쪽 회전
        // ================================
        case 'e':
        case 'E':

            sendMove(
                node,
                client,
                0.0,
                0.0,
                -ANGULAR_SPEED
            );

            break;


        // ================================
        // 정지
        // ================================
        case 'x':
        case 'X':

            sendMove(
                node,
                client,
                0.0,
                0.0,
                0.0
            );

            break;


        // ================================
        // ESC
        // ================================
        case 27:

            std::cout << "\nExit...\n";

            sendMove(
                node,
                client,
                0.0,
                0.0,
                0.0
            );

            rclcpp::shutdown();

            return 0;
        }

        std::this_thread::sleep_for(20ms);
    }


    // 종료 전 안전하게 정지
    sendMove(node, client, 0.0, 0.0, 0.0);

    rclcpp::shutdown();

    return 0;
}