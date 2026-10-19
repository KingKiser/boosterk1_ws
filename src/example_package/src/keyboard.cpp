
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


// ============================================================
// Booster K1 Keyboard Controller
// ============================================================
class KeyboardMoveNode : public rclcpp::Node
{
public:

    KeyboardMoveNode()
        : Node("keyboard_move_node")
    {
        client_ = this->create_client<
            booster_interface::srv::RpcService>(
                "/booster_rpc_service"
            );

        RCLCPP_INFO(
            this->get_logger(),
            "KeyboardMoveNode initialized"
        );
    }


    // ========================================================
    // RPC 서비스 연결 대기
    // ========================================================
    bool waitForService()
    {
        while (rclcpp::ok())
        {
            if (client_->wait_for_service(1s))
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Booster RPC service connected"
                );

                return true;
            }

            RCLCPP_WARN(
                this->get_logger(),
                "Waiting for booster_rpc_service..."
            );
        }

        return false;
    }


    // ========================================================
    // 이동 명령 전송
    // ========================================================
    bool sendMove(
        double vx,
        double vy,
        double wz)
    {
        auto request =
            std::make_shared<
                booster_interface::srv::RpcService::Request>();

        auto move_msg =
            booster_interface::CreateMsg<
                booster::robot::b1::LocoApiId::kMove,
                booster::robot::b1::MoveParameter>(
                    vx,
                    vy,
                    wz
                );

        request->msg = move_msg;

        auto future = client_->async_send_request(request);

        auto result = rclcpp::spin_until_future_complete(
            this->get_node_base_interface(),
            future,
            500ms
        );

        if (result != rclcpp::FutureReturnCode::SUCCESS)
        {
            client_->remove_pending_request(future);

            RCLCPP_ERROR(
                this->get_logger(),
                "Move RPC failed or timed out"
            );

            return false;
        }

        // ROS 서비스 응답 확인
        auto response = future.get();

        if (response->msg.status != 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Move rejected, status: %ld",
                static_cast<long>(response->msg.status)
            );

            return false;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Move -> vx: %.2f, vy: %.2f, wz: %.2f",
            vx, vy, wz
        );

        return true;
    }


    // ========================================================
    // 키보드 입력
    // ========================================================
    char getKey()
    {
        char ch = 0;

        struct termios oldt;
        struct termios newt;

        if (tcgetattr(STDIN_FILENO, &oldt) != 0)
        {
            return 0;
        }

        newt = oldt;

        newt.c_lflag &= ~(ICANON | ECHO);

        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        int oldf = fcntl(STDIN_FILENO, F_GETFL, 0);

        if (oldf < 0)
        {
            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
            return 0;
        }

        fcntl(
            STDIN_FILENO,
            F_SETFL,
            oldf | O_NONBLOCK
        );

        read(STDIN_FILENO, &ch, 1);

        // 터미널 설정 복구
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

        fcntl(
            STDIN_FILENO,
            F_SETFL,
            oldf
        );

        return ch;
    }


    // ========================================================
    // 키보드 안내
    // ========================================================
    void printGuide()
    {
        std::cout << "\n";
        std::cout << "=============================\n";
        std::cout << " Booster Keyboard Controller\n";
        std::cout << "=============================\n";

        std::cout << "W : 전진\n";
        std::cout << "X : 후진\n";
        std::cout << "A : 왼쪽 횡이동\n";
        std::cout << "D : 오른쪽 횡이동\n";
        std::cout << "Q : 왼쪽 회전\n";
        std::cout << "E : 오른쪽 회전\n";
        std::cout << "S : 정지\n";
        std::cout << "ESC : 종료\n\n";
    }


    // ========================================================
    // 키보드 제어 루프
    // ========================================================
    void run()
    {
        printGuide();

        // 초기 정지
        if (!sendMove(0.0, 0.0, 0.0))
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Initial stop command failed"
            );
            return;
        }

        while (rclcpp::ok())
        {
            char key = getKey();

            if (key == 0)
            {
                std::this_thread::sleep_for(20ms);
                continue;
            }

            bool success = true;

            switch (key)
            {
                case 'w':
                case 'W':
                    success = sendMove(LINEAR_SPEED, 0.0, 0.0);
                    break;

                case 'x':
                case 'X':
                    success = sendMove(-LINEAR_SPEED, 0.0, 0.0);
                    break;

                case 'a':
                case 'A':
                    success = sendMove(0.0, LATERAL_SPEED, 0.0);
                    break;

                case 'd':
                case 'D':
                    success = sendMove(0.0, -LATERAL_SPEED, 0.0);
                    break;

                case 'q':
                case 'Q':
                    success = sendMove(0.0, 0.0, ANGULAR_SPEED);
                    break;

                case 'e':
                case 'E':
                    success = sendMove(0.0, 0.0, -ANGULAR_SPEED);
                    break;

                case 's':
                case 'S':
                    success = sendMove(0.0, 0.0, 0.0);
                    break;

                case 27:
                    RCLCPP_INFO(
                        this->get_logger(),
                        "ESC pressed. Stopping..."
                    );

                    sendMove(0.0, 0.0, 0.0);
                    return;

                default:
                    continue;
            }

            if (!success)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Command failed. Requesting stop."
                );

                sendMove(0.0, 0.0, 0.0);
            }

            std::this_thread::sleep_for(20ms);
        }

        // 종료 전 정지 요청
        if (client_->service_is_ready())
        {
            sendMove(0.0, 0.0, 0.0);
        }
    }


private:

    rclcpp::Client<
        booster_interface::srv::RpcService
    >::SharedPtr client_;

    static constexpr double LINEAR_SPEED  = 0.3;
    static constexpr double LATERAL_SPEED = 0.2;
    static constexpr double ANGULAR_SPEED = 0.5;
};


// ============================================================
// Main
// ============================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<KeyboardMoveNode>();

    if (node->waitForService())
    {
        node->run();
    }

    node.reset();

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }

    return 0;
}
