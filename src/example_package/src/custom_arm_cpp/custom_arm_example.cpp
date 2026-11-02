// 프로그램 진입점: 신호 처리 → ROS 노드 실행 → 복구 후 ROS 종료.
#include "../include/example_package/custom_arm_node.h"

#include <csignal>
#include <exception>
#include <memory>

namespace
{
// 신호 처리기에서는 플래그만 변경하고, RPC 호출과 로그는 일반 실행 흐름에서 처리한다.
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) {stop_requested = 1;}
}  // namespace

int main(int argc, char ** argv)
{
  // 기본 SIGINT 처리는 ROS를 즉시 종료하므로 비활성화한다.
  // Ctrl+C 후에도 Walking 복귀 RPC가 끝날 때까지 ROS 통신을 유지한다.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  int result = 1;
  try {
    auto node = std::make_shared<CustomArmExample>(stop_requested);
    result = node->run();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("custom_arm_example_cpp"), "%s", e.what());
  }
  // run() 내부의 Walking 복귀 시도가 끝난 뒤 ROS 자원을 정리한다.
  rclcpp::shutdown();
  return result;
}
