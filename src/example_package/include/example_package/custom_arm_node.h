#pragma once

#include "custom_arm_trajectory.h"
#include "booster_interface/msg/low_cmd.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/srv/rpc_service.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <csignal>
#include <functional>
#include <string>

// 노드의 함수와 상태 선언. 토픽/동작 구현은 node.cpp, 모드 전환 구현은 rpc.cpp에 둔다.
class CustomArmExample : public rclcpp::Node
{
public:
  CustomArmExample(const volatile std::sig_atomic_t & stop_requested);
  // 연결 확인부터 준비 자세, 팔 궤적, Walking 복귀까지 수행한다.
  int run();

private:
  // 시스템 시각 변경의 영향을 받지 않는 시계로 주기와 타임아웃을 계산한다.
  using Clock = std::chrono::steady_clock;
  using Rpc = booster_interface::srv::RpcService;

  // recovery=true이면 사용자 정지 플래그를 무시해 모드 복구를 계속한다.
  // ROS 컨텍스트 자체가 종료되면 복구 중에도 중단한다.
  void check_running(bool recovery);
  void pump(bool recovery);
  void wait_for(const std::function<bool()> & ready, Clock::duration timeout,
    bool recovery, const char * error);
  void wait_deadline(Clock::time_point deadline, bool recovery);
  void pause(Clock::duration duration, bool recovery);
  // 서비스 응답 대기 중에도 executor를 실행해 응답과 관절 상태를 처리한다.
  std::string call(int api_id, const std::string & body, bool recovery);
  int get_mode(bool recovery);
  void change_mode(int target, bool recovery);
  void check_state();
  void publish(const custom_arm::Angles & q);

  // main의 신호 플래그를 참조한다. 참조 대상은 노드보다 오래 유지되어야 한다.
  const volatile std::sig_atomic_t & stop_requested_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<booster_interface::msg::LowCmd>::SharedPtr publisher_;
  rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr subscription_;
  rclcpp::Client<Rpc>::SharedPtr rpc_;
  // command_: 발행할 목표 명령 / angles_: 가장 최근 수신한 실제 관절각.
  booster_interface::msg::LowCmd command_;
  custom_arm::Angles angles_{};
  // 수신 시각은 통신 중단 판단에 사용하며 관절 오차 보정에는 사용하지 않는다.
  Clock::time_point last_state_{};
  bool received_state_ = false;
  // 전환 응답이 실패해도 실제 모드가 바뀌었을 수 있어 요청 여부를 기록한다.
  bool custom_requested_ = false;
  std::size_t sent_ = 0;
};
