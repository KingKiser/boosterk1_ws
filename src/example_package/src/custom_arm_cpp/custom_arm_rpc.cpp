#include "../include/example_package/custom_arm_node.h"
#include "booster_interface/third_party/nlohmann_json/json.hpp"

#include <future>
#include <stdexcept>

using namespace std::chrono_literals;

namespace
{
// Booster LocoApiId에 정의된 API 번호. DDS SDK 클라이언트 대신 ROS 서비스를 사용한다.
constexpr int kChangeMode = 2000;
constexpr int kGetMode = 2017;
}  // namespace

std::string CustomArmExample::call(int api_id, const std::string & body, bool recovery)
{
  check_running(recovery);
  auto request = std::make_shared<Rpc::Request>();
  request->msg.api_id = api_id;
  request->msg.body = body;
  // 비동기 요청을 보낸 뒤 future가 준비될 때까지 최대 3초 대기한다.
  // wait_for()가 executor를 실행하므로 단일 스레드에서도 응답을 받을 수 있다.
  auto future = rpc_->async_send_request(request);
  try {
    wait_for([&future] {return future.wait_for(0s) == std::future_status::ready;},
      3s, recovery, "RPC response timeout");
  } catch (...) {
    // 응답 대기를 포기한 요청을 클라이언트 내부 목록에서 제거한다.
    // 로봇에 전달된 요청 자체를 취소하는 것은 아니다.
    rpc_->remove_pending_request(future);
    throw;
  }
  const auto response = future.get();
  // 서비스 응답 수신 성공과 로봇 API 처리 성공은 별개다. status=0도 확인한다.
  if (response->msg.status != 0) {
    throw std::runtime_error("RPC status=" + std::to_string(response->msg.status) +
      ", body=" + response->msg.body);
  }
  return response->msg.body;
}


int CustomArmExample::get_mode(bool recovery)
{
  // 일시적인 통신 실패를 고려해 최대 5회 시도한다.
  for (int attempt = 1; attempt <= 5; ++attempt) {
    check_running(recovery);
    try {
      // GetMode의 JSON 응답 예: {"mode": 2}. 모드 값은 정수여야 한다.
      const auto body = nlohmann::json::parse(call(kGetMode, "", recovery));
      if (!body.at("mode").is_number_integer()) {
        throw std::runtime_error("GetMode response mode is not an integer");
      }
      const int mode = body.at("mode").get<int>();
      RCLCPP_INFO(get_logger(), "[RPC] GetMode=%d", mode);
      return mode;
    } catch (const std::exception & e) {
      check_running(recovery);
      RCLCPP_WARN(get_logger(), "[RPC] GetMode %d/5: %s", attempt, e.what());
      if (attempt < 5) {pause(1s, recovery);}
    }
  }
  throw std::runtime_error("GetMode failed after 5 attempts");
}


void CustomArmExample::change_mode(int target, bool recovery)
{
  // 일시적인 통신 실패를 고려해 최대 5회 시도한다.
  for (int attempt = 1; attempt <= 5; ++attempt) {
    check_running(recovery);
    try {
      if (get_mode(recovery) == target) {return;}
    } catch (const std::exception & e) {
      check_running(recovery);
      // 복구 중에는 현재 모드 조회가 실패해도 Walking 전환 요청을 계속한다.
      if (!recovery) {throw;}
      RCLCPP_WARN(get_logger(), "[CLEANUP] Mode query failed: %s", e.what());
    }
    RCLCPP_INFO(get_logger(), "[MODE] Request target=%d (%d/5)", target, attempt);
    try {
      call(kChangeMode, nlohmann::json{{"mode", target}}.dump(), recovery);
    } catch (const std::exception & e) {
      check_running(recovery);
      RCLCPP_WARN(get_logger(), "[MODE] Request error; checking actual mode: %s", e.what());
    }
    // 전환 요청이 타임아웃돼도 실제 전환은 됐을 수 있으므로 다시 조회한다.
    pause(1s, recovery);
    try {
      if (get_mode(recovery) == target) {return;}
    } catch (const std::exception & e) {
      check_running(recovery);
      RCLCPP_WARN(get_logger(), "[MODE] Confirmation failed: %s", e.what());
    }
  }
  throw std::runtime_error("Could not confirm target mode " + std::to_string(target));
}

