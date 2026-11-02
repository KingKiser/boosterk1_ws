// ROS 토픽 송수신, 전체 동작 순서와 대기 중 콜백 처리를 구현한다.
#include "../include/example_package/custom_arm_node.h"

#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;

namespace
{
constexpr int kWalking = 2;
constexpr int kCustom = 3;
}  // namespace

CustomArmExample::CustomArmExample(const volatile std::sig_atomic_t & stop_requested)
: Node("custom_arm_example_cpp"), stop_requested_(stop_requested)
{
  // 별도 수신 스레드 없이 같은 스레드에서 상태 콜백과 서비스 응답을 처리한다.
  executor_.add_node(get_node_base_interface());
  // 명령은 Reliable, 상태 수신은 Best effort로 Python 예제와 QoS를 맞춘다.
  // depth=1: 대기 큐에 최신 메시지 1개를 보관한다. 토픽은 네임스페이스/리매핑을 따른다.
  publisher_ = create_publisher<booster_interface::msg::LowCmd>(
    "joint_ctrl", rclcpp::QoS(1).reliable());
  subscription_ = create_subscription<booster_interface::msg::LowState>(
    "low_state", rclcpp::QoS(1).best_effort(),
    [this](booster_interface::msg::LowState::ConstSharedPtr msg) {
      // 관절 수 부족, NaN, 무한대가 있으면 이번 메시지 전체를 사용하지 않는다.
      if (msg->motor_state_serial.size() < custom_arm::kJointCount) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "[STATE] Expected >=22 serial joints, received %zu",
          msg->motor_state_serial.size());
        return;
      }
      for (std::size_t i = 0; i < angles_.size(); ++i) {
        if (!std::isfinite(msg->motor_state_serial[i].q)) {
          return;
        }
      }
      for (std::size_t i = 0; i < angles_.size(); ++i) {
        angles_[i] = msg->motor_state_serial[i].q;
      }
      last_state_ = Clock::now();
      received_state_ = true;
    });
  rpc_ = create_client<Rpc>("booster_rpc_service");
  // LowState의 motor_state_serial과 같은 순서로 22개 관절 명령을 구성한다.
  command_.cmd_type = booster_interface::msg::LowCmd::CMD_TYPE_SERIAL;
  command_.motor_cmd.resize(custom_arm::kJointCount);
  RCLCPP_INFO(get_logger(), "[TOPIC] Publish %s (RELIABLE, depth=1)",
    publisher_->get_topic_name());
  RCLCPP_INFO(get_logger(), "[TOPIC] Subscribe %s (BEST_EFFORT, depth=1)",
    subscription_->get_topic_name());
}


int CustomArmExample::run()
{
  int result = 0;
  try {
    // 1. RPC 연결 및 시작 모드 확인. Walking 상태에서만 동작을 시작한다.
    wait_for([this] {return rpc_->service_is_ready();}, 5s, false,
      "booster_rpc_service unavailable");
    change_mode(kCustom, false);
    if (get_mode(false) != kWalking) {
      throw std::runtime_error("Start mode must be Walking");
    }
    // 2. 초기 관절각과 명령 구독자를 기다린다.
    // rqt도 구독자 수에 포함되므로 이 수치만으로 로봇의 명령 적용을 보장하지 않는다.
    wait_for([this] {return received_state_;}, 5s, false, "No valid low_state");
    wait_for([this] {return publisher_->get_subscription_count() > 0;}, 5s,
      false, "No joint_ctrl subscriber");
    check_state();
    const auto start_angles = angles_;
    RCLCPP_INFO(get_logger(), "[READY] State received; command subscribers=%zu",
      publisher_->get_subscription_count());
    // 3. Custom 진입 전에 현재 자세를 목표로 한 번 전송한다.
    publish(start_angles);
    pause(100ms, false);

    // 4. 타임아웃이 나도 전환됐을 가능성이 있으므로 요청 전에 복구 필요 여부를 기록한다.
    custom_requested_ = true;
    RCLCPP_INFO(get_logger(), "[MOTION] Prepare: 500 samples, target 500 Hz / 1 s");
    // 5. 현재 자세 → 준비 자세: 500점 선형 보간, 목표 2ms 주기로 약 1초.
    // i/499.0은 첫 점을 현재 자세, 마지막 점을 준비 자세에 정확히 맞춘다.
    auto start = Clock::now();
    for (int i = 0; i < 500; ++i) {
      check_running(false);
      check_state();
      publish(custom_arm::interpolate(start_angles, custom_arm::kPrepare, i / 499.0));
      wait_deadline(start + (i + 1) * 2ms, false);
    }
    RCLCPP_INFO(get_logger(), "[MOTION] Arms: 500 samples, target 50 Hz / 10 s");
    // 6. 미리 정의된 팔 궤적: 목표 20ms 주기로 500점, 약 10초.
    // 실제 관절각에 따라 궤적을 보정하는 상위 피드백 제어는 수행하지 않는다.
    start = Clock::now();
    for (int k = 0; k < 500; ++k) {
      check_running(false);
      check_state();
      publish(custom_arm::trajectory(k * 0.02));
      // 50Hz 발행 중 로그는 50점마다 한 번씩 남겨 출력량을 줄인다.
      if (k % 50 == 0) {
        RCLCPP_INFO(get_logger(),
          "[TX] t=%.2f s, shoulder roll=(%.3f, %.3f), elbow pitch=(%.3f, %.3f) rad",
          k * 0.02, command_.motor_cmd[3].q, command_.motor_cmd[7].q,
          command_.motor_cmd[4].q, command_.motor_cmd[8].q);
      }
      wait_deadline(start + (k + 1) * 20ms, false);
    }
    RCLCPP_INFO(get_logger(), "[DONE] Arm trajectory completed; published=%zu", sent_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "[STOP] %s", e.what());
    result = 1;
  }

  // 7. 정상 완료, 오류, 사용자 정지 모두 Custom 요청 이력이 있으면 Walking 복귀를 시도한다.
  if (custom_requested_) {
    try {
      RCLCPP_INFO(get_logger(), "[CLEANUP] Returning to Walking");
      change_mode(kWalking, true);
      RCLCPP_INFO(get_logger(), "[CLEANUP] Walking confirmed");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "[CRITICAL] Walking recovery failed: %s", e.what());
      result = 1;
    }
  }
  return result;
}


void CustomArmExample::check_running(bool recovery)
{
  if (!rclcpp::ok() || (!recovery && stop_requested_)) {
    throw std::runtime_error("Stop requested");
  }
}


void CustomArmExample::pump(bool recovery)
{
  check_running(recovery);
  // 대기만 하고 spin을 하지 않으면 구독 콜백과 서비스 응답 처리가 진행되지 않는다.
  executor_.spin_some();
}


void CustomArmExample::wait_for(const std::function<bool()> & ready, Clock::duration timeout,
    bool recovery, const char * error)
{
  // 조건 충족 또는 타임아웃까지 콜백을 처리하면서 대기한다.
  const auto deadline = Clock::now() + timeout;
  while (true) {
    pump(recovery);
    if (ready()) {return;}
    if (Clock::now() >= deadline) {throw std::runtime_error(error);}
    std::this_thread::sleep_for(1ms);
  }
}


void CustomArmExample::wait_deadline(Clock::time_point deadline, bool recovery)
{
  // 시작 시각 기준의 절대 마감 시각을 사용해 매회 처리 시간의 누적 오차를 줄인다.
  // 일반 OS 스케줄링이므로 정확한 실시간 주기는 보장하지 않는다.
  // 이미 마감 시각이 지났다면 추가로 기다리지 않고 반환한다.
  do {
    pump(recovery);
    if (Clock::now() >= deadline) {return;}
    std::this_thread::sleep_until(std::min(deadline, Clock::now() + 500us));
  } while (true);
}


void CustomArmExample::pause(Clock::duration duration, bool recovery)
{
  wait_deadline(Clock::now() + duration, recovery);
}


void CustomArmExample::check_state()
{
  // 최초 수신 이후에도 상태가 계속 들어오는지 확인한다.
  if (!received_state_ || Clock::now() - last_state_ > 1s) {
    throw std::runtime_error("low_state is stale (>1 s)");
  }
}


void CustomArmExample::publish(const custom_arm::Angles & q)
{
  // 팔뿐 아니라 머리와 다리를 포함한 22개 관절 모두에 목표와 게인을 보낸다.
  for (std::size_t i = 0; i < q.size(); ++i) {
    auto & motor = command_.motor_cmd[i];
    motor.q = q[i];
    // 목표 속도와 추가 토크는 0으로 두고 위치 목표 및 PD 게인을 사용한다.
    motor.dq = 0;
    motor.tau = 0;
    motor.kp = custom_arm::kKp[i];
    motor.kd = custom_arm::kKd[i];
    // 원본 Python 예제의 설정값을 그대로 유지한다.
    motor.weight = 0;
  }
  publisher_->publish(command_);
  ++sent_;
}

