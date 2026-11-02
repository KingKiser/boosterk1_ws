#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace custom_arm
{
constexpr std::size_t kJointCount = 22;
using Angles = std::array<float, kJointCount>;
constexpr double kPi = 3.14159265358979323846;

// Python change.py의 준비 자세와 게인을 C++ 상수로 옮겼다. 각도 단위는 rad.
// 직렬 관절 순서: 머리 0~1, 왼팔 2~5, 오른팔 6~9, 왼다리 10~15, 오른다리 16~21.
// 어깨 롤: 3/7, 팔꿈치 피치: 4/8. Python 파일 수정은 여기에 자동 반영되지 않는다.
inline constexpr Angles kPrepare = {
  0, 0, 0, -1.3F, 0, 0, 0, 1.3F, 0, 0,
  0, 0, 0, 0.105F, -0.10F, 0, 0, 0, 0, 0.105F, -0.10F, 0};
// 위치 오차에 대한 비례 게인.
inline constexpr Angles kKp = {
  40, 40, 40, 50, 20, 20, 40, 50, 20, 20,
  350, 350, 180, 350, 250, 250, 350, 350, 180, 350, 250, 250};
// 속도 오차에 대한 감쇠 게인.
inline constexpr Angles kKd = {
  1.5F, 1.5F, 0.5F, 1.5F, 0.2F, 0.2F, 0.5F, 1.5F, 0.2F, 0.2F,
  7.5F, 7.5F, 3, 5.5F, 5, 5, 7.5F, 7.5F, 3, 5.5F, 5, 5};

// 진행률을 0~1로 제한하고, 양 끝의 기울기가 0인 코사인 보간 계수로 바꾼다.
inline double soft(double u)
{
  return 0.5 - 0.5 * std::cos(kPi * std::clamp(u, 0.0, 1.0));
}

// 관절별 선형 보간. w=0이면 from, w=1이면 to를 반환한다.
inline Angles interpolate(const Angles & from, const Angles & to, double w)
{
  Angles q{};
  for (std::size_t j = 0; j < kJointCount; ++j) {
    q[j] = static_cast<float>((1.0 - w) * from[j] + w * to[j]);
  }
  return q;
}

// moving_way()와 같은 궤적을 시각 t(초)에 대해 계산한다.
// 호출 측에서 t=0, 0.02, ..., 9.98의 500점을 사용한다.
// 머리와 다리는 준비 자세를 유지하고 어깨 롤과 팔꿈치 피치 목표만 바꾼다.
inline Angles trajectory(double t)
{
  // 팔을 든 기준 자세: 좌우 어깨 롤 ∓15도, 팔꿈치 피치 ∓30도.
  Angles raised = kPrepare;
  raised[3] = static_cast<float>(-15 * kPi / 180);
  raised[7] = static_cast<float>(15 * kPi / 180);
  raised[4] = static_cast<float>(-30 * kPi / 180);
  raised[8] = static_cast<float>(30 * kPi / 180);
  // 1~3초: 준비 자세에서 팔을 든 자세로 부드럽게 이동한다.
  if (t >= 1 && t < 3) {
    return interpolate(kPrepare, raised, soft((t - 1) / 2));
  }
  // 3~7초: 팔꿈치를 최대 ±35도, 1초 주기로 흔든다.
  // 양쪽 soft 계수는 시작/종료 각 0.5초 동안 진폭을 부드럽게 키우고 줄인다.
  if (t >= 3 && t < 7) {
    const double shake = 35 * kPi / 180 * std::sin(2 * kPi * (t - 3)) *
      soft((t - 3) / 0.5) * soft((7 - t) / 0.5);
    raised[4] -= static_cast<float>(shake);
    raised[8] += static_cast<float>(shake);
    return raised;
  }
  // 7~9초: 준비 자세로 돌아온다.
  if (t >= 7 && t < 9) {
    return interpolate(raised, kPrepare, soft((t - 7) / 2));
  }
  // 0~1초와 9~10초에는 준비 자세를 유지한다.
  return kPrepare;
}
}  // namespace custom_arm
