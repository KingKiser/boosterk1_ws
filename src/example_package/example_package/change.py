#!/usr/bin/env python3

# -*- coding: utf-8 -*-
"""팔동작_궤적.py — K1 커스텀 모드 팔 동작 예시 궤적 (2026-10-19)
  다리·머리는 준비 자세(prepare_state)에 고정, 양팔만 움직인다. 모든 구간 코사인 보간(시작·끝 속도 0).
  시간표 (50Hz):
    0.0~1.0초  준비 자세 유지
    1.0~3.0초  양팔을 옆으로 든다 (어깨 롤 ±74.5° → ±15°, 팔꿈치 살짝 굽힘)
    3.0~7.0초  든 채로 팔꿈치 흔들기 (팔꿈치 피치 ±35°, 1초 주기 × 4)
    7.0~9.0초  팔을 내린다 (준비 자세로)
    9.0~10.0초 준비 자세 유지
  관절 순서(22): 머리요·머리피치 · 왼어깨피치·왼어깨롤·왼팔꿈치피치·왼팔꿈치요 · 오른팔(같은 순) · 왼다리 6 · 오른다리 6
"""
import numpy as np

FPS = 50
pre_angle = np.array([0, 0, 0.0, -1.3, 0, 0.0, 0.0, 1.3, 0, 0.0,
               -0.0, 0, 0, 0.105, -0.10, 0., -0.0, 0, 0, 0.105, -0.10, 0.], np.float32)
# 준비 자세 게인 (booster_deploy K1_CFG.prepare_state) — 다리를 단단히 세워 둔다
KP = np.array([40., 40., 40., 50., 20., 20, 40., 50., 20., 20,
               350., 350., 180., 350., 250., 250., 350., 350., 180., 350., 250., 250.], np.float32)
KD = np.array([1.5, 1.5, 0.5, 1.5, 0.2, 0.2, 0.5, 1.5, 0.2, 0.2,
               7.5, 7.5, 3., 5.5, 5.0, 5.0, 7.5, 7.5, 3., 5.5, 5.0, 5.0], np.float32)


def soft(u):
    u = np.clip(u, 0, 1)
    return 0.5 - 0.5 * np.cos(np.pi * u)


def moving_way():
    """(N, 22) 목표각과 시간(초)을 돌려준다."""
    t = np.arange(0, 10.0, 1 / FPS)
    q = np.tile(pre_angle, (len(t), 1))
    take_arm = pre_angle.copy()
    take_arm[3], take_arm[7] = np.radians(-15), np.radians(15)        # 어깨 롤: 옆으로 든다
    take_arm[4], take_arm[8] = np.radians(-30), np.radians(30)        # 팔꿈치 피치: 살짝 굽힘 (좌우 부호 반대)
    for i, s in enumerate(t):
        if 1.0 <= s < 3.0:
            w = soft((s - 1.0) / 2.0); q[i] = (1 - w) * pre_angle + w * take_arm
        elif 3.0 <= s < 7.0:
            q[i] = take_arm.copy()
            shake = np.radians(35) * np.sin(2 * np.pi * (s - 3.0) / 1.0) * soft((s - 3.0) / 0.5) * soft((7.0 - s) / 0.5)
            q[i, 4] = take_arm[4] - shake; q[i, 8] = take_arm[8] + shake
        elif 7.0 <= s < 9.0:
            w = soft((s - 7.0) / 2.0); q[i] = (1 - w) * take_arm + w * pre_angle
    return t, q.astype(np.float32)
