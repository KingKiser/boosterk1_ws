# booster K1 for ROS2

> 이 리포지토리는 [탑로봇틱스](https://toprobotics.co.kr/) 와 [한국교육원](https://www.hkit.kr/)이 함께합니다\n
> 사용한 휴머노이드 로봇은 booster로보틱스의 K1입니다 [booster_K1](https://www.booster.tech/)\n
> 기존 공식 [git](https://github.com/BoosterRobotics/booster_robotics_sdk_ros2) 계정에 있는 코드를 리팩토링 및 상세 가이드를 덧붙혔습니다

**Version:** v1  
**Last Updated:** 2026.08.29  
**Author:** 박승휘

---

## ✅ 휴머노이드 로봇의 ROS2 연동(FastDDS 설정)

- 기존 공식 사이트에는 해당 설정에 대한 가이드가 없어 공식 git 계정에서 받은 걸로 바로 빌드하시면 안됩니다

![joseock](images/joseock.png)

---

### 빌드하는 법

```bash
cd hkit_yahboom_ws
colcon build
```

---

- 워크스페이스 내부에서

### Slam 실행

```bash
source install/setup.bash
ros2 launch amr_server server_slam.launch.py
```

### Nav2 실행

```bash
source install/setup.bash
ros2 launch amr_server server_nav2.launch.py
```

---

## 📖 Notes

- docker_config 폴더는 yahboom 라즈베리파이 내부에서 도커 셋팅하기 위해 첨부한 파일입니다
- yahboom 도커 내부 셋팅은 해당 폴더의 README를 참고해주세요

![kimseongmo](images/kimsungmo.webp)

- 힘들었다

![kiriko](images/kiriko.png)

- 수업 준비하느라 고생했으면 팔로우 해주세요

![arona](images/aronaddabong.png)
