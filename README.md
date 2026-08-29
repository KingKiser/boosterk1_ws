# booster K1 for ROS2

> 이 프로젝트는 [탑로보틱스](https://toprobotics.co.kr/) 와 [한국it교육원](https://www.hkit.kr/)이 함께합니다  
<img src="images/toprobotics.png" width="200"> <img src="images/hkit.png" width="200">

> 사용한 휴머노이드 로봇은 부스터로보틱스의 K1입니다 [booster_K1](https://www.booster.tech/)

> 기존 공식 [git](https://github.com/BoosterRobotics/booster_robotics_sdk_ros2) 계정에 있는 코드를 리팩토링 및 상세 가이드를 덧붙혔습니다

**Version:** v1  
**Last Updated:** 2026.08.29  
**Author:** 박승휘

---

## ✅ 휴머노이드 로봇의 ROS2 연동(FastDDS 설정)

- 기존 공식 사이트에는 해당 설정에 대한 가이드가 없어 공식 git 계정에서 받은 걸로 바로 빌드하시면 안됩니다
- ~공식도 어떻게 연결하는지 잘 모르는 거 같다~

<img src="images/jockto.jpg" width="300">

<img src="images/molu.png" width="300">

- 먼저 FastDDS를 설치합니다

```bash
sudo apt update
sudo apt install ros-humble-rmw-fastrtps-cpp
```

로봇 내부에 fastdds_profile.xml 을 PC로 복사하여 해당 값을 고칩니다

```bash
scp booster@<ROBOT_IP>:/opt/booster/BoosterRos2/fastdds_profile.xml ~/fastdds_profile.xml
```

현재 이더넷 IP 주소는 다음 명령어로 확인할 수 있습니다.

```bash
ip addr
```

`fastdds_profile.xml`의 다음 부분을 수정합니다.

```xml
<interfaceWhiteList>
    <address>127.0.0.1</address>
    <address><PC_NETWORK_IP></address>
</interfaceWhiteList>
```

예를 들어 현재 PC의 이더넷 IP가 `192.168.0.10`인 경우:

```xml
<interfaceWhiteList>
    <address>127.0.0.1</address>
    <address>192.168.0.10</address>
</interfaceWhiteList>
```

> `<PC_NETWORK_IP>`에는 로봇과 통신하는 PC의 이더넷 인터페이스에 할당된 실제 IP 주소를 입력합니다.

- 또한 사용하시는 휴머노이드의 fastdds_profile.xml과 같은 폴더에 있는 fastdds_profile_udp_only.xml 에는 현재 로봇의 ssh ip를 추가하여 수정합니다

```xml
<interfaceWhiteList>
    <address>127.0.0.1</address>
    <address>192.168.127.101</address> 
    <address><ROBOT_NETWORK_IP></address>
</interfaceWhiteList>
```

> `<ROBOT_NETWORK_IP>`에는 로봇 ssh ip 주소를 넣어줍니다.

- 이후 로봇은 재부팅하여 해당 xml을 적용해주시고

- PC에서는 해당 터미널에 xml 설정적용과 FastDDS 설정을 하신 후 해당 git의 코드를 이용하시기 바랍니다
- (혹시 모르니 토픽이나 노드는 조회해보시길 바랍니다)

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastdds_profile.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY

ros2 daemon stop
pkill -f _ros2_daemon
```

이후에 빌드 후 실행하시면 됩니다

```bash
cd boosterk1_ws
colcon build
source install/setup.bash
```

![aronaddabong](images/aronaddabong.png)

---

## 📖 Notes

- 현재 예제는 부스터로보틱스에서 준 예제 + 새로 개발한 예제를 포함하고 있습니다(키보드 동작)
- 로봇 구매 대여 관련 연락은 탑로보틱스를 통해 부탁 드립니다(053-384-3023)
- 개발자 연락은 박승휘(ham9301@gmail.com) 입니다

- 힘들었다

![kiriko](images/kiriko.png)

