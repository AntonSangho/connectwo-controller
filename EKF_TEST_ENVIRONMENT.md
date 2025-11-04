# EKF 테스트 환경 구성 및 실행 가이드

## 테스트 환경 구성

### 하드웨어 및 네트워크 구성

```
┌─────────────────────────────────────────────────────────────┐
│                    무선 네트워크 (WiFi)                        │
└─────────────────────────────────────────────────────────────┘
            │                                    │
            │                                    │
    ┌───────▼────────┐                  ┌────────▼───────┐
    │   Pi400        │                  │   XPS13        │
    │ Ubuntu 20.04   │                  │ Ubuntu 20.04   │
    │ ROS Noetic     │                  │ ROS Noetic     │
    │                │                  │                │
    │ - roscore      │                  │ - RViz         │
    │ - rosserial    │                  │ - rqt_graph    │
    │ - EKF node     │                  │ - 분석 도구     │
    │ - rosbag       │                  │                │
    └────────┬───────┘                  └────────────────┘
             │ USB Serial
             │ /dev/Connectwo
             │ Baud: 57600
    ┌────────▼───────┐
    │  Connectwo     │
    │  STM32F446RE   │
    │                │
    │ - Motor Control│
    │ - Encoder ODom │
    │ - IMU          │
    │ - rosserial    │
    └────────────────┘
```

### 장비 사양

| 장비 | OS | ROS | 역할 | 연결 |
|------|----|----|------|------|
| **Pi400** | Ubuntu 20.04 | Noetic | 메인 컨트롤러, 데이터 수집 | STM32에 USB 시리얼 연결 |
| **XPS13** | Ubuntu 20.04 | Noetic | 모니터링, 시각화 | Pi400과 WiFi 연결 |
| **Connectwo** | STM32F446RE | rosserial | 로봇 제어, 센서 데이터 | Pi400에 시리얼 연결 |

---

## 네트워크 설정

### Pi400 설정 (ROS Master)

```bash
# Pi400에서 실행
# ~/.bashrc 또는 테스트 세션마다 실행

# Pi400의 IP 주소 확인
hostname -I
# 예: 192.168.1.100

# ROS Master를 Pi400로 설정
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_IP=192.168.1.100  # Pi400의 실제 IP

# ROS 환경 설정
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

**자동 설정 (권장)**:
```bash
# Pi400 ~/.bashrc에 추가
echo "export ROS_MASTER_URI=http://\$(hostname -I | awk '{print \$1}'):11311" >> ~/.bashrc
echo "export ROS_IP=\$(hostname -I | awk '{print \$1}')" >> ~/.bashrc
source ~/.bashrc
```

### XPS13 설정 (Client)

```bash
# XPS13에서 실행
# ~/.bashrc 또는 테스트 세션마다 실행

# Pi400의 IP 주소 (위에서 확인한 값)
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_IP=192.168.1.50  # XPS13의 실제 IP (확인: hostname -I)

# ROS 환경 설정
source /opt/ros/noetic/setup.bash
```

### 네트워크 테스트

```bash
# Pi400에서 roscore 시작
roscore

# XPS13에서 토픽 확인 (다른 터미널)
rostopic list

# XPS13에서 테스트 메시지 발행
rostopic pub -1 /test std_msgs/String "Hello from XPS13"

# Pi400에서 수신 확인
rostopic echo /test
```

성공하면 네트워크 설정 완료!

---

## 테스트 시나리오

### 시나리오 1: Baseline - Raw Odom만 사용

**목적**: EKF 없이 순수 엔코더 오도메트리 성능 측정

#### Pi400에서 실행

```bash
# Terminal 1: roscore
roscore

# Terminal 2: STM32 연결
roslaunch connectwo_bringup connectwo_core.launch

# Terminal 3: 키보드 제어
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# Terminal 4: 데이터 수집
cd ~/bagfiles  # 또는 원하는 경로
rosbag record -O baseline_$(date +%Y%m%d_%H%M%S).bag \
  /odom \
  /imu \
  /cmd_vel \
  /tf
```

#### XPS13에서 모니터링 (선택)

```bash
# 토픽 모니터링
rostopic hz /odom
rostopic echo /odom

# RViz 시각화
rviz
# Fixed Frame: odom
# Add -> Odometry -> Topic: /odom
# Add -> TF
```

#### 주행 패턴

**패턴 A: 직진 (1m)**
- `i` 키를 눌러 전진
- 1m 지점에서 `k` 키로 정지
- 실제 거리 측정

**패턴 B: 제자리 회전 (360°)**
- `j` 키로 천천히 좌회전 (또는 `l` 우회전)
- 시작 방향으로 복귀
- Ctrl+C로 rosbag 종료

---

### 시나리오 2: EKF with IMU - 센서 융합

**목적**: IMU를 포함한 EKF 융합 성능 측정

#### Pi400에서 실행

```bash
# Terminal 1: roscore
roscore

# Terminal 2: STM32 연결
roslaunch connectwo_bringup connectwo_core.launch

# Terminal 3: EKF 실행
roslaunch connectwo_bringup connectwo_ekf.launch

# Terminal 4: 키보드 제어
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# Terminal 5: 데이터 수집
cd ~/bagfiles
rosbag record -O ekf_with_imu_$(date +%Y%m%d_%H%M%S).bag \
  /odom \
  /imu \
  /odometry/filtered \
  /cmd_vel \
  /tf
```

#### XPS13에서 실시간 비교

```bash
# Terminal 1: Raw Odom
rostopic echo /odom | grep -A 4 "position:\|orientation:"

# Terminal 2: Filtered Odom
rostopic echo /odometry/filtered | grep -A 4 "position:\|orientation:"

# Terminal 3: RViz
rviz
# Fixed Frame: odom
# Add -> Odometry -> Topic: /odom (빨강)
# Add -> Odometry -> Topic: /odometry/filtered (파랑)
# 두 경로가 얼마나 겹치는지 확인
```

#### 주행 패턴
- 시나리오 1과 동일한 패턴 수행
- Raw Odom vs Filtered 실시간 비교

---

### 시나리오 3: EKF without IMU - 오도메트리만

**목적**: IMU 문제 격리 테스트

#### 준비: IMU 비활성화 launch 파일 생성

**Pi400에서:**
```bash
cd ~/catkin_ws/src/connectwo_bringup/launch
cp connectwo_ekf.launch connectwo_ekf_no_imu.launch
nano connectwo_ekf_no_imu.launch
```

**수정 내용:**
```xml
<!-- 26번째 줄: IMU 토픽 주석 처리 -->
<!-- <param name="imu0" value="/imu"/> -->

<!-- 45-54번째 줄: IMU 설정 전체 주석 처리 -->
<!--
<rosparam param="imu0_config">[false, false, false,
                                false, false, true,
                                false, false, false,
                                false, false, false,
                                false, false, false]</rosparam>

<param name="imu0_differential" value="false"/>
<param name="imu0_relative" value="false"/>
<param name="imu0_remove_gravitational_acceleration" value="true"/>
-->
```

저장: `Ctrl+X`, `Y`, `Enter`

#### Pi400에서 실행

```bash
# Terminal 1-2: 이전과 동일
roscore
roslaunch connectwo_bringup connectwo_core.launch

# Terminal 3: EKF (IMU 제외)
roslaunch connectwo_bringup connectwo_ekf_no_imu.launch

# Terminal 4: 키보드 제어
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# Terminal 5: 데이터 수집
rosbag record -O ekf_no_imu_$(date +%Y%m%d_%H%M%S).bag \
  /odom \
  /odometry/filtered \
  /cmd_vel \
  /tf
```

**예상 결과**: `/odometry/filtered`가 `/odom`과 거의 동일 (차이 < 1cm)

---

### 시나리오 4: SLAM with EKF - 통합 테스트

**목적**: EKF가 SLAM 맵핑 품질에 미치는 영향 확인

**사전 준비**: RPLidar 연결 확인
```bash
ls /dev/Rplidar /dev/ttyUSB*
```

#### Pi400에서 실행

```bash
# Terminal 1: roscore
roscore

# Terminal 2: STM32
roslaunch connectwo_bringup connectwo_core.launch

# Terminal 3: RPLidar
roslaunch connectwo_bringup connectwo_rplidar.launch

# Terminal 4: EKF
roslaunch connectwo_bringup connectwo_ekf.launch

# Terminal 5: Gmapping
roslaunch connectwo_bringup connectwo_gmapping_ekf.launch

# Terminal 6: 키보드
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# Terminal 7: 데이터 수집
rosbag record -O slam_ekf_$(date +%Y%m%d_%H%M%S).bag \
  /odom \
  /imu \
  /odometry/filtered \
  /scan \
  /map \
  /cmd_vel \
  /tf
```

#### XPS13에서 시각화

```bash
# RViz
rviz -d $(rospack find connectwo_bringup)/rviz/slam.rviz

# 또는 수동 설정:
rviz
# Fixed Frame: map
# Add -> Map (/map)
# Add -> LaserScan (/scan)
# Add -> Odometry (/odometry/filtered)
# Add -> TF
```

**확인사항**:
- 벽이 한 겹으로 그려지는가?
- 맵이 일관성 있게 생성되는가?

---

## 데이터 분석

### Pi400에서 분석

```bash
cd ~/bagfiles

# 단일 파일 분석
python3 ~/Downloads/analyze_odom_bag.py baseline_20251030_143022.bag

# 결과:
# - 터미널: 상세 통계
# - PNG 파일: baseline_20251030_143022_analysis.png
```

### XPS13으로 그래프 전송

```bash
# Pi400에서 실행
scp ~/bagfiles/*_analysis.png anton@192.168.1.50:~/Downloads/

# XPS13에서 확인
cd ~/Downloads
eog *_analysis.png  # 이미지 뷰어로 열기
```

또는 XPS13에서 직접 분석:

```bash
# XPS13에서 bag 파일 가져오기
scp anton@192.168.1.100:~/bagfiles/*.bag ~/Downloads/

# 분석
cd ~/Downloads
python3 analyze_odom_bag.py baseline_*.bag
```

---

## 분석 지표 및 평가

### 1. 위치 오차 (Position Error)

```
Final Position:
  Raw Odom:     x=0.244m, y=-0.057m
  EKF Filtered: x=0.244m, y=-0.057m
  Difference:   Δx=0.00cm, Δy=0.00cm

평가:
  ✅ < 5cm: 우수
  ⚠️  5-10cm: 보통
  ❌ > 10cm: 불량
```

### 2. 회전 오차 (Yaw Error)

```
Final Yaw:
  Raw Odom:     -20.56°
  EKF Filtered:  13.57°
  Difference:    34.13°

평가:
  ✅ < 5°: 우수
  ⚠️  5-10°: 보통
  ❌ > 10°: 불량 (IMU 문제 가능성)
```

### 3. 시나리오별 기대 결과

| 시나리오 | Δx/Δy | Δyaw | 판단 |
|---------|-------|------|------|
| 1. Baseline | - | - | 기준값 |
| 2. EKF with IMU | < 5cm | < 5° | ✅ IMU 융합 정상 |
| 2. EKF with IMU | < 5cm | > 20° | ❌ IMU yaw 부정확 |
| 3. EKF no IMU | < 1cm | < 1° | ✅ EKF 정상 작동 |
| 4. SLAM | - | - | 벽 중복 여부 확인 |

---

## 문제 해결

### 문제 1: XPS13에서 토픽이 안 보임

**증상**:
```bash
# XPS13에서
rostopic list
# ERROR: Unable to communicate with master!
```

**해결**:
```bash
# 1. Pi400에서 roscore 실행 중인지 확인
# Pi400:
roscore

# 2. XPS13에서 네트워크 설정 확인
echo $ROS_MASTER_URI
# 출력: http://192.168.1.100:11311

echo $ROS_IP
# 출력: 192.168.1.50 (XPS13의 IP)

# 3. Pi400와 통신 테스트
ping 192.168.1.100

# 4. 방화벽 확인 (Pi400와 XPS13 모두)
sudo ufw status
# 필요시:
sudo ufw allow 11311/tcp
```

---

### 문제 2: EKF 노드가 시작 안 됨

**증상**:
```bash
[ERROR] [ekf_localization]: Timed out waiting for transform
```

**해결**:
```bash
# 1. 입력 토픽 확인
rostopic list | grep -E "odom|imu"
# 출력되어야 함: /odom, /imu

# 2. 토픽 주파수 확인
rostopic hz /odom
# 출력: average rate: 30.000 (또는 유사한 값)

rostopic hz /imu
# 출력: average rate: 30.000

# 3. TF 확인
rosrun tf view_frames
evince frames.pdf
# odom -> base_footprint 연결 확인

# 4. Frame ID 확인
rostopic echo /odom -n 1 | grep frame_id
# 출력: frame_id: "odom"

rostopic echo /imu -n 1 | grep frame_id
# 출력: frame_id: "imu_link"
```

---

### 문제 3: IMU yaw가 엔코더와 크게 다름 (> 20°)

**진단**:
```bash
# Pi400 또는 XPS13에서 실시간 비교

# Terminal 1: Raw Odom yaw
rostopic echo /odom | grep -A 4 "orientation:"

# Terminal 2: IMU yaw
rostopic echo /imu | grep -A 4 "orientation:"

# 로봇을 천천히 360도 회전
# 두 값이 비슷한 패턴으로 변해야 함
```

**패턴별 해결**:

| 증상 | 원인 | 해결 |
|------|------|------|
| IMU yaw 반대 방향 | 좌표계 반대 | STM32 펌웨어에서 부호 변경 |
| IMU yaw 2배 빠름 | 단위 변환 오류 | 라디안/도 확인 |
| IMU yaw 드리프트 | 자이로 바이어스 | IMU 캘리브레이션 |
| IMU yaw 불규칙 점프 | 자기 간섭 | 모터와 IMU 거리 확보 |

**임시 해결**: 시나리오 3 (EKF without IMU) 실행

---

### 문제 4: rosbag 파일이 너무 큼

**증상**: 수백 MB 이상

**해결**:
```bash
# 필요한 토픽만 녹화
rosbag record -O test.bag /odom /odometry/filtered /cmd_vel

# 또는 시간 제한
timeout 60s rosbag record -O test_60s.bag /odom /odometry/filtered

# 압축 사용
rosbag record -O test.bag --bz2 /odom /odometry/filtered
```

---

## 빠른 시작 가이드

### 가장 간단한 테스트 (시나리오 2)

#### Pi400에서 한 번에 실행

```bash
#!/bin/bash
# 파일명: test_ekf.sh

# ROS 환경 설정
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# 1. roscore
gnome-terminal -- bash -c "roscore; exec bash"
sleep 3

# 2. STM32 연결
gnome-terminal -- bash -c "roslaunch connectwo_bringup connectwo_core.launch; exec bash"
sleep 3

# 3. EKF
gnome-terminal -- bash -c "roslaunch connectwo_bringup connectwo_ekf.launch; exec bash"
sleep 2

# 4. 키보드 제어
gnome-terminal -- bash -c "rosrun teleop_twist_keyboard teleop_twist_keyboard.py; exec bash"

# 5. rosbag 녹화 (현재 터미널)
cd ~/bagfiles
rosbag record -O ekf_test_$(date +%Y%m%d_%H%M%S).bag \
  /odom /imu /odometry/filtered /cmd_vel /tf

echo "테스트 완료! Ctrl+C로 종료"
```

실행:
```bash
chmod +x test_ekf.sh
./test_ekf.sh
```

---

## 체크리스트

### 테스트 전 준비

**하드웨어**:
- [ ] Pi400 부팅 완료
- [ ] Connectwo (STM32) USB 연결 (`ls /dev/Connectwo`)
- [ ] RPLidar USB 연결 (SLAM 시) (`ls /dev/Rplidar`)
- [ ] 배터리 충전 확인

**네트워크**:
- [ ] Pi400와 XPS13 같은 WiFi 연결
- [ ] Pi400: `ROS_MASTER_URI` 설정
- [ ] XPS13: `ROS_MASTER_URI` 설정 (Pi400 IP)
- [ ] `rostopic list` 통신 테스트

**소프트웨어**:
- [ ] Pi400: ROS 환경 설정 (`source ~/catkin_ws/devel/setup.bash`)
- [ ] XPS13: ROS 환경 설정
- [ ] 분석 스크립트 위치 확인 (`~/Downloads/analyze_odom_bag.py`)

**테스트 공간**:
- [ ] 최소 2m x 2m 공간 확보
- [ ] 바닥 마킹 (초기 위치)
- [ ] 줄자 준비

### 테스트 중

- [ ] rosbag 녹화 시작 확인 (`rostopic list` 에서 확인)
- [ ] 초기 위치 마킹
- [ ] 주행 패턴 수행
- [ ] 최종 위치 측정
- [ ] Ctrl+C로 rosbag 종료
- [ ] bag 파일명에 시나리오 명시

### 테스트 후

- [ ] bag 파일 저장 확인 (`ls -lh ~/bagfiles/*.bag`)
- [ ] 즉시 분석 (`python3 analyze_odom_bag.py <bagfile>`)
- [ ] PNG 파일 생성 확인
- [ ] XPS13으로 결과 전송 (선택)
- [ ] 결과 기록 (위치 오차, yaw 오차)

---

## 요약: 네트워크 구성별 명령어

### 모든 작업을 Pi400에서만 (XPS13 없이)

```bash
# Pi400
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1

roscore &
roslaunch connectwo_bringup connectwo_keyboard.launch &
roslaunch connectwo_bringup connectwo_ekf.launch

rosbag record -O test.bag /odom /odometry/filtered /cmd_vel /tf
```

### Pi400 (실행) + XPS13 (모니터링)

```bash
# Pi400
export ROS_MASTER_URI=http://$(hostname -I | awk '{print $1}'):11311
export ROS_IP=$(hostname -I | awk '{print $1}')

roscore &
roslaunch connectwo_bringup connectwo_keyboard.launch &
roslaunch connectwo_bringup connectwo_ekf.launch
rosbag record -O test.bag /odom /odometry/filtered

# XPS13
export ROS_MASTER_URI=http://192.168.1.100:11311  # Pi400 IP
export ROS_IP=192.168.1.50  # XPS13 IP

rviz  # 시각화
rostopic echo /odometry/filtered  # 모니터링
```

---

**작성일**: 2025-10-30
**버전**: 1.0
**테스트 환경**: Pi400 (Ubuntu 20.04, ROS Noetic) + XPS13 (Ubuntu 20.04, ROS Noetic) + Connectwo (STM32F446RE)
**관련 문서**: EKF_TROUBLESHOOTING.md, ROBOT_LOCALIZATION_GUIDE.md
