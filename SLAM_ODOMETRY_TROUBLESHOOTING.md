# SLAM 및 Odometry 트러블슈팅 전체 과정

**작성일**: 2025-11-03
**환경**: Pi400 (ROS Master) + XPS13 (RViz 모니터링) + Connectwo STM32F446RE
**작업 브랜치**: `odom4`

---

## 목차

1. [초기 문제 상황](#초기-문제-상황)
2. [진단 과정](#진단-과정)
3. [해결한 문제들](#해결한-문제들)
4. [현재 남은 문제](#현재-남은-문제)
5. [참고 문서](#참고-문서)

---

## 초기 문제 상황

### 문제 1: ROS Topic 통신 불가 (XPS13)

**증상:**
```bash
# XPS13에서
rostopic list
# ERROR: Unable to communicate with master!
```

**원인:**
- `roscore`가 Pi400에서 실행 중이지만 XPS13 환경 변수 미설정

**해결:**
```bash
# XPS13에서
export ROS_MASTER_URI=http://192.168.1.18:11311  # Pi400 IP
export ROS_IP=192.168.1.38  # XPS13 IP
```

**참고**: EKF_TEST_ENVIRONMENT.md (Line 49-106)

---

### 문제 2: Gmapping 맵 품질 불량

**증상:**
- 만들어지는 맵이 실제 구성과 안 맞음
- 벽을 인지 못하는 경우 있음
- RViz에서 `odom` 프레임이 `map`, `base_footprint`, `laser`와 멀리 떨어져 있음
- 맵이 계속 뭉개지고 중복됨 (같은 벽이 여러 겹)

**실행 중인 노드:**
```bash
/base_to_imu
/base_to_laser
/connectwo_core
/ekf_localization      # ← EKF 실행 중
/rosout
/rplidarNode
/rviz_*
/teleop_twist_keyboard
/turtlebot3_slam_gmapping
```

**발행되는 토픽:**
```
/odom                  # Raw encoder odometry
/odometry/filtered     # EKF filtered odometry
/imu
/scan
/map
/tf
```

---

## 진단 과정

### 1단계: TF 구조 확인

**실행:**
```bash
# Pi400에서
rosrun tf view_frames
evince frames.pdf
```

**결과:**
```
map → odom → base_footprint → laser
                            → imu_link
```

**판정:** ✅ TF 구조 정상

- `map → odom`: Gmapping이 발행
- `odom → base_footprint`: connectwo_core가 발행
- `base_footprint → laser/imu_link`: static_transform_publisher

---

### 2단계: EKF 충돌 확인

**문제 발견:**
- EKF 노드(`/ekf_localization`)가 `/odometry/filtered` 발행
- Gmapping이 여전히 `/odom` (raw encoder) 사용
- EKF는 실행되지만 Gmapping이 활용하지 못함

**결론:**
- EKF 없이 Gmapping만 실행하기로 결정
- Raw encoder odometry의 정확도가 맵 품질의 핵심

---

### 3단계: Raw Odom 품질 확인

**EKF 없이 Gmapping 실행 결과:**

![맵 뭉개짐](이미지 참조)

- 같은 벽이 여러 겹으로 중복
- 방 경계가 일치하지 않음
- 맵이 지속적으로 drift

**결론:** Raw encoder odometry의 정확도가 매우 낮음

---

### 4단계: 엔코더 파라미터 확인

**현재 설정값:**
```cpp
// connectwo_config.h
#define TICK2RAD           0.00805537  // 2π / 780 PPR
#define WHEEL_RADIUS       0.0575      // 57.5mm
#define WHEEL_SEPARATION   0.385       // 385mm
```

**엔코더 코드:**
```cpp
// ros_main.cpp:185-188 (수정 전)
int32_t left_front_tick  = -motor[0];  // 왼쪽 음수
int32_t left_rear_tick   = -motor[1];
int32_t right_front_tick = +motor[2];  // 오른쪽 양수
int32_t right_rear_tick  = +motor[3];
```

---

### 5단계: 직진 테스트로 엔코더 방향 확인

**테스트:**
```bash
# Pi400에서
rostopic echo /odom/pose/pose/position -n 1
# x: 0.0, y: 0.0, z: 0.0

# 로봇을 'i' 키로 전진

rostopic echo /odom/pose/pose/position -n 1
# x: -1.50, y: 0.006, z: 0.0
```

**결과 분석:**
- ❌ **x: 0.0 → -1.50** (감소!) - 엔코더 방향 반대!
- ✅ **y: 0.0 → 0.006** (거의 0) - 좌우 균형은 정상

**RViz 증거:**
- 직진 명령('i')을 했는데 odom 경로(빨간선)가 **반대 방향**으로 기록됨
- 로봇 화살표는 올바른 방향인데 경로는 역방향

---

## 해결한 문제들

### 해결 1: 엔코더 방향 수정

**파일:** `stm32cubeide/MC_lib/main/ros/src/ros_main.cpp:185-188`

**수정 내용:**
```cpp
// 수정 전 (모든 엔코더 부호를 반대로)
int32_t left_front_tick  = -motor[0];
int32_t left_rear_tick   = -motor[1];
int32_t right_front_tick = +motor[2];
int32_t right_rear_tick  = +motor[3];

// 수정 후
int32_t left_front_tick  = +motor[0];  // 양수로 변경
int32_t left_rear_tick   = +motor[1];  // 양수로 변경
int32_t right_front_tick = -motor[2];  // 음수로 변경
int32_t right_rear_tick  = -motor[3];  // 음수로 변경
```

**테스트 결과:**
```bash
# 수정 후 직진 테스트
rostopic echo /odom/pose/pose/position -n 1
# x: 0.0 → 2.25

# ✅ x 증가 (정상!)
# ✅ y: 0.08 (8cm drift / 2.25m 이동 - 허용 범위)
```

**커밋:**
```
Commit: e4a4088
Author: AntonSangho <sanghoemail@gmail.com>
Date:   Mon Nov 3 13:42:48 2025

Fix: 엔코더 부호 보정으로 오도메트리 방향 수정
```

---

### 해결 2: IMU 의존성 제거 (이전 작업)

**배경:**
- Raw Odom이 IMU 절대 yaw 값을 사용하고 있었음
- IMU yaw가 잘못되면 오도메트리 전체가 망가짐
- 360도 회전해도 yaw가 거의 변화 없음

**해결:** 엔코더 기반 Differential Drive 오도메트리로 변경

**파일:** `stm32cubeide/MC_lib/main/ros/src/ros_main.cpp:698`

```cpp
// 수정 전 (IMU 사용)
theta = DEG2RAD(__imu.data.e_yaw);  // ❌ IMU 의존
delta_theta = theta - last_theta;

// 수정 후 (엔코더 사용)
delta_theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;  // ✅ 순수 엔코더
```

**참고 문서:** ODOM_FIX_PATCH.md

---

### 해결 3: Reset 콜백 구현

**기능:**
- `/reset` 토픽으로 오도메트리를 초기화할 수 있음
- 엔코더 베이스라인 재설정

**사용법:**
```bash
# Pi400에서
rostopic pub /reset std_msgs/Empty "{}"
```

**코드:** `ros_main.cpp:365-387`

---

## 현재 남은 문제

### 문제: 맵이 반대 방향으로 생성됨

**증상:**
- ✅ 로봇 움직이는 방향과 화살표(base_footprint)는 일치
- ✅ odom position.x가 전진 시 증가
- ✅ laser scan이 로봇과 함께 이동
- ❌ **벽의 반대 방향으로 가서 매핑을 함**
- ❌ **로봇이 전진하면 맵이 생성된 반대 방향으로 전진**

**RViz 관찰:**
- Fixed Frame: `map`
- base_footprint는 올바른 방향으로 이동
- LaserScan도 base_footprint와 함께 이동
- 하지만 맵이 반대로 그려짐 (실제 앞에 벽이 있는데 뒤에 생성)

**시도한 해결책:**
1. ❌ RPLidar launch 파일에서 `inverted=true` 설정 → 여전히 반대로 감

---

## 가능한 원인 및 다음 시도 사항

### 원인 1: LaserScan angle 방향

**확인 필요:**
```bash
# Pi400에서
rostopic echo /scan -n 1 | grep -E "angle_min|angle_max|angle_increment"
```

**정상:**
```
angle_min: 0.0
angle_max: 6.28
angle_increment: 0.0087 (양수)
```

**문제:**
```
angle_min > angle_max
또는
angle_increment: 음수
```

---

### 원인 2: laser TF 180도 회전 필요

**현재 TF 확인:**
```bash
rosrun tf tf_echo base_footprint laser
```

**임시 테스트:**
```bash
# 현재 base_to_laser 종료
rosnode kill /base_to_laser

# 180도 회전된 TF 발행
rosrun tf static_transform_publisher 0.1 0 0.2 3.14159 0 0 base_footprint laser 100
```

맵이 올바르게 생성되면 → launch 파일 영구 수정

---

### 원인 3: Odometry angular.z 부호 반대

**확인:**
```bash
rostopic echo /odom/twist/twist/angular

# 로봇을 왼쪽('j')으로 회전:
# - z > 0 (반시계방향) ← 정상
# - z < 0 (시계방향) ← 반대!
```

**만약 부호가 반대라면:**
- `ros_main.cpp:698`의 `delta_theta` 계산식에 음수 부호 추가
```cpp
delta_theta = -WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;
//            ^ 음수 추가
```

---

### 원인 4: RPLidar 물리적 장착 방향

**확인 사항:**
1. 라이다가 로봇 전방을 향하고 있는가?
2. 라이다 케이블이 어느 방향인가? (케이블 방향 = 후방)
3. 라이다가 180도 뒤집혀 장착되어 있지 않은가?

---

## 디버깅 체크리스트

### 1. LaserScan 데이터 확인
```bash
# Pi400
rostopic echo /scan -n 1 | grep angle
```
- [ ] angle_min < angle_max
- [ ] angle_increment > 0

### 2. LaserScan 시각화 (base_footprint 기준)
```bash
# XPS13 RViz
# Fixed Frame: base_footprint
# Add -> LaserScan (/scan)
```
- [ ] 앞에 벽을 두면 로봇 앞쪽에 빨간 점들이 보임
- [ ] 뒤에 벽을 두면 로봇 뒤쪽에 빨간 점들이 보임

### 3. Odometry 회전 방향
```bash
rostopic echo /odom/twist/twist/angular
# 'j' 키(왼쪽 회전) 시
```
- [ ] z > 0 (정상)
- [ ] z < 0 (반대)

### 4. TF laser 방향
```bash
rosrun tf tf_echo base_footprint laser
```
- [ ] Rotation: [0, 0, 0, 1] (0도)
- [ ] Rotation: [0, 0, 1, 0] (180도)

---

## 시스템 정보

### 하드웨어
- **로봇**: Connectwo (Tresc3 mobile robot base)
- **MCU**: STM32F446RE
- **라이다**: RPLidar
- **모터**: 4륜 차동구동 (differential drive)
- **엔코더**: 780 PPR (추정)

### 소프트웨어
- **OS**: Ubuntu 20.04
- **ROS**: Noetic
- **통신**: rosserial (57600 baud, /dev/Connectwo)

### 네트워크 구성
```
Pi400 (192.168.1.18)         XPS13 (192.168.1.38)
- ROS Master                  - RViz
- roscore                     - 모니터링
- rosserial
- Gmapping
- teleop
  |
  | USB Serial
  |
Connectwo STM32F446RE
- Motor control
- Encoder odometry
- IMU
```

---

## 빌드 및 플래시 명령어

### STM32 펌웨어 빌드
```bash
cd /home/anton/projects/connectwo-controller/stm32cubeide/motor_controller
rm -rf build
mkdir build
cd build
cmake ..
make
make flash
```

### ROS 패키지 빌드
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 실행 명령어

### Pi400에서 SLAM 실행
```bash
# Terminal 1: roscore
roscore

# Terminal 2: STM32 연결
roslaunch connectwo_bringup connectwo_core.launch

# Terminal 3: RPLidar
roslaunch connectwo_bringup connectwo_rplidar.launch

# Terminal 4: Gmapping (EKF 없이)
roslaunch connectwo_bringup connectwo_gmapping.launch

# Terminal 5: 키보드 제어
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### XPS13에서 모니터링
```bash
export ROS_MASTER_URI=http://192.168.1.18:11311
export ROS_IP=192.168.1.38

# RViz
rviz
```

**RViz 설정:**
- Fixed Frame: `map`
- Add → Map (`/map`)
- Add → LaserScan (`/scan`)
- Add → Odometry (`/odom`)
- Add → TF

---

## 참고 문서

### 프로젝트 문서
- `ODOM_FIX_PATCH.md` - IMU 제거 및 엔코더 기반 오도메트리 패치
- `ENCODER_TEST_COMBINATIONS.md` - 엔코더 조합 테스트 가이드
- `EKF_TEST_ENVIRONMENT.md` - EKF 테스트 환경 및 네트워크 설정
- `CLAUDE.md` - 프로젝트 개요 및 아키텍처

### 주요 파일 위치
- **엔코더 파라미터**: `stm32cubeide/MC_lib/main/ros/inc/connectwo_config.h`
- **오도메트리 계산**: `stm32cubeide/MC_lib/main/ros/src/ros_main.cpp:666-717`
- **엔코더 읽기**: `stm32cubeide/MC_lib/main/ros/src/ros_main.cpp:182-195`

### Git 커밋 히스토리
```bash
git log --oneline --graph
```

주요 커밋:
- `e4a4088` - Fix: 엔코더 부호 보정으로 오도메트리 방향 수정
- `a2211be` - ROS 메시지 규격 준수: Covariance 추가 및 통신 속도 개선
- `c383a59` - Motor 소멸자 버그 수정 및 timer10ms 루프 유지

---

## 다음 단계

### 우선순위 1: 맵 반대 방향 문제 해결

**진행 순서:**

1. **LaserScan angle 확인**
   ```bash
   rostopic echo /scan -n 1 | grep angle
   ```

2. **LaserScan 시각화 확인** (base_footprint 기준)
   - 앞에 벽 → 앞쪽에 스캔 표시?
   - 뒤에 벽 → 뒤쪽에 스캔 표시?

3. **Odometry angular.z 부호 확인**
   ```bash
   rostopic echo /odom/twist/twist/angular
   # 'j' 키(왼쪽): z > 0 인가?
   ```

4. **TF laser 180도 회전 테스트**
   ```bash
   rosnode kill /base_to_laser
   rosrun tf static_transform_publisher 0.1 0 0.2 3.14159 0 0 base_footprint laser 100
   ```

5. **각 테스트 후 맵 생성 확인**

---

### 우선순위 2: 휠 파라미터 보정

**직진 거리 테스트:**
```bash
# 실제 1m 이동 시 odom 값 확인
rostopic echo /odom/pose/pose/position
```

**회전 각도 테스트:**
```bash
# 실제 360도 회전 시 odom yaw 확인
rostopic echo /odom/pose/pose/orientation
```

**보정 공식:**
```
새 WHEEL_RADIUS = 현재값 × (실제거리 / odom거리)
새 WHEEL_SEPARATION = 현재값 × (odom각도 / 실제각도)
```

---

### 우선순위 3: Gmapping 파라미터 튜닝

**맵 품질 개선:**
```xml
<param name="linearUpdate" value="0.2"/>    <!-- 20cm마다 업데이트 -->
<param name="angularUpdate" value="0.25"/>  <!-- 14도마다 업데이트 -->
<param name="particles" value="30"/>        <!-- 파티클 수 -->
<param name="minimumScore" value="50"/>     <!-- 스캔 매칭 임계값 -->
```

---

## 트러블슈팅 히스토리

| 날짜 | 문제 | 해결 | 상태 |
|------|------|------|------|
| 2025-11-03 | XPS13에서 rostopic 통신 불가 | ROS_MASTER_URI 설정 | ✅ 해결 |
| 2025-11-03 | 맵 품질 불량, 뭉개짐 | 엔코더 방향 문제 진단 | ✅ 원인 파악 |
| 2025-11-03 | 직진 시 odom x 감소 | 엔코더 부호 반전 | ✅ 해결 |
| 2025-11-03 | 맵이 반대 방향으로 생성 | LaserScan 방향 문제 추정 | 🔄 진행 중 |

---

## 연락처 및 참고자료

### ROS 표준 문서
- [REP-105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
- [Differential Drive Kinematics](http://rossum.sourceforge.net/papers/DiffSteer/)
- [ROS Navigation Tuning Guide](http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom)
- [Gmapping Documentation](http://wiki.ros.org/gmapping)

### 이슈 트래킹
- GitHub Repository: (추가 필요)
- 이슈 번호: (추가 필요)

---

**마지막 업데이트**: 2025-11-03 13:50
**작성자**: AntonSangho
**버전**: 1.0
