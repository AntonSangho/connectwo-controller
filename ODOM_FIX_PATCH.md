# Odometry Calculation Fix - 엔코더 기반 오도메트리로 수정

## 문제점

**파일**: `stm32cubeide/MC_lib/main/ros/src/ros_main.cpp:670`

Raw Odometry (`/odom`)가 **IMU의 절대 yaw 값**을 사용하고 있었습니다.
이것은 치명적인 버그로, 다음과 같은 문제를 야기했습니다:

1. **Raw Odom이 IMU에 의존**:
   - IMU yaw가 잘못되면 오도메트리 전체가 망가짐

2. **회전을 감지하지 못함**:
   - 360도 회전해도 yaw가 0.10°로 거의 변화 없음
   - 엔코더로 회전을 계산하지 않고 IMU에만 의존

3. **직진 시 좌표 계산 오류**:
   - IMU yaw가 진동하면 직진인데도 X, Y 위치가 틀어짐

## 해결 방법

**Differential Drive 오도메트리 공식 사용** (엔코더만 사용)

```
delta_theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION
```

---

## 수정 코드

### 파일: stm32cubeide/MC_lib/main/ros/src/ros_main.cpp

**함수**: `bool calcOdometry(double diff_time)` (642-693번 줄)

### 수정 전 (잘못된 코드)

```cpp
bool calcOdometry(double diff_time)
{
    float orientation[4];
    double wheel_l, wheel_r;
    double delta_s, theta, delta_theta;
    static double last_theta = 0.0;  // ← 삭제 필요
    double v, w;
    double step_time;

    wheel_l = wheel_r = 0.0;
    delta_s = delta_theta = theta = 0.0;
    v= w = 0;
    step_time = 0.0;

    step_time = diff_time;

    if (step_time == 0)
        return false;

    wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
    wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

    if(isnan(wheel_l))
        wheel_l = 0.0;
    if(isnan(wheel_r))
        wheel_r = 0.0;

    delta_s = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
    theta = DEG2RAD(__imu.data.e_yaw);  // ❌ 670번 줄: 삭제!

    delta_theta = theta - last_theta;    // ❌ 672번 줄: 삭제!

    // compute odometric pose
    odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
    odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
    odom_pose[2] += delta_theta;

    // compute odometric instantaneouse velocity

    v = delta_s / step_time;
    w = delta_theta / step_time;

    odom_vel[0] = v;
    odom_vel[1] = 0.0;
    odom_vel[2] = w;

    last_velocity[LEFT]  = wheel_l / step_time;
    last_velocity[RIGHT] = wheel_r / step_time;
    last_theta = theta;  // ❌ 690번 줄: 삭제!

    return true;
}
```

### 수정 후 (올바른 코드)

```cpp
bool calcOdometry(double diff_time)
{
    float orientation[4];
    double wheel_l, wheel_r;
    double delta_s, delta_theta;
    // static double last_theta = 0.0;  // ✅ 삭제됨
    double v, w;
    double step_time;

    wheel_l = wheel_r = 0.0;
    delta_s = delta_theta = 0.0;  // ✅ theta 제거
    v = w = 0;
    step_time = 0.0;

    step_time = diff_time;

    if (step_time == 0)
        return false;

    // Convert encoder ticks to radians
    wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
    wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

    if(isnan(wheel_l))
        wheel_l = 0.0;
    if(isnan(wheel_r))
        wheel_r = 0.0;

    // Calculate linear distance traveled (average of both wheels)
    delta_s = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;

    // ✅ Calculate rotation from wheel difference (encoder-based!)
    // Differential drive kinematics: delta_theta = (right - left) / separation
    delta_theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;

    // Compute odometric pose using differential drive kinematics
    odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
    odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
    odom_pose[2] += delta_theta;

    // Compute odometric instantaneous velocity
    v = delta_s / step_time;
    w = delta_theta / step_time;

    odom_vel[0] = v;
    odom_vel[1] = 0.0;
    odom_vel[2] = w;

    last_velocity[LEFT]  = wheel_l / step_time;
    last_velocity[RIGHT] = wheel_r / step_time;
    // last_theta = theta;  // ✅ 삭제됨

    return true;
}
```

---

## 변경 사항 요약

### 삭제된 코드

1. **645번 줄**: `static double last_theta = 0.0;` 변수 선언
2. **652번 줄**: `theta = 0.0;` 초기화
3. **670번 줄**: `theta = DEG2RAD(__imu.data.e_yaw);` IMU 사용
4. **672번 줄**: `delta_theta = theta - last_theta;` IMU 차분 계산
5. **690번 줄**: `last_theta = theta;` IMU 저장

### 추가된 코드

```cpp
// 669-671번 줄에 추가
// Calculate rotation from wheel difference (encoder-based!)
// Differential drive kinematics: delta_theta = (right - left) / separation
delta_theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;
```

---

## 적용 방법

### 1. 파일 백업

```bash
cd /home/anton/projects/connectwo-controller/stm32cubeide/MC_lib/main/ros/src
cp ros_main.cpp ros_main.cpp.backup_imu_odom
```

### 2. 파일 수정

```bash
nano ros_main.cpp
```

**수정할 부분**:
- 645번 줄: `static double last_theta = 0.0;` 주석 또는 삭제
- 652번 줄: `theta = 0.0;` 제거
- 670번 줄: `theta = DEG2RAD(__imu.data.e_yaw);` 주석 또는 삭제
- 672번 줄: `delta_theta = theta - last_theta;` 주석 또는 삭제
- 669-671번 줄에 추가:
  ```cpp
  delta_theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;
  ```
- 690번 줄: `last_theta = theta;` 주석 또는 삭제

### 3. 빌드 및 플래시

```bash
cd /home/anton/projects/connectwo-controller/stm32cubeide/motor_controller
rm -rf build
mkdir build
cd build
cmake ..
make
make flash
```

### 4. 테스트

```bash
# Pi400에서

# STM32 재부팅 (USB 뽑았다 꽂기 또는 리셋)

# rosserial 연결
roslaunch connectwo_bringup connectwo_core.launch

# 회전 테스트
rosbag record -O rotation_fixed.bag /odom /cmd_vel /tf

# J 키로 360도 회전
# Ctrl+C

# 분석
python3 ~/Downloads/analyze_odom_bag.py rotation_fixed.bag
```

**예상 결과**:
- Final Yaw: **약 360°** (또는 0°로 복귀)
- X, Y Position: 거의 0 근처 (제자리 회전)

---

## 이론적 배경

### Differential Drive 오도메트리 공식

4륜 차동 구동 로봇의 경우:

```
left_distance = WHEEL_RADIUS * (left_front_rad + left_rear_rad) / 2
right_distance = WHEEL_RADIUS * (right_front_rad + right_rear_rad) / 2

delta_s = (left_distance + right_distance) / 2        # 이동 거리
delta_theta = (right_distance - left_distance) / WHEEL_SEPARATION  # 회전각

x += delta_s * cos(theta + delta_theta/2)
y += delta_s * sin(theta + delta_theta/2)
theta += delta_theta
```

### 왜 IMU를 사용하면 안 되는가?

1. **Raw Odom의 정의**:
   - 엔코더만 사용한 순수한 오도메트리
   - EKF가 나중에 IMU와 융합

2. **센서 융합의 원리**:
   - Raw Odom (엔코더) → 위치 추정, 회전 드리프트 있음
   - IMU → 회전 정확, 위치 추정 불가
   - EKF → 두 센서의 장점을 결합

3. **IMU를 Raw Odom에 사용하면**:
   - Raw Odom이 이미 IMU에 오염됨
   - EKF가 융합할 독립적인 센서가 없음
   - IMU 오류가 오도메트리 전체를 망침

---

## 검증 방법

### 직진 테스트
```
예상: X 증가, Y ≈ 0, Yaw ≈ 0°
```

### 회전 테스트 (360도)
```
예상: X ≈ 0, Y ≈ 0, Yaw ≈ 360° (또는 0°)
```

### EKF 테스트
```
Raw Odom: 엔코더만 사용
IMU: 회전 정보 제공
EKF Filtered: 두 센서 융합

예상: EKF가 Raw Odom보다 정확
```

---

## 참고 자료

- [Differential Drive Kinematics](http://rossum.sourceforge.net/papers/DiffSteer/)
- [ROS Navigation Tuning Guide](http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom)
- [REP-105: Coordinate Frames](https://www.ros.org/reps/rep-0105.html)

---

**작성일**: 2025-10-30
**버전**: 1.0
**파일**: `/home/anton/projects/connectwo-controller/ODOM_FIX_PATCH.md`
