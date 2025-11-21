# EKF Tuning - Next Task

## 현재 상태
- Odometry calibration 완료 (실측 기반)
- IMU yaw discontinuity 처리 완료
- Odometry 발행 주기: 20Hz
- 단순 환경: 매핑 잘 됨
- 복잡 환경/고속 이동: 여전히 약간의 맵 왜곡 존재

## EKF 설정 파일 위치
- Launch: `~/catkin_ws/src/connectwo_bringup/launch/connectwo_ekf.launch`

## 튜닝해야 할 항목

### 1. Odometry Covariance (STM32 코드)
**파일:** `stm32cubeide/MC_lib/main/ros/src/ros_main.cpp:536-549`

**현재 문제:**
```cpp
// 고정 covariance (속도와 무관)
odom.pose.covariance[0] = 0.01;   // x
odom.pose.covariance[35] = 0.05;  // yaw
```

**개선 방향:**
- 속도에 비례하는 동적 covariance
- 빠르게 이동 시 불확실성 증가
- 정지/저속 시 불확실성 감소

**예시 코드:**
```cpp
double speed = sqrt(odom_vel[0]*odom_vel[0] + odom_vel[2]*odom_vel[2]);
double base_pos_var = 0.01;
double base_yaw_var = 0.05;
odom.pose.covariance[0] = base_pos_var * (1.0 + speed);
odom.pose.covariance[35] = base_yaw_var * (1.0 + fabs(odom_vel[2]));
```

### 2. IMU Covariance 검증
**파일:** `stm32cubeide/MC_lib/main/ros/src/ros_main.cpp:400-413`

**현재 설정:**
```cpp
imu_msg.orientation_covariance[0] = 0.01;  // roll
imu_msg.orientation_covariance[4] = 0.01;  // pitch
imu_msg.orientation_covariance[8] = 0.01;  // yaw
imu_msg.angular_velocity_covariance[8] = 0.1;  // vyaw (noisy!)
```

**확인 사항:**
- IMU 정지 상태에서 분산 측정
- 회전 중 각속도 분산 측정
- 실측값과 설정값 비교

### 3. EKF Process Noise Tuning
**파일:** `~/catkin_ws/src/connectwo_bringup/launch/connectwo_ekf.launch:67-81`

**현재 설정:**
```yaml
process_noise_covariance:
  - x:  0.05
  - y:  0.05
  - yaw: 0.06
  - vx: 0.025
  - vy: 0.025
  - vyaw: 0.02
```

**튜닝 방향:**
- x, y: odometry drift rate 반영
- yaw: 회전 시 드리프트 고려
- vyaw: IMU gyro 신뢰도 반영

### 4. EKF 센서 가중치 조정
**파일:** `connectwo_ekf.launch:36-50`

**현재 IMU 설정:**
```yaml
imu0_config: 
  # ONLY vyaw (angular velocity z)
  [false, false, false,  # x, y, z
   false, false, false,  # roll, pitch, yaw
   false, false, false,  # vx, vy, vz
   false, false, true,   # vroll, vpitch, vyaw ← 오직 이것만 사용
   false, false, false]  # ax, ay, az
```

**고려사항:**
- IMU orientation (yaw)도 사용할지 검토
- 현재는 각속도만 사용 중

## 테스트 시나리오

### 1. 정지 상태 테스트
```bash
# IMU, Odometry variance 측정
rostopic echo /imu | tee imu_static.log
rostopic echo /odom | tee odom_static.log
# 1분간 기록 후 분산 계산
```

### 2. 저속 이동 테스트
```bash
# 0.1 m/s로 직진
rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
# Odometry vs Ground Truth 비교
```

### 3. 고속 회전 테스트
```bash
# 빠른 회전
rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: 1.5}}'
# 360도 회전 후 실제 각도 측정
```

### 4. 복합 경로 테스트
```bash
# Gmapping으로 복잡한 환경 매핑
roslaunch connectwo_bringup connectwo_slam.launch slam_methods:=gmapping_ekf
# 맵 왜곡 정도 측정
```

## 참고 자료

### Robot Localization 문서
- http://docs.ros.org/en/noetic/api/robot_localization/html/state_estimation_nodes.html
- Covariance tuning guide

### 디버깅 도구
```bash
# EKF 출력 확인
rostopic echo /odometry/filtered

# 센서 업데이트 주파수 확인
rostopic hz /odom
rostopic hz /imu
rostopic hz /odometry/filtered

# TF 확인
rosrun tf tf_echo odom base_footprint
```

## 작업 우선순위

1. **동적 Odometry Covariance 구현** (가장 효과적)
2. IMU Covariance 실측 및 보정
3. EKF Process Noise 미세 조정
4. 전체 시스템 통합 테스트

## 예상 소요 시간
- Covariance 구현: 1-2시간
- 실측 및 테스트: 2-3시간
- 전체 튜닝: 4-6시간
