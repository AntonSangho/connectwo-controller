# ROS 메시지 규격 준수 검사 결과

## 요약

STM32 펌웨어의 ROS 메시지는 **기본 데이터는 올바르지만, Covariance 행렬이 누락**되어 있습니다.

---

## 1. Odometry 메시지 (nav_msgs/Odometry)

### ✅ 준수 항목
- frame_id: `odom` (REP-105 준수)
- child_frame_id: `base_footprint` (REP-105 준수)
- timestamp: `nh.now()` 사용
- position (x, y, z): 미터 단위, z=0 (2D)
- orientation: 쿼터니언 변환 정상
- twist (linear.x, angular.z): m/s, rad/s

### ❌ 누락 항목
- **pose.covariance[36]**: 위치/방향 불확실성 (6x6 행렬)
- **twist.covariance[36]**: 속도/각속도 불확실성 (6x6 행렬)

### 영향
- robot_localization 같은 센서 융합 노드가 불확실성을 알 수 없음
- 엔코더 슬립 시 오차가 커도 EKF가 이를 보정하지 못함

### 코드 위치
`/home/anton/projects/connectwo-controller-build/stm32cubeide/MC_lib/main/ros/src/ros_main.cpp:505-517`

---

## 2. IMU 메시지 (sensor_msgs/Imu)

### ✅ 준수 항목
- frame_id: `imu_link`
- timestamp: `nh.now()` 사용
- orientation: Roll, Pitch, Yaw → 쿼터니언 수동 계산 (정확함)
- angular_velocity: g_x, g_y, g_z (rad/s)
- linear_acceleration: a_x, a_y, a_z (m/s²)

### ❌ 누락 항목
- **orientation_covariance[9]**: 방향 불확실성 (3x3 행렬)
- **angular_velocity_covariance[9]**: 각속도 불확실성 (3x3 행렬)
- **linear_acceleration_covariance[9]**: 가속도 불확실성 (3x3 행렬)

### 영향
- EKF가 IMU 노이즈 수준을 모름
- 노이즈가 많은 angular_velocity를 너무 신뢰 → `/odometry/filtered` 불안정

### 코드 위치
`/home/anton/projects/connectwo-controller-build/stm32cubeide/MC_lib/main/ros/src/ros_main.cpp:365-401`

---

## 3. TF (Transform)

### ✅ 완벽하게 준수
- odom → base_footprint 변환 정상
- Timestamp Odometry와 동기화
- REP-105 완전 준수

### 코드 위치
`/home/anton/projects/connectwo-controller-build/stm32cubeide/MC_lib/main/ros/src/ros_main.cpp:522-530`

---

## 4. JointState 메시지

### ⚠️ 비활성화됨
- rosserial 버퍼 오버플로우 방지를 위해 퍼블리싱 중단
- 맵핑에는 영향 없음 (Odometry만 필요)

---

## 해결 방법

### 방법 1: Odometry Covariance 추가 (권장)

```cpp
void updateOdometry(void)
{
    odom.header.frame_id = odom_header_frame_id;
    odom.child_frame_id  = odom_child_frame_id;

    odom.pose.pose.position.x = odom_pose[0];
    odom.pose.pose.position.y = odom_pose[1];
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

    odom.twist.twist.linear.x  = odom_vel[0];
    odom.twist.twist.angular.z = odom_vel[2];

    // Pose covariance (6x6 행렬, row-major 순서)
    // [x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis]
    // 2D 로봇이므로 x, y, yaw만 설정
    odom.pose.covariance[0] = 0.01;   // x 분산 (1cm 오차)
    odom.pose.covariance[7] = 0.01;   // y 분산
    odom.pose.covariance[14] = 1e6;   // z 미사용
    odom.pose.covariance[21] = 1e6;   // roll 미사용
    odom.pose.covariance[28] = 1e6;   // pitch 미사용
    odom.pose.covariance[35] = 0.05;  // yaw 분산 (~13도 오차)

    // Twist covariance
    odom.twist.covariance[0] = 0.01;   // vx 분산
    odom.twist.covariance[7] = 1e6;    // vy 미사용
    odom.twist.covariance[14] = 1e6;   // vz 미사용
    odom.twist.covariance[21] = 1e6;   // 회전 속도 x 미사용
    odom.twist.covariance[28] = 1e6;   // 회전 속도 y 미사용
    odom.twist.covariance[35] = 0.05;  // 각속도 z 분산
}
```

### 방법 2: IMU Covariance 추가

```cpp
void publishImuMsg(void)
{
    // ... (기존 코드) ...

    // Orientation covariance (3x3 행렬)
    imu_msg.orientation_covariance[0] = 0.01;  // roll
    imu_msg.orientation_covariance[4] = 0.01;  // pitch
    imu_msg.orientation_covariance[8] = 0.01;  // yaw (안정적)

    // Angular velocity covariance (3x3 행렬)
    imu_msg.angular_velocity_covariance[0] = 0.02;  // g_x
    imu_msg.angular_velocity_covariance[4] = 0.02;  // g_y
    imu_msg.angular_velocity_covariance[8] = 0.1;   // g_z (노이즈 많음!)

    // Linear acceleration covariance (3x3 행렬)
    imu_msg.linear_acceleration_covariance[0] = 0.05;
    imu_msg.linear_acceleration_covariance[4] = 0.05;
    imu_msg.linear_acceleration_covariance[8] = 0.05;

    imu_pub.publish(&imu_msg);
}
```

---

## Covariance 값 튜닝 가이드

### 작은 값 (예: 0.01)
- "이 센서는 매우 정확함"
- EKF가 이 센서를 더 신뢰

### 큰 값 (예: 0.1)
- "이 센서는 노이즈가 많음"
- EKF가 이 센서를 덜 신뢰

### 매우 큰 값 (예: 1e6)
- "이 데이터는 사용하지 마세요"
- 2D 로봇의 z, roll, pitch 같은 미사용 축

---

## 권장 사항

1. **최소한 Odometry Covariance 추가** (방법 1)
   - 이것만으로도 EKF 성능이 크게 향상됨
   - 특히 `pose.covariance[35]` (yaw)가 중요

2. **IMU Covariance도 추가** (방법 2)
   - `angular_velocity_covariance[8]`를 크게 설정 (0.1)
   - g_z 노이즈가 많다는 것을 EKF에 알림

3. **실험적 튜닝**
   - 초기값으로 시작 후 맵핑 테스트
   - 벽이 여전히 중복되면 값을 조정

---

## 참고 자료

- [REP-105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
- [REP-145: Conventions for IMU Sensor Drivers](https://www.ros.org/reps/rep-0145.html)
- [nav_msgs/Odometry](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html)
- [sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)
- [robot_localization Covariance 튜닝](http://docs.ros.org/en/noetic/api/robot_localization/html/preparing_sensor_data.html)
