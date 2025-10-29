# EKF 센서 융합 문제 해결 가이드

## 📋 문제 요약

EKF (Extended Kalman Filter)로 엔코더 오도메트리(`/odom`)와 IMU(`/imu`)를 융합한 결과(`/odometry/filtered`)가 Raw Odom과 큰 차이를 보이고 있습니다.

### 분석 결과 (2025-10-29)

```
🎯 Final Position:
  Raw Odom:     x=  0.244m, y= -0.057m
  EKF Filtered: x=  0.244m, y= -0.057m
  Difference:   Δx=  0.00cm, Δy=  0.00cm  ✅ 위치는 정상

🔄 Final Yaw:
  Raw Odom:     -20.56°
  EKF Filtered:  13.57°
  Difference:    34.13°  ❌ 매우 큰 차이!

📐 Average Errors Over Time:
  Avg X Error:    5.59cm (± 9.76cm)
  Avg Y Error:    1.03cm (± 1.83cm)
  Avg Yaw Error:  36.69°  (± 8.63°)  ❌ 평균 36도 차이!

📏 Total Distance Traveled:
  Raw Odom:     0.519m
  EKF Filtered: 0.685m
  Difference:   16.59cm (32% 차이!)  ❌
```

---

## 🔍 근본 원인 분석

### 1. IMU Angular Velocity (g_z) 사용 문제 ✅ **해결됨**

**초기 문제**:
```yaml
# 잘못된 설정
<rosparam param="imu0_config">[false, false, false,
                                true,  true,  true,      # roll, pitch, yaw
                                false, false, false,
                                false, false, true,       # vyaw (g_z) 사용! ← 문제
                                false, false, false]</rosparam>
```

**해결**:
```yaml
# 수정된 설정
<rosparam param="imu0_config">[false, false, false,
                                false, false, true,       # yaw만 사용
                                false, false, false,
                                false, false, false,      # vyaw 사용 안 함 ← 수정
                                false, false, false]</rosparam>
```

**결과**: 여전히 Yaw 차이가 34도로 큼 → IMU orientation 자체에 문제가 있음

---

### 2. IMU Orientation (Yaw) 자체가 부정확 ❌ **미해결**

EKF가 IMU의 **orientation.z (yaw)만** 사용하도록 설정했는데도 차이가 크다는 것은:
- IMU가 보고하는 **절대 방향(yaw)**이 엔코더 기반 오도메트리와 **완전히 다름**
- 가능한 원인:
  1. **IMU 캘리브레이션 문제**
  2. **IMU 좌표계가 반대** (거꾸로 장착되었거나)
  3. **STM32 펌웨어에서 yaw 계산 오류**
  4. **자기 간섭** (주변 전자기기나 모터의 영향)

---

## 🛠️ 해결 방법

### 방법 1: IMU 없이 EKF 실행 (임시 해결책)

IMU가 오히려 방해가 된다면, 일단 제거하고 Raw Odom만 사용합니다.

#### 1-1. EKF 설정 수정

```bash
# connectwo_bringup 패키지로 이동
cd ~/catkin_ws/src/connectwo_bringup/launch

# 백업 생성
cp connectwo_ekf.launch connectwo_ekf.launch.backup

# 편집
nano connectwo_ekf.launch
```

다음과 같이 수정:

```xml
<!-- Published topics -->
<param name="odom0" value="/odom"/>
<!-- IMU 비활성화 -->
<!-- <param name="imu0" value="/imu"/> -->

<!-- IMU configuration 주석 처리 -->
<!--
<rosparam param="imu0_config">[false, false, false,
                                false, false, true,
                                false, false, false,
                                false, false, false,
                                false, false, false]</rosparam>
-->

<!-- IMU 관련 파라미터도 주석 처리 -->
<!--
<param name="imu0_differential" value="false"/>
<param name="imu0_relative" value="false"/>
<param name="imu0_remove_gravitational_acceleration" value="true"/>
-->
```

#### 1-2. 재테스트

```bash
# EKF 노드 재시작
rosnode kill /ekf_localization
roslaunch connectwo_bringup connectwo_ekf.launch &

# 새로운 bag 녹화
rosbag record -O odom_no_imu_$(date +%Y%m%d_%H%M%S).bag \
  /odom \
  /odometry/filtered \
  /cmd_vel \
  /tf

# 키보드로 로봇 제어 (직진, 회전, 사각형 주행)
# ...

# Ctrl+C로 녹화 종료

# 분석
python3 analyze_odom_bag.py odom_no_imu_*.bag
```

**기대 결과**: Raw Odom과 Filtered가 **거의 동일**해야 함 (차이 < 1cm, < 1°)

---

### 방법 2: IMU Yaw 값 직접 확인 및 디버깅

#### 2-1. 실시간 비교

```bash
# Pi400에서 터미널 2개 열기

# 터미널 1: Raw Odom의 yaw
rostopic echo /odom | grep -A 4 "orientation:"

# 터미널 2: IMU의 yaw
rostopic echo /imu | grep -A 4 "orientation:"

# 로봇을 천천히 360도 회전시키면서 비교
# - Raw Odom의 yaw: 0° → 90° → 180° → 270° → 360° (0°)
# - IMU의 yaw: 같은 패턴이어야 함
```

#### 2-2. Quaternion을 Degree로 변환해서 확인

```bash
# 간단한 Python 스크립트
rostopic echo /odom /imu | python3 -c "
import sys
import math

def quat_to_yaw(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))

for line in sys.stdin:
    if 'orientation:' in line:
        next(sys.stdin)  # x
        next(sys.stdin)  # y
        z_line = next(sys.stdin)
        w_line = next(sys.stdin)
        z = float(z_line.split(':')[1])
        w = float(w_line.split(':')[1])
        x, y = 0, 0  # 2D robot
        print(f'Yaw: {quat_to_yaw(x, y, z, w):.2f}°')
"
```

#### 2-3. 문제 패턴 확인

| 현상 | 원인 | 해결 방법 |
|------|------|----------|
| IMU yaw가 반대 방향 (좌회전 시 감소) | 좌표계가 반대 | STM32 펌웨어에서 yaw 부호 변경 |
| IMU yaw가 2배 빠름 | 단위 변환 오류 | 라디안/도 변환 확인 |
| IMU yaw가 드리프트 (계속 증가/감소) | 자이로 바이어스 | IMU 캘리브레이션 필요 |
| IMU yaw가 불규칙하게 점프 | 자기 간섭 | 모터/전원과 IMU 거리 확보 |

---

### 방법 3: STM32 펌웨어에서 IMU 데이터 확인

#### 3-1. IMU Raw 데이터 출력

STM32 펌웨어에서 IMU의 실제 값을 확인합니다.

**파일**: `/home/anton/projects/connectwo-controller-build/stm32cubeide/MC_lib/main/ros/src/ros_main.cpp`

```cpp
void publishImuMsg(void)
{
    // 디버깅: Raw IMU 데이터 출력 (printf 사용 시 rosserial 버퍼 오버플로우 주의!)
    // printf("IMU: roll=%.2f, pitch=%.2f, yaw=%.2f\n",
    //        __imu.data.e_roll, __imu.data.e_pitch, __imu.data.e_yaw);

    // ... 기존 코드 ...
}
```

#### 3-2. Yaw 부호 변경 테스트

만약 IMU yaw가 반대 방향이라면:

```cpp
void publishImuMsg(void)
{
    imu_msg.header.stamp = rosNow();
    imu_msg.header.frame_id = imu_frame_id;

    float roll__ = DEG2RAD(__imu.data.e_roll);
    float pitch__ = DEG2RAD(__imu.data.e_pitch);
    float yaw__ = DEG2RAD(-__imu.data.e_yaw);  // ← 부호 변경!

    // ... 나머지 코드 ...
}
```

---

## 📊 분석 도구 사용법

### analyze_odom_bag.py

Raw Odom과 EKF Filtered를 비교 분석하는 Python 스크립트입니다.

#### 위치
```
/home/anton/Downloads/analyze_odom_bag.py
```

#### 사용법
```bash
# Pi400에서 실행 (ROS1 환경)
python3 analyze_odom_bag.py <bagfile>

# 예시
python3 analyze_odom_bag.py odom_test_20251029_174926.bag
```

#### 출력
- **터미널**: 상세 통계 분석
- **PNG 파일**: 4개의 비교 그래프
  - X Position vs Time
  - Y Position vs Time
  - Yaw vs Time
  - 2D Trajectory

#### 그래프 전송 (Pi400 → XPS)
```bash
scp odom_test_*_analysis.png anton@anton-XPS-13-9350:~/Downloads/
```

---

## ✅ 성공 기준

### 정상적인 EKF 융합 결과

```
🎯 Final Position:
  Difference:   Δx < 5cm, Δy < 5cm  ✅

🔄 Final Yaw:
  Difference:   < 5°  ✅

📐 Average Errors Over Time:
  Avg X Error:   < 5cm  ✅
  Avg Y Error:   < 5cm  ✅
  Avg Yaw Error: < 5°   ✅

📏 Total Distance Traveled:
  Difference:   < 5% of total distance  ✅
```

---

## 🔧 추가 튜닝 (고급)

### Covariance 값 조정

만약 센서 신뢰도를 조정하고 싶다면:

#### STM32 펌웨어 (ros_main.cpp)

```cpp
// IMU를 덜 신뢰하려면 covariance 값을 증가
imu_msg.orientation_covariance[8] = 0.1;  // yaw (기본: 0.01)

// Odometry를 더 신뢰하려면 covariance 값을 감소
odom.pose.covariance[35] = 0.01;  // yaw (기본: 0.05)
```

#### EKF 설정 (connectwo_ekf.launch)

```xml
<!-- Process noise covariance: 높을수록 센서를 덜 신뢰 -->
<rosparam param="process_noise_covariance">
  [0.05, 0, 0, 0, 0, 0, ...]  <!-- x, y, z, roll, pitch, yaw, ... -->
</rosparam>
```

---

## 📝 테스트 시나리오

### 1. 직진 테스트 (1m)
```bash
# 목적: 위치 정확도 확인
# 조작: 1m 직진
# 확인: Raw vs Filtered 차이 < 5cm
```

### 2. 제자리 회전 테스트 (360°)
```bash
# 목적: Yaw 정확도 확인
# 조작: 360도 제자리 회전
# 확인:
#   - Raw Odom yaw: 360° → 0° 복귀
#   - Filtered yaw: 360° → 0° 복귀
#   - 차이 < 5°
```

### 3. 사각형 주행 테스트
```bash
# 목적: 누적 오차 확인
# 조작: 1m × 1m 사각형 주행 (4번 회전)
# 확인:
#   - 원점 복귀 오차 < 10cm
#   - Filtered가 Raw보다 원점에 가까움
```

---

## 🐛 알려진 문제

### 1. Pi400에서 그래프 표시 불가
```
Unable to init server: Could not connect: Connection refused
Gdk-CRITICAL: gdk_cursor_new_for_display: assertion 'GDK_IS_DISPLAY (display)' failed
```

**원인**: SSH로 접속한 환경에서 GUI 없음

**해결**: PNG 파일은 정상 생성되므로, XPS로 복사해서 확인
```bash
scp *.png anton@anton-XPS-13-9350:~/Downloads/
```

### 2. ROS1/ROS2 충돌 (XPS Ubuntu 22.04)
```
ImportError: cannot import name 'Log' from 'rosgraph_msgs.msg'
```

**원인**: ROS2 Humble이 환경변수에 설정되어 있음

**해결**: Pi400에서 실행하거나, XPS에서 ROS1 환경 설정
```bash
unset ROS_DISTRO
unset AMENT_PREFIX_PATH
source /opt/ros/noetic/setup.bash
```

---

## 📚 참고 자료

### ROS 문서
- [REP-105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
- [REP-145: Conventions for IMU Sensor Drivers](https://www.ros.org/reps/rep-0145.html)
- [robot_localization Documentation](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html)
- [robot_localization Covariance Tuning](http://docs.ros.org/en/noetic/api/robot_localization/html/preparing_sensor_data.html)

### 관련 파일
- **STM32 펌웨어**: `/home/anton/projects/connectwo-controller-build/stm32cubeide/MC_lib/main/ros/src/ros_main.cpp`
- **EKF 설정**: `/home/anton/projects/connectwo_bringup/launch/connectwo_ekf.launch`
- **분석 스크립트**: `/home/anton/Downloads/analyze_odom_bag.py`
- **Compliance 문서**: `/home/anton/projects/connectwo-controller-build/ROS_MESSAGE_COMPLIANCE_CHECK.md`

---

## 🎯 다음 단계

1. **IMU 없이 EKF 테스트** (방법 1)
   - 가장 빠르게 문제 격리 가능
   - Raw Odom만으로도 SLAM 가능

2. **IMU 디버깅** (방법 2)
   - Yaw 값 직접 비교
   - 좌표계 확인

3. **IMU 캘리브레이션**
   - 필요하다면 IMU 제조사 도구 사용
   - STM32 펌웨어에서 보정값 적용

4. **SLAM 맵핑 테스트**
   - EKF 튜닝 완료 후
   - gmapping 또는 cartographer로 맵 생성
   - 벽 중복 현상 확인

---

**작성일**: 2025-10-29
**업데이트**: 2025-10-29
