# 엔코더 방향 테스트 조합

## 테스트 절차

로봇을 **전진**시킬 때:
- ✅ 정상: odom position.x **증가**, angular.z **0 근처**
- ❌ 문제 1: position.x **감소** → 전체 부호 반전 필요
- ❌ 문제 2: angular.z **0이 아님** → 좌우 불균형 또는 swap

---

## 조합 1 (현재): Left(-), Right(+)

```cpp
left_front_tick  = -motor[0]
left_rear_tick   = -motor[1]
right_front_tick = +motor[2]
right_rear_tick  = +motor[3]
```

**테스트 결과:**
- [ ] position.x: 증가 / 감소
- [ ] angular.z: 0 근처 / 왼쪽회전 / 오른쪽회전

---

## 조합 2: Left(+), Right(-)

```cpp
left_front_tick  = +motor[0]
left_rear_tick   = +motor[1]
right_front_tick = -motor[2]
right_rear_tick  = -motor[3]
```

**테스트 결과:**
- [ ] position.x: 증가 / 감소
- [ ] angular.z: 0 근처 / 왼쪽회전 / 오른쪽회전

---

## 조합 3: 좌우 Swap + Left(-), Right(+)

```cpp
left_front_tick  = +motor[2]  // swap!
left_rear_tick   = +motor[3]
right_front_tick = -motor[0]
right_rear_tick  = -motor[1]
```

---

## 조합 4: 좌우 Swap + Left(+), Right(-)

```cpp
left_front_tick  = -motor[2]  // swap!
left_rear_tick   = -motor[3]
right_front_tick = +motor[0]
right_rear_tick  = +motor[1]
```

---

## 간단한 테스트 방법

### Pi400에서 실행:

```bash
# Terminal 1: odom 모니터링
rostopic echo /odom/pose/pose/position

# Terminal 2: 로봇 제어
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# 'i' 키로 천천히 전진:
# - position.x가 증가하는가?
# - position.y가 거의 0으로 유지되는가?
```

### XPS13에서 더 쉽게:

```bash
export ROS_MASTER_URI=http://192.168.1.18:11311
export ROS_IP=192.168.1.38

# 실시간 그래프
rqt_plot /odom/pose/pose/position/x /odom/pose/pose/position/y
```

**직진 시:**
- X는 증가 (직선)
- Y는 평평 (수평선)

**만약 Y가 증가/감소하면 = 회전하고 있음!**
