# AFT cold-start 120분 warm-up drift 실험

- 수행일: 2026-08-18~19
- 대상: 오른팔 `/aft_sensor2`
- 범위: AFT cold power-cycle 뒤 로봇 이동 없이 0/15/30/60/120분 wrench 측정

## 목적

[post-zero drift 실험](aft_post_zero_drift_and_tare_repeatability.md)에서 관찰된
Fz의 단조 감소가 AFT 전원 인가 후 thermal settling 때문에 재현되는지 확인한다.

## 조건과 한계

- robot 자세, tool/hand/cable 배치와 일반 V2 impedance controller를 유지했다.
- robot/hand 이동 명령은 발행하지 않았다.
- AFT driver 시작은 `2026-08-18 22:53:52 KST`였다.
- 실제 AFT 전원 ON 시각을 별도로 기록하지 못해 driver 시작을 대용값으로 썼다.
- zero-set은 driver 시작 약 9분 42초 후인 `23:03:34 KST`에 수행했다.
- t0 bag은 zero-set 약 28초 후 시작했으므로 power-on 직후 0~10분 transient는
  포함하지 않는다.

## 실행 명령

robot driver, 일반 V2 impedance controller와 hand driver는
[실행 명령](../command.md)에 따라 먼저 실행하고 기준 자세에서 정지시킨다. AFT를
cold start한 뒤 실제 전원 ON 시각을 운용 기록에 남기고 driver를 실행한다.

```bash
source /opt/ros/humble/setup.bash
source /home/vision/doosan_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0

ros2 launch aft_can_hardware aft_sensor.launch.py
```

500 Hz 운용 설정과 topic 상태를 확인한다.

```bash
ros2 topic info /aft_sensor2/sample_rate_setting --verbose
ros2 topic pub --once /aft_sensor2/sample_rate_setting \
  std_msgs/msg/Int32 "{data: 500}"

ros2 topic info /aft_sensor2/wrench --verbose
timeout 15s ros2 topic hz /aft_sensor2/wrench
```

정의한 power-on 경과시간에 기준 자세·정지·무접촉을 확인하고 hardware zero-set을
한 번 실행한다.

```bash
ros2 launch aft_can_hardware aft_zero_set2.launch.py \
  sensor_name:=aft_sensor2
```

추가 zero-set 없이 0/15/30/60/120분에 약 34초씩 기록한다. 아래 `t0`를 각 시점
이름으로 바꾼다.

```bash
export EXP_DIR=/home/vision/dualarm_ws/src/ft_fb_leaderarm/data/FT-20260808-01/cold_warmup_YYYYMMDD_HHMMSS
mkdir -p "${EXP_DIR}"

timeout --signal=INT 34s ros2 bag record \
  -o "${EXP_DIR}/t0" \
  /aft_sensor2/wrench /contact_state/observer_input
```

시점 사이에는 AFT, robot driver와 controller를 재시작하지 않고 robot을 움직이지
않는다. 각 기록 전후에 다음 상태를 확인한다.

```bash
ros2 topic echo /contact_state/observer_input --once
ros2 topic echo /aft_sensor2/wrench --once
```

## 결과

| 시점 | force median [N] | force std [N] | median force norm [N] | gate |
|---:|---|---|---:|---|
| 0분 | `[+0.04,-0.02,+0.30]` | `[0.123,0.172,0.258]` | 0.303 | PASS |
| 15분 | `[+0.12,0.00,+0.64]` | `[0.126,0.170,0.272]` | 0.651 | PASS |
| 30분 | `[+0.35,+0.06,+1.00]` | `[0.123,0.172,0.231]` | 1.061 | FAIL |
| 60분 | `[+0.38,+0.02,+0.01]` | `[0.128,0.169,0.259]` | 0.381 | PASS |
| 120분 | `[+0.39,-0.01,-0.73]` | `[0.129,0.171,0.233]` | 0.828 | PASS |

- t0→t120 force median 변화는 `[+0.35,+0.01,-1.03] N`이었다.
- Fz는 `[+0.30,+0.64,+1.00,+0.01,-0.73] N`으로 변했고 선형 기울기는
  `-0.01137 N/min`, `R²=0.6702`였다.
- 앞선 post-zero 실험의 단조 Fz 감소 `R²=0.9951`은 재현되지 않았다.
- 30분에서만 median force norm `1.061 N`으로 gate를 실패했다.
- 모든 구간에서 active command sample은 0이었다. 관절 자세 오차는 최대
  `0.282 deg`였다.

## 결론과 운용 해석

단순하고 결정적인 thermal warm-up만으로 앞선 drift를 설명할 근거는 약해졌다.
그러나 30분 gate 실패와 t0→t120 Fz `-1.03 N` 변화가 남아 있다. 따라서 현재
formal data 수집과 실기 모델 검증에서는 AFT cold start 후 120분 warm-up을 한 번
수행하는 보수적 절차를 사용한다. 같은 전원 session의 episode/zero-set마다 120분을
반복할 필요는 없다.

## Artifact와 관련 문제

- raw rosbag:
  `data/FT-20260808-01/cold_warmup_20260818_230334/{t0,t15,t30,t60,t120}`
- 각 bag topic: `/aft_sensor2/wrench`, `/contact_state/observer_input`
- 전체 6축 결과와 한계:
  [FT-20260808-01의 cold power-cycle 절](../problem/FT-20260808-01.md#2026-08-1819-aft-cold-power-cycle-warm-up-재실험)
- 기준 자세 문제:
  [FT-20260808-03](../problem/FT-20260808-03.md)
- 실제 sensor rate 계약:
  [FT-20260808-04](../problem/FT-20260808-04.md)

