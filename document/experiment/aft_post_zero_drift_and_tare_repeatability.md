# AFT post-zero 120분 drift와 zero-set 10회 반복 실험

- 수행일: 2026-08-18
- 대상: 오른팔 `/aft_sensor2`
- 범위: 로봇 이동 명령 없이 같은 tare의 시간 drift와 독립 tare 반복성 평가

## 목적

이미 켜져 있던 AFT를 기준 자세에서 hardware zero-set한 뒤, 같은 tare의 중심값이
120분 동안 얼마나 변하는지 측정한다. 이어서 같은 조건의 hardware zero-set을
10회 반복하여 tare별 offset 변동이 free-space 모델의 최대 오차 `1 N` 목표보다
충분히 작은지 확인한다.

이 실험은 AFT cold power-on 실험이 아니다. AFT는 t=0 전에 이미 켜져 있었다.

## 조건

- source commit: `384ed565226eb2877d0180e3dda8d85cb7386f96`
- 기준 자세: `[5.5, 52.0, 112.0, 28.0, -107.0, -35.0] deg`
- robot driver, 일반 V2 impedance controller, hand driver와 AFT driver를 계속
  실행했다.
- tool, payload, hand, cable 배치를 유지했다.
- 모든 측정에서 `has_active_command=false`; robot/hand 이동 명령은 없었다.
- 07:12:20 KST의 hardware zero-set 완료를 t=0으로 사용했다.

## 실행 명령

장비 전체 실행은 [실행 명령](../command.md)의 2~4절을 따른다. hand driver를
같은 상태로 유지해야 할 때는 별도 terminal에서 다음 launch를 사용한다.

```bash
source /opt/ros/humble/setup.bash
source /home/vision/dualarm_ws/install/setup.bash
ros2 launch aidin_hand_controllers joint_position_controller.launch.py
```

AFT를 500 Hz 운용 계약으로 설정하고 기준 자세를 확인한다.

```bash
ros2 topic pub --once /aft_sensor2/sample_rate_setting \
  std_msgs/msg/Int32 "{data: 500}"

ros2 topic echo /contact_state/observer_input --once
ros2 topic echo /aft_sensor2/wrench --once
```

기준 자세·정지·무접촉 확인 후 t=0 zero-set을 한 번 실행한다.

```bash
ros2 launch aft_can_hardware aft_zero_set2.launch.py \
  sensor_name:=aft_sensor2
```

같은 tare를 유지한 채 0/15/30/60/120분에 각각 약 30초 기록한다. 아래 `t0`를
각 시점 이름으로 바꾸며, 중간에 zero-set이나 driver 재시작을 하지 않는다.

```bash
export EXP_DIR=/home/vision/.ros/ft_fb_leaderarm/experiments/aft_post_zero_120m
mkdir -p "${EXP_DIR}"

timeout --signal=INT 31s ros2 bag record \
  -o "${EXP_DIR}/t0" \
  /aft_sensor2/wrench /contact_state/observer_input
```

120분 측정 후에는 다음 두 명령을 `tare01`부터 `tare10`까지 운용자가 10회
수동 반복한다. 매회 기준 자세·정지·무접촉을 다시 확인하며 loop로 자동화하지 않는다.

```bash
ros2 launch aft_can_hardware aft_zero_set2.launch.py \
  sensor_name:=aft_sensor2

sleep 3
timeout --signal=INT 6s ros2 bag record \
  -o "${EXP_DIR}/tare01" \
  /aft_sensor2/wrench /contact_state/observer_input
```

## 같은 tare의 120분 결과

| 경과 | force median [N] | force std [N] | median force norm [N] | gate |
|---:|---|---|---:|---|
| 0분 | `[-0.03,-0.07,+0.22]` | `[0.121,0.164,0.280]` | 0.233 | PASS |
| 15분 | `[+0.04,-0.04,-0.06]` | `[0.123,0.176,0.202]` | 0.082 | PASS |
| 30분 | `[+0.07,-0.05,-0.45]` | `[0.123,0.170,0.266]` | 0.458 | PASS |
| 60분 | `[+0.35,0.00,-1.13]` | `[0.122,0.177,0.209]` | 1.183 | FAIL |
| 120분 | `[+0.60,-0.01,-2.19]` | `[0.121,0.172,0.196]` | 2.271 | FAIL |

- t=0→120분 force median 변화는 `[+0.63,+0.06,-2.41] N`이다.
- Fz 선형 drift는 `-0.02028 N/min`, `R²=0.9951`이었다.
- force STD는 전 구간 `0.40 N` 이내였다. 60/120분 실패 원인은 noise 증가가
  아니라 median offset 증가였다.
- 시작·종료 관절 차이는 최대 약 `0.04 deg`였다.

## 120분 후 zero-set 10회 결과

| 회 | force median [N] | force std [N] | median force norm [N] | gate |
|---:|---|---|---:|---|
| 1 | `[+0.18,-0.20,+0.56]` | `[0.124,0.175,0.223]` | 0.621 | PASS |
| 2 | `[+0.07,-0.05,-0.45]` | `[0.130,0.176,0.258]` | 0.458 | PASS |
| 3 | `[+0.11,+0.02,+0.61]` | `[0.124,0.174,0.251]` | 0.620 | PASS |
| 4 | `[+0.06,+0.01,+0.44]` | `[0.112,0.158,0.203]` | 0.444 | PASS |
| 5 | `[-0.03,-0.03,-0.38]` | `[0.122,0.171,0.238]` | 0.382 | PASS |
| 6 | `[+0.04,-0.14,-0.01]` | `[0.122,0.173,0.205]` | 0.146 | PASS |
| 7 | `[-0.05,0.00,-0.25]` | `[0.118,0.167,0.225]` | 0.255 | PASS |
| 8 | `[+0.09,-0.04,+0.56]` | `[0.110,0.159,0.200]` | 0.569 | PASS |
| 9 | `[-0.08,+0.03,-0.70]` | `[0.123,0.170,0.246]` | 0.705 | PASS |
| 10 | `[+0.10,+0.16,+0.57]` | `[0.117,0.175,0.253]` | 0.600 | PASS |

- 10회 force median의 평균/회차 간 STD는
  `[+0.049,-0.024,+0.095] / [0.076,0.093,0.482] N`이었다.
- force median의 peak-to-peak는 `[0.26,0.36,1.31] N`이었다.
- 현행 zero gate는 10회 모두 통과했지만 Fz tare 반복 범위 `1.31 N`은 모델의
  전체 최대 오차 목표 `1 N`보다 크다.

## 결론

이 session에서는 정지 noise 증가보다 zero 중심의 시간 drift가 지배적이었다.
또한 zero gate 통과가 tare 간 `1 N` 이내 반복성을 보장하지 않았다. 다만 AFT가
이미 켜진 상태에서 시작했으므로 원인을 thermal warm-up 하나로 확정할 수 없으며,
cable strain, 구조 하중과 controller 영향도 분리되지 않았다.

## Artifact와 관련 문제

- 2026-08-18 결과는 live ROS stream 통계로 기록했고 raw 파일은 보존하지 못했다.
- 전체 6축 수치와 해석:
  [FT-20260808-01의 post-zero 절](../problem/FT-20260808-01.md#2026-08-18-post-zero-장시간-drift와-tare-10회-재실험)
- zero-set 종료 문제와 `aft_zero_set2`:
  [FT-20260808-02](../problem/FT-20260808-02.md)
- 기준 자세 불일치 방지:
  [FT-20260808-03](../problem/FT-20260808-03.md)

