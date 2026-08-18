# AFT 정지 noise와 초기 zero-set 반복성 실험

- 수행일: 2026-08-08, 추가 확인 2026-08-11
- 대상: 오른팔 `/aft_sensor2`
- 범위: 로봇 이동 명령 없이 고정 자세에서 AFT wrench만 평가

## 목적

고정 자세에서 AFT의 정지 noise가 어느 정도인지, 같은 조건의 hardware zero-set을
반복했을 때 wrench 중심값이 얼마나 달라지는지 확인한다. 이 결과로 collector의
zero gate와 free-space 모델의 `1 N` 오차 목표가 현실적인지 판단한다.

## 조건과 안전 원칙

- 기준 자세는 `[5.5, 52.0, 112.0, 28.0, -107.0, -35.0] deg`다.
- tool, payload, hand 상태와 AFT cable 배치를 바꾸지 않는다.
- robot driver와 일반 V2 impedance controller는 자세 유지용으로 실행했지만
  robot/hand 이동 명령은 발행하지 않았다.
- zero-set은 기준 자세, 완전 정지, 완전 무접촉을 확인한 뒤 운용자가 매회 직접
  실행한다. 반복 zero-set을 shell loop로 자동화하지 않는다.

장비 구동 명령은 [실행 명령](../command.md#2-sbc-robot-driver)의 robot driver,
V2 impedance controller, AFT 절을 따른다. AFT 전용 terminal의 공통 환경과 실행
명령은 다음과 같다.

```bash
source /opt/ros/humble/setup.bash
source /home/vision/doosan_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0

ros2 launch aft_can_hardware aft_sensor.launch.py
```

## 실험 방법과 명령

먼저 topic, 기준 자세와 정지를 읽기 전용으로 확인한다.

```bash
ros2 topic info /aft_sensor2/wrench --verbose
ros2 topic echo /contact_state/observer_input --once
timeout 15s ros2 topic hz /aft_sensor2/wrench
```

각 독립 tare에서 다음 zero-set2를 한 번 실행한다.

```bash
ros2 topic info /aft_sensor2/bias_setting --verbose
ros2 launch aft_can_hardware aft_zero_set2.launch.py \
  sensor_name:=aft_sensor2
```

정상 완료 로그는 다음과 같다.

```text
Hardware bias (tare) requested and acknowledged
Zero set completed. Exiting after callback.
```

zero-set 후 최소 3초 기다린 다음 5초 raw wrench를 기록한다. `tare01`을
`tare02`, `tare03`으로 바꿔 총 3회 수행한다.

```bash
export EXP_DIR=/home/vision/.ros/ft_fb_leaderarm/experiments/aft_stationary_zero
mkdir -p "${EXP_DIR}"

timeout --signal=INT 5s ros2 bag record \
  -o "${EXP_DIR}/tare01" \
  /aft_sensor2/wrench /contact_state/observer_input
```

같은 tare를 유지한 장기 정지 noise는 zero-set을 추가하지 않고 30초 또는 60초
기록한다.

```bash
timeout --signal=INT 60s ros2 bag record \
  -o "${EXP_DIR}/same_tare_60s" \
  /aft_sensor2/wrench /contact_state/observer_input
```

collector가 실행 중이라면 최근 1초 gate 통계도 확인한다.

```bash
ros2 topic echo /ft_free_space_collector/diagnostics --once
```

판정 기준은 force median vector norm `<= 1.0 N`, 각 force 축 STD
`<= 0.40 N`이다.

## 결과

| 조건 | force median [N] | force std [N] | force norm p95/p99/max [N] |
|---|---|---|---|
| 30초 정지 | `[+0.27,+0.20,+0.03]` | `[0.127,0.175,0.235]` | `0.693/0.841/1.271` |
| tare 1, 5초 | `[+0.05,-0.09,-0.06]` | `[0.125,0.171,0.218]` | - |
| tare 2, 5초 | `[+0.03,-0.10,-0.08]` | `[0.129,0.171,0.223]` | - |
| tare 3, 5초 | `[+0.01,-0.12,+0.29]` | `[0.136,0.179,0.232]` | - |
| zero-set2 후 60초 | `[+0.05,+0.09,-0.43]` | `[0.125,0.173,0.203]` | `0.793/0.933/1.327` |

- 세 tare의 Fz median은 `-0.08~+0.29 N`, 범위는 `0.37 N`이었다.
- 당시 `0.20 N` STD gate는 제조사 noise-free resolution `0.40 N`보다
  엄격해 1초 창을 간헐적으로 거부했다. 현재 gate는 `0.40 N`이다.
- zero-set2 후 같은 tare의 60초 선형 drift는 `[0.011,0.009,0.013] N/min`으로
  작았지만, force norm의 순간 최대는 `1.327 N`이었다.
- 관절 q/dq와 Fz의 선형 상관은 낮아 기록된 관절 미세운동만으로 noise와 tare
  중심값 차이를 설명할 수 없었다.

## 결론

정지 noise 자체는 현재 `0.40 N` 축별 STD gate 안에 들어오지만, 독립 tare 사이의
중심값 변화와 순간 force norm은 모델의 `1 N` 오차 예산에 비해 무시할 수 없다.
temperature, cable strain, 구조 하중과 sensor 내부 bias 반복성은 이 실험만으로
분리되지 않았다.

## Artifact와 관련 문제

- 임시 raw NPZ: `/tmp/aft_sensor2_noise_1786167728.npz`
- 전체 수치와 원인 가설:
  [FT-20260808-01](../problem/FT-20260808-01.md)
- legacy zero-set node 종료 교착과 안전한 `aft_zero_set2`:
  [FT-20260808-02](../problem/FT-20260808-02.md)
- 기준 자세 부호 불일치:
  [FT-20260808-03](../problem/FT-20260808-03.md)
- sensor/CAN 실제 갱신률:
  [FT-20260808-04](../problem/FT-20260808-04.md)

