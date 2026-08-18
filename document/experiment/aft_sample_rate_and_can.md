# AFT sample-rate와 CAN/ROS 실제 갱신률 실험

- 수행일: 2026-08-08
- 대상: 오른팔 AFT `can1`, `/aft_sensor2/wrench`
- 범위: robot 이동 없이 CAN frame 주기, ROS publish rate와 중복값 비교

## 목적

설정의 `1000 Hz`가 실제 AFT acquisition rate인지 확인하고, ROS wrench publish
rate와 CAN에서 새 force/torque sample이 도착하는 rate가 다른 원인을 분리한다.
또한 free-space collector 목표 `262.5 Hz`에 현재 AFT data rate가 충분한지 판단한다.

## 방법과 실행 명령

AFT driver를 실행한 뒤 CAN 상태와 ROS 연결을 확인한다.

```bash
source /opt/ros/humble/setup.bash
source /home/vision/doosan_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0

ip -details link show can1
ros2 topic info /aft_sensor2/wrench --verbose
timeout 15s ros2 topic hz /aft_sensor2/wrench
```

CAN force/torque frame과 pair 간격을 확인한다.

```bash
timeout 5s candump -L can1
candump -L -n 20 can1
```

ROS 값의 중복률과 timestamp를 분석하기 위한 raw data를 기록한다.

```bash
export EXP_DIR=/home/vision/.ros/ft_fb_leaderarm/experiments/aft_sample_rate
mkdir -p "${EXP_DIR}"

timeout --signal=INT 60s ros2 bag record \
  -o "${EXP_DIR}/wrench_60s" \
  /aft_sensor2/wrench
```

sensor2의 현재 운용 rate 500 Hz를 runtime command로 한 번 명시한다.

```bash
ros2 topic info /aft_sensor2/sample_rate_setting --verbose
ros2 topic pub --once /aft_sensor2/sample_rate_setting \
  std_msgs/msg/Int32 "{data: 500}"
```

command 뒤 같은 `topic hz`, `candump`와 60초 기록을 반복하여 전후를 비교한다.

## 결과

- ROS publish/stamp rate는 약 `1000.07 Hz`, 최대 stamp gap은 `1.066 ms`였다.
- 연속 force 값의 `50.0008%`가 완전히 같아 새 값의 유효 갱신률은 약
  `499.99 Hz`였다.
- raw CAN에서는 force ID `0x001`과 torque ID `0x002`가 한 sample pair를 이루며,
  pair 간격은 약 `2 ms`였다.
- CAN은 1 Mbps, `ERROR-ACTIVE`였고 bus error/drop/missed/bus-off는 0이었다.
- 500 Hz one-shot 뒤에도 ROS publish 약 1000 Hz, 연속 force 중복 약 50%, 유효
  갱신 약 500 Hz가 유지됐다.
- collector smoke는 `262.495 Hz`, 최대 gap `4.030 ms`, sync p99/max
  `1.223/1.397 ms`, rejection 0으로 통과했다.

## 원인과 결론

`AftCanHardware::on_configure()`가 rate 설정 CAN frame을 실제로 전송하지 않고,
broadcaster가 1000 Hz controller update마다 마지막 state를 반복 publish하는 것이
직접 원인이다. 현재 실제 500 Hz acquisition은 collector의 262.5 Hz 저장에는
충분하다.

실제 sensor를 1000 Hz로 바꾸려면 sample당 force/torque 두 frame, 즉 약
2000 frame/s를 처리해야 한다. 현재 driver는 1 ms `read()`마다 frame 하나만
소비하므로 먼저 pending CAN frame을 drain하도록 read 경로를 수정해야 한다.

## Artifact와 관련 문제

- 상세 원인·변경·rollback:
  [FT-20260808-04](../problem/FT-20260808-04.md)
- 통합 센서 설명:
  [AFT sensor 사양과 현재 이슈](../AFT_sensor_issue.md)
- sample-rate가 정지 noise 원인으로 확정되지 않았다는 기록:
  [FT-20260808-01](../problem/FT-20260808-01.md)
- collector smoke artifact:
  `/tmp/ft_fb_collector_gate_dryrun/right_free_space_20260808_151919.npz`

