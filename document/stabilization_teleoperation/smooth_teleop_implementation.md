# Smooth teleoperation 구현 및 검증

## 목적과 근거

`/home/chem/UMI-FT/analysis_results/logistic_box_motion_quality_comparison/report.md`의
공통 episode 100개 비교에서 contact-observer 방식은 VR 대비 다음 command artifact가
컸다.

| 지표 | contact-observer | VR | 상대차 |
|---|---:|---:|---:|
| position jitter RMS | 0.603 mm | 0.310 mm | +94.2% |
| position HF power | 0.015% | 0.006% | +134.2% |
| rotation jitter RMS | 0.164 deg | 0.075 deg | +120.5% |
| rotation HF power | 0.103% | 0.036% | +186.3% |

기존 follower command는 leader joint를 follower joint로 변환한 뒤 FK와 속도 제한만
거쳐 publish했다. 따라서 leader의 gravity/feedback artifact가 속도 제한보다 작은
고주파 움직임이면 그대로 follower command와 imitation-learning action에 포함됐다.

안정화 문서와 기존 구현을 대조한 결과는 다음과 같다.

| 요소 | 변경 전 상태 | 이번 처리 |
|---|---|---|
| gravity scale/ramp/LPF | 구현됨 | 기존 경로 유지 |
| virtual damping | 구현됨 | 기존 경로 유지 |
| feedback LPF/deadband/saturation/slew | 구현됨 | 기존 경로 유지 |
| canonical contact hysteresis/gain ramp | observer/leader에 구현됨 | 중복 classifier 추가 안 함 |
| narrow-band notch | 공진 peak 근거 없음 | 미적용 |
| 2차 intent generator | 없음 | 추가 |
| command acceleration limit | 없음 | 추가 |
| raw/intent/final command logging | 없음 | 추가 |
| clean expert action | raw 기반 publish | 최종 intent command topic으로 전환 |

## 현재 구현

FAST command 경로는 다음과 같다.

```text
leader joint raw
  -> follower joint mapping / joint clip
  -> follower FK / workspace clip
  -> 2nd-order intent reference generator
  -> velocity / acceleration limit
  -> existing final linear/angular slew safety limit
  -> /right_dsr_controller/task_space_command
```

`/right_dsr_controller/task_space_command`를 기록하는
`robot/command_quat_pose_se3.zarr`는 이제 raw FK가 아니라 최종 clean command를
저장한다. 기존 recorder나 학습 action source 계약은 바꾸지 않는다.

구현 위치는 다음과 같다.

- `include/ft_fb_leaderarm/intent_trajectory_generator.hpp`
- `src/intent_trajectory_generator.cpp`
- `src/single_impedance_pose_publisher.cpp`
- `config/single_impedance_leader_damping.yaml` (기존 leader/feedback 설정)
- `config/single_impedance_leader_smooth_teleop.yaml` (smooth tuning overlay)

## 시작 parameter

| Parameter | 값 | 의미 |
|---|---:|---|
| `intent_generator_enabled` | `true` | FAST에서 raw command 직접 전달 차단 |
| `intent_*_natural_frequency_hz` | `4.0` | 3 Hz 이상 artifact 감쇠 시작값 |
| `intent_damping_ratio` | `1.0` | critical damping |
| `intent_max_linear_velocity_mm_s` | `300` | 기존 최종 속도 제한과 동일 |
| `intent_max_linear_acceleration_mm_s2` | `1000` | command acceleration 제한 |
| `intent_max_angular_velocity_deg_s` | `300` | 기존 최종 각속도 제한과 동일 |
| `intent_max_angular_acceleration_deg_s2` | `720` | orientation acceleration 제한 |

이 값은 500 Hz software 시작값이다. 실제 task timing과 contact를 보존하는 최종값은
사용자 승인 아래 동일 task A/B evidence로 결정한다. 특정 좁은 공진 peak는 현재
report에서 확인되지 않았으므로 notch filter는 추가하지 않았다.

통합 launch는 기존 leader config 다음에 별도 smooth tuning overlay를 로드한다.
`smooth_teleop_enable:=true|false`는 마지막 override이므로 동일한 두 YAML에서 A/B를
전환하며, `false`일 때는 안정화 전 command elapsed/slew 규칙을 재현한다. 기본값은
`true`다.

2026-08-24 첫 Smooth ON pilot에서 jerk-limited acceleration이 고정 target에서도
limit cycle을 만들어 follower 자발 운동을 발생시켰다. 따라서 jerk limiter는 제거했고
velocity/acceleration limit과 최종 slew limit만 유지한다. 상세 evidence는
[`FT-20260824-01`](../problem/FT-20260824-01.md)에 기록했다.

## Logging 계약

leader teleoperation CSV에 다음 세 신호를 함께 저장한다.

- `task_raw_*`: 좌표 변환된 raw FK pose와 velocity
- `task_intent_*`: 2차 generator 출력 pose, velocity, acceleration
- `task_command_*`: 최종 safety slew 이후 실제 publish pose

따라서 문제가 leader 자체, intent generator, 최종 limiter, follower tracking 중 어디서
발생했는지 한 CSV에서 분리할 수 있다.

## Software 검증

`test_intent_trajectory_generator`는 다음을 검증한다.

- 첫 sample에서 command jump 없이 초기화
- 0.5 Hz 의도 동작 amplitude를 90% 이상 유지
- 10 Hz artifact amplitude를 raw의 20% 미만으로 감쇠
- 설정된 linear/angular velocity와 acceleration 제한 준수
- 고정된 위치·회전 target에서 intent 위치와 속도가 수렴
- 회전 출력의 SO(3) 유효성 유지
- 고정 workspace→command frame 변환에 대한 출력/속도 불변성

## Hardware A/B 검증

하드웨어 검증은 사용자 승인 없이는 실행하지 않는다. 승인 후에는 같은 task, episode
수, pre-roll, 속도 분포를 맞춰 기존 분석 script로 다음 순서로 비교한다.

운용자가 수행할 전체 순서와 중단·합격 기준은
[teleoperation 안정화 runbook](teleoperation_stabilization_runbook.md)을 따른다.

1. feedback OFF에서 raw와 intent 차이 및 조작 지연 확인
2. feedback ON + free-space에서 false motion과 HF power 확인
3. feedback ON + controlled contact에서 접촉 timing과 진동 확인
4. 기존 contact-observer baseline 및 VR과 command/actual/joint 지표 비교
5. task 성공률, 완료시간, 최대 접촉력에 regression이 없는지 확인

실측 전에는 실제 teleoperation 문제가 최종 해결됐다고 판정하지 않는다.
