# ft_fb_leaderarm

오른팔 follower의 **물리 AFT 센서**를 직접 사용해 free-space wrench를
수집·학습하고, 승인된 모델로

```text
contact_wrench = physical_raw_wrench - predicted_free_space_wrench
```

를 262.5 Hz로 발행하는 ROS 2 패키지다. 기존 `fb_leaderarm`의 오른팔
single-impedance leader teleop을 이 패키지 안에 동일한 C++ 소스로 복제해
독립 실행 파일로 빌드한다. 로봇 driver와 V2 impedance controller의 topic
계약은 유지하지만 feedback 입력은 JTS wrench가 아닌 물리 FT 기반
`ContactObservation`을 사용한다.

## 문서 바로가기

- [전체 순서](docs/flow.md)
- [실행 명령](docs/command.md)
- [GUI 기반 free-space wrench 데이터 수집](docs/free_space_wrench_data_collection.md)
- [구현 목표와 TODO](docs/TODO_LIST.md)
- [PC-SBC 시계 동기화와 FT sample 시간 정렬](docs/timing_sync.md)
- [학습 구조와 ablation](docs/base_architecture.md)
- [fb_leaderarm과의 비교](docs/compare%20ft_fb_leaderarm%20and%20fb_leader%20arm.md)
- [FT 센서 점검표](docs/FTsensor_check_list.md)
- [AFT 센서 사양과 현재 이슈](docs/AFT_sensor_issue.md)
- [실패·문제 기록](docs/failure_log.md)

## 문제 정의: 육하원칙

| 구분 | 계약 |
|---|---|
| 누가 | 일반 PC의 `ft_free_space_collect`, `ft_free_space_train`, `ft_contact_observer` |
| 언제 | 오른팔 follower가 고정 초기 자세에 있고 AFT가 무접촉 zero-set된 뒤 |
| 어디서 | `/contact_state/observer_input`, `/aft_sensor2/wrench`, `/contact_observer/right/observation` |
| 무엇을 | `[q,dq,causal-qdd]`와 물리 FT `[Fx,Fy,Fz,Mx,My,Mz]` |
| 어떻게 | 독립 zero-set group 분할, 5개 ablation, held-out 최대 force-vector 오차 gate |
| 왜 | 기존 JTS `external_tcp_force` target에 있던 관측 불가능한 1 N 이상 drift를 물리 FT 직접 계측으로 분리하기 위해 |

```text
/contact_state/observer_input ─┐
                       ├─ collector ─ NPZ episodes ─ ablation trainer ─ model.ts
/aft_sensor2/wrench ───┘                                      │
                                                              ▼ 262.5 Hz
/contact_state/observer_input ─┐                              ft_contact_observer
/aft_sensor2/wrench ───┘                                      │
                       ┌───────────────────────────────────────┼──────────────┐
                       ▼                                       ▼              ▼
          ContactObservation(base)              predicted FT(sensor)  residual FT(sensor)
          기존 leader/IL recorder               분석/모니터링         분석/모니터링
```

## 1 N의 정확한 의미

승인 기준은 force 각 축의 오차를 따로 보는 느슨한 기준이 아니라 다음의
force-vector 최대 오차다.

```text
max_t ||F_raw(t) - F_prediction(t)||_2 <= 1.0 N
```

모델 선택에는 validation만 사용하고, 선택이 끝난 뒤 독립
`zero_set_id` held-out test를 한 번 평가한다. validation 또는 test가 1 N을
넘거나, CPU 추론이 262.5 Hz의 3.81 ms deadline을 통과하지 못하면
`metadata.json`에 `approved: false`가 기록되고 runtime은 그 모델을
로드하지 않는다.

코드만으로 실제 장비의 1 N 성능을 미리 보장할 수는 없다. 데이터 취득과
held-out 결과가 gate를 통과해야 보장이 성립한다.

## 빌드

```bash
cd /home/vision/dualarm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ft_fb_leaderarm
source install/setup.bash
source /home/vision/venv_act/bin/activate
```

현재 PC의 PyTorch는 system Python이 아니라 `venv_act`에 있다. CMake/CTest가
system `pytest`를 찾도록 build를 먼저 수행하고, `ros2 run/launch` 전에
venv를 활성화한다.

## 데이터 수집

SBC에서 기존과 동일하게 Doosan driver, V2 impedance controller,
`aft_sensor.launch.py`를 실행한다. follower를 아래 고정 초기 자세로
이동시킨다.

```text
[5.5, 52.0, 112.0, 28.0, -107.0, -35.0] degree
```

센서/tool이 어떤 물체와도 접촉하지 않고 로봇이 완전히 정지했을 때 SBC에서
한 번 실행한다.

```bash
ros2 launch aft_can_hardware aft_zero_set2.launch.py \
  sensor_name:=aft_sensor2
```

zero-set2 launch는 오른팔에 hardware tare를 한 번 요청하고 정상 전달 뒤 종료한다.
AFT driver ON만으로는 zero-set되지 않는다. 현재 관절각이 위 기준 자세와 1 deg
이내인지 먼저 확인한다. zero-set 직후 한 sample만 보고 영점을 판정하지 않는다.
최소 5초 구간의
축별 median과 표준편차를 확인하고, 독립 zero-set마다 새 `zero_set_id`를 사용한다.
자세 불일치나 launch 실패 시에는 [실행 절차](docs/command.md#4-sbc-aft-on과-hardware-zero-set)를
따른다.

각 실제 zero-set마다 중복되지 않는 `zero_set_id`를 만들고 collector를
실행한다.

```bash
ros2 launch ft_fb_leaderarm collect_free_space_gui.launch.py \
  zero_set_confirmed:=true \
  zero_set_id:=tare_20260806_01 \
  payload_id:=right_tool_m2p1kg_oz0p17m_v1 \
  controller_config_hash:=bae_r_v2_c113eabf7e13_ca07ae197213
```

이 launch는 `ft_fb_leaderarm`가 자체 소유하는 FT collector와 GUI만 실행한다.
teleop이나 robot 이동은 자동 실행하지 않는다. GUI는 고정 자세, joint 정지,
1초 연속 FT 안정성, frame, timestamp 동기화가 확인되기 전 START를 거부한 이유를
Zero Gate 배지와 팝업으로 표시한다.

```bash
# GUI: START FT EPISODE → CURRENT → SLOW 무접촉 → 접촉 전 STOP FT EPISODE
# 같은 service를 터미널에서 직접 호출해도 된다.
ros2 service call /ft_free_space_collector/start_episode std_srvs/srv/Trigger {}
ros2 service call /ft_free_space_collector/stop_episode std_srvs/srv/Trigger {}
```

한 episode 안에는 저속/고속, 가속/감속, 관절/Cartesian 전 작업 범위를
넣되 접촉 직전에 반드시 중지한다. 최소 3개의 독립 zero-set group이
필수이며, 일반화 검증에는 서로 다른 시간대·재기동을 포함한 10개 이상
group을 권장한다. payload, tool 체결, controller hash가 바뀌면 같은
dataset에 섞지 않는다.

## Ablation 학습

다음 다섯 후보를 같은 group split으로 비교한다.

| 후보 | 입력 | 목적 |
|---|---|---|
| `static_linear` | 현재 `sin(q),cos(q)` | payload 중력 중심의 기준선 |
| `dynamic_mlp` | 현재 `sin(q),cos(q),dq,qdd` | 순간 동역학 효과 |
| `history_mlp` | 최근 16 sample의 동일 입력 | 약 61 ms causal 이력 효과 |
| `history_lstm` | 최근 16 sample을 LSTM에 순서대로 입력 | 장기·비선형 시간 의존성 |
| `history_gru` | 최근 16 sample을 GRU에 순서대로 입력 | LSTM보다 단순한 recurrent 기준선 |

`task_error`, measured/raw joint torque, measured wrench는 실제 접촉을
모델이 free-space로 흡수할 수 있으므로 runtime 입력에서 금지한다.

```bash
ros2 run ft_fb_leaderarm ft_free_space_train -- \
  --data-dir /home/vision/.ros/ft_fb_leaderarm/data \
  --output-dir /home/vision/.ros/ft_fb_leaderarm/models/right_v1
```

결과는 `ablation_report.json`, `metadata.json`, `model.ts`다.
종료 코드 `2`는 artifact는 진단용으로 저장했지만 1 N 또는 runtime gate를
통과하지 못했다는 뜻이다.

## 실시간 Contact Observer

학습 때와 동일한 고정 자세에서 새 hardware zero-set을 수행한 뒤 실행한다.
observer도 1초 zero 검증을 다시 통과하기 전까지 `valid=false`를 발행한다.

```bash
ros2 launch ft_fb_leaderarm ft_contact_observer.launch.py \
  model_path:=/home/vision/.ros/ft_fb_leaderarm/models/right_v1/model.ts \
  zero_set_confirmed:=true \
  zero_set_id:=runtime_tare_20260806_01 \
  payload_id:=right_tool_m2p1kg_oz0p17m_v1 \
  controller_config_hash:=bae_r_v2_c113eabf7e13_ca07ae197213
```

확인 항목:

```bash
timeout 8s ros2 topic hz /contact_observer/right/observation
ros2 topic echo /ft_contact_observer/diagnostics --once
ros2 topic echo /contact_observer/right/observation --once
```

기본 contact detector는 force norm `2.0/1.2 N` ON/OFF hysteresis와
`8/20 ms` hold를 사용한다. threshold는 실제 contact SNR 검증 뒤 조정한다.

## FT 기반 leader teleoperation/IL 연동

기존 leader feedback까지 함께 실행할 때:

```bash
ros2 launch ft_fb_leaderarm ft_feedback_leader_teleop.launch.py \
  model_path:=/home/vision/.ros/ft_fb_leaderarm/models/right_v1/model.ts \
  zero_set_confirmed:=true \
  zero_set_id:=runtime_tare_20260806_01 \
  payload_id:=right_tool_m2p1kg_oz0p17m_v1 \
  controller_config_hash:=bae_r_v2_c113eabf7e13_ca07ae197213
```

이 launch는 이 패키지에 복제된
`ft_fb_leader_single_impedance_teleop`을 실행한다. leader Dynamixel 제어,
follower command, gravity compensation, keyboard FSM, 안전 gate는 기존
오른팔 single-impedance teleop과 동일하다. 달라진 입력 경로는 아래 하나다.

```text
기존: /bae_r/F_e (JTS wrench) → leader feedback
현재: /aft_sensor2/wrench - predicted_free_space_wrench
      → /contact_observer/right/observation → leader feedback
```

설정의 `feedback_source`는 항상 `contact_observer`이며 JT feedback은
비활성화되어 있다. 통합 launch의 기본값은 하드웨어 안전을 위해
`learned_feedback_enable:=false`다. 이때도 observer 구독과 contact-state
검사는 유지되고 오른팔 feedback gain만 0이 된다.

반력 검증은 `feedback OFF → 40% → 100%` 순서만 허용한다. 각 단계는
승인된 모델, 자동 분석을 통과한 FT/Leader CSV, 이전 단계 승인 파일의
SHA-256에 묶인 authorization JSON이 있어야 한다. Analyzer는 FREE false
contact, controlled CONTACT 횟수, 최대 force/feedback torque, observer health,
velocity reversal 기반 vibration 지표와 joint pose jump를 계산한다. 실제
feedback 방향은 운영자가 확인한다.

feedback-OFF FREE 로그 3개 이상과 controlled-CONTACT 로그를 먼저 분석한다.
`--max-contact-force-n`은 장비와 task에 맞춰 실험 전에 정한 값이어야 한다.

```bash
ros2 run ft_fb_leaderarm ft_feedback_analyze -- \
  --model /home/vision/.ros/ft_fb_leaderarm/models/right_v1/model.ts \
  --target-gain-scale 0.40 \
  --free-csv /absolute/path/off_free_01.csv /absolute/path/off_free_02.csv /absolute/path/off_free_03.csv \
  --contact-csv /absolute/path/off_contact.csv \
  --max-contact-force-n 10.0 \
  --output /absolute/path/off_to_40_analysis.json
```

분석이 `GO`일 때만 40% stage를 승인한다.

```bash
ros2 run ft_fb_leaderarm ft_feedback_authorize -- \
  --model-path /home/vision/.ros/ft_fb_leaderarm/models/right_v1/model.ts \
  --gain-scale 0.40 \
  --evidence /absolute/path/off_to_40_analysis.json \
  --operator-attestation "I verified at least three feedback-OFF free-space runs, controlled contact detection, and the passing automatic analysis" \
  --output /home/vision/.ros/ft_fb_leaderarm/models/right_v1/feedback_40.json
```

그 다음 40% feedback을 실행한다.

```bash
ros2 launch ft_fb_leaderarm ft_feedback_leader_teleop.launch.py \
  model_path:=/home/vision/.ros/ft_fb_leaderarm/models/right_v1/model.ts \
  zero_set_confirmed:=true \
  zero_set_id:=runtime_tare_01 \
  payload_id:=right_tool_m2p1kg_oz0p17m_v1 \
  controller_config_hash:=bae_r_v2_c113eabf7e13_ca07ae197213 \
  learned_feedback_enable:=true \
  feedback_gain_scale:=0.40 \
  feedback_authorization:=/home/vision/.ros/ft_fb_leaderarm/models/right_v1/feedback_40.json
```

40% FREE/CONTACT 로그도 같은 analyzer에서 `--target-gain-scale 1.00`으로
분석한다. 통과한 JSON을 `--evidence`에, `feedback_40.json`을
`--previous-authorization`에 넣어 100% authorization을 만든다. Analyzer
report와 authorization은 raw CSV까지 hash로 묶으므로 분석 후 파일이
변경되면 launch가 거부된다.

| 승인할 gain | `--operator-attestation`의 정확한 문장 |
|---|---|
| 40% | `I verified at least three feedback-OFF free-space runs, controlled contact detection, and the passing automatic analysis` |
| 100% | `I verified correct feedback direction with no vibration or pose jump at the 40 percent stage and reviewed the passing automatic analysis` |

IL recorder의 기존 source 계약도 유지된다.

- physical raw FT: `/aft_sensor2/wrench`, sensor frame, 1 kHz publish
- collector/runtime 처리율: 262.5 Hz
- contact 상태/residual: `/contact_observer/right/observation`, base frame
- controller state: `/contact_state/observer_input`

동일한 `/contact_observer/right/observation`에 기존
`contact_observer_node.py`와 새 `ft_contact_observer`를 동시에 실행하면
안 된다. 현재 `single_impedance_feedback_leaderarm_data_collection_v2.launch.py`
통합 launch는 기존 observer도 시작하므로, 이 새 backend를 사용할 때는
observer/leader는 위 launch로 실행하고 기존 UMI recorder와 GUI만 같은
topic 계약으로 실행한다.

## 좌표계와 영향 범위

모델 target과 분석 topic은 AFT sensor frame의
`[Fx,Fy,Fz,Mx,My,Mz]`다. `ContactObservation`은 기존 leader가 요구하는
`right_base_link`로 변환된다. 기본 sensor-to-tip transform은 기존
`right_ft_frame=right_link_6`과 같은 축/원점 가정이다. 실제 장착이 다르면
`config/observer.yaml`의 `sensor_to_tip_zyx_deg`와
`tip_to_sensor_translation_m`를 실측값으로 바꿔야 한다.

`ft_feedback_leader_teleop.launch.py`를 실행하면 이 패키지의 teleop 노드가
leader Dynamixel을 직접 구동하고 기존 오른팔 impedance controller에 motion
command를 발행한다. driver, impedance controller 구현과 AFT zero-set
command 자체는 이 패키지가 소유하지 않는다. 기존 `fb_leaderarm` 소스와
기존 dataset/model은 변경하지 않는다.
