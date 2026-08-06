# 실행 명령어

## 0. 모든 PC 터미널의 공통 설정과 주의사항

아래 명령은 오른팔, 현재 payload/controller 계약을 기준으로 한다. 장비 설정이
다르면 그대로 실행하지 말고 값을 먼저 바꾼다. **새 PC 터미널을 열 때마다**
아래 블록 전체를 먼저 실행한다. 이 설정이 빠지면 SBC에서 발행하는
`/bae_r/observer_input`과 `/aft_sensor2/wrench`가 PC에서 보이지 않을 수 있다.

```bash
source /opt/ros/humble/setup.bash
source /home/vision/contact_pipeline_ws/install/setup.bash
source /home/vision/dualarm_ws/install/setup.bash
source /home/vision/venv_act/bin/activate

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI=file:///home/vision/cyclonedds_dualarmPC.xml
export ROS_LOG_DIR=/tmp/ft_fb_leaderarm_ros_logs

export FT_PACKAGE_ROOT=/home/vision/dualarm_ws/src/ft_fb_leaderarm
export FT_DATA_DIR=/home/vision/.ros/ft_fb_leaderarm/data
export FT_MODEL_DIR=/home/vision/.ros/ft_fb_leaderarm/models/right_v1
export FT_LOG_DIR=/home/vision/dualarm_ws/src/ft_fb_leaderarm/logs
export FT_EVIDENCE_DIR=/home/vision/.ros/ft_fb_leaderarm/evidence/right_v1
export FT_PAYLOAD_ID=right_tool_m2p1kg_oz0p17m_v1
export FT_CONTROLLER_HASH=bae_r_v2_c113eabf7e13_ca07ae197213
export FT_MODEL=/home/vision/.ros/ft_fb_leaderarm/models/right_v1/model.ts
```

다음 값은 로봇, leader, tool, task의 안전 한계로 실험 전에 직접 결정한다.
문서가 임의로 확정할 수 있는 값이 아니다.

```bash
export FT_MAX_CONTACT_FORCE_N=REPLACE_WITH_APPROVED_TASK_LIMIT
```

`REPLACE_WITH_APPROVED_TASK_LIMIT`는 실행 가능한 숫자가 아니라 의도적인
안전 placeholder다. 예를 들어 사전 위험성 평가에서 `10.0 N`이 승인되었을
때만 `export FT_MAX_CONTACT_FORCE_N=10.0`처럼 바꾼다. 이 값이 미정이면
controlled-contact 실험과 단계 승인을 진행하지 않는다.

PC에서 두 입력 topic이 실제로 발견되는지 먼저 확인한다.

```bash
ros2 topic info /bae_r/observer_input --verbose
ros2 topic info /aft_sensor2/wrench --verbose
timeout 15s ros2 topic hz /bae_r/observer_input
timeout 15s ros2 topic hz /aft_sensor2/wrench
```

## 1. 빌드

PC에서 실행한다.

```bash
cd /home/vision/dualarm_ws
source /opt/ros/humble/setup.bash
source /home/vision/contact_pipeline_ws/install/setup.bash
colcon build --symlink-install --packages-select ft_fb_leaderarm
source /home/vision/dualarm_ws/install/setup.bash
source /home/vision/venv_act/bin/activate
```

설치된 실행 파일을 확인한다.

```bash
ros2 pkg executables ft_fb_leaderarm
```

다음 실행 파일이 보여야 한다.

```text
ft_contact_observer
ft_fb_leader_single_impedance_teleop
ft_feedback_analyze
ft_feedback_authorize
ft_free_space_collect
ft_free_space_train
ft_free_space_validate
```

소스 단위 테스트는 PyTorch가 있는 `venv_act`와 시스템 pytest를 함께 써야
한다. 이 PC의 venv에는 `pytest` 실행 파일이 따로 없으므로 다음처럼 실행한다.

```bash
cd /home/vision/dualarm_ws/src/ft_fb_leaderarm
export PYTHONPATH=/home/vision/venv_act/lib/python3.10/site-packages:/usr/lib/python3/dist-packages:${PYTHONPATH}
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
/home/vision/venv_act/bin/python3 -m pytest -q -p no:cacheprovider
```

## 2. SBC: robot driver

SBC 터미널 1에서 실행한다. IP는 현재 장비 설정을 확인한다.

```bash
source /opt/ros/humble/setup.bash
source /home/vision/doosan_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0
export ROBOT_NORMAL_IP=192.168.112.4
export ROBOT_RT_IP=192.168.137.50

ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
  name:=dsr01 \
  mode:=real \
  model:=m0609 \
  host:="${ROBOT_NORMAL_IP}" \
  port:=12345 \
  rt_host:="${ROBOT_RT_IP}"
```

확인한다.

```bash
ros2 control list_controllers -c /dsr01/controller_manager
ros2 service type /dsr01/realtime/read_data_rt
```

## 3. SBC: V2 impedance controller

SBC 터미널 2에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
source /home/vision/doosan_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0

ros2 launch dsr_realtime_control \
  impedance_control_vr_dls_f_comp_bae_r_v2.launch.py
```

`/bae_r/observer_input`을 확인한다.

```bash
ros2 param get /TorqueRtR control.enable_observer_input_publish
ros2 param get /TorqueRtR control.observer_input_frame_id
ros2 topic info /bae_r/observer_input --verbose
timeout 15s ros2 topic hz /bae_r/observer_input
ros2 topic echo /bae_r/observer_input --once
```

## 4. SBC: AFT ON과 hardware zero-set

SBC 터미널 3에서 AFT를 실행한다.

```bash
source /opt/ros/humble/setup.bash
source /home/vision/dualarm_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0

ros2 launch aft_can_hardware aft_sensor.launch.py
```

오른팔을 다음 자세로 이동하고 완전 정지·무접촉인지 확인한다.

```text
[5.5, 52.0, 112.0, 28.0, -107.0, -35.0] degree
```

별도 SBC 터미널에서 zero-set한다.

```bash
source /opt/ros/humble/setup.bash
source /home/vision/dualarm_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0

ros2 launch aft_can_hardware aft_zero_set.launch.py
```

출력을 확인한다.

```bash
ros2 topic info /aft_sensor2/wrench --verbose
timeout 15s ros2 topic hz /aft_sensor2/wrench
ros2 topic echo /aft_sensor2/wrench --once
```

## 5. PC: free-space 데이터 수집

### 5.1 Collector

zero-set을 실제로 새로 할 때마다 새로운 `zero_set_id`를 사용한다.

```bash
source /opt/ros/humble/setup.bash
source /home/vision/dualarm_ws/install/setup.bash
source /home/vision/venv_act/bin/activate

ros2 launch ft_fb_leaderarm collect_free_space.launch.py \
  output_dir:="${FT_DATA_DIR}" \
  zero_set_confirmed:=true \
  zero_set_id:=tare_YYYYMMDD_01 \
  payload_id:="${FT_PAYLOAD_ID}" \
  controller_config_hash:="${FT_CONTROLLER_HASH}"
```

`YYYYMMDD`는 실제 날짜로 교체한다. launch 인자만 확인하려면 collector에
`--ros-args --help`를 붙이지 말고 다음 명령을 사용한다.

```bash
ros2 launch ft_fb_leaderarm collect_free_space.launch.py --show-args
```

### 5.2 학습 전 feedback-OFF teleoperation

아직 모델이 없으므로 integrated observer launch가 아니라 복제된 teleop만
feedback source OFF로 실행한다.

```bash
ros2 run ft_fb_leaderarm ft_fb_leader_single_impedance_teleop --ros-args \
  --params-file "${FT_PACKAGE_ROOT}/config/single_impedance_leader_damping.yaml" \
  -p side:=right \
  -p feedback_source:=off
```

CURRENT → SLOW → FAST로 전환하고 collector episode를 시작한다.

```bash
ros2 service call /ft_free_space_collector/start_episode \
  std_srvs/srv/Trigger {}
```

로봇을 완전 무접촉 상태로 다양하게 움직인 뒤 접촉 전에 중지한다.

```bash
ros2 service call /ft_free_space_collector/stop_episode \
  std_srvs/srv/Trigger {}
```

같은 zero-set에서 여러 episode를 수집할 수 있다. 다른 zero group은 초기
자세로 복귀하여 AFT zero-set을 다시 실행하고 collector를 새 ID로 재실행한다.

```text
tare_YYYYMMDD_01
tare_YYYYMMDD_02
tare_YYYYMMDD_03
...
```

## 6. 데이터 검증

episode metadata를 빠르게 확인한다.

```bash
jq '{accepted,zero_set_id,duration_s,actual_hz,max_record_gap_ms,sync_rejections,invalid_rejections}' \
  "${FT_DATA_DIR}"/*.json
```

전체 dataset 계약과 독립 zero group split을 검증한다. 출력 파일은 기존 파일을
덮어쓰지 않으므로 매 검증마다 새 이름을 사용한다.

```bash
ros2 run ft_fb_leaderarm ft_free_space_validate -- \
  --data-dir "${FT_DATA_DIR}" \
  --seed 7 \
  --output "${FT_DATA_DIR}/dataset_validation_v1.json"
```

```bash
jq . "${FT_DATA_DIR}/dataset_validation_v1.json"
```

`passed=true`, 최소 3개 `zero_set_id`, 비어 있지 않은 train/validation/test를
확인한다.

## 7. 5개 모델 학습

기존 승인 모델을 덮어쓰지 않도록 매 실험마다 새 output directory를 사용한다.

```bash
ros2 run ft_fb_leaderarm ft_free_space_train -- \
  --data-dir "${FT_DATA_DIR}" \
  --output-dir "${FT_MODEL_DIR}" \
  --candidates static_linear dynamic_mlp history_mlp history_lstm history_gru \
  --epochs 60 \
  --batch-size 1024 \
  --learning-rate 0.001 \
  --max-force-error-n 1.0 \
  --benchmark-calls 2000
```

종료 코드 2는 artifact는 생성했지만 accuracy/runtime gate를 통과하지 못했다는
뜻이다.

## 8. 모델 성능 비교

모든 validation 결과를 표 형태로 확인한다.

```bash
jq -r '
  ["model","architecture","history","max_N","p95_N","rmse_N"],
  (.ablations[] | [
    .name,
    .architecture,
    (.history|tostring),
    (.validation.force_norm_max_n|tostring),
    (.validation.force_norm_p95_n|tostring),
    (.validation.force_norm_rmse_n|tostring)
  ]) | @tsv' \
  "${FT_MODEL_DIR}/ablation_report.json"
```

선택 모델과 최종 gate를 확인한다.

```bash
jq '{
  approved,
  selected_ablation,
  accuracy_gate,
  runtime_gate,
  held_out_test_selected_model_only
}' "${FT_MODEL_DIR}/ablation_report.json"

jq '{
  approved,
  ablation,
  architecture,
  validation,
  held_out_test,
  runtime_benchmark,
  model_sha256
}' "${FT_MODEL_DIR}/metadata.json"
```

`approved=true`가 아니면 다음 단계로 진행하지 않는다.

## 9. 승인 모델 observer-only 검증

새 runtime zero-set 후 실행한다.

```bash
ros2 launch ft_fb_leaderarm ft_contact_observer.launch.py \
  model_path:="${FT_MODEL}" \
  zero_set_confirmed:=true \
  zero_set_id:=runtime_tare_01 \
  payload_id:="${FT_PAYLOAD_ID}" \
  controller_config_hash:="${FT_CONTROLLER_HASH}"
```

```bash
timeout 15s ros2 topic hz /contact_observer/right/observation
ros2 topic echo /ft_contact_observer/diagnostics --once
ros2 topic echo /contact_observer/right/observation --once
ros2 topic echo /ft_free_space/right/predicted_wrench --once
ros2 topic echo /ft_free_space/right/contact_wrench --once
```

## 10. Feedback OFF 실기 evidence

통합 launch를 feedback OFF로 실행한다. observer 구독은 유지되고 gain만 0이다.

```bash
ros2 launch ft_fb_leaderarm ft_feedback_leader_teleop.launch.py \
  model_path:="${FT_MODEL}" \
  zero_set_confirmed:=true \
  zero_set_id:=runtime_off_01 \
  payload_id:="${FT_PAYLOAD_ID}" \
  controller_config_hash:="${FT_CONTROLLER_HASH}" \
  learned_feedback_enable:=false
```

각각 별도의 teleop process/CSV로 다음을 기록한다.

1. 무접촉 FREE run 1
2. 무접촉 FREE run 2
3. 무접촉 FREE run 3
4. 사전에 정한 최대 힘 이하의 controlled CONTACT run

CSV는 기본적으로 다음에 저장된다.

```bash
ls -lt "${FT_LOG_DIR}"/leader_teleop_right_*.csv
```

Analyzer의 고정 안전 계약은 FREE 최대 1 N, observer source age 20 ms,
CSV gap 10 ms, CSV 평균 250 Hz, pose step 1 deg, velocity reversal 8 Hz,
run당 최소 10초, controlled CONTACT 최소 3회다. CLI 인자는 이 기준을 더
엄격하게 만들 수 있지만 느슨하게 만들 수 없다.

## 11. OFF → 40% 자동 분석과 승인

자동 생성된 CSV 이름을 확인한 뒤, 실제 서로 다른 네 파일의 절대 경로를
변수에 넣는다. 아래 `/absolute/path/...`를 그대로 실행하면 안 된다.

```bash
export FT_OFF_FREE_01=/absolute/path/leader_teleop_right_OFF_FREE_01.csv
export FT_OFF_FREE_02=/absolute/path/leader_teleop_right_OFF_FREE_02.csv
export FT_OFF_FREE_03=/absolute/path/leader_teleop_right_OFF_FREE_03.csv
export FT_OFF_CONTACT_01=/absolute/path/leader_teleop_right_OFF_CONTACT_01.csv

test -f "${FT_OFF_FREE_01}" \
  -a -f "${FT_OFF_FREE_02}" \
  -a -f "${FT_OFF_FREE_03}" \
  -a -f "${FT_OFF_CONTACT_01}"

ros2 run ft_fb_leaderarm ft_feedback_analyze -- \
  --model "${FT_MODEL}" \
  --target-gain-scale 0.40 \
  --free-csv \
    "${FT_OFF_FREE_01}" \
    "${FT_OFF_FREE_02}" \
    "${FT_OFF_FREE_03}" \
  --contact-csv "${FT_OFF_CONTACT_01}" \
  --max-contact-force-n "${FT_MAX_CONTACT_FORCE_N}" \
  --max-free-force-error-n 1.0 \
  --max-pose-step-deg 1.0 \
  --max-velocity-reversal-hz 8.0 \
  --output "${FT_EVIDENCE_DIR}/off_to_40_analysis.json"
```

```bash
jq '{passed,aggregate,limits,failures}' \
  "${FT_EVIDENCE_DIR}/off_to_40_analysis.json"
```

`passed=true`일 때만 승인한다.

```bash
ros2 run ft_fb_leaderarm ft_feedback_authorize -- \
  --model-path "${FT_MODEL}" \
  --gain-scale 0.40 \
  --evidence "${FT_EVIDENCE_DIR}/off_to_40_analysis.json" \
  --operator-attestation \
    "I verified at least three feedback-OFF free-space runs, controlled contact detection, and the passing automatic analysis" \
  --output "${FT_EVIDENCE_DIR}/feedback_40_authorization.json"
```

## 12. 40% 실행

```bash
ros2 launch ft_fb_leaderarm ft_feedback_leader_teleop.launch.py \
  model_path:="${FT_MODEL}" \
  zero_set_confirmed:=true \
  zero_set_id:=runtime_40_01 \
  payload_id:="${FT_PAYLOAD_ID}" \
  controller_config_hash:="${FT_CONTROLLER_HASH}" \
  learned_feedback_enable:=true \
  feedback_gain_scale:=0.40 \
  feedback_authorization:="${FT_EVIDENCE_DIR}/feedback_40_authorization.json"
```

제한된 조건에서 feedback 방향을 확인한다. 방향 반대, 진동, pose jump가 있으면
즉시 중지하고 [failure_log.md](failure_log.md)에 기록한다. 40%에서도 FREE
3회와 controlled CONTACT CSV를 새 파일로 기록한다.

## 13. 40% → 100% 자동 분석과 승인

```bash
export FT_GAIN40_FREE_01=/absolute/path/leader_teleop_right_GAIN40_FREE_01.csv
export FT_GAIN40_FREE_02=/absolute/path/leader_teleop_right_GAIN40_FREE_02.csv
export FT_GAIN40_FREE_03=/absolute/path/leader_teleop_right_GAIN40_FREE_03.csv
export FT_GAIN40_CONTACT_01=/absolute/path/leader_teleop_right_GAIN40_CONTACT_01.csv

test -f "${FT_GAIN40_FREE_01}" \
  -a -f "${FT_GAIN40_FREE_02}" \
  -a -f "${FT_GAIN40_FREE_03}" \
  -a -f "${FT_GAIN40_CONTACT_01}"

ros2 run ft_fb_leaderarm ft_feedback_analyze -- \
  --model "${FT_MODEL}" \
  --target-gain-scale 1.00 \
  --free-csv \
    "${FT_GAIN40_FREE_01}" \
    "${FT_GAIN40_FREE_02}" \
    "${FT_GAIN40_FREE_03}" \
  --contact-csv "${FT_GAIN40_CONTACT_01}" \
  --max-contact-force-n "${FT_MAX_CONTACT_FORCE_N}" \
  --max-free-force-error-n 1.0 \
  --max-pose-step-deg 1.0 \
  --max-velocity-reversal-hz 8.0 \
  --output "${FT_EVIDENCE_DIR}/gain40_to_100_analysis.json"
```

```bash
jq '{passed,aggregate,limits,failures}' \
  "${FT_EVIDENCE_DIR}/gain40_to_100_analysis.json"
```

통과했을 때만 100% authorization을 생성한다.

```bash
ros2 run ft_fb_leaderarm ft_feedback_authorize -- \
  --model-path "${FT_MODEL}" \
  --gain-scale 1.00 \
  --evidence "${FT_EVIDENCE_DIR}/gain40_to_100_analysis.json" \
  --previous-authorization "${FT_EVIDENCE_DIR}/feedback_40_authorization.json" \
  --operator-attestation \
    "I verified correct feedback direction with no vibration or pose jump at the 40 percent stage and reviewed the passing automatic analysis" \
  --output "${FT_EVIDENCE_DIR}/feedback_100_authorization.json"
```

## 14. 100% 제한 실행

```bash
ros2 launch ft_fb_leaderarm ft_feedback_leader_teleop.launch.py \
  model_path:="${FT_MODEL}" \
  zero_set_confirmed:=true \
  zero_set_id:=runtime_100_01 \
  payload_id:="${FT_PAYLOAD_ID}" \
  controller_config_hash:="${FT_CONTROLLER_HASH}" \
  learned_feedback_enable:=true \
  feedback_gain_scale:=1.00 \
  feedback_authorization:="${FT_EVIDENCE_DIR}/feedback_100_authorization.json"
```

100%는 authorization이 있어도 자동 운전이나 임의 task 확장을 허용하는 것이
아니다. 승인 evidence와 동일한 tool/payload/controller/task 범위에서만 쓴다.

## 15. IL recorder 적용 전 확인

기존 recorder/GUI를 연결할 때 기존 V2 observer와 FT observer를 동시에 같은
topic으로 발행하면 안 된다.

```bash
ros2 topic info /contact_observer/right/observation --verbose
ros2 topic info /aft_sensor2/wrench --verbose
ros2 topic info /bae_r/observer_input --verbose
```

확인해야 할 source는 다음과 같다.

```text
physical raw FT : /aft_sensor2/wrench
contact residual: /contact_observer/right/observation
robot state     : /bae_r/observer_input
```

작은 IL test episode를 먼저 저장하여 timestamp, frame, model hash, raw/predicted/
contact wrench가 모두 남는지 검증한 후 본 수집을 시작한다.
