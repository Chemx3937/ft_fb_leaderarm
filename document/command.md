# 실행 명령어

## 0. 모든 PC 터미널의 공통 설정과 주의사항

아래 명령은 오른팔, 현재 payload/controller 계약을 기준으로 한다. 장비 설정이
다르면 그대로 실행하지 말고 값을 먼저 바꾼다. **새 PC 터미널을 열 때마다**
아래 블록 전체를 먼저 실행한다. 이 설정이 빠지면 SBC에서 발행하는
`/contact_state/observer_input`과 `/aft_sensor2/wrench`가 PC에서 보이지 않을 수 있다.

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
export FT_MODEL_DIR=/home/vision/.ros/ft_fb_leaderarm/models/right_train13_ridge_short_multiscale_bundle_v3_20260822
export FT_LOG_DIR=/home/vision/dualarm_ws/src/ft_fb_leaderarm/logs
export FT_EVIDENCE_DIR=/home/vision/.ros/ft_fb_leaderarm/evidence/right_current_model
export FT_PAYLOAD_ID=right_tool_m2p1kg_oz0p17m_v1
export FT_CONTROLLER_HASH=bae_r_v2_c113eabf7e13_ca07ae197213
export FT_MODEL=/home/vision/.ros/ft_fb_leaderarm/models/right_train13_ridge_short_multiscale_bundle_v3_20260822/model.ts
```

현재 controlled-contact 운용 한계는 목표 `20 N 이하`, hard abort `25 N`이다.

```bash
export FT_MAX_CONTACT_FORCE_N=25.0
```

`25.0`은 접근 목표가 아니라 즉시 중단 한계다. Contact 접근은 `50 mm/s 이하`로
시작하고 `20 N`에 도달하기 전에 멈춘다. Tool, fixture 또는 task가 바뀌면 더 낮은
한계를 다시 승인하기 전까지 controlled-contact를 진행하지 않는다.

PC에서 두 입력 topic이 실제로 발견되는지 먼저 확인한다.

```bash
ros2 topic info /contact_state/observer_input --verbose
ros2 topic info /aft_sensor2/wrench --verbose
timeout 15s ros2 topic hz /contact_state/observer_input
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

단위 테스트는 시스템 pytest와 `venv_act`의 PyTorch를 함께 사용한다. 기존
CMake cache가 venv Python을 기억할 수 있으므로 system Python을 명시해 다시
구성한 뒤 `colcon test`로 실행한다.

```bash
cd /home/vision/dualarm_ws
source /opt/ros/humble/setup.bash
source /home/vision/contact_pipeline_ws/install/setup.bash

colcon build --symlink-install --packages-select ft_fb_leaderarm \
  --cmake-force-configure \
  --cmake-args -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3

source /home/vision/dualarm_ws/install/setup.bash
export PYTHONPATH=/home/vision/venv_act/lib/python3.10/site-packages:${PYTHONPATH}
colcon test --packages-select ft_fb_leaderarm
colcon test-result --verbose
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

`/contact_state/observer_input`을 확인한다.

```bash
ros2 param get /TorqueRtR control.enable_observer_input_publish
ros2 param get /TorqueRtR control.observer_input_frame_id
ros2 topic info /contact_state/observer_input --verbose
timeout 15s ros2 topic hz /contact_state/observer_input
ros2 topic echo /contact_state/observer_input --once
```

## 4. SBC: AFT ON과 hardware zero-set

SBC 터미널 3에서 AFT를 실행한다.

```bash
source /opt/ros/humble/setup.bash
source /home/vision/doosan_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0

ros2 launch aft_can_hardware aft_sensor.launch.py
```

이 명령은 AFT 통신을 시작하지만 hardware zero-set을 수행하지 않는다. driver의
`on_configure()`에 있는 `bias_setting_mode(true)` 반환값은 CAN으로 전송되지 않는다.

오른팔을 다음 자세로 이동하고 완전 정지·무접촉인지 확인한다.

```text
[5.5, 52.0, 112.0, 28.0, -107.0, -35.0] degree
```

별도 SBC 터미널에서 zero-set한다.

```bash
source /opt/ros/humble/setup.bash
source /home/vision/doosan_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0

ros2 topic info /aft_sensor2/bias_setting --verbose
ros2 launch aft_can_hardware aft_zero_set2.launch.py \
  sensor_name:=aft_sensor2
```

`bias_setting`의 subscriber가 정확히 1개인지 확인한다. launch는 오른팔 wrench
100개를 받은 뒤 `Bool(data=true)`를 한 번 발행하고 종료한다. 정상 로그는
`Hardware bias (tare) requested and acknowledged`와
`Zero set completed. Exiting after callback.`이다. ACK는 DDS subscriber 전달만
뜻하므로 아래 wrench 검증도 반드시 수행한다. subscriber가 없다는 오류가 나오거나
node가 종료되지 않으면 Ctrl-C하고 반복 실행하지 않는다.

2026-08-08 수정본은 `aft_zero_set2`로 분리했다. callback 내부 ROS shutdown
교착을 제거하고 한 센서만 실행한다. 격리 회귀 테스트와 실제 오른팔 tare 후
clean exit(`RC=0`)를 확인했다. 기존 `aft_zero_set` script/launch는 기존 자동화
호환을 위해 원본 그대로 유지한다.

기존 명령은 다음과 같이 계속 실행할 수 있다.

```bash
ros2 launch aft_can_hardware aft_zero_set.launch.py
```

legacy launch는 sensor1/2를 동시에 실행하며 bias 후 node가 종료되지 않는 기존
동작도 그대로다. 단일 오른팔 zero와 자동 종료가 필요하면 `aft_zero_set2`를
사용한다.

zero-set2 launch 자체의 문제를 분리 진단해야 할 때만 다음 직접 명령을 한 번
사용한다.

```bash
ros2 topic pub --once /aft_sensor2/bias_setting std_msgs/msg/Bool \
  "{data: true}"
```

출력을 확인한다.

```bash
ros2 topic info /aft_sensor2/wrench --verbose
timeout 15s ros2 topic hz /aft_sensor2/wrench
ros2 topic echo /aft_sensor2/wrench --once
```

`--once` 출력은 통신·frame 확인용일 뿐 zero 품질 판정값이 아니다. zero-set 후
최소 3초 기다리고, collector diagnostics의 최근 1초 `force_median_n`,
`force_std_n`, `ready`, `reason`을 여러 번 확인한다. 현재 기준은 force median
norm 1.0 N 이하와 각 축 std 0.40 N 이하이다. 이 값은 공식 AFT force
noise-free resolution(STD)에 맞춘 것이다. 변경 근거와 영향은
[AFT 센서 이슈](AFT_sensor_issue.md), 측정값은 [FT sensor 확인 목록](FTsensor_check_list.md)과
[FT-20260808-01](problem/FT-20260808-01.md)을
먼저 확인한다.

현재 실측에서는 ROS topic은 약 1000 Hz지만 raw CAN의 새 force/torque 쌍은 약
500 Hz이고 연속 force 값 약 50%가 중복된다. sensor2 설정은 `sample_rate: 500`으로
맞췄지만 driver 초기화에서는 아직 실제 CAN으로 전송되지 않는다. collector의
262.5 Hz 목표에는 충분하지만 동적 timestamp 검증 전에는 이를 1000 Hz 실측
데이터로 해석하지 않는다.
현재 driver는 CAN frame을 controller cycle당 하나만 읽으므로 실제 500 Hz를 임시
운용 계약으로 사용한다. 공식 센서 사양상 1000 Hz는 가능하지만 sample당
force/torque 두 frame이어서 driver read 경로 수정 없이 올리면 backlog 위험이 있다.
`/aft_sensor2/sample_rate_setting`을 임의 발행하지 말고
[FT-20260808-04](problem/FT-20260808-04.md)의
rate 계약을 먼저 확정한다.

AFT를 새로 시작한 뒤 sensor2 rate를 명시적으로 적용할 때는 다음을 한 번 실행하고
raw CAN pair 주기가 약 2 ms인지 확인한다.

```bash
ros2 topic pub --once /aft_sensor2/sample_rate_setting \
  std_msgs/msg/Int32 "{data: 500}"
```

## 5. PC: free-space 데이터 수집

### 5.1 Collector

zero-set을 실제로 새로 할 때마다 새로운 `zero_set_id`를 사용한다.

`payload_id`는 같은 tool/질량/질량중심 조합을 식별하는 사람이 정한 이름이다.
`controller_config_hash`는 사용한 impedance controller 코드·설정의 지문이다. 둘 다
robot 명령이 아니라 episode metadata이며, 값이 다른 데이터를 한 모델에 섞지 않기
위해 필요하다. 두 값을 전달해도 payload나 controller 설정은 변경되지 않는다. 현재는
기존에 사용하던 payload와 controller 설정을 그대로 유지하고, 그 상태를 위의
`FT_PAYLOAD_ID`와 `FT_CONTROLLER_HASH`로 기록한다. 실제 값이 현재 설정과 일치하는지
읽기 전용으로 확인하기 전에는 정식 output directory에 수집하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source /home/vision/dualarm_ws/install/setup.bash
source /home/vision/venv_act/bin/activate

ros2 launch ft_fb_leaderarm collect_free_space_gui.launch.py \
  output_dir:="${FT_DATA_DIR}" \
  zero_set_confirmed:=true \
  zero_set_id:=tare_YYYYMMDD_01 \
  payload_id:="${FT_PAYLOAD_ID}" \
  controller_config_hash:="${FT_CONTROLLER_HASH}" \
  start_teleop:=true
```

`start_teleop:=true`는 collector, FT 전용 GUI와 feedback-OFF teleop을 함께
실행한다. GUI 창에서 `1/c/t/o/s/2` 키로 수집과 leader 상태를 제어한다. teleop을
이미 별도 terminal에서 실행했다면 `start_teleop:=false`로 중복 실행을 피한다.
GUI 없이 터미널만 사용할 때는 launch 파일 이름만
`collect_free_space.launch.py`로 바꾼다.

통합 launch에서는 terminal keyboard가 의도적으로 꺼져 있으므로 키는 활성화된 GUI
창에서 입력한다. 종료는 GUI에서 `q` 한 번 → `SHUTDOWN Done` 확인 → launch
terminal에서 `Ctrl+C` 한 번 순서로 수행한다.

`YYYYMMDD`는 실제 날짜로 교체한다. launch 인자만 확인하려면 collector에
`--ros-args --help`를 붙이지 말고 다음 명령을 사용한다.

```bash
ros2 launch ft_fb_leaderarm collect_free_space_gui.launch.py --show-args
```

### 5.2 학습 전 feedback-OFF teleoperation

아직 모델이 없으므로 integrated observer launch가 아니라 복제된 teleop만
feedback source OFF로 실행한다.

```bash
ros2 run ft_fb_leaderarm ft_fb_leader_single_impedance_teleop --ros-args \
  --params-file "${FT_PACKAGE_ROOT}/config/single_impedance_leader_damping.yaml" \
  -p side:=right \
  -p feedback_source:="'off'" \
  -p keyboard_input_enabled:=false
```

FT 전용 GUI 절차에서는 terminal key가 GUI의 순서 guard를 우회하지 않도록
`keyboard_input_enabled=false`를 사용한다. GUI 없이 terminal key로 운용할 때만 이
override를 제거한다.

leader는 startup에서 position mode로 현재 follower 자세에 자동 정렬된 뒤 `IDLE`이
되며, 이때 leader가 실제로 움직일 수 있다. startup 정렬은 follower command를
발행하지 않는다. follower가 이미 fixed zero pose라면 `INIT POSE/REALIGN`을 반복하지
않는다. follower가 다른 자세일 때만 `INIT POSE → REALIGN`을 수행한다. 아직 CURRENT로
전환하지 않고 fixed zero pose에서 `zero_verified`인지 확인한 뒤 먼저 collector
episode를 시작한다.

gate가 실패해도 zero-set, AFT publish 또는 robot이 차단되는 것은 아니다. 아래의
**새 수집 시작 요청만 거부**되고 collector는 계속 실행된다. 먼저 diagnostics를
확인하고, 바로 아래의 start service는 한 번만 호출한다. service 응답과 diagnostics
두 경로에 차단 이유가 표시된다.

```bash
ros2 topic echo /ft_free_space_collector/diagnostics --once
```

median norm이 `1.0 N`을 넘은 경우 service 출력 예시는 다음과 같다.

```text
response:
std_srvs.srv.Trigger_Response(success=False,
  message='fixed-pose zero verification failed: zero_force_offset_too_large')
```

diagnostics의 핵심은 `"collecting": false`,
`"zero": {"ready": false, "reason": "zero_force_offset_too_large", ...}`다.
반대로 start가 성공하면 `success: true`, `episode started`가 출력된다.

```bash
ros2 service call /ft_free_space_collector/start_episode \
  std_srvs/srv/Trigger {}
```

start 성공 후에만 운용자가 leader를 CURRENT로 전환한다. GUI 통합 launch는
`record_only_fast:=true`가 기본이므로 start는 episode를 `ARMED`로만 만들고,
CURRENT 전환과 SLOW 정렬 구간은 저장하지 않는다. 정렬 후 FAST로 전환하면 그때부터
sample이 증가한다. FAST에서 무접촉 task 궤적을 수행하고 접촉 전에 중지한다. robot
또는 leader를 움직이는 조작은 운용자만 실행한다.

FT 전용 GUI의 `START FT EPISODE (1)`와 `STOP FT EPISODE (2)`는 각각 위의
`/ft_free_space_collector/start_episode`, `stop_episode`를 호출한다. START 실패
사유는 경고 팝업과 Problem Logs에 표시된다. START 성공 후 diagnostics에서
`collecting=true`가 확인되어 FT Collector 배지가 `ARMED`가 되기 전까지 GUI는
CURRENT/SLOW/FAST 요청을 차단한다. Teleop 상태가 position 정렬 완료인 `IDLE`이
아니면 START를 차단한다. FAST가 확인되면 배지가 `RECORDING`으로 바뀐다. 수집 중에는
Zero Gate 배지를 `LATCHED`로 표시하며, 이는 zero pose를 벗어나도 이미 승인된
episode가 유지된다는 뜻이다.

이 GUI는 복제 원본인 `fb_leaderarm` GUI를 import하거나 실행하지 않는다. 소스와
launch, service 연결은 모두 `ft_fb_leaderarm` 내부에 있다.

```bash
ros2 service call /ft_free_space_collector/stop_episode \
  std_srvs/srv/Trigger {}
```

같은 zero-set에서 여러 episode를 수집할 때도 현재 구현은 매 `start_episode` 순간
follower가 fixed zero pose로 복귀해 zero gate를 다시 통과해야 한다. 복귀하지 않고
여러 구간을 기록하려면 한 episode를 계속 유지한다. 다른 zero group은 fixed pose로
복귀하여 AFT zero-set을 다시 실행하고 collector를 새 ID로 재실행한다.

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
  --max-force-p99-n 1.0 \
  --max-group-force-p95-n 1.0 \
  --hard-max-force-error-n 2.0 \
  --benchmark-calls 2000
```

종료 코드 2는 artifact는 생성했지만 accuracy/runtime gate를 통과하지 못했다는
뜻이다.

## 8. 모델 성능 비교

모든 validation 결과를 표 형태로 확인한다.

```bash
jq -r '
  ["model","architecture","history","p99_N","p95_N","max_N","rmse_N"],
  (.ablations[] | [
    .name,
    .architecture,
    (.history|tostring),
    (.validation.force_norm_p99_n|tostring),
    (.validation.force_norm_p95_n|tostring),
    (.validation.force_norm_max_n|tostring),
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

정식 모델은 `approved=true`여야 한다. 현재 operator-selected 모델은 예외적으로 아래
model/metadata SHA가 정확히 일치해야 한다.

```bash
sha256sum "${FT_MODEL}" "${FT_MODEL_DIR}/metadata.json"
```

```text
model.ts       8c61261bb2fdd0151291f9c52ca627e59e04d71bc5655c92df5081943280ee8b
metadata.json  025d761ba285d34850dfe4da1ba9b89d6f7c2109f9a03181fdfbadb55463d882
```

승인 dataset과 runtime model의 `sample_hz`는 모두 `262.5`여야 한다.

## 9. 운용 허용 모델 observer-only 검증

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

Analyzer의 고정 운용 안전 계약은 FREE p95 `1.2 N`, p99 `1.5 N`, hard max
`2.5 N`, false CONTACT 0회, observer source age 20 ms, CSV gap 10 ms, CSV
평균 250 Hz, pose step 1 deg, velocity reversal 8 Hz, run당 최소 10초,
controlled CONTACT 최소 3회다. CLI 인자는 이 기준을 더 엄격하게 만들 수 있지만
느슨하게 만들 수 없다. 정식 모델 승격용 `FS-03`의 `1/1/2 N` 기준은 별도다.

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
  --max-free-force-p95-n 1.2 \
  --max-free-force-p99-n 1.5 \
  --max-free-force-error-n 2.5 \
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
즉시 중지하고 [문제 기록](problem/README.md)에 새 문제 파일을 추가한다. 40%에서도 FREE
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
  --max-free-force-p95-n 1.2 \
  --max-free-force-p99-n 1.5 \
  --max-free-force-error-n 2.5 \
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
ros2 topic info /contact_state/observer_input --verbose
```

확인해야 할 source는 다음과 같다.

```text
physical raw FT : /aft_sensor2/wrench
contact residual: /contact_observer/right/observation
robot state     : /contact_state/observer_input
```

작은 IL test episode를 먼저 저장하여 timestamp, frame, model hash, raw/predicted/
contact wrench가 모두 남는지 검증한 후 본 수집을 시작한다.

## 16. 통합 Feedback IL GUI test episode

이 단계는 leader와 follower를 구동할 수 있으므로 명시적으로 승인한 실기에서만
실행한다. 먼저 feedback OFF의 새 session으로 시작하며 기존 session에 이어 쓰지
않는다.

```bash
export FT_UMI_ROOT=/home/vision/chem_UMI-FT_ACP
export FT_UMI_PYTHON=/home/vision/venv_act/bin/python
export FT_UMI_RECORDER="${FT_UMI_ROOT}/UMIFT_Data/wired_collection/Python/chem_acp_raw_data_collection_lowhz.py"
export FT_UMI_CONFIG="${FT_UMI_ROOT}/UMIFT_Data/wired_collection/Python/chem_acp_raw_data_collection_lowhz_v2.yaml"
export FT_IL_DATA_DIR=/data/sata500
export FT_IL_SESSION=logistic_box_ft_feedback_YYYYMMDD_off_01

test -x "${FT_UMI_PYTHON}"
test -f "${FT_UMI_RECORDER}"
test -f "${FT_UMI_CONFIG}"
test -w "${FT_IL_DATA_DIR}"
findmnt -T "${FT_IL_DATA_DIR}"
```

`YYYYMMDD`를 실제 날짜로 바꾸고, `findmnt` 결과가 root filesystem이 아닌 전용
저장 장치인지 확인한다. 그다음 observer, teleop, 기존 UMI recorder와 기존
Feedback Leader Arm GUI를 한 번에 실행한다.

```bash
ros2 launch ft_fb_leaderarm ft_feedback_leader_data_collection.launch.py \
  umi_root:="${FT_UMI_ROOT}" \
  umi_python:="${FT_UMI_PYTHON}" \
  umi_recorder_script:="${FT_UMI_RECORDER}" \
  umi_recorder_config:="${FT_UMI_CONFIG}" \
  data_output_dir:="${FT_IL_DATA_DIR}" \
  data_session_name:="${FT_IL_SESSION}" \
  enable_d435:=false \
  model_path:="${FT_MODEL}" \
  zero_set_confirmed:=true \
  zero_set_id:=runtime_il_off_01 \
  payload_id:="${FT_PAYLOAD_ID}" \
  controller_config_hash:="${FT_CONTROLLER_HASH}" \
  learned_feedback_enable:=false
```

현재 기본 수집 카메라는 D405 하나이며 D435는 연결하지 않아도 된다. 나중에 D435를
다시 사용할 때만 장치를 연결하고 같은 launch에 `enable_d435:=true`를 지정한다.

동일한 observer, teleop 또는 recorder를 다른 launch로 동시에 실행하지 않는다.
GUI에서 작은 test episode 하나를 저장한 뒤 읽기 전용으로 검증한다. feedback OFF의
저장 stage는 `0.0`이다.

```bash
ros2 run ft_fb_leaderarm ft_il_episode_verify -- \
  --episode "${FT_IL_DATA_DIR}/${FT_IL_SESSION}/episode_000" \
  --model "${FT_MODEL}" \
  --expected-stage 0.0 \
  --output "${FT_EVIDENCE_DIR}/il_episode_off_01.json"

jq '{passed,failures,arrays}' "${FT_EVIDENCE_DIR}/il_episode_off_01.json"
```

`passed=true`인 episode만 사용한다. 40%와 100% 수집은 각각 앞 단계 authorization을
만든 뒤 `learned_feedback_enable:=true`, 해당 `feedback_gain_scale`과
`feedback_authorization`을 전달하고 새 session으로 실행한다.
