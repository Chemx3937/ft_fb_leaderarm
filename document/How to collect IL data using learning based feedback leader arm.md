# ft_fb_leaderarm으로 모방학습 데이터 취득하기

## 1. 문서 목적

이 문서는 `ft_fb_leaderarm` 저장소만으로 오른팔 leader teleoperation, learned
contact observer, D405 기반 UMI-compatible recorder와 통합 GUI를 실행하여
모방학습용 raw episode를 저장하는 현재 절차를 설명한다.

이전 `fb_leaderarm` 문서와 달리 다음 외부 소스 경로는 사용하지 않는다.

- `~/dualarm_ws/src/fb_leaderarm`
- `~/chem_UMI-FT_ACP`
- 외부 `chem_acp_raw_data_collection_lowhz.py`

현재 package가 소유하는 핵심 파일은 다음과 같다.

| 역할 | 현재 파일 |
|---|---|
| 통합 launch | `launch/ft_feedback_leader_data_collection.launch.py` |
| Contact observer | `ft_fb_leaderarm/observer_node.py` |
| Leader teleoperation | `src/single_impedance_*.cpp` |
| IL recorder | `ft_fb_leaderarm/il_data_recorder.py` |
| 통합 GUI | `scripts/ft_feedback_leader_data_collection_gui.py` |
| Recorder 설정 | `config/il_data_collection.yaml` |
| Episode 검증 | `ft_fb_leaderarm/il_episode_verification.py` |

상세 gate와 단계별 분석·승인 명령은 [acceptance contract](../docs/acceptance-contract.md)와
[실행 명령](command.md)을 따른다.

## 2. 현재 상태와 중요한 제한

### 2.1 구현 상태

다음 software 경로는 구현되어 있다.

- 현재 operator-selected free-space wrench model 실행
- Canonical `/contact_observer/right/observation` 발행
- Feedback OFF 상태의 leader/follower teleoperation
- D405, arm/hand/pose, physical/JT FT와 contact observation 동시 저장
- GUI의 상태 표시, episode 시작·저장·폐기·복구
- 저장된 episode의 model hash, feedback stage, 배열·timestamp 검증
- Feedback force norm `25 N` 포화와 기존 per-joint torque clip

그러나 구현 완료와 feedback 사용 승인은 다르다. 현재 계약상 실제 feedback ON 전에
다음 항목이 남아 있다.

1. `FS-06` FREE residual gate는 현재 task evidence에서 실패 상태다.
2. 정성 contact smoke는 통과했지만 독립 ground truth 기반 `CO-04`는 미완료다.
3. Feedback OFF FREE 3개와 controlled-CONTACT CSV를 analyzer로 통과시켜야 한다.
4. `off_to_40_analysis.json`이 `passed=true`여야 40% authorization을 만들 수 있다.
5. CONTACT rise time과 최대 torque step 기준은 아직 확정되지 않았다.

따라서 현재 문서의 기본 실행은 `learned_feedback_enable:=false`다. 40%와 100%는
각 단계 authorization을 만든 뒤에만 실행한다.

### 2.2 25 N feedback limit의 의미

`config/single_impedance_leader_damping.yaml`의 현재 값은 다음과 같다.

```yaml
contact_feedback_force_limit_N: 25.0
```

Ramp가 적용된 contact force norm이 `25 N`을 넘으면 6축 wrench 전체를
`25 / ||F||`로 비례 축소한 뒤 leader feedback torque를 계산한다. Contact 판정과
저장되는 raw residual은 축소하지 않는다.

이 값은 leader에 반사되는 feedback의 상한이다. Follower가 물체에 가하는 실제 힘을
`25 N` 이하로 제어하거나 충돌을 막는 기능은 아니다.

## 3. 현재 data flow

```text
SBC
  Doosan driver + V2 impedance controller
      └─ /contact_state/observer_input
  AFT sensor2
      └─ /aft_sensor2/wrench
  Hand controller
      └─ /joint_states

일반 PC의 통합 launch
  ft_contact_observer
      └─ /contact_observer/right/observation
          ├─ leader_teleop_node
          └─ package-owned IL recorder
  leader_teleop_node
      └─ /right_dsr_controller/task_space_command
  D405 (+ 선택적 D435)
  Feedback Leader Arm Data Collection GUI
```

동일한 observer, leader teleop 또는 recorder를 다른 launch로 동시에 실행하면 안 된다.

## 4. 매 실행 전 장비 확인

이 절차는 날짜가 바뀌거나 process를 재시작한 뒤 이전 상태를 그대로 가정하지 않는다.
운영자가 다음 항목을 직접 확인한다.

1. SBC robot driver가 `RUNNING`인지 확인한다.
2. V2 impedance controller가 `RUNNING`이고 설정이 모델 계약과 같은지 확인한다.
3. AFT sensor2가 `RUNNING`인지 확인한다.
4. 마지막 hardware zero-set 이후 AFT, driver 또는 controller 재시작 여부를 확인한다.
5. Tool, payload와 센서 주변 cable 배치가 모델 조건과 같은지 확인한다.
6. D405가 연결되어 있는지 확인한다. D435는 기본적으로 연결하지 않는다.
7. Hand를 저장한다면 hand controller와 오른손 15축 source가 정상인지 확인한다.
8. E-stop에 바로 접근할 수 있고 follower 작업공간에 사람·장애물이 없는지 확인한다.

### 4.1 SBC read-only 상태 확인

아래 명령은 운영자가 SBC에서 실행한다.

```bash
date -Ins

ros2 control list_controllers \
  -c /dsr01/controller_manager

pgrep -af \
  'dsr_controller2|aft_sensor.launch.py|aft_controller_manager|ros2_control_node'

ros2 topic info /aft_sensor2/wrench \
  --no-daemon --verbose

timeout 5s ros2 topic echo \
  /aft_sensor2/wrench --once
```

정상 조건은 다음과 같다.

- `dsr_controller2`: `active`
- `joint_state_broadcaster`: `active`
- `/aft_sensor2/wrench`: publisher 1개
- Wrench 값이 갱신되고 frame이 `aft_sensor2`

### 4.2 Hardware zero-set

Driver, controller 또는 AFT를 재시작했거나 baseline 조건이 달라졌다면 기존 zero-set을
재사용하지 않는다. Follower를 fixed zero pose
`[5.5, 52, 112, 28, -107, -35] deg`에 정지시키고 FT/tool을 완전히 무접촉으로 만든
뒤 운영자가 SBC에서 실행한다.

```bash
date -Ins

ros2 topic info /aft_sensor2/bias_setting --verbose

ros2 launch aft_can_hardware aft_zero_set2.launch.py \
  sensor_name:=aft_sensor2

date -Ins
```

출력의 `Hardware bias (tare) requested and acknowledged`와 clean exit를 확인하고,
실행 시각을 새 `zero_set_id`에 반영한다. 움직이거나 접촉한 상태에서는 실행하지 않는다.

## 5. 일반 PC 준비

### 5.1 Build와 Python dependency

소스나 설정을 바꾼 뒤 한 번 실행한다.

```bash
cd /home/vision/dualarm_ws
source /opt/ros/humble/setup.bash
source /home/vision/contact_pipeline_ws/install/setup.bash

colcon build --symlink-install \
  --packages-select ft_fb_leaderarm

source /home/vision/dualarm_ws/install/setup.bash
```

IL recorder Python은 기본적으로 `/home/vision/venv_act/bin/python`을 사용한다.
새 환경을 만들 때는 [requirements-il-collection.txt](../requirements-il-collection.txt)의
추가 dependency를 설치한다. Recorder와 GUI 이전 코드의 출처는
[third-party notice](../THIRD_PARTY_NOTICES.md)에 기록되어 있다.

### 5.2 ROS 환경

통합 launch를 실행할 일반 PC 터미널에서 다음을 실행한다.

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
```

### 5.3 모델과 장비 계약

현재 operator-selected model은 다음과 같다.

```bash
export FT_PACKAGE_ROOT=/home/vision/dualarm_ws/src/ft_fb_leaderarm
export FT_MODEL_DIR=/home/vision/.ros/ft_fb_leaderarm/models/right_train13_ridge_short_multiscale_bundle_v3_20260822
export FT_MODEL="${FT_MODEL_DIR}/model.ts"

export FT_PAYLOAD_ID=right_tool_m2p1kg_oz0p17m_v1
export FT_CONTROLLER_HASH=bae_r_v2_c113eabf7e13_ca07ae197213

sha256sum "${FT_MODEL}" "${FT_MODEL_DIR}/metadata.json"
```

기대 SHA-256은 다음과 같다.

```text
model.ts      8c61261bb2fdd0151291f9c52ca627e59e04d71bc5655c92df5081943280ee8b
metadata.json 025d761ba285d34850dfe4da1ba9b89d6f7c2109f9a03181fdfbadb55463d882
```

둘 중 하나라도 다르면 현재 authorization과 evidence를 재사용하지 않는다.

### 5.4 입력 topic 확인

```bash
ros2 param get /TorqueRtR \
  control.enable_observer_input_publish

ros2 param get /TorqueRtR \
  control.observer_input_frame_id

ros2 topic info /contact_state/observer_input \
  --no-daemon --verbose

ros2 topic info /aft_sensor2/wrench \
  --no-daemon --verbose

timeout 5s ros2 topic echo \
  /contact_state/observer_input --once

timeout 5s ros2 topic echo \
  /aft_sensor2/wrench --once
```

정상 조건은 observer input publish `True`, frame `right_base_link`, 각 입력 publisher
1개와 유효한 message다.

기존 통합 process가 남아 있지 않은지도 확인한다.

```bash
ros2 node list --no-daemon | grep -E \
  'ft_contact_observer|leader_teleop_node|chem_acp_raw_data_collection|feedback_leaderarm_data_collection_gui' \
  || true
```

새 launch 전에는 위 네 노드가 없어야 한다. `/aft_controller_manager` 이름 중복 경고는
ROS graph 문제로 별도 관리하며, 실제 AFT publisher가 하나인지 endpoint로 확인한다.

## 6. 새 session 준비

기존 episode와 섞이지 않도록 매 검증 단계마다 새 session 이름을 사용한다.

```bash
export FT_IL_RECORDER_PYTHON=/home/vision/venv_act/bin/python
export FT_IL_DATA_DIR=/data/sata500
export FT_IL_SESSION=logistic_box_ft_feedback_YYYYMMDD_off_01
export FT_ZERO_SET_ID=runtime_tare_YYYYMMDD_HHMMSS
export FT_EVIDENCE_DIR=/home/vision/.ros/ft_fb_leaderarm/evidence/right_current_model

test -x "${FT_IL_RECORDER_PYTHON}"
test -f "${FT_PACKAGE_ROOT}/config/il_data_collection.yaml"
test -f "${FT_MODEL}"
test -w "${FT_IL_DATA_DIR}"
findmnt -T "${FT_IL_DATA_DIR}"
df -hT "${FT_IL_DATA_DIR}"

test ! -e "${FT_IL_DATA_DIR}/${FT_IL_SESSION}" \
  && echo SESSION_NAME_AVAILABLE \
  || echo SESSION_ALREADY_EXISTS

mkdir -p "${FT_EVIDENCE_DIR}"
```

`YYYYMMDD_HHMMSS`는 실제 값으로 바꾼다. `findmnt` 결과는 root filesystem이 아닌
쓰기 가능한 전용 storage여야 한다. 기존 session을 삭제하거나 덮어쓰지 않는다.

## 7. Feedback OFF 통합 launch

첫 IL test와 feedback 적용 전 evidence는 반드시 feedback OFF로 실행한다.

```bash
ros2 launch ft_fb_leaderarm \
  ft_feedback_leader_data_collection.launch.py \
  recorder_python:="${FT_IL_RECORDER_PYTHON}" \
  data_output_dir:="${FT_IL_DATA_DIR}" \
  data_session_name:="${FT_IL_SESSION}" \
  require_output_mount:=true \
  enable_d435:=false \
  model_path:="${FT_MODEL}" \
  zero_set_confirmed:=true \
  zero_set_id:="${FT_ZERO_SET_ID}" \
  payload_id:="${FT_PAYLOAD_ID}" \
  controller_config_hash:="${FT_CONTROLLER_HASH}" \
  learned_feedback_enable:=false \
  smooth_teleop_enable:=true \
  keyboard_input_enabled:=false
```

이 launch는 다음을 한 번에 실행한다.

- `ft_contact_observer`
- `leader_teleop_node`
- `/chem_acp_raw_data_collection`
- `feedback_leaderarm_data_collection_gui`

Launch는 robot-related node를 시작하기 전에 recorder 설정, Python dependency, 출력 경로,
session 호환성을 preflight한다. `keyboard_input_enabled:=false`이므로 launch 터미널에
`q`를 입력해도 leader shutdown 명령이 아니다. 제어 키는 GUI에 focus를 둔 상태에서
누르거나 GUI 버튼을 사용한다.

### 7.1 정상 준비 상태

GUI 상단이 다음 상태가 될 때까지 로봇을 움직이지 않는다.

```text
Contact  FREE
Teleop   IDLE
Recorder READY
System   OK
```

Recorder가 준비되지 않으면 `Data Health`에서 각 modality의 Hz, age, camera retry와
queue/memory 상태를 확인한다. `Problem Logs`에는 원인과 권장 조치가 기록된다.

CLI로 확인할 때는 다음을 사용한다.

```bash
timeout 5s ros2 topic echo \
  /ft_contact_observer/diagnostics --once --field data

timeout 5s ros2 topic echo \
  /leader_teleop_node/status --once --field data

timeout 5s ros2 topic echo \
  /chem_acp_raw_data_collection/diagnostics --once --field data

ros2 topic info /contact_observer/right/observation \
  --no-daemon --verbose
```

Observer diagnostics는 `approved_model`, `model_ready`, `baseline_ready`,
`observer_ready`, `zero_verified`가 모두 true여야 한다. Recorder diagnostics는
`state=READY`, `startup_ready=true`, 모든 필수 modality `ok=true`여야 한다.

## 8. GUI 조작과 episode 저장

### 8.1 상태 전환

Leader를 잡은 상태에서 GUI 버튼 또는 GUI shortcut으로 실행한다.

1. `c`: `CURRENT`
2. `t`: `SLOW`
3. GUI가 `FAST` 진입 가능을 표시할 때까지 leader/follower 정렬을 유지
4. `o`: `FAST`
5. Recorder가 다시 `READY`인지 확인

`r`은 status가 realign을 요구할 때만 사용한다. 상태를 건너뛰거나 차단 메시지를
무시하지 않는다.

### 8.2 Episode

1. `1` 또는 `START EPISODE`를 눌러 기록을 시작한다.
2. Recorder가 `RECORDING`인지 확인한다.
3. 승인된 task 범위와 속도로 teleoperation한다.
4. `2` 또는 `STOP`을 누른다.
5. 정상 episode면 `중지 후 저장`, 문제가 있으면 `중지 후 폐기`를 선택한다.
6. `DRAINING`이 끝나고 Recorder가 다시 `READY`인지 확인한다.
7. 상단의 `clean/diag` 개수가 기대대로 증가했는지 확인한다.

기록 중 `z`는 GUI가 차단한다. Source stale, writer error 또는 memory critical로 자동
중지된 episode는 diagnostic으로 남을 수 있으며 학습용 clean episode로 간주하지 않는다.

### 8.3 정상 종료

1. RECORDING 중이면 `2`로 저장 또는 폐기를 먼저 결정한다.
2. `s`로 `PAUSE`하고 gravity scale 복원을 확인한다.
3. 작업공간을 확인한 뒤 필요한 경우에만 `z`로 `INIT POSE`를 실행한다.
4. `q` 또는 `SHUTDOWN`으로 leader를 안전 종료한다.
5. `0` 또는 `RECORDER EXIT`으로 recorder를 종료한다.
6. Launch 터미널에서 `Ctrl+C`를 눌러 남은 process를 종료한다.

비상 상황에서는 이 순서를 기다리지 말고 장비의 E-stop 절차를 따른다.

## 9. 카메라 계약

현재 기본값은 D405 하나다.

```text
D405: enabled
D435: disabled
```

D435를 연결하지 않은 정상 운용에서는 반드시 `enable_d435:=false`를 유지한다.
Recorder는 D405 RGB/depth가 준비될 때까지 재연결을 시도하며, readiness 전 반복 실패에는
해당 camera serial만 hardware reset할 수 있다. Recording 중에는 camera를 교체하거나
hardware reset하지 않는다.

향후 D435를 다시 사용할 때만 장치를 연결하고 새 session에서 다음 값을 사용한다.

```text
enable_d435:=true
```

D405-only session과 D405+D435 session을 같은 session directory에 섞지 않는다.

## 10. 저장 구조

기본 경로는 다음과 같다.

```text
<data_output_dir>/<data_session_name>/
  meta.json
  collection_logs/<launch timestamp>/events.jsonl
  episode_000/
    meta.json
    camera_0_D405/
    robot/
    ft/
    contact/
```

`enable_d435=true`인 episode에만 `camera_1_D435/`가 추가된다.

### 10.1 필수 배열

| 분류 | 주요 배열 | 형식/의미 |
|---|---|---|
| D405 | `camera_0_D405/rgb.zarr` | `uint8 (T,H,W,3)` RGB |
| D405 | `camera_0_D405/depth.zarr` | `uint16 (T,H,W)` aligned depth |
| Arm | `robot/joint_deg.zarr` | follower J1~J6, degree |
| Hand | `robot/hand_joint.zarr` | 오른손 15축, radian |
| Pose | `robot/controller_current_pose_se3.zarr` | current TCP, `(T,4,4)` |
| Pose | `robot/command_pose_se3.zarr` | controller desired TCP, `(T,4,4)` |
| Action | `robot/command_quat_pose_se3.zarr` | teleop command, `(T,4,4)` |
| Physical FT | `ft/wrench_raw.zarr` | sensor-frame `[F,M]`, `(T,6)` |
| JT FT | `ft/jt_tared_wrench.zarr` | controller raw external wrench |
| JT FT | `ft/jt_tared_filtered_wrench.zarr` | controller filtered wrench |
| Prediction | `contact/free_space_wrench_prediction.zarr` | learned free-space prediction |
| Residual | `contact/contact_wrench.zarr` | canonical contact residual |
| State | `contact/contact_state.zarr` | `0=FREE`, `1=CONTACT` |
| Validity | `contact/contact_valid.zarr` | observation validity mask |
| Readiness | `contact/contact_model_ready.zarr` | model-ready mask |

각 stream에는 대응 timestamp가 저장된다. Contact에는 source/receive timestamp,
source sequence와 prediction age도 저장된다. `meta.json`에는 source topic/frame,
camera 역할·calibration, model SHA-256, feedback stage, writer 상태와 sample count가 남는다.

Recorder의 목표 저장률은 camera/robot/command `30 Hz`, physical/JT FT와 canonical
contact `262.5 Hz`다. 실제 source publish rate와 저장률은 같다고 가정하지 않고 episode
metadata와 verifier 결과로 확인한다.

## 11. 저장 episode 검증

저장된 실제 episode 번호를 먼저 확인한다.

```bash
find "${FT_IL_DATA_DIR}/${FT_IL_SESSION}" \
  -maxdepth 1 -type d -name 'episode_[0-9]*' \
  -printf '%f\n' | sort
```

Feedback OFF episode의 stage는 `0.0`이다.

```bash
export FT_IL_EPISODE="${FT_IL_DATA_DIR}/${FT_IL_SESSION}/episode_000"
export FT_IL_REPORT="${FT_EVIDENCE_DIR}/il_episode_off_$(date +%Y%m%d_%H%M%S).json"

test ! -e "${FT_IL_REPORT}"

ros2 run ft_fb_leaderarm ft_il_episode_verify -- \
  --episode "${FT_IL_EPISODE}" \
  --model "${FT_MODEL}" \
  --expected-stage 0.0 \
  --output "${FT_IL_REPORT}"

sed -n '1,280p' "${FT_IL_REPORT}"
sha256sum "${FT_IL_REPORT}"
```

`passed=true`, `failures=[]`인 episode만 학습 데이터 후보로 사용한다. Verifier는 다음을
검사한다.

- 현재 model hash와 저장 hash 일치
- Feedback stage 일치
- D405 및 선택적 D435 역할 일치
- Arm/hand/pose, physical/JT FT, prediction/residual/state 배열 존재와 shape
- Timestamp finite, strictly increasing, 배열 row count 일치
- Writer error와 interruption 없음

## 12. Feedback 40%와 100% IL 수집

Feedback ON launch는 authorization 없이 실행되지 않는다. Feedback OFF FREE 3개와
controlled-CONTACT CSV를 먼저 [실행 명령](command.md)의 analyzer로 평가한다.

OFF→40 report가 통과하고 40% authorization을 만든 뒤 새 session으로 실행한다.

```bash
ros2 launch ft_fb_leaderarm \
  ft_feedback_leader_data_collection.launch.py \
  recorder_python:="${FT_IL_RECORDER_PYTHON}" \
  data_output_dir:="${FT_IL_DATA_DIR}" \
  data_session_name:="${FT_IL_SESSION}" \
  enable_d435:=false \
  model_path:="${FT_MODEL}" \
  zero_set_confirmed:=true \
  zero_set_id:="${FT_ZERO_SET_ID}" \
  payload_id:="${FT_PAYLOAD_ID}" \
  controller_config_hash:="${FT_CONTROLLER_HASH}" \
  learned_feedback_enable:=true \
  feedback_gain_scale:=0.40 \
  feedback_authorization:="${FT_EVIDENCE_DIR}/feedback_40_authorization.json" \
  smooth_teleop_enable:=true \
  keyboard_input_enabled:=false
```

40% episode verifier에는 `--expected-stage 0.40`을 사용한다. 100%는 40% FREE/CONTACT
분석 통과와 chained authorization 뒤에만 실행하고 verifier에는
`--expected-stage 1.00`을 사용한다.

단계별 원칙은 다음과 같다.

```text
Feedback OFF evidence
  → OFF→40 analyzer GO
  → 40% authorization
  → 40% 제한 실기와 evidence
  → 40→100 analyzer GO
  → 100% authorization
  → 100% 제한 실기
```

## 13. 문제 해결

### Recorder가 OFFLINE

```bash
pgrep -af '[p]ython.*ft_fb_leaderarm.il_data_recorder'

ros2 topic info \
  /chem_acp_raw_data_collection/diagnostics \
  --no-daemon --verbose

timeout 5s ros2 topic echo \
  /chem_acp_raw_data_collection/diagnostics \
  --once --field data

ros2 service list | grep \
  '/chem_acp_raw_data_collection/'
```

Process, diagnostics publisher와 `start_episode`, `stop_save`, `stop_discard`, `recover`,
`shutdown` service가 모두 있어야 한다.

### Camera가 준비되지 않음

- D405 serial과 USB 연결을 확인한다.
- D435를 연결하지 않았다면 launch가 `enable_d435:=false`인지 확인한다.
- GUI `Data Health`의 `camera_retry_counts`와 `camera_retry_errors`를 확인한다.
- Recording 중에는 cable 재연결이나 reset을 시도하지 말고 episode를 먼저 중지한다.

### Pending 또는 writer 오류

정상 episode를 시작하기 전에 `DRAINING`이 끝날 때까지 기다린다. `p`/`RECOVER`는
RECORDING 중에는 차단되며 pending episode를 폐기할 수 있으므로 GUI 확인창을 읽고
운영자가 결정한다. `.episode_*_recording`을 자동 삭제하거나 기존 episode 번호를
강제로 재사용하지 않는다.

### GUI 키가 동작하지 않음

통합 launch의 leader terminal keyboard 입력은 기본 비활성이다. GUI 창에 focus를 주고
GUI shortcut을 누르거나 해당 버튼을 사용한다. Launch terminal은 node 로그 확인과 최종
`Ctrl+C`에만 사용한다.

### 중복 node 경고

```bash
ros2 node list --no-daemon | sort | uniq -d
```

동일 이름의 observer, teleop, recorder 또는 GUI가 있으면 새 launch를 시작하지 않는다.
기존 process의 소유 terminal에서 정상 종료한 뒤 endpoint publisher 수를 다시 확인한다.

## 14. 이 저장소의 범위

이 저장소는 free-space wrench 데이터 수집·학습, contact observer, feedback leader
teleoperation과 UMI-compatible raw episode 수집·검증을 제공한다. 모방학습 데이터의
후처리 converter와 policy training/inference stack은 현재 범위에 포함하지 않는다.

실패나 재현 가능한 문제는 [문제 기록](problem/README.md)에 남긴다. 현재 저장소의
source/config/launch와 이 문서가 다르면 source와 테스트를 기준으로 확인한 뒤 같은 변경에서
문서를 갱신한다.
