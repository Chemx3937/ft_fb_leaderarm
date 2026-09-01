#!/usr/bin/env python3
"""
UMI-compatible raw data recorder owned by ``ft_fb_leaderarm``.

Derived from the REAL lab Chem ACP recorder. See ``THIRD_PARTY_NOTICES.md``.
-----------------------
RGB/Depth: D405/D435
Joint/EE-pose: /dsr01/joint_states 토픽
Controller current/desired: direct topic 또는 /bae_r/observer_input aggregate
Quaternion command: /right_dsr_controller/task_space_command
FT @ FT sensor Frame: /aft_sensor2/wrench 토픽
FT @ base frame: /aft_sensor2/wrench_base 토픽
JT wrench: direct topic 또는 /bae_r/observer_input aggregate
-----------------------

이 스크립트는 실제 Doosan right arm teleoperation 중 D405/D435 RGB-depth,
right arm joint, FK End-Effector pose, command pose, End-Effector F/T wrench를
episode 단위로 기록한다. 각 optional stream은 `record_*` 설정으로 결정한다.
command pose topic을 사용하지 않을 때는
--no-cmd-pose를 주면 robot/command_pose_se3.zarr에 현재 EE pose를 대신 저장한다.

아래에서 "기본"이라고 쓴 값은 YAML을 생략했을 때의 CLI built-in default다.
실제 권장 실행에서는
`config/il_data_collection.yaml`이 그 값을 덮어쓰며, 최종 CLI
argument가 다시 가장 높은 우선순위를 갖는다.

RealSense depth는 기본적으로 RGB pixel grid에 맞춰 저장한다. 내부적으로
pyrealsense2의 rs.align(rs.stream.color)를 적용한 뒤 depth.zarr에 기록하며,
이 aligned depth가 목표 Hz를 만족해야 episode recording control로 넘어간다.
RGB-depth align을 끄고 raw depth grid를 저장하려면 --no-align-depth-to-color를
명시적으로 준다.

FT raw/payload-gravity/compensated 배열은 각각의 `record_ft_wrench_*` 설정이
켰을 때만 저장한다. Payload compensated output을 켠 경우 sensor raw 값에서 URDF
기반 payload/tool gravity wrench만 빼며, episode 시작 전 별도 zero/bias
subtraction은 하지 않는다. 계산식은
`wrench_comp_payload = wrench_raw - wrench_payload_gravity`이다.

FT 저장 schema 변경 사항
-----------------------
- 예전 `ft/wrench.zarr` 이름은 새 episode에서 더 이상 생성하지 않는다.
- 같은 값은 `ft/wrench_comp_payload.zarr`에 저장한다.
- timestamp는 기존과 동일하게 `ft/wrench_time_stamps.zarr`를 공유한다.
- readiness/health 출력의 `ft_wrench`는 내부 modality label이며, 실제 저장
  파일명은 `wrench_raw.zarr`, `wrench_payload_gravity.zarr`,
  `wrench_comp_payload.zarr`이다.
- `ft/wrench_base.zarr`는 recorder가 직접 URDF/FK로 변환하지 않고,
  compliant controller가 publish하는 `/aft_sensor2/wrench_base` WrenchStamped를
  별도 구독해서 저장한다. built-in target Hz는 `--ft-base-hz 350`이다.
- built-in과 두 제공 YAML 모두 base wrench 저장은 OFF다. 필요할 때
  `--record-ft-base`로 켠다.

Modality별 데이터 소스 요약
--------------------------
- camera_0_D405/rgb, camera_1_D435/rgb:
  --d405-serial, --d435-serial로 지정한 Intel RealSense D405/D435의
  pyrealsense2 rs.stream.color rgb8 stream에서 취득한다.
  alias parser는 --camera-0-serial, --camera-1-serial이다.
  저장 Hz는 --camera-fps, 기본 60 Hz.
- camera_0_D405/depth, camera_1_D435/depth:
  같은 RealSense device의 pyrealsense2 rs.stream.depth z16 stream에서 취득한다.
  depth align ON이면 rs.align(rs.stream.color) 이후의 depth frame을 저장한다.
  저장 Hz는 --camera-fps, 기본 60 Hz.
- robot/joint_deg:
  ROS2 JointState --joint-topic, 기본 /dsr01/joint_states의 position field.
  right_joint_1~6 또는 joint_1~6 순서로 읽고 degree로 변환해 저장한다.
  alias parser는 --robot-joint-topic이다.
  저장 Hz는 --robot-sample-hz, 기본 60 Hz.
- robot/hand_joint:
  ROS2 JointState --hand-joint-topic, 기본 /joint_states의 position field.
  custom_robot_utils.py의 JOINT_NAMES_HAND_RIGHT 순서인
  right_thumb_joint1~3, right_index_joint1~3, right_middle_joint1~3,
  right_ring_joint1~3, right_baby_joint1~3의 15개 값을 radian 단위로 저장한다.
  저장 Hz는 --robot-sample-hz, 기본 60 Hz. --no-hand로 끌 수 있다.
- robot/ee_pose_fk_se3:
  joint_deg와 같은 JointState position을 source로 하며, --follower-urdf를
  Pinocchio FK에 넣어 --base-frame 기준 --ee-frame pose를 계산한다.
  source parser는 --joint-topic 또는 --robot-joint-topic이다.
  저장 Hz는 --robot-sample-hz, 기본 60 Hz.
- robot/command_pose_se3:
  ROS2 PoseStamped 또는 Float64MultiArray --command-topic.
  --no-cmd-pose 사용 시 command topic 대신 같은 시점의 ee_pose_fk feedback을 복사한다.
  alias parser는 --robot-command-topic이다.
  /bae_r/desired_pose 같은 Float64MultiArray는 [x,y,z,a,b,c]로 읽고,
  command_float64_euler_order에 따라 orientation을 변환한다.
  저장 Hz는 command topic 사용 시 --command-hz, 기본 60 Hz이며,
  --no-cmd-pose 사용 시 --robot-sample-hz를 따른다.
- robot/contact_state:
  ROS2 Int32 --contact-state-topic, 기본 /leader_teleop_node/contact_state.
  leader teleop의 contact state를 저장한다. 값은 -1=no contact, 1=contact이다.
  저장 Hz는 --contact-state-hz, 기본 60 Hz. --record-contact-state로 켠다.
- robot/contact_phase:
  ROS2 Int32 --contact-phase-topic, 기본 /leader_teleop_node/contact_phase.
  contact_state에 pre-contact를 추가한 phase를 저장한다.
  값은 -1=no contact, 0=pre-contact, 1=contact이다.
  저장 Hz는 --contact-phase-hz, 기본 60 Hz. --record-contact-phase로 켠다.
- ft/wrench_raw:
  ROS2 WrenchStamped --ft-topic, 기본 /aft_sensor2/wrench에서 받은
  F/T sensor frame 기준 raw wrench.
  alias parser는 --ft-raw-topic이다.
  저장 Hz는 --ft-hz, 기본 350 Hz.
- ft/wrench_payload_gravity:
  센서에서 직접 받는 값이 아니라 --follower-urdf inertial, latest joint,
  --ft-frame, --ft-payload-root로 계산한 payload/tool gravity wrench이다.
  저장 Hz는 --ft-hz, 기본 350 Hz.
- ft/wrench_comp_payload:
  학습용 payload gravity compensated wrench이며
  wrench_raw - wrench_payload_gravity로 계산한다.
  저장 Hz는 --ft-hz, 기본 350 Hz.
- ft/wrench_base:
  ROS2 WrenchStamped --ft-base-topic, 기본 /aft_sensor2/wrench_base에서 받은
  base frame 기준 FT wrench. --record-ft-base로 켠다.
  저장 Hz는 --ft-base-hz, 기본 350 Hz.
- ft/jt_tared_wrench:
  ROS2 Float64MultiArray --jt-tared-wrench-topic, 기본 /right/F_e_raw의 첫 6개 원소.
  alias parser는 --jt-raw-wrench-topic이다.
  compliant controller가 joint torque 기반으로 계산한 controller-side wrench이다.
  저장 Hz는 --jt-tared-wrench-hz, 기본 350 Hz.
- ft/jt_tared_filtered_wrench:
  ROS2 Float64MultiArray --jt-tared-filtered-wrench-topic, 기본 /right/F_e의 첫 6개 원소.
  alias parser는 --jt-filtered-wrench-topic 또는 --jt-wrench-topic이다.
  compliant controller의 filtered controller-side wrench이다.
  저장 Hz는 --jt-tared-filtered-wrench-hz, 기본 350 Hz.
- jt/joint_torque:
  ROS2 JointState --joint-torque-topic, 기본 /dsr01/joint_states의 effort field.
  --joint-torque-topic을 생략하면 --joint-topic과 같은 topic을 사용한다.
  --no-jt 사용 시 기록하지 않는다.
  저장 Hz는 --robot-sample-hz, 기본 60 Hz.
- *_time_stamps:
  PC host timestamp(time.time())이며, RealSense frame은 별도로 hardware timestamp와
  frame number도 RealSense frame metadata에서 함께 저장한다.

RAM 사용을 줄이기 위해 recording 중에는 데이터를 가능한 한 바로 disk에
append/write한다. RAM에는 latest sample과 episode 시작 직전 pre-roll ring
buffer만 유지한다. acquisition callback이 disk I/O에 막히지 않도록 실제
disk write는 background writer thread가 처리하며, --writer-queue-size로
최대 대기 sample item 수를 제한한다. `--writer-queue-max-bytes`는 queue의
ndarray payload hard cap이며, 완전한 pre-roll보다 작으면 hardware 시작 전에
설정 오류로 종료한다.

실행 전 조건
-----------
1. 실제 data 취득 PC에서 ROS2/Doosan workspace를 source한다.
2. 아래 Python package가 설치된 환경을 활성화한다.
   - rclpy
   - pyrealsense2
   - pinocchio 또는 pin
   - zarr
   - numpy
   - opencv-python
3. D405/D435 serial number를 확인한다.

   ros2 run ft_fb_leaderarm ft_il_data_collect -- --list-realsense

기본 실행 예시
-------------
ros2 run ft_fb_leaderarm ft_il_data_collect -- \
  --config-yaml /path/to/ft_fb_leaderarm/config/il_data_collection.yaml

두 제공 YAML 모두 depth align이 ON이므로 camera_*/depth.zarr에는 color pixel
grid 기준 aligned depth가 저장된다. raw depth pixel grid를 저장하려면 최종 CLI
override로 아래 옵션을 추가한다.

  --no-align-depth-to-color


command pose topic을 사용하지 않고 현재 EE pose를 command pose로 저장하려면
마지막에 아래 옵션을 추가한다.

  --no-cmd-pose


시작 readiness check
--------------------
프로그램이 실행되면 camera pipeline, ROS subscriber, robot sampler를 먼저 시작하고,
episode control을 바로 열지 않는다. 그 대신 모든 modality가 정상인지 주기적으로
출력하면서 확인한다.

확인 대상:
  - camera_0_rgb, camera_0_depth
  - camera_1_rgb, camera_1_depth
  - robot_joint, robot_ee_pose_fk (--no-ee-pose-fk를 주면 FK pose 제외)
  - robot_hand_joint (--no-hand를 주면 제외)
  - jt_joint_torque (--no-jt를 주면 제외)
  - ft_wrench
  - ft_base_wrench (--no-ft-base를 주면 제외)
  - jt_tared_wrench (/right/F_e_raw에서 받은 controller-side wrench)
  - jt_tared_filtered_wrench (/right/F_e에서 받은 controller-side wrench)
  - command_pose (--no-cmd-pose를 주면 command topic check는 제외)
  - contact_state (--record-contact-state 사용 시)
  - contact_phase (--record-contact-phase 사용 시)

기본 동작:
  1. 모든 modality가 목표 Hz, latest sample age 조건을 만족하는지 확인한다.
     기본 --hz-min-ratio는 0.98이므로 60 Hz target은 58.8 Hz 이상이어야 한다.
  2. 문제가 없으면 --startup-check-count 횟수만큼 연속으로 READY를 확인한다.
     기본값은 10회다.
  3. 한 번이라도 문제가 있으면 NOT READY로 어떤 modality가 문제인지 출력하고,
     modality별 retry count를 증가시킨다.
  4. 최초 연결 때부터 D405/D435가 모두 시작될 때까지
     --camera-reconnect-period-sec 간격으로 실패한 camera만 다시 시도한다.
     시작 뒤 문제가 지속되어도 해당 RealSense pipeline을 restart한다.
  5. ROS topic 계열(joint, JT, FT, command)은 subscriber를 재생성하지 않고
     data가 다시 들어올 때까지 기다린다.
  6. 문제가 발생한 뒤 복구되면 --recovery-check-count 횟수만큼 연속으로 READY를
     확인한다. 기본값은 5회다.
  7. readiness가 통과한 뒤에야 "Controls: 1=start episode..."가 출력되고,
     그때부터 1/2/0 입력을 받는다.

Depth align과 readiness의 관계:
  - 기본값은 --align-depth-to-color ON이다.
  - camera loop에서 rs.align(rs.stream.color)를 적용한 뒤 latest_depth와
    depth_pre ring buffer에 넣는다.
  - 따라서 READY 출력의 camera_0_depth, camera_1_depth Hz는 raw depth가 아니라
    실제 저장될 aligned depth 기준 Hz다.
  - align이 켜져 있는데 저장될 depth frame shape이 color resolution과 다르면
    aligned depth frame shape mismatch error가 발생하고 READY로 넘어가지 않는다.
  - --no-align-depth-to-color를 주면 rs.align을 만들지 않고 raw depth frame을
    그대로 저장한다. 이때 metadata의 depth_aligned_to_color는 False가 된다.

출력 예시:
  READY: consecutive 3/10
    camera_0_rgb         59.8 Hz, age=0.010s     ok, retry=0, camera_restart=0
    camera_0_depth       59.7 Hz, age=0.012s     ok, retry=0, camera_restart=0
    camera_1_rgb         60.1 Hz, age=0.009s     ok, retry=0, camera_restart=0
    camera_1_depth       60.0 Hz, age=0.011s     ok, retry=0, camera_restart=0
    robot_joint          60.0 Hz, age=0.008s     ok, retry=0
    robot_hand_joint     60.0 Hz, age=0.008s     ok, retry=0
    robot_ee_pose_fk     60.0 Hz, age=0.008s     ok, retry=0
    jt_joint_torque      60.0 Hz, age=0.008s     ok, retry=0
    ft_wrench            349.5 Hz, age=0.003s    ok, retry=0
    ft_base_wrench       349.5 Hz, age=0.003s    ok, retry=0
    command_pose         60.0 Hz, age=0.010s     ok, retry=0

키보드 조작
-----------
1:
    새 episode 기록 시작.
    모든 modality가 연결되어 있고, 최근 수신 Hz가 기준을 만족해야 시작된다.
    --no-cmd-pose를 주면 command topic 대신 현재 EE pose를 command pose로 저장한다.
2:
    현재 episode 기록 중단.
    이후 y/n으로 해당 episode 저장 여부를 묻는다.
    저장/폐기 이후 idle 상태로 돌아간다.
0:
    raw data 취득 종료.
    기록 중인 episode가 있으면 먼저 저장 여부를 묻는다.

Raw data 저장 구조
------------------
<output-dir>/<session-name>/
  meta.json
    session 전체 args, camera_roles, camera_calibration, ft_processing.

  episode_000/
    meta.json
      episode count, frame 정보, camera_calibration, camera_timing,
      ft_processing, unit 정보를 저장한다.

    camera_0_D405/
      rgb.zarr
      rgb_time_stamps.zarr
      rgb_hardware_time_stamps_ms.zarr
      rgb_frame_numbers.zarr
      depth.zarr
      depth_time_stamps.zarr
      depth_hardware_time_stamps_ms.zarr
      depth_frame_numbers.zarr

    camera_1_D435/
      rgb.zarr
      rgb_time_stamps.zarr
      rgb_hardware_time_stamps_ms.zarr
      rgb_frame_numbers.zarr
      depth.zarr
      depth_time_stamps.zarr
      depth_hardware_time_stamps_ms.zarr
      depth_frame_numbers.zarr

    robot/
      joint_deg.zarr
      joint_time_stamps.zarr
      hand_joint.zarr
      hand_joint_time_stamps.zarr
      ee_pose_fk_se3.zarr                 # record_ee_pose_fk=true일 때만
      ee_pose_fk_time_stamps.zarr         # record_ee_pose_fk=true일 때만
      controller_current_pose_se3.zarr
      controller_current_pose_time_stamps.zarr
      command_pose_se3.zarr
      command_time_stamps.zarr

    ft/
      wrench_raw.zarr
      wrench_payload_gravity.zarr
      wrench_comp_payload.zarr
      wrench_time_stamps.zarr
      wrench_base.zarr
      wrench_base_time_stamps.zarr
      jt_tared_wrench.zarr
      jt_tared_wrench_time_stamps.zarr
      jt_tared_filtered_wrench.zarr
      jt_tared_filtered_wrench_time_stamps.zarr
      # 새 episode에서는 wrench.zarr를 생성하지 않는다.

    jt/
      joint_torque.zarr
      joint_torque_time_stamps.zarr
      # --no-jt를 주면 jt/ directory는 생성되지 않는다.

저장되는 data 의미
-----------------
camera_0_D405/rgb.zarr, camera_1_D435/rgb.zarr:
    RGB stream. uint8 RGB order, shape (T, H, W, 3).
    목표 Hz는 --camera-fps. 기본 해상도는 640x480이므로 기본 shape는
    (T, 480, 640, 3)이다.
camera_*/rgb_time_stamps.zarr:
    PC host timestamp. 단위는 second, time.time() 기준.
camera_*/rgb_hardware_time_stamps_ms.zarr:
    RealSense color frame.get_timestamp(). 단위는 millisecond.
camera_*/rgb_frame_numbers.zarr:
    RealSense color frame.get_frame_number().
camera_*/depth.zarr:
    depth stream. uint16 raw depth units, shape (T, H, W).
    meter 변환은 후처리에서 depth_meter = depth_raw * depth_scale로 수행한다.
    기본적으로 RealSense rs.align(rs.stream.color)를 적용한 color pixel grid
    기준 depth를 저장한다. --no-align-depth-to-color를 주면 raw depth pixel
    grid 기준 depth를 저장한다.
    align ON이면 shape는 color stream resolution, 기본 480x640이다.
    align OFF이면 shape는 depth stream resolution, 기본 480x640이다.
camera_*/depth_time_stamps.zarr:
    PC host timestamp. 단위는 second.
camera_*/depth_hardware_time_stamps_ms.zarr:
    RealSense depth frame.get_timestamp(). 단위는 millisecond.
camera_*/depth_frame_numbers.zarr:
    RealSense depth frame.get_frame_number().

robot/joint_deg.zarr:
    /dsr01/joint_states.position을 right arm 6개 joint 순서로 정렬한 값.
    단위는 degree, shape (T, 6).
robot/hand_joint.zarr:
    /joint_states 또는 --hand-joint-topic의 JointState.position에서
    custom_robot_utils.py의 JOINT_NAMES_HAND_RIGHT 순서로 정렬한 오른손 15개 값.
    단위는 radian, shape (T, 15).
robot/hand_joint_time_stamps.zarr:
    hand_joint.zarr의 PC host timestamp. 단위는 second.
robot/ee_pose_fk_se3.zarr:
    right_base_link 기준 right_link_6 absolute pose.
    형식은 4x4 SE(3), translation 단위는 meter, shape (T, 4, 4).
    --no-ee-pose-fk이면 저장하지 않는다. V2는 controller current pose를 기준으로
    사용하므로 기본 V2 YAML에서 이 중복 stream을 끈다.
robot/command_pose_se3.zarr:
    /right_dsr_controller/task_space_command의 PoseStamped command.
    --command-position-unit 기본값은 mm이며 meter로 변환해 저장한다.
    형식은 4x4 SE(3), shape (T, 4, 4).
    --no-cmd-pose를 주면 command topic을 구독하지 않고, 같은 시점의
    robot/ee_pose_fk_se3.zarr 값을 command_pose로 복사해 저장한다.

ft/wrench_raw.zarr:
    /aft_sensor2/wrench에서 받은 F/T sensor frame 기준 raw wrench.
    형식은 [Fx, Fy, Fz, Mx, My, Mz], shape (T, 6).
ft/wrench_payload_gravity.zarr:
    URDF inertial과 현재 joint로 계산한 FT sensor 아래 payload/tool 자중 wrench.
    이 출력을 요청하면 payload gravity compensation, 유효한 payload root/model,
    최신 joint가 모두 필요하다. 설정·초기화·실시간 계산 중 하나라도 실패하면
    0으로 대체하지 않고 취득을 fail-closed로 중단한다. 출력을 끄면 배열을 만들지 않는다.
ft/wrench_comp_payload.zarr:
    학습에 사용할 payload gravity compensated wrench.
    계산식은 wrench_raw - wrench_payload_gravity.
    이 값은 이전 schema의 ft/wrench.zarr에 해당하며, 새 기록에서는 명확한 이름을
    위해 wrench_comp_payload.zarr로 저장한다.
    FT sensor frame orientation이 right_link_6와 같다고 가정하고 frame 변환은 하지 않는다.
    FT topic이 --ft-hz보다 빠르게 publish되면 저장은 --ft-hz 기준으로 downsample된다.
    예를 들어 topic이 1000 Hz이고 --ft-hz 350이면 약 3개 중 1개 sample만 저장한다.
ft/wrench_base.zarr:
    /aft_sensor2/wrench_base에서 받은 base frame 기준 FT wrench.
    compliant controller가 raw FT sensor wrench를 base frame 축 기준으로 변환하고
    moment 기준점을 TCP origin으로 옮겨 publish한 WrenchStamped를 그대로 저장한다.
    기본 target Hz는 --ft-base-hz 350이며, --no-ft-base를 주면 저장하지 않는다.
ft/wrench_base_time_stamps.zarr:
    ft/wrench_base.zarr의 PC host timestamp. 단위는 second.
ft/jt_tared_wrench.zarr:
    compliant controller의 /right/F_e_raw에서 받은 JT 기반 외력 wrench.
    형식은 [Fx, Fy, Fz, Mx, My, Mz], shape (T, 6).
    /right/F_e_raw가 --jt-tared-wrench-hz보다 빠르게 publish되면 저장은
    --jt-tared-wrench-hz 기준으로 downsample된다.
ft/jt_tared_filtered_wrench.zarr:
    compliant controller의 /right/F_e에서 받은 JT 기반 filtered 외력 wrench.
    형식은 [Fx, Fy, Fz, Mx, My, Mz], shape (T, 6).
    /right/F_e가 --jt-tared-filtered-wrench-hz보다 빠르게 publish되면 저장은
    --jt-tared-filtered-wrench-hz 기준으로 downsample된다.

jt/joint_torque.zarr:
    --joint-torque-topic의 JointState.effort에 들어있는 6개 joint torque sensing 값.
    --joint-torque-topic을 생략하면 --joint-topic과 같은 topic을 사용한다.
    shape (T, 6).
    --no-jt를 주면 저장하지 않고 readiness/health check에서도 제외한다.

Camera calibration metadata
---------------------------
session meta.json과 episode meta.json의 camera_calibration에는 D405/D435 각각에
대해 아래 항목이 저장된다.

- color_stream, depth_stream: width, height, fps, format.
- depth_scale: raw uint16 depth 값에 곱하면 meter가 되는 값.
- depth_stored_as: "uint16_depth_units".
- depth_aligned_to_color: RGB pixel grid 기준 depth align 적용 여부.
- depth_alignment_method:
  align ON이면 "pyrealsense2.align(rs.stream.color)", align OFF이면 null.
- stored_depth_stream: 실제 depth.zarr가 따르는 pixel grid 정보.
  align ON이면 color stream width/height, align OFF이면 depth stream width/height.
- stored_depth_intrinsics: 실제 저장된 depth image의 pixel grid intrinsics.
  align을 켜면 color intrinsics이고, 끄면 depth intrinsics다.
- color_intrinsics, depth_intrinsics:
  width, height, fx, fy, ppx, ppy, distortion model, coeffs.
- depth_to_color_extrinsics, color_to_depth_extrinsics:
  RealSense stream 사이의 rotation/translation.

FT processing metadata
----------------------
session meta.json과 episode meta.json의 ft_processing에는 아래 항목이 저장된다.

- formula: wrench_comp_payload = wrench_raw - wrench_payload_gravity.
- payload_gravity_comp_enabled.
- payload_status: payload model enabled/disabled reason.
- payload_root, payload_gravity_sign, ft_frame.

학습용 zarr 변환 시 사용할 raw key
-------------------------------
- robot/controller_current_pose_se3.zarr -> ts_pose_fb_0 우선 source, xyz+qwxyz.
- robot/ee_pose_fk_se3.zarr -> FK diagnostic/fallback source, xyz+qwxyz.
- robot/command_quat_pose_se3.zarr -> ts_pose_command_0 source, xyz+qwxyz.
- robot/command_pose_se3.zarr -> legacy/diagnostic desired pose.
- robot/contact_state.zarr -> leader teleop contact label, -1=no contact, 1=contact.
- robot/contact_phase.zarr -> contact phase label, -1=no contact, 0=pre-contact, 1=contact.
- robot/hand_joint.zarr -> right hand joint state, radian, 15 dim.
- ft/wrench_comp_payload.zarr -> wrench_0 또는 UMI-FT 호환 key.
- ft/wrench_base.zarr -> wrench_base_0 등 base-frame FT 분석용 key.
- ft/jt_tared_wrench.zarr -> controller_jt_tared_wrench_0 등 비교/분석용 key.
- ft/jt_tared_filtered_wrench.zarr -> controller_jt_tared_filtered_wrench_0 등 비교/분석용 key.
- camera_*/rgb.zarr -> rgb_0, rgb_1.
- camera_*/depth.zarr + depth_scale -> depth_0, depth_1.
- rgb/depth timestamp -> map_to_d_idx_0, map_to_d_idx_1는 후처리에서 생성.
- 후처리 계산 -> ts_pose_virtual_target_0, stiffness_0.
- gripper는 이 환경에 없으므로 학습 config/action shape에서 제거해야 한다.

Parser option 설명
------------------
--list-realsense
    연결된 RealSense camera의 serial number와 model name을 출력하고 종료한다.
--d405-serial, --camera-0-serial
    D405 serial number. 이 camera는 항상 camera_0_D405로 저장된다.
--d435-serial, --camera-1-serial
    D435 serial number. 이 camera는 항상 camera_1_D435로 저장된다.
--output-dir
    session들이 저장될 root directory. 기본값은
    ~/.ros/ft_fb_leaderarm/il_data.
--session-name
    output-dir 아래에 생성될 session directory 이름.
    기본값은 session_YYYYMMDD_HHMMSS 형식.
--follower-urdf
    Pinocchio FK와 FT payload gravity 계산에 사용할 follower robot URDF 경로.
--joint-topic, --robot-joint-topic
    Doosan follower arm의 ROS2 JointState topic.
    position은 robot/joint_deg.zarr로 저장되고, FK 입력으로도 사용된다.
--hand-joint-topic
    오른손 hand joint position을 읽을 ROS2 JointState topic. 기본값은 /joint_states.
    custom_robot_utils.py의 JOINT_NAMES_HAND_RIGHT 15개 이름을 같은 순서로 저장한다.
--record-hand
    robot/hand_joint.zarr와 robot/hand_joint_time_stamps.zarr 기록을 켠다.
    기본값은 ON이다.
--no-hand, --disable-hand
    오른손 hand joint 기록과 readiness/health check를 끈다.
--record-ee-pose-fk
    joint FK를 robot/ee_pose_fk_se3.zarr로 기록한다. legacy 기본값은 ON이다.
--no-ee-pose-fk, --disable-ee-pose-fk
    FK pose 기록과 해당 readiness/health check를 끈다. joint와 controller current
    pose 기록은 유지된다.
--joint-torque-topic, --jt-joint-topic
    jt/joint_torque.zarr로 저장할 JointState effort source topic.
    생략하면 --joint-topic과 같은 topic을 사용한다.
--no-jt, --disable-jt
    joint_states.effort 기반 joint torque sensing 기록을 끈다.
    이 옵션을 주면 jt/ directory를 만들지 않고, JT readiness/health check도 제외한다.
--record-jt
    JT 기록을 명시적으로 켠다. 기본값은 ON이다.
--ft-topic, --ft-raw-topic
    End-Effector F/T sensor의 ROS2 WrenchStamped topic.
--ft-base-topic
    compliant controller가 publish하는 base-frame FT WrenchStamped topic.
    기본값은 /aft_sensor2/wrench_base이며 ft/wrench_base.zarr로 저장된다.
--record-ft-base
    ft-base-topic에서 base-frame FT wrench를 기록한다. 기본값은 ON이다.
--no-ft-base, --disable-ft-base
    ft-base-topic을 요구하지 않고 ft/wrench_base.zarr를 저장하지 않는다.
--jt-tared-wrench-topic, --jt-raw-wrench-topic
    compliant controller가 publish하는 JT 기반 tared wrench Float64MultiArray topic.
    기본값은 /right/F_e_raw이다.
--jt-tared-filtered-wrench-topic, --jt-filtered-wrench-topic, --jt-wrench-topic
    compliant controller가 publish하는 JT 기반 tared+filtered wrench Float64MultiArray topic.
    기본값은 /right/F_e이다.
--command-topic, --robot-command-topic
    impedance controller에 들어가는 command pose의 ROS2 topic.
--command-msg-type
    command-topic의 ROS message type. auto, pose_stamped, float64_multi_array 중 하나.
    auto는 */desired_pose이면 Float64MultiArray, 아니면 PoseStamped로 구독한다.
--command-float64-euler-order
    Float64MultiArray command pose [x,y,z,a,b,c]의 Euler axis order.
    angle은 degree로 해석한다.
--record-cmd-pose
    command-topic에서 command pose를 기록한다. 기본값은 ON이다.
--no-cmd-pose, --disable-cmd-pose, --no-command-pose
    command-topic을 요구하지 않는다. robot sampler가 같은 timestamp의 현재 EE pose를
    robot/command_pose_se3.zarr에 대신 저장한다.
--contact-state-topic
    leader teleop contact state의 ROS2 std_msgs/Int32 topic.
    기본값은 /leader_teleop_node/contact_state이다. -1=비접촉, 1=접촉이다.
--record-contact-state
    contact-state-topic에서 받은 state를 robot/contact_state.zarr와
    robot/contact_state_time_stamps.zarr에 저장한다.
--no-contact-state, --disable-contact-state
    contact state 기록과 readiness/health check를 끈다.
--contact-phase-topic
    leader teleop contact phase의 ROS2 std_msgs/Int32 topic.
    기본값은 /leader_teleop_node/contact_phase이다.
    -1=비접촉, 0=pre-contact, 1=접촉이다.
--record-contact-phase
    contact-phase-topic에서 받은 phase를 robot/contact_phase.zarr와
    robot/contact_phase_time_stamps.zarr에 저장한다.
--no-contact-phase, --disable-contact-phase
    contact phase 기록과 readiness/health check를 끈다.
--base-frame
    FK 기준 frame. EE pose는 base-frame_T_ee-frame으로 저장된다.
--ee-frame
    FK에서 사용할 End-Effector frame. 기본값은 right_link_6.
--ft-frame
    FT sensor frame 이름. payload gravity 계산 기준 frame이기도 하다.
    현재는 FT sensor frame orientation이 right_link_6와 같다고 가정한다.
--ft-payload-root
    FT sensor 아래에 달린 물체의 자중 wrench를 URDF inertial로 계산할 때
    사용할 시작 link. 기본값은 right_hand_base_link.
--no-ft-payload-gravity-comp
    URDF 기반 payload gravity subtraction을 끈다.
--ft-payload-gravity-sign
    계산된 payload gravity wrench에 곱할 부호.
    순서는 [Fx,Fy,Fz,Mx,My,Mz], 기본값은 1,1,1,1,1,1.
--camera-fps
    RealSense RGB/depth stream 목표 FPS. 기본값은 60.
--color-width, --color-height
    RealSense RGB stream 해상도. 기본값은 640x480.
--depth-width, --depth-height
    RealSense depth stream 해상도. 기본값은 640x480.
--align-depth-to-color / --no-align-depth-to-color
    RealSense rs.align(rs.stream.color) 적용 여부. 기본값은 align ON.
    align ON이면 camera_*/depth.zarr가 RGB/color pixel grid 기준 depth가 된다.
    align OFF이면 RGB-depth pixel grid 정합이 적용되지 않고 raw depth grid를 저장한다.
    시작 로그의 depth_align_to_color와 metadata의 depth_aligned_to_color로 실제
    적용 여부를 확인할 수 있다.
--robot-sample-hz
    FK, joint, joint torque 기록 timer rate. 기본값은 60 Hz.
--ft-hz
    F/T sensor 저장 target Hz이자 health monitor 기준 Hz. 기본값은 350 Hz.
    ROS topic이 이보다 빠르면 저장/pre-roll에는 --ft-hz에 맞춰 downsample한다.
--ft-base-hz
    /aft_sensor2/wrench_base 저장 target Hz이자 health monitor 기준 Hz.
    기본값은 350 Hz. topic이 이보다 빠르면 저장/pre-roll에는 --ft-base-hz에
    맞춰 downsample한다.
--jt-tared-wrench-hz
    /right/F_e_raw 저장 target Hz이자 health monitor 기준 Hz. 기본값은 350 Hz.
--jt-tared-filtered-wrench-hz
    /right/F_e 저장 target Hz이자 health monitor 기준 Hz. 기본값은 350 Hz.
--command-hz
    readiness check와 health check에서 기대하는 command pose Hz. 기본값은 60 Hz.
--contact-state-hz
    contact state 저장 target Hz이자 health monitor 기준 Hz. 기본값은 60 Hz.
    ROS topic이 이보다 빠르면 저장/pre-roll에는 이 값에 맞춰 downsample한다.
--hz-min-ratio
    modality별 목표 Hz 대비 최소 허용 비율. 기본값은 0.98.
    60 Hz 목표라면 58.8 Hz 이상이어야 READY/health check를 통과한다.
--pre-roll-sec
    episode 시작 시 함께 저장할 시작 직전 ring-buffer data 길이. 기본값은 1.0초.
--writer-queue-size
    recording 중 sensor callback과 disk writer를 분리하기 위한 queue 최대 item 수.
    queue가 가득 차면 health monitor가 episode를 중단하고 문제를 출력한다.
    기본값은 4096.
--writer-batch-size
    background writer가 한 번에 모아서 append할 최대 item 수.
    zarr를 sample마다 append하지 않고 batch append해서 저장 부하를 줄인다.
    기본값은 128.
--ready-window-sec
    episode 시작 가능 여부를 판단할 때 Hz를 계산하는 시간 window.
--ready-max-age-sec
    episode 시작 전 latest sample이 이 시간보다 오래됐으면 unhealthy로 본다.
--source-stale-sec
    robot sampling timer가 사용할 수 있는 latest ROS joint/JT data의 최대 age.
--health-window-sec
    episode 기록 중 health monitor가 Hz를 계산하는 시간 window.
--health-grace-sec
    episode 시작 직후 health auto-stop을 적용하지 않는 grace period.
--health-max-stale-sec
    episode 기록 중 latest sample이 이 시간보다 오래됐으면 자동 중단한다.
--preview
    OpenCV preview window를 띄운다.
    화면 구성은 D405 RGB/depth, D435 RGB/depth의 2x2 view.
--depth-preview-max-mm
    depth preview colormap scaling에 사용할 최대 raw depth 값. 기본값은 1000.
--command-position-unit {mm,m}
    PoseStamped command position의 단위.
    기본값은 mm이며, meter로 변환해 command_pose_se3.zarr에 저장한다.
--startup-check-count
    프로그램 시작 후 모든 modality가 정상인지 연속 확인할 횟수.
    기본값은 10.
--recovery-check-count
    시작 확인 중 문제가 한 번이라도 발생한 뒤, 복구 이후 다시 연속 정상 확인할
    횟수. 기본값은 5.
--startup-status-period-sec
    시작 readiness status를 출력하는 주기. 기본값은 1.0초.
--camera-reconnect-period-sec
    시작 readiness 중 camera stream 문제가 지속될 때 RealSense pipeline restart를
    시도하는 최소 간격. 기본값은 5.0초.
--camera-hardware-reset-after-restarts
    시작 readiness 중 같은 camera의 pipeline restart가 연속 실패하면 해당 횟수마다
    RealSense hardware reset으로 escalation한다. 기본값은 3회.
--camera-hardware-reset-settle-sec
    hardware reset 후 USB 재인식을 기다리는 시간. 기본값은 6.0초.
--skip-startup-check
    시작 직후 readiness gate를 건너뛴다. 디버깅용 옵션이다.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import re
import shutil
import signal
import sys
import threading
import time
import traceback
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path
from typing import Any, Callable, Deque, Dict, Iterable, List, Optional, Tuple

import numpy as np

from .il_hand_contract import (
    RIGHT_HAND_JOINT_NAMES,
    validate_right_hand_joint_measurements,
)

cv2 = None
pin = None
rs = None
zarr = None

JOINT_NAMES_HAND_RIGHT = RIGHT_HAND_JOINT_NAMES


def load_cv2():
    global cv2
    if cv2 is None:
        try:
            import cv2 as _cv2
        except ImportError as exc:
            raise RuntimeError(
                "opencv-python is required for preview display."
            ) from exc
        cv2 = _cv2
    return cv2


def load_zarr():
    global zarr
    if zarr is None:
        try:
            import zarr as _zarr
        except ImportError as exc:
            raise RuntimeError("zarr is required for raw array storage.") from exc
        zarr = _zarr
    return zarr


def load_realsense():
    global rs
    if rs is None:
        try:
            import pyrealsense2 as _rs
        except ImportError as exc:
            raise RuntimeError(
                "pyrealsense2 is required. Install librealsense Python bindings on "
                "the data collection PC."
            ) from exc
        rs = _rs
    return rs


def load_pinocchio():
    global pin
    if pin is None:
        try:
            import pinocchio as _pin
        except ImportError:
            try:
                import pin as _pin
            except ImportError as exc:
                raise RuntimeError(
                    "Pinocchio is required for FK. Install the pinocchio Python "
                    "package on the data collection PC."
                ) from exc
        pin = _pin
    return pin


def load_ros2():
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped, WrenchStamped
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray, Int32, String
        from std_srvs.srv import Trigger
    except ImportError as exc:
        raise RuntimeError(
            "ROS2 Python packages are required. Source the Doosan/ROS2 workspace "
            "before running this recorder."
        ) from exc
    try:
        from contact_observer_msgs.msg import ContactObservation, ObserverInput
    except ImportError:
        # Legacy recorder modes remain usable without the V2 interface package.
        ContactObservation = None
        ObserverInput = None
    return {
        "rclpy": rclpy,
        "Node": Node,
        "ContactObservation": ContactObservation,
        "ObserverInput": ObserverInput,
        "JointState": JointState,
        "PoseStamped": PoseStamped,
        "WrenchStamped": WrenchStamped,
        "Float64MultiArray": Float64MultiArray,
        "Int32": Int32,
        "String": String,
        "Trigger": Trigger,
        "QoSProfile": QoSProfile,
        "ReliabilityPolicy": ReliabilityPolicy,
        "MultiThreadedExecutor": MultiThreadedExecutor,
    }


def now_s() -> float:
    return time.time()


def current_process_rss_bytes() -> int:
    """Current Linux RSS without introducing a psutil dependency."""
    try:
        with open("/proc/self/statm", "r", encoding="ascii") as stream:
            resident_pages = int(stream.read().split()[1])
        return resident_pages * int(os.sysconf("SC_PAGE_SIZE"))
    except (OSError, ValueError, IndexError):
        return 0


def linux_memory_bytes() -> Tuple[int, int]:
    """Return Linux (MemTotal, MemAvailable) bytes without psutil."""
    values: Dict[str, int] = {}
    try:
        with open("/proc/meminfo", "r", encoding="ascii") as stream:
            for line in stream:
                key, _, raw = line.partition(":")
                if key not in ("MemTotal", "MemAvailable"):
                    continue
                fields = raw.strip().split()
                if fields:
                    values[key] = int(fields[0]) * 1024
    except (OSError, ValueError, IndexError):
        return 0, 0
    return values.get("MemTotal", 0), values.get("MemAvailable", 0)


def payload_nbytes(value: Any) -> int:
    """Conservative queued-payload size used by the writer byte budget."""
    if isinstance(value, np.ndarray):
        return int(value.nbytes)
    if isinstance(value, (tuple, list)):
        return 64 + sum(payload_nbytes(item) for item in value)
    if isinstance(value, (bytes, bytearray, memoryview)):
        return len(value)
    if isinstance(value, str):
        return len(value.encode("utf-8"))
    return 32


def ros_stamp_to_seconds(stamp: Any) -> float:
    """Convert builtin_interfaces/Time to the shared Unix/ROS clock in seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def normalize_contact_prediction_age_ms(
    value: Any,
    *,
    valid: bool,
    model_ready: bool,
) -> float:
    """Keep stored contact diagnostics finite without weakening safety masks.

    The contact observer publishes a negative sentinel when no prediction is
    available; older versions used +inf.  Either value is only valid on a
    fail-closed observation.  Raw and converted datasets require finite numeric
    arrays, so store 0.0 while the valid/model_ready masks retain the
    authoritative not-ready state.
    """
    age_ms = float(value)
    if math.isfinite(age_ms) and age_ms >= 0.0:
        return age_ms
    policy_ready = bool(valid) and bool(model_ready)
    if not policy_ready and (
        (math.isfinite(age_ms) and age_ms < 0.0)
        or (math.isinf(age_ms) and age_ms > 0.0)
    ):
        return 0.0
    raise ValueError(
        "prediction_age_ms must be finite and non-negative, or a negative/+inf "
        "sentinel only when ContactObservation is invalid/not-ready"
    )


def spin_ros_executor(executor: Any, ros_node: Any, stop_event: threading.Event) -> None:
    """Fail visibly instead of leaving a live recorder with a dead ROS executor."""
    try:
        executor.spin()
    except BaseException as exc:
        if stop_event.is_set():
            return
        ros_node.executor_error = f"{type(exc).__name__}: {exc}"
        traceback.print_exc()
        print(
            f"ROS executor stopped unexpectedly: {ros_node.executor_error}",
            file=sys.stderr,
        )
        stop_event.set()


def validate_and_record_message_frame(
    args: argparse.Namespace,
    msg: Any,
    *,
    stream_name: str,
    expected_frame: str,
) -> str:
    """Fail closed on a missing/changing ROS frame and retain observed provenance."""
    header = getattr(msg, "header", None)
    actual_frame = str(getattr(header, "frame_id", "")).strip()
    expected_frame = str(expected_frame).strip()
    if not actual_frame:
        raise ValueError(f"{stream_name} message has an empty header.frame_id")
    if not expected_frame:
        raise ValueError(f"{stream_name} has no configured expected frame")
    if actual_frame != expected_frame:
        raise ValueError(
            f"{stream_name} frame is {actual_frame!r}, expected {expected_frame!r}"
        )
    observed = getattr(args, "observed_source_frames", None)
    if observed is None:
        observed = {}
        setattr(args, "observed_source_frames", observed)
    previous = observed.get(stream_name)
    if previous is not None and previous != actual_frame:
        raise ValueError(
            f"{stream_name} frame changed from {previous!r} to {actual_frame!r}"
        )
    observed[stream_name] = actual_frame
    return actual_frame


def deadline_crossing_select(
    sample_t: float,
    target_hz: float,
    next_deadline: Optional[float],
) -> Tuple[bool, Optional[float]]:
    """Select the first source sample crossing each periodic deadline."""
    if target_hz <= 0.0:
        return True, next_deadline
    period = 1.0 / float(target_hz)
    if next_deadline is None:
        return True, float(sample_t) + period
    # A ROS/source clock can jump back (sim reset, bag replay, controller
    # restart).  A monotonic pre-deadline sample is never more than one target
    # period behind the next deadline, even when earlier samples skipped
    # periods.  Re-anchor only beyond that bound so ordinary early samples are
    # still suppressed while a clock reset cannot suppress the stream forever.
    if float(sample_t) + period + 1e-9 < next_deadline:
        return True, float(sample_t) + period
    if float(sample_t) + 1e-9 < next_deadline:
        return False, next_deadline
    while next_deadline <= float(sample_t) + 1e-9:
        next_deadline += period
    return True, next_deadline


def json_default(obj: Any):
    if isinstance(obj, Path):
        return str(obj)
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.floating, np.integer)):
        return obj.item()
    raise TypeError(f"Object of type {type(obj).__name__} is not JSON serializable")


def recorder_config_snapshot(args: argparse.Namespace) -> Dict[str, Any]:
    """Return the effective, parsed recorder settings stored with every episode.

    Keeping this snapshot next to the arrays lets the checker distinguish the
    settings that actually produced an episode from a later YAML file supplied
    as a validation reference. Runtime-populated calibration/FT dictionaries
    are recorded separately and intentionally excluded here.
    """
    excluded = {
        "camera_calibration",
        "ft_processing_metadata",
        "observed_source_frames",
    }
    return {
        key: value
        for key, value in vars(args).items()
        if key not in excluded
    }


def configured_camera_specs(
    args: argparse.Namespace,
) -> List[Tuple[int, str, Optional[str]]]:
    specs = [(0, "D405", getattr(args, "d405_serial", None))]
    if bool(getattr(args, "enable_d435", True)):
        specs.append((1, "D435", getattr(args, "d435_serial", None)))
    return specs


def configured_camera_roles(args: argparse.Namespace) -> Dict[str, Dict[str, Any]]:
    return {
        f"camera_{camera_id}": {"model": model, "serial": serial}
        for camera_id, model, serial in configured_camera_specs(args)
    }


_SESSION_NON_SEMANTIC_CONFIG_KEYS = {
    "config_yaml",
    "list_realsense",
    "validate_config_only",
    "output_dir",
    "session_name",
    "writer_queue_size",
    "writer_queue_max_bytes",
    "writer_batch_size",
    "writer_queue_warn_ratio",
    "control_mode",
    "teleop_status_topic",
    "recorder_diagnostics_topic",
    "recorder_ft_selected_topic",
    "diagnostics_period_sec",
    "require_teleop_fast",
    "recorder_rss_hard_bytes",
    "system_memory_warn_bytes",
    "system_memory_stop_bytes",
    "estimated_pre_roll_items",
    "estimated_pre_roll_payload_bytes",
    "estimated_payload_peak_bytes",
    "recorder_rss_hard_bytes_resolved",
    "recorder_rss_warn_bytes_resolved",
    "system_memory_warn_bytes_resolved",
    "system_memory_stop_bytes_resolved",
    "hz_min_ratio",
    "ready_window_sec",
    "ready_max_age_sec",
    "source_stale_sec",
    "health_window_sec",
    "health_grace_sec",
    "health_max_stale_sec",
    "health_failure_check_count",
    "startup_check_count",
    "recovery_check_count",
    "startup_status_period_sec",
    "camera_reconnect_period_sec",
    "camera_hardware_reset_after_restarts",
    "camera_hardware_reset_settle_sec",
    "skip_startup_check",
    "preview",
    "depth_preview_max_mm",
}


def _session_semantic_config(config: Dict[str, Any]) -> Dict[str, Any]:
    semantic = {
        key: config[key]
        for key in sorted(config)
        if key not in _SESSION_NON_SEMANTIC_CONFIG_KEYS
    }
    # Legacy recorder snapshots always used both cameras and predate this flag.
    semantic.setdefault("enable_d435", True)
    return semantic


def validate_session_compatibility(
    session_dir: Path,
    args: argparse.Namespace,
) -> None:
    """Refuse to append episodes produced by a different acquisition contract."""
    if not session_dir.exists():
        return
    malformed = sorted(
        path.name
        for path in session_dir.iterdir()
        if path.is_dir()
        and path.name.startswith("episode_")
        and re.fullmatch(r"episode_\d+", path.name) is None
    )
    if malformed:
        raise RuntimeError(
            "Session contains malformed episode directory names; expected "
            f"episode_<digits>: {malformed}"
        )
    unfinished = sorted(
        path.name
        for path in session_dir.iterdir()
        if path.is_dir()
        and re.fullmatch(r"\.episode_\d+_recording", path.name)
    )
    if unfinished:
        raise RuntimeError(
            "Session contains unfinished crash-recovery episode directories: "
            f"{unfinished}. Inspect/recover them before recording more data."
        )
    episodes = sorted(
        (
            path
            for path in session_dir.iterdir()
            if path.is_dir()
            and re.fullmatch(r"episode_\d+", path.name)
        ),
        key=lambda path: int(path.name.removeprefix("episode_")),
    )
    seen_ids: Dict[int, str] = {}
    for episode in episodes:
        numeric_id = int(episode.name.removeprefix("episode_"))
        if numeric_id in seen_ids:
            raise RuntimeError(
                "Session contains duplicate numeric episode aliases "
                f"{seen_ids[numeric_id]!r} and {episode.name!r}"
            )
        seen_ids[numeric_id] = episode.name
    if not episodes:
        return
    current = _session_semantic_config(recorder_config_snapshot(args))
    for episode in episodes:
        meta_path = episode / "meta.json"
        if not meta_path.is_file():
            raise RuntimeError(
                f"Existing {episode.name} has no meta.json; refusing to append "
                "to an unverifiable session."
            )
        try:
            with meta_path.open("r", encoding="utf-8") as stream:
                meta = json.load(stream)
        except Exception as exc:
            raise RuntimeError(
                f"Cannot read existing episode metadata {meta_path}"
            ) from exc
        stored_snapshot = meta.get("recorder_config")
        if not isinstance(stored_snapshot, dict):
            raise RuntimeError(
                f"Existing {episode.name} has no recorder_config snapshot; "
                "start a new session_name instead of mixing contracts."
            )
        stored = _session_semantic_config(stored_snapshot)
        differing = sorted(
            key
            for key in set(current) | set(stored)
            if current.get(key) != stored.get(key)
        )
        if differing:
            detail = ", ".join(
                f"{key}: existing={stored.get(key)!r}, "
                f"current={current.get(key)!r}"
                for key in differing[:8]
            )
            if len(differing) > 8:
                detail += f", ... ({len(differing) - 8} more)"
            raise RuntimeError(
                f"Existing {episode.name} recorder configuration is "
                f"incompatible with this run ({detail}). Use a new "
                "session_name."
            )


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        json.dump(payload, f, indent=2, default=json_default)


def parse_xyz_attr(text: Optional[str]) -> np.ndarray:
    if not text:
        return np.zeros(3, dtype=np.float64)
    values = [float(v) for v in str(text).replace(",", " ").split()]
    if len(values) != 3:
        raise ValueError(f"Expected xyz with 3 values, got: {text!r}")
    return np.asarray(values, dtype=np.float64)


def parse_six_floats(text: str, label: str) -> np.ndarray:
    try:
        values = [
            float(x.strip())
            for x in str(text).replace(";", ",").split(",")
            if x.strip()
        ]
    except Exception as exc:
        raise ValueError(f"Invalid {label} value: {text!r}") from exc
    if len(values) != 6:
        raise ValueError(
            f"{label} must have 6 comma-separated values, got {len(values)}: {text!r}"
        )
    return np.asarray(values, dtype=np.float64)


def realsense_intrinsics_to_dict(intrinsics: Any) -> Dict[str, Any]:
    return {
        "width": int(intrinsics.width),
        "height": int(intrinsics.height),
        "fx": float(intrinsics.fx),
        "fy": float(intrinsics.fy),
        "ppx": float(intrinsics.ppx),
        "ppy": float(intrinsics.ppy),
        "model": str(intrinsics.model),
        "coeffs": [float(x) for x in intrinsics.coeffs],
    }


def realsense_extrinsics_to_dict(extrinsics: Any) -> Dict[str, Any]:
    return {
        "rotation": [float(x) for x in extrinsics.rotation],
        "translation": [float(x) for x in extrinsics.translation],
    }


def quat_xyzw_to_rot(q: Iterable[float]) -> np.ndarray:
    x, y, z, w = [float(v) for v in q]
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        raise ValueError("Quaternion norm is zero.")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def euler_deg_to_rot(angles_deg: Iterable[float], order: str) -> np.ndarray:
    angles = [math.radians(float(v)) for v in angles_deg]
    order = str(order).lower()
    if len(order) != 3 or any(axis not in "xyz" for axis in order):
        raise ValueError(f"Unsupported Euler order '{order}'.")

    def rot_axis(axis: str, angle: float) -> np.ndarray:
        c = math.cos(angle)
        s = math.sin(angle)
        if axis == "x":
            return np.array(
                [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64
            )
        if axis == "y":
            return np.array(
                [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64
            )
        return np.array(
            [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64
        )

    rotation = np.eye(3, dtype=np.float64)
    for axis, angle in zip(order, angles):
        rotation = rotation @ rot_axis(axis, angle)
    return rotation


def make_se3(translation_m: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    mat[:3, 3] = np.asarray(translation_m, dtype=np.float64).reshape(3)
    return mat


def estimate_hz(timestamps: Iterable[float], window_sec: float, t_now: float) -> float:
    values = [float(t) for t in timestamps if float(t) >= t_now - window_sec]
    if len(values) < 2:
        return 0.0
    dt = values[-1] - values[0]
    if dt <= 1e-9:
        return 0.0
    return (len(values) - 1) / dt


def append_zarr_row(arr, value: np.ndarray) -> None:
    value = np.asarray(value)
    arr.append(value[np.newaxis, ...], axis=0)


def append_zarr_rows(arr, values: np.ndarray) -> None:
    values = np.asarray(values)
    if values.shape[0] == 0:
        return
    arr.append(values, axis=0)


def append_zarr_scalar(arr, value: float) -> None:
    arr.append(np.asarray([value], dtype=arr.dtype), axis=0)


def append_zarr_scalars(arr, values: Iterable[float]) -> None:
    values_arr = np.asarray(list(values), dtype=arr.dtype)
    if values_arr.shape[0] == 0:
        return
    arr.append(values_arr, axis=0)


def open_zarr_array(path: Path, shape_tail: Tuple[int, ...], dtype: str, chunk0: int):
    load_zarr()
    chunks = (max(1, int(chunk0)),) + tuple(shape_tail)
    return zarr.open_array(
        str(path),
        mode="w",
        shape=(0,) + tuple(shape_tail),
        chunks=chunks,
        dtype=dtype,
    )


def record_ft_wrench_enabled(args: argparse.Namespace) -> bool:
    return any(
        bool(getattr(args, name, False))
        for name in (
            "record_ft_wrench_raw",
            "record_ft_wrench_payload_gravity",
            "record_ft_wrench_comp_payload",
        )
    )


def recorded_ft_stream_names(args: argparse.Namespace) -> List[str]:
    streams = []
    for attr, name in (
        ("record_ft_wrench_raw", "wrench_raw"),
        ("record_ft_wrench_payload_gravity", "wrench_payload_gravity"),
        ("record_ft_wrench_comp_payload", "wrench_comp_payload"),
        ("record_ft_base", "wrench_base"),
        ("record_jt_tared_wrench", "jt_tared_wrench"),
        ("record_jt_tared_filtered_wrench", "jt_tared_filtered_wrench"),
    ):
        if bool(getattr(args, attr, False)):
            streams.append(name)
    return streams


class KeyReader:
    def __init__(self):
        self._use_raw = False
        self._old_attrs = None

    def __enter__(self):
        if sys.stdin.isatty():
            import termios
            import tty

            self._old_attrs = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            self._use_raw = True
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._use_raw and self._old_attrs is not None:
            import termios

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_attrs)

    def read(self, timeout: Optional[float] = None) -> Optional[str]:
        import select

        readable, _, _ = select.select([sys.stdin], [], [], timeout)
        if not readable:
            return None
        ch = sys.stdin.read(1)
        return ch if ch else None


def prompt_yes_no(prompt: str, key_reader: KeyReader) -> bool:
    print(prompt, end="", flush=True)
    while True:
        ch = key_reader.read(None)
        if ch is None:
            continue
        ch = ch.lower()
        if ch in ("y", "n"):
            print(ch)
            return ch == "y"


class CameraSink:
    def __init__(self, root: Path, model: str, fps: int):
        self.root = root
        self.model = model
        self.fps = fps
        self.root.mkdir(parents=True, exist_ok=True)

        self.rgb_ts = open_zarr_array(root / "rgb_time_stamps.zarr", tuple(), "f8", 2048)
        self.rgb_hw_ts = open_zarr_array(
            root / "rgb_hardware_time_stamps_ms.zarr", tuple(), "f8", 2048
        )
        self.rgb_frame_numbers = open_zarr_array(
            root / "rgb_frame_numbers.zarr", tuple(), "i8", 2048
        )
        self.depth_ts = open_zarr_array(
            root / "depth_time_stamps.zarr", tuple(), "f8", 2048
        )
        self.depth_hw_ts = open_zarr_array(
            root / "depth_hardware_time_stamps_ms.zarr", tuple(), "f8", 2048
        )
        self.depth_frame_numbers = open_zarr_array(
            root / "depth_frame_numbers.zarr", tuple(), "i8", 2048
        )
        self.rgb_arr = None
        self.depth_arr = None
        self.rgb_shape = None

    def write_rgb(
        self,
        timestamp: float,
        rgb: np.ndarray,
        hardware_timestamp_ms: float = math.nan,
        frame_number: int = -1,
    ) -> None:
        rgb = np.asarray(rgb)
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError(f"RGB frame for {self.model} must be HxWx3.")
        h, w = rgb.shape[:2]
        if self.rgb_arr is None:
            self.rgb_shape = (h, w, 3)
            self.rgb_arr = open_zarr_array(
                self.root / "rgb.zarr", self.rgb_shape, "u1", 32
            )
        elif tuple(rgb.shape) != tuple(self.rgb_shape):
            raise ValueError(
                f"RGB frame shape for {self.model} changed from "
                f"{self.rgb_shape} to {tuple(rgb.shape)}."
            )
        append_zarr_row(self.rgb_arr, rgb.astype(np.uint8, copy=False))
        append_zarr_scalar(self.rgb_ts, timestamp)
        append_zarr_scalar(self.rgb_hw_ts, hardware_timestamp_ms)
        append_zarr_scalar(self.rgb_frame_numbers, int(frame_number))

    def write_rgb_batch(
        self,
        samples: List[Tuple[float, np.ndarray, float, int]],
    ) -> None:
        if not samples:
            return
        first_rgb = np.asarray(samples[0][1])
        if first_rgb.ndim != 3 or first_rgb.shape[2] != 3:
            raise ValueError(f"RGB frame for {self.model} must be HxWx3.")
        if self.rgb_arr is None:
            self.rgb_shape = tuple(first_rgb.shape)
            self.rgb_arr = open_zarr_array(
                self.root / "rgb.zarr", self.rgb_shape, "u1", 32
            )
        timestamps = []
        hardware_timestamps = []
        frame_numbers = []
        rgb_frames = []

        def flush_rgb_frames() -> None:
            if not rgb_frames:
                return
            append_zarr_rows(self.rgb_arr, np.stack(rgb_frames, axis=0))
            rgb_frames.clear()

        for timestamp, rgb, hardware_timestamp_ms, frame_number in samples:
            rgb = np.asarray(rgb)
            if rgb.ndim != 3 or rgb.shape[2] != 3:
                raise ValueError(f"RGB frame for {self.model} must be HxWx3.")
            if tuple(rgb.shape) != tuple(self.rgb_shape):
                raise ValueError(
                    f"RGB frame shape for {self.model} changed from "
                    f"{self.rgb_shape} to {tuple(rgb.shape)}."
                )
            rgb_frames.append(rgb.astype(np.uint8, copy=False))
            if len(rgb_frames) >= 32:
                flush_rgb_frames()
            timestamps.append(timestamp)
            hardware_timestamps.append(hardware_timestamp_ms)
            frame_numbers.append(int(frame_number))
        flush_rgb_frames()
        append_zarr_scalars(self.rgb_ts, timestamps)
        append_zarr_scalars(self.rgb_hw_ts, hardware_timestamps)
        append_zarr_scalars(self.rgb_frame_numbers, frame_numbers)

    def write_depth(
        self,
        timestamp: float,
        depth: np.ndarray,
        hardware_timestamp_ms: float = math.nan,
        frame_number: int = -1,
    ) -> None:
        depth = np.asarray(depth)
        if depth.ndim != 2:
            raise ValueError(f"Depth frame for {self.model} must be HxW.")
        if self.depth_arr is None:
            h, w = depth.shape
            self.depth_arr = open_zarr_array(
                self.root / "depth.zarr", (h, w), "u2", 32
            )
        append_zarr_row(self.depth_arr, depth.astype(np.uint16, copy=False))
        append_zarr_scalar(self.depth_ts, timestamp)
        append_zarr_scalar(self.depth_hw_ts, hardware_timestamp_ms)
        append_zarr_scalar(self.depth_frame_numbers, int(frame_number))

    def write_depth_batch(
        self,
        samples: List[Tuple[float, np.ndarray, float, int]],
    ) -> None:
        if not samples:
            return
        first_depth = np.asarray(samples[0][1])
        if first_depth.ndim != 2:
            raise ValueError(f"Depth frame for {self.model} must be HxW.")
        if self.depth_arr is None:
            h, w = first_depth.shape
            self.depth_arr = open_zarr_array(
                self.root / "depth.zarr", (h, w), "u2", 32
            )
        timestamps = []
        hardware_timestamps = []
        frame_numbers = []
        depth_frames = []
        for timestamp, depth, hardware_timestamp_ms, frame_number in samples:
            depth = np.asarray(depth)
            if depth.ndim != 2:
                raise ValueError(f"Depth frame for {self.model} must be HxW.")
            depth_frames.append(depth.astype(np.uint16, copy=False))
            timestamps.append(timestamp)
            hardware_timestamps.append(hardware_timestamp_ms)
            frame_numbers.append(int(frame_number))
        append_zarr_rows(self.depth_arr, np.stack(depth_frames, axis=0))
        append_zarr_scalars(self.depth_ts, timestamps)
        append_zarr_scalars(self.depth_hw_ts, hardware_timestamps)
        append_zarr_scalars(self.depth_frame_numbers, frame_numbers)

    def close(self) -> None:
        self.rgb_arr = None


class EpisodeWriter:
    def __init__(self, temp_dir: Path, final_dir: Path, args: argparse.Namespace):
        self.temp_dir = temp_dir
        self.final_dir = final_dir
        self.args = args
        self.lock = threading.RLock()
        self.start_wall_time = now_s()
        self.stop_wall_time = None

        self.temp_dir.mkdir(parents=True, exist_ok=False)
        self.camera_sinks = {
            camera_id: CameraSink(
                self.temp_dir / f"camera_{camera_id}_{model}",
                model,
                args.camera_fps,
            )
            for camera_id, model, _serial in configured_camera_specs(args)
        }

        robot_dir = self.temp_dir / "robot"
        ft_dir = self.temp_dir / "ft"
        robot_dir.mkdir(parents=True, exist_ok=True)
        ft_dir.mkdir(parents=True, exist_ok=True)
        self.record_contact_observation = bool(
            getattr(args, "record_contact_observation", False)
        )
        contact_dir = self.temp_dir / "contact"
        if self.record_contact_observation:
            contact_dir.mkdir(parents=True, exist_ok=True)
        self.record_jt = bool(getattr(args, "record_jt", True))
        jt_dir = self.temp_dir / "jt" if self.record_jt else None
        if jt_dir is not None:
            jt_dir.mkdir(parents=True, exist_ok=True)

        self.joint_deg = open_zarr_array(robot_dir / "joint_deg.zarr", (6,), "f8", 2048)
        self.joint_ts = open_zarr_array(
            robot_dir / "joint_time_stamps.zarr", tuple(), "f8", 2048
        )
        self.record_hand = bool(getattr(args, "record_hand", True))
        self.hand_joint = None
        self.hand_joint_ts = None
        if self.record_hand:
            self.hand_joint = open_zarr_array(
                robot_dir / "hand_joint.zarr",
                (len(JOINT_NAMES_HAND_RIGHT),),
                "f8",
                2048,
            )
            self.hand_joint_ts = open_zarr_array(
                robot_dir / "hand_joint_time_stamps.zarr", tuple(), "f8", 2048
            )
        self.record_ee_pose_fk = bool(
            getattr(args, "record_ee_pose_fk", True)
        )
        self.ee_pose = None
        self.ee_pose_ts = None
        if self.record_ee_pose_fk:
            self.ee_pose = open_zarr_array(
                robot_dir / "ee_pose_fk_se3.zarr", (4, 4), "f8", 1024
            )
            self.ee_pose_ts = open_zarr_array(
                robot_dir / "ee_pose_fk_time_stamps.zarr", tuple(), "f8", 2048
            )
        self.command_pose = open_zarr_array(
            robot_dir / "command_pose_se3.zarr", (4, 4), "f8", 1024
        )
        self.command_pose_ts = open_zarr_array(
            robot_dir / "command_time_stamps.zarr", tuple(), "f8", 2048
        )
        self.record_current_pose = bool(getattr(args, "record_current_pose", False))
        self.current_pose = None
        self.current_pose_ts = None
        if self.record_current_pose:
            self.current_pose = open_zarr_array(
                robot_dir / "controller_current_pose_se3.zarr", (4, 4), "f8", 1024
            )
            self.current_pose_ts = open_zarr_array(
                robot_dir / "controller_current_pose_time_stamps.zarr",
                tuple(),
                "f8",
                2048,
            )
        self.record_cmd_quat_pose = bool(
            getattr(args, "record_cmd_quat_pose", False)
        )
        self.command_quat_pose = None
        self.command_quat_pose_ts = None
        if self.record_cmd_quat_pose:
            self.command_quat_pose = open_zarr_array(
                robot_dir / "command_quat_pose_se3.zarr", (4, 4), "f8", 1024
            )
            self.command_quat_pose_ts = open_zarr_array(
                robot_dir / "command_quat_time_stamps.zarr", tuple(), "f8", 2048
            )
        self.record_contact_state = bool(getattr(args, "record_contact_state", False))
        self.contact_state = None
        self.contact_state_ts = None
        if self.record_contact_state:
            self.contact_state = open_zarr_array(
                robot_dir / "contact_state.zarr", tuple(), "i4", 2048
            )
            self.contact_state_ts = open_zarr_array(
                robot_dir / "contact_state_time_stamps.zarr", tuple(), "f8", 2048
            )
        self.record_contact_phase = bool(getattr(args, "record_contact_phase", False))
        self.contact_phase = None
        self.contact_phase_ts = None
        if self.record_contact_phase:
            self.contact_phase = open_zarr_array(
                robot_dir / "contact_phase.zarr", tuple(), "i4", 2048
            )
            self.contact_phase_ts = open_zarr_array(
                robot_dir / "contact_phase_time_stamps.zarr", tuple(), "f8", 2048
            )
        self.contact_wrench = None
        self.free_space_wrench_prediction = None
        self.canonical_contact_state = None
        self.contact_valid = None
        self.contact_model_ready = None
        self.contact_source_ts = None
        self.contact_receive_ts = None
        self.contact_source_sequences = None
        self.contact_prediction_age_ms = None
        if self.record_contact_observation:
            # All fields are appended from one queue item so row i is one atomic
            # ContactObservation sample across every array.
            self.contact_wrench = open_zarr_array(
                contact_dir / "contact_wrench.zarr", (6,), "f8", 4096
            )
            self.free_space_wrench_prediction = open_zarr_array(
                contact_dir / "free_space_wrench_prediction.zarr",
                (6,),
                "f8",
                4096,
            )
            self.canonical_contact_state = open_zarr_array(
                contact_dir / "contact_state.zarr", tuple(), "u1", 4096
            )
            self.contact_valid = open_zarr_array(
                contact_dir / "contact_valid.zarr", tuple(), "u1", 4096
            )
            self.contact_model_ready = open_zarr_array(
                contact_dir / "contact_model_ready.zarr", tuple(), "u1", 4096
            )
            self.contact_source_ts = open_zarr_array(
                contact_dir / "source_time_stamps.zarr", tuple(), "f8", 4096
            )
            self.contact_receive_ts = open_zarr_array(
                contact_dir / "receive_time_stamps.zarr", tuple(), "f8", 4096
            )
            self.contact_source_sequences = open_zarr_array(
                contact_dir / "source_sequences.zarr", tuple(), "u8", 4096
            )
            self.contact_prediction_age_ms = open_zarr_array(
                contact_dir / "prediction_age_ms.zarr", tuple(), "f8", 4096
            )
        self.record_ft_wrench_raw = bool(getattr(args, "record_ft_wrench_raw", True))
        self.record_ft_wrench_payload_gravity = bool(
            getattr(args, "record_ft_wrench_payload_gravity", False)
        )
        self.record_ft_wrench_comp_payload = bool(
            getattr(args, "record_ft_wrench_comp_payload", False)
        )
        self.record_ft_wrench = record_ft_wrench_enabled(args)
        self.wrench_raw = None
        self.wrench_payload_gravity = None
        self.wrench_comp_payload = None
        self.wrench_ts = None
        if self.record_ft_wrench_raw:
            self.wrench_raw = open_zarr_array(
                ft_dir / "wrench_raw.zarr", (6,), "f8", 4096
            )
        if self.record_ft_wrench_payload_gravity:
            self.wrench_payload_gravity = open_zarr_array(
                ft_dir / "wrench_payload_gravity.zarr", (6,), "f8", 4096
            )
        if self.record_ft_wrench_comp_payload:
            self.wrench_comp_payload = open_zarr_array(
                ft_dir / "wrench_comp_payload.zarr", (6,), "f8", 4096
            )
        if self.record_ft_wrench:
            self.wrench_ts = open_zarr_array(
                ft_dir / "wrench_time_stamps.zarr", tuple(), "f8", 4096
            )
        self.record_ft_base = bool(getattr(args, "record_ft_base", False))
        self.wrench_base = None
        self.wrench_base_ts = None
        if self.record_ft_base:
            self.wrench_base = open_zarr_array(
                ft_dir / "wrench_base.zarr", (6,), "f8", 4096
            )
            self.wrench_base_ts = open_zarr_array(
                ft_dir / "wrench_base_time_stamps.zarr", tuple(), "f8", 4096
            )
        self.record_jt_tared_wrench = bool(
            getattr(args, "record_jt_tared_wrench", False)
        )
        self.record_jt_tared_filtered_wrench = bool(
            getattr(args, "record_jt_tared_filtered_wrench", True)
        )
        self.jt_tared_wrench = None
        self.jt_tared_wrench_ts = None
        if self.record_jt_tared_wrench:
            self.jt_tared_wrench = open_zarr_array(
                ft_dir / "jt_tared_wrench.zarr", (6,), "f8", 4096
            )
            self.jt_tared_wrench_ts = open_zarr_array(
                ft_dir / "jt_tared_wrench_time_stamps.zarr", tuple(), "f8", 4096
            )
        self.jt_tared_filtered_wrench = None
        self.jt_tared_filtered_wrench_ts = None
        if self.record_jt_tared_filtered_wrench:
            self.jt_tared_filtered_wrench = open_zarr_array(
                ft_dir / "jt_tared_filtered_wrench.zarr", (6,), "f8", 4096
            )
            self.jt_tared_filtered_wrench_ts = open_zarr_array(
                ft_dir / "jt_tared_filtered_wrench_time_stamps.zarr",
                tuple(),
                "f8",
                4096,
            )
        self.joint_torque = None
        self.joint_torque_ts = None
        if self.record_jt and jt_dir is not None:
            self.joint_torque = open_zarr_array(
                jt_dir / "joint_torque.zarr", (6,), "f8", 2048
            )
            self.joint_torque_ts = open_zarr_array(
                jt_dir / "joint_torque_time_stamps.zarr", tuple(), "f8", 2048
            )

        self.recent: Dict[str, Deque[float]] = {}
        self.counts: Dict[str, int] = {}
        modality_names = [
            name
            for camera_id in self.camera_sinks
            for name in (f"camera_{camera_id}_rgb", f"camera_{camera_id}_depth")
        ]
        modality_names.extend([
            "robot_joint",
            "robot_hand_joint",
            "robot_command_pose",
        ])
        if self.record_ee_pose_fk:
            modality_names.append("robot_ee_pose_fk")
        if self.record_current_pose:
            modality_names.append("robot_controller_current_pose")
        if self.record_cmd_quat_pose:
            modality_names.append("robot_command_quat_pose")
        if not self.record_hand:
            modality_names.remove("robot_hand_joint")
        if self.record_contact_state:
            modality_names.append("robot_contact_state")
        if self.record_contact_phase:
            modality_names.append("robot_contact_phase")
        if self.record_contact_observation:
            modality_names.append("contact_observation")
        if self.record_ft_wrench:
            modality_names.append("ft_wrench")
        if self.record_ft_base:
            modality_names.append("ft_base_wrench")
        if self.record_jt_tared_wrench:
            modality_names.append("jt_tared_wrench")
        if self.record_jt_tared_filtered_wrench:
            modality_names.append("jt_tared_filtered_wrench")
        if self.record_jt:
            modality_names.append("jt_joint_torque")
        for name in modality_names:
            self.recent[name] = deque(maxlen=20000)
            self.counts[name] = 0
        self.write_error: Optional[str] = None
        # A health/timestamp auto-stop can still be saved for diagnosis.  Keep
        # that episode distinguishable from a clean operator stop so the raw
        # checker cannot accidentally approve it for conversion.
        self.interruption_reason: Optional[str] = None
        self._closed = False
        self._queue_sentinel = object()
        self.write_queue: "queue.Queue[Any]" = queue.Queue(
            maxsize=max(1, int(getattr(args, "writer_queue_size", 4096)))
        )
        self.writer_queue_peak = 0
        self.writer_queue_max_bytes = max(
            1, int(getattr(args, "writer_queue_max_bytes", 512 * 1024 * 1024))
        )
        self.writer_queue_bytes = 0
        self.writer_queue_peak_bytes = 0
        self.rss_start_bytes = current_process_rss_bytes()
        self.rss_peak_bytes = self.rss_start_bytes
        self.writer_thread = threading.Thread(
            target=self._writer_loop,
            name=f"{final_dir.name}_writer",
            daemon=True,
        )
        self.writer_thread.start()

    def _mark(self, modality: str, timestamp: float) -> None:
        self.recent[modality].append(float(timestamp))
        self.counts[modality] += 1

    def _set_write_error(self, error: str) -> None:
        with self.lock:
            if self.write_error is None:
                self.write_error = error

    def set_interruption_reason(self, reason: str) -> None:
        reason = str(reason).strip()
        if not reason:
            return
        with self.lock:
            if self.interruption_reason is None:
                self.interruption_reason = reason

    def _enqueue(
        self,
        modality: str,
        timestamp: float,
        payload: Tuple[Any, ...],
        mark_modalities: Optional[Tuple[str, ...]] = None,
    ) -> None:
        payload_bytes = payload_nbytes(payload)
        with self.lock:
            if self._closed:
                self.write_error = self.write_error or "write requested after close"
                return
            if self.writer_queue_bytes + payload_bytes > self.writer_queue_max_bytes:
                self.write_error = self.write_error or (
                    f"writer byte budget exceeded at {modality}: "
                    f"{self.writer_queue_bytes + payload_bytes}/"
                    f"{self.writer_queue_max_bytes} bytes"
                )
                return
            try:
                self.write_queue.put_nowait(payload)
            except queue.Full:
                self.write_error = self.write_error or (
                    f"writer queue full at {modality}; disk writer cannot keep up"
                )
                return
            self.writer_queue_bytes += payload_bytes
            self.writer_queue_peak_bytes = max(
                self.writer_queue_peak_bytes, self.writer_queue_bytes)
            for name in mark_modalities or (modality,):
                self._mark(name, timestamp)
            self.writer_queue_peak = max(
                self.writer_queue_peak, self.write_queue.qsize()
            )

    def _writer_loop(self) -> None:
        while True:
            item = self.write_queue.get()
            batch = [item]
            max_batch = max(1, int(getattr(self.args, "writer_batch_size", 128)))
            stop_after_batch = item is self._queue_sentinel
            while not stop_after_batch and len(batch) < max_batch:
                try:
                    next_item = self.write_queue.get_nowait()
                except queue.Empty:
                    break
                batch.append(next_item)
                if next_item is self._queue_sentinel:
                    stop_after_batch = True
                    break
            try:
                self._write_batch(batch)
            except Exception as exc:
                kind = "sentinel" if item is self._queue_sentinel else str(item[0])
                self._set_write_error(f"writer failed on {kind}: {exc}")
            finally:
                released_bytes = sum(
                    payload_nbytes(queued)
                    for queued in batch
                    if queued is not self._queue_sentinel
                )
                with self.lock:
                    self.writer_queue_bytes = max(
                        0, self.writer_queue_bytes - released_bytes)
                for _ in batch:
                    self.write_queue.task_done()
            if stop_after_batch:
                return

    def _write_batch(self, batch: List[Any]) -> None:
        rgb_by_camera: Dict[int, List[Tuple[float, np.ndarray, float, int]]] = {}
        depth_by_camera: Dict[int, List[Tuple[float, np.ndarray, float, int]]] = {}
        robot_samples = []
        hand_samples = []
        command_samples = []
        current_pose_samples = []
        command_quat_samples = []
        contact_state_samples = []
        contact_phase_samples = []
        contact_observation_samples = []
        ft_samples = []
        ft_base_samples = []
        jt_tared_wrench_samples = []
        jt_tared_filtered_wrench_samples = []
        jt_samples = []

        for item in batch:
            if item is self._queue_sentinel:
                continue
            kind = item[0]
            if kind == "camera_rgb":
                _, camera_id, timestamp, rgb, hardware_timestamp_ms, frame_number = item
                rgb_by_camera.setdefault(camera_id, []).append(
                    (timestamp, rgb, hardware_timestamp_ms, frame_number)
                )
            elif kind == "camera_depth":
                _, camera_id, timestamp, depth, hardware_timestamp_ms, frame_number = item
                depth_by_camera.setdefault(camera_id, []).append(
                    (timestamp, depth, hardware_timestamp_ms, frame_number)
                )
            elif kind == "robot":
                robot_samples.append(item)
            elif kind == "hand":
                hand_samples.append(item)
            elif kind == "command":
                command_samples.append(item)
            elif kind == "current_pose":
                current_pose_samples.append(item)
            elif kind == "command_quat":
                command_quat_samples.append(item)
            elif kind == "contact_state":
                contact_state_samples.append(item)
            elif kind == "contact_phase":
                contact_phase_samples.append(item)
            elif kind == "contact_observation":
                contact_observation_samples.append(item)
            elif kind == "ft":
                ft_samples.append(item)
            elif kind == "ft_base":
                ft_base_samples.append(item)
            elif kind == "jt_tared_wrench":
                jt_tared_wrench_samples.append(item)
            elif kind == "jt_tared_filtered_wrench":
                jt_tared_filtered_wrench_samples.append(item)
            elif kind == "jt":
                jt_samples.append(item)
            else:
                self._set_write_error(f"unknown writer queue item: {kind}")

        for camera_id, samples in rgb_by_camera.items():
            self.camera_sinks[camera_id].write_rgb_batch(samples)
        for camera_id, samples in depth_by_camera.items():
            self.camera_sinks[camera_id].write_depth_batch(samples)

        if robot_samples:
            timestamps = [item[1] for item in robot_samples]
            joint_deg = np.asarray([item[2] for item in robot_samples], dtype=np.float64)
            append_zarr_rows(self.joint_deg, joint_deg)
            append_zarr_scalars(self.joint_ts, timestamps)
            if (
                self.record_ee_pose_fk
                and self.ee_pose is not None
                and self.ee_pose_ts is not None
            ):
                ee_pose = np.asarray(
                    [item[3] for item in robot_samples], dtype=np.float64
                )
                append_zarr_rows(self.ee_pose, ee_pose)
                append_zarr_scalars(self.ee_pose_ts, timestamps)

        if (
            hand_samples
            and self.hand_joint is not None
            and self.hand_joint_ts is not None
        ):
            timestamps = [item[1] for item in hand_samples]
            hand_joint = np.asarray(
                [item[2] for item in hand_samples], dtype=np.float64
            )
            append_zarr_rows(self.hand_joint, hand_joint)
            append_zarr_scalars(self.hand_joint_ts, timestamps)

        if command_samples:
            timestamps = [item[1] for item in command_samples]
            command_pose = np.asarray(
                [item[2] for item in command_samples], dtype=np.float64
            )
            append_zarr_rows(self.command_pose, command_pose)
            append_zarr_scalars(self.command_pose_ts, timestamps)

        if (
            current_pose_samples
            and self.current_pose is not None
            and self.current_pose_ts is not None
        ):
            timestamps = [item[1] for item in current_pose_samples]
            current_pose = np.asarray(
                [item[2] for item in current_pose_samples], dtype=np.float64
            )
            append_zarr_rows(self.current_pose, current_pose)
            append_zarr_scalars(self.current_pose_ts, timestamps)

        if (
            command_quat_samples
            and self.command_quat_pose is not None
            and self.command_quat_pose_ts is not None
        ):
            timestamps = [item[1] for item in command_quat_samples]
            command_quat_pose = np.asarray(
                [item[2] for item in command_quat_samples], dtype=np.float64
            )
            append_zarr_rows(self.command_quat_pose, command_quat_pose)
            append_zarr_scalars(self.command_quat_pose_ts, timestamps)

        if (
            contact_state_samples
            and self.contact_state is not None
            and self.contact_state_ts is not None
        ):
            timestamps = [item[1] for item in contact_state_samples]
            contact_state = [int(item[2]) for item in contact_state_samples]
            append_zarr_scalars(self.contact_state, contact_state)
            append_zarr_scalars(self.contact_state_ts, timestamps)

        if (
            contact_phase_samples
            and self.contact_phase is not None
            and self.contact_phase_ts is not None
        ):
            timestamps = [item[1] for item in contact_phase_samples]
            contact_phase = [int(item[2]) for item in contact_phase_samples]
            append_zarr_scalars(self.contact_phase, contact_phase)
            append_zarr_scalars(self.contact_phase_ts, timestamps)

        if contact_observation_samples and self.contact_wrench is not None:
            # Queue shape: kind, source_t, receive_t, sequence, residual,
            # state, valid, model_ready, prediction_age_ms, prediction.
            append_zarr_rows(
                self.contact_wrench,
                np.asarray(
                    [item[4] for item in contact_observation_samples],
                    dtype=np.float64,
                ),
            )
            append_zarr_rows(
                self.free_space_wrench_prediction,
                np.asarray(
                    [item[9] for item in contact_observation_samples],
                    dtype=np.float64,
                ),
            )
            append_zarr_scalars(
                self.canonical_contact_state,
                [int(item[5]) for item in contact_observation_samples],
            )
            append_zarr_scalars(
                self.contact_valid,
                [int(bool(item[6])) for item in contact_observation_samples],
            )
            append_zarr_scalars(
                self.contact_model_ready,
                [int(bool(item[7])) for item in contact_observation_samples],
            )
            append_zarr_scalars(
                self.contact_source_ts,
                [float(item[1]) for item in contact_observation_samples],
            )
            append_zarr_scalars(
                self.contact_receive_ts,
                [float(item[2]) for item in contact_observation_samples],
            )
            append_zarr_scalars(
                self.contact_source_sequences,
                [int(item[3]) for item in contact_observation_samples],
            )
            append_zarr_scalars(
                self.contact_prediction_age_ms,
                [float(item[8]) for item in contact_observation_samples],
            )

        if ft_samples and self.wrench_ts is not None:
            timestamps = [item[1] for item in ft_samples]
            if self.wrench_raw is not None:
                append_zarr_rows(
                    self.wrench_raw,
                    np.asarray([item[2] for item in ft_samples], dtype=np.float64),
                )
            if self.wrench_payload_gravity is not None:
                append_zarr_rows(
                    self.wrench_payload_gravity,
                    np.asarray([item[3] for item in ft_samples], dtype=np.float64),
                )
            if self.wrench_comp_payload is not None:
                append_zarr_rows(
                    self.wrench_comp_payload,
                    np.asarray([item[4] for item in ft_samples], dtype=np.float64),
                )
            append_zarr_scalars(self.wrench_ts, timestamps)

        if ft_base_samples and self.wrench_base is not None and self.wrench_base_ts is not None:
            timestamps = [item[1] for item in ft_base_samples]
            wrench_base = np.asarray(
                [item[2] for item in ft_base_samples], dtype=np.float64
            )
            append_zarr_rows(self.wrench_base, wrench_base)
            append_zarr_scalars(self.wrench_base_ts, timestamps)

        if (
            jt_tared_wrench_samples
            and self.jt_tared_wrench is not None
            and self.jt_tared_wrench_ts is not None
        ):
            timestamps = [item[1] for item in jt_tared_wrench_samples]
            jt_tared_wrench = np.asarray(
                [item[2] for item in jt_tared_wrench_samples], dtype=np.float64
            )
            append_zarr_rows(self.jt_tared_wrench, jt_tared_wrench)
            append_zarr_scalars(self.jt_tared_wrench_ts, timestamps)

        if (
            jt_tared_filtered_wrench_samples
            and self.jt_tared_filtered_wrench is not None
            and self.jt_tared_filtered_wrench_ts is not None
        ):
            timestamps = [item[1] for item in jt_tared_filtered_wrench_samples]
            jt_tared_filtered_wrench = np.asarray(
                [item[2] for item in jt_tared_filtered_wrench_samples], dtype=np.float64
            )
            append_zarr_rows(self.jt_tared_filtered_wrench, jt_tared_filtered_wrench)
            append_zarr_scalars(self.jt_tared_filtered_wrench_ts, timestamps)

        if jt_samples and self.joint_torque is not None and self.joint_torque_ts is not None:
            timestamps = [item[1] for item in jt_samples]
            joint_torque = np.asarray(
                [item[2] for item in jt_samples], dtype=np.float64
            )
            append_zarr_rows(self.joint_torque, joint_torque)
            append_zarr_scalars(self.joint_torque_ts, timestamps)

    def write_camera_rgb(
        self,
        camera_id: int,
        timestamp: float,
        rgb: np.ndarray,
        hardware_timestamp_ms: float = math.nan,
        frame_number: int = -1,
    ) -> None:
        self._enqueue(
            f"camera_{camera_id}_rgb",
            timestamp,
            ("camera_rgb", camera_id, timestamp, rgb, hardware_timestamp_ms, frame_number),
        )

    def write_camera_depth(
        self,
        camera_id: int,
        timestamp: float,
        depth: np.ndarray,
        hardware_timestamp_ms: float = math.nan,
        frame_number: int = -1,
    ) -> None:
        self._enqueue(
            f"camera_{camera_id}_depth",
            timestamp,
            (
                "camera_depth",
                camera_id,
                timestamp,
                depth,
                hardware_timestamp_ms,
                frame_number,
            ),
        )

    def write_robot(
        self,
        timestamp: float,
        joint_deg: np.ndarray,
        ee_pose: Optional[np.ndarray],
    ):
        stored_pose = None if ee_pose is None else np.asarray(ee_pose).copy()
        mark_modalities = ["robot_joint"]
        if self.record_ee_pose_fk:
            if stored_pose is None:
                raise ValueError("record_ee_pose_fk=true requires a computed FK pose")
            mark_modalities.append("robot_ee_pose_fk")
        self._enqueue(
            "robot_joint",
            timestamp,
            ("robot", timestamp, joint_deg.copy(), stored_pose),
            mark_modalities=tuple(mark_modalities),
        )

    def write_hand_joint(self, timestamp: float, hand_joint: np.ndarray) -> None:
        if not self.record_hand or self.hand_joint is None or self.hand_joint_ts is None:
            return
        hand_joint = np.asarray(hand_joint, dtype=np.float64).reshape(
            len(JOINT_NAMES_HAND_RIGHT)
        )
        self._enqueue(
            "robot_hand_joint",
            timestamp,
            ("hand", timestamp, hand_joint.copy()),
        )

    def write_command(self, timestamp: float, command_pose: np.ndarray) -> None:
        self._enqueue(
            "robot_command_pose",
            timestamp,
            ("command", timestamp, command_pose.copy()),
        )

    def write_current_pose(self, timestamp: float, current_pose: np.ndarray) -> None:
        if (
            not self.record_current_pose
            or self.current_pose is None
            or self.current_pose_ts is None
        ):
            return
        self._enqueue(
            "robot_controller_current_pose",
            timestamp,
            ("current_pose", timestamp, current_pose.copy()),
        )

    def write_command_quat(self, timestamp: float, command_pose: np.ndarray) -> None:
        if (
            not self.record_cmd_quat_pose
            or self.command_quat_pose is None
            or self.command_quat_pose_ts is None
        ):
            return
        self._enqueue(
            "robot_command_quat_pose",
            timestamp,
            ("command_quat", timestamp, command_pose.copy()),
        )

    def write_contact_state(self, timestamp: float, contact_state: int) -> None:
        if (
            not self.record_contact_state
            or self.contact_state is None
            or self.contact_state_ts is None
        ):
            return
        self._enqueue(
            "robot_contact_state",
            timestamp,
            ("contact_state", timestamp, int(contact_state)),
        )

    def write_contact_phase(self, timestamp: float, contact_phase: int) -> None:
        if (
            not self.record_contact_phase
            or self.contact_phase is None
            or self.contact_phase_ts is None
        ):
            return
        self._enqueue(
            "robot_contact_phase",
            timestamp,
            ("contact_phase", timestamp, int(contact_phase)),
        )

    def write_contact_observation(
        self,
        source_timestamp: float,
        receive_timestamp: float,
        source_sequence: int,
        contact_wrench: np.ndarray,
        contact_state: int,
        valid: bool,
        model_ready: bool,
        prediction_age_ms: float,
        free_space_wrench_prediction: Optional[np.ndarray] = None,
    ) -> None:
        if not self.record_contact_observation or self.contact_wrench is None:
            return
        wrench = np.asarray(contact_wrench, dtype=np.float64).reshape(6)
        prediction = np.asarray(
            np.zeros(6)
            if free_space_wrench_prediction is None
            else free_space_wrench_prediction,
            dtype=np.float64,
        ).reshape(6)
        self._enqueue(
            "contact_observation",
            source_timestamp,
            (
                "contact_observation",
                float(source_timestamp),
                float(receive_timestamp),
                int(source_sequence),
                wrench.copy(),
                int(contact_state),
                bool(valid),
                bool(model_ready),
                float(prediction_age_ms),
                prediction.copy(),
            ),
        )

    def write_ft(
        self,
        timestamp: float,
        wrench_raw: np.ndarray,
        wrench_payload_gravity: np.ndarray,
        wrench_comp_payload: np.ndarray,
    ) -> None:
        if not self.record_ft_wrench or self.wrench_ts is None:
            return
        self._enqueue(
            "ft_wrench",
            timestamp,
            (
                "ft",
                timestamp,
                wrench_raw.copy(),
                wrench_payload_gravity.copy(),
                wrench_comp_payload.copy(),
            ),
        )

    def write_ft_base(self, timestamp: float, wrench_base: np.ndarray) -> None:
        if not self.record_ft_base or self.wrench_base is None or self.wrench_base_ts is None:
            return
        self._enqueue(
            "ft_base_wrench",
            timestamp,
            ("ft_base", timestamp, wrench_base.copy()),
        )

    def write_jt_tared_wrench(
        self, timestamp: float, jt_tared_wrench: np.ndarray
    ) -> None:
        if (
            not self.record_jt_tared_wrench
            or self.jt_tared_wrench is None
            or self.jt_tared_wrench_ts is None
        ):
            return
        self._enqueue(
            "jt_tared_wrench",
            timestamp,
            ("jt_tared_wrench", timestamp, jt_tared_wrench.copy()),
        )

    def write_jt_tared_filtered_wrench(
        self, timestamp: float, jt_tared_filtered_wrench: np.ndarray
    ) -> None:
        if (
            not self.record_jt_tared_filtered_wrench
            or self.jt_tared_filtered_wrench is None
            or self.jt_tared_filtered_wrench_ts is None
        ):
            return
        self._enqueue(
            "jt_tared_filtered_wrench",
            timestamp,
            ("jt_tared_filtered_wrench", timestamp, jt_tared_filtered_wrench.copy()),
        )

    def write_jt(self, timestamp: float, joint_torque: np.ndarray) -> None:
        if not self.record_jt or self.joint_torque is None or self.joint_torque_ts is None:
            return
        self._enqueue(
            "jt_joint_torque",
            timestamp,
            ("jt", timestamp, joint_torque.copy()),
        )

    def health_failures(
        self,
        target_hz: Dict[str, float],
        window_sec: float,
        min_ratio: float,
        max_stale_sec: float,
    ) -> List[str]:
        failures = []
        t_now = now_s()
        with self.lock:
            if self.write_error:
                failures.append(self.write_error)
            queue_size = self.write_queue.qsize()
            self.writer_queue_peak = max(self.writer_queue_peak, queue_size)
            current_rss = current_process_rss_bytes()
            self.rss_peak_bytes = max(self.rss_peak_bytes, current_rss)
            # Queue pressure at writer_queue_warn_ratio is diagnostic only.
            # _enqueue() sets write_error at the hard item/byte limits, which
            # remains the fail-closed condition consumed above.
            for modality, target in target_hz.items():
                ts = list(self.recent.get(modality, []))
                hz = estimate_hz(ts, window_sec, t_now)
                last_age = None if not ts else t_now - ts[-1]
                if last_age is None:
                    failures.append(f"{modality}: no recorded samples")
                elif last_age > max_stale_sec:
                    failures.append(
                        f"{modality}: stale latest sample age={last_age:.3f}s"
                    )
                elif hz < target * min_ratio:
                    failures.append(
                        f"{modality}: hz={hz:.1f}, target={target:.1f}, "
                        f"min={target * min_ratio:.1f}"
                    )
        return failures

    def sample_counts(self) -> Dict[str, int]:
        with self.lock:
            return dict(self.counts)

    def close(self) -> None:
        with self.lock:
            if self._closed:
                return
            self._closed = True
        self.write_queue.join()
        self.write_queue.put(self._queue_sentinel)
        self.writer_thread.join(timeout=10.0)
        with self.lock:
            final_queue_items = self.write_queue.qsize()
            final_queue_bytes = self.writer_queue_bytes
            writer_thread_alive = self.writer_thread.is_alive()
            if writer_thread_alive or final_queue_items or final_queue_bytes:
                self.write_error = self.write_error or (
                    "episode resource release audit failed: "
                    f"thread_alive={writer_thread_alive}, "
                    f"queue_items={final_queue_items}, "
                    f"queue_bytes={final_queue_bytes}"
                )
            self.stop_wall_time = now_s()
            final_rss = current_process_rss_bytes()
            self.rss_peak_bytes = max(self.rss_peak_bytes, final_rss)
            self.writer_queue_peak = max(
                self.writer_queue_peak, self.write_queue.qsize()
            )
            for sink in self.camera_sinks.values():
                sink.close()
            meta = {
                "created_at": self.start_wall_time,
                "stopped_at": self.stop_wall_time,
                "counts": dict(self.counts),
                "writer_queue_capacity": self.write_queue.maxsize,
                "writer_queue_peak": self.writer_queue_peak,
                "writer_queue_peak_fraction": (
                    self.writer_queue_peak / max(1, self.write_queue.maxsize)
                ),
                "writer_queue_max_bytes": self.writer_queue_max_bytes,
                "writer_queue_peak_bytes": self.writer_queue_peak_bytes,
                "writer_queue_final_items": final_queue_items,
                "writer_queue_final_bytes": final_queue_bytes,
                "writer_thread_alive_after_close": writer_thread_alive,
                "episode_frame_reference_release_ok": (
                    not writer_thread_alive
                    and final_queue_items == 0
                    and final_queue_bytes == 0
                ),
                "writer_queue_peak_byte_fraction": (
                    self.writer_queue_peak_bytes
                    / max(1, self.writer_queue_max_bytes)
                ),
                "writer_error": self.write_error,
                "interruption_reason": self.interruption_reason,
                "model_sha256": str(getattr(self.args, "model_sha256", "")),
                "feedback_gain_scale_contract": float(
                    getattr(self.args, "feedback_gain_scale_contract", 0.0)
                ),
                "rss_start_bytes": self.rss_start_bytes,
                "rss_final_bytes": final_rss,
                "rss_peak_bytes": self.rss_peak_bytes,
                "rss_growth_bytes": final_rss - self.rss_start_bytes,
                "camera_fps": self.args.camera_fps,
                "robot_sample_hz": self.args.robot_sample_hz,
                "command_target_hz": self.args.command_hz,
                "ft_target_hz": self.args.ft_hz,
                "ft_base_target_hz": self.args.ft_base_hz,
                "jt_tared_wrench_target_hz": self.args.jt_tared_wrench_hz,
                "jt_tared_filtered_wrench_target_hz": (
                    self.args.jt_tared_filtered_wrench_hz
                ),
                "contact_state_target_hz": getattr(
                    self.args, "contact_state_hz", None
                ),
                "contact_phase_target_hz": getattr(
                    self.args, "contact_phase_hz", None
                ),
                "contact_observation_target_hz": getattr(
                    self.args, "contact_observation_hz", None
                ),
                "frames": {
                    "base_frame": self.args.base_frame,
                    "ee_frame": self.args.ee_frame,
                    "ft_frame": self.args.ft_frame,
                    "ft_sensor_origin_approximation": self.args.ft_frame,
                },
                # These are observed ROS header values, not declarations copied
                # from YAML. Callbacks reject a missing, mismatched, or changing
                # frame before the corresponding numeric row can be queued.
                "observed_source_frames": dict(
                    getattr(self.args, "observed_source_frames", {})
                ),
                "contact_observation_frame_id": (
                    getattr(self.args, "observed_source_frames", {}).get(
                        "contact_observation"
                    )
                    if self.record_contact_observation
                    else None
                ),
                "record_jt": self.record_jt,
                "record_hand": self.record_hand,
                "record_ee_pose_fk": self.record_ee_pose_fk,
                "record_cmd_pose": bool(getattr(self.args, "record_cmd_pose", True)),
                "record_current_pose": self.record_current_pose,
                "record_cmd_quat_pose": bool(
                    getattr(self.args, "record_cmd_quat_pose", False)
                ),
                "record_contact_state": self.record_contact_state,
                "record_contact_phase": self.record_contact_phase,
                "record_contact_observation": self.record_contact_observation,
                "use_observer_input_robot_streams": bool(
                    getattr(self.args, "use_observer_input_robot_streams", False)
                ),
                "command_pose_source": (
                    (
                        "ObserverInput.desired_pose"
                        if getattr(self.args, "use_observer_input_robot_streams", False)
                        else "command_topic"
                    )
                    if getattr(self.args, "record_cmd_pose", True)
                    else "ee_pose_fk_feedback"
                ),
                "command_msg_type": getattr(self.args, "command_msg_type", "auto"),
                "command_position_unit": getattr(
                    self.args, "command_position_unit", None
                ),
                "command_float64_euler_order": getattr(
                    self.args, "command_float64_euler_order", None
                ),
                "current_pose_target_hz": getattr(
                    self.args, "current_pose_hz", None
                ),
                "current_pose_position_unit": getattr(
                    self.args, "current_pose_position_unit", None
                ),
                "current_pose_float64_euler_order": getattr(
                    self.args, "current_pose_float64_euler_order", None
                ),
                "command_quat_target_hz": getattr(
                    self.args, "command_quat_hz", None
                ),
                "command_quat_msg_type": getattr(
                    self.args, "command_quat_msg_type", None
                ),
                "record_ft_wrench_raw": self.record_ft_wrench_raw,
                "record_ft_wrench_payload_gravity": (
                    self.record_ft_wrench_payload_gravity
                ),
                "record_ft_wrench_comp_payload": (
                    self.record_ft_wrench_comp_payload
                ),
                "record_ft_base": self.record_ft_base,
                "record_jt_tared_wrench": self.record_jt_tared_wrench,
                "record_jt_tared_filtered_wrench": (
                    self.record_jt_tared_filtered_wrench
                ),
                "recorded_ft_streams": recorded_ft_stream_names(self.args),
                "ft_base_topic": getattr(self.args, "ft_base_topic", None),
                "source_topics": {
                    "robot_joint_position": self.args.joint_topic,
                    "robot_hand_joint_position": (
                        self.args.hand_joint_topic if self.record_hand else None
                    ),
                    "robot_ee_pose_fk_input": (
                        self.args.joint_topic if self.record_ee_pose_fk else None
                    ),
                    "joint_torque_effort": self.args.joint_torque_topic,
                    "observer_input": (
                        self.args.observer_input_topic
                        if getattr(self.args, "use_observer_input_robot_streams", False)
                        else None
                    ),
                    "robot_command_pose": (
                        (
                            self.args.observer_input_topic
                            if getattr(self.args, "use_observer_input_robot_streams", False)
                            else self.args.command_topic
                        )
                        if getattr(self.args, "record_cmd_pose", True)
                        else None
                    ),
                    "robot_controller_current_pose": (
                        (
                            self.args.observer_input_topic
                            if getattr(self.args, "use_observer_input_robot_streams", False)
                            else self.args.current_pose_topic
                        )
                        if self.record_current_pose
                        else None
                    ),
                    "robot_command_quat_pose": (
                        self.args.command_quat_topic
                        if getattr(self.args, "record_cmd_quat_pose", False)
                        else None
                    ),
                    "robot_contact_state": (
                        self.args.contact_state_topic
                        if self.record_contact_state
                        else None
                    ),
                    "robot_contact_phase": (
                        self.args.contact_phase_topic
                        if self.record_contact_phase
                        else None
                    ),
                    "contact_observation": (
                        self.args.contact_observation_topic
                        if self.record_contact_observation
                        else None
                    ),
                    "ft_wrench_raw": (
                        self.args.ft_topic if self.record_ft_wrench else None
                    ),
                    "ft_wrench_base": (
                        self.args.ft_base_topic if self.record_ft_base else None
                    ),
                    "jt_tared_wrench": (
                        (
                            self.args.observer_input_topic
                            if getattr(self.args, "use_observer_input_robot_streams", False)
                            else self.args.jt_tared_wrench_topic
                        )
                        if self.record_jt_tared_wrench
                        else None
                    ),
                    "jt_tared_filtered_wrench": (
                        (
                            self.args.observer_input_topic
                            if getattr(self.args, "use_observer_input_robot_streams", False)
                            else self.args.jt_tared_filtered_wrench_topic
                        )
                        if self.record_jt_tared_filtered_wrench
                        else None
                    ),
                },
                "camera_calibration": getattr(self.args, "camera_calibration", {}),
                "camera_roles": configured_camera_roles(self.args),
                "recorder_config": recorder_config_snapshot(self.args),
                "camera_timing": {
                    "host_timestamp": "time.time() seconds",
                    "hardware_timestamp": "RealSense frame.get_timestamp() milliseconds",
                    "frame_number": "RealSense frame.get_frame_number()",
                },
                "ft_processing": getattr(self.args, "ft_processing_metadata", {}),
                "units": {
                    "joint_deg": "degree",
                    "hand_joint": "radian",
                    "ee_pose_fk_se3": "meter",
                    "command_pose_se3": "meter",
                    "controller_current_pose_se3": "meter",
                    "command_quat_pose_se3": "meter",
                    "contact_state": "int32, -1=no_contact, 1=contact",
                    "contact_phase": "int32, -1=no_contact, 0=pre_contact, 1=contact",
                    "canonical_contact_wrench": "[N, N, N, Nm, Nm, Nm]",
                    "free_space_wrench_prediction": "[N, N, N, Nm, Nm, Nm]",
                    "canonical_contact_state": "uint8, 0=free, 1=contact",
                    "canonical_contact_valid": "uint8 boolean mask",
                    "canonical_contact_model_ready": "uint8 boolean mask",
                    "contact_source_time_stamps": "ROS source clock seconds",
                    "contact_receive_time_stamps": "PC time.time() seconds",
                    "contact_prediction_age_ms": "millisecond",
                    "wrench_raw": "[N, N, N, Nm, Nm, Nm]",
                    "wrench_payload_gravity": "[N, N, N, Nm, Nm, Nm]",
                    "wrench_comp_payload": "[N, N, N, Nm, Nm, Nm]",
                    "wrench_base": "[N, N, N, Nm, Nm, Nm]",
                    "jt_tared_wrench": "[N, N, N, Nm, Nm, Nm]",
                    "jt_tared_filtered_wrench": "[N, N, N, Nm, Nm, Nm]",
                    "depth": "uint16_depth_units",
                },
                "hand_joint_names": list(JOINT_NAMES_HAND_RIGHT),
            }
            if not self.record_hand:
                meta["units"].pop("hand_joint", None)
            if not self.record_ee_pose_fk:
                meta["units"].pop("ee_pose_fk_se3", None)
            if not self.record_contact_state:
                meta["units"].pop("contact_state", None)
            if not self.record_contact_phase:
                meta["units"].pop("contact_phase", None)
            if not self.record_contact_observation:
                for key in (
                    "canonical_contact_wrench",
                    "canonical_contact_state",
                    "canonical_contact_valid",
                    "canonical_contact_model_ready",
                    "contact_source_time_stamps",
                    "contact_receive_time_stamps",
                    "contact_prediction_age_ms",
                ):
                    meta["units"].pop(key, None)
            if not self.record_current_pose:
                meta["units"].pop("controller_current_pose_se3", None)
            if not self.record_cmd_quat_pose:
                meta["units"].pop("command_quat_pose_se3", None)
            if not self.record_ft_wrench_raw:
                meta["units"].pop("wrench_raw", None)
            if not self.record_ft_wrench_payload_gravity:
                meta["units"].pop("wrench_payload_gravity", None)
            if not self.record_ft_wrench_comp_payload:
                meta["units"].pop("wrench_comp_payload", None)
            if not self.record_ft_base:
                meta["units"].pop("wrench_base", None)
            if not self.record_jt_tared_wrench:
                meta["units"].pop("jt_tared_wrench", None)
            if not self.record_jt_tared_filtered_wrench:
                meta["units"].pop("jt_tared_filtered_wrench", None)
            if self.record_jt:
                meta["units"]["joint_torque"] = "raw_effort"
            write_json(self.temp_dir / "meta.json", meta)


class FTProcessor:
    """Compute raw/payload-compensated wrench using the teleop FT convention."""

    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.lock = threading.RLock()
        self.latest_joint_rad: Optional[np.ndarray] = None
        self.latest_payload_gravity = np.zeros(6, dtype=np.float64)
        payload_output_requested = bool(
            getattr(args, "record_ft_wrench_payload_gravity", False)
            or getattr(args, "record_ft_wrench_comp_payload", False)
        )
        self.payload_enabled = bool(
            args.ft_payload_gravity_comp and payload_output_requested
        )
        self.payload_status = "disabled"
        self.payload_link_data: List[Tuple[str, int, float, np.ndarray]] = []
        self.payload_gravity_sign = parse_six_floats(
            args.ft_payload_gravity_sign, "--ft-payload-gravity-sign"
        )

        self.model = None
        self.data = None
        self.q_full = None
        self.idxq: List[int] = []
        self.ft_frame_id: Optional[int] = None
        if self.payload_enabled:
            self._setup_payload_gravity_model()
            if not self.payload_enabled:
                raise RuntimeError(
                    "Payload-gravity FT output was requested, but its model "
                    f"could not be initialized: {self.payload_status}"
                )
        elif args.ft_payload_gravity_comp and not payload_output_requested:
            self.payload_status = "disabled_no_payload_output_requested"
        else:
            self.payload_status = "disabled_by_cli"

    def _joint_q_indices(self, model, names: List[str]) -> Optional[List[int]]:
        idx = []
        for name in names:
            if not model.existJointName(name):
                return None
            jid = model.getJointId(name)
            joint = model.joints[jid]
            if joint.nq != 1:
                raise RuntimeError(f"Joint '{name}' must have nq=1, got {joint.nq}.")
            idx.append(joint.idx_q)
        return idx

    def _load_urdf_tables(
        self,
    ) -> Tuple[Dict[str, List[str]], Dict[str, Tuple[float, np.ndarray]]]:
        root = ET.parse(self.args.follower_urdf).getroot()
        children: Dict[str, List[str]] = {}
        for joint in root.findall("joint"):
            parent = joint.find("parent")
            child = joint.find("child")
            if parent is None or child is None:
                continue
            parent_link = parent.get("link")
            child_link = child.get("link")
            if parent_link and child_link:
                children.setdefault(parent_link, []).append(child_link)

        inertials: Dict[str, Tuple[float, np.ndarray]] = {}
        for link in root.findall("link"):
            link_name = link.get("name")
            if not link_name:
                continue
            inertial = link.find("inertial")
            if inertial is None:
                continue
            mass_node = inertial.find("mass")
            if mass_node is None:
                continue
            mass = float(mass_node.get("value", "0"))
            if mass <= 0.0:
                continue
            origin = inertial.find("origin")
            com_local = np.zeros(3, dtype=np.float64)
            if origin is not None:
                com_local = parse_xyz_attr(origin.get("xyz"))
            inertials[str(link_name)] = (mass, com_local)
        return children, inertials

    @staticmethod
    def _descendant_links(
        root_link: str, children_by_parent: Dict[str, List[str]]
    ) -> List[str]:
        out: List[str] = []
        stack = [str(root_link)]
        seen = set()
        while stack:
            link = stack.pop()
            if link in seen:
                continue
            seen.add(link)
            out.append(link)
            stack.extend(reversed(children_by_parent.get(link, [])))
        return out

    def _setup_payload_gravity_model(self) -> None:
        try:
            _pin = load_pinocchio()
            self.model = _pin.buildModelFromUrdf(str(self.args.follower_urdf))
            self.data = self.model.createData()
            self.q_full = np.zeros(self.model.nq, dtype=np.float64)

            right_names = [f"right_joint_{i}" for i in range(1, 7)]
            generic_names = [f"joint_{i}" for i in range(1, 7)]
            idxq = self._joint_q_indices(self.model, right_names)
            if idxq is None:
                idxq = self._joint_q_indices(self.model, generic_names)
            if idxq is None:
                raise RuntimeError("right_joint_1~6 or joint_1~6 not found in URDF.")
            self.idxq = idxq

            frame_names = {f.name for f in self.model.frames}
            if self.args.ft_frame not in frame_names:
                raise RuntimeError(f"FT frame '{self.args.ft_frame}' not found in URDF.")
            self.ft_frame_id = int(self.model.getFrameId(self.args.ft_frame))

            children, inertials = self._load_urdf_tables()
            root_link = str(self.args.ft_payload_root)
            if root_link not in children and root_link not in inertials:
                raise RuntimeError(
                    f"payload root link '{root_link}' not found in URDF graph."
                )
            payload_links = self._descendant_links(root_link, children)
            missing_frames = []
            for link_name in payload_links:
                inertial = inertials.get(link_name)
                if inertial is None:
                    continue
                if link_name not in frame_names:
                    missing_frames.append(link_name)
                    continue
                mass, com_local = inertial
                self.payload_link_data.append(
                    (
                        link_name,
                        int(self.model.getFrameId(link_name)),
                        float(mass),
                        np.asarray(com_local, dtype=np.float64).reshape(3),
                    )
                )
            if not self.payload_link_data:
                raise RuntimeError(
                    f"payload root '{root_link}' has no inertial payload links."
                )
            total_mass = sum(item[2] for item in self.payload_link_data)
            self.payload_status = (
                f"enabled root={root_link}, links={len(self.payload_link_data)}, "
                f"mass={total_mass:.6f}kg"
            )
            if missing_frames:
                self.payload_status += f", missing_frames={missing_frames[:6]}"
        except Exception as exc:
            self.payload_enabled = False
            self.payload_link_data = []
            self.payload_status = f"disabled_setup_failed: {exc}"

    def update_joint(self, joint_rad: np.ndarray) -> None:
        with self.lock:
            self.latest_joint_rad = np.asarray(joint_rad, dtype=np.float64).reshape(6)

    def _compute_payload_gravity(self) -> np.ndarray:
        if not self.payload_enabled:
            return np.zeros(6, dtype=np.float64)
        if (
            self.model is None
            or self.data is None
            or self.q_full is None
            or self.ft_frame_id is None
            or not self.payload_link_data
        ):
            raise RuntimeError(
                "payload gravity model is enabled but incompletely initialized"
            )

        with self.lock:
            joint = None if self.latest_joint_rad is None else self.latest_joint_rad.copy()
        if joint is None:
            raise RuntimeError(
                "payload gravity calculation is waiting for the first valid "
                "arm joint sample"
            )

        try:
            _pin = load_pinocchio()
            q = self.q_full.copy()
            for i, idx_q in enumerate(self.idxq):
                q[idx_q] = joint[i]
            _pin.forwardKinematics(self.model, self.data, q)
            _pin.updateFramePlacements(self.model, self.data)

            sensor_tf = self.data.oMf[int(self.ft_frame_id)]
            p_sensor_world = np.asarray(sensor_tf.translation, dtype=np.float64).reshape(3)
            r_world_sensor = np.asarray(sensor_tf.rotation, dtype=np.float64).reshape(3, 3)
            gravity_world = np.asarray(
                self.model.gravity.linear, dtype=np.float64
            ).reshape(3)
            force_world_total = np.zeros(3, dtype=np.float64)
            moment_world_total = np.zeros(3, dtype=np.float64)

            for _link_name, frame_id, mass, com_local in self.payload_link_data:
                link_tf = self.data.oMf[int(frame_id)]
                p_com_world = (
                    np.asarray(link_tf.translation, dtype=np.float64).reshape(3)
                    + np.asarray(link_tf.rotation, dtype=np.float64).reshape(3, 3)
                    @ np.asarray(com_local, dtype=np.float64).reshape(3)
                )
                force_world = float(mass) * gravity_world
                moment_world = np.cross(p_com_world - p_sensor_world, force_world)
                force_world_total += force_world
                moment_world_total += moment_world

            force_sensor = r_world_sensor.T @ force_world_total
            moment_sensor = r_world_sensor.T @ moment_world_total
            payload_gravity = self.payload_gravity_sign * np.concatenate(
                [force_sensor, moment_sensor]
            )
            with self.lock:
                self.latest_payload_gravity = payload_gravity.copy()
            return payload_gravity
        except Exception as exc:
            with self.lock:
                self.payload_status = f"disabled_runtime_failed: {exc}"
            raise RuntimeError(
                f"payload gravity calculation failed: {exc}"
            ) from exc

    def process(
        self, timestamp: float, wrench_raw: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        del timestamp
        raw = np.asarray(wrench_raw, dtype=np.float64).reshape(6)
        payload_gravity = self._compute_payload_gravity()
        wrench_comp_payload = raw - payload_gravity
        return raw, payload_gravity, wrench_comp_payload

    def metadata(self) -> Dict[str, Any]:
        with self.lock:
            return {
                "formula": (
                    "wrench_comp_payload = "
                    "wrench_raw - wrench_payload_gravity"
                ),
                "payload_gravity_comp_enabled": bool(self.payload_enabled),
                "payload_status": self.payload_status,
                "payload_root": self.args.ft_payload_root,
                "payload_gravity_sign": self.payload_gravity_sign.copy(),
                "ft_frame": self.args.ft_frame,
                "ft_sensor_origin_approximation": self.args.ft_frame,
            }


class RecordingController:
    def __init__(
        self,
        session_dir: Path,
        args: argparse.Namespace,
        ft_processor: Optional[FTProcessor] = None,
    ):
        self.session_dir = session_dir
        self.args = args
        self.ft_processor = ft_processor
        self.lock = threading.RLock()
        self.writer: Optional[EpisodeWriter] = None
        self.pending_writer: Optional[EpisodeWriter] = None
        self.draining_writer: Optional[EpisodeWriter] = None
        self.starting = False
        self._starting_writer: Optional[EpisodeWriter] = None
        self._pending_live: List[
            Tuple[
                str,
                float,
                Callable[[EpisodeWriter], None],
            ]
        ] = []
        self.auto_stop_reason: Optional[str] = None
        # Per-modality ordering watermarks bridge the pre-roll/live boundary.
        # Producers append to their pre-roll buffer before calling write_*;
        # therefore a callback blocked by start_episode can otherwise enqueue
        # the exact row that the captured pre-roll just flushed.
        self._last_enqueued_order: Dict[str, float] = {}
        self._pre_roll_boundary_order: Dict[str, float] = {}

    def saved_episode_count(self) -> int:
        return sum(
            1
            for path in self.session_dir.iterdir()
            if path.is_dir()
            and re.fullmatch(r"episode_\d+", path.name) is not None
        )

    def episode_counts(self) -> Tuple[int, int]:
        """Return (clean, diagnostic) saved episode counts."""
        clean = 0
        diagnostic = 0
        for path in self.session_dir.iterdir():
            if not path.is_dir() or re.fullmatch(r"episode_\d+", path.name) is None:
                continue
            try:
                metadata = json.loads((path / "meta.json").read_text(encoding="utf-8"))
            except (OSError, ValueError, TypeError, json.JSONDecodeError):
                diagnostic += 1
                continue
            if str(metadata.get("interruption_reason") or "").strip():
                diagnostic += 1
            else:
                clean += 1
        return clean, diagnostic

    def next_episode_index(self) -> int:
        used = {
            int(path.name.removeprefix("episode_"))
            for path in self.session_dir.iterdir()
            if path.is_dir()
            and re.fullmatch(r"episode_\d+", path.name) is not None
        }
        idx = 0
        while idx in used:
            idx += 1
        return idx

    def is_recording(self) -> bool:
        with self.lock:
            return self.writer is not None

    def has_pending_episode(self) -> bool:
        with self.lock:
            return self.pending_writer is not None

    def is_draining(self) -> bool:
        with self.lock:
            return self.draining_writer is not None

    def get_pending_writer(self) -> Optional[EpisodeWriter]:
        with self.lock:
            return self.pending_writer

    def get_draining_writer(self) -> Optional[EpisodeWriter]:
        with self.lock:
            return self.draining_writer

    def get_writer(self) -> Optional[EpisodeWriter]:
        with self.lock:
            return self.writer

    def start_episode(
        self,
        snapshot_provider: Any,
    ) -> None:
        """Build the sink without blocking producers, then publish atomically.

        ``snapshot_provider`` may be a callable (the production path) or an
        already-captured mapping for backward-compatible tests/tools. Writer
        construction can create many zarr arrays, so it runs outside the
        controller lock while producers continue filling pre-roll. Only the
        final snapshot/flush/publication barrier holds the lock.
        """
        with self.lock:
            if self.draining_writer is not None:
                raise RuntimeError(
                    "The previous episode writer is still draining queued data."
                )
            if self.writer is not None or self.starting:
                print("Already recording.")
                return
            if self.pending_writer is not None:
                raise RuntimeError(
                    "A stopped episode is pending. Save or discard it before starting."
                )
            episode_idx = self.next_episode_index()
            saved = self.saved_episode_count()
            print(
                f"Saved episodes: {saved}. Starting episode_{episode_idx:03d}."
            )
            temp_dir = self.session_dir / f".episode_{episode_idx:03d}_recording"
            final_dir = self.session_dir / f"episode_{episode_idx:03d}"
            if temp_dir.exists():
                raise RuntimeError(
                    f"Unfinished temporary episode exists at {temp_dir}. "
                    "Inspect, move, or explicitly remove it before recording; "
                    "the recorder will not destroy crash-recovery data."
                )
            if self.ft_processor is not None:
                self.args.ft_processing_metadata = self.ft_processor.metadata()
            self.starting = True
        try:
            writer = EpisodeWriter(temp_dir, final_dir, self.args)
        except BaseException:
            with self.lock:
                self.starting = False
            raise
        try:
            with self.lock:
                self._last_enqueued_order = {}
                self._pre_roll_boundary_order = {}
                snapshots = (
                    snapshot_provider()
                    if callable(snapshot_provider)
                    else snapshot_provider
                )
                if not isinstance(snapshots, dict):
                    raise TypeError(
                        "snapshot provider must return a modality mapping"
                    )
                # Rows whose producer-side pre-roll append raced with this
                # snapshot are captured as pending live callbacks below. The
                # snapshot and state transition share one short lock barrier.
                self._starting_writer = writer
                self._pending_live = []
                self.auto_stop_reason = None
            # Potentially large pre-roll flush runs without the controller
            # lock. Producers continue acquiring and append compact callbacks
            # to _pending_live rather than blocking their sensor threads.
            self._flush_pre_roll(writer, snapshots)
            with self.lock:
                for modality, order_value, enqueue in self._pending_live:
                    self._enqueue_to_writer_locked(
                        writer,
                        modality,
                        order_value,
                        enqueue,
                    )
                self._pending_live = []
                self._starting_writer = None
                writer.start_wall_time = now_s()
                self.writer = writer
                self.starting = False
        except BaseException as exc:
            with self.lock:
                self.starting = False
                self._starting_writer = None
                self._pending_live = []
            writer._set_write_error(f"pre-roll flush failed: {exc}")
            writer.close()
            raise
        print("Recording started.")

    @staticmethod
    def _ordering_relation(
        order_value: float,
        previous: Optional[float],
        tolerance: float = 1e-9,
    ) -> str:
        if previous is None:
            return "new"
        delta = float(order_value) - float(previous)
        if delta > tolerance:
            return "new"
        if abs(delta) <= tolerance:
            return "equal"
        return "backward"

    def _enqueue_pre_roll(
        self,
        writer: EpisodeWriter,
        modality: str,
        order_value: float,
        enqueue: Callable[[], None],
    ) -> bool:
        previous = self._last_enqueued_order.get(modality)
        relation = self._ordering_relation(order_value, previous)
        if relation != "new":
            raise ValueError(
                f"pre-roll {modality} ordering is {relation}: "
                f"timestamp={float(order_value):.9f}, "
                f"previous={float(previous):.9f}"
            )
        enqueue()
        self._last_enqueued_order[modality] = float(order_value)
        self._pre_roll_boundary_order[modality] = float(order_value)
        return True

    def _enqueue_live(
        self,
        modality: str,
        order_value: float,
        enqueue: Callable[[EpisodeWriter], None],
        pending_enqueue_factory: Optional[
            Callable[[], Callable[[EpisodeWriter], None]]
        ] = None,
    ) -> bool:
        # Keep the lock through EpisodeWriter's nonblocking queue enqueue.  A
        # concurrent stop cannot close this writer between pointer lookup and
        # write, eliminating the benign "write requested after close" race.
        with self.lock:
            if self.starting and self._starting_writer is not None:
                pending_enqueue = (
                    pending_enqueue_factory()
                    if pending_enqueue_factory is not None
                    else enqueue
                )
                self._pending_live.append(
                    (modality, float(order_value), pending_enqueue)
                )
                return True
            writer = self.writer
            if writer is None:
                return False
            return self._enqueue_to_writer_locked(
                writer, modality, order_value, enqueue
            )

    def _enqueue_to_writer_locked(
        self,
        writer: EpisodeWriter,
        modality: str,
        order_value: float,
        enqueue: Callable[[EpisodeWriter], None],
    ) -> bool:
        """Enqueue one ordered row while ``self.lock`` is held."""
        previous = self._last_enqueued_order.get(modality)
        relation = self._ordering_relation(order_value, previous)
        boundary = self._pre_roll_boundary_order.get(modality)
        is_boundary_duplicate = (
            relation == "equal"
            and boundary is not None
            and self._ordering_relation(order_value, boundary) == "equal"
        )
        if is_boundary_duplicate:
            # Exactly one producer callback may have appended this row to
            # pre-roll and then crossed the start barrier. Its pending/live
            # call is the only non-increasing event silently deduped.
            self._pre_roll_boundary_order.pop(modality, None)
            return False
        self._pre_roll_boundary_order.pop(modality, None)
        if relation != "new":
            message = (
                f"{modality} live timestamp moved {relation}: "
                f"timestamp={float(order_value):.9f}, "
                f"previous={float(previous):.9f}; recording stopped "
                "to preserve strict timestamp ordering"
            )
            writer._set_write_error(message)
            if self.auto_stop_reason is None:
                self.auto_stop_reason = message
            return False
        enqueue(writer)
        self._last_enqueued_order[modality] = float(order_value)
        return True

    def _flush_pre_roll(self, writer: EpisodeWriter, snapshots: Dict[str, Any]) -> None:
        for cam_id, _model, _serial in configured_camera_specs(self.args):
            cam_key = f"camera_{cam_id}"
            for sample in snapshots[cam_key]["rgb"]:
                timestamp, rgb = sample[:2]
                hw_ts = sample[2] if len(sample) > 2 else math.nan
                frame_number = sample[3] if len(sample) > 3 else -1
                self._enqueue_pre_roll(
                    writer,
                    f"camera_{cam_id}_rgb",
                    timestamp,
                    lambda: writer.write_camera_rgb(
                        cam_id, timestamp, rgb, hw_ts, frame_number
                    ),
                )
            for sample in snapshots[cam_key]["depth"]:
                timestamp, depth = sample[:2]
                hw_ts = sample[2] if len(sample) > 2 else math.nan
                frame_number = sample[3] if len(sample) > 3 else -1
                self._enqueue_pre_roll(
                    writer,
                    f"camera_{cam_id}_depth",
                    timestamp,
                    lambda: writer.write_camera_depth(
                        cam_id, timestamp, depth, hw_ts, frame_number
                    ),
                )

        for timestamp, joint_deg, ee_pose in snapshots["robot"]:
            self._enqueue_pre_roll(
                writer,
                "robot",
                timestamp,
                lambda: writer.write_robot(timestamp, joint_deg, ee_pose),
            )
            if not self.args.record_cmd_pose:
                if ee_pose is None:
                    raise RuntimeError(
                        "record_cmd_pose=false requires an FK pose in pre-roll"
                    )
                self._enqueue_pre_roll(
                    writer,
                    "command",
                    timestamp,
                    lambda: writer.write_command(timestamp, ee_pose),
                )
        for timestamp, hand_joint in snapshots.get("hand", []):
            self._enqueue_pre_roll(
                writer,
                "hand",
                timestamp,
                lambda: writer.write_hand_joint(timestamp, hand_joint),
            )
        for timestamp, joint_torque in snapshots["jt"]:
            self._enqueue_pre_roll(
                writer,
                "jt",
                timestamp,
                lambda: writer.write_jt(timestamp, joint_torque),
            )
        for timestamp, contact_state in snapshots.get("contact_state", []):
            self._enqueue_pre_roll(
                writer,
                "contact_state",
                timestamp,
                lambda: writer.write_contact_state(
                    timestamp, contact_state
                ),
            )
        for timestamp, contact_phase in snapshots.get("contact_phase", []):
            self._enqueue_pre_roll(
                writer,
                "contact_phase",
                timestamp,
                lambda: writer.write_contact_phase(
                    timestamp, contact_phase
                ),
            )
        for sample in snapshots.get("contact_observation", []):
            # receive timestamp is strictly increasing even for the one
            # permitted equal-source policy-readiness-loss transition.
            self._enqueue_pre_roll(
                writer,
                "contact_observation",
                float(sample[1]),
                lambda: writer.write_contact_observation(*sample),
            )
        for sample in snapshots["ft"]:
            if len(sample) >= 4:
                timestamp, raw, payload_gravity, _wrench_comp_payload = sample[:4]
                wrench_comp_payload = (
                    np.asarray(raw, dtype=np.float64).reshape(6)
                    - np.asarray(payload_gravity, dtype=np.float64).reshape(6)
                )
                self._enqueue_pre_roll(
                    writer,
                    "ft",
                    timestamp,
                    lambda: writer.write_ft(
                        timestamp,
                        raw,
                        payload_gravity,
                        wrench_comp_payload,
                    ),
                )
            else:
                timestamp, wrench = sample[:2]
                zeros = np.zeros(6, dtype=np.float64)
                wrench_comp_payload = np.asarray(wrench, dtype=np.float64).reshape(6)
                self._enqueue_pre_roll(
                    writer,
                    "ft",
                    timestamp,
                    lambda: writer.write_ft(
                        timestamp, wrench, zeros, wrench_comp_payload
                    ),
                )
        for timestamp, jt_tared_wrench in snapshots["jt_tared_wrench"]:
            self._enqueue_pre_roll(
                writer,
                "jt_tared_wrench",
                timestamp,
                lambda: writer.write_jt_tared_wrench(
                    timestamp, jt_tared_wrench
                ),
            )
        for timestamp, jt_tared_filtered_wrench in snapshots["jt_tared_filtered_wrench"]:
            self._enqueue_pre_roll(
                writer,
                "jt_tared_filtered_wrench",
                timestamp,
                lambda: writer.write_jt_tared_filtered_wrench(
                    timestamp, jt_tared_filtered_wrench
                ),
            )
        for timestamp, wrench_base in snapshots.get("ft_base", []):
            self._enqueue_pre_roll(
                writer,
                "ft_base",
                timestamp,
                lambda: writer.write_ft_base(timestamp, wrench_base),
            )
        if self.args.record_cmd_pose:
            for timestamp, command_pose in snapshots["command"]:
                self._enqueue_pre_roll(
                    writer,
                    "command",
                    timestamp,
                    lambda: writer.write_command(timestamp, command_pose),
                )
        if getattr(self.args, "record_current_pose", False):
            for timestamp, current_pose in snapshots.get("current_pose", []):
                self._enqueue_pre_roll(
                    writer,
                    "current_pose",
                    timestamp,
                    lambda: writer.write_current_pose(
                        timestamp, current_pose
                    ),
                )
        if getattr(self.args, "record_cmd_quat_pose", False):
            for timestamp, command_pose in snapshots.get("command_quat", []):
                self._enqueue_pre_roll(
                    writer,
                    "command_quat",
                    timestamp,
                    lambda: writer.write_command_quat(
                        timestamp, command_pose
                    ),
                )

    def stop_episode(
        self,
        interruption_reason: Optional[str] = None,
    ) -> Optional[EpisodeWriter]:
        with self.lock:
            writer = self.writer
            self.writer = None
            effective_interruption_reason = (
                interruption_reason or self.auto_stop_reason
            )
            self.auto_stop_reason = None
            if writer is not None:
                self.draining_writer = writer
        if writer is not None:
            if effective_interruption_reason:
                writer.set_interruption_reason(
                    effective_interruption_reason
                )
            try:
                writer.close()
            finally:
                with self.lock:
                    if self.draining_writer is writer:
                        self.draining_writer = None
        return writer

    def stop_to_pending(
        self,
        interruption_reason: Optional[str] = None,
    ) -> Optional[EpisodeWriter]:
        with self.lock:
            if self.pending_writer is not None:
                raise RuntimeError("another stopped episode is already pending")
            writer = self.writer
            if writer is None:
                return None
            self.writer = None
            effective_interruption_reason = (
                interruption_reason or self.auto_stop_reason
            )
            self.auto_stop_reason = None
            self.pending_writer = writer
            self.draining_writer = writer
        if effective_interruption_reason:
            writer.set_interruption_reason(effective_interruption_reason)
        try:
            writer.close()
        finally:
            with self.lock:
                if self.draining_writer is writer:
                    self.draining_writer = None
        return writer

    def finalize_pending(self, save: bool) -> Optional[str]:
        with self.lock:
            writer = self.pending_writer
            if writer is None:
                return None
            if self.draining_writer is writer:
                raise RuntimeError(
                    "The episode writer is still draining queued data; wait "
                    "for DRAINING to finish before saving or discarding."
                )
        self.finalize_episode(writer, save)
        with self.lock:
            if self.pending_writer is writer:
                self.pending_writer = None
        return writer.final_dir.name

    def finalize_episode(self, writer: EpisodeWriter, save: bool) -> None:
        counts = writer.sample_counts()
        representative_steps = counts.get(
            "robot_joint",
            counts.get("robot_ee_pose_fk", counts.get("robot_ee_pose", 0)),
        )
        episode_name = writer.final_dir.name
        if save:
            if writer.write_error:
                raise RuntimeError(
                    f"Refusing to save {episode_name} as a valid episode because "
                    f"the disk writer failed: {writer.write_error}. The temporary "
                    f"data remains at {writer.temp_dir} for inspection."
                )
            if writer.final_dir.exists():
                raise RuntimeError(f"Final episode directory exists: {writer.final_dir}")
            writer.temp_dir.rename(writer.final_dir)
            print(
                f"Saved {episode_name}. Representative steps: "
                f"{representative_steps}."
            )
            if writer.interruption_reason:
                print(
                    "This auto-stopped episode was saved for diagnosis only; "
                    "the raw checker will reject it for train-data conversion."
                )
            print("Sample counts:", counts)
        else:
            shutil.rmtree(writer.temp_dir, ignore_errors=True)
            print(f"Discarded {episode_name}. Next episode index is unchanged.")

    def request_auto_stop(self, reason: str) -> None:
        with self.lock:
            if self.writer is not None and self.auto_stop_reason is None:
                self.auto_stop_reason = reason

    def consume_auto_stop_reason(self) -> Optional[str]:
        with self.lock:
            reason = self.auto_stop_reason
            self.auto_stop_reason = None
        return reason

    def write_camera_rgb(
        self,
        camera_id: int,
        timestamp: float,
        rgb: np.ndarray,
        hardware_timestamp_ms: float = math.nan,
        frame_number: int = -1,
    ):
        self._enqueue_live(
            f"camera_{camera_id}_rgb",
            timestamp,
            lambda writer: writer.write_camera_rgb(
                camera_id,
                timestamp,
                rgb,
                hardware_timestamp_ms,
                frame_number,
            ),
            # RealSenseCamera publishes a fresh, read-only ndarray per frame.
            # Keep the same immutable reference across pre-roll, pending-live,
            # and the writer queue instead of briefly doubling image RAM.
            lambda: (
                lambda writer, pending_rgb=np.asarray(rgb): (
                    writer.write_camera_rgb(
                        camera_id,
                        timestamp,
                        pending_rgb,
                        hardware_timestamp_ms,
                        frame_number,
                    )
                )
            ),
        )

    def write_camera_depth(
        self,
        camera_id: int,
        timestamp: float,
        depth: np.ndarray,
        hardware_timestamp_ms: float = math.nan,
        frame_number: int = -1,
    ):
        self._enqueue_live(
            f"camera_{camera_id}_depth",
            timestamp,
            lambda writer: writer.write_camera_depth(
                camera_id,
                timestamp,
                depth,
                hardware_timestamp_ms,
                frame_number,
            ),
            lambda: (
                lambda writer, pending_depth=np.asarray(depth): (
                    writer.write_camera_depth(
                        camera_id,
                        timestamp,
                        pending_depth,
                        hardware_timestamp_ms,
                        frame_number,
                    )
                )
            ),
        )

    def write_robot(
        self,
        timestamp: float,
        joint_deg: np.ndarray,
        ee_pose: Optional[np.ndarray],
    ):
        self._enqueue_live(
            "robot",
            timestamp,
            lambda writer: writer.write_robot(
                timestamp, joint_deg, ee_pose
            ),
            lambda: (
                lambda writer,
                pending_joint=np.asarray(joint_deg).copy(),
                pending_pose=(
                    None if ee_pose is None else np.asarray(ee_pose).copy()
                ): writer.write_robot(
                    timestamp, pending_joint, pending_pose
                )
            ),
        )

    def write_hand_joint(self, timestamp: float, hand_joint: np.ndarray):
        self._enqueue_live(
            "hand",
            timestamp,
            lambda writer: writer.write_hand_joint(timestamp, hand_joint),
            lambda: (
                lambda writer,
                pending_hand=np.asarray(hand_joint).copy(): (
                    writer.write_hand_joint(timestamp, pending_hand)
                )
            ),
        )

    def write_jt(self, timestamp: float, joint_torque: np.ndarray):
        self._enqueue_live(
            "jt",
            timestamp,
            lambda writer: writer.write_jt(timestamp, joint_torque),
            lambda: (
                lambda writer,
                pending_torque=np.asarray(joint_torque).copy(): (
                    writer.write_jt(timestamp, pending_torque)
                )
            ),
        )

    def write_ft(
        self,
        timestamp: float,
        wrench_raw: np.ndarray,
        wrench_payload_gravity: np.ndarray,
        wrench_comp_payload: np.ndarray,
    ):
        self._enqueue_live(
            "ft",
            timestamp,
            lambda writer: writer.write_ft(
                timestamp,
                wrench_raw,
                wrench_payload_gravity,
                wrench_comp_payload,
            ),
            lambda: (
                lambda writer,
                pending_raw=np.asarray(wrench_raw).copy(),
                pending_gravity=np.asarray(
                    wrench_payload_gravity
                ).copy(),
                pending_comp=np.asarray(wrench_comp_payload).copy(): (
                    writer.write_ft(
                        timestamp,
                        pending_raw,
                        pending_gravity,
                        pending_comp,
                    )
                )
            ),
        )

    def write_ft_base(self, timestamp: float, wrench_base: np.ndarray):
        self._enqueue_live(
            "ft_base",
            timestamp,
            lambda writer: writer.write_ft_base(timestamp, wrench_base),
            lambda: (
                lambda writer,
                pending_wrench=np.asarray(wrench_base).copy(): (
                    writer.write_ft_base(timestamp, pending_wrench)
                )
            ),
        )

    def write_jt_tared_wrench(
        self, timestamp: float, jt_tared_wrench: np.ndarray
    ):
        self._enqueue_live(
            "jt_tared_wrench",
            timestamp,
            lambda writer: writer.write_jt_tared_wrench(
                timestamp, jt_tared_wrench
            ),
            lambda: (
                lambda writer,
                pending_wrench=np.asarray(jt_tared_wrench).copy(): (
                    writer.write_jt_tared_wrench(
                        timestamp, pending_wrench
                    )
                )
            ),
        )

    def write_jt_tared_filtered_wrench(
        self, timestamp: float, jt_tared_filtered_wrench: np.ndarray
    ):
        self._enqueue_live(
            "jt_tared_filtered_wrench",
            timestamp,
            lambda writer: writer.write_jt_tared_filtered_wrench(
                timestamp, jt_tared_filtered_wrench
            ),
            lambda: (
                lambda writer,
                pending_wrench=np.asarray(
                    jt_tared_filtered_wrench
                ).copy(): writer.write_jt_tared_filtered_wrench(
                    timestamp, pending_wrench
                )
            ),
        )

    def write_command(self, timestamp: float, command_pose: np.ndarray):
        self._enqueue_live(
            "command",
            timestamp,
            lambda writer: writer.write_command(timestamp, command_pose),
            lambda: (
                lambda writer,
                pending_pose=np.asarray(command_pose).copy(): (
                    writer.write_command(timestamp, pending_pose)
                )
            ),
        )

    def write_current_pose(self, timestamp: float, current_pose: np.ndarray):
        self._enqueue_live(
            "current_pose",
            timestamp,
            lambda writer: writer.write_current_pose(
                timestamp, current_pose
            ),
            lambda: (
                lambda writer,
                pending_pose=np.asarray(current_pose).copy(): (
                    writer.write_current_pose(timestamp, pending_pose)
                )
            ),
        )

    def write_command_quat(self, timestamp: float, command_pose: np.ndarray):
        self._enqueue_live(
            "command_quat",
            timestamp,
            lambda writer: writer.write_command_quat(
                timestamp, command_pose
            ),
            lambda: (
                lambda writer,
                pending_pose=np.asarray(command_pose).copy(): (
                    writer.write_command_quat(timestamp, pending_pose)
                )
            ),
        )

    def write_contact_state(self, timestamp: float, contact_state: int):
        self._enqueue_live(
            "contact_state",
            timestamp,
            lambda writer: writer.write_contact_state(
                timestamp, contact_state
            ),
        )

    def write_contact_phase(self, timestamp: float, contact_phase: int):
        self._enqueue_live(
            "contact_phase",
            timestamp,
            lambda writer: writer.write_contact_phase(
                timestamp, contact_phase
            ),
        )

    def write_contact_observation(
        self,
        source_timestamp: float,
        receive_timestamp: float,
        source_sequence: int,
        contact_wrench: np.ndarray,
        contact_state: int,
        valid: bool,
        model_ready: bool,
        prediction_age_ms: float,
        free_space_wrench_prediction: np.ndarray,
    ):
        self._enqueue_live(
            "contact_observation",
            receive_timestamp,
            lambda writer: writer.write_contact_observation(
                source_timestamp,
                receive_timestamp,
                source_sequence,
                contact_wrench,
                contact_state,
                valid,
                model_ready,
                prediction_age_ms,
                free_space_wrench_prediction,
            ),
            lambda: (
                lambda writer,
                pending_wrench=np.asarray(contact_wrench).copy(),
                pending_prediction=np.asarray(
                    free_space_wrench_prediction
                ).copy(): (
                    writer.write_contact_observation(
                        source_timestamp,
                        receive_timestamp,
                        source_sequence,
                        pending_wrench,
                        contact_state,
                        valid,
                        model_ready,
                        prediction_age_ms,
                        pending_prediction,
                    )
                )
            ),
        )


class RealSenseCamera:
    def __init__(
        self,
        camera_id: int,
        model: str,
        serial: str,
        args: argparse.Namespace,
        controller: RecordingController,
    ):
        self.camera_id = camera_id
        self.model = model
        self.serial = serial
        self.args = args
        self.controller = controller
        self.pipeline = None
        self.align = None
        self.thread = None
        self.stop_event = threading.Event()
        self.lock = threading.RLock()
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_rgb_ts = None
        self.latest_depth_ts = None
        self.latest_error = None
        maxlen = max(2, int(math.ceil(args.pre_roll_sec * args.camera_fps)) + 2)
        self.rgb_pre: Deque[Tuple[float, np.ndarray, float, int]] = deque(maxlen=maxlen)
        self.depth_pre: Deque[Tuple[float, np.ndarray, float, int]] = deque(maxlen=maxlen)
        self.depth_scale = None
        self.device_name = None
        self.color_intrinsics: Dict[str, Any] = {}
        self.depth_intrinsics: Dict[str, Any] = {}
        self.depth_to_color_extrinsics: Dict[str, Any] = {}
        self.color_to_depth_extrinsics: Dict[str, Any] = {}

    def start(self) -> None:
        _rs = load_realsense()
        self.stop_event = threading.Event()
        self.pipeline = _rs.pipeline()
        cfg = _rs.config()
        cfg.enable_device(self.serial)
        cfg.enable_stream(
            _rs.stream.color,
            self.args.color_width,
            self.args.color_height,
            _rs.format.rgb8,
            self.args.camera_fps,
        )
        cfg.enable_stream(
            _rs.stream.depth,
            self.args.depth_width,
            self.args.depth_height,
            _rs.format.z16,
            self.args.camera_fps,
        )
        profile = self.pipeline.start(cfg)
        device = profile.get_device()
        self.device_name = device.get_info(_rs.camera_info.name)
        if self.model not in self.device_name:
            self.pipeline.stop()
            raise RuntimeError(
                f"Serial {self.serial} resolved to '{self.device_name}', "
                f"expected {self.model}."
            )
        depth_sensor = device.first_depth_sensor()
        self.depth_scale = float(depth_sensor.get_depth_scale())
        color_profile = profile.get_stream(_rs.stream.color).as_video_stream_profile()
        depth_profile = profile.get_stream(_rs.stream.depth).as_video_stream_profile()
        self.color_intrinsics = realsense_intrinsics_to_dict(
            color_profile.get_intrinsics()
        )
        self.depth_intrinsics = realsense_intrinsics_to_dict(
            depth_profile.get_intrinsics()
        )
        self.depth_to_color_extrinsics = realsense_extrinsics_to_dict(
            depth_profile.get_extrinsics_to(color_profile)
        )
        self.color_to_depth_extrinsics = realsense_extrinsics_to_dict(
            color_profile.get_extrinsics_to(depth_profile)
        )
        self.align = (
            _rs.align(_rs.stream.color)
            if bool(getattr(self.args, "align_depth_to_color", False))
            else None
        )
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self) -> None:
        while not self.stop_event.is_set():
            try:
                frames = self.pipeline.wait_for_frames(1000)
                if self.align is not None:
                    frames = self.align.process(frames)
                color = frames.get_color_frame()
                depth = frames.get_depth_frame()
                t_sample = now_s()
                if color:
                    rgb = np.asanyarray(color.get_data()).copy()
                    rgb.setflags(write=False)
                    rgb_hw_ts = float(color.get_timestamp())
                    rgb_frame_number = int(color.get_frame_number())
                    with self.lock:
                        self.latest_rgb = rgb
                        self.latest_rgb_ts = t_sample
                        self.rgb_pre.append(
                            (t_sample, rgb, rgb_hw_ts, rgb_frame_number)
                        )
                    self.controller.write_camera_rgb(
                        self.camera_id, t_sample, rgb, rgb_hw_ts, rgb_frame_number
                    )
                if depth:
                    depth_arr = np.asanyarray(depth.get_data()).copy()
                    depth_arr.setflags(write=False)
                    if bool(getattr(self.args, "align_depth_to_color", False)):
                        expected_shape = (
                            int(self.args.color_height),
                            int(self.args.color_width),
                        )
                        if tuple(depth_arr.shape) != expected_shape:
                            raise RuntimeError(
                                "aligned depth frame shape mismatch: "
                                f"got {tuple(depth_arr.shape)}, "
                                f"expected color grid {expected_shape}"
                            )
                    depth_hw_ts = float(depth.get_timestamp())
                    depth_frame_number = int(depth.get_frame_number())
                    with self.lock:
                        self.latest_depth = depth_arr
                        self.latest_depth_ts = t_sample
                        self.depth_pre.append(
                            (t_sample, depth_arr, depth_hw_ts, depth_frame_number)
                        )
                    self.controller.write_camera_depth(
                        self.camera_id,
                        t_sample,
                        depth_arr,
                        depth_hw_ts,
                        depth_frame_number,
                    )
                with self.lock:
                    self.latest_error = None
            except Exception as exc:
                with self.lock:
                    self.latest_error = str(exc)
                time.sleep(0.01)

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
            self.thread = None
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
            self.pipeline = None
        self.align = None

    def _hardware_reset_device(self, settle_sec: float) -> None:
        """Reset only this serial and wait for USB re-enumeration."""
        _rs = load_realsense()
        matches = []
        for device in _rs.context().query_devices():
            try:
                serial = device.get_info(_rs.camera_info.serial_number)
            except Exception:
                continue
            if str(serial) == str(self.serial):
                matches.append(device)
        if len(matches) != 1:
            raise RuntimeError(
                f"hardware reset expected one RealSense serial={self.serial}, "
                f"found {len(matches)}"
            )
        matches[0].hardware_reset()
        time.sleep(max(0.1, float(settle_sec)))

    def restart(
        self,
        *,
        hardware_reset: bool = False,
        hardware_reset_settle_sec: float = 6.0,
    ) -> None:
        self.stop()
        with self.lock:
            self.latest_rgb = None
            self.latest_depth = None
            self.latest_rgb_ts = None
            self.latest_depth_ts = None
            self.latest_error = (
                "camera hardware reset requested"
                if hardware_reset
                else "camera restart requested"
            )
            self.rgb_pre.clear()
            self.depth_pre.clear()
        if hardware_reset:
            self._hardware_reset_device(hardware_reset_settle_sec)
        self.start()

    def snapshot(self) -> Dict[str, List[Tuple[float, np.ndarray, float, int]]]:
        with self.lock:
            return {"rgb": list(self.rgb_pre), "depth": list(self.depth_pre)}

    def calibration_metadata(self) -> Dict[str, Any]:
        with self.lock:
            depth_aligned_to_color = bool(
                getattr(self.args, "align_depth_to_color", False)
            )
            stored_depth_intrinsics = (
                dict(self.color_intrinsics)
                if depth_aligned_to_color
                else dict(self.depth_intrinsics)
            )
            stored_depth_stream = {
                "width": int(
                    self.args.color_width
                    if depth_aligned_to_color
                    else self.args.depth_width
                ),
                "height": int(
                    self.args.color_height
                    if depth_aligned_to_color
                    else self.args.depth_height
                ),
                "fps": int(self.args.camera_fps),
                "format": "z16",
                "pixel_grid": "color" if depth_aligned_to_color else "depth",
            }
            return {
                "role": f"camera_{self.camera_id}",
                "model": self.model,
                "serial": self.serial,
                "device_name": self.device_name,
                "color_stream": {
                    "width": int(self.args.color_width),
                    "height": int(self.args.color_height),
                    "fps": int(self.args.camera_fps),
                    "format": "rgb8",
                },
                "depth_stream": {
                    "width": int(self.args.depth_width),
                    "height": int(self.args.depth_height),
                    "fps": int(self.args.camera_fps),
                    "format": "z16",
                },
                "depth_scale": self.depth_scale,
                "depth_stored_as": "uint16_depth_units",
                "depth_aligned_to_color": depth_aligned_to_color,
                "depth_alignment_method": (
                    "pyrealsense2.align(rs.stream.color)"
                    if depth_aligned_to_color
                    else None
                ),
                "stored_depth_stream": stored_depth_stream,
                "stored_depth_intrinsics": stored_depth_intrinsics,
                "color_intrinsics": dict(self.color_intrinsics),
                "depth_intrinsics": dict(self.depth_intrinsics),
                "depth_to_color_extrinsics": dict(self.depth_to_color_extrinsics),
                "color_to_depth_extrinsics": dict(self.color_to_depth_extrinsics),
            }

    def preview_frames(self):
        with self.lock:
            rgb = None if self.latest_rgb is None else self.latest_rgb.copy()
            depth = None if self.latest_depth is None else self.latest_depth.copy()
        return rgb, depth

    def readiness(self, window_sec: float, min_ratio: float, max_age: float):
        t_now = now_s()
        with self.lock:
            checks = []
            for label, latest_ts, buf, target in [
                (
                    f"camera_{self.camera_id}_{self.model}_rgb",
                    self.latest_rgb_ts,
                    self.rgb_pre,
                    self.args.camera_fps,
                ),
                (
                    f"camera_{self.camera_id}_{self.model}_depth",
                    self.latest_depth_ts,
                    self.depth_pre,
                    self.args.camera_fps,
                ),
            ]:
                timestamps = [x[0] for x in buf]
                hz = estimate_hz(timestamps, window_sec, t_now)
                age = None if latest_ts is None else t_now - latest_ts
                checks.append((label, target, hz, age, self.latest_error))
        return evaluate_ready_checks(checks, min_ratio, max_age)


class FKComputer:
    def __init__(self, urdf_path: Path, base_frame: str, ee_frame: str):
        _pin = load_pinocchio()
        if not urdf_path.exists():
            raise FileNotFoundError(f"Follower URDF not found: {urdf_path}")
        self.model = _pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()
        self.q_full = np.zeros(self.model.nq)
        self.joint_names = [f"right_joint_{i}" for i in range(1, 7)]
        self.idxq = self._joint_q_indices(self.joint_names)

        frame_names = {f.name for f in self.model.frames}
        if base_frame not in frame_names:
            raise RuntimeError(f"Base frame '{base_frame}' not found in URDF.")
        if ee_frame not in frame_names:
            raise RuntimeError(f"EE frame '{ee_frame}' not found in URDF.")
        self.base_frame_id = self.model.getFrameId(base_frame)
        self.ee_frame_id = self.model.getFrameId(ee_frame)

    def _joint_q_indices(self, names: List[str]) -> List[int]:
        idx = []
        for name in names:
            if not self.model.existJointName(name):
                raise RuntimeError(f"Joint '{name}' not found in follower URDF.")
            jid = self.model.getJointId(name)
            joint = self.model.joints[jid]
            if joint.nq != 1:
                raise RuntimeError(f"Joint '{name}' must have nq=1, got {joint.nq}.")
            idx.append(joint.idx_q)
        return idx

    def compute(self, joint_rad: np.ndarray) -> np.ndarray:
        _pin = load_pinocchio()
        q = self.q_full.copy()
        joint_rad = np.asarray(joint_rad, dtype=np.float64).reshape(6)
        for i, idx_q in enumerate(self.idxq):
            q[idx_q] = joint_rad[i]
        _pin.forwardKinematics(self.model, self.data, q)
        _pin.updateFramePlacements(self.model, self.data)
        base = self.data.oMf[int(self.base_frame_id)]
        ee = self.data.oMf[int(self.ee_frame_id)]
        base_to_ee = base.inverse() * ee
        return make_se3(
            np.asarray(base_to_ee.translation, dtype=np.float64),
            np.asarray(base_to_ee.rotation, dtype=np.float64),
        )


class RobotSampler:
    def __init__(
        self,
        ros_node,
        fk: Optional[FKComputer],
        args: argparse.Namespace,
        controller: RecordingController,
    ):
        self.ros_node = ros_node
        self.fk = fk
        self.args = args
        self.controller = controller
        self.stop_event = threading.Event()
        self.thread = None
        self.lock = threading.RLock()
        maxlen = max(2, int(math.ceil(args.pre_roll_sec * args.robot_sample_hz)) + 2)
        self.robot_pre: Deque[
            Tuple[float, np.ndarray, Optional[np.ndarray]]
        ] = deque(maxlen=maxlen)
        self.hand_pre: Deque[Tuple[float, np.ndarray]] = deque(maxlen=maxlen)
        self.jt_pre: Deque[Tuple[float, np.ndarray]] = deque(maxlen=maxlen)
        self.latest_joint_ts = None
        self.latest_hand_ts = None
        self.latest_ee_ts = None
        self.latest_jt_ts = None
        self.last_consumed_joint_rx_t: Optional[float] = None
        self.last_consumed_hand_rx_t: Optional[float] = None
        self.last_consumed_jt_rx_t: Optional[float] = None

    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)

    def _run(self):
        period = 1.0 / float(self.args.robot_sample_hz)
        # Scheduling uses a monotonic clock so NTP/Chrony corrections to the
        # shared wall-clock timestamps cannot stall or burst sampling.
        next_t = time.monotonic()
        while not self.stop_event.is_set():
            sample_t = now_s()
            joint, joint_rx_t, hand_joint, hand_rx_t, jt, jt_rx_t = (
                self.ros_node.latest_joint_hand_and_jt()
            )
            if (
                joint is not None
                and sample_t - joint_rx_t <= self.args.source_stale_sec
                and (
                    self.last_consumed_joint_rx_t is None
                    or joint_rx_t != self.last_consumed_joint_rx_t
                )
            ):
                # Mark the source receipt consumed before FK so a bad sample is
                # not retried at the target timer rate and misreported as new.
                self.last_consumed_joint_rx_t = float(joint_rx_t)
                try:
                    ee_pose = None if self.fk is None else self.fk.compute(joint)
                    if getattr(self.args, "record_ee_pose_fk", True) and ee_pose is None:
                        raise RuntimeError(
                            "record_ee_pose_fk=true but the FK model is unavailable"
                        )
                    joint_deg = np.rad2deg(joint)
                    with self.lock:
                        stored_pose = (
                            None if ee_pose is None else ee_pose.copy()
                        )
                        self.robot_pre.append(
                            (sample_t, joint_deg.copy(), stored_pose)
                        )
                        self.latest_joint_ts = sample_t
                        if getattr(self.args, "record_ee_pose_fk", True):
                            self.latest_ee_ts = sample_t
                    self.controller.write_robot(sample_t, joint_deg, ee_pose)
                    if not self.args.record_cmd_pose:
                        if ee_pose is None:
                            raise RuntimeError(
                                "record_cmd_pose=false requires FK for the command-pose fallback"
                            )
                        self.controller.write_command(sample_t, ee_pose)
                except Exception as exc:
                    self.ros_node.set_robot_error(str(exc))
            if (
                self.args.record_hand
                and hand_joint is not None
                and sample_t - hand_rx_t <= self.args.source_stale_sec
                and (
                    self.last_consumed_hand_rx_t is None
                    or hand_rx_t != self.last_consumed_hand_rx_t
                )
            ):
                self.last_consumed_hand_rx_t = float(hand_rx_t)
                with self.lock:
                    self.hand_pre.append((sample_t, hand_joint.copy()))
                    self.latest_hand_ts = sample_t
                self.controller.write_hand_joint(sample_t, hand_joint)
            if (
                self.args.record_jt
                and jt is not None
                and sample_t - jt_rx_t <= self.args.source_stale_sec
                and (
                    self.last_consumed_jt_rx_t is None
                    or jt_rx_t != self.last_consumed_jt_rx_t
                )
            ):
                self.last_consumed_jt_rx_t = float(jt_rx_t)
                with self.lock:
                    self.jt_pre.append((sample_t, jt.copy()))
                    self.latest_jt_ts = sample_t
                self.controller.write_jt(sample_t, jt)
            next_t += period
            monotonic_now = time.monotonic()
            if next_t <= monotonic_now:
                missed = (
                    math.floor((monotonic_now - next_t) / period) + 1
                )
                next_t += missed * period
            if self.stop_event.wait(max(0.0, next_t - monotonic_now)):
                break

    def snapshot(self) -> Dict[str, List[Any]]:
        with self.lock:
            return {
                "robot": list(self.robot_pre),
                "hand": list(self.hand_pre),
                "jt": list(self.jt_pre),
            }

    def readiness(self, window_sec: float, min_ratio: float, max_age: float):
        t_now = now_s()
        with self.lock:
            robot_ts = [x[0] for x in self.robot_pre]
            checks = [
                (
                    "robot_joint",
                    self.args.robot_sample_hz,
                    estimate_hz(robot_ts, window_sec, t_now),
                    None if self.latest_joint_ts is None else t_now - self.latest_joint_ts,
                    None,
                ),
            ]
            if getattr(self.args, "record_ee_pose_fk", True):
                checks.append(
                    (
                        "robot_ee_pose_fk",
                        self.args.robot_sample_hz,
                        estimate_hz(robot_ts, window_sec, t_now),
                        None
                        if self.latest_ee_ts is None
                        else t_now - self.latest_ee_ts,
                        None,
                    )
                )
            if self.args.record_hand:
                hand_ts = [x[0] for x in self.hand_pre]
                checks.append(
                    (
                        "robot_hand_joint",
                        self.args.robot_sample_hz,
                        estimate_hz(hand_ts, window_sec, t_now),
                        None
                        if self.latest_hand_ts is None
                        else t_now - self.latest_hand_ts,
                        None,
                    )
                )
            if self.args.record_jt:
                jt_ts = [x[0] for x in self.jt_pre]
                checks.append(
                    (
                        "jt_joint_torque",
                        self.args.robot_sample_hz,
                        estimate_hz(jt_ts, window_sec, t_now),
                        None
                        if self.latest_jt_ts is None
                        else t_now - self.latest_jt_ts,
                        None,
                    )
                )
        return evaluate_ready_checks(checks, min_ratio, max_age)


def make_ros_node_class(ros):
    Node = ros["Node"]
    ContactObservation = ros["ContactObservation"]
    ObserverInput = ros["ObserverInput"]
    JointState = ros["JointState"]
    PoseStamped = ros["PoseStamped"]
    WrenchStamped = ros["WrenchStamped"]
    Float64MultiArray = ros["Float64MultiArray"]
    Int32 = ros["Int32"]
    String = ros["String"]
    Trigger = ros["Trigger"]
    QoSProfile = ros["QoSProfile"]
    ReliabilityPolicy = ros["ReliabilityPolicy"]

    class ChemAcpROSNode(Node):
        def __init__(
            self,
            args: argparse.Namespace,
            controller: RecordingController,
            ft_processor: Optional[FTProcessor] = None,
        ):
            super().__init__("chem_acp_raw_data_collection")
            self.args = args
            self.controller = controller
            self.ft_processor = ft_processor
            self.lock = threading.RLock()
            self.joint_names = [f"right_joint_{i}" for i in range(1, 7)]
            self.generic_joint_names = [f"joint_{i}" for i in range(1, 7)]
            self.hand_joint_names = list(JOINT_NAMES_HAND_RIGHT)
            self.hand_joint_topic = str(
                getattr(args, "hand_joint_topic", "/joint_states")
            )
            self.hand_joint_shares_joint_topic = (
                self.hand_joint_topic == str(args.joint_topic)
            )
            self.joint_torque_topic = str(
                getattr(args, "joint_torque_topic", args.joint_topic)
            )
            self.joint_torque_shares_joint_topic = (
                self.joint_torque_topic == str(args.joint_topic)
            )
            self.latest_joint = None
            self.latest_joint_rx_t = 0.0
            self.latest_hand = None
            self.latest_hand_rx_t = 0.0
            self.latest_jt = None
            self.latest_jt_rx_t = 0.0
            self.latest_ft = None
            self.latest_ft_rx_t = 0.0
            self.latest_ft_base = None
            self.latest_ft_base_rx_t = 0.0
            self.latest_jt_tared_wrench = None
            self.latest_jt_tared_wrench_rx_t = 0.0
            self.latest_jt_tared_filtered_wrench = None
            self.latest_jt_tared_filtered_wrench_rx_t = 0.0
            self.latest_command = None
            self.latest_command_rx_t = 0.0
            self.latest_command_frame = None
            self.latest_current_pose = None
            self.latest_current_pose_rx_t = 0.0
            self.latest_command_quat = None
            self.latest_command_quat_rx_t = 0.0
            self.latest_command_quat_frame = None
            self.latest_contact_state = None
            self.latest_contact_state_rx_t = 0.0
            self.latest_contact_phase = None
            self.latest_contact_phase_rx_t = 0.0
            self.latest_contact_observation = None
            self.latest_contact_observation_source_t = 0.0
            self.latest_contact_observation_rx_t = 0.0
            self.latest_contact_observation_sequence: Optional[int] = None
            self.latest_contact_observation_status: Optional[
                Tuple[int, bool, bool]
            ] = None
            self.ft_processing_error: Optional[str] = None
            self.contact_observation_source_restarts = 0
            self.contact_observation_invalid_transitions = 0
            self.use_observer_input_robot_streams = bool(
                getattr(args, "use_observer_input_robot_streams", False)
            )
            self.robot_error = None
            self.executor_error = ""
            self.cameras: List[Any] = []
            self.robot_sampler: Optional[RobotSampler] = None
            self.shutdown_event: Optional[threading.Event] = None
            self.startup_ready = False
            self.camera_retry_counts: Dict[int, int] = {}
            self.camera_retry_errors: Dict[int, str] = {}
            self.latest_teleop_status: Dict[str, Any] = {}
            self.latest_teleop_status_rx_t = 0.0
            self.last_control_error = ""
            self.next_ft_record_t: Optional[float] = None
            self.next_ft_base_record_t: Optional[float] = None
            self.next_jt_tared_wrench_record_t: Optional[float] = None
            self.next_jt_tared_filtered_wrench_record_t: Optional[float] = None
            self.next_current_pose_record_t: Optional[float] = None
            self.next_command_quat_record_t: Optional[float] = None
            self.next_contact_state_record_t: Optional[float] = None
            self.next_contact_phase_record_t: Optional[float] = None
            self.next_contact_observation_record_t: Optional[float] = None
            self.next_command_record_t: Optional[float] = None
            max_ft = max(2, int(math.ceil(args.pre_roll_sec * args.ft_hz)) + 2)
            max_ft_base = max(2, int(math.ceil(args.pre_roll_sec * args.ft_base_hz)) + 2)
            max_jt_tared_wrench = max(
                2, int(math.ceil(args.pre_roll_sec * args.jt_tared_wrench_hz)) + 2
            )
            max_jt_tared_filtered_wrench = max(
                2, int(math.ceil(args.pre_roll_sec * args.jt_tared_filtered_wrench_hz)) + 2
            )
            max_cmd = max(2, int(math.ceil(args.pre_roll_sec * args.command_hz)) + 2)
            max_current_pose = max(
                2, int(math.ceil(args.pre_roll_sec * args.current_pose_hz)) + 2
            )
            max_cmd_quat = max(
                2, int(math.ceil(args.pre_roll_sec * args.command_quat_hz)) + 2
            )
            max_contact_state = max(
                2, int(math.ceil(args.pre_roll_sec * args.contact_state_hz)) + 2
            )
            max_contact_phase = max(
                2, int(math.ceil(args.pre_roll_sec * args.contact_phase_hz)) + 2
            )
            max_contact_observation = max(
                2,
                int(
                    math.ceil(args.pre_roll_sec * args.contact_observation_hz)
                )
                + 2,
            )
            self.ft_pre: Deque[
                Tuple[float, np.ndarray, np.ndarray, np.ndarray]
            ] = deque(maxlen=max_ft)
            self.ft_base_pre: Deque[Tuple[float, np.ndarray]] = deque(
                maxlen=max_ft_base
            )
            self.jt_tared_wrench_pre: Deque[Tuple[float, np.ndarray]] = deque(
                maxlen=max_jt_tared_wrench
            )
            self.jt_tared_filtered_wrench_pre: Deque[Tuple[float, np.ndarray]] = deque(
                maxlen=max_jt_tared_filtered_wrench
            )
            self.command_pre: Deque[Tuple[float, np.ndarray]] = deque(maxlen=max_cmd)
            self.current_pose_pre: Deque[Tuple[float, np.ndarray]] = deque(
                maxlen=max_current_pose
            )
            self.command_quat_pre: Deque[Tuple[float, np.ndarray]] = deque(
                maxlen=max_cmd_quat
            )
            self.contact_state_pre: Deque[Tuple[float, int]] = deque(
                maxlen=max_contact_state
            )
            self.contact_phase_pre: Deque[Tuple[float, int]] = deque(
                maxlen=max_contact_phase
            )
            self.contact_observation_pre: Deque[Tuple[Any, ...]] = deque(
                maxlen=max_contact_observation
            )

            self.create_subscription(
                JointState, args.joint_topic, self._joint_cb, 50
            )
            if args.record_hand and not self.hand_joint_shares_joint_topic:
                self.create_subscription(
                    JointState, self.hand_joint_topic, self._hand_joint_cb, 50
                )
            if args.record_jt and not self.joint_torque_shares_joint_topic:
                self.create_subscription(
                    JointState, self.joint_torque_topic, self._joint_torque_cb, 50
                )
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            if record_ft_wrench_enabled(args):
                self.create_subscription(WrenchStamped, args.ft_topic, self._ft_cb, qos)
            if args.record_ft_base:
                self.create_subscription(
                    WrenchStamped, args.ft_base_topic, self._ft_base_cb, qos
                )
            if args.record_jt_tared_wrench and not self.use_observer_input_robot_streams:
                self.create_subscription(
                    Float64MultiArray,
                    args.jt_tared_wrench_topic,
                    self._jt_tared_wrench_cb,
                    50,
                )
            if (
                args.record_jt_tared_filtered_wrench
                and not self.use_observer_input_robot_streams
            ):
                self.create_subscription(
                    Float64MultiArray,
                    args.jt_tared_filtered_wrench_topic,
                    self._jt_tared_filtered_wrench_cb,
                    50,
                )
            if args.record_cmd_pose and not self.use_observer_input_robot_streams:
                command_msg_type = str(args.command_msg_type)
                if command_msg_type == "auto":
                    if str(args.command_topic).endswith("/desired_pose"):
                        command_msg_type = "float64_multi_array"
                    else:
                        command_msg_type = "pose_stamped"
                command_cls = (
                    Float64MultiArray
                    if command_msg_type == "float64_multi_array"
                    else PoseStamped
                )
                self.create_subscription(
                    command_cls, args.command_topic, self._command_cb, 20
                )
            if args.record_current_pose and not self.use_observer_input_robot_streams:
                self.create_subscription(
                    Float64MultiArray,
                    args.current_pose_topic,
                    self._current_pose_cb,
                    20,
                )
            if args.record_cmd_quat_pose:
                command_quat_msg_type = str(args.command_quat_msg_type)
                if command_quat_msg_type not in (
                    "geometry_msgs/PoseStamped",
                    "geometry_msgs/msg/PoseStamped",
                    "pose_stamped",
                ):
                    self.robot_error = (
                        "command_quat_msg_type must be geometry_msgs/PoseStamped, "
                        f"got {command_quat_msg_type!r}"
                    )
                else:
                    self.create_subscription(
                        PoseStamped,
                        args.command_quat_topic,
                        self._command_quat_cb,
                        20,
                    )
            if args.record_contact_state:
                self.create_subscription(
                    Int32, args.contact_state_topic, self._contact_state_cb, 20
                )
            if args.record_contact_phase:
                self.create_subscription(
                    Int32, args.contact_phase_topic, self._contact_phase_cb, 20
                )
            if args.record_contact_observation:
                if ContactObservation is None:
                    raise RuntimeError(
                        "--record-contact-observation requires contact_observer_msgs; "
                        "build and source the shared interface package first"
                    )
                contact_qos = QoSProfile(
                    depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
                )
                self.create_subscription(
                    ContactObservation,
                    args.contact_observation_topic,
                    self._contact_observation_cb,
                    contact_qos,
                )
            if self.use_observer_input_robot_streams:
                if ObserverInput is None:
                    raise RuntimeError(
                        "--use-observer-input-robot-streams requires "
                        "contact_observer_msgs; build and source the interface package"
                    )
                observer_qos = QoSProfile(
                    depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
                )
                self.create_subscription(
                    ObserverInput,
                    args.observer_input_topic,
                    self._observer_input_cb,
                    observer_qos,
                )
            self.contact_observer_reset_client = None
            if bool(getattr(args, "reset_contact_observer_each_episode", False)):
                self.contact_observer_reset_client = self.create_client(
                    Trigger, args.contact_observer_reset_service)

            self.diagnostics_publisher = None
            self.ft_selected_publisher = None
            self.control_services = []
            if str(getattr(args, "control_mode", "terminal")) == "ros":
                self.diagnostics_publisher = self.create_publisher(
                    String, args.recorder_diagnostics_topic, 10)
                self.ft_selected_publisher = self.create_publisher(
                    WrenchStamped, args.recorder_ft_selected_topic, qos)
                self.create_subscription(
                    String,
                    args.teleop_status_topic,
                    self._teleop_status_cb,
                    10,
                )
                self.control_services = [
                    self.create_service(Trigger, "~/start_episode", self._start_service),
                    self.create_service(Trigger, "~/stop_save", self._stop_save_service),
                    self.create_service(
                        Trigger, "~/stop_discard", self._stop_discard_service),
                    self.create_service(Trigger, "~/recover", self._recover_service),
                    self.create_service(Trigger, "~/shutdown", self._shutdown_service),
                ]
                self.diagnostics_timer = self.create_timer(
                    float(args.diagnostics_period_sec),
                    self._publish_recorder_diagnostics,
                )

        def bind_runtime(
            self,
            cameras: List[Any],
            robot_sampler: Optional[RobotSampler],
            shutdown_event: threading.Event,
        ) -> None:
            self.cameras = list(cameras)
            self.robot_sampler = robot_sampler
            self.shutdown_event = shutdown_event

        def note_camera_retry(self, camera_id: int, error: str) -> None:
            with self.lock:
                self.camera_retry_counts[int(camera_id)] = (
                    self.camera_retry_counts.get(int(camera_id), 0) + 1
                )
                self.camera_retry_errors[int(camera_id)] = str(error)

        def note_camera_connected(self, camera_id: int) -> None:
            with self.lock:
                self.camera_retry_errors.pop(int(camera_id), None)

        def _teleop_status_cb(self, msg) -> None:
            try:
                payload = json.loads(str(msg.data))
                if not isinstance(payload, dict):
                    raise ValueError("teleop status must be a JSON object")
            except (TypeError, ValueError, json.JSONDecodeError) as exc:
                with self.lock:
                    self.last_control_error = f"invalid teleop status: {exc}"
                return
            with self.lock:
                self.latest_teleop_status = payload
                self.latest_teleop_status_rx_t = now_s()

        def _teleop_fast_ready(self) -> Tuple[bool, str]:
            if not bool(getattr(self.args, "require_teleop_fast", True)):
                return True, "teleop FAST gate disabled"
            with self.lock:
                status = dict(self.latest_teleop_status)
                receive_t = self.latest_teleop_status_rx_t
            if not status:
                return False, "leader teleop status has not been received"
            if now_s() - receive_t > max(0.5, float(self.args.ready_max_age_sec)):
                return False, "leader teleop status is stale"
            if str(status.get("state", "")).upper() != "FAST":
                return False, f"leader teleop state is {status.get('state', 'UNKNOWN')}, not FAST"
            if str(getattr(self.args, "model_sha256", "")):
                try:
                    actual_stage = float(status["feedback_gain_scale_contract"])
                except (KeyError, TypeError, ValueError):
                    return False, "leader teleop feedback stage is missing"
                expected_stage = float(self.args.feedback_gain_scale_contract)
                if not math.isclose(actual_stage, expected_stage, abs_tol=1.0e-9):
                    return False, (
                        "leader teleop feedback stage does not match recorder contract"
                    )
            return True, "leader teleop FAST"

        def _start_service(self, _request, response):
            try:
                if not self.startup_ready or self.robot_sampler is None:
                    raise RuntimeError("recorder sources are not READY")
                if self.controller.is_draining():
                    raise RuntimeError("the previous episode is still draining")
                if self.controller.is_recording():
                    raise RuntimeError("an episode is already recording")
                if self.controller.has_pending_episode():
                    raise RuntimeError("save or discard the pending episode first")
                memory = memory_status(self.args)
                if memory["level"] == "CRITICAL":
                    raise RuntimeError("; ".join(memory["critical_reasons"]))
                teleop_ok, teleop_message = self._teleop_fast_ready()
                if not teleop_ok:
                    raise RuntimeError(teleop_message)
                failures = readiness_failures(
                    self.cameras, self.robot_sampler, self, self.args)
                if failures:
                    raise RuntimeError("; ".join(failures))
                self.controller.start_episode(
                    lambda: collect_snapshots(
                        self.cameras, self.robot_sampler, self))
                response.success = True
                response.message = "episode recording started"
                self.last_control_error = ""
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                self.last_control_error = str(exc)
            return response

        def _finish_service(self, save: bool, response):
            try:
                if self.controller.is_draining():
                    raise RuntimeError(
                        "the episode writer is still draining; wait and retry"
                    )
                if self.controller.is_recording():
                    self.controller.stop_to_pending()
                name = self.controller.finalize_pending(save)
                if name is None:
                    raise RuntimeError("there is no active or pending episode")
                response.success = True
                response.message = (
                    f"saved {name}" if save else f"discarded {name}")
                self.last_control_error = ""
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                self.last_control_error = str(exc)
            return response

        def _stop_save_service(self, _request, response):
            return self._finish_service(True, response)

        def _stop_discard_service(self, _request, response):
            return self._finish_service(False, response)

        def _recover_service(self, _request, response):
            try:
                if not self.startup_ready or self.robot_sampler is None:
                    raise RuntimeError("recorder sources are not READY")
                if self.controller.is_recording():
                    raise RuntimeError("stop the active episode before recovery")
                if self.controller.is_draining():
                    raise RuntimeError("the episode writer is still draining; wait and retry")
                discarded = self.controller.finalize_pending(False)
                memory = memory_status(self.args)
                if memory["level"] == "CRITICAL":
                    raise RuntimeError("; ".join(memory["critical_reasons"]))
                rows = startup_status_rows(
                    self.cameras,
                    self.robot_sampler,
                    self,
                    self.args,
                    include_activation_gated=False,
                )
                failures = [
                    f"{row['name']}: {row['reason']}"
                    for row in rows
                    if not row["ok"]
                ]
                if failures:
                    raise RuntimeError("; ".join(failures))
                response.success = True
                response.message = (
                    f"discarded {discarded}; recorder sources READY"
                    if discarded is not None
                    else "recorder sources READY"
                )
                self.last_control_error = ""
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                self.last_control_error = str(exc)
            return response

        def _shutdown_service(self, _request, response):
            if (
                self.controller.is_recording()
                or self.controller.is_draining()
                or self.controller.has_pending_episode()
            ):
                response.success = False
                response.message = (
                    "resolve the active/draining/pending episode before shutdown"
                )
                return response
            response.success = True
            response.message = "recorder shutdown requested"
            if self.shutdown_event is not None:
                self.shutdown_event.set()
            return response

        def _publish_recorder_diagnostics(self) -> None:
            if self.diagnostics_publisher is None:
                return
            rows: List[Dict[str, Any]] = []
            if self.robot_sampler is not None and self.cameras:
                try:
                    with self.lock:
                        teleop_status = dict(self.latest_teleop_status)
                    rows = startup_status_rows(
                        self.cameras,
                        self.robot_sampler,
                        self,
                        self.args,
                        include_activation_gated=(
                            activation_gated_sources_expected(teleop_status)
                        ),
                    )
                except Exception as exc:
                    self.last_control_error = f"diagnostics readiness failed: {exc}"
            writer = self.controller.get_writer()
            draining = self.controller.get_draining_writer()
            pending = self.controller.get_pending_writer()
            queue_writer = writer or draining or pending
            queue_payload = {
                "items": 0,
                "capacity": int(self.args.writer_queue_size),
                "bytes": 0,
                "max_bytes": int(self.args.writer_queue_max_bytes),
                "warn_ratio": float(self.args.writer_queue_warn_ratio),
                "peak_items": 0,
                "peak_bytes": 0,
                "write_error": "",
            }
            if queue_writer is not None:
                with queue_writer.lock:
                    queue_payload.update({
                        "items": queue_writer.write_queue.qsize(),
                        "capacity": queue_writer.write_queue.maxsize,
                        "bytes": queue_writer.writer_queue_bytes,
                        "max_bytes": queue_writer.writer_queue_max_bytes,
                        "peak_items": queue_writer.writer_queue_peak,
                        "peak_bytes": queue_writer.writer_queue_peak_bytes,
                        "write_error": queue_writer.write_error or "",
                    })
            clean, diagnostic = self.controller.episode_counts()
            active_error = (
                queue_payload.get("write_error")
                or self.last_control_error
                or self.robot_error
                or ""
            )
            if draining is not None:
                state = "DRAINING ERROR" if active_error else "DRAINING"
            elif pending is not None:
                state = "PENDING ERROR" if active_error else "PENDING"
            elif active_error:
                state = "ERROR"
            elif writer is not None:
                state = "RECORDING"
            elif self.startup_ready:
                state = "READY"
            elif self.cameras:
                state = "CONNECTING"
            else:
                state = "WAITING OBSERVER"
            message = String()
            message.data = json.dumps({
                "schema_version": 1,
                "stamp": now_s(),
                "state": state,
                "startup_ready": bool(self.startup_ready),
                "recording": writer is not None,
                "draining": draining is not None,
                "pending": pending is not None,
                "episode": (
                    writer.final_dir.name if writer is not None else
                    draining.final_dir.name if draining is not None else
                    pending.final_dir.name if pending is not None else ""),
                "clean_saved_episodes": clean,
                "diagnostic_saved_episodes": diagnostic,
                "next_episode_index": self.controller.next_episode_index(),
                "modalities": rows,
                "camera_retry_counts": dict(self.camera_retry_counts),
                "camera_retry_errors": dict(self.camera_retry_errors),
                "queue": queue_payload,
                "memory": memory_status(self.args),
                "estimated_pre_roll_items": self.args.estimated_pre_roll_items,
                "estimated_pre_roll_payload_bytes": (
                    self.args.estimated_pre_roll_payload_bytes),
                "estimated_payload_peak_bytes": self.args.estimated_payload_peak_bytes,
                "last_error": active_error,
                "teleop": dict(self.latest_teleop_status),
            }, separators=(",", ":"), default=json_default)
            self.diagnostics_publisher.publish(message)

        @staticmethod
        def _deadline_crossed(
            sample_t: float,
            target_hz: float,
            next_deadline: Optional[float],
        ) -> Tuple[bool, Optional[float]]:
            """Select the first source sample crossing each rate deadline.

            This is timestamp based rather than count based, so 1000 Hz input is
            stored at an average 262.5 Hz without a 250 Hz every-fourth bias.
            """
            return deadline_crossing_select(sample_t, target_hz, next_deadline)

        def _should_record_command_sample(self, t_rx: float) -> bool:
            selected, self.next_command_record_t = self._deadline_crossed(
                t_rx, float(self.args.command_hz), self.next_command_record_t
            )
            return selected

        def _should_record_contact_observation_sample(self, source_t: float) -> bool:
            selected, self.next_contact_observation_record_t = self._deadline_crossed(
                source_t,
                float(self.args.contact_observation_hz),
                self.next_contact_observation_record_t,
            )
            return selected

        def _should_record_ft_sample(self, t_rx: float) -> bool:
            selected, self.next_ft_record_t = self._deadline_crossed(
                t_rx, float(self.args.ft_hz), self.next_ft_record_t
            )
            return selected

        def _should_record_ft_base_sample(self, t_rx: float) -> bool:
            selected, self.next_ft_base_record_t = self._deadline_crossed(
                t_rx, float(self.args.ft_base_hz), self.next_ft_base_record_t
            )
            return selected

        def _should_record_jt_tared_filtered_wrench_sample(self, t_rx: float) -> bool:
            selected, self.next_jt_tared_filtered_wrench_record_t = (
                self._deadline_crossed(
                    t_rx,
                    float(self.args.jt_tared_filtered_wrench_hz),
                    self.next_jt_tared_filtered_wrench_record_t,
                )
            )
            return selected

        def _should_record_jt_tared_wrench_sample(self, t_rx: float) -> bool:
            selected, self.next_jt_tared_wrench_record_t = self._deadline_crossed(
                t_rx,
                float(self.args.jt_tared_wrench_hz),
                self.next_jt_tared_wrench_record_t,
            )
            return selected

        def _should_record_contact_state_sample(self, t_rx: float) -> bool:
            selected, self.next_contact_state_record_t = self._deadline_crossed(
                t_rx,
                float(self.args.contact_state_hz),
                self.next_contact_state_record_t,
            )
            return selected

        def _should_record_contact_phase_sample(self, t_rx: float) -> bool:
            selected, self.next_contact_phase_record_t = self._deadline_crossed(
                t_rx,
                float(self.args.contact_phase_hz),
                self.next_contact_phase_record_t,
            )
            return selected

        def _should_record_command_quat_sample(self, t_rx: float) -> bool:
            selected, self.next_command_quat_record_t = self._deadline_crossed(
                t_rx,
                float(self.args.command_quat_hz),
                self.next_command_quat_record_t,
            )
            return selected

        def _should_record_current_pose_sample(self, t_rx: float) -> bool:
            selected, self.next_current_pose_record_t = self._deadline_crossed(
                t_rx,
                float(self.args.current_pose_hz),
                self.next_current_pose_record_t,
            )
            return selected

        def _pose_stamped_to_se3(self, msg, scale: float) -> Tuple[np.ndarray, str]:
            p = np.array(
                [
                    msg.pose.position.x * scale,
                    msg.pose.position.y * scale,
                    msg.pose.position.z * scale,
                ],
                dtype=np.float64,
            )
            q_xyzw = [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
            command = make_se3(p, quat_xyzw_to_rot(q_xyzw))
            frame = str(msg.header.frame_id) if hasattr(msg, "header") else ""
            return command, frame

        @staticmethod
        def _pose_values_to_se3(
            data,
            scale: float,
            euler_order: str,
            label: str,
        ) -> np.ndarray:
            data = np.asarray(data, dtype=np.float64)
            if data.size < 6:
                raise ValueError(
                    f"{label} has {data.size} elements, "
                    "expected at least 6"
                )
            values = data[:6]
            if not np.all(np.isfinite(values)):
                raise ValueError(f"{label} contains non-finite values")
            p = values[:3] * scale
            rotation = euler_deg_to_rot(values[3:6], euler_order)
            return make_se3(p, rotation)

        @staticmethod
        def _float64_pose_to_se3(
            msg,
            scale: float,
            euler_order: str,
            label: str,
        ) -> np.ndarray:
            return ChemAcpROSNode._pose_values_to_se3(
                msg.data, scale, euler_order, f"{label} Float64MultiArray")

        def _values_by_name(self, msg, values) -> Optional[np.ndarray]:
            if values is None or len(values) < 6:
                return None
            names = [str(n) for n in msg.name]
            if names and len(set(names)) != len(names):
                return None
            name_to_value = {
                str(name): float(value) for name, value in zip(names, values)
            }
            for ordered_names in (self.joint_names, self.generic_joint_names):
                ordered = [name_to_value.get(name) for name in ordered_names]
                if all(v is not None and np.isfinite(v) for v in ordered):
                    return np.asarray(ordered, dtype=np.float64)
            # A named JointState must never silently fall back to its first six
            # positions: mixed left/right messages make that ordering unsafe.
            if names:
                return None
            fallback = np.asarray(values[:6], dtype=np.float64)
            if np.all(np.isfinite(fallback)):
                return fallback
            return None

        def _hand_values_by_name(self, msg) -> Optional[np.ndarray]:
            values = getattr(msg, "position", None)
            if values is None:
                return None
            names = [str(name) for name in msg.name]
            if len(set(names)) != len(names):
                return None
            name_to_value = {
                name: float(value) for name, value in zip(names, values)
            }
            ordered = [name_to_value.get(name) for name in self.hand_joint_names]
            if all(v is not None and np.isfinite(v) for v in ordered):
                try:
                    return validate_right_hand_joint_measurements(
                        np.asarray(ordered, dtype=np.float64),
                        label="received right-hand JointState",
                    )
                except ValueError as exc:
                    with self.lock:
                        self.robot_error = str(exc)
            return None

        @staticmethod
        def _wrench_msg_to_array(msg) -> np.ndarray:
            f = np.array(
                [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z],
                dtype=np.float64,
            )
            tau = np.array(
                [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z],
                dtype=np.float64,
            )
            return np.concatenate([f, tau], axis=0)

        def _joint_cb(self, msg):
            t_rx = now_s()
            joint = self._values_by_name(msg, msg.position)
            hand_joint = (
                self._hand_values_by_name(msg)
                if self.args.record_hand and self.hand_joint_shares_joint_topic
                else None
            )
            jt = (
                self._values_by_name(msg, msg.effort)
                if self.args.record_jt and self.joint_torque_shares_joint_topic
                else None
            )
            with self.lock:
                if joint is not None:
                    self.latest_joint = joint
                    self.latest_joint_rx_t = t_rx
                    if self.ft_processor is not None:
                        self.ft_processor.update_joint(joint)
                if hand_joint is not None:
                    self.latest_hand = hand_joint
                    self.latest_hand_rx_t = t_rx
                if jt is not None:
                    self.latest_jt = jt
                    self.latest_jt_rx_t = t_rx

        def _hand_joint_cb(self, msg):
            if not self.args.record_hand:
                return
            t_rx = now_s()
            hand_joint = self._hand_values_by_name(msg)
            if hand_joint is None:
                return
            with self.lock:
                self.latest_hand = hand_joint
                self.latest_hand_rx_t = t_rx

        def _joint_torque_cb(self, msg):
            if not self.args.record_jt:
                return
            t_rx = now_s()
            jt = self._values_by_name(msg, msg.effort)
            if jt is None:
                return
            with self.lock:
                self.latest_jt = jt
                self.latest_jt_rx_t = t_rx

        def _ft_cb(self, msg):
            t_rx = now_s()
            try:
                validate_and_record_message_frame(
                    self.args,
                    msg,
                    stream_name="ft_wrench_raw",
                    expected_frame=self.args.ft_frame,
                )
            except ValueError as exc:
                with self.lock:
                    self.robot_error = str(exc)
                return
            wrench_raw = self._wrench_msg_to_array(msg)
            if not np.all(np.isfinite(wrench_raw)):
                with self.lock:
                    self.robot_error = (
                        f"FT topic '{self.args.ft_topic}' contains non-finite values"
                    )
                return
            if self.ft_processor is not None:
                try:
                    raw, payload_gravity, wrench_comp_payload = (
                        self.ft_processor.process(t_rx, wrench_raw)
                    )
                except Exception as exc:
                    with self.lock:
                        self.ft_processing_error = str(exc)
                    return
                self.args.ft_processing_metadata = self.ft_processor.metadata()
                with self.lock:
                    self.ft_processing_error = None
            else:
                raw = wrench_raw
                payload_gravity = np.zeros(6, dtype=np.float64)
                wrench_comp_payload = wrench_raw
            should_record = self._should_record_ft_sample(t_rx)
            with self.lock:
                self.latest_ft = wrench_comp_payload
                self.latest_ft_rx_t = t_rx
                if should_record:
                    self.ft_pre.append(
                        (
                            t_rx,
                            raw.copy(),
                            payload_gravity.copy(),
                            wrench_comp_payload.copy(),
                        )
                    )
            if should_record:
                self.controller.write_ft(t_rx, raw, payload_gravity, wrench_comp_payload)
                if self.ft_selected_publisher is not None:
                    self.ft_selected_publisher.publish(msg)

        def _ft_base_cb(self, msg):
            t_rx = now_s()
            try:
                validate_and_record_message_frame(
                    self.args,
                    msg,
                    stream_name="ft_wrench_base",
                    expected_frame=self.args.base_frame,
                )
            except ValueError as exc:
                with self.lock:
                    self.robot_error = str(exc)
                return
            wrench_base = self._wrench_msg_to_array(msg)
            if not np.all(np.isfinite(wrench_base)):
                with self.lock:
                    self.robot_error = (
                        f"ft base topic '{self.args.ft_base_topic}' contains "
                        "non-finite values"
                    )
                return
            should_record = self._should_record_ft_base_sample(t_rx)
            with self.lock:
                self.latest_ft_base = wrench_base
                self.latest_ft_base_rx_t = t_rx
                if should_record:
                    self.ft_base_pre.append((t_rx, wrench_base.copy()))
            if should_record:
                self.controller.write_ft_base(t_rx, wrench_base)

        def _observer_input_cb(self, msg):
            """Use the canonical 1 kHz controller aggregate for V2 robot streams."""
            source_t = ros_stamp_to_seconds(msg.header.stamp)
            expected_frame = str(self.args.observer_input_frame_id)
            if not bool(msg.valid) or source_t <= 0.0:
                return
            try:
                validate_and_record_message_frame(
                    self.args,
                    msg,
                    stream_name="observer_input",
                    expected_frame=expected_frame,
                )
            except ValueError as exc:
                with self.lock:
                    self.robot_error = str(exc)
                return
            try:
                command = self._pose_values_to_se3(
                    msg.desired_pose,
                    0.001 if self.args.command_position_unit == "mm" else 1.0,
                    self.args.command_float64_euler_order,
                    "ObserverInput desired_pose",
                )
                current_pose = self._pose_values_to_se3(
                    msg.current_pose,
                    0.001 if self.args.current_pose_position_unit == "mm" else 1.0,
                    self.args.current_pose_float64_euler_order,
                    "ObserverInput current_pose",
                )
                raw_wrench = np.asarray(
                    msg.measured_wrench_raw, dtype=np.float64).reshape(6)
                filtered_wrench = np.asarray(
                    msg.measured_wrench, dtype=np.float64).reshape(6)
                if not (
                    np.all(np.isfinite(raw_wrench))
                    and np.all(np.isfinite(filtered_wrench))
                ):
                    raise ValueError("ObserverInput wrench is non-finite")
            except Exception as exc:
                with self.lock:
                    self.robot_error = f"ObserverInput conversion failed: {exc}"
                return

            record_command = (
                self.args.record_cmd_pose
                and self._should_record_command_sample(source_t)
            )
            record_current = (
                self.args.record_current_pose
                and self._should_record_current_pose_sample(source_t)
            )
            record_raw = (
                self.args.record_jt_tared_wrench
                and self._should_record_jt_tared_wrench_sample(source_t)
            )
            record_filtered = (
                self.args.record_jt_tared_filtered_wrench
                and self._should_record_jt_tared_filtered_wrench_sample(source_t)
            )
            with self.lock:
                self.latest_command = command
                self.latest_command_rx_t = source_t
                self.latest_command_frame = str(msg.header.frame_id)
                self.latest_current_pose = current_pose
                self.latest_current_pose_rx_t = source_t
                self.latest_jt_tared_wrench = raw_wrench
                self.latest_jt_tared_wrench_rx_t = source_t
                self.latest_jt_tared_filtered_wrench = filtered_wrench
                self.latest_jt_tared_filtered_wrench_rx_t = source_t
                if record_command:
                    self.command_pre.append((source_t, command.copy()))
                if record_current:
                    self.current_pose_pre.append((source_t, current_pose.copy()))
                if record_raw:
                    self.jt_tared_wrench_pre.append((source_t, raw_wrench.copy()))
                if record_filtered:
                    self.jt_tared_filtered_wrench_pre.append(
                        (source_t, filtered_wrench.copy()))
            if record_command:
                self.controller.write_command(source_t, command)
            if record_current:
                self.controller.write_current_pose(source_t, current_pose)
            if record_raw:
                self.controller.write_jt_tared_wrench(source_t, raw_wrench)
            if record_filtered:
                self.controller.write_jt_tared_filtered_wrench(
                    source_t, filtered_wrench)

        def _jt_tared_wrench_cb(self, msg):
            t_rx = now_s()
            data = np.asarray(msg.data, dtype=np.float64)
            if data.size < 6:
                with self.lock:
                    self.robot_error = (
                        f"jt tared wrench topic '{self.args.jt_tared_wrench_topic}' "
                        f"has {data.size} elements, expected at least 6"
                    )
                return
            jt_tared_wrench = data[:6].copy()
            if not np.all(np.isfinite(jt_tared_wrench)):
                with self.lock:
                    self.robot_error = (
                        f"jt tared wrench topic '{self.args.jt_tared_wrench_topic}' "
                        "contains non-finite values"
                    )
                return
            should_record = self._should_record_jt_tared_wrench_sample(t_rx)
            with self.lock:
                self.latest_jt_tared_wrench = jt_tared_wrench
                self.latest_jt_tared_wrench_rx_t = t_rx
                if should_record:
                    self.jt_tared_wrench_pre.append((t_rx, jt_tared_wrench.copy()))
            if should_record:
                self.controller.write_jt_tared_wrench(t_rx, jt_tared_wrench)

        def _jt_tared_filtered_wrench_cb(self, msg):
            t_rx = now_s()
            data = np.asarray(msg.data, dtype=np.float64)
            if data.size < 6:
                with self.lock:
                    self.robot_error = (
                        "jt tared filtered wrench topic "
                        f"'{self.args.jt_tared_filtered_wrench_topic}' has "
                        f"{data.size} elements, expected at least 6"
                    )
                return
            jt_tared_filtered_wrench = data[:6].copy()
            if not np.all(np.isfinite(jt_tared_filtered_wrench)):
                with self.lock:
                    self.robot_error = (
                        "jt tared filtered wrench topic "
                        f"'{self.args.jt_tared_filtered_wrench_topic}' contains "
                        "non-finite values"
                    )
                return
            should_record = self._should_record_jt_tared_filtered_wrench_sample(t_rx)
            with self.lock:
                self.latest_jt_tared_filtered_wrench = jt_tared_filtered_wrench
                self.latest_jt_tared_filtered_wrench_rx_t = t_rx
                if should_record:
                    self.jt_tared_filtered_wrench_pre.append(
                        (t_rx, jt_tared_filtered_wrench.copy())
                    )
            if should_record:
                self.controller.write_jt_tared_filtered_wrench(t_rx, jt_tared_filtered_wrench)

        def _command_cb(self, msg):
            t_rx = now_s()
            scale = 0.001 if self.args.command_position_unit == "mm" else 1.0
            try:
                if hasattr(msg, "pose"):
                    command, frame = self._pose_stamped_to_se3(msg, scale)
                    validate_and_record_message_frame(
                        self.args,
                        msg,
                        stream_name="robot_command_pose",
                        expected_frame=self.args.base_frame,
                    )
                else:
                    command = self._float64_pose_to_se3(
                        msg,
                        scale,
                        self.args.command_float64_euler_order,
                        "command pose",
                    )
                    frame = ""
            except Exception as exc:
                with self.lock:
                    self.robot_error = f"command pose conversion failed: {exc}"
                return
            should_record = self._should_record_command_sample(t_rx)
            with self.lock:
                self.latest_command = command
                self.latest_command_rx_t = t_rx
                self.latest_command_frame = frame
                if should_record:
                    self.command_pre.append((t_rx, command.copy()))
            if should_record:
                self.controller.write_command(t_rx, command)

        def _current_pose_cb(self, msg):
            if not self.args.record_current_pose:
                return
            t_rx = now_s()
            scale = 0.001 if self.args.current_pose_position_unit == "mm" else 1.0
            try:
                current_pose = self._float64_pose_to_se3(
                    msg,
                    scale,
                    self.args.current_pose_float64_euler_order,
                    "controller current pose",
                )
            except Exception as exc:
                with self.lock:
                    self.robot_error = f"current pose conversion failed: {exc}"
                return
            should_record = self._should_record_current_pose_sample(t_rx)
            with self.lock:
                self.latest_current_pose = current_pose
                self.latest_current_pose_rx_t = t_rx
                if should_record:
                    self.current_pose_pre.append((t_rx, current_pose.copy()))
            if should_record:
                self.controller.write_current_pose(t_rx, current_pose)

        def _command_quat_cb(self, msg):
            t_rx = now_s()
            scale = 0.001 if self.args.command_position_unit == "mm" else 1.0
            try:
                command, frame = self._pose_stamped_to_se3(msg, scale)
                validate_and_record_message_frame(
                    self.args,
                    msg,
                    stream_name="robot_command_quat_pose",
                    expected_frame=self.args.base_frame,
                )
            except Exception as exc:
                with self.lock:
                    self.robot_error = f"command quat pose conversion failed: {exc}"
                return
            should_record = self._should_record_command_quat_sample(t_rx)
            with self.lock:
                self.latest_command_quat = command
                self.latest_command_quat_rx_t = t_rx
                self.latest_command_quat_frame = frame
                if should_record:
                    self.command_quat_pre.append((t_rx, command.copy()))
            if should_record:
                self.controller.write_command_quat(t_rx, command)

        def _contact_state_cb(self, msg):
            if not self.args.record_contact_state:
                return
            t_rx = now_s()
            contact_state = int(msg.data)
            if contact_state not in (-1, 1):
                with self.lock:
                    self.robot_error = (
                        f"legacy contact_state={contact_state}, expected -1 or 1"
                    )
                return
            should_record = self._should_record_contact_state_sample(t_rx)
            with self.lock:
                self.latest_contact_state = contact_state
                self.latest_contact_state_rx_t = t_rx
                if should_record:
                    self.contact_state_pre.append((t_rx, contact_state))
            if should_record:
                self.controller.write_contact_state(t_rx, contact_state)

        def _contact_phase_cb(self, msg):
            if not self.args.record_contact_phase:
                return
            t_rx = now_s()
            contact_phase = int(msg.data)
            if contact_phase not in (-1, 0, 1):
                with self.lock:
                    self.robot_error = (
                        f"legacy contact_phase={contact_phase}, expected -1, 0, or 1"
                    )
                return
            should_record = self._should_record_contact_phase_sample(t_rx)
            with self.lock:
                self.latest_contact_phase = contact_phase
                self.latest_contact_phase_rx_t = t_rx
                if should_record:
                    self.contact_phase_pre.append((t_rx, contact_phase))
            if should_record:
                self.controller.write_contact_phase(t_rx, contact_phase)

        def _contact_observation_cb(self, msg):
            if not self.args.record_contact_observation:
                return
            receive_t = now_s()
            source_t = ros_stamp_to_seconds(msg.header.stamp)
            source_sequence = int(msg.source_sequence)
            contact_state = int(msg.contact_state)
            wrench = np.asarray(msg.contact_wrench, dtype=np.float64).reshape(-1)
            prediction = np.asarray(
                msg.free_space_wrench_prediction, dtype=np.float64
            ).reshape(-1)
            valid = bool(msg.valid)
            model_ready = bool(msg.model_ready)
            try:
                validate_and_record_message_frame(
                    self.args,
                    msg,
                    stream_name="contact_observation",
                    expected_frame=self.args.observer_input_frame_id,
                )
            except ValueError as exc:
                with self.lock:
                    self.robot_error = str(exc)
                return
            if wrench.size != 6 or not np.all(np.isfinite(wrench)):
                with self.lock:
                    self.robot_error = (
                        "malformed ContactObservation: contact_wrench must contain "
                        "six finite values"
                    )
                return
            if prediction.size != 6 or not np.all(np.isfinite(prediction)):
                with self.lock:
                    self.robot_error = (
                        "malformed ContactObservation: free_space_wrench_prediction "
                        "must contain six finite values"
                    )
                return
            if contact_state not in (0, 1):
                with self.lock:
                    self.robot_error = (
                        f"ContactObservation contact_state={contact_state}, expected 0 or 1"
                    )
                return
            status = (contact_state, valid, model_ready)
            if source_t <= 0.0:
                if not (valid and model_ready):
                    # Before the first ObserverInput, the observer's stale
                    # watchdog has no source timestamp.  Retain its fail-closed
                    # status for readiness without poisoning robot_error.
                    with self.lock:
                        self.latest_contact_observation_rx_t = receive_t
                        self.latest_contact_observation_status = status
                    return
                with self.lock:
                    self.robot_error = (
                        "malformed ready ContactObservation: source stamp must "
                        "be positive"
                    )
                return
            try:
                prediction_age_ms = normalize_contact_prediction_age_ms(
                    msg.prediction_age_ms,
                    valid=valid,
                    model_ready=model_ready,
                )
            except (TypeError, ValueError) as exc:
                with self.lock:
                    self.robot_error = f"malformed ContactObservation: {exc}"
                return

            with self.lock:
                previous_sequence = self.latest_contact_observation_sequence
                previous_source_t = self.latest_contact_observation_source_t
                previous_status = self.latest_contact_observation_status
            same_source = (
                previous_sequence is not None
                and source_sequence == previous_sequence
                and abs(source_t - previous_source_t) <= 1e-9
            )
            restarted = (
                previous_sequence is not None
                and source_sequence <= previous_sequence
                and source_t > previous_source_t + 1e-9
            )
            forward_source = (
                previous_sequence is None
                or (
                    source_sequence > previous_sequence
                    and source_t > previous_source_t
                )
            )
            status_transition = same_source and status != previous_status
            same_identity_readiness_loss = (
                status_transition
                and previous_status is not None
                and bool(previous_status[1])
                and bool(previous_status[2])
                and not (bool(status[1]) and bool(status[2]))
            )

            # Lower sequence plus a newer Chrony-synchronised source stamp is a
            # controller process restart.  A lower/older sample is only DDS
            # reordering and must not replace the latest state.
            if not (forward_source or same_source or restarted):
                with self.lock:
                    self.robot_error = (
                        "ContactObservation source identity moved backwards: "
                        f"seq={source_sequence}, stamp={source_t:.9f}; previous "
                        f"seq={previous_sequence}, stamp={previous_source_t:.9f}"
                    )
                return
            if status_transition and not same_identity_readiness_loss:
                with self.lock:
                    self.robot_error = (
                        "ContactObservation equal-identity status transition must "
                        "only change policy readiness from valid+model-ready to "
                        "not-ready; every other change must advance "
                        "source_sequence and source timestamp"
                    )
                return

            sample = (
                source_t,
                receive_t,
                source_sequence,
                wrench.copy(),
                contact_state,
                valid,
                model_ready,
                prediction_age_ms,
                prediction.copy(),
            )
            # Normal rows use the source-time 262.5 Hz deadline.  Also retain a
            # single equal-sequence validity/readiness transition (typically
            # valid->invalid when RT input becomes stale).  Its contact_state
            # may become fail-closed FREE because the validity mask is the
            # authoritative semantic gate for that row.
            should_record = False
            if forward_source:
                should_record = self._should_record_contact_observation_sample(source_t)
            elif restarted or same_identity_readiness_loss:
                should_record = True
            with self.lock:
                self.latest_contact_observation = sample
                self.latest_contact_observation_source_t = source_t
                self.latest_contact_observation_rx_t = receive_t
                self.latest_contact_observation_sequence = source_sequence
                self.latest_contact_observation_status = status
                if restarted:
                    self.contact_observation_source_restarts += 1
                if same_identity_readiness_loss and (not valid or not model_ready):
                    self.contact_observation_invalid_transitions += 1
                if should_record:
                    self.contact_observation_pre.append(sample)
            if should_record:
                self.controller.write_contact_observation(*sample)

        def latest_joint_hand_and_jt(self):
            with self.lock:
                joint = None if self.latest_joint is None else self.latest_joint.copy()
                hand = None if self.latest_hand is None else self.latest_hand.copy()
                jt = None if self.latest_jt is None else self.latest_jt.copy()
                return (
                    joint,
                    self.latest_joint_rx_t,
                    hand,
                    self.latest_hand_rx_t,
                    jt,
                    self.latest_jt_rx_t,
                )

        def reset_contact_observer_for_episode(self) -> Tuple[bool, str]:
            client = self.contact_observer_reset_client
            if client is None:
                return True, "contact observer reset disabled"
            timeout_s = float(self.args.contact_observer_ready_timeout_sec)
            if not client.wait_for_service(timeout_sec=min(2.0, timeout_s)):
                return False, (
                    f"contact observer reset service unavailable: "
                    f"{self.args.contact_observer_reset_service}"
                )
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic() + timeout_s
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            if not future.done():
                return False, "contact observer reset service timed out"
            try:
                response = future.result()
            except Exception as exc:
                return False, f"contact observer reset failed: {exc}"
            if response is None or not bool(response.success):
                message = "no response" if response is None else response.message
                return False, f"contact observer rejected reset: {message}"
            with self.lock:
                self.contact_observation_pre.clear()
                self.latest_contact_observation = None
                self.latest_contact_observation_source_t = 0.0
                self.latest_contact_observation_rx_t = 0.0
                self.latest_contact_observation_sequence = None
                self.latest_contact_observation_status = None
                self.next_contact_observation_record_t = None
            return True, response.message

        def wait_for_contact_observer_ready(self) -> Tuple[bool, str]:
            if not self.args.record_contact_observation:
                return True, "contact observation recording disabled"
            deadline = (
                time.monotonic()
                + float(self.args.contact_observer_ready_timeout_sec)
            )
            while time.monotonic() < deadline:
                with self.lock:
                    latest = self.latest_contact_observation
                    status = self.latest_contact_observation_status
                    receive_t = self.latest_contact_observation_rx_t
                if (
                    latest is not None
                    and status is not None
                    and bool(status[1])
                    and bool(status[2])
                    and now_s() - receive_t
                    <= float(self.args.ready_max_age_sec)
                ):
                    return True, "contact observer baseline/model ready"
                time.sleep(0.01)
            return False, (
                "contact observer did not become valid after baseline reset; "
                "keep the follower contact-free and stationary"
            )

        def set_robot_error(self, error: str):
            with self.lock:
                self.robot_error = error

        def snapshot(self):
            with self.lock:
                return {
                    "ft": list(self.ft_pre),
                    "ft_base": list(self.ft_base_pre),
                    "jt_tared_wrench": list(self.jt_tared_wrench_pre),
                    "jt_tared_filtered_wrench": list(self.jt_tared_filtered_wrench_pre),
                    "command": list(self.command_pre),
                    "current_pose": list(self.current_pose_pre),
                    "command_quat": list(self.command_quat_pre),
                    "contact_state": list(self.contact_state_pre),
                    "contact_phase": list(self.contact_phase_pre),
                    "contact_observation": list(self.contact_observation_pre),
                }

        def readiness(self, window_sec: float, min_ratio: float, max_age: float):
            t_now = now_s()
            with self.lock:
                ft_ts = [x[0] for x in self.ft_pre]
                ft_base_ts = [x[0] for x in self.ft_base_pre]
                jt_tared_wrench_ts = [x[0] for x in self.jt_tared_wrench_pre]
                jt_tared_filtered_wrench_ts = [x[0] for x in self.jt_tared_filtered_wrench_pre]
                cmd_ts = [x[0] for x in self.command_pre]
                current_pose_ts = [x[0] for x in self.current_pose_pre]
                cmd_quat_ts = [x[0] for x in self.command_quat_pre]
                contact_state_ts = [x[0] for x in self.contact_state_pre]
                contact_phase_ts = [x[0] for x in self.contact_phase_pre]
                contact_observation_ts = [
                    x[0] for x in self.contact_observation_pre
                ]
                contact_observation_error = None
                if self.latest_contact_observation_status is not None:
                    _state, latest_valid, latest_model_ready = (
                        self.latest_contact_observation_status)
                    if not latest_valid or not latest_model_ready:
                        contact_observation_error = (
                            "absolute-latest observation is invalid or model not ready"
                        )
                command_frame_error = None
                if (
                    self.latest_command_frame
                    and self.latest_command_frame != self.args.base_frame
                ):
                    command_frame_error = (
                        f"command frame is '{self.latest_command_frame}', "
                        f"expected '{self.args.base_frame}'"
                    )
                command_quat_frame_error = None
                if (
                    self.latest_command_quat_frame
                    and self.latest_command_quat_frame != self.args.base_frame
                ):
                    command_quat_frame_error = (
                        f"command quat frame is '{self.latest_command_quat_frame}', "
                        f"expected '{self.args.base_frame}'"
                    )
                checks = []
                if record_ft_wrench_enabled(self.args):
                    checks.append(
                        (
                            "ft_wrench",
                            self.args.ft_hz,
                            estimate_hz(ft_ts, window_sec, t_now),
                            None
                            if self.latest_ft is None
                            else t_now - self.latest_ft_rx_t,
                            None,
                        )
                    )
                if self.args.record_ft_base:
                    checks.append(
                        (
                            "ft_base_wrench",
                            self.args.ft_base_hz,
                            estimate_hz(ft_base_ts, window_sec, t_now),
                            None
                            if self.latest_ft_base is None
                            else t_now - self.latest_ft_base_rx_t,
                            None,
                        )
                    )
                if self.args.record_jt_tared_wrench:
                    checks.append(
                        (
                            "jt_tared_wrench",
                            self.args.jt_tared_wrench_hz,
                            estimate_hz(jt_tared_wrench_ts, window_sec, t_now),
                            None
                            if self.latest_jt_tared_wrench is None
                            else t_now - self.latest_jt_tared_wrench_rx_t,
                            None,
                        )
                    )
                if self.args.record_jt_tared_filtered_wrench:
                    checks.append(
                        (
                            "jt_tared_filtered_wrench",
                            self.args.jt_tared_filtered_wrench_hz,
                            estimate_hz(
                                jt_tared_filtered_wrench_ts, window_sec, t_now
                            ),
                            None
                            if self.latest_jt_tared_filtered_wrench is None
                            else t_now - self.latest_jt_tared_filtered_wrench_rx_t,
                            None,
                        )
                    )
                if self.args.record_cmd_pose:
                    checks.append(
                        (
                            "robot_command_pose",
                            self.args.command_hz,
                            estimate_hz(cmd_ts, window_sec, t_now),
                            None
                            if self.latest_command is None
                            else t_now - self.latest_command_rx_t,
                            command_frame_error,
                        )
                    )
                if self.args.record_current_pose:
                    checks.append(
                        (
                            "robot_controller_current_pose",
                            self.args.current_pose_hz,
                            estimate_hz(current_pose_ts, window_sec, t_now),
                            None
                            if self.latest_current_pose is None
                            else t_now - self.latest_current_pose_rx_t,
                            None,
                        )
                    )
                if self.args.record_cmd_quat_pose:
                    checks.append(
                        (
                            "robot_command_quat_pose",
                            self.args.command_quat_hz,
                            estimate_hz(cmd_quat_ts, window_sec, t_now),
                            None
                            if self.latest_command_quat is None
                            else t_now - self.latest_command_quat_rx_t,
                            command_quat_frame_error,
                        )
                    )
                if self.args.record_contact_state:
                    checks.append(
                        (
                            "robot_contact_state",
                            self.args.contact_state_hz,
                            estimate_hz(contact_state_ts, window_sec, t_now),
                            None
                            if self.latest_contact_state is None
                            else t_now - self.latest_contact_state_rx_t,
                            None,
                        )
                    )
                if self.args.record_contact_phase:
                    checks.append(
                        (
                            "robot_contact_phase",
                            self.args.contact_phase_hz,
                            estimate_hz(contact_phase_ts, window_sec, t_now),
                            None
                            if self.latest_contact_phase is None
                            else t_now - self.latest_contact_phase_rx_t,
                            None,
                        )
                    )
                if self.args.record_contact_observation:
                    checks.append(
                        (
                            "contact_observation",
                            self.args.contact_observation_hz,
                            estimate_hz(contact_observation_ts, window_sec, t_now),
                            None
                            if self.latest_contact_observation is None
                            else t_now - self.latest_contact_observation_rx_t,
                            contact_observation_error,
                        )
                    )
                robot_error = self.robot_error
                ft_processing_error = self.ft_processing_error
            if robot_error:
                checks.append(
                    ("robot_input_validation", 1.0, 0.0, None, robot_error)
                )
            if ft_processing_error:
                checks.append(
                    (
                        "ft_payload_processing",
                        1.0,
                        0.0,
                        None,
                        ft_processing_error,
                    )
                )
            failures = evaluate_ready_checks(checks, min_ratio, max_age)
            return failures

    return ChemAcpROSNode


def evaluate_ready_checks(
    checks: List[Tuple[str, float, float, Optional[float], Optional[str]]],
    min_ratio: float,
    max_age: float,
) -> List[str]:
    failures = []
    for name, target, hz, age, error in checks:
        if error:
            failures.append(f"{name}: {error}")
            continue
        if age is None:
            failures.append(f"{name}: no data")
            continue
        if age > max_age:
            failures.append(f"{name}: stale latest sample age={age:.3f}s")
            continue
        if target > 0 and hz < target * min_ratio:
            failures.append(
                f"{name}: hz={hz:.1f}, target={target:.1f}, "
                f"min={target * min_ratio:.1f}"
            )
    return failures


def activation_gated_sources_expected(teleop_status: Dict[str, Any]) -> bool:
    """Return whether teleop-controlled command streams should be live now."""
    state = str(teleop_status.get("state", "")).strip().upper()
    return state in {"SLOW", "FAST", "INIT POSE", "INIT_POSE"}


def list_realsense_devices() -> None:
    _rs = load_realsense()
    ctx = _rs.context()
    devices = ctx.query_devices()
    if len(devices) == 0:
        print("No RealSense devices found.")
        return
    print("Connected RealSense devices:")
    for dev in devices:
        name = dev.get_info(_rs.camera_info.name)
        serial = dev.get_info(_rs.camera_info.serial_number)
        print(f"  serial={serial} name={name}")


def make_preview_image(cameras: List[RealSenseCamera], depth_max_mm: float) -> Optional[np.ndarray]:
    load_cv2()
    panels = []
    for cam in cameras:
        rgb, depth = cam.preview_frames()
        if rgb is None or depth is None:
            return None
        rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        depth_norm = np.clip(depth.astype(np.float32), 0, depth_max_mm)
        depth_vis = (depth_norm / max(1.0, depth_max_mm) * 255.0).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
        panels.append((rgb_bgr, depth_color))

    h = min(x.shape[0] for pair in panels for x in pair)
    w = min(x.shape[1] for pair in panels for x in pair)
    resized = []
    for rgb_bgr, depth_color in panels:
        resized.append(
            (
                cv2.resize(rgb_bgr, (w, h)),
                cv2.resize(depth_color, (w, h)),
            )
        )
    rows = [np.concatenate(pair, axis=1) for pair in resized]
    return np.concatenate(rows, axis=0)


def collect_snapshots(
    cameras: List[RealSenseCamera], robot_sampler: RobotSampler, ros_node
) -> Dict[str, Any]:
    snapshots: Dict[str, Any] = {}
    for cam in cameras:
        snapshots[f"camera_{cam.camera_id}"] = cam.snapshot()
    snapshots.update(robot_sampler.snapshot())
    snapshots.update(ros_node.snapshot())
    return snapshots


def readiness_failures(
    cameras: List[RealSenseCamera],
    robot_sampler: RobotSampler,
    ros_node,
    args: argparse.Namespace,
) -> List[str]:
    failures = []
    window_sec = args.ready_window_sec
    for cam in cameras:
        failures.extend(cam.readiness(window_sec, args.hz_min_ratio, args.ready_max_age_sec))
    failures.extend(
        robot_sampler.readiness(window_sec, args.hz_min_ratio, args.ready_max_age_sec)
    )
    failures.extend(ros_node.readiness(window_sec, args.hz_min_ratio, args.ready_max_age_sec))
    return failures


def make_status_row(
    name: str,
    target_hz: float,
    timestamps: Iterable[float],
    latest_ts: Optional[float],
    t_now: float,
    args: argparse.Namespace,
    error: Optional[str] = None,
) -> Dict[str, Any]:
    ts = list(timestamps)
    hz = estimate_hz(ts, args.ready_window_sec, t_now)
    age = None if latest_ts is None else t_now - latest_ts
    ok = True
    reason = ""
    if error:
        ok = False
        reason = str(error)
    elif age is None:
        ok = False
        reason = "no data"
    elif age > args.ready_max_age_sec:
        ok = False
        reason = f"stale age={age:.3f}s"
    elif target_hz > 0 and hz < target_hz * args.hz_min_ratio:
        ok = False
        reason = (
            f"low hz min={target_hz * args.hz_min_ratio:.1f}, "
            f"target={target_hz:.1f}"
        )
    return {
        "name": name,
        "target_hz": float(target_hz),
        "hz": float(hz),
        "age": age,
        "ok": ok,
        "reason": reason,
    }


def startup_status_rows(
    cameras: List[RealSenseCamera],
    robot_sampler: RobotSampler,
    ros_node,
    args: argparse.Namespace,
    *,
    include_activation_gated: bool = True,
) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    t_now = now_s()

    for cam in cameras:
        with cam.lock:
            rows.append(
                make_status_row(
                    f"camera_{cam.camera_id}_rgb",
                    args.camera_fps,
                    [x[0] for x in cam.rgb_pre],
                    cam.latest_rgb_ts,
                    t_now,
                    args,
                    cam.latest_error,
                )
            )
            rows.append(
                make_status_row(
                    f"camera_{cam.camera_id}_depth",
                    args.camera_fps,
                    [x[0] for x in cam.depth_pre],
                    cam.latest_depth_ts,
                    t_now,
                    args,
                    cam.latest_error,
                )
            )

    with robot_sampler.lock:
        robot_ts = [x[0] for x in robot_sampler.robot_pre]
        rows.append(
            make_status_row(
                "robot_joint",
                args.robot_sample_hz,
                robot_ts,
                robot_sampler.latest_joint_ts,
                t_now,
                args,
            )
        )
        if args.record_hand:
            hand_ts = [x[0] for x in robot_sampler.hand_pre]
            rows.append(
                make_status_row(
                    "robot_hand_joint",
                    args.robot_sample_hz,
                    hand_ts,
                    robot_sampler.latest_hand_ts,
                    t_now,
                    args,
                )
            )
        if args.record_ee_pose_fk:
            rows.append(
                make_status_row(
                    "robot_ee_pose_fk",
                    args.robot_sample_hz,
                    robot_ts,
                    robot_sampler.latest_ee_ts,
                    t_now,
                    args,
                )
            )
        if args.record_jt:
            jt_ts = [x[0] for x in robot_sampler.jt_pre]
            rows.append(
                make_status_row(
                    "jt_joint_torque",
                    args.robot_sample_hz,
                    jt_ts,
                    robot_sampler.latest_jt_ts,
                    t_now,
                    args,
                )
            )

    with ros_node.lock:
        ft_ts = [x[0] for x in ros_node.ft_pre]
        ft_base_ts = [x[0] for x in ros_node.ft_base_pre]
        jt_tared_wrench_ts = [x[0] for x in ros_node.jt_tared_wrench_pre]
        jt_tared_filtered_wrench_ts = [x[0] for x in ros_node.jt_tared_filtered_wrench_pre]
        cmd_ts = [x[0] for x in ros_node.command_pre]
        current_pose_ts = [x[0] for x in ros_node.current_pose_pre]
        cmd_quat_ts = [x[0] for x in ros_node.command_quat_pre]
        contact_state_ts = [x[0] for x in ros_node.contact_state_pre]
        contact_phase_ts = [x[0] for x in ros_node.contact_phase_pre]
        contact_observation_ts = [x[0] for x in ros_node.contact_observation_pre]
        command_frame_error = None
        if ros_node.latest_command_frame and ros_node.latest_command_frame != args.base_frame:
            command_frame_error = (
                f"frame={ros_node.latest_command_frame}, expected={args.base_frame}"
            )
        command_quat_frame_error = None
        if (
            ros_node.latest_command_quat_frame
            and ros_node.latest_command_quat_frame != args.base_frame
        ):
            command_quat_frame_error = (
                f"frame={ros_node.latest_command_quat_frame}, "
                f"expected={args.base_frame}"
            )
        if record_ft_wrench_enabled(args):
            rows.append(
                make_status_row(
                    "ft_wrench",
                    args.ft_hz,
                    ft_ts,
                    None if ros_node.latest_ft is None else ros_node.latest_ft_rx_t,
                    t_now,
                    args,
                )
            )
        if args.record_ft_base:
            rows.append(
                make_status_row(
                    "ft_base_wrench",
                    args.ft_base_hz,
                    ft_base_ts,
                    None
                    if ros_node.latest_ft_base is None
                    else ros_node.latest_ft_base_rx_t,
                    t_now,
                    args,
                )
            )
        if args.record_jt_tared_wrench:
            rows.append(
                make_status_row(
                    "jt_tared_wrench",
                    args.jt_tared_wrench_hz,
                    jt_tared_wrench_ts,
                    None
                    if ros_node.latest_jt_tared_wrench is None
                    else ros_node.latest_jt_tared_wrench_rx_t,
                    t_now,
                    args,
                )
            )
        if args.record_jt_tared_filtered_wrench:
            rows.append(
                make_status_row(
                    "jt_tared_filtered_wrench",
                    args.jt_tared_filtered_wrench_hz,
                    jt_tared_filtered_wrench_ts,
                    None
                    if ros_node.latest_jt_tared_filtered_wrench is None
                    else ros_node.latest_jt_tared_filtered_wrench_rx_t,
                    t_now,
                    args,
                )
            )
        if args.record_cmd_pose:
            rows.append(
                make_status_row(
                    "command_pose",
                    args.command_hz,
                    cmd_ts,
                    None
                    if ros_node.latest_command is None
                    else ros_node.latest_command_rx_t,
                    t_now,
                    args,
                    command_frame_error,
                )
            )
        if args.record_current_pose:
            rows.append(
                make_status_row(
                    "controller_current_pose",
                    args.current_pose_hz,
                    current_pose_ts,
                    None
                    if ros_node.latest_current_pose is None
                    else ros_node.latest_current_pose_rx_t,
                    t_now,
                    args,
                )
            )
        # The leader publishes this command only in commanding states such as
        # SLOW, FAST, and INIT POSE.
        # Exclude it from the initial IDLE startup gate to avoid a circular wait;
        # readiness_failures() still requires it when an episode starts in FAST.
        if args.record_cmd_quat_pose and include_activation_gated:
            rows.append(
                make_status_row(
                    "command_quat_pose",
                    args.command_quat_hz,
                    cmd_quat_ts,
                    None
                    if ros_node.latest_command_quat is None
                    else ros_node.latest_command_quat_rx_t,
                    t_now,
                    args,
                    command_quat_frame_error,
                )
            )
        if args.record_contact_state:
            rows.append(
                make_status_row(
                    "contact_state",
                    args.contact_state_hz,
                    contact_state_ts,
                    None
                    if ros_node.latest_contact_state is None
                    else ros_node.latest_contact_state_rx_t,
                    t_now,
                    args,
                )
            )
        if args.record_contact_phase:
            rows.append(
                make_status_row(
                    "contact_phase",
                    args.contact_phase_hz,
                    contact_phase_ts,
                    None
                    if ros_node.latest_contact_phase is None
                    else ros_node.latest_contact_phase_rx_t,
                    t_now,
                    args,
                )
            )
        if args.record_contact_observation:
            rows.append(
                make_status_row(
                    "contact_observation",
                    args.contact_observation_hz,
                    contact_observation_ts,
                    None
                    if ros_node.latest_contact_observation is None
                    else ros_node.latest_contact_observation_rx_t,
                    t_now,
                    args,
                )
            )
        robot_error = ros_node.robot_error
    if robot_error:
        rows.append(
            {
                "name": "robot_input_validation",
                "target_hz": 1.0,
                "hz": 0.0,
                "age": None,
                "ok": False,
                "reason": robot_error,
            }
        )

    return rows


def print_startup_status(
    rows: List[Dict[str, Any]],
    retry_counts: Dict[str, int],
    camera_restart_counts: Dict[int, int],
    consecutive_ok: int,
    required_ok: int,
) -> None:
    all_ok = all(row["ok"] for row in rows)
    title = "READY" if all_ok else "NOT READY"
    print(f"\n{title}: consecutive {consecutive_ok}/{required_ok}")
    for row in rows:
        name = row["name"]
        status = "ok" if row["ok"] else "problem"
        hz = row["hz"]
        age = row["age"]
        detail = f"{hz:.1f} Hz"
        if age is not None:
            detail += f", age={age:.3f}s"
        retry = retry_counts.get(name, 0)
        restart = ""
        if name.startswith("camera_"):
            try:
                camera_id = int(name.split("_")[1])
                restart = f", camera_restart={camera_restart_counts.get(camera_id, 0)}"
            except Exception:
                restart = ""
        reason = "" if row["ok"] else f", reason={row['reason']}"
        print(f"  {name:<20} {detail:<24} {status}, retry={retry}{restart}{reason}")


def format_vec6(values: Iterable[float], precision: int = 4) -> str:
    arr = np.asarray(list(values), dtype=np.float64).reshape(6)
    fmt = f"{{:+.{precision}f}}"
    return "[" + ", ".join(fmt.format(float(v)) for v in arr) + "]"


def maybe_restart_failed_cameras(
    rows: List[Dict[str, Any]],
    cameras: List[RealSenseCamera],
    last_restart: Dict[int, float],
    camera_restart_counts: Dict[int, int],
    args: argparse.Namespace,
) -> None:
    t_now = now_s()
    failed_camera_ids = set()
    for row in rows:
        if row["ok"] or not row["name"].startswith("camera_"):
            continue
        try:
            failed_camera_ids.add(int(row["name"].split("_")[1]))
        except Exception:
            continue
    for cam in cameras:
        if cam.camera_id not in failed_camera_ids:
            continue
        last = last_restart.setdefault(cam.camera_id, t_now)
        if t_now - last < args.camera_reconnect_period_sec:
            continue
        attempt = camera_restart_counts.get(cam.camera_id, 0) + 1
        reset_after = int(args.camera_hardware_reset_after_restarts)
        use_hardware_reset = attempt % reset_after == 0
        action = (
            "hardware-resetting device and restarting RealSense pipeline"
            if use_hardware_reset
            else "restarting RealSense pipeline"
        )
        print(
            f"Retry camera_{cam.camera_id}_{cam.model}: {action} "
            f"(attempt {attempt})"
        )
        try:
            cam.restart(
                hardware_reset=use_hardware_reset,
                hardware_reset_settle_sec=args.camera_hardware_reset_settle_sec,
            )
            last_restart[cam.camera_id] = now_s()
            camera_restart_counts[cam.camera_id] = (
                camera_restart_counts.get(cam.camera_id, 0) + 1
            )
        except Exception as exc:
            last_restart[cam.camera_id] = now_s()
            camera_restart_counts[cam.camera_id] = (
                camera_restart_counts.get(cam.camera_id, 0) + 1
            )
            with cam.lock:
                cam.latest_error = f"restart failed: {exc}"


def wait_for_startup_readiness(
    cameras: List[RealSenseCamera],
    robot_sampler: RobotSampler,
    ros_node,
    args: argparse.Namespace,
    stop_event: threading.Event,
) -> None:
    if args.skip_startup_check:
        print("Startup readiness check skipped by --skip-startup-check.")
        return

    print(
        "\nStartup readiness check: waiting for startup-available modalities. "
        "Teleop-activated commands are checked before episode recording."
    )
    retry_counts: Dict[str, int] = {}
    camera_restart_counts: Dict[int, int] = {}
    last_restart: Dict[int, float] = {}
    consecutive_ok = 0
    saw_failure = False

    while not stop_event.is_set():
        rows = startup_status_rows(
            cameras,
            robot_sampler,
            ros_node,
            args,
            include_activation_gated=False,
        )
        all_ok = all(row["ok"] for row in rows)
        if all_ok:
            consecutive_ok += 1
        else:
            saw_failure = True
            consecutive_ok = 0
            for row in rows:
                if not row["ok"]:
                    retry_counts[row["name"]] = retry_counts.get(row["name"], 0) + 1
        required_ok = (
            int(args.recovery_check_count)
            if saw_failure
            else int(args.startup_check_count)
        )
        print_startup_status(
            rows,
            retry_counts,
            camera_restart_counts,
            consecutive_ok,
            required_ok,
        )
        if all_ok and consecutive_ok >= required_ok:
            print("\nStartup readiness passed. Controls are enabled.")
            return
        maybe_restart_failed_cameras(
            rows,
            cameras,
            last_restart,
            camera_restart_counts,
            args,
        )
        if args.preview:
            preview = make_preview_image(cameras, args.depth_preview_max_mm)
            if preview is not None:
                cv2.imshow("Chem ACP Raw Preview", preview)
                cv2.waitKey(1)
        time.sleep(max(0.1, float(args.startup_status_period_sec)))


def health_monitor(
    controller: RecordingController,
    args: argparse.Namespace,
    stop_event: threading.Event,
) -> None:
    target_hz = {
        name: float(args.camera_fps)
        for camera_id, _model, _serial in configured_camera_specs(args)
        for name in (f"camera_{camera_id}_rgb", f"camera_{camera_id}_depth")
    }
    target_hz.update({
        "robot_joint": float(args.robot_sample_hz),
        "robot_hand_joint": float(args.robot_sample_hz),
        "robot_command_pose": (
            float(args.command_hz)
            if args.record_cmd_pose
            else float(args.robot_sample_hz)
        ),
    })
    if args.record_ee_pose_fk:
        target_hz["robot_ee_pose_fk"] = float(args.robot_sample_hz)
    if record_ft_wrench_enabled(args):
        target_hz["ft_wrench"] = float(args.ft_hz)
    if args.record_ft_base:
        target_hz["ft_base_wrench"] = float(args.ft_base_hz)
    if args.record_jt_tared_wrench:
        target_hz["jt_tared_wrench"] = float(args.jt_tared_wrench_hz)
    if args.record_jt_tared_filtered_wrench:
        target_hz["jt_tared_filtered_wrench"] = float(
            args.jt_tared_filtered_wrench_hz
        )
    if args.record_contact_state:
        target_hz["robot_contact_state"] = float(args.contact_state_hz)
    if args.record_contact_phase:
        target_hz["robot_contact_phase"] = float(args.contact_phase_hz)
    if args.record_contact_observation:
        target_hz["contact_observation"] = float(args.contact_observation_hz)
    if args.record_current_pose:
        target_hz["robot_controller_current_pose"] = float(args.current_pose_hz)
    if args.record_cmd_quat_pose:
        target_hz["robot_command_quat_pose"] = float(args.command_quat_hz)
    if not args.record_hand:
        target_hz.pop("robot_hand_joint", None)
    if args.record_jt:
        target_hz["jt_joint_torque"] = float(args.robot_sample_hz)
    consecutive_source_failures = 0
    observed_writer = None
    while not stop_event.is_set():
        writer = controller.get_writer()
        if writer is not observed_writer:
            observed_writer = writer
            consecutive_source_failures = 0
        if writer is not None:
            elapsed = now_s() - writer.start_wall_time
            if elapsed >= args.health_grace_sec:
                failures = writer.health_failures(
                    target_hz,
                    args.health_window_sec,
                    args.hz_min_ratio,
                    args.health_max_stale_sec,
                )
                memory = memory_status(args)
                failures.extend(memory["critical_reasons"])
                if failures:
                    with writer.lock:
                        writer_hard_error = bool(writer.write_error)
                    hard_failure = writer_hard_error or bool(
                        memory["critical_reasons"]
                    )
                    consecutive_source_failures += 1
                    if (
                        not hard_failure
                        and consecutive_source_failures
                        < int(args.health_failure_check_count)
                    ):
                        time.sleep(0.2)
                        continue
                    reason = "Health monitor stopped episode:\n  " + "\n  ".join(
                        failures
                    )
                    controller.request_auto_stop(reason)
                else:
                    consecutive_source_failures = 0
        time.sleep(0.2)


def load_yaml_config(path: Path) -> Dict[str, Any]:
    try:
        import yaml
    except ImportError as exc:
        raise ImportError(
            "--config-yaml requires PyYAML. Install pyyaml or omit --config-yaml."
        ) from exc
    with path.expanduser().open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML config must contain a mapping at top level: {path}")
    return data


def apply_yaml_defaults(parser: argparse.ArgumentParser, path: Path) -> None:
    data = load_yaml_config(path)
    actions_by_dest: Dict[str, List[argparse.Action]] = {}
    for action in parser._actions:
        if action.dest in (argparse.SUPPRESS, "help"):
            continue
        actions_by_dest.setdefault(action.dest, []).append(action)
    # Runtime-only switches are intentionally not recursive YAML settings.
    valid_dests = set(actions_by_dest) - {"config_yaml", "list_realsense"}
    unknown = sorted(set(data) - valid_dests)
    if unknown:
        raise ValueError(
            f"Unknown YAML config keys in {path}: {', '.join(unknown)}"
        )
    # argparse converts string defaults through an action's ``type`` callable.
    # Validate the YAML node before set_defaults so a quoted number cannot be
    # silently accepted as if it had been an actual numeric YAML scalar.
    for key, value in data.items():
        action_types = {
            action.type
            for action in actions_by_dest[key]
            if action.type is not None
        }
        if float in action_types and (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
        ):
            raise ValueError(
                f"YAML key {key!r} must be an actual numeric scalar, got "
                f"{value!r}"
            )
        if int in action_types and (
            isinstance(value, bool) or not isinstance(value, int)
        ):
            raise ValueError(
                f"YAML key {key!r} must be an actual integer scalar, got "
                f"{value!r}"
            )
    parser.set_defaults(**data)


def normalize_pose_stamped_msg_type(value: Any) -> str:
    value = str(value)
    aliases = {
        "pose_stamped",
        "PoseStamped",
        "geometry_msgs/PoseStamped",
        "geometry_msgs/msg/PoseStamped",
    }
    if value not in aliases:
        raise ValueError(
            "command_quat_msg_type must be geometry_msgs/PoseStamped "
            f"(got {value!r})"
        )
    return "geometry_msgs/PoseStamped"


def estimated_pre_roll_queue_items(args: argparse.Namespace) -> int:
    """Conservative item count needed to enqueue one complete pre-roll."""
    modality_rates = [
        float(args.camera_fps)
        for _camera in configured_camera_specs(args)
        for _stream in range(2)
    ]
    modality_rates.append(float(args.robot_sample_hz))
    if args.record_hand:
        modality_rates.append(float(args.robot_sample_hz))
    if args.record_jt:
        modality_rates.append(float(args.robot_sample_hz))
    if (
        args.record_ft_wrench_raw
        or args.record_ft_wrench_payload_gravity
        or args.record_ft_wrench_comp_payload
    ):
        modality_rates.append(float(args.ft_hz))
    if args.record_ft_base:
        modality_rates.append(float(args.ft_base_hz))
    if args.record_jt_tared_wrench:
        modality_rates.append(float(args.jt_tared_wrench_hz))
    if args.record_jt_tared_filtered_wrench:
        modality_rates.append(float(args.jt_tared_filtered_wrench_hz))
    modality_rates.append(
        float(args.command_hz)
        if args.record_cmd_pose
        else float(args.robot_sample_hz)
    )
    if args.record_current_pose:
        modality_rates.append(float(args.current_pose_hz))
    if args.record_cmd_quat_pose:
        modality_rates.append(float(args.command_quat_hz))
    if args.record_contact_state:
        modality_rates.append(float(args.contact_state_hz))
    if args.record_contact_phase:
        modality_rates.append(float(args.contact_phase_hz))
    if args.record_contact_observation:
        modality_rates.append(float(args.contact_observation_hz))
    return sum(
        max(2, int(math.ceil(float(args.pre_roll_sec) * rate)) + 2)
        for rate in modality_rates
    )


def estimated_pre_roll_payload_bytes(args: argparse.Namespace) -> int:
    """Conservative ndarray payload bytes in a completely full pre-roll.

    ``writer_queue_max_bytes`` deliberately accounts for ndarray storage rather
    than Python object overhead.  The estimate mirrors the recorder's ring
    capacities and queue payloads so an undersized byte cap fails before any
    hardware is acquired instead of corrupting the pre-roll flush.
    """

    def capacity(rate: float) -> int:
        return max(
            2,
            int(math.ceil(float(args.pre_roll_sec) * float(rate))) + 2,
        )

    camera_rows = capacity(float(args.camera_fps))
    color_pixels = int(args.color_width) * int(args.color_height)
    depth_pixels = (
        color_pixels
        if bool(args.align_depth_to_color)
        else int(args.depth_width) * int(args.depth_height)
    )
    # One RGB ndarray and one depth ndarray per enabled camera frameset.
    total = len(configured_camera_specs(args)) * camera_rows * (
        color_pixels * np.dtype(np.uint8).itemsize * 3
        + depth_pixels * np.dtype(np.uint16).itemsize
    )

    robot_rows = capacity(float(args.robot_sample_hz))
    total += robot_rows * 6 * np.dtype(np.float64).itemsize
    if args.record_ee_pose_fk or not args.record_cmd_pose:
        total += robot_rows * 16 * np.dtype(np.float64).itemsize
    if args.record_hand:
        total += (
            robot_rows
            * len(JOINT_NAMES_HAND_RIGHT)
            * np.dtype(np.float64).itemsize
        )
    if args.record_jt:
        total += robot_rows * 6 * np.dtype(np.float64).itemsize

    command_rate = (
        float(args.command_hz)
        if args.record_cmd_pose
        else float(args.robot_sample_hz)
    )
    total += capacity(command_rate) * 16 * np.dtype(np.float64).itemsize
    if args.record_current_pose:
        total += (
            capacity(float(args.current_pose_hz))
            * 16
            * np.dtype(np.float64).itemsize
        )
    if args.record_cmd_quat_pose:
        total += (
            capacity(float(args.command_quat_hz))
            * 16
            * np.dtype(np.float64).itemsize
        )

    if record_ft_wrench_enabled(args):
        # The queue carries raw, modeled gravity, and compensated arrays
        # atomically even when only a subset is persisted.
        total += (
            capacity(float(args.ft_hz))
            * 3
            * 6
            * np.dtype(np.float64).itemsize
        )
    for enabled, rate in (
        (args.record_ft_base, args.ft_base_hz),
        (args.record_jt_tared_wrench, args.jt_tared_wrench_hz),
        (
            args.record_jt_tared_filtered_wrench,
            args.jt_tared_filtered_wrench_hz,
        ),
    ):
        if enabled:
            total += (
                capacity(float(rate))
                * 6
                * np.dtype(np.float64).itemsize
            )
    if args.record_contact_observation:
        total += (
            capacity(float(args.contact_observation_hz))
            * 6
            * np.dtype(np.float64).itemsize
        )
    return int(total)


def configure_memory_budget(args: argparse.Namespace) -> None:
    """Resolve bounded recorder/system memory thresholds for this machine."""
    gib = 1024 ** 3
    mib = 1024 ** 2
    total_bytes, _available_bytes = linux_memory_bytes()
    estimated_peak = (
        estimated_pre_roll_payload_bytes(args)
        + int(args.writer_queue_max_bytes)
        + 512 * mib
    )
    configured_hard = int(getattr(args, "recorder_rss_hard_bytes", 0))
    if configured_hard > 0:
        rss_hard = configured_hard
    else:
        memory_fraction = int(0.35 * total_bytes) if total_bytes > 0 else 0
        rss_hard = min(
            3 * gib,
            max(int(math.ceil(1.5 * estimated_peak)), memory_fraction),
        )
    args.estimated_pre_roll_items = estimated_pre_roll_queue_items(args)
    args.estimated_pre_roll_payload_bytes = estimated_pre_roll_payload_bytes(args)
    args.estimated_payload_peak_bytes = int(estimated_peak)
    args.recorder_rss_hard_bytes_resolved = int(rss_hard)
    args.recorder_rss_warn_bytes_resolved = int(0.80 * rss_hard)

    configured_warn = int(getattr(args, "system_memory_warn_bytes", 0))
    configured_stop = int(getattr(args, "system_memory_stop_bytes", 0))
    args.system_memory_warn_bytes_resolved = configured_warn or max(
        2 * gib,
        int(0.15 * total_bytes) if total_bytes > 0 else 0,
    )
    args.system_memory_stop_bytes_resolved = configured_stop or max(
        1 * gib,
        int(0.08 * total_bytes) if total_bytes > 0 else 0,
    )
    if rss_hard < int(math.ceil(1.20 * estimated_peak)):
        raise ValueError(
            "resolved recorder RSS hard limit is too small for the configured "
            f"pre-roll/queue budget: hard={rss_hard} bytes, "
            f"estimated_peak={estimated_peak} bytes. Reduce pre_roll_sec, "
            "camera resolution/rates or writer_queue_max_bytes, or explicitly "
            "configure a safe recorder_rss_hard_bytes value."
        )


def memory_status(args: argparse.Namespace) -> Dict[str, Any]:
    rss = current_process_rss_bytes()
    total, available = linux_memory_bytes()
    rss_warn = int(getattr(args, "recorder_rss_warn_bytes_resolved", 0))
    rss_hard = int(getattr(args, "recorder_rss_hard_bytes_resolved", 0))
    system_warn = int(getattr(args, "system_memory_warn_bytes_resolved", 0))
    system_stop = int(getattr(args, "system_memory_stop_bytes_resolved", 0))
    critical_reasons = []
    warning_reasons = []
    if rss_hard > 0 and rss >= rss_hard:
        critical_reasons.append(
            f"recorder RSS {rss}/{rss_hard} bytes reached hard limit")
    elif rss_warn > 0 and rss >= rss_warn:
        warning_reasons.append(
            f"recorder RSS {rss}/{rss_warn} bytes reached warning limit")
    if available > 0 and system_stop > 0 and available <= system_stop:
        critical_reasons.append(
            f"system MemAvailable {available}/{system_stop} bytes reached stop reserve")
    elif available > 0 and system_warn > 0 and available <= system_warn:
        warning_reasons.append(
            f"system MemAvailable {available}/{system_warn} bytes reached warning reserve")
    level = "CRITICAL" if critical_reasons else "WARNING" if warning_reasons else "OK"
    return {
        "level": level,
        "rss_bytes": int(rss),
        "rss_warn_bytes": rss_warn,
        "rss_hard_bytes": rss_hard,
        "system_total_bytes": int(total),
        "system_available_bytes": int(available),
        "system_warn_bytes": system_warn,
        "system_stop_bytes": system_stop,
        "warning_reasons": warning_reasons,
        "critical_reasons": critical_reasons,
    }


def validate_recorder_args(
    parser: argparse.ArgumentParser, args: argparse.Namespace
) -> None:
    """Fail before hardware startup when CLI/YAML settings are unsafe."""
    boolean_fields = (
        "enable_d435",
        "record_hand",
        "record_jt",
        "record_ee_pose_fk",
        "record_ft_wrench_raw",
        "record_ft_wrench_payload_gravity",
        "record_ft_wrench_comp_payload",
        "record_ft_base",
        "record_jt_tared_wrench",
        "record_jt_tared_filtered_wrench",
        "record_cmd_pose",
        "record_current_pose",
        "record_cmd_quat_pose",
        "record_contact_state",
        "record_contact_phase",
        "record_contact_observation",
        "use_observer_input_robot_streams",
        "reset_contact_observer_each_episode",
        "align_depth_to_color",
        "ft_payload_gravity_comp",
        "skip_startup_check",
        "preview",
        "require_teleop_fast",
    )
    for field in boolean_fields:
        if not isinstance(getattr(args, field), bool):
            parser.error(
                f"{field} must be a YAML boolean (true/false), "
                f"got {getattr(args, field)!r}"
            )

    model_sha256 = str(getattr(args, "model_sha256", "")).strip().lower()
    if model_sha256 and (
        len(model_sha256) != 64
        or any(character not in "0123456789abcdef" for character in model_sha256)
    ):
        parser.error(
            "model_sha256 must be empty or 64 lowercase hexadecimal characters"
        )
    args.model_sha256 = model_sha256
    feedback_stage = float(getattr(args, "feedback_gain_scale_contract", 0.0))
    if feedback_stage not in (0.0, 0.40, 1.00):
        parser.error(
            "feedback_gain_scale_contract must be exactly 0.0, 0.40, or 1.00"
        )
    args.feedback_gain_scale_contract = feedback_stage

    positive_integer_fields = (
        "camera_fps",
        "color_width",
        "color_height",
        "depth_width",
        "depth_height",
        "writer_queue_size",
        "writer_queue_max_bytes",
        "writer_batch_size",
        "startup_check_count",
        "recovery_check_count",
        "health_failure_check_count",
        "camera_hardware_reset_after_restarts",
    )
    for field in positive_integer_fields:
        value = getattr(args, field)
        if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
            parser.error(f"{field} must be an integer, got {value!r}")
        if int(value) <= 0:
            parser.error(f"{field} must be positive, got {value!r}")

    for field in (
        "recorder_rss_hard_bytes",
        "system_memory_warn_bytes",
        "system_memory_stop_bytes",
    ):
        value = getattr(args, field)
        if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
            parser.error(f"{field} must be an integer, got {value!r}")
        if int(value) < 0:
            parser.error(f"{field} must be non-negative, got {value!r}")

    positive_float_fields = (
        "robot_sample_hz",
        "ft_hz",
        "ft_base_hz",
        "jt_tared_wrench_hz",
        "jt_tared_filtered_wrench_hz",
        "command_hz",
        "current_pose_hz",
        "command_quat_hz",
        "contact_state_hz",
        "contact_phase_hz",
        "contact_observation_hz",
        "ready_window_sec",
        "ready_max_age_sec",
        "source_stale_sec",
        "health_window_sec",
        "health_max_stale_sec",
        "startup_status_period_sec",
        "camera_reconnect_period_sec",
        "camera_hardware_reset_settle_sec",
        "contact_observer_ready_timeout_sec",
        "depth_preview_max_mm",
        "diagnostics_period_sec",
    )
    for field in positive_float_fields:
        raw_value = getattr(args, field)
        if (
            isinstance(raw_value, bool)
            or not isinstance(
                raw_value,
                (int, float, np.integer, np.floating),
            )
        ):
            parser.error(
                f"{field} must be an actual YAML/CLI number, got {raw_value!r}"
            )
        value = float(raw_value)
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f"{field} must be finite and positive, got {value!r}")
        setattr(args, field, value)

    nonnegative_float_fields = ("pre_roll_sec", "health_grace_sec")
    for field in nonnegative_float_fields:
        raw_value = getattr(args, field)
        if (
            isinstance(raw_value, bool)
            or not isinstance(
                raw_value,
                (int, float, np.integer, np.floating),
            )
        ):
            parser.error(
                f"{field} must be an actual YAML/CLI number, got {raw_value!r}"
            )
        value = float(raw_value)
        if not math.isfinite(value) or value < 0.0:
            parser.error(f"{field} must be finite and non-negative, got {value!r}")
        setattr(args, field, value)

    for field in ("hz_min_ratio", "writer_queue_warn_ratio"):
        raw_value = getattr(args, field)
        if (
            isinstance(raw_value, bool)
            or not isinstance(
                raw_value,
                (int, float, np.integer, np.floating),
            )
        ):
            parser.error(
                f"{field} must be an actual YAML/CLI number, got {raw_value!r}"
            )
        value = float(raw_value)
        if not math.isfinite(value) or not (0.0 < value <= 1.0):
            parser.error(f"{field} must be in (0, 1], got {value!r}")
        setattr(args, field, value)

    pre_roll_items = estimated_pre_roll_queue_items(args)
    if int(args.writer_queue_size) < pre_roll_items:
        parser.error(
            "writer_queue_size is too small for one configured pre-roll: "
            f"{int(args.writer_queue_size)} < estimated {pre_roll_items}. "
            "Increase the item capacity; writer_queue_max_bytes remains the "
            "payload memory bound."
        )
    pre_roll_bytes = estimated_pre_roll_payload_bytes(args)
    if int(args.writer_queue_max_bytes) < pre_roll_bytes:
        parser.error(
            "writer_queue_max_bytes is too small for one complete configured "
            f"pre-roll: {int(args.writer_queue_max_bytes)} < estimated "
            f"{pre_roll_bytes} ndarray payload bytes. Increase the byte cap or "
            "reduce pre_roll_sec/camera resolution/rates."
        )

    valid_euler_orders = {
        "xyz", "xzy", "yxz", "yzx", "zxy", "zyx",
        "zyz", "zxz", "yxy", "yzy", "xyx", "xzx",
    }
    for field in (
        "command_float64_euler_order",
        "current_pose_float64_euler_order",
    ):
        if str(getattr(args, field)) not in valid_euler_orders:
            parser.error(f"{field} has unsupported value {getattr(args, field)!r}")
    if str(args.command_msg_type) not in {
        "auto", "pose_stamped", "float64_multi_array"
    }:
        parser.error(f"command_msg_type has unsupported value {args.command_msg_type!r}")
    for field in ("command_position_unit", "current_pose_position_unit"):
        if str(getattr(args, field)) not in {"mm", "m"}:
            parser.error(f"{field} must be 'mm' or 'm'")

    required_string_fields = (
        "output_dir",
        "session_name",
        "follower_urdf",
        "joint_topic",
        "hand_joint_topic",
        "ft_topic",
        "ft_base_topic",
        "jt_tared_wrench_topic",
        "jt_tared_filtered_wrench_topic",
        "command_topic",
        "current_pose_topic",
        "observer_input_topic",
        "observer_input_frame_id",
        "command_quat_topic",
        "contact_state_topic",
        "contact_phase_topic",
        "contact_observation_topic",
        "base_frame",
        "ee_frame",
        "ft_frame",
        "ft_payload_root",
        "contact_observer_reset_service",
        "ft_payload_gravity_sign",
    )
    for field in required_string_fields:
        value = getattr(args, field)
        if not isinstance(value, str) or not value.strip():
            parser.error(f"{field} must be a non-empty string, got {value!r}")
        setattr(args, field, value.strip())

    for field in ("d405_serial", "d435_serial"):
        value = getattr(args, field)
        if value is not None and (
            not isinstance(value, str) or not value.strip()
        ):
            parser.error(
                f"{field} must be a non-empty quoted string when set, got {value!r}"
            )
        if isinstance(value, str):
            setattr(args, field, value.strip())

    if args.joint_torque_topic is not None:
        if (
            not isinstance(args.joint_torque_topic, str)
            or not args.joint_torque_topic.strip()
        ):
            parser.error(
                "joint_torque_topic must be null or a non-empty string, "
                f"got {args.joint_torque_topic!r}"
            )
        args.joint_torque_topic = args.joint_torque_topic.strip()
    if (
        args.enable_d435
        and args.d405_serial
        and args.d435_serial
        and args.d405_serial == args.d435_serial
    ):
        parser.error("d405_serial and d435_serial must identify different devices")
    session_path = Path(args.session_name)
    if (
        session_path.is_absolute()
        or len(session_path.parts) != 1
        or args.session_name in {".", ".."}
    ):
        parser.error(
            "session_name must be one directory name, not an absolute or "
            f"nested path: {args.session_name!r}"
        )
    if (
        args.use_observer_input_robot_streams
        or args.record_contact_observation
    ) and args.observer_input_frame_id != args.base_frame:
        parser.error(
            "observer_input_frame_id must equal base_frame when ObserverInput "
            "or canonical ContactObservation recording is enabled "
            f"({args.observer_input_frame_id!r} != {args.base_frame!r})"
        )
    if (
        args.reset_contact_observer_each_episode
        and not args.record_contact_observation
    ):
        parser.error(
            "reset_contact_observer_each_episode=true requires "
            "record_contact_observation=true"
        )
    payload_output_requested = bool(
        args.record_ft_wrench_payload_gravity
        or args.record_ft_wrench_comp_payload
    )
    if payload_output_requested and not args.ft_payload_gravity_comp:
        parser.error(
            "payload-gravity/compensated FT output requires "
            "ft_payload_gravity_comp=true"
        )


def parse_args() -> argparse.Namespace:
    default_urdf = (
        "~/dualarm_ws/src/aidin_dsr_dualarm_description/"
        "urdf/aidin_dsr_dualarm.urdf"
    )
    config_parser = argparse.ArgumentParser(add_help=False)
    config_parser.add_argument(
        "--config-yaml",
        type=Path,
        help="Optional recorder config YAML. CLI arguments override YAML values.",
    )
    config_args, _ = config_parser.parse_known_args()
    parser = argparse.ArgumentParser(
        description="UMI-compatible D405/D435 and Doosan right-arm recorder.",
        parents=[config_parser],
    )
    parser.add_argument("--list-realsense", action="store_true")
    parser.add_argument(
        "--validate-config-only",
        action="store_true",
        help="Validate paths/config/memory budgets and exit without opening hardware.",
    )
    parser.add_argument(
        "--d405-serial", "--camera-0-serial", dest="d405_serial", default=None
    )
    parser.add_argument(
        "--d435-serial", "--camera-1-serial", dest="d435_serial", default=None
    )
    parser.add_argument(
        "--enable-d435",
        dest="enable_d435",
        action="store_true",
        default=True,
        help="Require and record camera_1 D435.",
    )
    parser.add_argument(
        "--disable-d435",
        dest="enable_d435",
        action="store_false",
        help="Record D405 only; D435 may remain disconnected.",
    )
    parser.add_argument(
        "--output-dir", default="~/.ros/ft_fb_leaderarm/il_data"
    )
    parser.add_argument(
        "--session-name",
        default=time.strftime("session_%Y%m%d_%H%M%S"),
    )
    parser.add_argument("--follower-urdf", default=default_urdf)
    parser.add_argument(
        "--joint-topic",
        "--robot-joint-topic",
        dest="joint_topic",
        default="/dsr01/joint_states",
    )
    parser.add_argument(
        "--hand-joint-topic",
        dest="hand_joint_topic",
        default="/joint_states",
        help=(
            "JointState topic containing right hand position joints. "
            "Defaults to /joint_states, matching custom_robot_utils.py."
        ),
    )
    parser.add_argument(
        "--record-hand",
        dest="record_hand",
        action="store_true",
        default=True,
        help="Record HAND_RIGHT joints to robot/hand_joint.zarr.",
    )
    parser.add_argument(
        "--no-hand",
        "--disable-hand",
        dest="record_hand",
        action="store_false",
        help="Disable right hand joint recording.",
    )
    parser.add_argument(
        "--joint-torque-topic",
        "--jt-joint-topic",
        dest="joint_torque_topic",
        default=None,
        help=(
            "JointState topic whose effort field is stored as jt/joint_torque. "
            "Defaults to --joint-topic when omitted."
        ),
    )
    parser.add_argument("--record-jt", dest="record_jt", action="store_true", default=True)
    parser.add_argument(
        "--no-jt",
        "--disable-jt",
        dest="record_jt",
        action="store_false",
        help="Disable joint torque sensing recording from JointState.effort.",
    )

    # FT: ft/wrench_raw에서 사용한는 topic
    parser.add_argument(
        "--ft-topic",
        "--ft-raw-topic",
        dest="ft_topic",
        default="/aft_sensor2/wrench",
    )
    parser.add_argument(
        "--record-ft-wrench-raw",
        dest="record_ft_wrench_raw",
        action="store_true",
        default=True,
        help="Record raw FT wrench to ft/wrench_raw.zarr.",
    )
    parser.add_argument(
        "--no-ft-wrench-raw",
        "--disable-ft-wrench-raw",
        dest="record_ft_wrench_raw",
        action="store_false",
        help="Disable ft/wrench_raw.zarr recording.",
    )
    parser.add_argument(
        "--record-ft-wrench-payload-gravity",
        dest="record_ft_wrench_payload_gravity",
        action="store_true",
        default=False,
        help="Record modeled payload gravity wrench to ft/wrench_payload_gravity.zarr.",
    )
    parser.add_argument(
        "--no-ft-wrench-payload-gravity",
        "--disable-ft-wrench-payload-gravity",
        dest="record_ft_wrench_payload_gravity",
        action="store_false",
        help="Disable ft/wrench_payload_gravity.zarr recording.",
    )
    parser.add_argument(
        "--record-ft-wrench-comp-payload",
        dest="record_ft_wrench_comp_payload",
        action="store_true",
        default=False,
        help="Record payload-compensated FT wrench to ft/wrench_comp_payload.zarr.",
    )
    parser.add_argument(
        "--no-ft-wrench-comp-payload",
        "--disable-ft-wrench-comp-payload",
        dest="record_ft_wrench_comp_payload",
        action="store_false",
        help="Disable ft/wrench_comp_payload.zarr recording.",
    )
    # FT: ft/wrench_base에서 사용한는 topic
    parser.add_argument("--ft-base-topic", default="/aft_sensor2/wrench_base")
    parser.add_argument(
        "--record-ft-base",
        dest="record_ft_base",
        action="store_true",
        default=False,
        help="Record base-frame FT wrench from --ft-base-topic.",
    )
    parser.add_argument(
        "--no-ft-base",
        "--disable-ft-base",
        dest="record_ft_base",
        action="store_false",
        help="Disable recording of base-frame FT wrench from --ft-base-topic.",
    )
    # JT_Wrench_base: ft/jt_tared_wrench에서 사용한는 topic
    parser.add_argument(
        "--jt-tared-wrench-topic",
        "--jt-raw-wrench-topic",
        dest="jt_tared_wrench_topic",
        default="/right/F_e_raw",
    )
    parser.add_argument(
        "--record-jt-tared-wrench",
        dest="record_jt_tared_wrench",
        action="store_true",
        default=False,
        help="Record controller tared JT wrench to ft/jt_tared_wrench.zarr.",
    )
    parser.add_argument(
        "--no-jt-tared-wrench",
        "--disable-jt-tared-wrench",
        dest="record_jt_tared_wrench",
        action="store_false",
        help="Disable ft/jt_tared_wrench.zarr recording.",
    )
    # JT_Wrench_base: ft/jt_tared_wrench_filtered에서 사용한는 topic
    parser.add_argument(
        "--jt-tared-filtered-wrench-topic",
        "--jt-filtered-wrench-topic",
        "--jt-wrench-topic",
        dest="jt_tared_filtered_wrench_topic",
        default="/right/F_e",
    )
    parser.add_argument(
        "--record-jt-tared-filtered-wrench",
        "--record-jt-filtered-wrench",
        dest="record_jt_tared_filtered_wrench",
        action="store_true",
        default=True,
        help=(
            "Record controller filtered JT wrench to "
            "ft/jt_tared_filtered_wrench.zarr."
        ),
    )
    parser.add_argument(
        "--no-jt-tared-filtered-wrench",
        "--disable-jt-tared-filtered-wrench",
        "--no-jt-filtered-wrench",
        dest="record_jt_tared_filtered_wrench",
        action="store_false",
        help="Disable ft/jt_tared_filtered_wrench.zarr recording.",
    )
    # robot/command_pose_se3에서 사용하는 topic
    parser.add_argument(
        "--command-topic",
        "--robot-command-topic",
        dest="command_topic",
        default="/right_dsr_controller/task_space_command",
    )
    parser.add_argument(
        "--teleop-control-topic",
        default="",
        help="Accepted for YAML compatibility; lowhz recorder does not publish it.",
    )
    parser.add_argument(
        "--command-msg-type",
        choices=("auto", "pose_stamped", "float64_multi_array"),
        default="auto",
        help=(
            "ROS message type for --command-topic. auto uses Float64MultiArray "
            "for */desired_pose and PoseStamped otherwise."
        ),
    )
    parser.add_argument(
        "--command-float64-euler-order",
        choices=(
            "xyz",
            "xzy",
            "yxz",
            "yzx",
            "zxy",
            "zyx",
            "zyz",
            "zxz",
            "yxy",
            "yzy",
            "xyx",
            "xzx",
        ),
        default="zyz",
        help=(
            "Euler axis order for Float64MultiArray command pose [x,y,z,a,b,c]. "
            "Angles are interpreted in degrees."
        ),
    )
    parser.add_argument(
        "--record-cmd-pose",
        dest="record_cmd_pose",
        action="store_true",
        default=True,
        help="Record command pose from --command-topic.",
    )
    parser.add_argument(
        "--no-cmd-pose",
        "--disable-cmd-pose",
        "--no-command-pose",
        dest="record_cmd_pose",
        action="store_false",
        help=(
            "Do not require --command-topic. Store current EE pose into "
            "robot/command_pose_se3.zarr instead."
        ),
    )
    parser.add_argument(
        "--record-ee-pose-fk",
        dest="record_ee_pose_fk",
        action="store_true",
        default=True,
        help=(
            "Record joint-derived FK pose to robot/ee_pose_fk_se3.zarr "
            "(enabled by default for legacy compatibility)."
        ),
    )
    parser.add_argument(
        "--no-ee-pose-fk",
        "--disable-ee-pose-fk",
        dest="record_ee_pose_fk",
        action="store_false",
        help="Disable robot/ee_pose_fk_se3.zarr recording.",
    )
    parser.add_argument(
        "--current-pose-topic",
        dest="current_pose_topic",
        default="/bae_r/current_pose",
        help=(
            "Controller-published current TCP pose topic. Expected "
            "std_msgs/Float64MultiArray [x,y,z,a,b,c]."
        ),
    )
    parser.add_argument(
        "--use-observer-input-robot-streams",
        action="store_true",
        default=False,
        help=(
            "Read current_pose, desired_pose and controller wrench fields from "
            "the canonical ObserverInput aggregate instead of debug topics."
        ),
    )
    parser.add_argument(
        "--observer-input-topic", default="/bae_r/observer_input")
    parser.add_argument(
        "--observer-input-frame-id", default="right_base_link")
    parser.add_argument(
        "--record-current-pose",
        dest="record_current_pose",
        action="store_true",
        default=False,
        help=(
            "Record controller current pose to "
            "robot/controller_current_pose_se3.zarr."
        ),
    )
    parser.add_argument(
        "--no-current-pose",
        "--disable-current-pose",
        dest="record_current_pose",
        action="store_false",
        help="Disable robot/controller_current_pose_se3.zarr recording.",
    )
    parser.add_argument(
        "--current-pose-float64-euler-order",
        choices=(
            "xyz",
            "xzy",
            "yxz",
            "yzx",
            "zxy",
            "zyx",
            "zyz",
            "zxz",
            "yxy",
            "yzy",
            "xyx",
            "xzx",
        ),
        default="zyx",
        help=(
            "Euler axis order for current pose Float64MultiArray [x,y,z,a,b,c]. "
            "Angles are interpreted in degrees."
        ),
    )
    parser.add_argument(
        "--record-cmd-quat-pose",
        dest="record_cmd_quat_pose",
        action="store_true",
        default=False,
        help=(
            "Record PoseStamped quaternion command pose from --command-quat-topic "
            "to robot/command_quat_pose_se3.zarr."
        ),
    )
    parser.add_argument(
        "--no-cmd-quat-pose",
        "--disable-cmd-quat-pose",
        dest="record_cmd_quat_pose",
        action="store_false",
        help="Disable robot/command_quat_pose_se3.zarr recording.",
    )
    parser.add_argument(
        "--command-quat-topic",
        default="/right_dsr_controller/task_space_command",
        help="geometry_msgs/PoseStamped topic for quaternion command pose recording.",
    )
    parser.add_argument(
        "--command-quat-hz",
        type=float,
        default=30.0,
        help="Target recording Hz for --command-quat-topic.",
    )
    parser.add_argument(
        "--command-quat-msg-type",
        default="geometry_msgs/PoseStamped",
        help="Message type for --command-quat-topic. Only geometry_msgs/PoseStamped is supported.",
    )
    parser.add_argument(
        "--contact-state-topic",
        dest="contact_state_topic",
        default="/leader_teleop_node/contact_state",
        help=(
            "std_msgs/Int32 topic published by leader teleop. "
            "-1=no contact, 1=contact."
        ),
    )
    parser.add_argument(
        "--record-contact-state",
        dest="record_contact_state",
        action="store_true",
        default=False,
        help="Record contact state to robot/contact_state.zarr.",
    )
    parser.add_argument(
        "--no-contact-state",
        "--disable-contact-state",
        dest="record_contact_state",
        action="store_false",
        help="Disable leader teleop contact state recording.",
    )
    parser.add_argument(
        "--contact-phase-topic",
        dest="contact_phase_topic",
        default="/leader_teleop_node/contact_phase",
        help=(
            "std_msgs/Int32 topic published by leader teleop. "
            "-1=no contact, 0=pre-contact, 1=contact."
        ),
    )
    parser.add_argument(
        "--record-contact-phase",
        dest="record_contact_phase",
        action="store_true",
        default=False,
        help="Record contact phase to robot/contact_phase.zarr.",
    )
    parser.add_argument(
        "--no-contact-phase",
        "--disable-contact-phase",
        dest="record_contact_phase",
        action="store_false",
        help="Disable leader teleop contact phase recording.",
    )
    parser.add_argument(
        "--contact-observation-topic",
        default="/contact_observer/right/observation",
        help=(
            "contact_observer_msgs/ContactObservation topic. Source timestamp "
            "drives canonical contact resampling."
        ),
    )
    parser.add_argument(
        "--record-contact-observation",
        dest="record_contact_observation",
        action="store_true",
        default=False,
        help="Record aligned canonical contact diagnostics under contact/.",
    )
    parser.add_argument(
        "--no-contact-observation",
        "--disable-contact-observation",
        dest="record_contact_observation",
        action="store_false",
        help="Disable canonical ContactObservation recording.",
    )
    parser.add_argument("--base-frame", default="right_base_link")
    parser.add_argument("--ee-frame", default="right_link_6")
    parser.add_argument("--ft-frame", default="right_link_6")
    parser.add_argument(
        "--ft-payload-root",
        default="right_hand_base_link",
        help="URDF link root below the FT sensor used for payload gravity compensation.",
    )
    parser.add_argument(
        "--no-ft-payload-gravity-comp",
        dest="ft_payload_gravity_comp",
        action="store_false",
        default=True,
        help="Disable URDF-based payload gravity subtraction for FT data.",
    )
    parser.add_argument(
        "--ft-payload-gravity-sign",
        default="1,1,1,1,1,1",
        help="Sign applied to modeled payload gravity [Fx,Fy,Fz,Mx,My,Mz].",
    )
    parser.add_argument("--camera-fps", type=int, default=60)
    parser.add_argument("--color-width", type=int, default=640)
    parser.add_argument("--color-height", type=int, default=480)
    parser.add_argument("--depth-width", type=int, default=640)
    parser.add_argument("--depth-height", type=int, default=480)
    parser.add_argument(
        "--align-depth-to-color",
        dest="align_depth_to_color",
        action="store_true",
        default=True,
        help=(
            "Apply RealSense rs.align(rs.stream.color) before storing depth. "
            "When enabled, readiness checks measure the aligned depth stream rate."
        ),
    )
    parser.add_argument(
        "--no-align-depth-to-color",
        dest="align_depth_to_color",
        action="store_false",
        help=(
            "Disable RGB-depth pixel grid alignment and store raw depth stream "
            "pixels instead."
        ),
    )
    parser.add_argument("--robot-sample-hz", type=float, default=60.0)
    parser.add_argument("--ft-hz", type=float, default=350.0)
    parser.add_argument("--ft-base-hz", type=float, default=350.0)
    parser.add_argument("--jt-tared-wrench-hz", type=float, default=350.0)
    parser.add_argument(
        "--jt-tared-filtered-wrench-hz",
        "--jt-wrench-hz",
        dest="jt_tared_filtered_wrench_hz",
        type=float,
        default=350.0,
    )
    parser.add_argument("--command-hz", type=float, default=60.0)
    parser.add_argument("--current-pose-hz", type=float, default=60.0)
    parser.add_argument("--contact-state-hz", type=float, default=60.0)
    parser.add_argument("--contact-phase-hz", type=float, default=60.0)
    parser.add_argument("--contact-observation-hz", type=float, default=262.5)
    parser.add_argument(
        "--reset-contact-observer-each-episode",
        action="store_true",
        default=False,
        help="Reset the robust observer baseline before every saved demonstration.",
    )
    parser.add_argument(
        "--contact-observer-reset-service",
        default="/contact_observer_node/reset_baseline",
    )
    parser.add_argument(
        "--contact-observer-ready-timeout-sec", type=float, default=5.0)
    parser.add_argument("--hz-min-ratio", type=float, default=0.98)
    parser.add_argument("--pre-roll-sec", type=float, default=1.0)
    parser.add_argument("--writer-queue-size", type=int, default=4096)
    parser.add_argument(
        "--writer-queue-max-bytes",
        type=int,
        default=512 * 1024 * 1024,
        help="Hard cap for queued ndarray payload bytes.",
    )
    parser.add_argument("--writer-batch-size", type=int, default=128)
    parser.add_argument(
        "--writer-queue-warn-ratio",
        type=float,
        default=0.90,
        help=(
            "Report bounded writer queue pressure when occupancy reaches this "
            "ratio. Hard queue limits and writer errors still auto-stop."
        ),
    )
    parser.add_argument(
        "--control-mode",
        choices=("terminal", "ros"),
        default="terminal",
        help="terminal keeps 0/1/2 stdin controls; ros exposes Trigger services.",
    )
    parser.add_argument(
        "--teleop-status-topic",
        default="/leader_teleop_node/status",
    )
    parser.add_argument("--model-sha256", default="")
    parser.add_argument(
        "--feedback-gain-scale-contract",
        type=float,
        default=0.0,
    )
    parser.add_argument(
        "--recorder-diagnostics-topic",
        default="/chem_acp_raw_data_collection/diagnostics",
    )
    parser.add_argument(
        "--recorder-ft-selected-topic",
        default="/chem_acp_raw_data_collection/ft_selected",
    )
    parser.add_argument("--diagnostics-period-sec", type=float, default=0.5)
    parser.add_argument(
        "--require-teleop-fast",
        action="store_true",
        default=True,
        help="Reject episode start unless leader status reports FAST.",
    )
    parser.add_argument(
        "--no-require-teleop-fast",
        dest="require_teleop_fast",
        action="store_false",
    )
    parser.add_argument(
        "--recorder-rss-hard-bytes",
        type=int,
        default=0,
        help="Recorder RSS hard limit; zero resolves an adaptive limit.",
    )
    parser.add_argument(
        "--system-memory-warn-bytes",
        type=int,
        default=0,
        help="MemAvailable warning reserve; zero resolves an adaptive limit.",
    )
    parser.add_argument(
        "--system-memory-stop-bytes",
        type=int,
        default=0,
        help="MemAvailable stop reserve; zero resolves an adaptive limit.",
    )
    parser.add_argument("--ready-window-sec", type=float, default=1.0)
    parser.add_argument("--ready-max-age-sec", type=float, default=1.0)
    parser.add_argument("--source-stale-sec", type=float, default=0.25)
    parser.add_argument("--health-window-sec", type=float, default=2.0)
    parser.add_argument("--health-grace-sec", type=float, default=2.0)
    parser.add_argument("--health-max-stale-sec", type=float, default=0.5)
    parser.add_argument("--health-failure-check-count", type=int, default=5)
    parser.add_argument("--startup-check-count", type=int, default=10)
    parser.add_argument("--recovery-check-count", type=int, default=5)
    parser.add_argument("--startup-status-period-sec", type=float, default=1.0)
    parser.add_argument("--camera-reconnect-period-sec", type=float, default=5.0)
    parser.add_argument(
        "--camera-hardware-reset-after-restarts", type=int, default=3
    )
    parser.add_argument(
        "--camera-hardware-reset-settle-sec", type=float, default=6.0
    )
    parser.add_argument("--skip-startup-check", action="store_true")
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--depth-preview-max-mm", type=float, default=1000.0)
    parser.add_argument(
        "--command-position-unit",
        choices=("mm", "m"),
        default="mm",
        help="Unit used by PoseStamped command position.",
    )
    parser.add_argument(
        "--current-pose-position-unit",
        choices=("mm", "m"),
        default="mm",
        help="Unit used by controller current pose Float64MultiArray position.",
    )
    if config_args.config_yaml is not None:
        apply_yaml_defaults(parser, config_args.config_yaml)
    args = parser.parse_args()
    validate_recorder_args(parser, args)
    configure_memory_budget(args)
    if args.joint_torque_topic is None:
        args.joint_torque_topic = args.joint_topic
    args.command_quat_msg_type = normalize_pose_stamped_msg_type(
        args.command_quat_msg_type
    )
    return args


def require_camera_serials(args: argparse.Namespace) -> None:
    missing = []
    if not args.d405_serial:
        missing.append("--d405-serial")
    if args.enable_d435 and not args.d435_serial:
        missing.append("--d435-serial")
    if missing:
        raise RuntimeError(
            "Camera serial numbers are required: " + ", ".join(missing)
        )


def cleanup_runtime(
    shutdown_event: Optional[threading.Event],
    robot_sampler: Optional[RobotSampler],
    cameras: Iterable[RealSenseCamera],
    executor: Any,
    ros_node: Any,
    rclpy: Any,
    ros_thread: Optional[threading.Thread],
    health_thread: Optional[threading.Thread],
    preview_enabled: bool,
) -> None:
    """Best-effort cleanup for both partial startup and normal shutdown."""
    if shutdown_event is not None:
        shutdown_event.set()
    if robot_sampler is not None:
        try:
            robot_sampler.stop()
        except Exception:
            pass
    for camera in cameras:
        try:
            camera.stop()
        except Exception:
            pass
    if executor is not None:
        try:
            executor.shutdown()
        except Exception:
            pass
    if ros_node is not None:
        try:
            ros_node.destroy_node()
        except Exception:
            pass
    if rclpy is not None:
        try:
            rclpy.shutdown()
        except Exception:
            pass
    for thread in (ros_thread, health_thread):
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
    if preview_enabled and cv2 is not None:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


def start_cameras_until_connected(
    cameras: List[RealSenseCamera],
    reconnect_period_sec: float,
    status_node: Optional[Any] = None,
) -> None:
    """Keep retrying each unavailable RealSense until every pipeline starts."""
    pending = {camera.camera_id: camera for camera in cameras}
    attempts = {camera.camera_id: 0 for camera in cameras}
    retry_period = max(0.1, float(reconnect_period_sec))
    while pending:
        executor_error = getattr(status_node, "executor_error", "")
        if executor_error:
            raise RuntimeError(f"ROS executor failed: {executor_error}")
        for camera_id, camera in list(pending.items()):
            attempts[camera_id] += 1
            try:
                camera.start()
            except Exception as exc:
                # pipeline.start() can fail after partially acquiring resources.
                # Always unwind that camera before the next attempt.
                camera.stop()
                print(
                    f"Waiting for camera_{camera.camera_id} {camera.model} "
                    f"(serial={camera.serial}, attempt={attempts[camera_id]}): "
                    f"{exc}. Retry in {retry_period:.1f}s."
                )
                if status_node is not None:
                    status_node.note_camera_retry(camera.camera_id, str(exc))
                continue
            print(
                f"Started camera_{camera.camera_id} {camera.model}: "
                f"serial={camera.serial}, name={camera.device_name}, "
                f"depth_scale={camera.depth_scale}, "
                f"depth_align_to_color={bool(camera.args.align_depth_to_color)}"
            )
            if status_node is not None:
                status_node.note_camera_connected(camera.camera_id)
            del pending[camera_id]
        if pending:
            time.sleep(retry_period)


def start_runtime(
    args: argparse.Namespace,
    controller: RecordingController,
    ft_processor: FTProcessor,
    ros: Dict[str, Any],
    fk: Optional[FKComputer],
) -> Tuple[
    List[RealSenseCamera],
    Any,
    Any,
    Any,
    threading.Thread,
    RobotSampler,
    threading.Event,
    threading.Thread,
]:
    """Start hardware/ROS workers and unwind every acquired resource on error."""
    cameras = [
        RealSenseCamera(camera_id, model, serial, args, controller)
        for camera_id, model, serial in configured_camera_specs(args)
    ]
    rclpy = None
    ros_node = None
    executor = None
    ros_thread = None
    robot_sampler = None
    health_thread = None
    shutdown_event = threading.Event()
    try:
        rclpy = ros["rclpy"]
        rclpy.init(args=None)
        ChemAcpROSNode = make_ros_node_class(ros)
        ros_node = ChemAcpROSNode(args, controller, ft_processor)
        ros_node.bind_runtime(cameras, None, shutdown_event)
        executor = ros["MultiThreadedExecutor"](num_threads=4)
        executor.add_node(ros_node)
        ros_thread = threading.Thread(
            target=spin_ros_executor,
            args=(executor, ros_node, shutdown_event),
            daemon=True,
        )
        ros_thread.start()

        start_cameras_until_connected(
            cameras,
            args.camera_reconnect_period_sec,
            ros_node,
        )
        args.camera_calibration = {
            f"camera_{cam.camera_id}": cam.calibration_metadata()
            for cam in cameras
        }

        robot_sampler = RobotSampler(ros_node, fk, args, controller)
        robot_sampler.start()
        ros_node.bind_runtime(cameras, robot_sampler, shutdown_event)

        health_thread = threading.Thread(
            target=health_monitor,
            args=(controller, args, shutdown_event),
            daemon=True,
        )
        health_thread.start()
        return (
            cameras,
            rclpy,
            ros_node,
            executor,
            ros_thread,
            robot_sampler,
            shutdown_event,
            health_thread,
        )
    except BaseException:
        cleanup_runtime(
            shutdown_event,
            robot_sampler,
            cameras,
            executor,
            ros_node,
            rclpy,
            ros_thread,
            health_thread,
            bool(args.preview),
        )
        raise


def main() -> int:
    args = parse_args()
    # Runtime-observed ROS header frames are kept separate from the effective
    # recorder configuration so metadata can distinguish proof from declaration.
    args.observed_source_frames = {}

    if args.list_realsense:
        list_realsense_devices()
        return 0

    require_camera_serials(args)
    args.follower_urdf = str(Path(args.follower_urdf).expanduser().resolve())
    if not Path(args.follower_urdf).is_file():
        raise FileNotFoundError(
            "Follower URDF does not exist. Check follower_urdf in the recorder "
            f"YAML: {args.follower_urdf}"
        )
    output_dir = Path(args.output_dir).expanduser()
    session_dir = output_dir / args.session_name
    # Config-only validation is the integrated launch's hardware preflight.
    # Include the existing session contract so unfinished crash-recovery data
    # is reported before the observer, teleop, cameras, or GUI are started.
    validate_session_compatibility(session_dir, args)
    if args.validate_config_only:
        # Validate the selected Python environment before launch starts any
        # observer, teleop, camera, or GUI process. These imports do not open
        # hardware devices.
        load_cv2()
        load_zarr()
        load_realsense()
        load_ros2()
        if args.record_ee_pose_fk or not args.record_cmd_pose:
            load_pinocchio()
        print(json.dumps({
            "valid": True,
            "output_dir": str(output_dir.resolve()),
            "session_name": args.session_name,
            "estimated_pre_roll_items": args.estimated_pre_roll_items,
            "estimated_pre_roll_payload_bytes": args.estimated_pre_roll_payload_bytes,
            "estimated_payload_peak_bytes": args.estimated_payload_peak_bytes,
            "recorder_rss_hard_bytes": args.recorder_rss_hard_bytes_resolved,
            "system_memory_warn_bytes": args.system_memory_warn_bytes_resolved,
            "system_memory_stop_bytes": args.system_memory_stop_bytes_resolved,
        }, separators=(",", ":")))
        return 0

    # FK is only needed when it is persisted or used as the legacy command-pose
    # fallback. V2 records controller-current pose instead, so it avoids an
    # unnecessary duplicate pose stream and Pinocchio computation.
    fk = None
    if args.record_ee_pose_fk or not args.record_cmd_pose:
        load_pinocchio()
        fk = FKComputer(Path(args.follower_urdf), args.base_frame, args.ee_frame)

    load_cv2()
    load_zarr()
    load_realsense()
    ros = load_ros2()

    session_dir.mkdir(parents=True, exist_ok=True)
    args.camera_calibration = {}
    ft_processor = FTProcessor(args)
    args.ft_processing_metadata = ft_processor.metadata()

    write_json(
        session_dir / "meta.json",
        {
            "created_at": now_s(),
            "args": vars(args),
            "camera_roles": configured_camera_roles(args),
            "camera_calibration": args.camera_calibration,
            "ft_processing": args.ft_processing_metadata,
            "recorded_ft_streams": recorded_ft_stream_names(args),
            "recorder_config": recorder_config_snapshot(args),
        },
    )

    controller = RecordingController(session_dir, args, ft_processor)
    (
        cameras,
        rclpy,
        ros_node,
        executor,
        ros_thread,
        robot_sampler,
        shutdown_event,
        health_thread,
    ) = start_runtime(args, controller, ft_processor, ros, fk)
    def _signal_handler(signum, frame):
        print("\nSignal received. Shutting down.")
        shutdown_event.set()

    try:
        write_json(
            session_dir / "meta.json",
            {
                "created_at": now_s(),
                "args": vars(args),
                "camera_roles": configured_camera_roles(args),
                "camera_calibration": args.camera_calibration,
                "ft_processing": args.ft_processing_metadata,
                "recorded_ft_streams": recorded_ft_stream_names(args),
                "recorder_config": recorder_config_snapshot(args),
            },
        )
        signal.signal(signal.SIGINT, _signal_handler)
        signal.signal(signal.SIGTERM, _signal_handler)
    except BaseException:
        cleanup_runtime(
            shutdown_event,
            robot_sampler,
            cameras,
            executor,
            ros_node,
            rclpy,
            ros_thread,
            health_thread,
            bool(args.preview),
        )
        raise

    try:
        wait_for_startup_readiness(
            cameras, robot_sampler, ros_node, args, shutdown_event
        )
        if ros_node.executor_error:
            raise RuntimeError(f"ROS executor failed: {ros_node.executor_error}")
        ros_node.startup_ready = True
        args.camera_calibration = {
            f"camera_{cam.camera_id}": cam.calibration_metadata()
            for cam in cameras
        }
        write_json(
            session_dir / "meta.json",
            {
                "created_at": now_s(),
                "args": vars(args),
                "camera_roles": configured_camera_roles(args),
                "camera_calibration": args.camera_calibration,
                "ft_processing": args.ft_processing_metadata,
                "recorded_ft_streams": recorded_ft_stream_names(args),
                "recorder_config": recorder_config_snapshot(args),
            },
        )
    except BaseException:
        cleanup_runtime(
            shutdown_event,
            robot_sampler,
            cameras,
            executor,
            ros_node,
            rclpy,
            ros_thread,
            health_thread,
            bool(args.preview),
        )
        raise

    print("\nControls: 1=start episode, 2=stop episode, 0=exit")
    print(f"Session directory: {session_dir}")
    resolved_memory = memory_status(args)
    print(
        "Memory budget: "
        f"pre-roll={args.estimated_pre_roll_payload_bytes / (1024 ** 2):.2f} MiB, "
        f"estimated peak={args.estimated_payload_peak_bytes / (1024 ** 2):.2f} MiB, "
        f"RSS hard={args.recorder_rss_hard_bytes_resolved / (1024 ** 3):.2f} GiB, "
        f"MemAvailable={resolved_memory['system_available_bytes'] / (1024 ** 3):.2f} GiB"
    )

    if args.control_mode == "ros":
        print(
            "ROS controls enabled: ~/start_episode, ~/stop_save, "
            "~/stop_discard, ~/shutdown"
        )
        try:
            while not shutdown_event.is_set():
                reason = controller.consume_auto_stop_reason()
                if reason and controller.is_recording():
                    print("\n" + reason)
                    controller.stop_to_pending(interruption_reason=reason)
                    ros_node.last_control_error = reason
                if args.preview:
                    preview = make_preview_image(cameras, args.depth_preview_max_mm)
                    if preview is not None:
                        cv2.imshow("Chem ACP Raw Preview", preview)
                        cv2.waitKey(1)
                time.sleep(0.1)
        finally:
            if controller.is_recording():
                reason = "Recorder shutdown interrupted an active episode"
                controller.stop_to_pending(interruption_reason=reason)
                print(
                    "Recorder shutdown preserved the interrupted temporary episode "
                    "for inspection."
                )
            cleanup_runtime(
                shutdown_event,
                robot_sampler,
                cameras,
                executor,
                ros_node,
                rclpy,
                ros_thread,
                health_thread,
                bool(args.preview),
            )
        return 0

    try:
        with KeyReader() as key_reader:
            while not shutdown_event.is_set():
                reason = controller.consume_auto_stop_reason()
                if reason:
                    print("\n" + reason)
                    writer = controller.stop_episode(
                        interruption_reason=reason
                    )
                    if writer is not None:
                        save = prompt_yes_no("Save this interrupted episode? [y/n] ", key_reader)
                        controller.finalize_episode(writer, save)

                if args.preview:
                    preview = make_preview_image(cameras, args.depth_preview_max_mm)
                    if preview is not None:
                        cv2.imshow("Chem ACP Raw Preview", preview)
                        cv2.waitKey(1)

                ch = key_reader.read(0.1)
                if ch is None:
                    continue
                if ch == "1":
                    if controller.is_recording():
                        print("Already recording. Press 2 to stop current episode.")
                        continue
                    if args.reset_contact_observer_each_episode:
                        print(
                            "Resetting contact observer baseline; keep the follower "
                            "contact-free and stationary..."
                        )
                        ok, message = ros_node.reset_contact_observer_for_episode()
                        if not ok:
                            print(f"Cannot start recording: {message}")
                            continue
                        ok, message = ros_node.wait_for_contact_observer_ready()
                        if not ok:
                            print(f"Cannot start recording: {message}")
                            continue
                        print(message)
                    failures = readiness_failures(
                        cameras, robot_sampler, ros_node, args
                    )
                    if failures:
                        print("Cannot start recording. Missing or unhealthy modalities:")
                        for failure in failures:
                            print("  " + failure)
                        continue
                    controller.start_episode(
                        lambda: collect_snapshots(
                            cameras, robot_sampler, ros_node
                        )
                    )
                elif ch == "2":
                    if not controller.is_recording():
                        print("Not recording.")
                        continue
                    writer = controller.stop_episode()
                    if writer is not None:
                        save = prompt_yes_no("Save this episode? [y/n] ", key_reader)
                        controller.finalize_episode(writer, save)
                elif ch == "0":
                    if controller.is_recording():
                        writer = controller.stop_episode()
                        if writer is not None:
                            save = prompt_yes_no(
                                "Recording is active. Save this episode? [y/n] ",
                                key_reader,
                            )
                            controller.finalize_episode(writer, save)
                    print("Exiting raw data collection.")
                    shutdown_event.set()
                elif ch in ("\n", "\r", " "):
                    continue
                else:
                    print("Unknown key. Controls: 1=start, 2=stop, 0=exit")
    finally:
        if controller.is_recording():
            writer = controller.stop_episode()
            if writer is not None:
                print("Recorder shutdown discarded active temp episode.")
                controller.finalize_episode(writer, save=False)
        cleanup_runtime(
            shutdown_event,
            robot_sampler,
            cameras,
            executor,
            ros_node,
            rclpy,
            ros_thread,
            health_thread,
            bool(args.preview),
        )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise
