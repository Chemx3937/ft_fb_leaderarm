// single_impedance_teleop_node.hpp
// C++ 리라이트 of doosan_leaderarm_teleop_impedance_weight_comp.py (DualTeleopFT)
//
// 원본 Python 대비 변경점:
//  - 단일 팔 전용 (dual 제거) → ArmConfig 1개, 12축 연산 불필요
//  - impedance 출력 전용 (position/servoj_stream 제거)
//  - JT wrench 또는 physical F/T sensor 기반 feedback 선택 지원
//  - DXL fallback 반복 루프 제거 → single-read 1회 시도 후 cache 사용
//  - Python GIL, pybind 오버헤드, rclpy executor 오버헤드 완전 제거
//  - Pinocchio C++ API 직접 호출 (Python 바인딩 경유 X)
//  - MultiThreadedExecutor → subscriber 콜백이 제어루프 블로킹 안 함
//  - getch termios 매 루프 set/reset → 1회 초기화 + atexit 복원
//  - TerminalDashboard TUI 제거 → RCLCPP_INFO 로깅으로 대체
//  - Hz 모니터 내장 (2초 주기 실측 Hz 출력)
//  - 모든 하드웨어 상수 YAML config로 분리 (코드 수정 없이 팔 교체 가능)

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <optional>
#include <thread>
#include <vector>

#include <pinocchio/fwd.hpp>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <contact_observer_msgs/msg/contact_observation.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>

#include "ft_fb_leaderarm/dynamixel_bus.hpp"
#include "ft_fb_leaderarm/intent_trajectory_generator.hpp"

namespace teleop_cpp {

using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat3 = Eigen::Matrix3d;
using Vec3 = Eigen::Vector3d;

enum class TeleopState {
  INIT,
  ALIGN,
  MAIN_IDLE,
  PAUSED,
  CURRENT_READY,
  SLOW_SYNC,
  INIT_POSE,
  FAST,
  SHUTDOWN
};

enum class InitPosePhase {
  IDLE,
  MOVE,
  VERIFY
};

struct DiagnosticsConfig {
  double log_period_sec{1.0};
  bool color_log{true};
};

struct SlowSyncConfig {
  double max_vel_deg_s{1.0};
  double ready_tol_deg{2.0};
  double ready_hold_sec{0.5};
};

struct ArmConfig {
  std::string side;
  std::array<int, 6> dxl_ids;
  std::array<int, 6> zero_ticks;
  Vec6 joint_signs;
  Vec6 offset_rad;
  std::array<std::string, 6> leader_joint_names;
  std::array<std::string, 6> follower_joint_names;       // Pinocchio dual-arm URDF names (e.g. "left_joint_1")
  std::array<std::string, 6> follower_state_joint_names; // names as published on the live joint_states topic (e.g. "joint_1")
  std::string ee_frame;

  Vec6 torque_constant;  // Nm/A per joint
  Vec6 current_unit;     // A per DXL unit per joint
  Eigen::Matrix<int, 6, 1> max_current;  // DXL units

  Vec6 grav_gain;
  Vec6 jt_wrench_fb_gain;
};

class LeaderTeleopNode : public rclcpp::Node {
public:
  explicit LeaderTeleopNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~LeaderTeleopNode();

private:
  // ── Parameters ──
  void declare_and_load_params();
  void build_arm_config();

  // ── Pinocchio ──
  void init_pinocchio();

  // ── DXL ──
  void init_dxl();

  // ── ROS pub/sub ──
  void init_ros_io();

  // ── Control loop (called by timer) ──
  void control_loop();

  // ── State machine keyboard transitions ──
  void handle_keyboard();
  bool handle_control_command(
      char key, const char* input_source, std::string& message);
  void log_control_command_audit(
      const char* input_source, char key, const char* phase,
      bool accepted, const std::string& message);
  bool enqueue_control_command(char key, std::string& message);
  void process_pending_control_commands();
  void handle_control_service(
      char key,
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void publish_status();
  void publish_status_if_due();
  bool poll_shutdown_key(const char* context);
  void transition_to_current_ready();
  void transition_to_slow_sync();
  void transition_to_fast();
  bool follower_joint_ready_for_teleop(std::string& reason);
  bool contact_observation_ready_for_teleop(std::string& reason);
  void pause_teleop();
  void return_to_zero_and_shutdown();

  // ── Alignment ──
  bool align_leader_to_follower();
  void enter_dxl_fault(const std::string& reason);

  // ── Leader read ──
  Vec6 read_leader_positions();
  Vec6 compute_leader_damping_torque(const Vec6& q_leader);
  double compute_leader_tip_linear_speed_m_s();

  // ── Gravity compensation ──
  Vec6 compute_gravity_torque(const Vec6& q_leader);

  // ── Impedance FK + publish ──
  void publish_impedance_pose(const Vec6& follower_joint_rad);

  // ── JT wrench feedback ──
  Vec6 compute_jt_wrench_feedback();
  Vec6 apply_contact_gate_to_wrench_delta(const Vec6& delta_ft);
  void reset_contact_gate_state();
  void clamp_contact_bias();
  Vec6 condition_feedback_torque(Vec6 tau_fb);
  void jt_wrench_tare_step();
  Vec6 compute_ft_sensor_feedback();
  void ft_sensor_tare_step();

  // ── Shared: wrench → leader-base spatial wrench [M;F] + leader space Jacobian ──
  // Returns false (and zeros diagnostics) when feedback must be zero
  // (tare in progress · stale wrench · missing frames · no follower state).
  bool compute_reflected_wrench_and_jacobian(Vec6& w_spatial, Eigen::MatrixXd& J_space);
  bool compute_reflected_wrench_and_jacobian_from_delta(
      const Vec6& delta_ft,
      int source_frame_id,
      int application_frame_id,
      const Vec6& spatial_sign,
      Vec6& w_spatial,
      Eigen::MatrixXd& J_space);

  // ── Torque → current → DXL write ──
  bool send_torque_currents(const Vec6& tau);

  // ── Slow sync ──
  Vec6 slow_sync_step(const Vec6& target_rad);

  // ── Utility ──
  Vec6 clip_follower_joint_rad(const Vec6& leader_q);
  Eigen::Vector4d rotation_to_quat_xyzw(const Mat3& R);
  std::string green(const std::string& text) const;
  std::string blue(const std::string& text) const;
  std::string red(const std::string& text) const;
  std::string state_tag(const std::string& state) const;
  const char* teleop_state_name() const;
  std::string format_joint_deg(const Vec6& q, bool color_values = true) const;
  std::string format_joint_nm(const Vec6& tau) const;
  std::string format_joint_scale(const Vec6& scale) const;
  std::string format_wrench(const Vec6& wrench) const;
  std::string format_diff_deg(const Vec6& diff, double tol_deg) const;
  std::string format_ready(bool ready) const;
  std::string feedback_source_label() const;
  bool runtime_gravity_enabled() const;
  bool runtime_feedback_enabled() const;
  void log_align_status(
      const Vec6& current,
      const Vec6& target,
      double max_err_rad,
      double tol_rad,
      const std::string& status) const;

  // ── Hz monitor ──
  void hz_log_if_due();

  // ── CSV movement log ──
  void init_csv_log();
  void csv_log_row();

  // ── Follower init pose (z key) ──
  void send_follower_to_init_pose();
  void update_follower_init_pose();
  void finish_follower_init_pose(bool success, std::optional<double> max_err_deg = std::nullopt);

  // ── FT payload gravity compensation ──
  struct PayloadLinkData {
    int frame_id{-1};
    double mass{0.0};
    Vec3 com_local{Vec3::Zero()};
  };
  void setup_ft_payload_gravity_comp(const std::string& follower_urdf_path);
  Vec6 compute_ft_payload_gravity_wrench();

  // ── Callbacks ──
  void follower_joint_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
  void jt_wrench_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void contact_observation_cb(
      const contact_observer_msgs::msg::ContactObservation::SharedPtr msg);
  void ft_sensor_cb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

  // ─────────────────── State ───────────────────
  ArmConfig arm_;
  TeleopState state_{TeleopState::INIT};
  bool aligned_once_{false};
  double dt_{0.01};  // 1/hz

  // DXL
  std::unique_ptr<DxlBus> bus_;
  bool dxl_current_mode_{false};

  // Pinocchio leader
  pinocchio::Model model_leader_;
  pinocchio::Data data_leader_;
  std::array<int, 6> idxq_leader_;  // q indices for active 6 joints
  std::array<int, 6> idxv_leader_;  // v indices for active 6 joints

  // Pinocchio follower
  pinocchio::Model model_follower_;
  pinocchio::Data data_follower_;
  std::array<int, 6> idxq_follower_;
  std::array<int, 6> idxv_follower_;
  int impedance_workspace_frame_id_{-1};
  int impedance_command_base_frame_id_{-1};
  int impedance_tip_frame_id_{-1};
  int follower_base_frame_id_{-1};
  int leader_base_frame_id_{-1};
  int leader_tip_frame_id_{-1};
  std::mutex pin_follower_mtx_;

  // Leader state
  Vec6 q_leader_{Vec6::Zero()};
  Vec6 q_leader_last_{Vec6::Zero()};
  Eigen::VectorXd q_full_leader_;  // full model nq
  bool last_leader_position_read_fresh_{false};

  // Leader damping: C=differentiate_pose, D=dxl_vel.
  bool use_leader_damping_{false};
  std::string leader_damping_velocity_source_{"differentiate_pose"};
  bool leader_damping_use_dxl_velocity_{false};
  Vec6 leader_damping_gain_{Vec6::Zero()};
  Vec6 leader_damping_lpf_cutoff_hz_{Vec6::Zero()};
  Vec6 leader_damping_lpf_alpha_{Vec6::Ones()};
  Vec6 leader_damping_clip_Nm_{Vec6::Zero()};
  Vec6 leader_dq_{Vec6::Zero()};
  Vec6 leader_dq_lpf_state_{Vec6::Zero()};
  Vec6 leader_dq_prev_q_{Vec6::Zero()};
  Vec6 leader_dxl_velocity_rad_s_{Vec6::Zero()};
  bool leader_dq_lpf_init_{false};
  bool leader_dq_prev_init_{false};
  double leader_dq_prev_t_{0.0};
  Vec6 last_tau_damp_{Vec6::Zero()};

  // Follower state
  std::mutex follower_mtx_;
  std::optional<Vec6> follower_joint_rad_;
  std::chrono::steady_clock::time_point follower_joint_receive_steady_{};
  bool follower_joint_receive_steady_valid_{false};
  double follower_joint_stale_timeout_{0.050};

  // Gravity comp
  bool use_gravity_comp_{true};
  Vec6 grav_gain_base_{Vec6::Zero()};
  Vec6 grav_scale_{Vec6::Zero()};
  Vec6 grav_scale_target_{Vec6::Zero()};
  Vec6 grav_sync_scale_{Vec6::Constant(4.5)};
  double grav_ramp_sec_{1.0};
  Vec6 tau_grav_lpf_cutoff_hz_{Vec6::Zero()};
  Vec6 tau_grav_lpf_alpha_{Vec6::Ones()};
  Vec6 tau_grav_lpf_state_{Vec6::Zero()};
  bool tau_grav_lpf_init_{false};

  // Impedance
  std::string impedance_topic_;
  std::string impedance_base_frame_;
  std::string impedance_workspace_frame_{"base_link"};
  double impedance_lin_speed_m_s_{0.15};
  double impedance_ang_speed_rad_s_{0.0};
  double impedance_first_publish_clip_guard_m_{0.02};
  Vec3 workspace_min_;
  Vec3 workspace_max_;
  std::optional<Vec3> impedance_last_pos_;
  std::optional<Mat3> impedance_last_R_;
  double impedance_last_t_{0.0};

  // FAST command path: raw follower FK -> leader intent -> final safety slew.
  IntentTrajectoryGenerator intent_generator_;
  bool intent_generator_enabled_{true};
  std::optional<Vec3> task_raw_last_pos_;
  std::optional<Mat3> task_raw_last_R_;
  double task_raw_last_t_{0.0};
  Vec6 last_task_raw_mm_rpy_deg_{Vec6::Zero()};
  Vec6 last_task_intent_mm_rpy_deg_{Vec6::Zero()};
  Vec6 last_task_raw_velocity_{Vec6::Zero()};
  Vec6 last_task_intent_velocity_{Vec6::Zero()};
  Vec6 last_task_intent_acceleration_{Vec6::Zero()};

  // SE3 transform: workspace -> command base
  bool has_workspace_M_command_{false};
  pinocchio::SE3 workspace_M_command_;

  // JT wrench feedback
  bool use_jt_wrench_fb_{true};
  bool use_contact_observer_fb_{false};
  Vec6 jt_wrench_fb_gain_;
  Vec6 jt_wrench_fb_gain_base_{Vec6::Zero()};
  Vec6 jt_wrench_fb_clip_{Vec6::Constant(1.0)};  // per-joint torque clip (Nm), sized to each motor
  int jt_wrench_tare_N_{30};
  double jt_wrench_stale_timeout_{0.5};

  std::mutex jt_wrench_mtx_;
  Vec6 jt_wrench_raw_{Vec6::Zero()};
  double jt_wrench_stamp_{0.0};
  bool jt_wrench_tare_req_{true};
  Vec6 jt_wrench_baseline_{Vec6::Zero()};
  int jt_wrench_tare_count_{0};
  Vec6 jt_wrench_tare_accum_{Vec6::Zero()};
  Vec6 jt_wrench_sign_{Vec6::Ones()};
  Vec6 last_tau_fb_{Vec6::Zero()};       // final feedback torque (post single clip) — for CSV log
  Vec6 last_tau_unclipped_{Vec6::Zero()};// total feedback BEFORE the single clip — true saturation diagnostic
  Vec6 last_w_base_{Vec6::Zero()};       // base-frame feedback wrench [Fx,Fy,Fz,Mx,My,Mz] — for CSV log

  // Canonical learned contact observation. The callback may run at 1 kHz;
  // the 500 Hz control loop consumes only the latest valid value.
  std::mutex contact_observation_mtx_;
  Vec6 contact_observation_wrench_{Vec6::Zero()};
  double contact_observation_receive_stamp_{0.0};
  double contact_observation_source_stamp_{0.0};
  std::chrono::steady_clock::time_point contact_observation_receive_steady_{};
  bool contact_observation_receive_steady_valid_{false};
  double contact_observation_stale_timeout_{0.020};
  double contact_observation_clock_future_tolerance_{0.002};
  uint64_t contact_observation_source_sequence_{0};
  uint64_t contact_observation_prediction_sequence_{0};
  uint8_t contact_observation_state_{0};
  bool contact_observation_valid_{false};
  bool contact_observation_model_ready_{false};
  double contact_observation_score_n_{0.0};
  double contact_observation_prediction_age_ms_{-1.0};
  double contact_observation_latency_ms_{-1.0};
  bool contact_observation_feedback_active_{false};  // control-thread owned

  // Physical F/T sensor feedback (/aft_sensor*/wrench)
  bool use_ft_sensor_feedback_{false};
  Vec6 ft_fb_gain_{Vec6::Zero()};
  Vec6 ft_fb_gain_base_{Vec6::Zero()};
  Vec6 ft_fb_clip_{Vec6::Constant(1.0)};
  int ft_tare_N_{30};
  double ft_stale_timeout_{0.5};
  std::string ft_topic_;
  std::string ft_frame_name_;
  int ft_sensor_frame_id_{-1};
  Vec6 ft_feedback_wrench_sign_{Vec6::Ones()};  // spatial [Mx,My,Mz,Fx,Fy,Fz]
  std::mutex ft_sensor_mtx_;
  Vec6 ft_sensor_raw_{Vec6::Zero()};       // raw [Fx,Fy,Fz,Mx,My,Mz]
  Vec6 ft_sensor_baseline_{Vec6::Zero()};
  Vec6 ft_sensor_tare_accum_{Vec6::Zero()};
  int ft_sensor_tare_count_{0};
  bool ft_sensor_tare_req_{true};
  double ft_sensor_stamp_{0.0};
  double ft_sensor_tare_last_stamp_{0.0};

  bool use_ft_payload_gravity_comp_{false};
  std::string ft_payload_root_frame_;
  Vec6 ft_payload_gravity_sign_{Vec6::Ones()};  // [Fx,Fy,Fz,Mx,My,Mz]
  std::vector<PayloadLinkData> ft_payload_links_;
  Vec6 last_ft_payload_gravity_{Vec6::Zero()};

  // Per-joint saturation counters (reset each Hz-log period) — live visibility
  // into how often each joint rails at jt_wrench_fb_clip_.
  std::array<int, 6> sat_count_{};
  int sat_ticks_{0};

  // 1st-order low-pass on the reflected feedback torque output (per joint).
  // Applied in control_loop to tau_fb before the final safety clip.
  Vec6 tau_lpf_cutoff_hz_{Vec6::Zero()};   // PER-JOINT output-torque LPF cutoff (Hz); <=0 = off
  Vec6 tau_lpf_alpha_{Vec6::Ones()};       // per-joint EMA coeff (1.0 = passthrough)
  Vec6 tau_lpf_state_{Vec6::Zero()};       // filter state
  bool tau_lpf_init_{false};               // state primed
  Vec6 tau_fb_deadband_Nm_{Vec6::Zero()};
  Vec6 tau_fb_slew_rate_Nm_s_{Vec6::Zero()};
  Vec6 tau_fb_slew_state_{Vec6::Zero()};
  bool tau_fb_slew_init_{false};
  bool tau_fb_motion_gate_enable_{false};
  std::string tau_fb_motion_gate_speed_source_{"ee_linear"};
  double tau_fb_motion_gate_speed_low_m_s_{0.0};
  double tau_fb_motion_gate_speed_high_m_s_{0.0};
  double tau_fb_motion_gate_speed_low_rad_s_{0.0};
  double tau_fb_motion_gate_speed_high_rad_s_{0.0};
  double tau_fb_motion_gate_min_scale_{1.0};
  double last_tau_fb_gate_scale_{1.0};
  double last_tau_fb_motion_gate_speed_{0.0};
  bool tau_fb_passivity_gate_enable_{false};
  double tau_fb_passivity_power_start_W_{0.0};
  double tau_fb_passivity_power_full_W_{0.05};
  double tau_fb_passivity_min_scale_{0.0};
  double last_tau_fb_passivity_power_W_{0.0};
  double last_tau_fb_passivity_scale_{1.0};
  bool tau_fb_contact_gate_enable_{false};
  bool use_pre_contact_phase_{false};
  double tau_fb_contact_force_on_N_{4.0};
  double tau_fb_contact_force_off_N_{2.0};
  double tau_fb_contact_moment_on_Nm_{0.20};
  double tau_fb_contact_moment_off_Nm_{0.08};
  double tau_fb_contact_on_hold_s_{0.030};
  double tau_fb_contact_off_hold_s_{0.120};
  double tau_fb_contact_free_scale_{0.03};
  double tau_fb_contact_ramp_up_s_{0.030};
  double tau_fb_contact_ramp_down_s_{0.120};
  bool tau_fb_contact_speed_gate_enable_{false};
  double tau_fb_contact_speed_low_m_s_{0.04};
  double tau_fb_contact_speed_high_m_s_{0.20};
  double tau_fb_contact_force_on_fast_N_{8.0};
  double tau_fb_contact_force_off_fast_N_{6.5};
  double tau_fb_contact_moment_on_fast_Nm_{1.0};
  double tau_fb_contact_moment_off_fast_Nm_{0.5};
  double tau_fb_contact_on_max_joint_speed_rad_s_{0.0};
  double tau_fb_contact_on_max_ee_speed_m_s_{0.0};
  bool tau_fb_contact_bias_enable_{false};
  double tau_fb_contact_bias_lpf_cutoff_hz_{0.3};
  double tau_fb_contact_bias_update_max_ee_speed_m_s_{0.04};
  double tau_fb_contact_bias_update_max_joint_speed_rad_s_{0.0};
  double tau_fb_contact_bias_force_clip_N_{0.0};
  bool tau_fb_contact_stale_bias_reset_enable_{false};
  double tau_fb_contact_stale_bias_raw_force_max_N_{3.5};
  double tau_fb_contact_stale_bias_residual_force_min_N_{4.0};
  double tau_fb_contact_stale_bias_residual_force_max_N_{8.0};
  double tau_fb_contact_stale_bias_speed_max_m_s_{0.08};
  double tau_fb_contact_stale_bias_hold_s_{0.50};
  double tau_fb_contact_bias_alpha_{1.0};
  Vec6 tau_fb_contact_bias_{Vec6::Zero()};
  bool tau_fb_contact_bias_init_{false};
  bool tau_fb_contact_state_{false};
  int tau_fb_contact_phase_{-1};  // -1=free, 0=pre-contact, 1=contact
  double tau_fb_contact_scale_{1.0};
  double tau_fb_contact_on_since_{-1.0};
  double tau_fb_contact_off_since_{-1.0};
  double tau_fb_contact_last_t_{0.0};
  double tau_fb_contact_stale_bias_since_{-1.0};
  double last_tau_fb_contact_f_norm_N_{0.0};
  double last_tau_fb_contact_m_norm_Nm_{0.0};
  double last_tau_fb_contact_raw_f_norm_N_{0.0};
  double last_tau_fb_contact_ee_speed_m_s_{0.0};
  double last_tau_fb_contact_joint_speed_rad_s_{0.0};
  double last_tau_fb_contact_force_on_eff_N_{0.0};
  double last_tau_fb_contact_force_off_eff_N_{0.0};
  double last_tau_fb_contact_moment_on_eff_Nm_{0.0};
  double last_tau_fb_contact_moment_off_eff_Nm_{0.0};
  bool last_tau_fb_contact_on_speed_ok_{true};
  bool last_tau_fb_contact_stale_bias_reset_{false};
  bool tau_fb_contact_gate_active_period_{false};
  double tau_fb_contact_min_scale_period_{1.0};
  double tau_fb_contact_max_f_norm_period_N_{0.0};
  double tau_fb_contact_max_m_norm_period_Nm_{0.0};
  bool tau_fb_motion_gate_active_period_{false};
  double tau_fb_motion_gate_max_speed_period_{0.0};
  double tau_fb_motion_gate_min_scale_period_{1.0};
  bool tau_fb_passivity_gate_active_period_{false};
  double tau_fb_passivity_max_power_period_W_{0.0};
  double tau_fb_passivity_min_scale_period_{1.0};
  std::array<bool, 6> tau_fb_passivity_gate_joint_{};
  std::array<bool, 6> tau_fb_passivity_gate_joint_period_{};
  std::array<bool, 6> tau_fb_deadband_active_{};
  std::array<bool, 6> tau_fb_deadband_active_period_{};
  std::array<bool, 6> tau_fb_slew_active_{};
  std::array<bool, 6> tau_fb_slew_active_period_{};
  std::array<bool, 6> tau_fb_motion_gate_driver_{};
  std::array<bool, 6> tau_fb_motion_gate_driver_period_{};
  Vec6 tau_fb_motion_gate_driver_speed_period_{Vec6::Zero()};

  // Slow sync
  double slow_sync_max_vel_rad_s_{0.0};
  double slow_sync_capture_delay_s_{0.6};
  double slow_sync_ready_hold_sec_{0.5};
  double slow_sync_ready_tol_rad_{0.0};
  SlowSyncConfig slow_sync_config_;
  std::optional<Vec6> slow_sync_cmd_;
  double slow_sync_last_t_{0.0};
  double slow_sync_ready_since_{0.0};
  bool slow_sync_ready_{false};
  double slow_sync_capture_until_{0.0};

  // ROS
  // Reentrant so subscription callbacks can run while the (blocking) align
  // timer callback sleeps — they'd otherwise share the default Mutually
  // Exclusive group and follower_joint_cb could never run during align.
  rclcpp::CallbackGroup::SharedPtr sub_callback_group_;
  rclcpp::CallbackGroup::SharedPtr control_service_callback_group_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_impedance_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_contact_state_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_contact_phase_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_follower_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_jt_wrench_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_jt_wrench_fallback_;
  rclcpp::Subscription<contact_observer_msgs::msg::ContactObservation>::SharedPtr
    sub_contact_observation_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_ft_sensor_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> control_services_;

  // Keyboard
  std::atomic<bool> shutdown_requested_{false};
  std::atomic<bool> shutdown_started_{false};
  std::atomic<bool> shutdown_command_pending_{false};
  std::atomic<int> pending_control_command_{0};
  std::atomic<int> active_control_command_{0};
  bool dxl_fault_active_{false};
  std::string dxl_fault_reason_;
  bool keyboard_input_enabled_{true};
  double status_publish_hz_{10.0};
  std::chrono::steady_clock::time_point last_status_publish_steady_{};
  bool last_status_publish_steady_valid_{false};
  std::string last_control_message_;

  // Hz monitor
  int loop_count_{0};
  int dxl_read_count_{0};
  int dxl_fresh_count_{0};
  int dxl_fallback_count_{0};
  int dxl_missing_count_{0};
  int vel_read_count_{0};
  int vel_fresh_count_{0};
  int vel_fallback_count_{0};
  int vel_missing_count_{0};
  int dxl_degraded_read_streak_{0};
  int teleop_cmd_count_{0};
  double hz_log_last_t_{0.0};
  double hz_log_period_{2.0};
  bool color_log_{true};
  DiagnosticsConfig diagnostics_config_;
  std::string feedback_source_{"contact_observer"};
  double feedback_gain_scale_contract_{0.0};
  bool follower_command_publish_enabled_{true};
  Vec6 last_tau_grav_{Vec6::Zero()};
  Vec6 last_tau_cmd_{Vec6::Zero()};
  Vec6 last_follower_target_{Vec6::Zero()};
  Vec6 last_task_cmd_mm_rpy_deg_{Vec6::Zero()};
  bool last_task_cmd_valid_{false};

  // CSV movement log
  bool csv_log_enabled_{true};
  std::ofstream csv_file_;
  std::string csv_log_path_;
  double csv_last_t_{-1.0};
  int csv_flush_counter_{0};

  // Follower init pose
  Vec6 follower_init_pose_rad_{Vec6::Zero()};
  double follower_init_pose_duration_sec_{5.0};
  std::atomic<bool> init_pose_in_progress_{false};
  std::atomic<bool> init_pose_reached_{false};
  std::atomic<bool> init_pose_verified_{false};
  InitPosePhase init_pose_phase_{InitPosePhase::IDLE};
  Vec6 init_pose_start_rad_{Vec6::Zero()};
  Vec6 init_pose_target_rad_{Vec6::Zero()};
  double init_pose_start_t_{0.0};
  double init_pose_duration_s_{0.0};
  double init_pose_verify_start_t_{0.0};
  double init_pose_settle_start_t_{0.0};
  double init_pose_last_log_t_{0.0};
};

}  // namespace teleop_cpp
