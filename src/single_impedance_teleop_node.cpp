// single_impedance_teleop_node.cpp
// Single impedance teleop 노드 생성, 파라미터 로딩, Pinocchio/DXL 초기화, 정렬, 메인 제어루프
//
// 원본 Python 대비 변경점:
//  - __init__() 8000줄 → 파라미터/Pinocchio/DXL/ROS I/O 분리
//  - rclpy.spin() → MultiThreadedExecutor (subscriber 콜백 비차단)
//  - 정렬(align) 시 shared_from_this() 대신 executor 콜백에 의존
//  - run_step() → control_loop(): 동일 순서 (read→grav→fb→torque→publish→keyboard)

#include <pinocchio/fwd.hpp>

#include "ft_fb_leaderarm/single_impedance_teleop_node.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <stack>
#include <unordered_set>
#include <unordered_map>
#include <tinyxml2.h>

#ifndef FT_FB_LEADERARM_PACKAGE_SOURCE_DIR
#define FT_FB_LEADERARM_PACKAGE_SOURCE_DIR ""
#endif

namespace teleop_cpp {

static const std::string& log_rule();
static std::string log_separator();
static bool any_joint_flag(const std::array<bool, 6>& flags);
static std::string format_joint_flags(const std::array<bool, 6>& flags);
static std::string format_joint_flags_with_abs_values(
    const std::array<bool, 6>& flags,
    const Vec6& values,
    const std::string& unit);
static double smoothstep01(double x);
static std::filesystem::path ft_fb_leaderarm_package_root();
static std::filesystem::path resolve_csv_log_dir(const std::string& configured_dir);

// ═══════════════════════════════════════════════════════════════════════════════
//  Constructor
// ═══════════════════════════════════════════════════════════════════════════════

LeaderTeleopNode::LeaderTeleopNode(const rclcpp::NodeOptions& options)
  : Node("leader_teleop_node", options)
{
  declare_and_load_params();
  build_arm_config();
  init_pinocchio();
  init_dxl();
  init_ros_io();
  init_csv_log();

  state_ = TeleopState::INIT;
  RCLCPP_INFO(get_logger(), "[INIT] Side=%s, hz=%.0f, DXL IDs=[%d,%d,%d,%d,%d,%d]",
    arm_.side.c_str(), 1.0 / dt_,
    arm_.dxl_ids[0], arm_.dxl_ids[1], arm_.dxl_ids[2],
    arm_.dxl_ids[3], arm_.dxl_ids[4], arm_.dxl_ids[5]);
  RCLCPP_INFO(get_logger(), "[INIT] Impedance topic: %s", impedance_topic_.c_str());
  RCLCPP_INFO(get_logger(),
    "[INIT] Wrench feedback: source=%s contact_observer=%s jt_wrench=%s ft=%s",
    feedback_source_label().c_str(),
    use_contact_observer_fb_ ? "ON" : "OFF",
    use_jt_wrench_fb_ ? "ON" : "OFF",
    use_ft_sensor_feedback_ ? "ON" : "OFF");

  // Alignment uses sleep + expects executor to deliver subscriber callbacks.
  // Must happen after constructor returns → deferred via one-shot timer.
  // The timer must be kept alive in a member: CallbackGroup only stores a
  // weak_ptr, so a local shared_ptr here would destroy the timer (and cancel
  // it) the instant the constructor returns, before the executor ever spins.
  init_timer_ = create_wall_timer(
    std::chrono::milliseconds(0),
    [this]() {
      if (aligned_once_) return;

      active_control_command_.store(static_cast<int>('r'));
      const bool aligned = align_leader_to_follower();
      active_control_command_.store(0);
      if (shutdown_requested_.load() || !rclcpp::ok() || state_ == TeleopState::SHUTDOWN) {
        init_timer_->cancel();
        return;
      }

      auto period = std::chrono::duration<double>(dt_);
      control_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&LeaderTeleopNode::control_loop, this));

      RCLCPP_INFO(get_logger(), "%s", log_rule().c_str());
      RCLCPP_INFO(get_logger(),
        " Leader Teleop C++ — Ready");
      RCLCPP_INFO(get_logger(),
        " Keys: c=current_ready, t=slow_sync, o=fast, z=init_pose");
      RCLCPP_INFO(get_logger(),
        "       s=pause, r=re-align, g/f=grav+/-, q=quit");
      if (!aligned) {
        RCLCPP_ERROR(get_logger(),
          "[ALIGN] Startup alignment failed closed. DXL torque is OFF; "
          "resolve the reported cause, then use 'r' to retry or 'q' to exit.");
      }
      RCLCPP_INFO(get_logger(), "%s\n", log_rule().c_str());

      // Cancel this one-shot timer
      init_timer_->cancel();
    });
}

LeaderTeleopNode::~LeaderTeleopNode() {
  shutdown_requested_ = true;
  if (bus_) {
    bus_->torque_off(arm_.dxl_ids);
  }
  if (csv_file_.is_open()) {
    csv_file_.flush();
    csv_file_.close();
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Parameter loading
// ═══════════════════════════════════════════════════════════════════════════════

void LeaderTeleopNode::declare_and_load_params() {
  auto p = [this](const std::string& n, auto def) {
    this->declare_parameter(n, def);
  };

  p("side", std::string("right"));
  p("control_hz", 500);
  p("log_period_sec", 1.0);
  p("color_log", true);
  p("feedback_source", std::string("contact_observer"));
  p("contact_observation_topic", std::string("/contact_observer/right/observation"));
  p("contact_observation_stale_timeout", 0.020);
  p("contact_observation_clock_future_tolerance", 0.002);
  p("follower_joint_stale_timeout", 0.050);
  p("contact_state_topic", std::string("~/contact_state"));
  p("use_pre_contact_phase", false);
  p("contact_phase_topic", std::string("~/contact_phase"));
  p("dxl_device", std::string("/dev/ttyUSB0"));
  p("dxl_baud", 2000000);

  p("left_dxl_ids", std::vector<int64_t>{1, 2, 3, 4, 5, 6});
  p("right_dxl_ids", std::vector<int64_t>{11, 12, 13, 14, 15, 16});
  p("left_zero_ticks", std::vector<int64_t>{2048, 2048, 2048, 2048, 2048, 2048});
  p("right_zero_ticks", std::vector<int64_t>{2048, 2048, 2048, 2048, 2048, 2048});
  p("left_joint_signs", std::vector<double>{1, 1, 1, 1, 1, 1});
  p("right_joint_signs", std::vector<double>{1, 1, 1, 1, 1, 1});
  p("left_offset_rad", std::vector<double>{0, 0, 0, 0, 0, 0});
  p("right_offset_rad", std::vector<double>{0, 0, 0, 0, 0, 0});

  p("torque_constant_Nm_per_A", std::vector<double>{2.020, 2.020, 1.783, 1.136, 1.783, 1.136});
  p("current_unit_A", std::vector<double>{0.00269, 0.00269, 0.00269, 0.001, 0.00269, 0.001});
  p("max_current_unit", std::vector<int64_t>{2047, 2047, 1193, 910, 1193, 910});

  p("use_gravity_comp", true);
  p("grav_gain", std::vector<double>{0.3, 0.3, 0.3, 0.1, 0.1, 0.3});
  p("grav_sync_scale", 4.5);
  p("grav_sync_scale_per_joint", std::vector<double>{});
  p("grav_ramp_sec", 1.0);

  p("leader_urdf_path", std::string(""));
  p("follower_urdf_path", std::string(""));

  p("left_leader_joint_names", std::vector<std::string>{
    "leader_left_joint_1","leader_left_joint_2","leader_left_joint_3",
    "leader_left_joint_4","leader_left_joint_5","leader_left_joint_6"});
  p("right_leader_joint_names", std::vector<std::string>{
    "leader_right_joint_1","leader_right_joint_2","leader_right_joint_3",
    "leader_right_joint_4","leader_right_joint_5","leader_right_joint_6"});
  p("left_follower_joint_names", std::vector<std::string>{
    "left_joint_1","left_joint_2","left_joint_3",
    "left_joint_4","left_joint_5","left_joint_6"});
  p("right_follower_joint_names", std::vector<std::string>{
    "right_joint_1","right_joint_2","right_joint_3",
    "right_joint_4","right_joint_5","right_joint_6"});
  // Doosan /dsrXX/joint_states publishes generic "joint_1".."joint_6" (no side prefix),
  // unlike the Pinocchio dual-arm URDF names above — kept as a separate param.
  p("left_follower_state_joint_names", std::vector<std::string>{
    "joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"});
  p("right_follower_state_joint_names", std::vector<std::string>{
    "joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"});

  p("left_ee_frame", std::string("left_link_6"));
  p("right_ee_frame", std::string("right_link_6"));

  p("left_impedance_topic", std::string("/left_dsr_controller/task_space_command"));
  p("right_impedance_topic", std::string("/right_dsr_controller/task_space_command"));
  p("follower_command_publish_enabled", true);
  p("impedance_base_frame", std::string("base_link"));
  p("left_command_base_frame", std::string("left_base_link"));
  p("right_command_base_frame", std::string("right_base_link"));
  p("impedance_linear_speed_mm_s", 300.0);
  p("impedance_angular_speed_deg_s", 300.0);
  p("impedance_first_publish_clip_guard_mm", 20.0);
  p("intent_generator_enabled", true);
  p("intent_linear_natural_frequency_hz", 4.0);
  p("intent_angular_natural_frequency_hz", 4.0);
  p("intent_damping_ratio", 1.0);
  p("intent_max_linear_velocity_mm_s", 300.0);
  p("intent_max_linear_acceleration_mm_s2", 1000.0);
  p("intent_max_angular_velocity_deg_s", 300.0);
  p("intent_max_angular_acceleration_deg_s2", 720.0);

  p("left_workspace_min", std::vector<double>{0.00, -0.25, -0.50});
  p("left_workspace_max", std::vector<double>{0.50, 0.40, 0.40});
  p("right_workspace_min", std::vector<double>{0.00, -0.35, -0.37});
  p("right_workspace_max", std::vector<double>{0.67, 0.25, 0.20});

  p("feedback_gain_scale_contract", 0.0);
  p("use_jt_wrench_feedback", true);
  p("left_jt_wrench_topic", std::string("/left/F_e"));
  p("right_jt_wrench_topic", std::string("/right/F_e"));
  // Fallback /F_e topic: also subscribed so the wrench is received whether the
  // publisher uses the side-specific (/left/F_e) or generic (/F_e) topic. Empty disables.
  p("jt_wrench_fallback_topic", std::string("/F_e"));
  p("jt_wrench_fb_gain", std::vector<double>{0.02, 0.02, 0.0225, 0.035, 0.0225, 0.035});
  p("left_jt_wrench_fb_gain", std::vector<double>{});
  p("right_jt_wrench_fb_gain", std::vector<double>{});
  // Python reference uses scalar 1.0; C++ keeps a per-joint vector.
  p("jt_wrench_fb_clip", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
  p("jt_wrench_tare_N", 30);
  p("jt_wrench_stale_timeout", 0.5);
  p("use_ft_feedback", true);
  p("left_ft_topic", std::string("/aft_sensor1/wrench"));
  p("right_ft_topic", std::string("/aft_sensor2/wrench"));
  p("left_ft_frame", std::string("left_link_6"));
  p("right_ft_frame", std::string("right_link_6"));
  p("ft_fb_gain", std::vector<double>{0.02, 0.02, 0.0225, 0.035, 0.0225, 0.035});
  p("left_ft_fb_gain", std::vector<double>{});
  p("right_ft_fb_gain", std::vector<double>{});
  p("ft_fb_clip", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
  p("ft_tare_N", 30);
  p("ft_stale_timeout", 0.5);
  p("ft_feedback_wrench_sign", std::vector<double>{1, 1, 1, 1, 1, 1});
  p("use_ft_payload_gravity_comp", false);
  p("left_ft_payload_root_frame", std::string("left_hand_base_link"));
  p("right_ft_payload_root_frame", std::string("right_hand_base_link"));
  p("ft_payload_gravity_sign", std::vector<double>{1, 1, 1, 1, 1, 1});
  // Optional per-joint gravity torque smoothing. <=0 Hz is passthrough.
  p("tau_grav_lpf_cutoff_hz", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  // Optional per-joint output-torque smoothing. This smooths tau_fb only.
  p("tau_lpf_cutoff_hz", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  p("use_leader_damping", false);
  p("leader_damping_velocity_source", std::string("differentiate_pose"));
  p("leader_damping_gain", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  p("leader_damping_lpf_cutoff_hz", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  p("leader_damping_clip_Nm", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  p("tau_fb_deadband_Nm", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  p("tau_fb_slew_rate_Nm_s", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  p("tau_fb_motion_gate_enable", false);
  p("tau_fb_motion_gate_speed_source", std::string("ee_linear"));
  p("tau_fb_motion_gate_speed_low_m_s", 0.04);
  p("tau_fb_motion_gate_speed_high_m_s", 0.20);
  p("tau_fb_motion_gate_speed_low_rad_s", 0.15);
  p("tau_fb_motion_gate_speed_high_rad_s", 0.80);
  p("tau_fb_motion_gate_min_scale", 1.0);
  p("tau_fb_passivity_gate_enable", false);
  p("tau_fb_passivity_power_start_W", 0.0);
  p("tau_fb_passivity_power_full_W", 0.05);
  p("tau_fb_passivity_min_scale", 0.0);
  p("tau_fb_contact_gate_enable", false);
  p("tau_fb_contact_force_on_N", 4.0);
  p("tau_fb_contact_force_off_N", 2.0);
  p("tau_fb_contact_moment_on_Nm", 0.20);
  p("tau_fb_contact_moment_off_Nm", 0.08);
  p("tau_fb_contact_on_hold_ms", 30.0);
  p("tau_fb_contact_off_hold_ms", 120.0);
  p("tau_fb_contact_free_scale", 0.03);
  p("tau_fb_contact_ramp_up_ms", 30.0);
  p("tau_fb_contact_ramp_down_ms", 120.0);
  p("tau_fb_contact_speed_gate_enable", false);
  p("tau_fb_contact_speed_low_m_s", 0.04);
  p("tau_fb_contact_speed_high_m_s", 0.20);
  p("tau_fb_contact_force_on_fast_N", 8.0);
  p("tau_fb_contact_force_off_fast_N", 6.5);
  p("tau_fb_contact_moment_on_fast_Nm", 1.0);
  p("tau_fb_contact_moment_off_fast_Nm", 0.5);
  p("tau_fb_contact_on_max_joint_speed_rad_s", 0.0);
  p("tau_fb_contact_on_max_ee_speed_m_s", 0.0);
  p("tau_fb_contact_bias_enable", true);
  p("tau_fb_contact_bias_lpf_cutoff_hz", 0.3);
  p("tau_fb_contact_bias_update_max_ee_speed_m_s", 0.04);
  p("tau_fb_contact_bias_update_max_joint_speed_rad_s", 0.0);
  p("tau_fb_contact_bias_force_clip_N", 0.0);
  p("tau_fb_contact_stale_bias_reset_enable", false);
  p("tau_fb_contact_stale_bias_raw_force_max_N", 3.5);
  p("tau_fb_contact_stale_bias_residual_force_min_N", 4.0);
  p("tau_fb_contact_stale_bias_residual_force_max_N", 8.0);
  p("tau_fb_contact_stale_bias_speed_max_m_s", 0.08);
  p("tau_fb_contact_stale_bias_hold_ms", 500.0);

  p("left_follower_state_topic", std::string("/dsr01/joint_states"));
  p("right_follower_state_topic", std::string("/dsr02/joint_states"));

  p("leader_base_frame", std::string("leader_base"));
  p("left_leader_tip_frame", std::string("leader_left_link_6"));
  p("right_leader_tip_frame", std::string("leader_right_link_6"));

  p("jt_wrench_feedback_wrench_sign", std::vector<double>{1, 1, 1, 1, 1, 1});

  p("slow_sync_max_vel_deg_s", 1.0);
  p("slow_sync_capture_delay_s", 0.6);
  p("slow_sync_ready_hold_sec", 0.5);
  p("slow_sync_ready_hold_s", 0.5);  // legacy key, prefer slow_sync_ready_hold_sec
  p("slow_sync_ready_tol_deg", 2.0);
  p("keyboard_input_enabled", true);
  p("status_publish_hz", 10.0);
  p("status_topic", std::string("~/status"));

  p("left_follower_home_deg", std::vector<double>{-16.95, -17.39, -80.55, 87.56, -106.59, -8.31});
  p("right_follower_home_deg", std::vector<double>{-2.0, 48.0, 112.0, 12.0, -75.0, -0.14});
  this->declare_parameter<std::vector<double>>("left_follower_init_pose_deg", std::vector<double>{});
  this->declare_parameter<std::vector<double>>("right_follower_init_pose_deg", std::vector<double>{});
  p("follower_init_pose_duration_sec", 5.0);

  p("csv_log_enabled", true);
  p("csv_log_dir", std::string("logs"));
}

void LeaderTeleopNode::build_arm_config() {
  std::string side = get_parameter("side").as_string();
  arm_.side = side;

  int hz = get_parameter("control_hz").as_int();
  if (hz <= 0) {
    throw std::invalid_argument("control_hz must be > 0");
  }
  dt_ = 1.0 / static_cast<double>(hz);
  diagnostics_config_.log_period_sec = get_parameter("log_period_sec").as_double();
  diagnostics_config_.color_log = get_parameter("color_log").as_bool();
  hz_log_period_ = std::max(0.05, diagnostics_config_.log_period_sec);
  color_log_ = diagnostics_config_.color_log;
  feedback_source_ = get_parameter("feedback_source").as_string();

  auto to_arr6i = [](const std::vector<int64_t>& v) {
    std::array<int, 6> a{};
    for (int i = 0; i < 6 && i < (int)v.size(); ++i) a[i] = static_cast<int>(v[i]);
    return a;
  };
  auto to_vec6 = [](const std::vector<double>& v) {
    Vec6 out = Vec6::Zero();
    for (int i = 0; i < 6 && i < (int)v.size(); ++i) out[i] = v[i];
    return out;
  };
  auto to_vec6_with_fallback =
    [this, &to_vec6](const std::string& primary, const std::string& fallback) {
      auto primary_value = get_parameter(primary).as_double_array();
      if (primary_value.size() == 6) {
        return to_vec6(primary_value);
      }
      if (!primary_value.empty()) {
        RCLCPP_WARN(get_logger(),
          "Parameter %s must contain 6 values; using %s instead.",
          primary.c_str(), fallback.c_str());
      }
      return to_vec6(get_parameter(fallback).as_double_array());
    };
  auto to_arr6s = [](const std::vector<std::string>& v) {
    std::array<std::string, 6> a{};
    for (int i = 0; i < 6 && i < (int)v.size(); ++i) a[i] = v[i];
    return a;
  };

  arm_.dxl_ids = to_arr6i(get_parameter(side + "_dxl_ids").as_integer_array());
  arm_.zero_ticks = to_arr6i(get_parameter(side + "_zero_ticks").as_integer_array());
  arm_.joint_signs = to_vec6(get_parameter(side + "_joint_signs").as_double_array());
  arm_.offset_rad = to_vec6(get_parameter(side + "_offset_rad").as_double_array());
  arm_.leader_joint_names = to_arr6s(get_parameter(side + "_leader_joint_names").as_string_array());
  arm_.follower_joint_names = to_arr6s(get_parameter(side + "_follower_joint_names").as_string_array());
  arm_.follower_state_joint_names = to_arr6s(get_parameter(side + "_follower_state_joint_names").as_string_array());
  arm_.ee_frame = get_parameter(side + "_ee_frame").as_string();

  arm_.torque_constant = to_vec6(get_parameter("torque_constant_Nm_per_A").as_double_array());
  arm_.current_unit = to_vec6(get_parameter("current_unit_A").as_double_array());
  {
    auto mc = get_parameter("max_current_unit").as_integer_array();
    for (int i = 0; i < 6; ++i) arm_.max_current[i] = static_cast<int>(mc[i]);
  }

  use_gravity_comp_ = get_parameter("use_gravity_comp").as_bool();
  arm_.grav_gain = to_vec6(get_parameter("grav_gain").as_double_array());
  grav_sync_scale_ = Vec6::Constant(get_parameter("grav_sync_scale").as_double());
  {
    auto per_joint_scale = get_parameter("grav_sync_scale_per_joint").as_double_array();
    if (per_joint_scale.size() == 6) {
      grav_sync_scale_ = to_vec6(per_joint_scale);
    } else if (!per_joint_scale.empty()) {
      RCLCPP_WARN(get_logger(),
        "Parameter grav_sync_scale_per_joint must contain 6 values; using scalar grav_sync_scale instead.");
    }
  }
  grav_ramp_sec_ = get_parameter("grav_ramp_sec").as_double();
  tau_grav_lpf_cutoff_hz_ = to_vec6(get_parameter("tau_grav_lpf_cutoff_hz").as_double_array());
  for (int i = 0; i < 6; ++i) {
    if (tau_grav_lpf_cutoff_hz_[i] > 0.0) {
      double rc = 1.0 / (2.0 * M_PI * tau_grav_lpf_cutoff_hz_[i]);
      tau_grav_lpf_alpha_[i] = dt_ / (rc + dt_);
    } else {
      tau_grav_lpf_alpha_[i] = 1.0;
    }
  }

  // Impedance params
  if (side == "left") {
    impedance_topic_ = get_parameter("left_impedance_topic").as_string();
    impedance_base_frame_ = get_parameter("left_command_base_frame").as_string();
  } else {
    impedance_topic_ = get_parameter("right_impedance_topic").as_string();
    impedance_base_frame_ = get_parameter("right_command_base_frame").as_string();
  }
  follower_command_publish_enabled_ =
    get_parameter("follower_command_publish_enabled").as_bool();
  if (!follower_command_publish_enabled_) {
    RCLCPP_WARN(get_logger(),
      "[LEADER ONLY] follower PoseStamped publishing is DISABLED; "
      "raw/intent computation and CSV logging remain active");
  }
  impedance_workspace_frame_ = get_parameter("impedance_base_frame").as_string();

  double lin_mm = get_parameter("impedance_linear_speed_mm_s").as_double();
  double ang_deg = get_parameter("impedance_angular_speed_deg_s").as_double();
  impedance_lin_speed_m_s_ = std::max(1e-6, lin_mm / 1000.0);
  impedance_ang_speed_rad_s_ = std::max(1e-6, ang_deg * M_PI / 180.0);
  impedance_first_publish_clip_guard_m_ =
    std::max(0.0, get_parameter("impedance_first_publish_clip_guard_mm").as_double()) / 1000.0;

  IntentTrajectoryConfig intent_config;
  intent_config.enabled = get_parameter("intent_generator_enabled").as_bool();
  intent_config.linear_natural_frequency_hz =
    get_parameter("intent_linear_natural_frequency_hz").as_double();
  intent_config.angular_natural_frequency_hz =
    get_parameter("intent_angular_natural_frequency_hz").as_double();
  intent_config.damping_ratio =
    get_parameter("intent_damping_ratio").as_double();
  intent_config.max_linear_velocity_m_s =
    get_parameter("intent_max_linear_velocity_mm_s").as_double() / 1000.0;
  intent_config.max_linear_acceleration_m_s2 =
    get_parameter("intent_max_linear_acceleration_mm_s2").as_double() / 1000.0;
  intent_config.max_angular_velocity_rad_s =
    get_parameter("intent_max_angular_velocity_deg_s").as_double() * M_PI / 180.0;
  intent_config.max_angular_acceleration_rad_s2 =
    get_parameter("intent_max_angular_acceleration_deg_s2").as_double() * M_PI / 180.0;
  if (intent_config.max_linear_velocity_m_s >
        impedance_lin_speed_m_s_ + 1.0e-12 ||
      intent_config.max_angular_velocity_rad_s >
        impedance_ang_speed_rad_s_ + 1.0e-12) {
    throw std::invalid_argument(
      "intent velocity limits must not exceed final impedance slew limits");
  }
  intent_generator_.configure(intent_config);
  intent_generator_enabled_ = intent_config.enabled;
  RCLCPP_INFO(get_logger(),
    "[INTENT] %s | fn linear/angular %.2f/%.2f Hz | zeta %.2f | "
    "v/a %.0f/%.0f mm units",
    intent_generator_enabled_ ? "enabled" : "disabled",
    intent_config.linear_natural_frequency_hz,
    intent_config.angular_natural_frequency_hz,
    intent_config.damping_ratio,
    intent_config.max_linear_velocity_m_s * 1000.0,
    intent_config.max_linear_acceleration_m_s2 * 1000.0);

  auto ws_min = get_parameter(side + "_workspace_min").as_double_array();
  auto ws_max = get_parameter(side + "_workspace_max").as_double_array();
  workspace_min_ = Vec3(ws_min[0], ws_min[1], ws_min[2]);
  workspace_max_ = Vec3(ws_max[0], ws_max[1], ws_max[2]);

  // Wrench feedback source
  feedback_gain_scale_contract_ =
    get_parameter("feedback_gain_scale_contract").as_double();
  const bool gain_contract_valid = std::isfinite(feedback_gain_scale_contract_) &&
    (std::abs(feedback_gain_scale_contract_) <= 1.0e-9 ||
     std::abs(feedback_gain_scale_contract_ - 0.40) <= 1.0e-9 ||
     std::abs(feedback_gain_scale_contract_ - 1.00) <= 1.0e-9);
  if (!gain_contract_valid) {
    throw std::invalid_argument(
      "feedback_gain_scale_contract must be exactly 0.0, 0.40, or 1.00");
  }
  use_jt_wrench_fb_ = get_parameter("use_jt_wrench_feedback").as_bool();
  use_ft_sensor_feedback_ = false;
  use_contact_observer_fb_ = false;
  if (feedback_source_ == "off") {
    use_jt_wrench_fb_ = false;
  } else if (feedback_source_ == "contact_observer") {
    use_jt_wrench_fb_ = false;
    use_contact_observer_fb_ = true;
  } else if (feedback_source_ == "jt_wrench") {
    use_jt_wrench_fb_ = true;
  } else if (feedback_source_ == "ft") {
    use_jt_wrench_fb_ = false;
    use_ft_sensor_feedback_ = get_parameter("use_ft_feedback").as_bool();
  } else {
    RCLCPP_WARN(get_logger(),
      "Unknown feedback_source=%s. Disabling feedback.", feedback_source_.c_str());
    feedback_source_ = "off";
    use_jt_wrench_fb_ = false;
  }
  jt_wrench_fb_gain_ = to_vec6_with_fallback(side + "_jt_wrench_fb_gain", "jt_wrench_fb_gain");
  arm_.jt_wrench_fb_gain = jt_wrench_fb_gain_;
  jt_wrench_fb_gain_base_ = jt_wrench_fb_gain_;
  jt_wrench_fb_clip_ = to_vec6(get_parameter("jt_wrench_fb_clip").as_double_array());
  jt_wrench_tare_N_ = get_parameter("jt_wrench_tare_N").as_int();
  jt_wrench_stale_timeout_ = get_parameter("jt_wrench_stale_timeout").as_double();
  contact_observation_stale_timeout_ = std::max(
    0.001, get_parameter("contact_observation_stale_timeout").as_double());
  contact_observation_clock_future_tolerance_ = std::max(
    0.0, get_parameter("contact_observation_clock_future_tolerance").as_double());
  follower_joint_stale_timeout_ = std::max(
    0.001, get_parameter("follower_joint_stale_timeout").as_double());

  ft_fb_gain_ = to_vec6_with_fallback(side + "_ft_fb_gain", "ft_fb_gain");
  ft_fb_gain_base_ = ft_fb_gain_;
  ft_fb_clip_ = to_vec6(get_parameter("ft_fb_clip").as_double_array());
  ft_tare_N_ = get_parameter("ft_tare_N").as_int();
  ft_stale_timeout_ = get_parameter("ft_stale_timeout").as_double();
  ft_topic_ = get_parameter(side + "_ft_topic").as_string();
  ft_frame_name_ = get_parameter(side + "_ft_frame").as_string();
  use_ft_payload_gravity_comp_ = get_parameter("use_ft_payload_gravity_comp").as_bool();
  ft_payload_root_frame_ = get_parameter(side + "_ft_payload_root_frame").as_string();
  ft_payload_gravity_sign_ = to_vec6(get_parameter("ft_payload_gravity_sign").as_double_array());
  ft_feedback_wrench_sign_ = to_vec6(get_parameter("ft_feedback_wrench_sign").as_double_array());
  if (use_ft_sensor_feedback_) {
    jt_wrench_fb_clip_ = ft_fb_clip_;
  }

  auto ws_sign = get_parameter("jt_wrench_feedback_wrench_sign").as_double_array();
  for (int i = 0; i < 6; ++i) jt_wrench_sign_[i] = ws_sign[i];

  tau_lpf_cutoff_hz_ = to_vec6(get_parameter("tau_lpf_cutoff_hz").as_double_array());
  for (int i = 0; i < 6; ++i) {
    if (tau_lpf_cutoff_hz_[i] > 0.0) {
      double rc = 1.0 / (2.0 * M_PI * tau_lpf_cutoff_hz_[i]);
      tau_lpf_alpha_[i] = dt_ / (rc + dt_);
    } else {
      tau_lpf_alpha_[i] = 1.0;  // passthrough
    }
  }

  use_leader_damping_ = get_parameter("use_leader_damping").as_bool();
  leader_damping_velocity_source_ = get_parameter("leader_damping_velocity_source").as_string();
  std::transform(leader_damping_velocity_source_.begin(), leader_damping_velocity_source_.end(),
                 leader_damping_velocity_source_.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (leader_damping_velocity_source_ != "differentiate_pose" &&
      leader_damping_velocity_source_ != "dxl_vel") {
    RCLCPP_WARN(get_logger(),
      "Unknown leader_damping_velocity_source=%s. Falling back to differentiate_pose.",
      leader_damping_velocity_source_.c_str());
    leader_damping_velocity_source_ = "differentiate_pose";
  }
  leader_damping_use_dxl_velocity_ = (leader_damping_velocity_source_ == "dxl_vel");
  leader_damping_gain_ = to_vec6(get_parameter("leader_damping_gain").as_double_array());
  leader_damping_lpf_cutoff_hz_ =
    to_vec6(get_parameter("leader_damping_lpf_cutoff_hz").as_double_array());
  leader_damping_clip_Nm_ = to_vec6(get_parameter("leader_damping_clip_Nm").as_double_array());
  for (int i = 0; i < 6; ++i) {
    if (leader_damping_lpf_cutoff_hz_[i] > 0.0) {
      double rc = 1.0 / (2.0 * M_PI * leader_damping_lpf_cutoff_hz_[i]);
      leader_damping_lpf_alpha_[i] = dt_ / (rc + dt_);
    } else {
      leader_damping_lpf_alpha_[i] = 1.0;
    }
  }
  tau_fb_deadband_Nm_ = to_vec6(get_parameter("tau_fb_deadband_Nm").as_double_array());
  tau_fb_slew_rate_Nm_s_ = to_vec6(get_parameter("tau_fb_slew_rate_Nm_s").as_double_array());
  tau_fb_motion_gate_enable_ = get_parameter("tau_fb_motion_gate_enable").as_bool();
  tau_fb_motion_gate_speed_source_ =
    get_parameter("tau_fb_motion_gate_speed_source").as_string();
  std::transform(tau_fb_motion_gate_speed_source_.begin(), tau_fb_motion_gate_speed_source_.end(),
                 tau_fb_motion_gate_speed_source_.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (tau_fb_motion_gate_speed_source_ != "ee_linear" &&
      tau_fb_motion_gate_speed_source_ != "joint_max") {
    RCLCPP_WARN(get_logger(),
      "[FB] invalid tau_fb_motion_gate_speed_source='%s'. Fallback to 'ee_linear'.",
      tau_fb_motion_gate_speed_source_.c_str());
    tau_fb_motion_gate_speed_source_ = "ee_linear";
  }
  tau_fb_motion_gate_speed_low_m_s_ =
    std::max(0.0, get_parameter("tau_fb_motion_gate_speed_low_m_s").as_double());
  tau_fb_motion_gate_speed_high_m_s_ =
    std::max(0.0, get_parameter("tau_fb_motion_gate_speed_high_m_s").as_double());
  tau_fb_motion_gate_speed_low_rad_s_ =
    std::max(0.0, get_parameter("tau_fb_motion_gate_speed_low_rad_s").as_double());
  tau_fb_motion_gate_speed_high_rad_s_ =
    std::max(0.0, get_parameter("tau_fb_motion_gate_speed_high_rad_s").as_double());
  tau_fb_motion_gate_min_scale_ =
    std::clamp(get_parameter("tau_fb_motion_gate_min_scale").as_double(), 0.0, 1.0);
  tau_fb_passivity_gate_enable_ = get_parameter("tau_fb_passivity_gate_enable").as_bool();
  tau_fb_passivity_power_start_W_ =
    std::max(0.0, get_parameter("tau_fb_passivity_power_start_W").as_double());
  tau_fb_passivity_power_full_W_ =
    std::max(0.0, get_parameter("tau_fb_passivity_power_full_W").as_double());
  tau_fb_passivity_min_scale_ =
    std::clamp(get_parameter("tau_fb_passivity_min_scale").as_double(), 0.0, 1.0);
  if (tau_fb_passivity_power_full_W_ <= tau_fb_passivity_power_start_W_ + 1e-12) {
    tau_fb_passivity_power_full_W_ = tau_fb_passivity_power_start_W_;
  }
  tau_fb_contact_gate_enable_ = get_parameter("tau_fb_contact_gate_enable").as_bool();
  use_pre_contact_phase_ = get_parameter("use_pre_contact_phase").as_bool();
  tau_fb_contact_force_on_N_ =
    std::max(0.0, get_parameter("tau_fb_contact_force_on_N").as_double());
  tau_fb_contact_force_off_N_ =
    std::max(0.0, get_parameter("tau_fb_contact_force_off_N").as_double());
  tau_fb_contact_force_off_N_ =
    std::min(tau_fb_contact_force_off_N_, tau_fb_contact_force_on_N_);
  tau_fb_contact_moment_on_Nm_ =
    std::max(0.0, get_parameter("tau_fb_contact_moment_on_Nm").as_double());
  tau_fb_contact_moment_off_Nm_ =
    std::max(0.0, get_parameter("tau_fb_contact_moment_off_Nm").as_double());
  tau_fb_contact_moment_off_Nm_ =
    std::min(tau_fb_contact_moment_off_Nm_, tau_fb_contact_moment_on_Nm_);
  tau_fb_contact_on_hold_s_ =
    std::max(0.0, get_parameter("tau_fb_contact_on_hold_ms").as_double()) / 1000.0;
  tau_fb_contact_off_hold_s_ =
    std::max(0.0, get_parameter("tau_fb_contact_off_hold_ms").as_double()) / 1000.0;
  tau_fb_contact_free_scale_ =
    std::clamp(get_parameter("tau_fb_contact_free_scale").as_double(), 0.0, 1.0);
  tau_fb_contact_ramp_up_s_ =
    std::max(0.0, get_parameter("tau_fb_contact_ramp_up_ms").as_double()) / 1000.0;
  tau_fb_contact_ramp_down_s_ =
    std::max(0.0, get_parameter("tau_fb_contact_ramp_down_ms").as_double()) / 1000.0;
  tau_fb_contact_speed_gate_enable_ =
    get_parameter("tau_fb_contact_speed_gate_enable").as_bool();
  tau_fb_contact_speed_low_m_s_ =
    std::max(0.0, get_parameter("tau_fb_contact_speed_low_m_s").as_double());
  tau_fb_contact_speed_high_m_s_ =
    std::max(0.0, get_parameter("tau_fb_contact_speed_high_m_s").as_double());
  tau_fb_contact_force_on_fast_N_ =
    std::max(tau_fb_contact_force_on_N_,
      get_parameter("tau_fb_contact_force_on_fast_N").as_double());
  tau_fb_contact_force_off_fast_N_ =
    std::clamp(get_parameter("tau_fb_contact_force_off_fast_N").as_double(),
      tau_fb_contact_force_off_N_, tau_fb_contact_force_on_fast_N_);
  tau_fb_contact_moment_on_fast_Nm_ =
    std::max(tau_fb_contact_moment_on_Nm_,
      get_parameter("tau_fb_contact_moment_on_fast_Nm").as_double());
  tau_fb_contact_moment_off_fast_Nm_ =
    std::clamp(get_parameter("tau_fb_contact_moment_off_fast_Nm").as_double(),
      tau_fb_contact_moment_off_Nm_, tau_fb_contact_moment_on_fast_Nm_);
  tau_fb_contact_on_max_joint_speed_rad_s_ =
    std::max(0.0, get_parameter("tau_fb_contact_on_max_joint_speed_rad_s").as_double());
  tau_fb_contact_on_max_ee_speed_m_s_ =
    std::max(0.0, get_parameter("tau_fb_contact_on_max_ee_speed_m_s").as_double());
  tau_fb_contact_bias_enable_ = get_parameter("tau_fb_contact_bias_enable").as_bool();
  tau_fb_contact_bias_lpf_cutoff_hz_ =
    std::max(0.0, get_parameter("tau_fb_contact_bias_lpf_cutoff_hz").as_double());
  tau_fb_contact_bias_update_max_ee_speed_m_s_ =
    std::max(0.0, get_parameter("tau_fb_contact_bias_update_max_ee_speed_m_s").as_double());
  tau_fb_contact_bias_update_max_joint_speed_rad_s_ =
    std::max(0.0, get_parameter("tau_fb_contact_bias_update_max_joint_speed_rad_s").as_double());
  tau_fb_contact_bias_force_clip_N_ =
    std::max(0.0, get_parameter("tau_fb_contact_bias_force_clip_N").as_double());
  tau_fb_contact_stale_bias_reset_enable_ =
    get_parameter("tau_fb_contact_stale_bias_reset_enable").as_bool();
  tau_fb_contact_stale_bias_raw_force_max_N_ =
    std::max(0.0, get_parameter("tau_fb_contact_stale_bias_raw_force_max_N").as_double());
  tau_fb_contact_stale_bias_residual_force_min_N_ =
    std::max(0.0, get_parameter("tau_fb_contact_stale_bias_residual_force_min_N").as_double());
  tau_fb_contact_stale_bias_residual_force_max_N_ =
    std::max(tau_fb_contact_stale_bias_residual_force_min_N_,
      get_parameter("tau_fb_contact_stale_bias_residual_force_max_N").as_double());
  tau_fb_contact_stale_bias_speed_max_m_s_ =
    std::max(0.0, get_parameter("tau_fb_contact_stale_bias_speed_max_m_s").as_double());
  tau_fb_contact_stale_bias_hold_s_ =
    std::max(0.0, get_parameter("tau_fb_contact_stale_bias_hold_ms").as_double()) / 1000.0;
  if (tau_fb_contact_bias_lpf_cutoff_hz_ > 0.0) {
    const double rc = 1.0 / (2.0 * M_PI * tau_fb_contact_bias_lpf_cutoff_hz_);
    tau_fb_contact_bias_alpha_ = dt_ / (rc + dt_);
  } else {
    tau_fb_contact_bias_alpha_ = 1.0;
  }
  reset_contact_gate_state();

  // Slow sync
  slow_sync_config_.max_vel_deg_s = get_parameter("slow_sync_max_vel_deg_s").as_double();
  slow_sync_config_.ready_tol_deg = get_parameter("slow_sync_ready_tol_deg").as_double();
  slow_sync_config_.ready_hold_sec = get_parameter("slow_sync_ready_hold_sec").as_double();
  follower_init_pose_duration_sec_ =
    std::max(0.1, get_parameter("follower_init_pose_duration_sec").as_double());
  double legacy_ready_hold_s = get_parameter("slow_sync_ready_hold_s").as_double();
  if (std::abs(legacy_ready_hold_s - 0.5) > 1e-12 &&
      std::abs(slow_sync_config_.ready_hold_sec - 0.5) < 1e-12) {
    slow_sync_config_.ready_hold_sec = legacy_ready_hold_s;
    RCLCPP_WARN(get_logger(),
      "Parameter slow_sync_ready_hold_s is deprecated; use slow_sync_ready_hold_sec.");
  }
  slow_sync_max_vel_rad_s_ = slow_sync_config_.max_vel_deg_s * M_PI / 180.0;
  slow_sync_capture_delay_s_ = get_parameter("slow_sync_capture_delay_s").as_double();
  slow_sync_ready_hold_sec_ = slow_sync_config_.ready_hold_sec;
  slow_sync_ready_tol_rad_ = slow_sync_config_.ready_tol_deg * M_PI / 180.0;
  keyboard_input_enabled_ = get_parameter("keyboard_input_enabled").as_bool();
  status_publish_hz_ = std::max(1.0, get_parameter("status_publish_hz").as_double());
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Pinocchio init
// ═══════════════════════════════════════════════════════════════════════════════

static std::string resolve_urdf(const std::string& param_path,
                                const std::vector<std::string>& fallbacks) {
  if (!param_path.empty() && std::filesystem::exists(param_path)) return param_path;
  for (auto& f : fallbacks) {
    if (std::filesystem::exists(f)) return f;
  }
  throw std::runtime_error("URDF not found. Set leader_urdf_path / follower_urdf_path.");
}

void LeaderTeleopNode::init_pinocchio() {
  std::string leader_urdf = resolve_urdf(
    get_parameter("leader_urdf_path").as_string(),
    {
      "/home/vision/dualarm_ws/src/teleop/description/Dual_leader_Arm/urdf/Dual_leader_Arm2.urdf",
    });
  std::string follower_urdf = resolve_urdf(
    get_parameter("follower_urdf_path").as_string(),
    {
      "/home/vision/dualarm_ws/src/aidin_dsr_dualarm/aidin_dsr_dualarm_description/urdf/aidin_dsr_dualarm_aligned_hand.urdf",
    });

  RCLCPP_INFO(get_logger(), "[PIN] Leader URDF: %s", leader_urdf.c_str());
  RCLCPP_INFO(get_logger(), "[PIN] Follower URDF: %s", follower_urdf.c_str());

  pinocchio::urdf::buildModel(leader_urdf, model_leader_);
  data_leader_ = pinocchio::Data(model_leader_);

  pinocchio::urdf::buildModel(follower_urdf, model_follower_);
  data_follower_ = pinocchio::Data(model_follower_);

  // Map joint names → pinocchio indices for leader
  for (int i = 0; i < 6; ++i) {
    const auto& jname = arm_.leader_joint_names[i];
    if (!model_leader_.existJointName(jname)) {
      throw std::runtime_error("Leader joint not found: " + jname);
    }
    auto jid = model_leader_.getJointId(jname);
    idxq_leader_[i] = model_leader_.joints[jid].idx_q();
    idxv_leader_[i] = model_leader_.joints[jid].idx_v();
  }

  // Map joint names → pinocchio indices for follower
  for (int i = 0; i < 6; ++i) {
    const auto& jname = arm_.follower_joint_names[i];
    if (!model_follower_.existJointName(jname)) {
      throw std::runtime_error("Follower joint not found: " + jname);
    }
    auto jid = model_follower_.getJointId(jname);
    idxq_follower_[i] = model_follower_.joints[jid].idx_q();
    idxv_follower_[i] = model_follower_.joints[jid].idx_v();
  }

  q_full_leader_ = Eigen::VectorXd::Zero(model_leader_.nq);

  // Follower frames for impedance FK
  auto require_frame = [this](const pinocchio::Model& m, const std::string& name) -> int {
    if (!m.existFrame(name)) {
      throw std::runtime_error("Frame not found in follower URDF: " + name);
    }
    return static_cast<int>(m.getFrameId(name));
  };

  impedance_workspace_frame_id_ = require_frame(model_follower_, impedance_workspace_frame_);
  impedance_command_base_frame_id_ = require_frame(model_follower_, impedance_base_frame_);
  impedance_tip_frame_id_ = require_frame(model_follower_, arm_.ee_frame);

  // Follower base for JT wrench transform
  std::string foll_base = get_parameter("impedance_base_frame").as_string();
  if (model_follower_.existFrame(foll_base)) {
    follower_base_frame_id_ = static_cast<int>(model_follower_.getFrameId(foll_base));
  }

  // Leader frames for Jacobian
  std::string leader_base = get_parameter("leader_base_frame").as_string();
  std::string leader_tip = get_parameter(arm_.side + "_leader_tip_frame").as_string();
  if (model_leader_.existFrame(leader_base)) {
    leader_base_frame_id_ = static_cast<int>(model_leader_.getFrameId(leader_base));
  }
  if (model_leader_.existFrame(leader_tip)) {
    leader_tip_frame_id_ = static_cast<int>(model_leader_.getFrameId(leader_tip));
  }

  if (!ft_frame_name_.empty() && model_follower_.existFrame(ft_frame_name_)) {
    ft_sensor_frame_id_ = static_cast<int>(model_follower_.getFrameId(ft_frame_name_));
  } else if (model_follower_.existFrame(arm_.ee_frame)) {
    ft_sensor_frame_id_ = static_cast<int>(model_follower_.getFrameId(arm_.ee_frame));
    if (use_ft_sensor_feedback_) {
      RCLCPP_WARN(get_logger(),
        "[FT] frame '%s' not found in follower URDF. Falling back to ee_frame=%s.",
        ft_frame_name_.c_str(), arm_.ee_frame.c_str());
    }
  }

  setup_ft_payload_gravity_comp(follower_urdf);

  // Load follower init pose. Prefer the explicit init_pose key; keep
  // follower_home_deg as a legacy fallback for existing configs.
  {
    const std::string init_key = arm_.side + "_follower_init_pose_deg";
    const std::string home_key = arm_.side + "_follower_home_deg";
    auto init_deg = get_parameter(init_key).as_double_array();
    const bool use_legacy_home = init_deg.empty();
    const auto target_deg = use_legacy_home ? get_parameter(home_key).as_double_array() : init_deg;
    if (target_deg.size() != 6) {
      RCLCPP_WARN(get_logger(),
        "[INIT] %s has %zu values; expected 6. Missing joints keep 0 deg.",
        (use_legacy_home ? home_key : init_key).c_str(), target_deg.size());
    }
    for (int i = 0; i < 6 && i < static_cast<int>(target_deg.size()); ++i) {
      follower_init_pose_rad_[i] = target_deg[i] * M_PI / 180.0;
    }
    RCLCPP_INFO(get_logger(), "[INIT] follower init pose source=%s target(deg)= %s",
      (use_legacy_home ? home_key : init_key).c_str(),
      format_joint_deg(follower_init_pose_rad_).c_str());
    if (use_legacy_home) {
      RCLCPP_WARN(get_logger(),
        "[INIT] %s is deprecated for z init pose; prefer %s.",
        home_key.c_str(), init_key.c_str());
    }
  }

  // Compute workspace→command base transform
  if (impedance_workspace_frame_id_ != impedance_command_base_frame_id_) {
    Eigen::VectorXd q0 = Eigen::VectorXd::Zero(model_follower_.nq);
    pinocchio::forwardKinematics(model_follower_, data_follower_, q0);
    pinocchio::updateFramePlacements(model_follower_, data_follower_);
    auto oMws = data_follower_.oMf[impedance_workspace_frame_id_];
    auto oMcb = data_follower_.oMf[impedance_command_base_frame_id_];
    workspace_M_command_ = oMws.inverse() * oMcb;
    has_workspace_M_command_ = true;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  DXL init
// ═══════════════════════════════════════════════════════════════════════════════

void LeaderTeleopNode::init_dxl() {
  std::string dev = get_parameter("dxl_device").as_string();
  int baud = get_parameter("dxl_baud").as_int();

  bus_ = std::make_unique<DxlBus>(dev, baud);
  if (!bus_->force_status_return(arm_.dxl_ids, 2, 0)) {
    throw std::runtime_error("DXL status-return setup failed: " + bus_->last_error());
  }

  if (!bus_->ping_all(arm_.dxl_ids)) {
    throw std::runtime_error("DXL ping failed for one or more motors");
  }
  bus_->register_ids(arm_.dxl_ids);

  // Start with torque off. ALIGN will explicitly enter position mode, and
  // gravity-comp current mode should only start after the user presses 'c'.
  if (!bus_->torque_off(arm_.dxl_ids)) {
    throw std::runtime_error("DXL initial torque-off failed: " + bus_->last_error());
  }
  dxl_current_mode_ = false;
  RCLCPP_INFO(get_logger(), "[DXL] Initialized with torque OFF");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  ROS I/O
// ═══════════════════════════════════════════════════════════════════════════════

void LeaderTeleopNode::init_ros_io() {
  pub_impedance_ = create_publisher<geometry_msgs::msg::PoseStamped>(impedance_topic_, 10);
  if (!use_contact_observer_fb_) {
    const std::string contact_state_topic = get_parameter("contact_state_topic").as_string();
    pub_contact_state_ = create_publisher<std_msgs::msg::Int32>(contact_state_topic, 10);
    RCLCPP_INFO(get_logger(), "[ROS] Publishing legacy contact state: %s", contact_state_topic.c_str());
    if (use_pre_contact_phase_) {
      const std::string contact_phase_topic = get_parameter("contact_phase_topic").as_string();
      pub_contact_phase_ = create_publisher<std_msgs::msg::Int32>(contact_phase_topic, 10);
      RCLCPP_INFO(get_logger(), "[ROS] Publishing legacy contact phase: %s", contact_phase_topic.c_str());
    }
  }

  // Dedicated reentrant group: keeps follower/JT-wrench callbacks runnable
  // on another executor thread even while the align timer callback blocks.
  sub_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  // Services only enqueue commands. Keeping them outside the continuously-ready
  // 500 Hz control timer's default MutuallyExclusive group prevents service
  // starvation when a DXL read is slow.
  control_service_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = sub_callback_group_;

  std::string follower_topic;
  if (arm_.side == "left") {
    follower_topic = get_parameter("left_follower_state_topic").as_string();
  } else {
    follower_topic = get_parameter("right_follower_state_topic").as_string();
  }
  sub_follower_ = create_subscription<sensor_msgs::msg::JointState>(
    follower_topic, 50,
    std::bind(&LeaderTeleopNode::follower_joint_cb, this, std::placeholders::_1),
    sub_opts);

  if (use_jt_wrench_fb_) {
    std::string wrench_topic;
    if (arm_.side == "left") {
      wrench_topic = get_parameter("left_jt_wrench_topic").as_string();
    } else {
      wrench_topic = get_parameter("right_jt_wrench_topic").as_string();
    }
    sub_jt_wrench_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      wrench_topic, 50,
      std::bind(&LeaderTeleopNode::jt_wrench_cb, this, std::placeholders::_1),
      sub_opts);
    RCLCPP_INFO(get_logger(), "[ROS] Subscribing JT wrench: %s", wrench_topic.c_str());

    // Also subscribe to the generic fallback /F_e (same callback). Whichever
    // topic publishes feeds jt_wrench_raw_ — so it works with either /left/F_e
    // or /F_e. Skipped if empty or identical to the primary topic.
    std::string fallback_topic = get_parameter("jt_wrench_fallback_topic").as_string();
    if (!fallback_topic.empty() && fallback_topic != wrench_topic) {
      sub_jt_wrench_fallback_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        fallback_topic, 50,
        std::bind(&LeaderTeleopNode::jt_wrench_cb, this, std::placeholders::_1),
        sub_opts);
      RCLCPP_INFO(get_logger(), "[ROS] Subscribing JT wrench fallback: %s", fallback_topic.c_str());
    }
  }

  if (use_contact_observer_fb_) {
    const std::string topic = get_parameter("contact_observation_topic").as_string();
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.best_effort();
    qos.durability_volatile();
    sub_contact_observation_ =
      create_subscription<contact_observer_msgs::msg::ContactObservation>(
        topic, qos,
        std::bind(&LeaderTeleopNode::contact_observation_cb, this, std::placeholders::_1),
        sub_opts);
    RCLCPP_INFO(get_logger(),
      "[ROS] Subscribing canonical contact observation: %s stale_timeout=%.1f ms",
      topic.c_str(), contact_observation_stale_timeout_ * 1000.0);
  }

  if (use_ft_sensor_feedback_) {
    sub_ft_sensor_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      ft_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LeaderTeleopNode::ft_sensor_cb, this, std::placeholders::_1),
      sub_opts);
    RCLCPP_INFO(get_logger(), "[ROS] Subscribing FT sensor: %s frame=%s payload_comp=%s",
      ft_topic_.c_str(), ft_frame_name_.c_str(), use_ft_payload_gravity_comp_ ? "ON" : "OFF");
  }

  const std::string status_topic = get_parameter("status_topic").as_string();
  pub_status_ = create_publisher<std_msgs::msg::String>(status_topic, 10);
  const std::array<std::pair<const char*, char>, 7> commands{{
    {"current", 'c'}, {"slow", 't'}, {"fast", 'o'}, {"init_pose", 'z'},
    {"pause", 's'}, {"realign", 'r'}, {"shutdown", 'q'}}};
  for (const auto& command : commands) {
    control_services_.push_back(create_service<std_srvs::srv::Trigger>(
      std::string("~/command/") + command.first,
      [this, key = command.second](
          const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        handle_control_service(key, request, response);
      }, rmw_qos_profile_services_default, control_service_callback_group_));
  }
  RCLCPP_INFO(get_logger(),
    "[ROS] Teleop status=%s at %.1f Hz from control loop; "
    "services=~/command/{current,slow,fast,init_pose,pause,realign,shutdown}",
    status_topic.c_str(), status_publish_hz_);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Subscriber callbacks
// ═══════════════════════════════════════════════════════════════════════════════

void LeaderTeleopNode::follower_joint_cb(const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (msg->position.size() < 6) return;

  // Map joint names → 6-DOF array
  Vec6 q = Vec6::Zero();
  int found = 0;
  for (int i = 0; i < 6; ++i) {
    for (size_t j = 0; j < msg->name.size(); ++j) {
      if (msg->name[j] == arm_.follower_state_joint_names[i]) {
        q[i] = msg->position[j];
        ++found;
        break;
      }
    }
  }
  if (found < 6 || !q.allFinite()) return;

  std::lock_guard<std::mutex> lock(follower_mtx_);
  follower_joint_rad_ = q;
  follower_joint_receive_steady_ = std::chrono::steady_clock::now();
  follower_joint_receive_steady_valid_ = true;
}

void LeaderTeleopNode::jt_wrench_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() < 6) return;
  std::lock_guard<std::mutex> lock(jt_wrench_mtx_);
  for (int i = 0; i < 6; ++i) jt_wrench_raw_[i] = msg->data[i];
  jt_wrench_stamp_ = now().seconds();
}

void LeaderTeleopNode::contact_observation_cb(
    const contact_observer_msgs::msg::ContactObservation::SharedPtr msg) {
  Vec6 wrench = Vec6::Zero();
  bool finite = true;
  for (int i = 0; i < 6; ++i) {
    wrench[i] = msg->contact_wrench[static_cast<std::size_t>(i)];
    finite = finite && std::isfinite(wrench[i]);
  }

  const double source_stamp =
    static_cast<double>(msg->header.stamp.sec) +
    static_cast<double>(msg->header.stamp.nanosec) * 1.0e-9;
  const bool state_in_range =
    msg->contact_state == contact_observer_msgs::msg::ContactObservation::FREE ||
    msg->contact_state == contact_observer_msgs::msg::ContactObservation::CONTACT;
  const bool frame_matches = msg->header.frame_id == impedance_base_frame_;
  if (!frame_matches) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[CONTACT OBSERVER] frame mismatch: received='%s' expected='%s'",
      msg->header.frame_id.c_str(), impedance_base_frame_.c_str());
  }

  const auto receive_steady = std::chrono::steady_clock::now();
  const double receive_ros = now().seconds();
  std::lock_guard<std::mutex> lock(contact_observation_mtx_);
  if (!std::isfinite(source_stamp) || source_stamp <= 0.0) {
    contact_observation_valid_ = false;
    return;
  }
  const bool first_source = !contact_observation_receive_steady_valid_;
  const bool source_advanced =
    msg->source_sequence > contact_observation_source_sequence_ &&
    source_stamp > contact_observation_source_stamp_;
  const bool same_source =
    msg->source_sequence == contact_observation_source_sequence_ &&
    std::abs(source_stamp - contact_observation_source_stamp_) <= 1.0e-9;
  const bool source_restarted =
    msg->source_sequence <= contact_observation_source_sequence_ &&
    source_stamp > contact_observation_source_stamp_ + 1.0e-9;
  if (!first_source && !source_advanced && !same_source && !source_restarted) {
    // Do not let an older/reordered DDS sample replace the latest source.
    return;
  }
  if (!finite) wrench.setZero();
  // Same-source publications may carry a valid/model_ready transition when
  // the controller source becomes stale.  Update the safety payload on every
  // accepted publication; header source age independently prevents a stale
  // sequence from being kept alive by repeated DDS traffic.
  contact_observation_wrench_ = wrench;
  contact_observation_receive_stamp_ = receive_ros;
  contact_observation_source_stamp_ = source_stamp;
  contact_observation_receive_steady_ = receive_steady;
  contact_observation_receive_steady_valid_ = true;
  contact_observation_source_sequence_ = msg->source_sequence;
  contact_observation_prediction_sequence_ = msg->prediction_sequence;
  contact_observation_state_ = state_in_range ? msg->contact_state : 0;
  contact_observation_model_ready_ = msg->model_ready;
  contact_observation_score_n_ = msg->contact_score;
  contact_observation_prediction_age_ms_ = msg->prediction_age_ms;
  contact_observation_latency_ms_ = msg->observer_latency_ms;
  const bool diagnostics_finite =
    std::isfinite(contact_observation_score_n_) &&
    std::isfinite(contact_observation_prediction_age_ms_) &&
    std::isfinite(contact_observation_latency_ms_);
  contact_observation_valid_ =
    msg->valid && msg->model_ready && finite && diagnostics_finite &&
    state_in_range && frame_matches;
}

void LeaderTeleopNode::ft_sensor_cb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  Vec6 w;
  w << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
       msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
  std::lock_guard<std::mutex> lock(ft_sensor_mtx_);
  ft_sensor_raw_ = w;
  ft_sensor_stamp_ = now().seconds();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Alignment
// ═══════════════════════════════════════════════════════════════════════════════

void LeaderTeleopNode::enter_dxl_fault(const std::string& reason) {
  const std::string primary_reason = reason.empty() ? "unknown DXL error" : reason;
  const bool init_pose_cancelled = init_pose_in_progress_.exchange(false);
  if (init_pose_cancelled) {
    init_pose_phase_ = InitPosePhase::IDLE;
    init_pose_reached_ = false;
    init_pose_verified_ = false;
  }
  dxl_fault_active_ = true;
  dxl_fault_reason_ = primary_reason;
  dxl_degraded_read_streak_ = 0;
  aligned_once_ = false;
  dxl_current_mode_ = false;
  grav_scale_.setZero();
  grav_scale_target_.setZero();
  state_ = TeleopState::INIT;
  last_control_message_ = "DXL fault: " + primary_reason +
    "; torque OFF, REALIGN required";
  if (init_pose_cancelled) {
    last_control_message_ += "; active INIT POSE cancelled";
  }

  const bool torque_off_ok = bus_ && bus_->torque_off(arm_.dxl_ids);
  RCLCPP_ERROR(get_logger(),
    "[DXL SAFETY] %s | init_pose_cancelled=%s | torque_off=%s%s%s",
    primary_reason.c_str(), init_pose_cancelled ? "true" : "false",
    torque_off_ok ? "OK" : "FAILED",
    (!torque_off_ok && bus_) ? " | " : "",
    (!torque_off_ok && bus_) ? bus_->last_error().c_str() : "");
  publish_status();
}

bool LeaderTeleopNode::align_leader_to_follower() {
  state_ = TeleopState::ALIGN;
  aligned_once_ = false;
  dxl_current_mode_ = false;
  publish_status();
  RCLCPP_INFO(get_logger(), "%s waiting follower joint_states | side=%s",
    state_tag("ALIGN").c_str(), arm_.side.c_str());

  // Wait up to 5s for a fresh follower sample (subscriber callbacks run in a
  // separate Reentrant callback group while this function waits).
  std::string follower_reason;
  for (int i = 0; i < 500; ++i) {
    if (!rclcpp::ok()) {
      shutdown_requested_ = true;
      bus_->torque_off(arm_.dxl_ids);
      return false;
    }
    if (poll_shutdown_key("align_wait_follower")) {
      return false;
    }
    if (follower_joint_ready_for_teleop(follower_reason)) break;
    if (i % 50 == 0) publish_status();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (!follower_joint_ready_for_teleop(follower_reason)) {
    enter_dxl_fault("ALIGN blocked: " + follower_reason);
    return false;
  }

  Vec6 target_leader;
  {
    std::lock_guard<std::mutex> lock(follower_mtx_);
    // follower→leader: q_leader = S^{-1} * (q_follower - offset)
    const Vec6 q_foll = follower_joint_rad_.value();
    target_leader = arm_.joint_signs.cwiseInverse().cwiseProduct(q_foll - arm_.offset_rad);
  }

  // Safe phased position-mode transition.  Keep every motor torque-off while
  // changing mode and profile.  These leader DXLs mirror Goal Position to
  // Present Position while torque is off, so enable torque first (which holds
  // the current position), then write and verify the alignment target.
  if (!bus_->prepare_operating_mode(arm_.dxl_ids, 3)) {
    enter_dxl_fault(
      "ALIGN torque-off/position-mode preparation failed: " + bus_->last_error());
    return false;
  }

  // Set profile: ~15 deg/s velocity, ~100 deg/s^2 acceleration.
  constexpr double kAlignVelocityDegS = 15.0;
  if (!bus_->set_profile_deg(arm_.dxl_ids, kAlignVelocityDegS, 100.0)) {
    enter_dxl_fault("ALIGN profile setup failed: " + bus_->last_error());
    return false;
  }

  if (!bus_->torque_on(arm_.dxl_ids)) {
    enter_dxl_fault("ALIGN verified torque-on failed: " + bus_->last_error());
    return false;
  }

  // Convert target_leader (rad) → ticks, then send after verified torque-on.
  std::map<int, int32_t> ticks_map;
  for (int i = 0; i < 6; ++i) {
    int32_t tick = static_cast<int32_t>(std::round(
      arm_.joint_signs[i] * target_leader[i] * RAD_TO_TICKS)) + arm_.zero_ticks[i];
    ticks_map[arm_.dxl_ids[i]] = tick;
  }
  if (!bus_->write_goal_positions_verified(ticks_map)) {
    enter_dxl_fault("ALIGN verified goal write failed: " + bus_->last_error());
    return false;
  }

  constexpr double kAlignToleranceDeg = 5.0;
  const double tol_rad = kAlignToleranceDeg * M_PI / 180.0;
  Vec6 current_leader = read_leader_positions();
  if (!last_leader_position_read_fresh_) {
    enter_dxl_fault("ALIGN initial leader position read was not fresh: " + bus_->last_error());
    return false;
  }
  const double initial_max_err_rad =
    (target_leader - current_leader).cwiseAbs().maxCoeff();
  log_align_status(
    current_leader,
    target_leader,
    initial_max_err_rad,
    tol_rad,
    "START");
  // A fixed 6 s timeout was shorter than a valid 100+ degree move at 15 deg/s.
  // Size the timeout from the measured travel, with acceleration/settling margin.
  const double initial_max_err_deg = initial_max_err_rad * 180.0 / M_PI;
  const double align_timeout_s = std::clamp(
    initial_max_err_deg / kAlignVelocityDegS + 3.0, 6.0, 20.0);
  RCLCPP_INFO(get_logger(),
    "%s moving leader in position mode | timeout=%.1fs (initial max_err=%.2fdeg)",
    state_tag("ALIGN").c_str(), align_timeout_s, initial_max_err_deg);

  // Wait for convergence (check position periodically)
  double last_align_log_t = 0.0;
  bool reached = initial_max_err_rad < tol_rad;
  int consecutive_unfresh_reads = 0;
  double final_max_err_rad = initial_max_err_rad;
  const auto align_deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(align_timeout_s));
  while (!reached && std::chrono::steady_clock::now() < align_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    if (!rclcpp::ok()) {
      shutdown_requested_ = true;
      bus_->torque_off(arm_.dxl_ids);
      return false;
    }
    if (poll_shutdown_key("align_move")) {
      return false;
    }

    auto pos = bus_->read_positions();
    if (!bus_->last_read_all_fresh()) {
      ++consecutive_unfresh_reads;
      if (consecutive_unfresh_reads >= 10) {
        enter_dxl_fault(
          "ALIGN lost fresh leader positions for 10 consecutive reads: " +
          bus_->last_error());
        return false;
      }
      continue;
    }
    consecutive_unfresh_reads = 0;
    double max_err = 0.0;
    Vec6 q_now = current_leader;
    for (int i = 0; i < 6; ++i) {
      auto it = pos.find(arm_.dxl_ids[i]);
      if (it == pos.end()) continue;
      double current_rad = arm_.joint_signs[i] *
        (static_cast<double>(it->second) - arm_.zero_ticks[i]) * TICKS_TO_RAD;
      q_now[i] = current_rad;
      max_err = std::max(max_err, std::abs(current_rad - target_leader[i]));
    }
    current_leader = q_now;
    final_max_err_rad = max_err;
    double t = now().seconds();
    if (last_align_log_t <= 0.0 || (t - last_align_log_t) >= hz_log_period_) {
      last_align_log_t = t;
      bool ok = max_err < tol_rad;
      std::string status = ok ? std::string("OK") : std::string("MOVING");
      log_align_status(q_now, target_leader, max_err, tol_rad, status);
      if (ok) {
        RCLCPP_INFO(get_logger(), "%s press 'c' for CURRENT MODE", state_tag("ALIGN").c_str());
      }
      publish_status();
    }
    reached = max_err < tol_rad;
  }

  if (!reached) {
    std::ostringstream reason;
    reason << std::fixed << std::setprecision(2)
           << "ALIGN timeout: max error " << final_max_err_rad * 180.0 / M_PI
           << " deg exceeds " << kAlignToleranceDeg << " deg tolerance";
    log_align_status(
      current_leader, target_leader, final_max_err_rad, tol_rad, "FAILED");
    enter_dxl_fault(reason.str());
    return false;
  }

  // The last verified fresh sample becomes the initial state.
  q_leader_ = current_leader;
  q_leader_last_ = q_leader_;
  for (int i = 0; i < 6; ++i) {
    q_full_leader_[idxq_leader_[i]] = q_leader_[i];
  }

  // Keep position mode holding the aligned pose. Gravity-comp current mode is
  // intentionally delayed until the user presses 'c', matching the Python node.
  dxl_current_mode_ = false;
  grav_scale_.setZero();
  grav_scale_target_.setZero();

  state_ = TeleopState::MAIN_IDLE;
  aligned_once_ = true;
  dxl_fault_active_ = false;
  dxl_fault_reason_.clear();
  dxl_degraded_read_streak_ = 0;
  last_control_message_ = "ALIGN 완료";
  RCLCPP_INFO(get_logger(), "%s", blue("[ALIGN] Done. State -> IDLE").c_str());
  RCLCPP_INFO(get_logger(), "%s press 'c' for CURRENT MODE", state_tag("ALIGN").c_str());
  publish_status();
  return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Leader read
// ═══════════════════════════════════════════════════════════════════════════════

Vec6 LeaderTeleopNode::read_leader_positions() {
  Vec6 q = q_leader_last_;
  leader_dxl_velocity_rad_s_.setZero();
  last_leader_position_read_fresh_ = false;

  if (leader_damping_use_dxl_velocity_) {
    auto state_raw = bus_->read_position_velocity();
    ++dxl_read_count_;
    ++vel_read_count_;
    if (bus_->last_read_all_fresh()) ++dxl_fresh_count_;
    if (bus_->last_velocity_read_all_fresh()) ++vel_fresh_count_;
    dxl_fallback_count_ += bus_->last_fallback_count();
    dxl_missing_count_ += bus_->last_missing_count();
    vel_fallback_count_ += bus_->last_velocity_fallback_count();
    vel_missing_count_ += bus_->last_velocity_missing_count();
    last_leader_position_read_fresh_ = bus_->last_read_all_fresh();

    for (int i = 0; i < 6; ++i) {
      auto it = state_raw.find(arm_.dxl_ids[i]);
      if (it == state_raw.end()) continue;
      int32_t ticks = it->second.position - arm_.zero_ticks[i];
      q[i] = arm_.joint_signs[i] * static_cast<double>(ticks) * TICKS_TO_RAD;
      leader_dxl_velocity_rad_s_[i] =
        arm_.joint_signs[i] * static_cast<double>(it->second.velocity) * DXL_VELOCITY_UNIT_RAD_S;
    }
  } else {
    auto pos_raw = bus_->read_positions();
    ++dxl_read_count_;
    ++vel_read_count_;
    if (bus_->last_read_all_fresh()) {
      ++dxl_fresh_count_;
      ++vel_fresh_count_;
    }
    dxl_fallback_count_ += bus_->last_fallback_count();
    dxl_missing_count_ += bus_->last_missing_count();
    vel_fallback_count_ += bus_->last_fallback_count();
    vel_missing_count_ += bus_->last_missing_count();
    last_leader_position_read_fresh_ = bus_->last_read_all_fresh();

    for (int i = 0; i < 6; ++i) {
      auto it = pos_raw.find(arm_.dxl_ids[i]);
      if (it == pos_raw.end()) continue;
      int32_t ticks = it->second - arm_.zero_ticks[i];
      q[i] = arm_.joint_signs[i] * static_cast<double>(ticks) * TICKS_TO_RAD;
    }
  }

  // Glitch protection: if all joints read zero but last was nonzero, hold
  bool all_zero = q.cwiseAbs().maxCoeff() < 1e-6;
  bool last_nonzero = q_leader_last_.cwiseAbs().maxCoeff() > 1e-6;
  if (all_zero && last_nonzero && state_ != TeleopState::SHUTDOWN) {
    leader_dxl_velocity_rad_s_.setZero();
    return q_leader_last_;
  }

  q_leader_last_ = q;
  return q;
}

Vec6 LeaderTeleopNode::compute_leader_damping_torque(const Vec6& q_leader) {
  const double t = now().seconds();
  Vec6 dq_raw = Vec6::Zero();

  if (leader_damping_use_dxl_velocity_) {
    dq_raw = leader_dxl_velocity_rad_s_;
  } else {
    if (leader_dq_prev_init_) {
      const double measured_dt = t - leader_dq_prev_t_;
      if (measured_dt > 1e-6) {
        dq_raw = (q_leader - leader_dq_prev_q_) / measured_dt;
      }
    }
    leader_dq_prev_q_ = q_leader;
    leader_dq_prev_t_ = t;
    leader_dq_prev_init_ = true;
  }

  if (!leader_dq_lpf_init_) {
    leader_dq_lpf_state_ = dq_raw;
    leader_dq_lpf_init_ = true;
  } else {
    for (int i = 0; i < 6; ++i) {
      leader_dq_lpf_state_[i] +=
        leader_damping_lpf_alpha_[i] * (dq_raw[i] - leader_dq_lpf_state_[i]);
    }
  }
  leader_dq_ = leader_dq_lpf_state_;

  if (!use_leader_damping_) {
    last_tau_damp_.setZero();
    return Vec6::Zero();
  }

  Vec6 tau_damp = -leader_damping_gain_.cwiseProduct(leader_dq_);
  for (int i = 0; i < 6; ++i) {
    const double lim = leader_damping_clip_Nm_[i];
    if (lim > 0.0) {
      tau_damp[i] = std::clamp(tau_damp[i], -lim, lim);
    }
  }
  last_tau_damp_ = tau_damp;
  return tau_damp;
}

double LeaderTeleopNode::compute_leader_tip_linear_speed_m_s() {
  if (leader_tip_frame_id_ < 0 || model_leader_.nv <= 0) {
    return 0.0;
  }

  Eigen::VectorXd v_full = Eigen::VectorXd::Zero(model_leader_.nv);
  for (int i = 0; i < 6; ++i) {
    v_full[idxv_leader_[i]] = leader_dq_[i];
  }

  pinocchio::forwardKinematics(model_leader_, data_leader_, q_full_leader_);
  pinocchio::computeJointJacobians(model_leader_, data_leader_, q_full_leader_);
  pinocchio::updateFramePlacements(model_leader_, data_leader_);

  Eigen::MatrixXd J_world(6, model_leader_.nv);
  J_world.setZero();
  pinocchio::getFrameJacobian(model_leader_, data_leader_,
    static_cast<pinocchio::FrameIndex>(leader_tip_frame_id_),
    pinocchio::WORLD, J_world);

  const Vec3 tip_linear_velocity_world = J_world.topRows(3) * v_full;
  return tip_linear_velocity_world.norm();
}

void LeaderTeleopNode::clamp_contact_bias() {
  if (tau_fb_contact_bias_force_clip_N_ > 0.0) {
    const double force_norm = tau_fb_contact_bias_.head<3>().norm();
    if (force_norm > tau_fb_contact_bias_force_clip_N_ && force_norm > 1e-12) {
      tau_fb_contact_bias_.head<3>() *= tau_fb_contact_bias_force_clip_N_ / force_norm;
    }
  }
}

void LeaderTeleopNode::reset_contact_gate_state() {
  tau_fb_contact_bias_.setZero();
  tau_fb_contact_bias_init_ = false;
  tau_fb_contact_state_ = false;
  tau_fb_contact_phase_ = -1;
  tau_fb_contact_scale_ = use_contact_observer_fb_
    ? 0.0
    : (tau_fb_contact_gate_enable_ ? tau_fb_contact_free_scale_ : 1.0);
  tau_fb_contact_on_since_ = -1.0;
  tau_fb_contact_off_since_ = -1.0;
  tau_fb_contact_last_t_ = 0.0;
  tau_fb_contact_stale_bias_since_ = -1.0;
  last_tau_fb_contact_f_norm_N_ = 0.0;
  last_tau_fb_contact_m_norm_Nm_ = 0.0;
  last_tau_fb_contact_raw_f_norm_N_ = 0.0;
  last_tau_fb_contact_ee_speed_m_s_ = 0.0;
  last_tau_fb_contact_joint_speed_rad_s_ = 0.0;
  last_tau_fb_contact_force_on_eff_N_ = tau_fb_contact_force_on_N_;
  last_tau_fb_contact_force_off_eff_N_ = tau_fb_contact_force_off_N_;
  last_tau_fb_contact_moment_on_eff_Nm_ = tau_fb_contact_moment_on_Nm_;
  last_tau_fb_contact_moment_off_eff_Nm_ = tau_fb_contact_moment_off_Nm_;
  last_tau_fb_contact_on_speed_ok_ = true;
  last_tau_fb_contact_stale_bias_reset_ = false;
  tau_fb_contact_gate_active_period_ = false;
  tau_fb_contact_min_scale_period_ = 1.0;
  tau_fb_contact_max_f_norm_period_N_ = 0.0;
  tau_fb_contact_max_m_norm_period_Nm_ = 0.0;
}

Vec6 LeaderTeleopNode::apply_contact_gate_to_wrench_delta(const Vec6& delta_ft) {
  const double f_norm_raw = delta_ft.head<3>().norm();
  const double m_norm_raw = delta_ft.tail<3>().norm();
  last_tau_fb_contact_raw_f_norm_N_ = f_norm_raw;
  last_tau_fb_contact_stale_bias_reset_ = false;
  if (!tau_fb_contact_gate_enable_) {
    last_tau_fb_contact_f_norm_N_ = f_norm_raw;
    last_tau_fb_contact_m_norm_Nm_ = m_norm_raw;
    last_tau_fb_contact_ee_speed_m_s_ = 0.0;
    last_tau_fb_contact_joint_speed_rad_s_ = 0.0;
    last_tau_fb_contact_on_speed_ok_ = true;
    tau_fb_contact_stale_bias_since_ = -1.0;
    tau_fb_contact_scale_ = 1.0;
    tau_fb_contact_phase_ = -1;
    return delta_ft;
  }

  const double t = now().seconds();
  const double elapsed =
    (tau_fb_contact_last_t_ > 0.0) ? std::max(0.0, t - tau_fb_contact_last_t_) : dt_;
  tau_fb_contact_last_t_ = t;

  Vec6 delta_unbiased =
    tau_fb_contact_bias_enable_ ? (delta_ft - tau_fb_contact_bias_) : delta_ft;
  double f_norm = delta_unbiased.head<3>().norm();
  double m_norm = delta_unbiased.tail<3>().norm();
  last_tau_fb_contact_f_norm_N_ = f_norm;
  last_tau_fb_contact_m_norm_Nm_ = m_norm;
  last_tau_fb_contact_ee_speed_m_s_ = compute_leader_tip_linear_speed_m_s();
  last_tau_fb_contact_joint_speed_rad_s_ = leader_dq_.cwiseAbs().maxCoeff();
  last_tau_fb_contact_on_speed_ok_ =
    (tau_fb_contact_on_max_joint_speed_rad_s_ <= 0.0 ||
      last_tau_fb_contact_joint_speed_rad_s_ <= tau_fb_contact_on_max_joint_speed_rad_s_) &&
    (tau_fb_contact_on_max_ee_speed_m_s_ <= 0.0 ||
      last_tau_fb_contact_ee_speed_m_s_ <= tau_fb_contact_on_max_ee_speed_m_s_);
  double speed_blend = 0.0;
  if (tau_fb_contact_speed_gate_enable_) {
    if (tau_fb_contact_speed_high_m_s_ <= tau_fb_contact_speed_low_m_s_ + 1e-9) {
      speed_blend = (last_tau_fb_contact_ee_speed_m_s_ > tau_fb_contact_speed_low_m_s_) ? 1.0 : 0.0;
    } else {
      speed_blend = smoothstep01(
        (last_tau_fb_contact_ee_speed_m_s_ - tau_fb_contact_speed_low_m_s_) /
        (tau_fb_contact_speed_high_m_s_ - tau_fb_contact_speed_low_m_s_));
    }
  }
  last_tau_fb_contact_force_on_eff_N_ =
    tau_fb_contact_force_on_N_ +
    speed_blend * (tau_fb_contact_force_on_fast_N_ - tau_fb_contact_force_on_N_);
  last_tau_fb_contact_force_off_eff_N_ =
    tau_fb_contact_force_off_N_ +
    speed_blend * (tau_fb_contact_force_off_fast_N_ - tau_fb_contact_force_off_N_);
  last_tau_fb_contact_moment_on_eff_Nm_ =
    tau_fb_contact_moment_on_Nm_ +
    speed_blend * (tau_fb_contact_moment_on_fast_Nm_ - tau_fb_contact_moment_on_Nm_);
  last_tau_fb_contact_moment_off_eff_Nm_ =
    tau_fb_contact_moment_off_Nm_ +
    speed_blend * (tau_fb_contact_moment_off_fast_Nm_ - tau_fb_contact_moment_off_Nm_);
  tau_fb_contact_max_f_norm_period_N_ =
    std::max(tau_fb_contact_max_f_norm_period_N_, f_norm);
  tau_fb_contact_max_m_norm_period_Nm_ =
    std::max(tau_fb_contact_max_m_norm_period_Nm_, m_norm);

  bool contact_on_candidate =
    last_tau_fb_contact_on_speed_ok_ &&
    (f_norm >= last_tau_fb_contact_force_on_eff_N_ ||
      m_norm >= last_tau_fb_contact_moment_on_eff_Nm_);
  bool contact_off_candidate =
    f_norm <= last_tau_fb_contact_force_off_eff_N_ &&
    m_norm <= last_tau_fb_contact_moment_off_eff_Nm_;

  const bool stale_bias_candidate =
    tau_fb_contact_stale_bias_reset_enable_ &&
    tau_fb_contact_bias_enable_ &&
    tau_fb_contact_state_ &&
    f_norm_raw <= tau_fb_contact_stale_bias_raw_force_max_N_ &&
    f_norm >= tau_fb_contact_stale_bias_residual_force_min_N_ &&
    f_norm <= tau_fb_contact_stale_bias_residual_force_max_N_ &&
    last_tau_fb_contact_ee_speed_m_s_ <= tau_fb_contact_stale_bias_speed_max_m_s_;
  if (stale_bias_candidate) {
    if (tau_fb_contact_stale_bias_since_ < 0.0) tau_fb_contact_stale_bias_since_ = t;
    if (t - tau_fb_contact_stale_bias_since_ >= tau_fb_contact_stale_bias_hold_s_) {
      tau_fb_contact_bias_ = delta_ft;
      clamp_contact_bias();
      tau_fb_contact_bias_init_ = true;
      tau_fb_contact_state_ = false;
      tau_fb_contact_on_since_ = -1.0;
      tau_fb_contact_off_since_ = t;
      last_tau_fb_contact_stale_bias_reset_ = true;

      delta_unbiased = delta_ft - tau_fb_contact_bias_;
      f_norm = delta_unbiased.head<3>().norm();
      m_norm = delta_unbiased.tail<3>().norm();
      last_tau_fb_contact_f_norm_N_ = f_norm;
      last_tau_fb_contact_m_norm_Nm_ = m_norm;
      contact_on_candidate = false;
      contact_off_candidate = true;
    }
  } else {
    tau_fb_contact_stale_bias_since_ = -1.0;
  }

  if (!tau_fb_contact_state_) {
    tau_fb_contact_off_since_ = -1.0;
    if (contact_on_candidate) {
      if (tau_fb_contact_on_since_ < 0.0) tau_fb_contact_on_since_ = t;
      if (t - tau_fb_contact_on_since_ >= tau_fb_contact_on_hold_s_) {
        tau_fb_contact_state_ = true;
        tau_fb_contact_off_since_ = -1.0;
      }
    } else {
      tau_fb_contact_on_since_ = -1.0;
    }
  } else {
    tau_fb_contact_on_since_ = -1.0;
    if (contact_off_candidate) {
      if (tau_fb_contact_off_since_ < 0.0) tau_fb_contact_off_since_ = t;
      if (t - tau_fb_contact_off_since_ >= tau_fb_contact_off_hold_s_) {
        tau_fb_contact_state_ = false;
        tau_fb_contact_on_since_ = -1.0;
      }
    } else {
      tau_fb_contact_off_since_ = -1.0;
    }
  }

  tau_fb_contact_phase_ = -1;
  if (tau_fb_contact_state_) {
    tau_fb_contact_phase_ = 1;
  } else if (use_pre_contact_phase_ && contact_on_candidate &&
             tau_fb_contact_on_since_ >= 0.0 &&
             tau_fb_contact_on_hold_s_ > 1e-9) {
    const double pre_hold_s = 0.2 * tau_fb_contact_on_hold_s_;
    const double candidate_age_s = t - tau_fb_contact_on_since_;
    if (candidate_age_s >= pre_hold_s &&
        candidate_age_s < tau_fb_contact_on_hold_s_) {
      tau_fb_contact_phase_ = 0;
    }
  }

  const double target_scale = tau_fb_contact_state_ ? 1.0 : tau_fb_contact_free_scale_;
  const double ramp_s =
    (target_scale > tau_fb_contact_scale_) ? tau_fb_contact_ramp_up_s_ : tau_fb_contact_ramp_down_s_;
  if (ramp_s <= 1e-9) {
    tau_fb_contact_scale_ = target_scale;
  } else {
    const double max_delta = elapsed / ramp_s;
    tau_fb_contact_scale_ +=
      std::clamp(target_scale - tau_fb_contact_scale_, -max_delta, max_delta);
  }
  tau_fb_contact_scale_ = std::clamp(tau_fb_contact_scale_, 0.0, 1.0);

  if (tau_fb_contact_scale_ < 1.0 - 1e-9) {
    tau_fb_contact_gate_active_period_ = true;
    tau_fb_contact_min_scale_period_ =
      std::min(tau_fb_contact_min_scale_period_, tau_fb_contact_scale_);
  }

  const bool can_update_bias =
    tau_fb_contact_bias_enable_ &&
    !tau_fb_contact_state_ &&
    !contact_on_candidate &&
    last_tau_fb_contact_ee_speed_m_s_ <= tau_fb_contact_bias_update_max_ee_speed_m_s_ &&
    (tau_fb_contact_bias_update_max_joint_speed_rad_s_ <= 0.0 ||
      last_tau_fb_contact_joint_speed_rad_s_ <= tau_fb_contact_bias_update_max_joint_speed_rad_s_);
  if (can_update_bias) {
    if (!tau_fb_contact_bias_init_) {
      tau_fb_contact_bias_ = delta_ft;
      tau_fb_contact_bias_init_ = true;
    } else {
      tau_fb_contact_bias_ += tau_fb_contact_bias_alpha_ * (delta_ft - tau_fb_contact_bias_);
    }
    clamp_contact_bias();
  }

  return tau_fb_contact_scale_ * delta_unbiased;
}

Vec6 LeaderTeleopNode::condition_feedback_torque(Vec6 tau_fb) {
  tau_fb_deadband_active_.fill(false);
  tau_fb_motion_gate_driver_.fill(false);
  tau_fb_passivity_gate_joint_.fill(false);
  last_tau_fb_motion_gate_speed_ = 0.0;
  last_tau_fb_passivity_power_W_ = 0.0;
  last_tau_fb_passivity_scale_ = 1.0;

  for (int i = 0; i < 6; ++i) {
    const double db = std::max(0.0, tau_fb_deadband_Nm_[i]);
    const double before = tau_fb[i];
    const double mag = std::abs(tau_fb[i]);
    if (db > 0.0) {
      if (mag <= db) {
        tau_fb[i] = 0.0;
      } else {
        tau_fb[i] = std::copysign(mag - db, tau_fb[i]);
      }
      if (std::abs(tau_fb[i] - before) > 1e-12) {
        tau_fb_deadband_active_[i] = true;
        tau_fb_deadband_active_period_[i] = true;
      }
    }
  }

  last_tau_fb_gate_scale_ = 1.0;
  if (tau_fb_motion_gate_enable_) {
    const bool use_joint_speed = tau_fb_motion_gate_speed_source_ == "joint_max";
    const double speed = use_joint_speed
      ? leader_dq_.cwiseAbs().maxCoeff()
      : compute_leader_tip_linear_speed_m_s();
    last_tau_fb_motion_gate_speed_ = speed;
    const double low = use_joint_speed
      ? tau_fb_motion_gate_speed_low_rad_s_
      : tau_fb_motion_gate_speed_low_m_s_;
    const double high = use_joint_speed
      ? tau_fb_motion_gate_speed_high_rad_s_
      : tau_fb_motion_gate_speed_high_m_s_;
    double scale = 1.0;
    if (high <= low + 1e-9) {
      scale = (speed > low) ? tau_fb_motion_gate_min_scale_ : 1.0;
    } else if (speed >= high) {
      scale = tau_fb_motion_gate_min_scale_;
    } else if (speed > low) {
      const double x = std::clamp((speed - low) / (high - low), 0.0, 1.0);
      const double smooth = x * x * (3.0 - 2.0 * x);
      scale = 1.0 - (1.0 - tau_fb_motion_gate_min_scale_) * smooth;
    }
    last_tau_fb_gate_scale_ = std::clamp(scale, 0.0, 1.0);
    if (last_tau_fb_gate_scale_ < 1.0 - 1e-9 && speed > low) {
      tau_fb_motion_gate_active_period_ = true;
      if (use_joint_speed) {
        for (int i = 0; i < 6; ++i) {
          const bool driver = std::abs(std::abs(leader_dq_[i]) - speed) <= 1e-9;
          tau_fb_motion_gate_driver_[i] = driver;
          if (driver) {
            tau_fb_motion_gate_driver_period_[i] = true;
            tau_fb_motion_gate_driver_speed_period_[i] =
              std::max(tau_fb_motion_gate_driver_speed_period_[i], std::abs(leader_dq_[i]));
          }
        }
      }
      tau_fb_motion_gate_max_speed_period_ =
        std::max(tau_fb_motion_gate_max_speed_period_, speed);
      tau_fb_motion_gate_min_scale_period_ =
        std::min(tau_fb_motion_gate_min_scale_period_, last_tau_fb_gate_scale_);
    }
    tau_fb *= last_tau_fb_gate_scale_;
  }

  if (tau_fb_passivity_gate_enable_) {
    for (int i = 0; i < 6; ++i) {
      const double power = tau_fb[i] * leader_dq_[i];
      last_tau_fb_passivity_power_W_ =
        std::max(last_tau_fb_passivity_power_W_, power);
      if (power <= tau_fb_passivity_power_start_W_) {
        continue;
      }

      double scale = 1.0;
      if (tau_fb_passivity_power_full_W_ <= tau_fb_passivity_power_start_W_ + 1e-12 ||
          power >= tau_fb_passivity_power_full_W_) {
        scale = tau_fb_passivity_min_scale_;
      } else {
        const double x = std::clamp(
          (power - tau_fb_passivity_power_start_W_) /
          (tau_fb_passivity_power_full_W_ - tau_fb_passivity_power_start_W_),
          0.0, 1.0);
        const double smooth = x * x * (3.0 - 2.0 * x);
        scale = 1.0 - (1.0 - tau_fb_passivity_min_scale_) * smooth;
      }
      const double joint_scale = std::clamp(scale, 0.0, 1.0);
      last_tau_fb_passivity_scale_ =
        std::min(last_tau_fb_passivity_scale_, joint_scale);
      if (joint_scale < 1.0 - 1e-9) {
        tau_fb_passivity_gate_joint_[i] = true;
        tau_fb_passivity_gate_joint_period_[i] = true;
        tau_fb_passivity_gate_active_period_ = true;
        tau_fb_passivity_max_power_period_W_ =
          std::max(tau_fb_passivity_max_power_period_W_, power);
        tau_fb_passivity_min_scale_period_ =
          std::min(tau_fb_passivity_min_scale_period_, joint_scale);
        tau_fb[i] *= joint_scale;
      }
    }
  }

  return tau_fb;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Main control loop
// ═══════════════════════════════════════════════════════════════════════════════

void LeaderTeleopNode::control_loop() {
  process_pending_control_commands();
  if (shutdown_requested_) return;
  if (!aligned_once_) {
    handle_keyboard();
    publish_status_if_due();
    return;
  }

  // 1) Read leader positions
  q_leader_ = read_leader_positions();
  const bool degraded_dxl_read =
    !last_leader_position_read_fresh_ || bus_->last_fallback_count() > 0 ||
    (leader_damping_use_dxl_velocity_ && bus_->last_velocity_fallback_count() > 0);
  if (degraded_dxl_read) {
    ++dxl_degraded_read_streak_;
  } else {
    dxl_degraded_read_streak_ = 0;
  }
  if (dxl_degraded_read_streak_ >= 10) {
    std::ostringstream reason;
    reason << "10 consecutive degraded DXL reads"
           << " (position fallback=" << bus_->last_fallback_count()
           << ", missing=" << bus_->last_missing_count()
           << ", velocity fallback=" << bus_->last_velocity_fallback_count()
           << ", velocity missing=" << bus_->last_velocity_missing_count() << ")";
    if (!bus_->last_error().empty()) reason << ": " << bus_->last_error();
    enter_dxl_fault(reason.str());
    return;
  }
  for (int i = 0; i < 6; ++i) {
    q_full_leader_[idxq_leader_[i]] = q_leader_[i];
  }

  // 2) Gravity compensation
  Vec6 tau_g = compute_gravity_torque(q_leader_);
  last_tau_grav_ = tau_g;

  // 2b) Leader viscous damping (C: differentiate_pose, D: dxl_vel)
  Vec6 tau_damp = compute_leader_damping_torque(q_leader_);
  if (!dxl_current_mode_) {
    tau_damp.setZero();
    last_tau_damp_.setZero();
  }

  // 3) Feedback (FAST only) — physical FT sensor or per-joint JT wrench
  Vec6 tau_fb = Vec6::Zero();
  if (state_ == TeleopState::FAST) {
    if (use_ft_sensor_feedback_) {
      tau_fb = compute_ft_sensor_feedback();
    } else if (use_jt_wrench_fb_ || use_contact_observer_fb_) {
      tau_fb = compute_jt_wrench_feedback();
    }
    tau_fb = condition_feedback_torque(tau_fb);
    if (use_contact_observer_fb_ && !contact_observation_feedback_active_) {
      // Invalid, stale, model-not-ready and canonical FREE states must never
      // leak a previous LPF/slew state back to the leader.
      tau_fb.setZero();
      tau_lpf_state_.setZero();
      tau_lpf_init_ = false;
      tau_fb_slew_state_.setZero();
      tau_fb_slew_init_ = false;
    }
  } else {
    contact_observation_feedback_active_ = false;
    last_tau_fb_gate_scale_ = 1.0;
    last_tau_fb_motion_gate_speed_ = 0.0;
    last_tau_fb_passivity_power_W_ = 0.0;
    last_tau_fb_passivity_scale_ = 1.0;
    tau_fb_deadband_active_.fill(false);
    tau_fb_motion_gate_driver_.fill(false);
    tau_fb_passivity_gate_joint_.fill(false);
    reset_contact_gate_state();
  }

  if (pub_contact_state_) {
    std_msgs::msg::Int32 msg;
    msg.data = tau_fb_contact_state_ ? 1 : -1;
    pub_contact_state_->publish(msg);
  }
  if (pub_contact_phase_) {
    std_msgs::msg::Int32 msg;
    msg.data = tau_fb_contact_phase_;
    pub_contact_phase_->publish(msg);
  }

  // 3b) Per-joint low-pass on the reflection torque (passthrough where alpha==1).
  //     Use only on joints that need residual current-buzz smoothing.
  if (state_ == TeleopState::FAST) {
    if (!tau_lpf_init_) {
      tau_lpf_state_ = tau_fb;
      tau_lpf_init_ = true;
    } else {
      for (int i = 0; i < 6; ++i) {
        tau_lpf_state_[i] += tau_lpf_alpha_[i] * (tau_fb[i] - tau_lpf_state_[i]);
        tau_fb[i] = tau_lpf_state_[i];
      }
    }
  } else {
    tau_lpf_init_ = false;
  }

  // 3c) Per-joint slew-rate limit after LPF and before the final safety clip.
  if (state_ == TeleopState::FAST) {
    tau_fb_slew_active_.fill(false);
    if (!tau_fb_slew_init_) {
      tau_fb_slew_state_ = tau_fb;
      tau_fb_slew_init_ = true;
    } else {
      for (int i = 0; i < 6; ++i) {
        const double rate = tau_fb_slew_rate_Nm_s_[i];
        if (rate > 0.0) {
          const double max_delta = rate * dt_;
          const double raw_delta = tau_fb[i] - tau_fb_slew_state_[i];
          const double delta = std::clamp(raw_delta, -max_delta, max_delta);
          if (std::abs(raw_delta - delta) > 1e-12) {
            tau_fb_slew_active_[i] = true;
            tau_fb_slew_active_period_[i] = true;
          }
          tau_fb_slew_state_[i] += delta;
          tau_fb[i] = tau_fb_slew_state_[i];
        } else {
          tau_fb_slew_state_[i] = tau_fb[i];
        }
      }
    }
  } else {
    tau_fb_slew_init_ = false;
    tau_fb_slew_active_.fill(false);
  }

  if (!tau_fb.allFinite()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[SAFETY] Non-finite feedback torque rejected");
    tau_fb.setZero();
    contact_observation_feedback_active_ = false;
    tau_lpf_state_.setZero();
    tau_lpf_init_ = false;
    tau_fb_slew_state_.setZero();
    tau_fb_slew_init_ = false;
    if (use_contact_observer_fb_) {
      tau_fb_contact_state_ = false;
      tau_fb_contact_phase_ = -1;
      tau_fb_contact_scale_ = 0.0;
    }
  }

  // 3d) Single per-joint clip — the ONLY saturation point, so last_tau_unclipped_
  //     vs last_tau_fb_ is a true saturation diagnostic. Count rails per joint.
  last_tau_unclipped_ = tau_fb;  // total feedback before the clip
  if (state_ == TeleopState::FAST) {
    for (int i = 0; i < 6; ++i) {
      double lim = jt_wrench_fb_clip_[i];
      if (std::abs(tau_fb[i]) > lim) {
        tau_fb[i] = std::clamp(tau_fb[i], -lim, lim);
        ++sat_count_[i];
      }
    }
    ++sat_ticks_;
  }
  last_tau_fb_ = tau_fb;  // for CSV log (not gravity comp torque)

  // 4) Total torque
  Vec6 tau_total = tau_g + tau_fb + tau_damp;
  if (!tau_total.allFinite()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[SAFETY] Non-finite total torque rejected; commanding zero current");
    tau_total.setZero();
    tau_fb.setZero();
    last_tau_fb_.setZero();
    last_tau_unclipped_.setZero();
    contact_observation_feedback_active_ = false;
    tau_lpf_state_.setZero();
    tau_lpf_init_ = false;
    tau_fb_slew_state_.setZero();
    tau_fb_slew_init_ = false;
    if (use_contact_observer_fb_) {
      tau_fb_contact_state_ = false;
      tau_fb_contact_phase_ = -1;
      tau_fb_contact_scale_ = 0.0;
    }
  }
  last_tau_cmd_ = tau_total;

  // 5) Send torque only after explicit current-mode entry.
  if (dxl_current_mode_) {
    if (!send_torque_currents(tau_total)) {
      enter_dxl_fault("goal-current write failed: " + bus_->last_error());
      return;
    }
  } else {
    last_tau_grav_.setZero();
    last_tau_fb_.setZero();
    last_tau_unclipped_.setZero();
    last_tau_damp_.setZero();
    last_tau_cmd_.setZero();
  }

  if (state_ == TeleopState::INIT_POSE) {
    update_follower_init_pose();
    handle_keyboard();
    publish_status_if_due();
    return;
  }

  // 6) Compute follower target
  Vec6 follower_target = clip_follower_joint_rad(q_leader_);
  last_follower_target_ = follower_target;

  // 7) Publish based on state
  if (state_ == TeleopState::SLOW_SYNC) {
    Vec6 cmd = slow_sync_step(follower_target);
    publish_impedance_pose(cmd);
  } else if (state_ == TeleopState::FAST) {
    publish_impedance_pose(follower_target);
  }

  // 8) CSV movement log + Hz monitor
  csv_log_row();
  hz_log_if_due();

  // 9) Keyboard
  handle_keyboard();

  // Publish operator status from the same callback as the control state.  A
  // separate low-rate timer in the default MutuallyExclusive callback group
  // can be starved by this continuously-ready 500 Hz control timer.
  publish_status_if_due();
}

static const char* state_name(TeleopState s) {
  switch (s) {
    case TeleopState::INIT:          return "ALIGN";
    case TeleopState::ALIGN:         return "ALIGN";
    case TeleopState::MAIN_IDLE:     return "IDLE";
    case TeleopState::PAUSED:        return "PAUSE";
    case TeleopState::CURRENT_READY: return "CURRENT";
    case TeleopState::SLOW_SYNC:     return "SLOW";
    case TeleopState::INIT_POSE:     return "INIT POSE";
    case TeleopState::FAST:          return "FAST";
    case TeleopState::SHUTDOWN:      return "SHUTDOWN";
  }
  return "?";
}

const char* LeaderTeleopNode::teleop_state_name() const {
  return state_name(state_);
}

std::string LeaderTeleopNode::green(const std::string& text) const {
  return color_log_ ? ("\033[32m" + text + "\033[0m") : text;
}

std::string LeaderTeleopNode::blue(const std::string& text) const {
  return color_log_ ? ("\033[34m" + text + "\033[0m") : text;
}

std::string LeaderTeleopNode::red(const std::string& text) const {
  return color_log_ ? ("\033[31m" + text + "\033[0m") : text;
}

std::string LeaderTeleopNode::state_tag(const std::string& state) const {
  return "[" + green(state) + "]";
}

static std::string format_labeled_vec6(
    const Vec6& values,
    const std::array<const char*, 6>& labels,
    double scale,
    int precision,
    int width) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(precision);
  for (int i = 0; i < 6; ++i) {
    if (i > 0) os << " | ";
    os << labels[static_cast<std::size_t>(i)] << " "
       << std::showpos << std::setw(width) << (values[i] * scale)
       << std::noshowpos;
  }
  return os.str();
}

std::string LeaderTeleopNode::format_joint_deg(const Vec6& q, bool color_values) const {
  (void)color_values;  // Keep the old call signature; numeric alignment wins over color.
  static constexpr std::array<const char*, 6> kJointLabels{{"J1", "J2", "J3", "J4", "J5", "J6"}};
  return format_labeled_vec6(q, kJointLabels, 180.0 / M_PI, 2, 8);
}

std::string LeaderTeleopNode::format_joint_nm(const Vec6& tau) const {
  static constexpr std::array<const char*, 6> kJointLabels{{"J1", "J2", "J3", "J4", "J5", "J6"}};
  return format_labeled_vec6(tau, kJointLabels, 1.0, 3, 8);
}

std::string LeaderTeleopNode::format_joint_scale(const Vec6& scale) const {
  static constexpr std::array<const char*, 6> kJointLabels{{"J1", "J2", "J3", "J4", "J5", "J6"}};
  return format_labeled_vec6(scale, kJointLabels, 1.0, 2, 7);
}

std::string LeaderTeleopNode::format_wrench(const Vec6& wrench) const {
  static constexpr std::array<const char*, 6> kWrenchLabels{{"Fx", "Fy", "Fz", "Mx", "My", "Mz"}};
  return format_labeled_vec6(wrench, kWrenchLabels, 1.0, 3, 8);
}

std::string LeaderTeleopNode::format_diff_deg(const Vec6& diff, double tol_deg) const {
  static constexpr std::array<const char*, 6> kJointLabels{{"J1", "J2", "J3", "J4", "J5", "J6"}};
  std::ostringstream os;
  for (int i = 0; i < 6; ++i) {
    if (i > 0) os << " | ";
    std::ostringstream segment;
    segment << std::fixed << std::setprecision(2)
            << kJointLabels[static_cast<std::size_t>(i)] << " "
            << std::showpos << std::setw(8) << (diff[i] * 180.0 / M_PI)
            << std::noshowpos;
    const std::string text = segment.str();
    os << ((std::abs(diff[i] * 180.0 / M_PI) > tol_deg) ? red(text) : text);
  }
  return os.str();
}

std::string LeaderTeleopNode::format_ready(bool ready) const {
  return ready ? "true" : "false";
}

std::string LeaderTeleopNode::feedback_source_label() const {
  if (feedback_source_ == "contact_observer") return "CONTACT_OBSERVER";
  if (feedback_source_ == "jt_wrench") return "JT_WRENCH";
  if (feedback_source_ == "ft") return "FT";
  if (feedback_source_ == "off") return "OFF";
  return feedback_source_;
}

bool LeaderTeleopNode::runtime_gravity_enabled() const {
  return use_gravity_comp_ && arm_.grav_gain.cwiseAbs().maxCoeff() > 1e-12;
}

bool LeaderTeleopNode::runtime_feedback_enabled() const {
  if (feedback_source_ == "off") return false;
  if (use_jt_wrench_fb_ || use_contact_observer_fb_) {
    return arm_.jt_wrench_fb_gain.cwiseAbs().maxCoeff() > 1e-12;
  }
  if (use_ft_sensor_feedback_) {
    return ft_fb_gain_.cwiseAbs().maxCoeff() > 1e-12;
  }
  return false;
}

static const std::string& log_rule() {
  static const std::string rule = [] {
    std::string line;
    for (int i = 0; i < 120; ++i) {
      line += "═";
    }
    return line;
  }();
  return rule;
}

static std::string log_separator() {
  return "  " + log_rule();
}

static bool any_joint_flag(const std::array<bool, 6>& flags) {
  return std::any_of(flags.begin(), flags.end(), [](bool flag) { return flag; });
}

static std::string format_joint_flags(const std::array<bool, 6>& flags) {
  static constexpr std::array<const char*, 6> kJointLabels{{"J1", "J2", "J3", "J4", "J5", "J6"}};
  std::ostringstream os;
  bool first = true;
  for (int i = 0; i < 6; ++i) {
    if (!flags[static_cast<std::size_t>(i)]) continue;
    if (!first) os << ",";
    os << kJointLabels[static_cast<std::size_t>(i)];
    first = false;
  }
  return first ? std::string("none") : os.str();
}

static std::string format_joint_flags_with_abs_values(
    const std::array<bool, 6>& flags,
    const Vec6& values,
    const std::string& unit) {
  static constexpr std::array<const char*, 6> kJointLabels{{"J1", "J2", "J3", "J4", "J5", "J6"}};
  std::ostringstream os;
  os << std::fixed << std::setprecision(3);
  bool first = true;
  for (int i = 0; i < 6; ++i) {
    if (!flags[static_cast<std::size_t>(i)]) continue;
    if (!first) os << ", ";
    os << kJointLabels[static_cast<std::size_t>(i)]
       << " |dq|=" << std::abs(values[i]) << unit;
    first = false;
  }
  return first ? std::string("none") : os.str();
}

static double smoothstep01(double x) {
  x = std::clamp(x, 0.0, 1.0);
  return x * x * (3.0 - 2.0 * x);
}

static std::filesystem::path ft_fb_leaderarm_package_root() {
  std::error_code ec;
  const std::filesystem::path compiled_root(FT_FB_LEADERARM_PACKAGE_SOURCE_DIR);
  if (!compiled_root.empty() &&
      std::filesystem::exists(compiled_root / "package.xml", ec)) {
    return compiled_root;
  }

  const std::filesystem::path cwd = std::filesystem::current_path(ec);
  if (!ec) {
    for (std::filesystem::path p = cwd; !p.empty(); p = p.parent_path()) {
      if (p.filename() == "ft_fb_leaderarm" &&
          std::filesystem::exists(p / "package.xml", ec)) {
        return p;
      }
      if (p == p.root_path()) break;
    }
    return cwd;
  }

  return std::filesystem::path(".");
}

static std::filesystem::path resolve_csv_log_dir(const std::string& configured_dir) {
  std::filesystem::path dir(configured_dir.empty() ? std::string("logs") : configured_dir);
  if (dir.is_absolute()) {
    return dir.lexically_normal();
  }
  return (ft_fb_leaderarm_package_root() / dir).lexically_normal();
}

void LeaderTeleopNode::log_align_status(
    const Vec6& current,
    const Vec6& target,
    double max_err_rad,
    double tol_rad,
    const std::string& status) const {
  const Vec6 err = target - current;
  RCLCPP_INFO(get_logger(),
    "%s Leader position [deg]\n"
    "  current : %s\n"
    "  target  : %s\n"
    "  error   : %s\n"
    "  max_err : %.2f deg / tol %.2f deg\n"
    "  status  : %s\n",
    state_tag("ALIGN").c_str(),
    format_joint_deg(current, false).c_str(),
    format_joint_deg(target, false).c_str(),
    format_joint_deg(err, false).c_str(),
    max_err_rad * 180.0 / M_PI,
    tol_rad * 180.0 / M_PI,
    status.c_str());
}

void LeaderTeleopNode::hz_log_if_due() {
  ++loop_count_;
  double t = now().seconds();
  if (hz_log_last_t_ <= 0.0) {
    hz_log_last_t_ = t;
    return;
  }
  double elapsed = t - hz_log_last_t_;
  if (elapsed >= hz_log_period_) {
    double completed_hz = static_cast<double>(loop_count_) / elapsed;
    double dxl_read_hz = static_cast<double>(dxl_read_count_) / elapsed;
    double effective_hz = std::min(completed_hz, dxl_read_hz);
    double vel_read_hz = static_cast<double>(vel_read_count_) / elapsed;
    double vel_fresh_hz = static_cast<double>(vel_fresh_count_) / elapsed;
    double teleop_cmd_hz = static_cast<double>(teleop_cmd_count_) / elapsed;

    Vec6 q_foll;
    bool has_foll = false;
    {
      std::lock_guard<std::mutex> lock(follower_mtx_);
      if (follower_joint_rad_.has_value()) {
        q_foll = follower_joint_rad_.value();
        has_foll = true;
      }
    }

    Vec6 fe_raw;
    double fe_age_ms;
    bool observer_received = false;
    bool observer_valid = false;
    bool observer_model_ready = false;
    bool observer_fresh = false;
    uint8_t observer_contact_state =
      contact_observer_msgs::msg::ContactObservation::FREE;
    double observer_source_age_ms = -1.0;
    double observer_local_age_ms = -1.0;
    if (use_contact_observer_fb_) {
      std::lock_guard<std::mutex> lock(contact_observation_mtx_);
      fe_raw = contact_observation_wrench_;
      fe_age_ms = (contact_observation_receive_stamp_ > 0.0)
        ? (t - contact_observation_receive_stamp_) * 1000.0 : -1.0;
      observer_received = contact_observation_receive_steady_valid_;
      observer_valid = contact_observation_valid_;
      observer_model_ready = contact_observation_model_ready_;
      observer_contact_state = contact_observation_state_;
      if (observer_received) {
        observer_local_age_ms = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - contact_observation_receive_steady_).count()
          * 1000.0;
        observer_source_age_ms =
          (t - contact_observation_source_stamp_) * 1000.0;
        observer_fresh =
          observer_local_age_ms >= 0.0 &&
          observer_local_age_ms <= contact_observation_stale_timeout_ * 1000.0 &&
          observer_source_age_ms >=
            -contact_observation_clock_future_tolerance_ * 1000.0 &&
          observer_source_age_ms <= contact_observation_stale_timeout_ * 1000.0;
      }
    } else if (use_ft_sensor_feedback_) {
      std::lock_guard<std::mutex> lock(ft_sensor_mtx_);
      fe_raw = ft_sensor_raw_;
      fe_age_ms = (ft_sensor_stamp_ > 0.0) ? (t - ft_sensor_stamp_) * 1000.0 : -1.0;
    } else {
      std::lock_guard<std::mutex> lock(jt_wrench_mtx_);
      fe_raw = jt_wrench_raw_;
      fe_age_ms = (jt_wrench_stamp_ > 0.0) ? (t - jt_wrench_stamp_) * 1000.0 : -1.0;
    }

    const std::string state = state_name(state_);
    if (state_ == TeleopState::CURRENT_READY) {
      std::string tag = state_tag("CURRENT");
      std::string leader_deg = format_joint_deg(q_leader_);
      std::ostringstream block;
      block << tag << " Leader teleop\n"
            << "  mode    : current_ready\n"
            << "  gravity : " << (runtime_gravity_enabled() ? "ON" : "OFF")
            << "\n"
            << "  scale   : " << format_joint_scale(grav_scale_) << "\n"
            << "  leader  : " << leader_deg << "\n"
            << "  next    : press 't' for slow sync\n";
      RCLCPP_INFO(get_logger(), "%s", block.str().c_str());
    } else if (state_ == TeleopState::SLOW_SYNC) {
      std::string tag = state_tag("SLOW");
      Vec6 diff = Vec6::Zero();
      if (has_foll) diff = last_follower_target_ - q_foll;
      double max_diff = has_foll ? diff.cwiseAbs().maxCoeff() * 180.0 / M_PI : 0.0;
      std::string leader_deg = format_joint_deg(q_leader_);
      std::string follower_deg = has_foll ? format_joint_deg(q_foll) : std::string("(no data)");
      std::string diff_deg = format_diff_deg(diff, slow_sync_config_.ready_tol_deg);
      std::string ready = format_ready(slow_sync_ready_);
      std::ostringstream block;
      block << tag << " Sync status\n"
            << std::fixed << std::setprecision(1)
            << "  speed   : max " << slow_sync_config_.max_vel_deg_s << " deg/s\n"
            << "  ready   : " << ready
            << ", tol " << slow_sync_config_.ready_tol_deg
            << " deg, hold " << slow_sync_config_.ready_hold_sec << " s\n"
            << log_separator() << "\n"
            << "  leader  : " << leader_deg << "\n"
            << "  follower: " << follower_deg << "\n"
            << log_separator() << "\n"
            << "  diff    : " << diff_deg << "\n"
            << "  max_err : " << std::fixed << std::setprecision(2) << max_diff << " deg\n";
      RCLCPP_INFO(get_logger(), "%s", block.str().c_str());
    } else if (state_ == TeleopState::FAST) {
      std::string tag = state_tag("FAST");
      std::string leader_deg = format_joint_deg(q_leader_);
      std::string follower_deg = has_foll ? format_joint_deg(q_foll) : std::string("(no data)");
      std::string wrench = format_wrench(fe_raw);
      std::string tau_grav = format_joint_nm(last_tau_grav_);
      std::string tau_fb = format_joint_nm(last_tau_fb_);
      std::string tau_damp = format_joint_nm(last_tau_damp_);
      std::string tau_cmd = format_joint_nm(last_tau_cmd_);
      std::string vel_source = leader_damping_velocity_source_;
      vel_source += leader_damping_use_dxl_velocity_ ? " (dxl register)" : " (pos-diff estimate)";
      const bool deadband_applied = any_joint_flag(tau_fb_deadband_active_period_);
      const bool slew_applied = any_joint_flag(tau_fb_slew_active_period_);
      const bool motion_gate_applied = tau_fb_motion_gate_active_period_;
      const bool passivity_gate_applied = tau_fb_passivity_gate_active_period_;
      const bool contact_gate_applied = tau_fb_contact_gate_active_period_;
      const char* contact_phase_name =
        tau_fb_contact_phase_ == 1 ? "CONTACT" :
        tau_fb_contact_phase_ == 0 ? "PRE_CONTACT" : "FREE";
      std::ostringstream block;
      block << tag << " Teleop status\n"
            << std::fixed << std::setprecision(1)
            << "  hz      : loop " << completed_hz
            << " | dxl " << dxl_read_hz
            << " | effective " << effective_hz
            << " | cmd " << teleop_cmd_hz << "\n"
            << "  vel     : src " << vel_source
            << " | read " << vel_read_hz
            << " | fresh " << vel_fresh_hz
            << " | fallback " << vel_fallback_count_
            << " | missing " << vel_missing_count_ << "\n"
            << "  feedback: "
            << (runtime_feedback_enabled() ? "ON" : "OFF")
            << ", source " << feedback_source_label()
            << ", gate " << std::fixed << std::setprecision(2) << last_tau_fb_gate_scale_ << "\n"
            << std::setprecision(1);
      if (use_contact_observer_fb_) {
        std::string observer_state_text;
        if (!observer_received) {
          observer_state_text = red("WAITING");
        } else if (!observer_model_ready) {
          observer_state_text = red("MODEL_NOT_READY");
        } else if (!observer_fresh) {
          observer_state_text = red("STALE");
        } else if (!observer_valid) {
          observer_state_text = red("INVALID");
        } else if (
          observer_contact_state ==
          contact_observer_msgs::msg::ContactObservation::CONTACT)
        {
          observer_state_text = red("CONTACT (1)");
        } else {
          observer_state_text = blue("FREE (0)");
        }
        block << "  contact : " << observer_state_text
              << " | valid " << (observer_valid ? "true" : "false")
              << " model " << (observer_model_ready ? "ready" : "not_ready")
              << " | source_age " << std::fixed << std::setprecision(2)
              << observer_source_age_ms << "ms"
              << " local_age " << observer_local_age_ms << "ms\n";
      } else if (tau_fb_contact_gate_enable_) {
        block << "  contact : " << (tau_fb_contact_state_ ? red("ON") : blue("OFF"))
              << " scale " << std::fixed << std::setprecision(2) << tau_fb_contact_scale_
              << " phase " << contact_phase_name
              << " | F " << std::setprecision(2) << last_tau_fb_contact_f_norm_N_ << "N"
              << " raw " << last_tau_fb_contact_raw_f_norm_N_ << "N"
              << " M " << last_tau_fb_contact_m_norm_Nm_ << "Nm"
              << " | v " << last_tau_fb_contact_ee_speed_m_s_ << "m/s"
              << " jq " << last_tau_fb_contact_joint_speed_rad_s_ << "rad/s"
              << " | Fthr " << last_tau_fb_contact_force_on_eff_N_
              << "/" << last_tau_fb_contact_force_off_eff_N_ << "N"
              << " | bias " << (tau_fb_contact_bias_enable_ ? "ON" : "OFF")
              << " on_speed " << (last_tau_fb_contact_on_speed_ok_ ? "OK" : "BLOCK")
              << (last_tau_fb_contact_stale_bias_reset_ ? " reset" : "")
              << "\n";
      }
      if (deadband_applied || slew_applied || motion_gate_applied ||
          passivity_gate_applied || contact_gate_applied) {
        std::ostringstream cond;
        cond << "  fb_limit: ";
        bool first = true;
        if (contact_gate_applied) {
          cond << "contact_gate " << (tau_fb_contact_state_ ? red("ON") : blue("OFF"))
               << " scale " << std::fixed << std::setprecision(2)
               << tau_fb_contact_min_scale_period_;
          first = false;
        }
        if (deadband_applied) {
          if (!first) cond << " | ";
          cond << "deadband " << format_joint_flags(tau_fb_deadband_active_period_);
          first = false;
        }
        if (slew_applied) {
          if (!first) cond << " | ";
          cond << "slew_rate " << format_joint_flags(tau_fb_slew_active_period_);
          first = false;
        }
        if (motion_gate_applied) {
          if (!first) cond << " | ";
          if (tau_fb_motion_gate_speed_source_ == "joint_max") {
            cond << "motion_gate by "
                 << format_joint_flags_with_abs_values(
                      tau_fb_motion_gate_driver_period_,
                      tau_fb_motion_gate_driver_speed_period_,
                      "rad/s")
                 << " min_scale " << std::fixed << std::setprecision(2)
                 << tau_fb_motion_gate_min_scale_period_
                 << " max_speed " << std::setprecision(3)
                 << tau_fb_motion_gate_max_speed_period_ << "rad/s";
          } else {
            cond << "motion_gate by ee_linear v="
                 << std::fixed << std::setprecision(3)
                 << tau_fb_motion_gate_max_speed_period_ << "m/s"
                 << " min_scale " << std::setprecision(2)
                 << tau_fb_motion_gate_min_scale_period_;
          }
          first = false;
        }
        if (passivity_gate_applied) {
          if (!first) cond << " | ";
          cond << "passivity " << format_joint_flags(tau_fb_passivity_gate_joint_period_)
               << " P="
               << std::fixed << std::setprecision(4)
               << tau_fb_passivity_max_power_period_W_ << "W"
               << " min_scale " << std::setprecision(2)
               << tau_fb_passivity_min_scale_period_;
        }
        block << red(cond.str()) << "\n";
      }
      block << std::setprecision(1)
            << "  gravity : " << (runtime_gravity_enabled() ? "ON" : "OFF")
            << ", side " << arm_.side << "\n"
            << "  follower command: "
            << (follower_command_publish_enabled_ ? "ENABLED" : "DISABLED") << "\n"
            << "  scale   : " << format_joint_scale(grav_scale_) << "\n"
            << log_separator() << "\n"
            << "  leader  : " << leader_deg << "\n"
            << "  follower: " << follower_deg << "\n";
      if (last_task_cmd_valid_) {
        block << std::showpos
              << std::fixed << std::setprecision(1)
              << "  task_cmd: x " << std::setw(7) << last_task_cmd_mm_rpy_deg_[0] << " mm | "
              << "y " << std::setw(7) << last_task_cmd_mm_rpy_deg_[1] << " mm | "
              << "z " << std::setw(7) << last_task_cmd_mm_rpy_deg_[2] << " mm | "
              << "rx " << std::setw(7) << last_task_cmd_mm_rpy_deg_[3] << " deg | "
              << "ry " << std::setw(7) << last_task_cmd_mm_rpy_deg_[4] << " deg | "
              << "rz " << std::setw(7) << last_task_cmd_mm_rpy_deg_[5] << " deg\n"
              << std::noshowpos;
      }
      block << log_separator() << "\n";
      block << "  wrench  : "
            << (use_contact_observer_fb_ ? "contact_observer" :
                (use_ft_sensor_feedback_ ? "ft" : "jt_wrench"))
            << " age " << std::fixed << std::setprecision(1) << fe_age_ms << " ms | "
            << wrench << "\n"
            << "  tau[Nm] : grav " << tau_grav << "\n"
            << "            fb   " << tau_fb << "\n"
            << "            damp " << tau_damp << "\n"
            << "            cmd  " << tau_cmd << "\n";
      RCLCPP_INFO(get_logger(), "%s", block.str().c_str());
      if (dxl_read_count_ > 0 && dxl_read_hz + 1e-6 < completed_hz) {
        RCLCPP_WARN(get_logger(),
          "%s dxl_read is slower than loop; commands may use stale DXL samples",
          tag.c_str());
      }
    } else {
      std::ostringstream block;
      block << state_tag(state) << " Node status\n"
            << std::fixed << std::setprecision(1)
            << "  hz      : loop " << completed_hz
            << " | dxl " << dxl_read_hz
            << " | effective " << effective_hz << "\n"
            << "  gravity : " << (runtime_gravity_enabled() ? "ON" : "OFF")
            << ", scale " << format_joint_scale(grav_scale_) << "\n";
      RCLCPP_INFO(get_logger(), "%s", block.str().c_str());
    }
    sat_count_.fill(0);
    sat_ticks_ = 0;
    tau_fb_deadband_active_period_.fill(false);
    tau_fb_slew_active_period_.fill(false);
    tau_fb_motion_gate_active_period_ = false;
    tau_fb_motion_gate_driver_period_.fill(false);
    tau_fb_motion_gate_driver_speed_period_.setZero();
    tau_fb_motion_gate_max_speed_period_ = 0.0;
    tau_fb_motion_gate_min_scale_period_ = 1.0;
    tau_fb_passivity_gate_active_period_ = false;
    tau_fb_passivity_max_power_period_W_ = 0.0;
    tau_fb_passivity_min_scale_period_ = 1.0;
    tau_fb_passivity_gate_joint_period_.fill(false);
    tau_fb_contact_gate_active_period_ = false;
    tau_fb_contact_min_scale_period_ = 1.0;
    tau_fb_contact_max_f_norm_period_N_ = 0.0;
    tau_fb_contact_max_m_norm_period_Nm_ = 0.0;
    dxl_read_count_ = 0;
    dxl_fresh_count_ = 0;
    dxl_fallback_count_ = 0;
    dxl_missing_count_ = 0;
    vel_read_count_ = 0;
    vel_fresh_count_ = 0;
    vel_fallback_count_ = 0;
    vel_missing_count_ = 0;
    teleop_cmd_count_ = 0;

    loop_count_ = 0;
    hz_log_last_t_ = t;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  CSV movement log
// ═══════════════════════════════════════════════════════════════════════════════

void LeaderTeleopNode::init_csv_log() {
  csv_log_enabled_ = get_parameter("csv_log_enabled").as_bool();
  if (!csv_log_enabled_) return;

  const std::string configured_dir = get_parameter("csv_log_dir").as_string();
  const std::filesystem::path dir = resolve_csv_log_dir(configured_dir);
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) {
    RCLCPP_WARN(get_logger(), "[CSV] Failed to create log directory: %s (%s)",
      dir.string().c_str(), ec.message().c_str());
    csv_log_enabled_ = false;
    return;
  }

  std::time_t now_c = std::time(nullptr);
  char stamp[32];
  std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now_c));

  csv_log_path_ = (dir / ("leader_teleop_" + arm_.side + "_" + stamp + ".csv")).string();
  csv_file_.open(csv_log_path_, std::ios::out);
  if (!csv_file_.is_open()) {
    RCLCPP_WARN(get_logger(), "[CSV] Failed to open log file: %s", csv_log_path_.c_str());
    csv_log_enabled_ = false;
    return;
  }

  csv_file_ << "t_s,dt_ms,hz_inst,state,feedback_gain_scale_contract,smooth_teleop_enabled,"
               "follower_command_publish_enabled,"
               "grav_scale_j1,grav_scale_j2,grav_scale_j3,grav_scale_j4,grav_scale_j5,grav_scale_j6,"
               "leader_j1_deg,leader_j2_deg,leader_j3_deg,leader_j4_deg,leader_j5_deg,leader_j6_deg,"
               "follower_j1_deg,follower_j2_deg,follower_j3_deg,follower_j4_deg,follower_j5_deg,follower_j6_deg,"
               "task_raw_x_mm,task_raw_y_mm,task_raw_z_mm,task_raw_rx_deg,task_raw_ry_deg,task_raw_rz_deg,"
               "task_raw_vx_m_s,task_raw_vy_m_s,task_raw_vz_m_s,"
               "task_raw_wx_rad_s,task_raw_wy_rad_s,task_raw_wz_rad_s,"
               "task_intent_x_mm,task_intent_y_mm,task_intent_z_mm,"
               "task_intent_rx_deg,task_intent_ry_deg,task_intent_rz_deg,"
               "task_intent_vx_m_s,task_intent_vy_m_s,task_intent_vz_m_s,"
               "task_intent_wx_rad_s,task_intent_wy_rad_s,task_intent_wz_rad_s,"
               "task_intent_ax_m_s2,task_intent_ay_m_s2,task_intent_az_m_s2,"
               "task_intent_alphax_rad_s2,task_intent_alphay_rad_s2,task_intent_alphaz_rad_s2,"
               "task_command_x_mm,task_command_y_mm,task_command_z_mm,"
               "task_command_rx_deg,task_command_ry_deg,task_command_rz_deg,"
               "fe_raw_fx,fe_raw_fy,fe_raw_fz,fe_raw_mx,fe_raw_my,fe_raw_mz,fe_age_ms,"
               "observer_valid,observer_model_ready,observer_source_age_ms,"
               "observer_contact_state,observer_contact_score_N,"
               "observer_prediction_age_ms,observer_latency_ms,"
               "observer_source_sequence,observer_prediction_sequence,"
               "wb_fx,wb_fy,wb_fz,wb_mx,wb_my,wb_mz,"
               "leader_dq_j1,leader_dq_j2,leader_dq_j3,leader_dq_j4,leader_dq_j5,leader_dq_j6,"
               "tau_grav_j1,tau_grav_j2,tau_grav_j3,tau_grav_j4,tau_grav_j5,tau_grav_j6,"
               "tau_damp_j1,tau_damp_j2,tau_damp_j3,tau_damp_j4,tau_damp_j5,tau_damp_j6,"
               "tau_fb_gate_scale,tau_fb_motion_gate_speed,contact_ee_speed_m_s,contact_joint_speed_rad_s,"
               "contact_state,contact_phase,contact_scale,contact_f_norm_N,contact_m_norm_Nm,"
               "contact_raw_f_norm_N,contact_on_speed_ok,contact_stale_bias_reset,"
               "contact_force_on_eff_N,contact_force_off_eff_N,"
               "contact_moment_on_eff_Nm,contact_moment_off_eff_Nm,"
               "contact_bias_fx,contact_bias_fy,contact_bias_fz,contact_bias_mx,contact_bias_my,contact_bias_mz,"
               "tau_unclip_j1,tau_unclip_j2,tau_unclip_j3,tau_unclip_j4,tau_unclip_j5,tau_unclip_j6,"
               "tau_fb_j1,tau_fb_j2,tau_fb_j3,tau_fb_j4,tau_fb_j5,tau_fb_j6,"
               "tau_cmd_j1,tau_cmd_j2,tau_cmd_j3,tau_cmd_j4,tau_cmd_j5,tau_cmd_j6\n";
  csv_file_.flush();

  RCLCPP_INFO(get_logger(), "[CSV] Logging leader/follower joints to: %s", csv_log_path_.c_str());
}

void LeaderTeleopNode::csv_log_row() {
  if (!csv_log_enabled_ || !csv_file_.is_open()) return;

  double t = now().seconds();
  double dt_ms = (csv_last_t_ < 0.0) ? 0.0 : (t - csv_last_t_) * 1000.0;
  double hz_inst = (dt_ms > 1e-6) ? (1000.0 / dt_ms) : 0.0;
  csv_last_t_ = t;

  Vec6 q_foll;
  bool has_foll = false;
  {
    std::lock_guard<std::mutex> lock(follower_mtx_);
    if (follower_joint_rad_.has_value()) {
      q_foll = follower_joint_rad_.value();
      has_foll = true;
    }
  }

  Vec6 fe_raw;
  double fe_age_ms;
  bool observer_valid = false;
  bool observer_model_ready = false;
  double observer_source_age_ms = -1.0;
  uint8_t observer_contact_state = 0;
  double observer_contact_score_n = 0.0;
  double observer_prediction_age_ms = -1.0;
  double observer_latency_ms = -1.0;
  uint64_t observer_source_sequence = 0;
  uint64_t observer_prediction_sequence = 0;
  if (use_contact_observer_fb_) {
    std::lock_guard<std::mutex> lock(contact_observation_mtx_);
    fe_raw = contact_observation_wrench_;
    fe_age_ms = (contact_observation_receive_stamp_ > 0.0)
      ? (t - contact_observation_receive_stamp_) * 1000.0 : -1.0;
    observer_valid = contact_observation_valid_;
    observer_model_ready = contact_observation_model_ready_;
    observer_source_age_ms = (contact_observation_source_stamp_ > 0.0)
      ? (t - contact_observation_source_stamp_) * 1000.0 : -1.0;
    observer_contact_state = contact_observation_state_;
    observer_contact_score_n = contact_observation_score_n_;
    observer_prediction_age_ms = contact_observation_prediction_age_ms_;
    observer_latency_ms = contact_observation_latency_ms_;
    observer_source_sequence = contact_observation_source_sequence_;
    observer_prediction_sequence = contact_observation_prediction_sequence_;
  } else if (use_ft_sensor_feedback_) {
    std::lock_guard<std::mutex> lock(ft_sensor_mtx_);
    fe_raw = ft_sensor_raw_;
    fe_age_ms = (ft_sensor_stamp_ > 0.0) ? (t - ft_sensor_stamp_) * 1000.0 : -1.0;
  } else {
    std::lock_guard<std::mutex> lock(jt_wrench_mtx_);
    fe_raw = jt_wrench_raw_;
    fe_age_ms = (jt_wrench_stamp_ > 0.0) ? (t - jt_wrench_stamp_) * 1000.0 : -1.0;
  }

  csv_file_ << std::fixed << std::setprecision(4)
            << t << ',' << dt_ms << ',' << hz_inst << ','
            << state_name(state_) << ',' << feedback_gain_scale_contract_
            << ',' << (intent_generator_enabled_ ? 1 : 0)
            << ',' << (follower_command_publish_enabled_ ? 1 : 0);
  for (int i = 0; i < 6; ++i) csv_file_ << ',' << grav_scale_[i];
  for (int i = 0; i < 6; ++i) csv_file_ << ',' << (q_leader_[i] * 180.0 / M_PI);
  for (int i = 0; i < 6; ++i) {
    csv_file_ << ',';
    if (has_foll) csv_file_ << (q_foll[i] * 180.0 / M_PI);
  }
  for (int i = 0; i < 6; ++i) csv_file_ << ',' << last_task_raw_mm_rpy_deg_[i];
  for (int i = 0; i < 6; ++i) csv_file_ << ',' << last_task_raw_velocity_[i];
  for (int i = 0; i < 6; ++i) csv_file_ << ',' << last_task_intent_mm_rpy_deg_[i];
  for (int i = 0; i < 6; ++i) csv_file_ << ',' << last_task_intent_velocity_[i];
  for (int i = 0; i < 6; ++i) csv_file_ << ',' << last_task_intent_acceleration_[i];
  for (int i = 0; i < 6; ++i) csv_file_ << ',' << last_task_cmd_mm_rpy_deg_[i];
  for (int i = 0; i < 6; ++i) csv_file_ << ',' << fe_raw[i];
  csv_file_ << ',' << fe_age_ms;
  csv_file_ << ',' << (observer_valid ? 1 : 0)
            << ',' << (observer_model_ready ? 1 : 0)
            << ',' << observer_source_age_ms
            << ',' << static_cast<int>(observer_contact_state)
            << ',' << observer_contact_score_n
            << ',' << observer_prediction_age_ms
            << ',' << observer_latency_ms
            << ',' << observer_source_sequence
            << ',' << observer_prediction_sequence;
  for (int i = 0; i < 6; ++i) csv_file_ << ',' << last_w_base_[i];
  for (int i = 0; i < 6; ++i) csv_file_ << ',' << leader_dq_[i];
  for (int i = 0; i < 6; ++i) csv_file_ << ',' << last_tau_grav_[i];
  for (int i = 0; i < 6; ++i) csv_file_ << ',' << last_tau_damp_[i];
  csv_file_ << ',' << last_tau_fb_gate_scale_
            << ',' << last_tau_fb_motion_gate_speed_
            << ',' << last_tau_fb_contact_ee_speed_m_s_
            << ',' << last_tau_fb_contact_joint_speed_rad_s_;
  csv_file_ << ',' << (tau_fb_contact_state_ ? 1 : (use_contact_observer_fb_ ? 0 : -1))
            << ',' << tau_fb_contact_phase_
            << ',' << tau_fb_contact_scale_
            << ',' << last_tau_fb_contact_f_norm_N_
            << ',' << last_tau_fb_contact_m_norm_Nm_;
  csv_file_ << ',' << last_tau_fb_contact_raw_f_norm_N_
            << ',' << (last_tau_fb_contact_on_speed_ok_ ? 1 : 0)
            << ',' << (last_tau_fb_contact_stale_bias_reset_ ? 1 : 0);
  csv_file_ << ',' << last_tau_fb_contact_force_on_eff_N_
            << ',' << last_tau_fb_contact_force_off_eff_N_
            << ',' << last_tau_fb_contact_moment_on_eff_Nm_
            << ',' << last_tau_fb_contact_moment_off_eff_Nm_;
  for (int i = 0; i < 6; ++i) csv_file_ << ',' << tau_fb_contact_bias_[i];
  for (int i = 0; i < 6; ++i) csv_file_ << ',' << last_tau_unclipped_[i];
  for (int i = 0; i < 6; ++i) csv_file_ << ',' << last_tau_fb_[i];
  for (int i = 0; i < 6; ++i) csv_file_ << ',' << last_tau_cmd_[i];
  csv_file_ << '\n';

  if (++csv_flush_counter_ >= 100) {
    csv_file_.flush();
    csv_flush_counter_ = 0;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Physical F/T sensor feedback and payload gravity compensation
// ═══════════════════════════════════════════════════════════════════════════════

namespace {

Vec3 parse_xyz_attr(const char* text) {
  Vec3 out = Vec3::Zero();
  if (!text) return out;
  std::istringstream ss(text);
  for (int i = 0; i < 3; ++i) {
    if (!(ss >> out[i])) return Vec3::Zero();
  }
  return out;
}

struct UrdfInertialLite {
  double mass{0.0};
  Vec3 com_local{Vec3::Zero()};
};

}  // namespace

void LeaderTeleopNode::setup_ft_payload_gravity_comp(const std::string& follower_urdf_path) {
  ft_payload_links_.clear();
  if (!use_ft_payload_gravity_comp_) return;
  if (follower_urdf_path.empty() || ft_payload_root_frame_.empty()) {
    RCLCPP_WARN(get_logger(), "[FT PAYLOAD] missing follower URDF or payload root. Disabling payload compensation.");
    use_ft_payload_gravity_comp_ = false;
    return;
  }

  tinyxml2::XMLDocument doc;
  const auto err = doc.LoadFile(follower_urdf_path.c_str());
  if (err != tinyxml2::XML_SUCCESS || doc.RootElement() == nullptr) {
    RCLCPP_WARN(get_logger(), "[FT PAYLOAD] failed to parse URDF XML: %s", follower_urdf_path.c_str());
    use_ft_payload_gravity_comp_ = false;
    return;
  }

  std::unordered_map<std::string, std::vector<std::string>> children_by_parent;
  std::unordered_map<std::string, UrdfInertialLite> inertials_by_link;
  auto* root = doc.RootElement();

  for (auto* joint = root->FirstChildElement("joint"); joint != nullptr;
       joint = joint->NextSiblingElement("joint")) {
    auto* parent = joint->FirstChildElement("parent");
    auto* child = joint->FirstChildElement("child");
    if (!parent || !child) continue;
    const char* parent_link = parent->Attribute("link");
    const char* child_link = child->Attribute("link");
    if (parent_link && child_link) {
      children_by_parent[parent_link].push_back(child_link);
    }
  }

  for (auto* link = root->FirstChildElement("link"); link != nullptr;
       link = link->NextSiblingElement("link")) {
    const char* name = link->Attribute("name");
    if (!name) continue;
    auto* inertial = link->FirstChildElement("inertial");
    if (!inertial) continue;
    auto* mass_node = inertial->FirstChildElement("mass");
    if (!mass_node) continue;
    double mass = 0.0;
    mass_node->QueryDoubleAttribute("value", &mass);
    if (mass <= 0.0) continue;
    Vec3 com_local = Vec3::Zero();
    auto* origin = inertial->FirstChildElement("origin");
    if (origin) com_local = parse_xyz_attr(origin->Attribute("xyz"));
    inertials_by_link[name] = UrdfInertialLite{mass, com_local};
  }

  std::stack<std::string> stack;
  std::unordered_set<std::string> seen;
  stack.push(ft_payload_root_frame_);
  while (!stack.empty()) {
    const std::string link_name = stack.top();
    stack.pop();
    if (!seen.insert(link_name).second) continue;

    const auto it_inertial = inertials_by_link.find(link_name);
    if (it_inertial != inertials_by_link.end()) {
      if (model_follower_.existFrame(link_name)) {
        ft_payload_links_.push_back(PayloadLinkData{
          static_cast<int>(model_follower_.getFrameId(link_name)),
          it_inertial->second.mass,
          it_inertial->second.com_local});
      }
    }

    const auto it_children = children_by_parent.find(link_name);
    if (it_children == children_by_parent.end()) continue;
    for (const auto& child : it_children->second) stack.push(child);
  }

  if (ft_payload_links_.empty()) {
    RCLCPP_WARN(get_logger(),
      "[FT PAYLOAD] no inertial payload links found below root=%s. Disabling payload compensation.",
      ft_payload_root_frame_.c_str());
    use_ft_payload_gravity_comp_ = false;
    return;
  }

  double mass_sum = 0.0;
  for (const auto& link : ft_payload_links_) mass_sum += link.mass;
  RCLCPP_INFO(get_logger(), "[FT PAYLOAD] root=%s links=%zu mass=%.3fkg sign=[%.1f %.1f %.1f %.1f %.1f %.1f]",
    ft_payload_root_frame_.c_str(), ft_payload_links_.size(), mass_sum,
    ft_payload_gravity_sign_[0], ft_payload_gravity_sign_[1], ft_payload_gravity_sign_[2],
    ft_payload_gravity_sign_[3], ft_payload_gravity_sign_[4], ft_payload_gravity_sign_[5]);
}

Vec6 LeaderTeleopNode::compute_ft_payload_gravity_wrench() {
  last_ft_payload_gravity_.setZero();
  if (!use_ft_payload_gravity_comp_ || ft_payload_links_.empty() || ft_sensor_frame_id_ < 0) {
    return Vec6::Zero();
  }

  Vec6 q_foll;
  {
    std::lock_guard<std::mutex> lock(follower_mtx_);
    if (!follower_joint_rad_.has_value()) return Vec6::Zero();
    q_foll = follower_joint_rad_.value();
  }

  try {
    std::lock_guard<std::mutex> lock(pin_follower_mtx_);
    Eigen::VectorXd q_full_foll = Eigen::VectorXd::Zero(model_follower_.nq);
    for (int i = 0; i < 6; ++i) q_full_foll[idxq_follower_[i]] = q_foll[i];

    pinocchio::forwardKinematics(model_follower_, data_follower_, q_full_foll);
    pinocchio::updateFramePlacements(model_follower_, data_follower_);

    const auto& oMs = data_follower_.oMf[ft_sensor_frame_id_];
    const Vec3 p_sensor_world = oMs.translation();
    const Mat3 R_world_sensor = oMs.rotation();
    const Vec3 gravity_world = model_follower_.gravity.linear();

    Vec3 force_world_total = Vec3::Zero();
    Vec3 moment_world_total = Vec3::Zero();
    for (const auto& payload : ft_payload_links_) {
      if (payload.frame_id < 0 || payload.frame_id >= static_cast<int>(model_follower_.nframes)) continue;
      const auto& oMl = data_follower_.oMf[payload.frame_id];
      const Vec3 p_com_world = oMl.translation() + oMl.rotation() * payload.com_local;
      const Vec3 force_world = payload.mass * gravity_world;
      force_world_total += force_world;
      moment_world_total += (p_com_world - p_sensor_world).cross(force_world);
    }

    Vec6 wrench_ft;
    wrench_ft << R_world_sensor.transpose() * force_world_total,
                 R_world_sensor.transpose() * moment_world_total;
    last_ft_payload_gravity_ = ft_payload_gravity_sign_.cwiseProduct(wrench_ft);
    return last_ft_payload_gravity_;
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "[FT PAYLOAD] gravity wrench computation failed: %s", e.what());
    return Vec6::Zero();
  }
}

void LeaderTeleopNode::ft_sensor_tare_step() {
  if (!use_ft_sensor_feedback_) return;

  double stamp = 0.0;
  Vec6 raw = Vec6::Zero();
  {
    std::lock_guard<std::mutex> lock(ft_sensor_mtx_);
    if (!ft_sensor_tare_req_) return;
    stamp = ft_sensor_stamp_;
    raw = ft_sensor_raw_;
    if (stamp <= 0.0 || now().seconds() - stamp > ft_stale_timeout_) return;
    if (stamp == ft_sensor_tare_last_stamp_) return;
    ft_sensor_tare_last_stamp_ = stamp;
  }

  {
    std::lock_guard<std::mutex> lock(ft_sensor_mtx_);
    if (!ft_sensor_tare_req_) return;
    ft_sensor_tare_accum_ += raw;
    ++ft_sensor_tare_count_;
    if (ft_sensor_tare_count_ >= ft_tare_N_) {
      ft_sensor_baseline_ = ft_sensor_tare_accum_ / static_cast<double>(std::max(1, ft_sensor_tare_count_));
      ft_sensor_tare_req_ = false;
      ft_sensor_tare_count_ = 0;
      ft_sensor_tare_accum_.setZero();
      RCLCPP_INFO(get_logger(), "[FT TARE] done baseline[Fx Fy Fz Mx My Mz]=[%.3f %.3f %.3f %.3f %.3f %.3f]",
        ft_sensor_baseline_[0], ft_sensor_baseline_[1], ft_sensor_baseline_[2],
        ft_sensor_baseline_[3], ft_sensor_baseline_[4], ft_sensor_baseline_[5]);
    }
  }
}

Vec6 LeaderTeleopNode::compute_ft_sensor_feedback() {
  Vec6 w_spatial;
  Eigen::MatrixXd J_space;
  if (!compute_reflected_wrench_and_jacobian(w_spatial, J_space)) {
    return Vec6::Zero();
  }

  Eigen::VectorXd tau_full = J_space.transpose() * w_spatial;
  Vec6 tau_ext;
  for (int i = 0; i < 6; ++i) tau_ext[i] = tau_full[idxv_leader_[i]];
  return ft_fb_gain_.cwiseProduct(tau_ext);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Utility
// ═══════════════════════════════════════════════════════════════════════════════

Vec6 LeaderTeleopNode::clip_follower_joint_rad(const Vec6& leader_q) {
  // leader→follower: q_follower = S * q_leader + offset
  return arm_.joint_signs.cwiseProduct(leader_q) + arm_.offset_rad;
}

Eigen::Vector4d LeaderTeleopNode::rotation_to_quat_xyzw(const Mat3& R) {
  double tr = R.trace();
  double qx, qy, qz, qw;

  if (tr > 0.0) {
    double s = std::sqrt(tr + 1.0) * 2.0;
    qw = 0.25 * s;
    qx = (R(2, 1) - R(1, 2)) / s;
    qy = (R(0, 2) - R(2, 0)) / s;
    qz = (R(1, 0) - R(0, 1)) / s;
  } else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
    double s = std::sqrt(std::max(1.0 + R(0, 0) - R(1, 1) - R(2, 2), 1e-12)) * 2.0;
    qw = (R(2, 1) - R(1, 2)) / s;
    qx = 0.25 * s;
    qy = (R(0, 1) + R(1, 0)) / s;
    qz = (R(0, 2) + R(2, 0)) / s;
  } else if (R(1, 1) > R(2, 2)) {
    double s = std::sqrt(std::max(1.0 + R(1, 1) - R(0, 0) - R(2, 2), 1e-12)) * 2.0;
    qw = (R(0, 2) - R(2, 0)) / s;
    qx = (R(0, 1) + R(1, 0)) / s;
    qy = 0.25 * s;
    qz = (R(1, 2) + R(2, 1)) / s;
  } else {
    double s = std::sqrt(std::max(1.0 + R(2, 2) - R(0, 0) - R(1, 1), 1e-12)) * 2.0;
    qw = (R(1, 0) - R(0, 1)) / s;
    qx = (R(0, 2) + R(2, 0)) / s;
    qy = (R(1, 2) + R(2, 1)) / s;
    qz = 0.25 * s;
  }

  Eigen::Vector4d q(qx, qy, qz, qw);
  double n = q.norm();
  if (!std::isfinite(n) || n < 1e-12) return Eigen::Vector4d(0, 0, 0, 1);
  return q / n;
}

}  // namespace teleop_cpp
