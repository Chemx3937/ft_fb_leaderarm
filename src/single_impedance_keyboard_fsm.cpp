// single_impedance_keyboard_fsm.cpp
// Single impedance teleop 키보드 FSM + 상태전환 + slow sync + torque→current 변환 + follower init pose
//
// 원본 _handle_keyboard_input(), _slow_sync_targets(), _send_torque_currents() 대응
//  - getch: termios 1회 초기화 + atexit 복원 (원본: 매 루프 set/reset → syscall 제거)
//  - 상태전환: c→current_ready, t→slow_sync, o→fast, s→pause, z→init pose,
//              r→re-align, g/f/j→gain toggle, q→return-to-zero shutdown
//  - slow_sync: capture_delay 후 velocity-limited tracking (원본과 동일)
//  - send_torque_currents: tau(Nm) → current(A) / Kt → DXL unit / cu → clip → SyncWrite

#include "ft_fb_leaderarm/single_impedance_teleop_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <chrono>
#include <limits>
#include <map>
#include <iomanip>
#include <sstream>

namespace teleop_cpp {

// Set terminal to raw non-blocking once at startup, restore at exit
static struct termios g_orig_termios;
static bool g_termios_saved = false;
static bool g_keyboard_setup = false;
static int g_keyboard_fd = -1;
static int g_orig_flags = -1;
static bool g_close_keyboard_fd = false;

static void restore_keyboard_input() {
  if (g_keyboard_fd < 0) return;
  if (g_termios_saved) {
    tcsetattr(g_keyboard_fd, TCSANOW, &g_orig_termios);
  }
  if (g_orig_flags >= 0) {
    fcntl(g_keyboard_fd, F_SETFL, g_orig_flags);
  }
  if (g_close_keyboard_fd) {
    close(g_keyboard_fd);
  }
  g_keyboard_fd = -1;
}

static void setup_raw_terminal() {
  if (g_keyboard_setup) return;
  g_keyboard_setup = true;

  g_keyboard_fd = open("/dev/tty", O_RDONLY | O_NONBLOCK);
  if (g_keyboard_fd >= 0) {
    g_close_keyboard_fd = true;
  } else {
    g_keyboard_fd = STDIN_FILENO;
  }

  int flags = fcntl(g_keyboard_fd, F_GETFL, 0);
  if (flags >= 0) {
    g_orig_flags = flags;
    fcntl(g_keyboard_fd, F_SETFL, flags | O_NONBLOCK);
  }

  if (tcgetattr(g_keyboard_fd, &g_orig_termios) == 0) {
    g_termios_saved = true;

    struct termios raw = g_orig_termios;
    raw.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(g_keyboard_fd, TCSANOW, &raw);
  }

  std::atexit(restore_keyboard_input);
}

static int getch_nonblock() {
  setup_raw_terminal();
  if (g_keyboard_fd < 0) return -1;
  unsigned char ch = 0;
  const ssize_t n = ::read(g_keyboard_fd, &ch, 1);
  return n == 1 ? static_cast<int>(ch) : -1;
}

static std::string json_string(const std::string& value) {
  std::ostringstream out;
  out << '"';
  for (const unsigned char ch : value) {
    switch (ch) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (ch < 0x20) {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(ch) << std::dec << std::setfill(' ');
        } else {
          out << static_cast<char>(ch);
        }
    }
  }
  out << '"';
  return out.str();
}

static void append_json_vec6(std::ostringstream& out, const Vec6& values) {
  out << '[';
  for (int i = 0; i < 6; ++i) {
    if (i) out << ',';
    out << values[i];
  }
  out << ']';
}

// ═══════════════════════════════════════════════════════════════════════════════

bool LeaderTeleopNode::poll_shutdown_key(const char* context) {
  bool service_request = shutdown_command_pending_.exchange(false);
  bool keyboard_request = false;
  if (!service_request && keyboard_input_enabled_) {
    const int ch = getch_nonblock();
    if (ch >= 0) {
      const char key = static_cast<char>(ch);
      keyboard_request = key == 'q';
      if (!keyboard_request &&
          (key == 'c' || key == 't' || key == 'o' || key == 's' ||
           key == 'z' || key == 'r')) {
        const std::string msg = std::string("blocking transition 중 명령 거부") +
          " | context=" + (context ? context : "-");
        log_control_command_audit(
          "terminal_keyboard", key, "blocking_transition", false, msg);
      }
    }
  }
  if (!service_request && !keyboard_request) return false;

  if (service_request) {
    int queued = static_cast<int>('q');
    pending_control_command_.compare_exchange_strong(queued, 0);
  }

  const std::string origin = service_request ? "ROS service" : "key 'q'";
  const std::string msg = std::string("[SHUTDOWN] requested by ") + origin + " | state=" +
    teleop_state_name() + " | context=" + (context ? context : "-");
  log_control_command_audit(
    service_request ? "ros_service" : "terminal_keyboard",
    'q', "blocking_shutdown", true, msg);
  RCLCPP_INFO(get_logger(), "%s", blue(msg).c_str());
  return_to_zero_and_shutdown();
  return true;
}

void LeaderTeleopNode::log_control_command_audit(
    const char* input_source, char key, const char* phase,
    bool accepted, const std::string& message) {
  const char* source = input_source ? input_source : "unknown";
  const char* audit_phase = phase ? phase : "unknown";
  const char* result = accepted ? "accepted" : "rejected";
  // ROS service callbacks only enqueue work and run outside the control-loop
  // callback group.  Do not read non-atomic FSM state from that thread.
  const bool enqueue_phase = std::string(audit_phase) == "enqueue";
  const char* state_label = enqueue_phase ? "QUEUED" : teleop_state_name();
  if (accepted) {
    RCLCPP_INFO(get_logger(),
      "[COMMAND AUDIT] source=%s key=%c phase=%s result=%s state=%s message=%s",
      source, key, audit_phase, result, state_label, message.c_str());
  } else {
    RCLCPP_WARN(get_logger(),
      "[COMMAND AUDIT] source=%s key=%c phase=%s result=%s state=%s message=%s",
      source, key, audit_phase, result, state_label, message.c_str());
  }
}

void LeaderTeleopNode::handle_keyboard() {
  if (!keyboard_input_enabled_) return;
  int ch = getch_nonblock();
  if (ch < 0) return;

  char key = static_cast<char>(ch);

  if (key == 'c' || key == 't' || key == 'o' || key == 's' ||
      key == 'z' || key == 'r' || key == 'q') {
    active_control_command_ = static_cast<int>(key);
    std::string message;
    const bool accepted = handle_control_command(key, "terminal_keyboard", message);
    active_control_command_ = 0;
    if (!accepted) RCLCPP_WARN(get_logger(), "[KEY] %s", message.c_str());
    return;
  }

  switch (key) {
    case 'g': {
      if (state_ != TeleopState::FAST) {
        RCLCPP_WARN(get_logger(), "[KEY] 'g' is only active in FAST.");
        break;
      }
      const bool off = arm_.grav_gain.cwiseAbs().maxCoeff() <= 1e-12;
      arm_.grav_gain = off ? grav_gain_base_ : Vec6::Zero();
      RCLCPP_INFO(get_logger(),
        "[GAIN] grav_gain %s -> [%.3f,%.3f,%.3f,%.3f,%.3f,%.3f] | runtime=%s",
        off ? "RESTORE" : "ZERO",
        arm_.grav_gain[0], arm_.grav_gain[1], arm_.grav_gain[2],
        arm_.grav_gain[3], arm_.grav_gain[4], arm_.grav_gain[5],
        runtime_gravity_enabled() ? "ON" : "OFF");
      break;
    }

    case 'f': {
      if (state_ != TeleopState::FAST) {
        RCLCPP_WARN(get_logger(), "[KEY] 'f' is only active in FAST.");
        break;
      }

      Vec6* gain = nullptr;
      Vec6* base = nullptr;
      const char* label = nullptr;
      if (use_jt_wrench_fb_ || use_contact_observer_fb_) {
        gain = &arm_.jt_wrench_fb_gain;
        base = &jt_wrench_fb_gain_base_;
        label = use_contact_observer_fb_
          ? "contact_observer_fb_gain"
          : "jt_wrench_fb_gain";
      } else if (use_ft_sensor_feedback_) {
        gain = &ft_fb_gain_;
        base = &ft_fb_gain_base_;
        label = "ft_fb_gain";
      }

      if (!gain || !base) {
        RCLCPP_WARN(get_logger(), "[KEY] No active wrench feedback gain to toggle.");
        break;
      }

      const bool off = gain->cwiseAbs().maxCoeff() <= 1e-12;
      *gain = off ? *base : Vec6::Zero();
      if (use_jt_wrench_fb_ || use_contact_observer_fb_) {
        jt_wrench_fb_gain_ = arm_.jt_wrench_fb_gain;
      }
      RCLCPP_INFO(get_logger(),
        "[GAIN] %s %s -> [%.3f,%.3f,%.3f,%.3f,%.3f,%.3f] | runtime=%s",
        label, off ? "RESTORE" : "ZERO",
        (*gain)[0], (*gain)[1], (*gain)[2], (*gain)[3], (*gain)[4], (*gain)[5],
        runtime_feedback_enabled() ? "ON" : "OFF");
      break;
    }

    case 'j': {
      if (state_ != TeleopState::FAST) {
        RCLCPP_WARN(get_logger(), "[KEY] 'j' is only active in FAST.");
        break;
      }
      if (!use_jt_wrench_fb_) {
        RCLCPP_WARN(get_logger(), "[KEY] 'j' requires JT wrench feedback to be active.");
        break;
      }

      const bool off = arm_.jt_wrench_fb_gain.cwiseAbs().maxCoeff() <= 1e-12;
      arm_.jt_wrench_fb_gain = off ? jt_wrench_fb_gain_base_ : Vec6::Zero();
      jt_wrench_fb_gain_ = arm_.jt_wrench_fb_gain;
      RCLCPP_INFO(get_logger(),
        "[GAIN] jt_wrench_fb_gain %s -> [%.3f,%.3f,%.3f,%.3f,%.3f,%.3f] | runtime=%s",
        off ? "RESTORE" : "ZERO",
        arm_.jt_wrench_fb_gain[0], arm_.jt_wrench_fb_gain[1], arm_.jt_wrench_fb_gain[2],
        arm_.jt_wrench_fb_gain[3], arm_.jt_wrench_fb_gain[4], arm_.jt_wrench_fb_gain[5],
        runtime_feedback_enabled() ? "ON" : "OFF");
      break;
    }

    default:
      break;
  }
}

bool LeaderTeleopNode::handle_control_command(
    char key, const char* input_source, std::string& message) {
  message.clear();
  log_control_command_audit(
    input_source, key, "received", true, "control command received");
  auto finish = [&](bool accepted) {
    log_control_command_audit(
      input_source, key, "completed", accepted, message);
    return accepted;
  };

  if (state_ == TeleopState::SHUTDOWN && key != 'q') {
    message = "SHUTDOWN 상태에서는 명령을 받을 수 없습니다.";
    last_control_message_ = message;
    return finish(false);
  }
  if (dxl_fault_active_ && key != 'r' && key != 'q') {
    message = "DXL fault 상태에서는 REALIGN(r) 또는 SHUTDOWN(q)만 사용할 수 있습니다.";
    if (!dxl_fault_reason_.empty()) message += " 원인: " + dxl_fault_reason_;
    last_control_message_ = message;
    return finish(false);
  }
  if (state_ == TeleopState::INIT_POSE && key != 'z' && key != 'r' && key != 'q') {
    message = "INIT POSE가 진행 중입니다. z, r, q만 사용할 수 있습니다.";
    last_control_message_ = message;
    return finish(false);
  }

  const TeleopState before = state_;
  switch (key) {
    case 'c': transition_to_current_ready(); break;
    case 't': transition_to_slow_sync(); break;
    case 'o': transition_to_fast(); break;
    case 's': pause_teleop(); break;
    case 'z': send_follower_to_init_pose(); break;
    case 'q':
      shutdown_command_pending_ = false;
      message = "SHUTDOWN 요청을 수락했습니다.";
      last_control_message_ = message;
      finish(true);
      return_to_zero_and_shutdown();
      return true;
    case 'r':
      if (init_pose_in_progress_.load()) {
        message = "INIT POSE 이동이 끝난 뒤 REALIGN할 수 있습니다.";
        last_control_message_ = message;
        return finish(false);
      }
      RCLCPP_INFO(get_logger(), "[CONTROL] Re-aligning leader → follower...");
      state_ = TeleopState::INIT;
      aligned_once_ = false;
      impedance_last_pos_.reset();
      impedance_last_R_.reset();
      if (!align_leader_to_follower()) {
        if (state_ == TeleopState::SHUTDOWN || shutdown_requested_) {
          message = "REALIGN 중 SHUTDOWN 요청을 처리했습니다.";
        } else {
          message = last_control_message_.empty()
            ? "REALIGN에 실패했습니다. DXL 오류 로그를 확인하세요."
            : last_control_message_;
        }
        last_control_message_ = message;
        return finish(false);
      }
      message = "REALIGN을 완료했습니다.";
      last_control_message_ = message;
      return finish(true);
    default:
      message = "지원하지 않는 control command입니다.";
      last_control_message_ = message;
      return finish(false);
  }

  if (dxl_fault_active_) {
    message = last_control_message_.empty()
      ? "DXL fault가 발생했습니다. REALIGN이 필요합니다."
      : last_control_message_;
    last_control_message_ = message;
    return finish(false);
  }

  const bool accepted = state_ != before ||
    (key == 's' && state_ == TeleopState::PAUSED) ||
    (key == 'z' && state_ == TeleopState::INIT_POSE);
  std::ostringstream stream;
  if (accepted) {
    stream << "명령 '" << key << "' 수락: " << teleop_state_name();
  } else {
    stream << "명령 '" << key << "' 거부: 현재 상태 " << teleop_state_name()
           << "와 readiness 조건을 확인하세요.";
  }
  message = stream.str();
  last_control_message_ = message;
  return finish(accepted);
}

bool LeaderTeleopNode::enqueue_control_command(char key, std::string& message) {
  if (key == 'q') {
    if (shutdown_started_.load()) {
      message = "SHUTDOWN이 이미 진행 중입니다.";
      return true;
    }
    shutdown_command_pending_ = true;
    pending_control_command_.store(static_cast<int>('q'));
    message = "SHUTDOWN 요청을 큐에 등록했습니다.";
    return true;
  }

  if (shutdown_requested_.load() || shutdown_command_pending_.load()) {
    message = "SHUTDOWN 요청이 진행 중이어서 다른 명령을 받을 수 없습니다.";
    return false;
  }

  const int active = active_control_command_.load();
  if (active != 0) {
    message = std::string("명령 '") + static_cast<char>(active) +
      "' 처리 중입니다. 완료될 때까지 기다리세요.";
    return false;
  }

  int expected = 0;
  if (!pending_control_command_.compare_exchange_strong(
        expected, static_cast<int>(key))) {
    message = std::string("명령 '") + static_cast<char>(expected) +
      "' 대기 중입니다. 반복 입력하지 마세요.";
    return false;
  }
  message = std::string("명령 '") + key + "' 요청을 큐에 등록했습니다.";
  return true;
}

void LeaderTeleopNode::process_pending_control_commands() {
  // Two passes allow a q service request that arrives during blocking REALIGN
  // to run immediately after the alignment loop observes and aborts for it.
  for (int pass = 0; pass < 2; ++pass) {
    const int queued = pending_control_command_.exchange(0);
    if (queued == 0) return;

    const char key = static_cast<char>(queued);
    if (key == 'q') shutdown_command_pending_ = false;
    active_control_command_ = queued;
    std::string message;
    const bool accepted = handle_control_command(key, "ros_service", message);
    active_control_command_ = 0;
    if (!accepted) {
      RCLCPP_WARN(get_logger(), "[CONTROL] %s", message.c_str());
    }
    if (key == 'q' || shutdown_requested_.load()) return;
  }
}

void LeaderTeleopNode::handle_control_service(
    char key,
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  response->success = enqueue_control_command(key, response->message);
  log_control_command_audit(
    "ros_service", key, "enqueue", response->success, response->message);
}

void LeaderTeleopNode::publish_status() {
  if (!pub_status_) return;

  const auto steady_now = std::chrono::steady_clock::now();
  const double ros_now = now().seconds();
  std::optional<Vec6> follower;
  double follower_age_s = -1.0;
  {
    std::lock_guard<std::mutex> lock(follower_mtx_);
    follower = follower_joint_rad_;
    if (follower_joint_receive_steady_valid_) {
      follower_age_s = std::chrono::duration<double>(
        steady_now - follower_joint_receive_steady_).count();
    }
  }
  const bool follower_fresh = follower.has_value() && follower_age_s >= 0.0 &&
    follower_age_s <= follower_joint_stale_timeout_;
  Vec6 error = Vec6::Zero();
  if (follower.has_value()) {
    error = last_follower_target_ - follower.value();
  }

  std::string observer_reason;
  const bool observer_ready = contact_observation_ready_for_teleop(observer_reason);
  std::string contact_status = "WAITING";
  int contact_state = 0;
  double observer_age_s = -1.0;
  {
    std::lock_guard<std::mutex> lock(contact_observation_mtx_);
    contact_state = contact_observation_state_;
    if (contact_observation_receive_steady_valid_) {
      observer_age_s = std::chrono::duration<double>(
        steady_now - contact_observation_receive_steady_).count();
    }
    if (!contact_observation_receive_steady_valid_) {
      contact_status = "WAITING";
    } else if (!contact_observation_model_ready_) {
      contact_status = "MODEL NOT READY";
    } else if (observer_age_s > contact_observation_stale_timeout_) {
      contact_status = "STALE";
    } else if (!contact_observation_valid_) {
      contact_status = "INVALID";
    } else if (contact_state == contact_observer_msgs::msg::ContactObservation::CONTACT) {
      contact_status = "CONTACT";
    } else {
      contact_status = "FREE";
    }
  }

  std::vector<std::string> blockers;
  std::string next_command;
  bool next_ready = false;
  if (dxl_fault_active_) {
    next_command = "REALIGN";
    next_ready = false;
  } else if (state_ == TeleopState::ALIGN || state_ == TeleopState::INIT) {
    next_command = "CURRENT";
    next_ready = aligned_once_ && follower_fresh;
    if (!aligned_once_) blockers.emplace_back("leader/follower ALIGN 진행 중");
  } else if (state_ == TeleopState::MAIN_IDLE || state_ == TeleopState::PAUSED) {
    next_command = dxl_current_mode_ ? "SLOW" : "CURRENT";
    next_ready = follower_fresh && (dxl_current_mode_ ? observer_ready : true);
  } else if (state_ == TeleopState::CURRENT_READY) {
    next_command = "SLOW";
    next_ready = follower_fresh && observer_ready;
  } else if (state_ == TeleopState::SLOW_SYNC) {
    next_command = "FAST";
    next_ready = follower_fresh && observer_ready && slow_sync_ready_;
    for (int i = 0; i < 6; ++i) {
      if (!follower.has_value() || std::abs(error[i]) >= slow_sync_ready_tol_rad_) {
        blockers.emplace_back("J" + std::to_string(i + 1) + " tolerance 초과");
      }
    }
    if (blockers.empty() && !slow_sync_ready_) blockers.emplace_back("0.5초 hold 진행 중");
  } else if (state_ == TeleopState::INIT_POSE) {
    next_command = "REALIGN";
    next_ready = !init_pose_in_progress_.load();
  } else if (state_ == TeleopState::FAST) {
    next_command = "PAUSE";
    next_ready = true;
  }
  if (!follower_fresh && state_ != TeleopState::SHUTDOWN) {
    blockers.emplace_back("follower joint stale");
  }
  if (!observer_ready &&
      (state_ == TeleopState::CURRENT_READY || state_ == TeleopState::SLOW_SYNC ||
       state_ == TeleopState::PAUSED)) {
    blockers.emplace_back(observer_reason);
  }
  if (dxl_fault_active_) {
    next_ready = false;
    blockers.emplace_back(dxl_fault_reason_);
  }

  double hold_progress = slow_sync_ready_ ? 1.0 : 0.0;
  if (!slow_sync_ready_ && slow_sync_ready_since_ > 0.0 && slow_sync_ready_hold_sec_ > 0.0) {
    hold_progress = std::clamp(
      (ros_now - slow_sync_ready_since_) / slow_sync_ready_hold_sec_, 0.0, 1.0);
  }

  std::ostringstream out;
  out << std::fixed << std::setprecision(8)
      << "{\"schema_version\":1,\"stamp\":" << ros_now
      << ",\"state\":" << json_string(teleop_state_name())
      << ",\"feedback_gain_scale_contract\":" << feedback_gain_scale_contract_
      << ",\"smooth_teleop_enabled\":"
      << (intent_generator_enabled_ ? "true" : "false")
      << ",\"follower_command_publish_enabled\":"
      << (follower_command_publish_enabled_ ? "true" : "false")
      << ",\"contact\":" << json_string(contact_status)
      << ",\"contact_state\":" << contact_state
      << ",\"observer_ready\":" << (observer_ready ? "true" : "false")
      << ",\"observer_age_s\":" << observer_age_s
      << ",\"observer_reason\":" << json_string(observer_reason)
      << ",\"follower_valid\":" << (follower.has_value() ? "true" : "false")
      << ",\"follower_fresh\":" << (follower_fresh ? "true" : "false")
      << ",\"follower_age_s\":" << follower_age_s
      << ",\"dxl_fault\":" << (dxl_fault_active_ ? "true" : "false")
      << ",\"dxl_fault_reason\":" << json_string(dxl_fault_reason_)
      << ",\"tolerance_rad\":" << slow_sync_ready_tol_rad_
      << ",\"hold_sec\":" << slow_sync_ready_hold_sec_
      << ",\"hold_progress\":" << hold_progress
      << ",\"slow_ready\":" << (slow_sync_ready_ ? "true" : "false")
      << ",\"next_command\":" << json_string(next_command)
      << ",\"next_ready\":" << (next_ready ? "true" : "false")
      << ",\"last_control_message\":" << json_string(last_control_message_)
      << ",\"leader_rad\":";
  append_json_vec6(out, q_leader_);
  out << ",\"mapped_target_rad\":";
  append_json_vec6(out, last_follower_target_);
  out << ",\"follower_rad\":";
  if (follower.has_value()) append_json_vec6(out, follower.value()); else out << "null";
  out << ",\"error_rad\":";
  append_json_vec6(out, error);
  out << ",\"over_tolerance\":[";
  for (int i = 0; i < 6; ++i) {
    if (i) out << ',';
    out << ((!follower.has_value() || std::abs(error[i]) >= slow_sync_ready_tol_rad_)
      ? "true" : "false");
  }
  out << "],\"blockers\":[";
  for (std::size_t i = 0; i < blockers.size(); ++i) {
    if (i) out << ',';
    out << json_string(blockers[i]);
  }
  out << "]}";
  std_msgs::msg::String msg;
  msg.data = out.str();
  pub_status_->publish(msg);
}

void LeaderTeleopNode::publish_status_if_due() {
  if (!rclcpp::ok()) return;
  const auto steady_now = std::chrono::steady_clock::now();
  const double period_s = 1.0 / std::max(1.0, status_publish_hz_);
  if (last_status_publish_steady_valid_ &&
      std::chrono::duration<double>(
        steady_now - last_status_publish_steady_).count() < period_s) {
    return;
  }
  last_status_publish_steady_ = steady_now;
  last_status_publish_steady_valid_ = true;
  publish_status();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  State transitions
// ═══════════════════════════════════════════════════════════════════════════════

bool LeaderTeleopNode::follower_joint_ready_for_teleop(std::string& reason) {
  reason.clear();
  const auto steady_now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(follower_mtx_);
  if (!follower_joint_rad_.has_value() || !follower_joint_receive_steady_valid_) {
    reason = "follower joint state unavailable";
    return false;
  }
  const double age = std::chrono::duration<double>(
    steady_now - follower_joint_receive_steady_).count();
  if (age < 0.0 || age > follower_joint_stale_timeout_) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2)
           << "follower joint state stale (age=" << age * 1000.0 << "ms)";
    reason = stream.str();
    return false;
  }
  return true;
}

bool LeaderTeleopNode::contact_observation_ready_for_teleop(
    std::string& reason) {
  reason.clear();
  if (!use_contact_observer_fb_) return true;

  const auto steady_now = std::chrono::steady_clock::now();
  const double ros_now = now().seconds();
  std::lock_guard<std::mutex> lock(contact_observation_mtx_);

  if (!contact_observation_receive_steady_valid_) {
    reason = "no ContactObservation received";
    return false;
  }
  if (!contact_observation_model_ready_) {
    reason = "Observer model is not ready";
    return false;
  }
  if (!contact_observation_valid_) {
    reason = "Observer is calibrating or invalid";
    return false;
  }

  const double local_age = std::chrono::duration<double>(
    steady_now - contact_observation_receive_steady_).count();
  const double source_age = ros_now - contact_observation_source_stamp_;
  const bool fresh =
    local_age >= 0.0 && local_age <= contact_observation_stale_timeout_ &&
    source_age >= -contact_observation_clock_future_tolerance_ &&
    source_age <= contact_observation_stale_timeout_;
  if (!fresh) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2)
           << "ContactObservation is stale (local=" << local_age * 1000.0
           << "ms, source=" << source_age * 1000.0 << "ms)";
    reason = stream.str();
    return false;
  }
  return true;
}

void LeaderTeleopNode::transition_to_current_ready() {
  if (state_ == TeleopState::FAST || state_ == TeleopState::SLOW_SYNC) {
    RCLCPP_WARN(get_logger(), "[KEY] Already in active state. Press 's' first.");
    return;
  }
  if (!aligned_once_) {
    RCLCPP_WARN(get_logger(),
      "[KEY] CURRENT blocked: leader/follower ALIGN is not verified. Press 'r'.");
    return;
  }
  std::string follower_reason;
  if (!follower_joint_ready_for_teleop(follower_reason)) {
    RCLCPP_WARN(get_logger(), "[KEY] CURRENT blocked: %s", follower_reason.c_str());
    return;
  }

  // Safe phased current-mode transition:
  //   all torque OFF -> all mode=0 -> all goal current=0 -> all torque ON.
  // Every non-real-time register write is ID-by-ID, retried, and read back.
  if (!bus_->prepare_operating_mode(arm_.dxl_ids, 0)) {
    enter_dxl_fault("CURRENT torque-off/mode preparation failed: " + bus_->last_error());
    return;
  }
  std::map<int, int16_t> zero_current;
  for (int id : arm_.dxl_ids) zero_current[id] = 0;
  if (!bus_->write_goal_currents_verified(zero_current)) {
    enter_dxl_fault("CURRENT verified zero-current preload failed: " + bus_->last_error());
    return;
  }
  if (!bus_->torque_on(arm_.dxl_ids)) {
    enter_dxl_fault("CURRENT verified torque-on failed: " + bus_->last_error());
    return;
  }
  dxl_current_mode_ = true;
  dxl_fault_active_ = false;
  dxl_fault_reason_.clear();
  grav_scale_target_ = grav_sync_scale_;
  state_ = TeleopState::CURRENT_READY;
  RCLCPP_INFO(get_logger(), "%s DXL current mode ON (grav scale target: %s)",
    state_tag("CURRENT").c_str(), format_joint_scale(grav_sync_scale_).c_str());
}

void LeaderTeleopNode::transition_to_slow_sync() {
  if (!dxl_current_mode_ ||
      (state_ != TeleopState::CURRENT_READY && state_ != TeleopState::MAIN_IDLE &&
       state_ != TeleopState::PAUSED)) {
    RCLCPP_WARN(get_logger(), "[KEY] Press 'c' first to enter current_ready.");
    return;
  }

  std::string follower_reason;
  if (!follower_joint_ready_for_teleop(follower_reason)) {
    RCLCPP_WARN(get_logger(), "[KEY] SLOW blocked: %s", follower_reason.c_str());
    return;
  }

  std::string observer_reason;
  if (!contact_observation_ready_for_teleop(observer_reason)) {
    RCLCPP_WARN(
      get_logger(),
      "[KEY] SLOW blocked: %s. Keep the follower stationary/contact-free "
      "until GUI valid=True and press 't' again.",
      observer_reason.c_str());
    return;
  }

  // Reset slow sync state
  slow_sync_cmd_.reset();
  slow_sync_last_t_ = 0.0;
  slow_sync_ready_since_ = 0.0;
  slow_sync_ready_ = false;
  slow_sync_capture_until_ = now().seconds() + slow_sync_capture_delay_s_;

  state_ = TeleopState::SLOW_SYNC;
  RCLCPP_INFO(get_logger(),
    "%s max_vel=%.1fdeg/s | ready_tol=%.1fdeg | ready_hold=%.1fs",
    state_tag("SLOW").c_str(),
    slow_sync_config_.max_vel_deg_s,
    slow_sync_config_.ready_tol_deg,
    slow_sync_config_.ready_hold_sec);
}

void LeaderTeleopNode::transition_to_fast() {
  if (state_ != TeleopState::SLOW_SYNC) {
    RCLCPP_WARN(get_logger(), "[KEY] Press 't' first for slow_sync.");
    return;
  }

  std::string follower_reason;
  if (!follower_joint_ready_for_teleop(follower_reason)) {
    RCLCPP_WARN(get_logger(), "[KEY] FAST blocked: %s", follower_reason.c_str());
    return;
  }

  std::string observer_reason;
  if (!contact_observation_ready_for_teleop(observer_reason)) {
    RCLCPP_WARN(
      get_logger(),
      "[KEY] FAST blocked: %s. Pause with 's', keep the follower "
      "stationary/contact-free, and restart SLOW after valid=True.",
      observer_reason.c_str());
    return;
  }

  if (!slow_sync_ready_) {
    Vec6 error = Vec6::Zero();
    std::string joints;
    {
      std::lock_guard<std::mutex> lock(follower_mtx_);
      if (!follower_joint_rad_.has_value()) {
        RCLCPP_WARN(get_logger(), "[KEY] FAST blocked: follower joint state unavailable.");
        return;
      }
      error = last_follower_target_ - follower_joint_rad_.value();
    }
    for (int i = 0; i < 6; ++i) {
      if (std::abs(error[i]) >= slow_sync_ready_tol_rad_) {
        if (!joints.empty()) joints += "/";
        joints += "J" + std::to_string(i + 1);
      }
    }
    if (joints.empty()) joints = "hold " + std::to_string(slow_sync_ready_hold_sec_) + "s 미완료";
    RCLCPP_WARN(get_logger(),
      "[KEY] FAST blocked: %s (tol=%.4frad). SLOW READY가 될 때까지 기다리세요.",
      joints.c_str(), slow_sync_ready_tol_rad_);
    return;
  }

  // Request wrench tare (baseline) — needed by both feedback modes
  if (use_jt_wrench_fb_) {
    std::lock_guard<std::mutex> lock(jt_wrench_mtx_);
    jt_wrench_tare_req_ = true;
    jt_wrench_tare_count_ = 0;
    jt_wrench_tare_accum_ = Vec6::Zero();
  }
  if (use_ft_sensor_feedback_) {
    std::lock_guard<std::mutex> lock(ft_sensor_mtx_);
    ft_sensor_tare_req_ = true;
    ft_sensor_tare_count_ = 0;
    ft_sensor_tare_accum_ = Vec6::Zero();
    ft_sensor_tare_last_stamp_ = 0.0;
  }

  // Reset torque LPF
  tau_lpf_init_ = false;
  tau_fb_slew_init_ = false;
  tau_fb_deadband_active_.fill(false);
  tau_fb_deadband_active_period_.fill(false);
  tau_fb_slew_active_.fill(false);
  tau_fb_slew_active_period_.fill(false);
  tau_fb_motion_gate_driver_.fill(false);
  tau_fb_motion_gate_active_period_ = false;
  tau_fb_motion_gate_driver_period_.fill(false);
  tau_fb_motion_gate_driver_speed_period_.setZero();
  tau_fb_motion_gate_max_speed_period_ = 0.0;
  tau_fb_motion_gate_min_scale_period_ = 1.0;
  tau_fb_passivity_gate_active_period_ = false;
  tau_fb_passivity_max_power_period_W_ = 0.0;
  tau_fb_passivity_min_scale_period_ = 1.0;
  tau_fb_passivity_gate_joint_.fill(false);
  tau_fb_passivity_gate_joint_period_.fill(false);
  last_tau_fb_passivity_power_W_ = 0.0;
  last_tau_fb_passivity_scale_ = 1.0;
  reset_contact_gate_state();
  leader_dq_lpf_init_ = false;
  leader_dq_prev_init_ = false;
  loop_count_ = 0;
  dxl_read_count_ = 0;
  dxl_fresh_count_ = 0;
  dxl_fallback_count_ = 0;
  dxl_missing_count_ = 0;
  vel_read_count_ = 0;
  vel_fresh_count_ = 0;
  vel_fallback_count_ = 0;
  vel_missing_count_ = 0;
  teleop_cmd_count_ = 0;
  hz_log_last_t_ = now().seconds();

  // Ramp down gravity to normal
  grav_scale_target_ = Vec6::Ones();

  state_ = TeleopState::FAST;
  RCLCPP_INFO(get_logger(), "%s teleop active | feedback=%s source=%s | gravity_comp=%s",
    state_tag("FAST").c_str(),
    runtime_feedback_enabled() ? "ON" : "OFF",
    feedback_source_label().c_str(),
    runtime_gravity_enabled() ? "ON" : "OFF");
}

void LeaderTeleopNode::pause_teleop() {
  grav_scale_target_ = grav_sync_scale_;
  impedance_last_pos_.reset();
  impedance_last_R_.reset();
  slow_sync_cmd_.reset();
  state_ = TeleopState::PAUSED;
  RCLCPP_INFO(get_logger(), "%s", blue("[STATE] -> PAUSE (grav sync scale)").c_str());
}

void LeaderTeleopNode::return_to_zero_and_shutdown() {
  if (shutdown_started_.exchange(true)) {
    RCLCPP_INFO(get_logger(), "%s shutdown already in progress",
      state_tag("SHUTDOWN").c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "%s zero-return shutdown started | previous_state=%s",
    state_tag("SHUTDOWN").c_str(), teleop_state_name());
  state_ = TeleopState::SHUTDOWN;
  shutdown_requested_ = true;
  shutdown_command_pending_ = false;
  pending_control_command_ = 0;
  publish_status();

  // Move leader to zero (tick=zero_ticks)
  bool zero_return_started = false;
  bool zero_reached = false;
  if (bus_->prepare_operating_mode(arm_.dxl_ids, 3)) {
    dxl_current_mode_ = false;
    if (!bus_->set_profile_deg(arm_.dxl_ids, 10.0, 50.0)) {
      RCLCPP_ERROR(get_logger(), "%s profile setup failed: %s",
        state_tag("SHUTDOWN").c_str(), bus_->last_error().c_str());
    } else {
      std::map<int, int32_t> zero_ticks;
      for (int i = 0; i < 6; ++i) {
        zero_ticks[arm_.dxl_ids[i]] = arm_.zero_ticks[i];
      }
      const bool position_torque_ready = bus_->torque_on(arm_.dxl_ids);
      if (!position_torque_ready) {
        RCLCPP_ERROR(get_logger(), "%s torque-on failed: %s",
          state_tag("SHUTDOWN").c_str(), bus_->last_error().c_str());
      } else {
        zero_return_started = bus_->write_goal_positions_verified(zero_ticks);
        if (!zero_return_started) {
          const std::string goal_error = bus_->last_error();
          const bool rollback_ok = bus_->torque_off(arm_.dxl_ids);
          RCLCPP_ERROR(get_logger(),
            "%s zero goal write failed: %s | immediate torque_off=%s%s%s",
            state_tag("SHUTDOWN").c_str(), goal_error.c_str(),
            rollback_ok ? "OK" : "FAILED",
            rollback_ok ? "" : " | ",
            rollback_ok ? "" : bus_->last_error().c_str());
        }
      }
    }
  } else {
    RCLCPP_ERROR(get_logger(), "%s position-mode transition failed: %s",
      state_tag("SHUTDOWN").c_str(), bus_->last_error().c_str());
  }

  if (zero_return_started) {
    const double max_start_deg = q_leader_.cwiseAbs().maxCoeff() * 180.0 / M_PI;
    const double timeout_s = std::clamp(max_start_deg / 10.0 + 3.0, 6.0, 20.0);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(timeout_s));
    int consecutive_unfresh_reads = 0;
    double last_status_t = 0.0;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      auto pos = bus_->read_positions();
      if (!bus_->last_read_all_fresh()) {
        if (++consecutive_unfresh_reads >= 10) {
          RCLCPP_ERROR(get_logger(),
            "%s lost fresh leader positions; stopping zero return: %s",
            state_tag("SHUTDOWN").c_str(), bus_->last_error().c_str());
          break;
        }
        continue;
      }
      consecutive_unfresh_reads = 0;
      double max_err = 0.0;
      for (int j = 0; j < 6; ++j) {
        auto it = pos.find(arm_.dxl_ids[j]);
        if (it == pos.end()) continue;
        max_err = std::max(max_err, std::abs(
          static_cast<double>(it->second - arm_.zero_ticks[j]) * TICKS_TO_RAD));
      }
      const double t = now().seconds();
      if (last_status_t <= 0.0 || t - last_status_t >= 0.1) {
        last_status_t = t;
        publish_status();
      }
      if (max_err < 3.0 * M_PI / 180.0) {
        zero_reached = true;
        break;
      }
    }
    if (!zero_reached && rclcpp::ok()) {
      RCLCPP_WARN(get_logger(),
        "%s zero position was not verified before timeout; torque will be turned off.",
        state_tag("SHUTDOWN").c_str());
    }
  }

  const bool torque_off_ok = bus_->torque_off(arm_.dxl_ids);
  RCLCPP_INFO(get_logger(), "%s Done. Torque OFF=%s, zero_reached=%s.",
    state_tag("SHUTDOWN").c_str(), torque_off_ok ? "true" : "false",
    zero_reached ? "true" : "false");
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Follower init pose (z key)
// ═══════════════════════════════════════════════════════════════════════════════

void LeaderTeleopNode::send_follower_to_init_pose() {
  if (init_pose_in_progress_.load()) {
    RCLCPP_WARN(get_logger(), "%s init pose trajectory is already running.",
      state_tag("INIT POSE").c_str());
    return;
  }

  pause_teleop();
  state_ = TeleopState::INIT_POSE;
  init_pose_reached_ = false;
  init_pose_verified_ = false;
  init_pose_in_progress_ = true;
  init_pose_phase_ = InitPosePhase::MOVE;
  init_pose_target_rad_ = follower_init_pose_rad_;
  init_pose_start_rad_ = follower_init_pose_rad_;
  {
    std::lock_guard<std::mutex> lock(follower_mtx_);
    if (follower_joint_rad_.has_value()) {
      init_pose_start_rad_ = follower_joint_rad_.value();
    } else {
      RCLCPP_WARN(get_logger(),
        "%s follower joint_state unavailable; starting trajectory from target pose.",
        state_tag("INIT POSE").c_str());
    }
  }
  init_pose_duration_s_ = std::max(0.1, follower_init_pose_duration_sec_);
  init_pose_start_t_ = now().seconds();
  init_pose_verify_start_t_ = 0.0;
  init_pose_settle_start_t_ = 0.0;
  init_pose_last_log_t_ = 0.0;
  loop_count_ = 0;
  dxl_read_count_ = 0;
  dxl_fresh_count_ = 0;
  dxl_fallback_count_ = 0;
  dxl_missing_count_ = 0;
  hz_log_last_t_ = now().seconds();

  if (bus_->prepare_operating_mode(arm_.dxl_ids, 0)) {
    std::map<int, int16_t> zero_current;
    for (int id : arm_.dxl_ids) zero_current[id] = 0;
    if (bus_->write_goal_currents_verified(zero_current) &&
        bus_->torque_on(arm_.dxl_ids)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      dxl_current_mode_ = true;
      grav_scale_target_ = grav_sync_scale_;
    } else {
      init_pose_in_progress_ = false;
      enter_dxl_fault(
        "INIT POSE verified zero-current/torque-on failed: " + bus_->last_error());
      return;
    }
  } else {
    init_pose_in_progress_ = false;
    enter_dxl_fault(
      "INIT POSE torque-off/mode preparation failed: " + bus_->last_error());
    return;
  }

  RCLCPP_INFO(get_logger(),
    "%s minimum-jerk init pose trajectory started | duration=%.1fs update_hz=%.1f | grav_scale %s -> %s over %.1fs | target(deg)= %s",
    state_tag("INIT POSE").c_str(),
    init_pose_duration_s_,
    1.0 / dt_,
    format_joint_scale(grav_scale_).c_str(),
    format_joint_scale(grav_scale_target_).c_str(),
    grav_ramp_sec_,
    format_joint_deg(init_pose_target_rad_).c_str());
}

static double minimum_jerk_alpha(double s) {
  s = std::clamp(s, 0.0, 1.0);
  return 10.0 * s * s * s - 15.0 * s * s * s * s + 6.0 * s * s * s * s * s;
}

void LeaderTeleopNode::finish_follower_init_pose(bool success, std::optional<double> max_err_deg) {
  init_pose_in_progress_ = false;
  init_pose_phase_ = InitPosePhase::IDLE;
  init_pose_reached_ = success;
  init_pose_verified_ = success;
  if (success) {
    if (max_err_deg) {
      RCLCPP_INFO(get_logger(),
        "%s follower init pose verified | max_err=%.2fdeg. Teleop paused; press 'r' to re-align.",
        state_tag("INIT POSE").c_str(), *max_err_deg);
    } else {
      RCLCPP_INFO(get_logger(),
        "%s follower init pose verified. Teleop paused; press 'r' to re-align.",
        state_tag("INIT POSE").c_str());
    }
  } else {
    if (max_err_deg) {
      RCLCPP_WARN(get_logger(),
        "%s follower init pose was not verified | max_err=%.2fdeg. Teleop paused; press 'z' to retry or 'r' to re-align.",
        state_tag("INIT POSE").c_str(), *max_err_deg);
    } else {
      RCLCPP_WARN(get_logger(),
        "%s follower init pose was not verified. Teleop paused; press 'z' to retry or 'r' to re-align.",
        state_tag("INIT POSE").c_str());
    }
  }
  state_ = TeleopState::PAUSED;
  grav_scale_target_ = grav_sync_scale_;
  loop_count_ = 0;
  dxl_read_count_ = 0;
  dxl_fresh_count_ = 0;
  dxl_fallback_count_ = 0;
  dxl_missing_count_ = 0;
  hz_log_last_t_ = now().seconds();
}

void LeaderTeleopNode::update_follower_init_pose() {
  if (state_ != TeleopState::INIT_POSE || !init_pose_in_progress_.load()) return;

  const double t = now().seconds();
  if (init_pose_phase_ == InitPosePhase::MOVE) {
    const double duration = std::max(1e-6, init_pose_duration_s_);
    const double elapsed = t - init_pose_start_t_;
    const double alpha = minimum_jerk_alpha(elapsed / duration);
    const Vec6 cmd = init_pose_start_rad_ + alpha * (init_pose_target_rad_ - init_pose_start_rad_);
    publish_impedance_pose(cmd);
    if (elapsed >= duration) {
      publish_impedance_pose(init_pose_target_rad_);
      init_pose_phase_ = InitPosePhase::VERIFY;
      init_pose_verify_start_t_ = t;
      init_pose_settle_start_t_ = 0.0;
      RCLCPP_INFO(get_logger(),
        "%s phase1 complete: minimum-jerk command sent for %.1fs at control Hz; verifying 5.0deg for 2.0s",
        state_tag("INIT POSE").c_str(), duration);
    }
    return;
  }

  if (init_pose_phase_ != InitPosePhase::VERIFY) return;

  constexpr double verify_timeout = 5.0;
  constexpr double settle_time = 2.0;
  constexpr double tolerance_rad = 5.0 * M_PI / 180.0;

  std::optional<Vec6> current;
  {
    std::lock_guard<std::mutex> lock(follower_mtx_);
    if (follower_joint_rad_.has_value()) current = follower_joint_rad_.value();
  }

  if (!current.has_value()) {
    if (t - init_pose_last_log_t_ >= 1.0) {
      init_pose_last_log_t_ = t;
      RCLCPP_WARN(get_logger(), "%s waiting for follower joint_state during verification.",
        state_tag("INIT POSE").c_str());
    }
    init_pose_settle_start_t_ = 0.0;
    if (t - init_pose_verify_start_t_ >= verify_timeout) {
      finish_follower_init_pose(false);
    }
    return;
  }

  const double max_err = (init_pose_target_rad_ - current.value()).cwiseAbs().maxCoeff();
  const double max_err_deg = max_err * 180.0 / M_PI;
  if (max_err <= tolerance_rad) {
    if (init_pose_settle_start_t_ <= 0.0) {
      init_pose_settle_start_t_ = t;
      RCLCPP_INFO(get_logger(), "%s init pose within tolerance %.2fdeg; holding %.1fs",
        state_tag("INIT POSE").c_str(), max_err_deg, settle_time);
    } else if (t - init_pose_settle_start_t_ >= settle_time) {
      finish_follower_init_pose(true, max_err_deg);
    }
  } else {
    init_pose_settle_start_t_ = 0.0;
  }

  if (state_ == TeleopState::INIT_POSE &&
      t - init_pose_verify_start_t_ >= verify_timeout) {
    finish_follower_init_pose(false, max_err_deg);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Slow sync
// ═══════════════════════════════════════════════════════════════════════════════

Vec6 LeaderTeleopNode::slow_sync_step(const Vec6& target_rad) {
  double t = now().seconds();

  // During capture delay, just hold follower's current position
  if (t < slow_sync_capture_until_) {
    std::lock_guard<std::mutex> lock(follower_mtx_);
    if (follower_joint_rad_.has_value()) {
      slow_sync_cmd_ = follower_joint_rad_.value();
    }
    return slow_sync_cmd_.value_or(target_rad);
  }

  if (!slow_sync_cmd_.has_value()) {
    std::lock_guard<std::mutex> lock(follower_mtx_);
    slow_sync_cmd_ = follower_joint_rad_.value_or(target_rad);
    slow_sync_last_t_ = t;
  }

  double dt = t - slow_sync_last_t_;
  if (!std::isfinite(dt) || dt <= 0.0) dt = dt_;
  dt = std::clamp(dt, dt_, 0.1);

  Vec6 cmd = slow_sync_cmd_.value();
  double max_step = slow_sync_max_vel_rad_s_ * dt;
  Vec6 delta = target_rad - cmd;

  for (int i = 0; i < 6; ++i) {
    cmd[i] += std::clamp(delta[i], -max_step, max_step);
  }

  slow_sync_cmd_ = cmd;
  slow_sync_last_t_ = t;

  // Check convergence
  double err = std::numeric_limits<double>::infinity();
  std::string follower_reason;
  const bool follower_fresh = follower_joint_ready_for_teleop(follower_reason);
  {
    std::lock_guard<std::mutex> lock(follower_mtx_);
    if (follower_fresh && follower_joint_rad_.has_value()) {
      err = (target_rad - follower_joint_rad_.value()).cwiseAbs().maxCoeff();
    }
  }

  if (err < slow_sync_ready_tol_rad_) {
    if (slow_sync_ready_since_ <= 0.0) slow_sync_ready_since_ = t;
    if ((t - slow_sync_ready_since_) >= slow_sync_ready_hold_sec_) {
      slow_sync_ready_ = true;
      const std::string ready_msg = blue("[SLOW] Ready! Press 'o' to start FAST.");
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(),
        static_cast<int64_t>(std::max(0.05, hz_log_period_) * 1000.0),
        "%s", ready_msg.c_str());
    }
  } else {
    slow_sync_ready_since_ = 0.0;
    slow_sync_ready_ = false;
  }

  return cmd;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Torque → current → DXL
// ═══════════════════════════════════════════════════════════════════════════════

bool LeaderTeleopNode::send_torque_currents(const Vec6& tau) {
  std::map<int, int16_t> units;

  for (int i = 0; i < 6; ++i) {
    double Kt = arm_.torque_constant[i];
    double cu = arm_.current_unit[i];
    int max_u = arm_.max_current[i];

    // tau → current(A) → DXL units
    double current_A = tau[i] / Kt;
    int unit = static_cast<int>(std::round(current_A / cu));

    // Clip to max
    unit = std::clamp(unit, -max_u, max_u);

    units[arm_.dxl_ids[i]] = static_cast<int16_t>(unit);
  }

  return bus_->write_goal_currents(units);
}

}  // namespace teleop_cpp
