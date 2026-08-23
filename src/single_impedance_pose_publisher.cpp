// single_impedance_pose_publisher.cpp
// Single impedance teleop follower FK → workspace clip → intent generator
// → final safety slew → PoseStamped publish
//
// 원본 _compute_impedance_task_pose_raw(), _limit_impedance_task_pose(),
// _publish_impedance_task_pose() 대응
//  - Pinocchio FK로 follower TCP pose 계산 (base_link 기준)
//  - workspace min/max clipping (원본과 동일)
//  - raw pose와 causal leader-intent pose를 분리
//  - 2차 reference generator + velocity/acceleration limit
//  - workspace→command base 좌표 변환 (left_base_link / right_base_link)
//  - 기존 linear/angular slew limit를 최종 safety backstop으로 유지
//  - 위치 단위 mm로 publish (원본과 동일, Doosan controller 기대값)

#include <pinocchio/fwd.hpp>

#include "ft_fb_leaderarm/single_impedance_teleop_node.hpp"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <algorithm>
#include <cmath>

namespace teleop_cpp {

namespace {

Vec6 pose_mm_rpy_deg(const Vec3& position_m, const Mat3& rotation) {
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  const double pitch = std::atan2(
    -rotation(2, 0),
    std::sqrt(rotation(2, 1) * rotation(2, 1) +
              rotation(2, 2) * rotation(2, 2)));
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  Vec6 pose;
  pose << position_m.x() * 1000.0,
    position_m.y() * 1000.0,
    position_m.z() * 1000.0,
    roll * 180.0 / M_PI,
    pitch * 180.0 / M_PI,
    yaw * 180.0 / M_PI;
  return pose;
}

}  // namespace

void LeaderTeleopNode::publish_impedance_pose(const Vec6& follower_joint_rad) {
  // ── 1) FK: compute base_link-relative TCP pose ──
  Eigen::VectorXd q_full = Eigen::VectorXd::Zero(model_follower_.nq);
  for (int i = 0; i < 6; ++i) {
    q_full[idxq_follower_[i]] = follower_joint_rad[i];
  }

  Vec3 pos_m;
  Mat3 R;
  {
    std::lock_guard<std::mutex> lock(pin_follower_mtx_);
    pinocchio::forwardKinematics(model_follower_, data_follower_, q_full);
    pinocchio::updateFramePlacements(model_follower_, data_follower_);

    auto oMbase = data_follower_.oMf[impedance_workspace_frame_id_];
    auto oMtip = data_follower_.oMf[impedance_tip_frame_id_];
    auto base_M_tip = oMbase.inverse() * oMtip;
    pos_m = base_M_tip.translation();
    R = base_M_tip.rotation();
  }

  // ── 2) Workspace clipping ──
  Vec3 rel = pos_m;  // origin is at workspace base
  Vec3 clipped_rel;
  for (int i = 0; i < 3; ++i) {
    clipped_rel[i] = std::clamp(rel[i], workspace_min_[i], workspace_max_[i]);
  }
  const Vec3 clip_delta = clipped_rel - rel;
  const double clip_dist_m = clip_delta.norm();
  const bool first_publish =
    !impedance_last_pos_.has_value() || !impedance_last_R_.has_value();
  if (first_publish &&
      impedance_first_publish_clip_guard_m_ > 0.0 &&
      clip_dist_m > impedance_first_publish_clip_guard_m_) {
    last_task_cmd_valid_ = false;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "[IMPEDANCE] first publish blocked: workspace clip %.1f mm exceeds guard %.1f mm | "
      "state=%s workspace=%s raw=(%.3f, %.3f, %.3f)m clipped=(%.3f, %.3f, %.3f)m",
      clip_dist_m * 1000.0,
      impedance_first_publish_clip_guard_m_ * 1000.0,
      teleop_state_name(),
      impedance_workspace_frame_.c_str(),
      rel.x(), rel.y(), rel.z(),
      clipped_rel.x(), clipped_rel.y(), clipped_rel.z());
    return;
  }
  const Vec3 raw_workspace_pos = clipped_rel;
  const Mat3 raw_workspace_R = R;
  const double now_sec = now().seconds();
  double elapsed = now_sec - impedance_last_t_;
  if (!std::isfinite(elapsed) || elapsed <= 0.0 ||
      !impedance_last_pos_.has_value() || !impedance_last_R_.has_value()) {
    elapsed = dt_;
  }
  // Smooth ON prevents a delayed callback from creating a large catch-up step.
  // Smooth OFF preserves the pre-stabilization slew timing for a valid A/B baseline.
  const double command_elapsed = intent_generator_enabled_
    ? std::min(elapsed, 2.0 * dt_)
    : std::clamp(elapsed, dt_, 0.1);

  auto to_command_frame = [this](const Vec3& ws_pos, const Mat3& ws_R) {
    if (!has_workspace_M_command_) {
      return pinocchio::SE3(ws_R, ws_pos);
    }
    return workspace_M_command_.inverse() * pinocchio::SE3(ws_R, ws_pos);
  };

  // ── 3) Preserve the transformed raw target for diagnostics ──
  const pinocchio::SE3 raw_command =
    to_command_frame(raw_workspace_pos, raw_workspace_R);
  last_task_raw_mm_rpy_deg_ =
    pose_mm_rpy_deg(raw_command.translation(), raw_command.rotation());
  if (task_raw_last_pos_.has_value() && task_raw_last_R_.has_value()) {
    double raw_elapsed = now_sec - task_raw_last_t_;
    if (!std::isfinite(raw_elapsed) || raw_elapsed <= 0.0) raw_elapsed = dt_;
    last_task_raw_velocity_.head<3>() =
      (raw_command.translation() - task_raw_last_pos_.value()) / raw_elapsed;
    last_task_raw_velocity_.tail<3>() = task_raw_last_R_.value() *
      pinocchio::log3(
        task_raw_last_R_.value().transpose() * raw_command.rotation()) /
      raw_elapsed;
  } else {
    last_task_raw_velocity_.setZero();
  }
  task_raw_last_pos_ = raw_command.translation();
  task_raw_last_R_ = raw_command.rotation();
  task_raw_last_t_ = now_sec;

  // ── 4) Generate the causal human-intent reference in the workspace frame ──
  Vec3 intent_workspace_pos = raw_workspace_pos;
  Mat3 intent_workspace_R = raw_workspace_R;
  last_task_intent_velocity_ = last_task_raw_velocity_;
  last_task_intent_acceleration_.setZero();
  if (intent_generator_enabled_ && state_ == TeleopState::FAST) {
    if (!intent_generator_.initialized()) {
      intent_generator_.reset(raw_workspace_pos, raw_workspace_R);
    }
    const IntentTrajectoryState& intent = intent_generator_.update(
      raw_workspace_pos, raw_workspace_R, command_elapsed);
    intent_workspace_pos = intent.position_m;
    intent_workspace_R = intent.rotation;

    // The reference generator may coast at a workspace boundary. Clamp again
    // after integration and reset dynamic state if a boundary is reached.
    Vec3 bounded_intent = intent_workspace_pos;
    for (int i = 0; i < 3; ++i) {
      bounded_intent[i] = std::clamp(
        bounded_intent[i], workspace_min_[i], workspace_max_[i]);
    }
    if ((bounded_intent - intent_workspace_pos).norm() > 1.0e-12) {
      intent_workspace_pos = bounded_intent;
      intent_generator_.reset(intent_workspace_pos, intent_workspace_R);
    }

    Mat3 command_R_workspace = Mat3::Identity();
    if (has_workspace_M_command_) {
      command_R_workspace = workspace_M_command_.rotation().transpose();
    }
    last_task_intent_velocity_.head<3>() =
      command_R_workspace * intent.linear_velocity_m_s;
    last_task_intent_velocity_.tail<3>() =
      command_R_workspace * intent.angular_velocity_rad_s;
    last_task_intent_acceleration_.head<3>() =
      command_R_workspace * intent.linear_acceleration_m_s2;
    last_task_intent_acceleration_.tail<3>() =
      command_R_workspace * intent.angular_acceleration_rad_s2;
  } else {
    // SLOW_SYNC keeps its established behavior and continuously seeds the
    // generator, giving FAST a continuous starting point without a raw jump.
    intent_generator_.reset(raw_workspace_pos, raw_workspace_R);
  }

  const pinocchio::SE3 intent_command =
    to_command_frame(intent_workspace_pos, intent_workspace_R);
  last_task_intent_mm_rpy_deg_ =
    pose_mm_rpy_deg(intent_command.translation(), intent_command.rotation());

  // ── 5) Preserve the existing final slew-rate safety limiter ──
  Vec3 command_pos = intent_command.translation();
  Mat3 command_R = intent_command.rotation();
  if (!impedance_last_pos_.has_value() || !impedance_last_R_.has_value()) {
    impedance_last_pos_ = command_pos;
    impedance_last_R_ = command_R;
    impedance_last_t_ = now_sec;
  } else {

    // Position slew
    Vec3 delta_pos = command_pos - impedance_last_pos_.value();
    double delta_norm = delta_pos.norm();
    double max_pos_step = impedance_lin_speed_m_s_ * command_elapsed;
    if (delta_norm > max_pos_step && max_pos_step > 0.0) {
      command_pos = impedance_last_pos_.value() +
        delta_pos * (max_pos_step / delta_norm);
    }

    // Rotation slew
    Mat3 R_delta = impedance_last_R_.value().transpose() * command_R;
    Eigen::Vector3d rotvec = pinocchio::log3(R_delta);
    double angle = rotvec.norm();
    double max_angle_step = impedance_ang_speed_rad_s_ * command_elapsed;
    if (angle > max_angle_step && max_angle_step > 0.0) {
      command_R = impedance_last_R_.value() *
        pinocchio::exp3(rotvec * (max_angle_step / angle));
    }

    impedance_last_pos_ = command_pos;
    impedance_last_R_ = command_R;
    impedance_last_t_ = now_sec;
  }

  if (state_ != TeleopState::FAST) {
    pinocchio::SE3 command_M_tip(command_R, command_pos);
    const pinocchio::SE3 workspace_M_tip = has_workspace_M_command_
      ? workspace_M_command_ * command_M_tip
      : command_M_tip;
    intent_generator_.reset(
      workspace_M_tip.translation(), workspace_M_tip.rotation());
  }

  // ── 6) Publish only the clean command (position in mm) ──
  auto msg = geometry_msgs::msg::PoseStamped();
  msg.header.stamp = this->now();
  msg.header.frame_id = impedance_base_frame_;

  Vec3 pos_mm = command_pos * 1000.0;
  msg.pose.position.x = pos_mm.x();
  msg.pose.position.y = pos_mm.y();
  msg.pose.position.z = pos_mm.z();

  last_task_cmd_mm_rpy_deg_ = pose_mm_rpy_deg(command_pos, command_R);
  last_task_cmd_valid_ = true;

  Eigen::Vector4d quat = rotation_to_quat_xyzw(command_R);
  msg.pose.orientation.x = quat[0];
  msg.pose.orientation.y = quat[1];
  msg.pose.orientation.z = quat[2];
  msg.pose.orientation.w = quat[3];

  if (follower_command_publish_enabled_) {
    pub_impedance_->publish(msg);
    if (state_ == TeleopState::FAST) ++teleop_cmd_count_;
  }
}

}  // namespace teleop_cpp
