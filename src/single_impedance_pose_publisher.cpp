// single_impedance_pose_publisher.cpp
// Single impedance teleop follower FK → workspace clip → slew rate limit → PoseStamped publish
//
// 원본 _compute_impedance_task_pose_raw(), _limit_impedance_task_pose(),
// _publish_impedance_task_pose() 대응
//  - Pinocchio FK로 follower TCP pose 계산 (base_link 기준)
//  - workspace min/max clipping (원본과 동일)
//  - workspace→command base 좌표 변환 (left_base_link / right_base_link)
//  - 위치: linear speed limit (mm/s), 자세: angular speed limit (deg/s)
//  - 위치 단위 mm로 publish (원본과 동일, Doosan controller 기대값)

#include <pinocchio/fwd.hpp>

#include "ft_fb_leaderarm/single_impedance_teleop_node.hpp"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <algorithm>
#include <cmath>

namespace teleop_cpp {

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
  Vec3 clipped_pos = clipped_rel;
  Mat3 R_target = R;

  // ── 3) Transform workspace → command base frame if needed ──
  if (has_workspace_M_command_) {
    pinocchio::SE3 ws_M_tip(R_target, clipped_pos);
    auto cmd_M_tip = workspace_M_command_.inverse() * ws_M_tip;
    clipped_pos = cmd_M_tip.translation();
    R_target = cmd_M_tip.rotation();
  }

  // ── 4) Slew-rate limiting ──
  double now_sec = now().seconds();

  if (!impedance_last_pos_.has_value() || !impedance_last_R_.has_value()) {
    impedance_last_pos_ = clipped_pos;
    impedance_last_R_ = R_target;
    impedance_last_t_ = now_sec;
  } else {
    double elapsed = now_sec - impedance_last_t_;
    if (!std::isfinite(elapsed) || elapsed <= 0.0) elapsed = dt_;
    elapsed = std::clamp(elapsed, dt_, 0.1);

    // Position slew
    Vec3 delta_pos = clipped_pos - impedance_last_pos_.value();
    double delta_norm = delta_pos.norm();
    double max_pos_step = impedance_lin_speed_m_s_ * elapsed;
    if (delta_norm > max_pos_step && max_pos_step > 0.0) {
      clipped_pos = impedance_last_pos_.value() + delta_pos * (max_pos_step / delta_norm);
    }

    // Rotation slew
    Mat3 R_delta = impedance_last_R_.value().transpose() * R_target;
    Eigen::Vector3d rotvec = pinocchio::log3(R_delta);
    double angle = rotvec.norm();
    double max_angle_step = impedance_ang_speed_rad_s_ * elapsed;
    if (angle > max_angle_step && max_angle_step > 0.0) {
      R_target = impedance_last_R_.value() * pinocchio::exp3(rotvec * (max_angle_step / angle));
    }

    impedance_last_pos_ = clipped_pos;
    impedance_last_R_ = R_target;
    impedance_last_t_ = now_sec;
  }

  // ── 5) Publish PoseStamped (position in mm) ──
  auto msg = geometry_msgs::msg::PoseStamped();
  msg.header.stamp = this->now();
  msg.header.frame_id = impedance_base_frame_;

  Vec3 pos_mm = clipped_pos * 1000.0;
  msg.pose.position.x = pos_mm.x();
  msg.pose.position.y = pos_mm.y();
  msg.pose.position.z = pos_mm.z();

  double roll = std::atan2(R_target(2, 1), R_target(2, 2));
  double pitch = std::atan2(-R_target(2, 0),
                            std::sqrt(R_target(2, 1) * R_target(2, 1) +
                                      R_target(2, 2) * R_target(2, 2)));
  double yaw = std::atan2(R_target(1, 0), R_target(0, 0));
  last_task_cmd_mm_rpy_deg_ << pos_mm.x(), pos_mm.y(), pos_mm.z(),
    roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI;
  last_task_cmd_valid_ = true;

  Eigen::Vector4d quat = rotation_to_quat_xyzw(R_target);
  msg.pose.orientation.x = quat[0];
  msg.pose.orientation.y = quat[1];
  msg.pose.orientation.z = quat[2];
  msg.pose.orientation.w = quat[3];

  pub_impedance_->publish(msg);
  if (state_ == TeleopState::FAST) ++teleop_cmd_count_;
}

}  // namespace teleop_cpp
