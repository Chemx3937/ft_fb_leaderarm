// single_impedance_wrench_feedback.cpp
// Single impedance teleop /F_e or F/T wrench → follower-base wrench → leader Jacobian^T feedback torque
//
// 원본 _apply_JT_Wrench_Feedback(), _compute_jt_wrench_feedback_base_wrench(),
//      _compute_leader_joint_torques_from_base_wrenches() 대응
//  - /left/F_e 또는 /right/F_e (Float64MultiArray [Fx,Fy,Fz,Mx,My,Mz])
//    · 적용점(point) = ee_frame (left_link_6), 축(axes) = command base (left_base_link)
//  - FAST 진입 시 tare (baseline 수집 N samples 평균)
//  - stale 검사 (timeout 초과 시 feedback=0)
//  - follower FK로 wrench를 follower base_link 기준 spatial [M;F]로 변환
//    (r×F 모멘트 이전항 포함 — Python 레퍼런스와 동일)
//  - leader WORLD Jacobian을 leader_base 축으로 재표현하고 [angular;linear]로 정렬해
//    spatial wrench [M;F]와 행 순서를 맞춰 J^T × w → 6축 관절 토크 → gain × per-joint clip

#include <pinocchio/fwd.hpp>

#include "ft_fb_leaderarm/single_impedance_teleop_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace teleop_cpp {

// ─── Shared: /F_e or physical F/T → leader-base spatial wrench [M;F] + leader space Jacobian ───
bool LeaderTeleopNode::compute_reflected_wrench_and_jacobian(
    Vec6& w_spatial, Eigen::MatrixXd& J_space) {
  if (use_ft_sensor_feedback_) {
    ft_sensor_tare_step();

    double stamp;
    Vec6 raw;
    Vec6 baseline;
    bool tare_pending;
    {
      std::lock_guard<std::mutex> lock(ft_sensor_mtx_);
      stamp = ft_sensor_stamp_;
      raw = ft_sensor_raw_;
      baseline = ft_sensor_baseline_;
      tare_pending = ft_sensor_tare_req_;
    }
    if (tare_pending || stamp <= 0.0 || now().seconds() - stamp > ft_stale_timeout_) {
      last_w_base_.setZero();
      last_tau_unclipped_.setZero();
      return false;
    }

    const Vec6 delta = raw - baseline;
    return compute_reflected_wrench_and_jacobian_from_delta(
        delta, ft_sensor_frame_id_, ft_sensor_frame_id_, ft_feedback_wrench_sign_,
        w_spatial, J_space);
  }

  if (use_contact_observer_fb_) {
    Vec6 contact_wrench = Vec6::Zero();
    double source_stamp = 0.0;
    std::chrono::steady_clock::time_point receive_steady;
    bool receive_steady_valid = false;
    uint8_t contact_state =
      contact_observer_msgs::msg::ContactObservation::FREE;
    bool valid = false;
    bool model_ready = false;
    {
      std::lock_guard<std::mutex> lock(contact_observation_mtx_);
      contact_wrench = contact_observation_wrench_;
      source_stamp = contact_observation_source_stamp_;
      receive_steady = contact_observation_receive_steady_;
      receive_steady_valid = contact_observation_receive_steady_valid_;
      contact_state = contact_observation_state_;
      valid = contact_observation_valid_;
      model_ready = contact_observation_model_ready_;
    }

    const double local_age = receive_steady_valid
      ? std::chrono::duration<double>(
          std::chrono::steady_clock::now() - receive_steady).count()
      : std::numeric_limits<double>::infinity();
    const double source_age = now().seconds() - source_stamp;
    const bool fresh = receive_steady_valid &&
      local_age >= 0.0 && local_age <= contact_observation_stale_timeout_ &&
      source_age >= -contact_observation_clock_future_tolerance_ &&
      source_age <= contact_observation_stale_timeout_;
    const bool in_contact =
      contact_state == contact_observer_msgs::msg::ContactObservation::CONTACT;

    // Mirror the canonical state for diagnostics only. The leader never
    // classifies contact in this mode.
    tau_fb_contact_state_ = valid && model_ready && fresh && in_contact;
    tau_fb_contact_phase_ = tau_fb_contact_state_ ? 1 : -1;
    tau_fb_contact_scale_ = tau_fb_contact_state_ ? 1.0 : 0.0;
    contact_observation_feedback_active_ = tau_fb_contact_state_;

    if (!valid || !model_ready || !fresh || !in_contact) {
      last_w_base_.setZero();
      last_tau_unclipped_.setZero();
      return false;
    }

    return compute_reflected_wrench_and_jacobian_from_delta(
      contact_wrench,
      impedance_command_base_frame_id_,
      impedance_tip_frame_id_,
      jt_wrench_sign_,
      w_spatial,
      J_space);
  }

  if (!use_jt_wrench_fb_) {
    last_w_base_.setZero();
    last_tau_unclipped_.setZero();
    return false;
  }

  jt_wrench_tare_step();

  double stamp;
  Vec6 raw;
  bool tare_pending;
  {
    std::lock_guard<std::mutex> lock(jt_wrench_mtx_);
    stamp = jt_wrench_stamp_;
    raw = jt_wrench_raw_;
    tare_pending = jt_wrench_tare_req_;
  }

  const double age = now().seconds() - stamp;
  if (tare_pending || stamp <= 0.0 || age > jt_wrench_stale_timeout_) {
    last_w_base_.setZero();
    last_tau_unclipped_.setZero();
    return false;
  }

  const Vec6 delta = raw - jt_wrench_baseline_;
  return compute_reflected_wrench_and_jacobian_from_delta(
      delta, impedance_command_base_frame_id_, impedance_tip_frame_id_, jt_wrench_sign_,
      w_spatial, J_space);
}

bool LeaderTeleopNode::compute_reflected_wrench_and_jacobian_from_delta(
    const Vec6& delta_ft,
    int source_frame_id,
    int application_frame_id,
    const Vec6& spatial_sign,
    Vec6& w_spatial,
    Eigen::MatrixXd& J_space) {
  // ContactObservation already contains the canonical state and residual.
  // Do not run the legacy leader-side bias estimator/classifier a second time.
  Vec6 delta = use_contact_observer_fb_
    ? delta_ft
    : apply_contact_gate_to_wrench_delta(delta_ft);

  auto bail = [this]() -> bool {
    last_w_base_.setZero();
    last_tau_unclipped_.setZero();
    if (use_contact_observer_fb_) {
      contact_observation_feedback_active_ = false;
    }
    return false;
  };

  if (leader_tip_frame_id_ < 0 || leader_base_frame_id_ < 0 ||
      follower_base_frame_id_ < 0 || source_frame_id < 0 ||
      application_frame_id < 0) {
    return bail();
  }

  Vec6 q_foll;
  std::chrono::steady_clock::time_point follower_receive_steady;
  bool follower_receive_steady_valid = false;
  {
    std::lock_guard<std::mutex> lock(follower_mtx_);
    if (!follower_joint_rad_.has_value()) return bail();
    q_foll = follower_joint_rad_.value();
    follower_receive_steady = follower_joint_receive_steady_;
    follower_receive_steady_valid = follower_joint_receive_steady_valid_;
  }
  if (!q_foll.allFinite()) return bail();
  if (use_contact_observer_fb_) {
    const double follower_age = follower_receive_steady_valid
      ? std::chrono::duration<double>(
          std::chrono::steady_clock::now() - follower_receive_steady).count()
      : std::numeric_limits<double>::infinity();
    if (follower_age < 0.0 || follower_age > follower_joint_stale_timeout_) {
      return bail();
    }
  }

  Vec3 force_base;
  Vec3 moment_base;
  {
    std::lock_guard<std::mutex> lock(pin_follower_mtx_);
    Eigen::VectorXd q_full_foll = Eigen::VectorXd::Zero(model_follower_.nq);
    for (int i = 0; i < 6; ++i) q_full_foll[idxq_follower_[i]] = q_foll[i];

    pinocchio::forwardKinematics(model_follower_, data_follower_, q_full_foll);
    pinocchio::updateFramePlacements(model_follower_, data_follower_);

    const auto& oMbase = data_follower_.oMf[follower_base_frame_id_];
    const auto& oMsource = data_follower_.oMf[source_frame_id];
    const auto& oMapp = data_follower_.oMf[application_frame_id];

    const auto base_M_source = oMbase.inverse() * oMsource;
    const auto base_M_app = oMbase.inverse() * oMapp;
    const Mat3 R_base_source = base_M_source.rotation();
    const Vec3 p_base_app = base_M_app.translation();

    const Vec3 force_source = delta.head<3>();
    const Vec3 moment_source = delta.tail<3>();
    force_base = R_base_source * force_source;
    moment_base = R_base_source * moment_source + p_base_app.cross(force_base);
  }

  w_spatial << moment_base, force_base;
  w_spatial = spatial_sign.cwiseProduct(w_spatial);
  last_w_base_ << w_spatial.tail<3>(), w_spatial.head<3>();

  const Eigen::VectorXd q_full = q_full_leader_;
  pinocchio::forwardKinematics(model_leader_, data_leader_, q_full);
  pinocchio::computeJointJacobians(model_leader_, data_leader_, q_full);
  pinocchio::updateFramePlacements(model_leader_, data_leader_);

  const Mat3 R_base_world = data_leader_.oMf[leader_base_frame_id_].rotation().transpose();

  Eigen::MatrixXd J_world(6, model_leader_.nv);
  J_world.setZero();
  pinocchio::getFrameJacobian(model_leader_, data_leader_,
    static_cast<pinocchio::FrameIndex>(leader_tip_frame_id_),
    pinocchio::WORLD, J_world);

  J_space.resize(6, model_leader_.nv);
  J_space.topRows(3) = R_base_world * J_world.bottomRows(3);
  J_space.bottomRows(3) = R_base_world * J_world.topRows(3);

  return true;
}

// ─── Tare (baseline collection) ───

void LeaderTeleopNode::jt_wrench_tare_step() {
  std::lock_guard<std::mutex> lock(jt_wrench_mtx_);

  if (!jt_wrench_tare_req_) return;

  jt_wrench_tare_accum_ += jt_wrench_raw_;
  jt_wrench_tare_count_++;

  if (jt_wrench_tare_count_ >= jt_wrench_tare_N_) {
    jt_wrench_baseline_ = jt_wrench_tare_accum_ / static_cast<double>(jt_wrench_tare_count_);
    jt_wrench_tare_req_ = false;
    jt_wrench_tare_count_ = 0;
    jt_wrench_tare_accum_ = Vec6::Zero();
    RCLCPP_INFO(get_logger(), "[JT WRENCH] Tare done. Baseline: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
      jt_wrench_baseline_[0], jt_wrench_baseline_[1], jt_wrench_baseline_[2],
      jt_wrench_baseline_[3], jt_wrench_baseline_[4], jt_wrench_baseline_[5]);
  }
}

// ─── Feedback computation: /F_e wrench → leader joint torques ───
// The /F_e → leader-base spatial wrench → leader J_space front-half is shared with
// physical F/T feedback. Here we only map it to joints with the per-joint gain.

Vec6 LeaderTeleopNode::compute_jt_wrench_feedback() {
  Vec6 w_spatial;
  Eigen::MatrixXd J_space;
  if (!compute_reflected_wrench_and_jacobian(w_spatial, J_space)) {
    return Vec6::Zero();
  }

  Eigen::VectorXd tau_full = J_space.transpose() * w_spatial;

  // Extract active 6 joints, apply per-joint reflection gain. The single clip and
  // saturation diagnostic live in control_loop after optional tau_fb smoothing.
  Vec6 tau_ext;
  for (int i = 0; i < 6; ++i) tau_ext[i] = tau_full[idxv_leader_[i]];

  return arm_.jt_wrench_fb_gain.cwiseProduct(tau_ext);
}

}  // namespace teleop_cpp
