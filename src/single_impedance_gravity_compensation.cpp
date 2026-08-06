// single_impedance_gravity_compensation.cpp
// Single impedance teleop Pinocchio g(q) 기반 중력보상 토크 계산
//
// 원본 _apply_Gravity_Compensation() 대응
//  - 12축 모델에서 활성 6축만 추출 (idxv_leader_)
//  - grav_gain × g(q) × per-joint grav_scale (ramp 적용)
//  - Python pybind 오버헤드 제거 → ~5μs (원본 ~30μs)

#include <pinocchio/fwd.hpp>

#include "ft_fb_leaderarm/single_impedance_teleop_node.hpp"

#include <pinocchio/algorithm/rnea.hpp>
#include <algorithm>
#include <cmath>

namespace teleop_cpp {

Vec6 LeaderTeleopNode::compute_gravity_torque(const Vec6& q_leader) {
  if (!use_gravity_comp_ || arm_.grav_gain.cwiseAbs().maxCoeff() <= 1e-12) {
    tau_grav_lpf_state_.setZero();
    tau_grav_lpf_init_ = true;
    return Vec6::Zero();
  }

  // Update full model q
  Eigen::VectorXd q = q_full_leader_;
  for (int i = 0; i < 6; ++i) {
    q[idxq_leader_[i]] = q_leader[i];
  }

  // g(q) via pinocchio
  pinocchio::computeGeneralizedGravity(model_leader_, data_leader_, q);
  const auto& g = data_leader_.g;

  // Extract active 6-DOF gravity torques and apply gain
  Vec6 tau_g;
  for (int i = 0; i < 6; ++i) {
    tau_g[i] = arm_.grav_gain[i] * g[idxv_leader_[i]];
  }

  // Ramp-based scale update
  if (grav_ramp_sec_ > 1e-6) {
    double step = std::clamp(dt_ / grav_ramp_sec_, 0.0, 1.0);
    grav_scale_ += (grav_scale_target_ - grav_scale_) * step;
  } else {
    grav_scale_ = grav_scale_target_;
  }

  const Vec6 tau_scaled = tau_g.cwiseProduct(grav_scale_);

  if (!tau_grav_lpf_init_) {
    tau_grav_lpf_state_ = tau_scaled;
    tau_grav_lpf_init_ = true;
    return tau_scaled;
  }

  for (int i = 0; i < 6; ++i) {
    if (tau_grav_lpf_alpha_[i] >= 1.0) {
      tau_grav_lpf_state_[i] = tau_scaled[i];
    } else {
      tau_grav_lpf_state_[i] +=
        tau_grav_lpf_alpha_[i] * (tau_scaled[i] - tau_grav_lpf_state_[i]);
    }
  }

  return tau_grav_lpf_state_;
}

}  // namespace teleop_cpp
