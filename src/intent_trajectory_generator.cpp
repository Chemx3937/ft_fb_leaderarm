#include "ft_fb_leaderarm/intent_trajectory_generator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>

namespace teleop_cpp {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMaxElapsedS = 0.050;
constexpr double kMaxIntegrationStepS = 0.002;

void require_positive_finite(double value, const char* name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and > 0");
  }
}

void require_nonnegative_finite(double value, const char* name) {
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and >= 0");
  }
}

}  // namespace

IntentTrajectoryGenerator::IntentTrajectoryGenerator(
    const IntentTrajectoryConfig& config) {
  configure(config);
}

void IntentTrajectoryGenerator::configure(
    const IntentTrajectoryConfig& config) {
  require_positive_finite(
    config.linear_natural_frequency_hz, "linear_natural_frequency_hz");
  require_positive_finite(
    config.angular_natural_frequency_hz, "angular_natural_frequency_hz");
  require_positive_finite(config.damping_ratio, "damping_ratio");
  require_nonnegative_finite(
    config.max_linear_velocity_m_s, "max_linear_velocity_m_s");
  require_nonnegative_finite(
    config.max_linear_acceleration_m_s2, "max_linear_acceleration_m_s2");
  require_nonnegative_finite(
    config.max_angular_velocity_rad_s, "max_angular_velocity_rad_s");
  require_nonnegative_finite(
    config.max_angular_acceleration_rad_s2,
    "max_angular_acceleration_rad_s2");
  config_ = config;
}

void IntentTrajectoryGenerator::validate_rotation(
    const Eigen::Matrix3d& rotation) {
  if (!rotation.allFinite()) {
    throw std::invalid_argument("rotation must be finite");
  }
  const Eigen::Matrix3d orthogonality =
    rotation.transpose() * rotation - Eigen::Matrix3d::Identity();
  if (orthogonality.norm() > 1.0e-6 ||
      std::abs(rotation.determinant() - 1.0) > 1.0e-6) {
    throw std::invalid_argument("rotation must be a valid SO(3) matrix");
  }
}

void IntentTrajectoryGenerator::reset(
    const Eigen::Vector3d& position_m,
    const Eigen::Matrix3d& rotation) {
  if (!position_m.allFinite()) {
    throw std::invalid_argument("position must be finite");
  }
  validate_rotation(rotation);
  state_ = IntentTrajectoryState{};
  state_.position_m = position_m;
  state_.rotation = rotation;
  initialized_ = true;
}

Eigen::Vector3d IntentTrajectoryGenerator::limit_norm(
    const Eigen::Vector3d& value, double maximum) {
  if (maximum <= 0.0) return value;
  const double norm = value.norm();
  if (norm <= maximum || norm <= 1.0e-15) return value;
  return value * (maximum / norm);
}

Eigen::Vector3d IntentTrajectoryGenerator::rotation_log(
    const Eigen::Matrix3d& rotation) {
  Eigen::AngleAxisd angle_axis(rotation);
  if (!std::isfinite(angle_axis.angle()) ||
      !angle_axis.axis().allFinite() ||
      std::abs(angle_axis.angle()) <= 1.0e-15) {
    return Eigen::Vector3d::Zero();
  }
  return angle_axis.axis() * angle_axis.angle();
}

Eigen::Matrix3d IntentTrajectoryGenerator::rotation_exp(
    const Eigen::Vector3d& rotation_vector) {
  const double angle = rotation_vector.norm();
  if (angle <= 1.0e-15) return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(angle, rotation_vector / angle).toRotationMatrix();
}

void IntentTrajectoryGenerator::integrate_once(
    const Eigen::Vector3d& raw_position_m,
    const Eigen::Matrix3d& raw_rotation,
    double dt_s) {
  const double wn_linear =
    2.0 * kPi * config_.linear_natural_frequency_hz;
  Eigen::Vector3d desired_linear_acceleration =
    wn_linear * wn_linear * (raw_position_m - state_.position_m) -
    2.0 * config_.damping_ratio * wn_linear *
      state_.linear_velocity_m_s;
  state_.linear_acceleration_m_s2 = limit_norm(
    desired_linear_acceleration, config_.max_linear_acceleration_m_s2);
  state_.linear_velocity_m_s = limit_norm(
    state_.linear_velocity_m_s +
      state_.linear_acceleration_m_s2 * dt_s,
    config_.max_linear_velocity_m_s);
  state_.position_m += state_.linear_velocity_m_s * dt_s;

  const double wn_angular =
    2.0 * kPi * config_.angular_natural_frequency_hz;
  const Eigen::Vector3d rotation_error =
    state_.rotation *
      rotation_log(state_.rotation.transpose() * raw_rotation);
  Eigen::Vector3d desired_angular_acceleration =
    wn_angular * wn_angular * rotation_error -
    2.0 * config_.damping_ratio * wn_angular *
      state_.angular_velocity_rad_s;
  state_.angular_acceleration_rad_s2 = limit_norm(
    desired_angular_acceleration,
    config_.max_angular_acceleration_rad_s2);
  state_.angular_velocity_rad_s = limit_norm(
    state_.angular_velocity_rad_s +
      state_.angular_acceleration_rad_s2 * dt_s,
    config_.max_angular_velocity_rad_s);
  state_.rotation =
    rotation_exp(state_.angular_velocity_rad_s * dt_s) * state_.rotation;
}

const IntentTrajectoryState& IntentTrajectoryGenerator::update(
    const Eigen::Vector3d& raw_position_m,
    const Eigen::Matrix3d& raw_rotation,
    double elapsed_s) {
  if (!raw_position_m.allFinite()) {
    throw std::invalid_argument("raw_position_m must be finite");
  }
  validate_rotation(raw_rotation);
  if (!std::isfinite(elapsed_s) || elapsed_s <= 0.0) {
    throw std::invalid_argument("elapsed_s must be finite and > 0");
  }
  if (!initialized_ || !config_.enabled) {
    reset(raw_position_m, raw_rotation);
    return state_;
  }

  const double bounded_elapsed = std::min(elapsed_s, kMaxElapsedS);
  const double frequency_limited_step = 1.0 /
    (20.0 * std::max(
      config_.linear_natural_frequency_hz,
      config_.angular_natural_frequency_hz));
  const double max_integration_step =
    std::min(kMaxIntegrationStepS, frequency_limited_step);
  const int substeps = std::max(
    1, static_cast<int>(std::ceil(
      bounded_elapsed / max_integration_step)));
  const double step_s = bounded_elapsed / static_cast<double>(substeps);
  for (int i = 0; i < substeps; ++i) {
    integrate_once(raw_position_m, raw_rotation, step_s);
  }
  return state_;
}

}  // namespace teleop_cpp
