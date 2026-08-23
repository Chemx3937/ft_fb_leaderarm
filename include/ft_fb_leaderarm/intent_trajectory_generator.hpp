#pragma once

#include <Eigen/Dense>

namespace teleop_cpp {

struct IntentTrajectoryConfig {
  bool enabled{true};
  double linear_natural_frequency_hz{4.0};
  double angular_natural_frequency_hz{4.0};
  double damping_ratio{1.0};
  double max_linear_velocity_m_s{0.30};
  double max_linear_acceleration_m_s2{1.0};
  double max_angular_velocity_rad_s{5.235987755982989};
  double max_angular_acceleration_rad_s2{12.566370614359172};
};

struct IntentTrajectoryState {
  Eigen::Vector3d position_m{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d linear_velocity_m_s{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_acceleration_m_s2{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity_rad_s{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_acceleration_rad_s2{Eigen::Vector3d::Zero()};
};

// Causal, critically-damped-by-default reference generator for the command path.
// Translation, rotation error, and angular state are integrated in the workspace
// frame. A fixed command-frame transform therefore only rotates the reported
// velocity/acceleration vectors; it does not change the filtering behavior.
class IntentTrajectoryGenerator {
public:
  explicit IntentTrajectoryGenerator(
      const IntentTrajectoryConfig& config = IntentTrajectoryConfig{});

  void configure(const IntentTrajectoryConfig& config);
  void reset(const Eigen::Vector3d& position_m, const Eigen::Matrix3d& rotation);
  const IntentTrajectoryState& update(
      const Eigen::Vector3d& raw_position_m,
      const Eigen::Matrix3d& raw_rotation,
      double elapsed_s);

  bool initialized() const { return initialized_; }
  const IntentTrajectoryConfig& config() const { return config_; }
  const IntentTrajectoryState& state() const { return state_; }

private:
  static Eigen::Vector3d limit_norm(
      const Eigen::Vector3d& value, double maximum);
  static Eigen::Vector3d rotation_log(const Eigen::Matrix3d& rotation);
  static Eigen::Matrix3d rotation_exp(const Eigen::Vector3d& rotation_vector);
  static void validate_rotation(const Eigen::Matrix3d& rotation);

  void integrate_once(
      const Eigen::Vector3d& raw_position_m,
      const Eigen::Matrix3d& raw_rotation,
      double dt_s);

  IntentTrajectoryConfig config_;
  IntentTrajectoryState state_;
  bool initialized_{false};
};

}  // namespace teleop_cpp
