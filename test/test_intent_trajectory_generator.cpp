#include "ft_fb_leaderarm/intent_trajectory_generator.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace teleop_cpp {
namespace {

constexpr double kPi = 3.14159265358979323846;

double sinusoid_amplitude(
    const std::vector<double>& samples,
    double frequency_hz,
    double sample_hz,
    int first_sample) {
  double sin_sum = 0.0;
  double cos_sum = 0.0;
  const int count = static_cast<int>(samples.size()) - first_sample;
  for (int i = first_sample; i < static_cast<int>(samples.size()); ++i) {
    const double phase = 2.0 * kPi * frequency_hz * i / sample_hz;
    sin_sum += samples[i] * std::sin(phase);
    cos_sum += samples[i] * std::cos(phase);
  }
  return 2.0 * std::hypot(sin_sum, cos_sum) / static_cast<double>(count);
}

TEST(IntentTrajectoryGenerator, InitializesWithoutACommandJump) {
  IntentTrajectoryGenerator generator;
  const Eigen::Vector3d position(0.2, -0.1, 0.3);
  const Eigen::Matrix3d rotation =
    Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()).toRotationMatrix();

  const auto& state = generator.update(position, rotation, 0.002);

  EXPECT_TRUE(generator.initialized());
  EXPECT_TRUE(state.position_m.isApprox(position, 1.0e-12));
  EXPECT_TRUE(state.rotation.isApprox(rotation, 1.0e-12));
  EXPECT_DOUBLE_EQ(state.linear_velocity_m_s.norm(), 0.0);
  EXPECT_DOUBLE_EQ(state.angular_velocity_rad_s.norm(), 0.0);
}

TEST(IntentTrajectoryGenerator, AttenuatesHighFrequencyWhileKeepingSlowIntent) {
  IntentTrajectoryConfig config;
  config.linear_natural_frequency_hz = 4.0;
  config.angular_natural_frequency_hz = 4.0;
  config.damping_ratio = 1.0;
  config.max_linear_velocity_m_s = 0.0;
  config.max_linear_acceleration_m_s2 = 0.0;
  config.max_linear_jerk_m_s3 = 0.0;
  IntentTrajectoryGenerator generator(config);

  constexpr double sample_hz = 500.0;
  constexpr double dt = 1.0 / sample_hz;
  std::vector<double> raw;
  std::vector<double> intent;
  raw.reserve(5000);
  intent.reserve(5000);
  for (int i = 0; i < 5000; ++i) {
    const double t = i * dt;
    const double x =
      0.050 * std::sin(2.0 * kPi * 0.5 * t) +
      0.005 * std::sin(2.0 * kPi * 10.0 * t);
    raw.push_back(x);
    intent.push_back(generator.update(
      Eigen::Vector3d(x, 0.0, 0.0), Eigen::Matrix3d::Identity(), dt)
      .position_m.x());
  }

  const double raw_slow = sinusoid_amplitude(raw, 0.5, sample_hz, 1000);
  const double intent_slow = sinusoid_amplitude(intent, 0.5, sample_hz, 1000);
  const double raw_high = sinusoid_amplitude(raw, 10.0, sample_hz, 1000);
  const double intent_high = sinusoid_amplitude(intent, 10.0, sample_hz, 1000);
  EXPECT_GT(intent_slow / raw_slow, 0.90);
  EXPECT_LT(intent_high / raw_high, 0.20);
}

TEST(IntentTrajectoryGenerator, EnforcesVelocityAccelerationAndJerkLimits) {
  IntentTrajectoryConfig config;
  config.max_linear_velocity_m_s = 0.10;
  config.max_linear_acceleration_m_s2 = 0.20;
  config.max_linear_jerk_m_s3 = 0.50;
  config.max_angular_velocity_rad_s = 0.30;
  config.max_angular_acceleration_rad_s2 = 0.60;
  config.max_angular_jerk_rad_s3 = 1.50;
  IntentTrajectoryGenerator generator(config);
  generator.reset(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());

  constexpr double dt = 0.002;
  Eigen::Vector3d previous_linear_acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d previous_angular_acceleration = Eigen::Vector3d::Zero();
  const Eigen::Matrix3d target_rotation =
    Eigen::AngleAxisd(kPi / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  for (int i = 0; i < 2000; ++i) {
    const auto& state = generator.update(
      Eigen::Vector3d(1.0, 0.0, 0.0), target_rotation, dt);
    EXPECT_LE(state.linear_velocity_m_s.norm(), 0.10 + 1.0e-12);
    EXPECT_LE(state.linear_acceleration_m_s2.norm(), 0.20 + 1.0e-12);
    EXPECT_LE(state.angular_velocity_rad_s.norm(), 0.30 + 1.0e-12);
    EXPECT_LE(state.angular_acceleration_rad_s2.norm(), 0.60 + 1.0e-12);
    EXPECT_LE(
      (state.linear_acceleration_m_s2 - previous_linear_acceleration).norm(),
      0.50 * dt + 1.0e-12);
    EXPECT_LE(
      (state.angular_acceleration_rad_s2 - previous_angular_acceleration).norm(),
      1.50 * dt + 1.0e-12);
    EXPECT_NEAR(
      (state.rotation.transpose() * state.rotation -
       Eigen::Matrix3d::Identity()).norm(),
      0.0, 1.0e-10);
    EXPECT_NEAR(state.rotation.determinant(), 1.0, 1.0e-10);
    previous_linear_acceleration = state.linear_acceleration_m_s2;
    previous_angular_acceleration = state.angular_acceleration_rad_s2;
  }
}

TEST(IntentTrajectoryGenerator, IsInvariantToFixedCommandFrameTransform) {
  IntentTrajectoryGenerator workspace_generator;
  IntentTrajectoryGenerator command_generator;
  const Eigen::Matrix3d command_R_workspace =
    Eigen::AngleAxisd(0.7, Eigen::Vector3d(1.0, 2.0, 3.0).normalized())
      .toRotationMatrix();
  const Eigen::Vector3d command_translation(0.4, -0.2, 0.1);
  const Eigen::Vector3d initial_position(0.2, 0.1, -0.1);
  const Eigen::Matrix3d initial_rotation =
    Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitY()).toRotationMatrix();
  workspace_generator.reset(initial_position, initial_rotation);
  command_generator.reset(
    command_R_workspace * initial_position + command_translation,
    command_R_workspace * initial_rotation);

  constexpr double dt = 0.002;
  for (int i = 0; i < 1000; ++i) {
    const double t = i * dt;
    const Eigen::Vector3d raw_position(
      0.2 + 0.04 * std::sin(t),
      0.1 - 0.02 * std::cos(0.7 * t),
      -0.1 + 0.03 * std::sin(0.4 * t));
    const Eigen::Matrix3d raw_rotation =
      Eigen::AngleAxisd(
        0.2 * std::sin(0.8 * t),
        Eigen::Vector3d(1.0, -1.0, 0.5).normalized()).toRotationMatrix() *
      initial_rotation;
    const auto& workspace_state = workspace_generator.update(
      raw_position, raw_rotation, dt);
    const auto& command_state = command_generator.update(
      command_R_workspace * raw_position + command_translation,
      command_R_workspace * raw_rotation,
      dt);

    EXPECT_TRUE(command_state.position_m.isApprox(
      command_R_workspace * workspace_state.position_m + command_translation,
      1.0e-10));
    EXPECT_TRUE(command_state.rotation.isApprox(
      command_R_workspace * workspace_state.rotation, 1.0e-10));
    EXPECT_TRUE(command_state.linear_velocity_m_s.isApprox(
      command_R_workspace * workspace_state.linear_velocity_m_s, 1.0e-10));
    EXPECT_TRUE(command_state.angular_velocity_rad_s.isApprox(
      command_R_workspace * workspace_state.angular_velocity_rad_s, 1.0e-10));
  }
}

TEST(IntentTrajectoryGenerator, RejectsUnsafeConfiguration) {
  IntentTrajectoryConfig config;
  config.damping_ratio = 0.0;
  EXPECT_THROW(IntentTrajectoryGenerator generator(config), std::invalid_argument);

  config = IntentTrajectoryConfig{};
  config.max_linear_acceleration_m_s2 = -1.0;
  EXPECT_THROW(IntentTrajectoryGenerator generator(config), std::invalid_argument);
}

}  // namespace
}  // namespace teleop_cpp
