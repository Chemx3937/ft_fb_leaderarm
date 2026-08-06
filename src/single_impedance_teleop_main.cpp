// single_impedance_teleop_main.cpp
// Single impedance leader teleop MultiThreadedExecutor 진입점
// (원본: rclpy.spin(node) → 단일 스레드, subscriber 콜백이 제어루프 블로킹)
// (C++: MultiThreadedExecutor → subscriber 콜백과 제어 timer 병렬 실행)

#include <rclcpp/rclcpp.hpp>
#include "ft_fb_leaderarm/single_impedance_teleop_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<teleop_cpp::LeaderTeleopNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
