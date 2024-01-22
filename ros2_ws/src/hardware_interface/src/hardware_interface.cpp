#include "rclcpp/rclcpp.hpp"
#include "humanoid_msgs/msg/motor_states.hpp"

void motor_states_cb(const std::shared_ptr<humanoid_msgs::msg::MotorStates> msg) {
  RCLCPP_INFO(rclcpp::get_logger("hardware_interface"), "left_ankle_pitch: %f", msg->left_ankle_pitch);
  RCLCPP_INFO(rclcpp::get_logger("hardware_interface"), "left_knee: %f", msg->left_knee);
  RCLCPP_INFO(rclcpp::get_logger("hardware_interface"), "left_hip_roll: %f", msg->left_hip_roll);
  RCLCPP_INFO(rclcpp::get_logger("hardware_interface"), "left_hip_pitch: %f", msg->left_hip_pitch);
  RCLCPP_INFO(rclcpp::get_logger("hardware_interface"), "left_hip_yaw: %f", msg->left_hip_yaw);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("hardware_interface");

  auto subscription = node->create_subscription<humanoid_msgs::msg::MotorStates>(
      "/microros/motor_states",
      rclcpp::QoS(rclcpp::KeepLast(10)),
      motor_states_cb);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
