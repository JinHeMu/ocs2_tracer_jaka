#include <rclcpp/rclcpp.hpp>
#include "force_admittance_servo/force_admittance_servo_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<force_admittance_servo::ForceAdmittanceServoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
