// Copyright <2021> 

#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <roboteq-flw200_node/roboteq-flw200.hpp>

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FlowSensor>("roboteq_flw200_node", rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();

  node = nullptr;
  
  return 0;
}
