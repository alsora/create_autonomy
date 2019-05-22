
#include "rclcpp/rclcpp.hpp"

#include "ca_driver/create_driver_node.hpp"


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CreateDriverNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}