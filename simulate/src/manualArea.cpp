#include "rclcpp/rclcpp.hpp"

class ManualArea : public rclcpp::Node
{
public:
  ManualArea() : Node("manual_area")
  {
    RCLCPP_INFO(this->get_logger(), "ManualArea node has been started.");
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManualArea>());
  rclcpp::shutdown();
  return 0;
}
