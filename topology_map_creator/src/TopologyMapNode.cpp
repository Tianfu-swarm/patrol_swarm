//Define all the topic names that are required for topological mapping
#include "rclcpp/rclcpp.hpp"
#include "topology_map_creator/msg/area2_d.hpp"

class TopologyMap : public rclcpp::Node
{
public:
  TopologyMap() : Node("topology_map")
  {
    patrol_area_subscription_ = this->create_subscription<topology_map_creator::msg::Area2D>(
      "/patrolArea", 10, std::bind(&TopologyMap::patrolAreaCallback, this, std::placeholders::_1));

    obstacle_area_subscription_ = this->create_subscription<topology_map_creator::msg::Area2D>(
      "/obstacleArea", 10, std::bind(&TopologyMap::obstacleAreaCallback, this, std::placeholders::_1));
  }

private:
  void patrolAreaCallback(const topology_map_creator::msg::Area2D::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received patrol area with %zu points", msg->points.size());
    // 在这里处理 patrolArea 消息
  }

  void obstacleAreaCallback(const topology_map_creator::msg::Area2D::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received obstacle area with %zu points", msg->points.size());
    // 在这里处理 obstacleArea 消息
  }

  rclcpp::Subscription<topology_map_creator::msg::Area2D>::SharedPtr patrol_area_subscription_;
  rclcpp::Subscription<topology_map_creator::msg::Area2D>::SharedPtr obstacle_area_subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopologyMap>());
  rclcpp::shutdown();
  return 0;
}
