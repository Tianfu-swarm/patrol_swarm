#include "rclcpp/rclcpp.hpp"
#include "topology_map_creator/msg/area2_d.hpp"
#include "topology_map_creator/msg/matrix.hpp"

class TopologyMap : public rclcpp::Node
{
public:
  TopologyMap() : Node("topology_mapping")
  {
    patrol_area_subscription_ = this->create_subscription<topology_map_creator::msg::Area2D>(
        "/patrolArea", 10, std::bind(&TopologyMap::patrolAreaCallback, this, std::placeholders::_1));

    obstacle_area_subscription_ = this->create_subscription<topology_map_creator::msg::Area2D>(
        "/obstacleArea", 10, std::bind(&TopologyMap::obstacleAreaCallback, this, std::placeholders::_1));

    topology_map_matrix_ = this->create_publisher<topology_map_creator::msg::Matrix>("/topoMapMatrix", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&TopologyMap::processAndPublish, this));
  }

private:

  static constexpr size_t max_saved_messages_ = 10; // max number of message
  std::vector<topology_map_creator::msg::Area2D::SharedPtr> patrol_area_msgs_;
  std::vector<topology_map_creator::msg::Area2D::SharedPtr> obstacle_area_msgs_;

  void patrolAreaCallback(const topology_map_creator::msg::Area2D::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received patrol area with %zu points", msg->points.size());
    patrol_area_msgs_.push_back(msg);
    if (patrol_area_msgs_.size() > max_saved_messages_)
    {
      patrol_area_msgs_.erase(patrol_area_msgs_.begin());
    }
  }

  void obstacleAreaCallback(const topology_map_creator::msg::Area2D::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received obstacle area with %zu points", msg->points.size());
    obstacle_area_msgs_.push_back(msg);
    if (obstacle_area_msgs_.size() > max_saved_messages_)
    {
      obstacle_area_msgs_.erase(obstacle_area_msgs_.begin());
    }
  }

  void processAndPublish();

  rclcpp::Subscription<topology_map_creator::msg::Area2D>::SharedPtr patrol_area_subscription_;
  rclcpp::Subscription<topology_map_creator::msg::Area2D>::SharedPtr obstacle_area_subscription_;
  rclcpp::Publisher<topology_map_creator::msg::Matrix>::SharedPtr topology_map_matrix_;
  rclcpp::TimerBase::SharedPtr timer_;
};

void TopologyMap::processAndPublish()
{
  topology_map_creator::msg::Matrix matrix_msg;
  if (!patrol_area_msgs_.empty() || !obstacle_area_msgs_.empty())
  {
    topology_map_creator::msg::Matrix matrix_msg;

    // 处理存储的消息并填充地图矩阵
    // 例如：matrix_msg.data = ... (填充矩阵数据)

    topology_map_matrix_->publish(matrix_msg);
    RCLCPP_INFO(rclcpp::get_logger("topology_mapping"), "Published topoMapMatrix");
  }
  topology_map_matrix_->publish(matrix_msg);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopologyMap>());
  rclcpp::shutdown();
  return 0;
}
