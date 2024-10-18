#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "topology_map_creator/msg/area2_d.hpp"
#include "topology_map_creator/msg/matrix.hpp"
#include "topology_map_creator/msg/obstacle.hpp"

class TopologyMap : public rclcpp::Node
{
public:
  TopologyMap() : Node("topology_mapping")
  {
    map_area_subscription_ = this->create_subscription<topology_map_creator::msg::Area2D>(
        "/mapArea", 10, std::bind(&TopologyMap::mapAreaCallback, this, std::placeholders::_1));

    obstacle_area_subscription_ = this->create_subscription<topology_map_creator::msg::Obstacle>(
        "/obstacleArea", 10, std::bind(&TopologyMap::obstacleAreaCallback, this, std::placeholders::_1));

    topology_map_matrix_ = this->create_publisher<topology_map_creator::msg::Matrix>("/topology_graph_matrix", 10);

    topology_map_vertex_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/topology_map_vertex", 10);

    topology_map_edge_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/topology_map_edge", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&TopologyMap::processAndPublish, this));
  }

private:
  static constexpr size_t max_saved_messages_ = 10; // max number of message
  std::vector<topology_map_creator::msg::Area2D::SharedPtr> map_area_msgs_;
  std::vector<topology_map_creator::msg::Obstacle::SharedPtr> obstacle_area_msgs_;
  std::vector<topology_map_creator::msg::Area2D::SharedPtr> patrol_area_msgs_;
  topology_map_creator::msg::Matrix::SharedPtr topology_graph_matrix_;

  void mapAreaCallback(const topology_map_creator::msg::Area2D::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received map area with %zu points", msg->points.size());
    map_area_msgs_.push_back(msg);
    if (map_area_msgs_.size() > max_saved_messages_)
    {
      map_area_msgs_.erase(map_area_msgs_.begin());
    }
  }

  void obstacleAreaCallback(const topology_map_creator::msg::Obstacle::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received obstacle area with %zu obstacles", msg->obstacles.size());
    obstacle_area_msgs_.push_back(msg);
    if (obstacle_area_msgs_.size() > max_saved_messages_)
    {
      obstacle_area_msgs_.erase(obstacle_area_msgs_.begin());
    }
  }

  // main loop
  void processAndPublish()
  {
    topology_map_creator::msg::Matrix matrix_msg;
    if (!map_area_msgs_.empty() || !obstacle_area_msgs_.empty())
    {
      topology_map_creator::msg::Matrix matrix_msg;

      // 处理存储的消息并填充地图矩阵
      // 例如：matrix_msg.data = ... (填充矩阵数据)

      topology_map_matrix_->publish(matrix_msg);
      RCLCPP_INFO(rclcpp::get_logger("topology_mapping"), "Published topoMapMatrix");
    }
    topology_map_matrix_->publish(matrix_msg);
  }

  // Converting map information and obstacle information into patrol messages
  void processPatrolArea()
  {
    if (!map_area_msgs_.empty())
    { // The area of the mapArea
    }

    if (!obstacle_area_msgs_.empty())
    { // the area of the obstacleArea
    }
  }

  // visualization
  //  Take the topological graph matrix, convert it to vertexs and edges and visualize it
  void visualize_topology()
  {
    if (!topology_graph_matrix_)
    {
      RCLCPP_WARN(this->get_logger(), "Topology graph matrix is not initialized!");
      return;
    }

    // pub topology_map_vertex
    visualization_msgs::msg::MarkerArray vertex_marker_array;

    for (size_t i = 0; i < topology_graph_matrix_->position_x.size(); ++i)
    {
      visualization_msgs::msg::Marker vertex_marker;
      vertex_marker.header.frame_id = "map";
      vertex_marker.header.stamp = this->get_clock()->now();
      vertex_marker.ns = "vertexs";
      vertex_marker.id = i;
      vertex_marker.type = visualization_msgs::msg::Marker::SPHERE;
      vertex_marker.action = visualization_msgs::msg::Marker::ADD;
      vertex_marker.pose.position.x = topology_graph_matrix_->position_x[i];
      vertex_marker.pose.position.y = topology_graph_matrix_->position_y[i];
      vertex_marker.pose.position.z = 0.0;
      vertex_marker.scale.x = 0.2;
      vertex_marker.scale.y = 0.2;
      vertex_marker.scale.z = 0.2;
      vertex_marker.color.r = 1.0;
      vertex_marker.color.g = 0.0;
      vertex_marker.color.b = 0.0;
      vertex_marker.color.a = 1.0;

      vertex_marker_array.markers.push_back(vertex_marker);
    }

    topology_map_vertex_pub_->publish(vertex_marker_array);

    // pub topology_map_edge
    for (size_t i = 0; i < topology_graph_matrix_->position_x.size(); ++i)
    {
      for (size_t j = 0; j < topology_graph_matrix_->position_x.size(); ++j)
      {
      }
    }
  }

  rclcpp::Subscription<topology_map_creator::msg::Area2D>::SharedPtr map_area_subscription_;
  rclcpp::Subscription<topology_map_creator::msg::Obstacle>::SharedPtr obstacle_area_subscription_;
  rclcpp::Publisher<topology_map_creator::msg::Matrix>::SharedPtr topology_map_matrix_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr topology_map_vertex_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr topology_map_edge_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopologyMap>());
  rclcpp::shutdown();
  return 0;
}
