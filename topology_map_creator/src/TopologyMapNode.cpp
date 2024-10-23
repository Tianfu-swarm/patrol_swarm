#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "topology_map_creator/msg/area2_d.hpp"
#include "topology_map_creator/msg/matrix.hpp"
#include "topology_map_creator/msg/obstacle.hpp"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_2_algorithms.h>

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

    map_area_edge_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/map_area_edge_pub", 10);

    obstacle_area_edge_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/obstacle_area_edge_pub", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&TopologyMap::processAndPublish, this));
  }

private:
  static constexpr size_t max_saved_messages_ = 10; // max number of message
  std::vector<topology_map_creator::msg::Area2D::SharedPtr> map_area_msgs_;
  std::vector<topology_map_creator::msg::Obstacle::SharedPtr> obstacle_area_msgs_;
  std::vector<topology_map_creator::msg::Area2D::SharedPtr> patrol_area_msgs_;
  topology_map_creator::msg::Matrix::SharedPtr topology_graph_matrix_;

  bool new_map_area_received_ = false;
  bool new_obstacle_area_received_ = false;

  void mapAreaCallback(const topology_map_creator::msg::Area2D::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received map area with %zu points", msg->points.size());
    map_area_msgs_.push_back(msg);
    if (map_area_msgs_.size() > max_saved_messages_)
    {
      map_area_msgs_.erase(map_area_msgs_.begin());
    }
    new_map_area_received_ = true;
  }

  void obstacleAreaCallback(const topology_map_creator::msg::Obstacle::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received obstacle area with %zu obstacles", msg->obstacles.size());
    obstacle_area_msgs_.push_back(msg);
    if (obstacle_area_msgs_.size() > max_saved_messages_)
    {
      obstacle_area_msgs_.erase(obstacle_area_msgs_.begin());
    }
    new_obstacle_area_received_ = true;
  }

  // main loop
  void processAndPublish()
  {

    visualization();
    if (new_map_area_received_ || new_obstacle_area_received_)
    {
      std::cout << "new message" << std::endl;
      processPatrolArea();
      topoMapMatrix();
    }
    new_map_area_received_ = false;
    new_obstacle_area_received_ = false;
  }

  // topoMapMatrix
  void topoMapMatrix()
  {
    topology_map_creator::msg::Matrix matrix_msg;
    if (!map_area_msgs_.empty() || !obstacle_area_msgs_.empty())
    {

      // 处理存储的消息并填充地图矩阵
      // 例如：matrix_msg.data = ... (填充矩阵数据)

      topology_map_matrix_->publish(matrix_msg);
      RCLCPP_INFO(rclcpp::get_logger("topology_mapping"), "Published topoMapMatrix");
    }
  }

  // Converting map and obstacle into patrol
  void processPatrolArea()
  {
    typedef CGAL::Simple_cartesian<double> K;
    typedef K::Point_2 Point_2;
    typedef CGAL::Polygon_2<K> Polygon_2;
    typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes_2;

    if (!map_area_msgs_.empty()) // The area of the mapArea
    {
      size_t lasted_message = map_area_msgs_.size();
      Polygon_2 P1, P2;

      for (const auto &point : map_area_msgs_[lasted_message - 1]->points)
      {
        P1.push_back(Point_2(point.x, point.y));
      }

      if (lasted_message > 1)
      {
        for (const auto &point : map_area_msgs_[lasted_message - 2]->points)
        {
          P2.push_back(Point_2(point.x, point.y));
        }
      }

      std::vector<Polygon_with_holes_2> result;
      CGAL::difference(P1, P2, std::back_inserter(result));

      double total_area = 0.0;
      for (const auto &poly : result)
      {
        total_area += CGAL::to_double(CGAL::polygon_area_2(poly.outer_boundary().vertices_begin(), poly.outer_boundary().vertices_end(), K()));
        for (const auto &hole : poly.holes())
        {
          total_area -= CGAL::to_double(CGAL::polygon_area_2(hole.vertices_begin(), hole.vertices_end(), K()));
        }
      }

      std::cout << "Updated patrol map total area is: " << total_area << std::endl;
    }

    if (!obstacle_area_msgs_.empty())
    {
      // 声明巡逻区域
      std::vector<Point_2> patrol_points;
      size_t last_message_index = map_area_msgs_.size() - 1;

      // 从地图消息中提取巡逻区域的点
      for (const auto &point : map_area_msgs_[last_message_index]->points)
      {
        patrol_points.push_back(Point_2(point.x, point.y));
      }

      // 创建外边界的 Polygon_2
      Polygon_2 outer_boundary(patrol_points.begin(), patrol_points.end());
      std::vector<Polygon_with_holes_2> patrol_area; // 将 patrol_area 修改为 vector 类型
      patrol_area.emplace_back(outer_boundary);      // 创建 patrol_area

      // 获取障碍物消息
      size_t last_obstacle_message_index = obstacle_area_msgs_.size() - 1;
      const auto &last_obstacle_msg = obstacle_area_msgs_[last_obstacle_message_index];

      // 遍历所有障碍物区域
      for (const auto &obstacle : last_obstacle_msg->obstacles)
      {
        Polygon_2 obstacle_area;

        // 将障碍物区域的点转换为 Polygon_2 对象
        for (const auto &point : obstacle.points)
        {
          obstacle_area.push_back(Point_2(point.x, point.y));
        }

        // 计算巡逻区域与障碍物区域的差集
        std::vector<Polygon_with_holes_2> difference_result;
        for (const auto &area : patrol_area) // 遍历当前的巡逻区域
        {
          CGAL::difference(area, obstacle_area, std::back_inserter(difference_result));
        }

        // 如果差集结果有效，更新巡逻区域为新的差集结果
        if (!difference_result.empty())
        {
          patrol_area = std::move(difference_result); // 更新巡逻区域为差集结果
        }
        else
        {
          RCLCPP_WARN(rclcpp::get_logger("PatrolAreaLogger"), "Patrol area is completely covered by obstacles.");
          // 可以在这里处理完全覆盖的情况
          break; // 直接跳出循环，因为后续没有意义
        }
      }

      // 计算更新后的巡逻区域的面积
      double total_area = 0.0;
      for (const auto &poly : patrol_area)
      {
        total_area += CGAL::to_double(CGAL::polygon_area_2(poly.outer_boundary().vertices_begin(), poly.outer_boundary().vertices_end(), K()));
        for (const auto &hole : poly.holes())
        {
          total_area -= CGAL::to_double(CGAL::polygon_area_2(hole.vertices_begin(), hole.vertices_end(), K()));
        }
      }

      std::cout << "Patrol area is: " << total_area << std::endl;
    }
  }

  //***************************** visualization****************************/
  void visualization()
  {
    visualize_topology_map();
    visualize_map_area();
    visualize_obstacle_area();
  }
  //  Take the topological graph matrix, convert it to vertexs and edges and visualize it
  void visualize_topology_map()
  {
    topology_graph_matrix_ = std::make_shared<topology_map_creator::msg::Matrix>();
    // 设置位置
    topology_graph_matrix_->position_x = {1.0, 2.0, 3.0, 2.0};
    topology_graph_matrix_->position_y = {1.0, 2.0, 1.0, 0.0};

    // 设置连接矩阵
    topology_graph_matrix_->matrix = {
        0.0, 1.0, 0.0, 1.0, // 点 0 与 点 1 和 点 3 相连
        1.0, 0.0, 1.0, 0.0, // 点 1 与 点 0 和 点 2 相连
        0.0, 1.0, 0.0, 1.0, // 点 2 与 点 1 和 点 3 相连
        1.0, 0.0, 1.0, 0.0  // 点 3 与 点 0 相连
    };

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
    visualization_msgs::msg::MarkerArray edge_marker_array;

    for (size_t i = 0; i < topology_graph_matrix_->position_x.size(); ++i)
    {
      for (size_t j = i; j < topology_graph_matrix_->position_x.size(); ++j)
      {
        if (topology_graph_matrix_->matrix[i * topology_graph_matrix_->position_x.size() + j] == 1)
        {
          visualization_msgs::msg::Marker line_marker;
          line_marker.header.frame_id = "map"; // 根据你的框架调整
          line_marker.header.stamp = rclcpp::Time();
          line_marker.ns = "edges";
          line_marker.id = edge_marker_array.markers.size(); // 递增的ID
          line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
          line_marker.action = visualization_msgs::msg::Marker::ADD;
          line_marker.scale.x = 0.1; // 线宽
          line_marker.color.r = 0.0;
          line_marker.color.g = 0.0;
          line_marker.color.b = 1.0;
          line_marker.color.a = 1.0; // 不透明

          // 设置起点和终点
          geometry_msgs::msg::Point start_point;
          start_point.x = topology_graph_matrix_->position_x[i];
          start_point.y = topology_graph_matrix_->position_y[i];
          start_point.z = 0.0;

          geometry_msgs::msg::Point end_point;
          end_point.x = topology_graph_matrix_->position_x[j];
          end_point.y = topology_graph_matrix_->position_y[j];
          end_point.z = 0.0;

          line_marker.points.push_back(start_point);
          line_marker.points.push_back(end_point);

          // 将线段添加到MarkerArray中
          edge_marker_array.markers.push_back(line_marker);
        }
      }
    }

    topology_map_edge_pub_->publish(edge_marker_array);
  }

  void visualize_map_area()
  {
    if (!map_area_msgs_.empty())
    {
      visualization_msgs::msg::MarkerArray map_area_marker_array;
      size_t last_message_index = map_area_msgs_.size() - 1;
      for (size_t i = 0; i < map_area_msgs_[last_message_index]->points.size(); ++i)
      {
        visualization_msgs::msg::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = rclcpp::Time();
        line_marker.ns = "edges";
        line_marker.id = map_area_marker_array.markers.size();
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        line_marker.scale.x = 0.05; // 线宽
        line_marker.color.r = 0.0;
        line_marker.color.g = 0.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 1.0; // 不透明

        // 设置起点和终点
        geometry_msgs::msg::Point start_point;
        geometry_msgs::msg::Point end_point;
        if (i == map_area_msgs_[last_message_index]->points.size() - 1)
        {
          start_point.x = map_area_msgs_[last_message_index]->points[i].x;
          start_point.y = map_area_msgs_[last_message_index]->points[i].y;
          start_point.z = 0.0;

          end_point.x = map_area_msgs_[last_message_index]->points[0].x;
          end_point.y = map_area_msgs_[last_message_index]->points[0].y;
          end_point.z = 0.0;
        }
        else
        {
          start_point.x = map_area_msgs_[last_message_index]->points[i].x;
          start_point.y = map_area_msgs_[last_message_index]->points[i].y;
          start_point.z = 0.0;

          end_point.x = map_area_msgs_[last_message_index]->points[i + 1].x;
          end_point.y = map_area_msgs_[last_message_index]->points[i + 1].y;
          end_point.z = 0.0;
        }

        line_marker.points.push_back(start_point);
        line_marker.points.push_back(end_point);

        // 将线段添加到MarkerArray中
        map_area_marker_array.markers.push_back(line_marker);
      }

      map_area_edge_pub_->publish(map_area_marker_array);
    }
  }

  void visualize_obstacle_area()
  {
    if (!obstacle_area_msgs_.empty())
    {
      visualization_msgs::msg::MarkerArray obstacle_area_marker_array;
      size_t last_message_index = obstacle_area_msgs_.size() - 1;

      for (size_t i = 0; i < obstacle_area_msgs_[last_message_index]->obstacles.size(); i++)
      {
        const auto &obstacle = obstacle_area_msgs_[last_message_index]->obstacles[i];
        for (size_t j = 0; j < obstacle.points.size(); j++)
        {
          visualization_msgs::msg::Marker line_marker;
          line_marker.header.frame_id = "map";
          line_marker.header.stamp = rclcpp::Time();
          line_marker.ns = "edges";
          line_marker.id = obstacle_area_marker_array.markers.size();
          line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
          line_marker.action = visualization_msgs::msg::Marker::ADD;
          line_marker.scale.x = 0.05; // 线宽
          line_marker.color.r = 0.0;
          line_marker.color.g = 0.0;
          line_marker.color.b = 0.0;
          line_marker.color.a = 1.0; // 不透明
                                     // 设置起点和终点
          geometry_msgs::msg::Point start_point;
          geometry_msgs::msg::Point end_point;

          if (j != obstacle.points.size() - 1)
          {
            start_point.x = obstacle.points[j].x;
            start_point.y = obstacle.points[j].y;
            start_point.z = 0.0;

            end_point.x = obstacle.points[j + 1].x;
            end_point.y = obstacle.points[j + 1].y;
            end_point.z = 0.0;
          }
          else
          {
            start_point.x = obstacle.points[j].x;
            start_point.y = obstacle.points[j].y;
            start_point.z = 0.0;

            end_point.x = obstacle.points[0].x;
            end_point.y = obstacle.points[0].y;
            end_point.z = 0.0;
          }
          line_marker.points.push_back(start_point);
          line_marker.points.push_back(end_point);

          // 将线段添加到MarkerArray中
          obstacle_area_marker_array.markers.push_back(line_marker);
        }
      }
      obstacle_area_edge_pub_->publish(obstacle_area_marker_array);
    }
  }

  rclcpp::Subscription<topology_map_creator::msg::Area2D>::SharedPtr map_area_subscription_;
  rclcpp::Subscription<topology_map_creator::msg::Obstacle>::SharedPtr obstacle_area_subscription_;
  rclcpp::Publisher<topology_map_creator::msg::Matrix>::SharedPtr topology_map_matrix_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr topology_map_vertex_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr topology_map_edge_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_area_edge_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_area_edge_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopologyMap>());
  rclcpp::shutdown();
  return 0;
}
