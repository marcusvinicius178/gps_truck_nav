#pragma once

#include <vector>
#include <string>
#include <mutex>
#include <memory>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav_virtual_lanes_planner
{

class VirtualLanesPlanner : public nav2_core::GlobalPlanner
{
public:
  VirtualLanesPlanner();
  ~VirtualLanesPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  void handleMarker(const visualization_msgs::msg::Marker::SharedPtr msg);

  geometry_msgs::msg::PoseStamped transformToGlobalFrame(
    const geometry_msgs::msg::PoseStamped & pose) const;

  std::size_t nearestIndex(
    const geometry_msgs::msg::PoseStamped & ref,
    const std::vector<geometry_msgs::msg::PoseStamped> & pts) const;

  std::vector<geometry_msgs::msg::PoseStamped> resamplePath(
    const std::vector<geometry_msgs::msg::PoseStamped> & input,
    double ds) const;

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string global_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  rclcpp::Logger logger_;

  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_sub_;

  std::vector<geometry_msgs::msg::PoseStamped> lane_points_;
  mutable std::mutex data_mutex_;

  std::string lane_marker_topic_;
  std::string lane_marker_namespace_;

  double resample_distance_;

  // novo parâmetro para escolher qual faixa usar (id do Marker)
  int lane_id_;
};

}  // namespace nav_virtual_lanes_planner
