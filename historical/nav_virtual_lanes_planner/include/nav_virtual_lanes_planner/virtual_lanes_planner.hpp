#ifndef NAV_VIRTUAL_LANES_PLANNER__VIRTUAL_LANES_PLANNER_HPP_
#define NAV_VIRTUAL_LANES_PLANNER__VIRTUAL_LANES_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"

#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav_virtual_lanes_planner
{

class VirtualLanesPlanner : public nav2_core::GlobalPlanner
{
public:
  VirtualLanesPlanner();

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

private:
  void handleMarker(const visualization_msgs::msg::Marker::SharedPtr msg);

  geometry_msgs::msg::PoseStamped transformToGlobalFrame(
    const geometry_msgs::msg::PoseStamped & pose) const;

  std::size_t nearestIndex(
    const geometry_msgs::msg::PoseStamped & ref,
    const std::vector<geometry_msgs::msg::PoseStamped> & pts) const;

  std::vector<geometry_msgs::msg::PoseStamped> resamplePath(
    const std::vector<geometry_msgs::msg::PoseStamped> & input,
    double ds) const;

private:
  rclcpp::Logger logger_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string global_frame_;

  // params
  std::string lane_marker_topic_;
  std::string lane_marker_namespace_;
  double resample_distance_;
  int lane_id_;

  // lane data
  mutable std::mutex data_mutex_;
  std::vector<geometry_msgs::msg::PoseStamped> lane_points_;

  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_sub_;
};

}  // namespace nav_virtual_lanes_planner

#endif  // NAV_VIRTUAL_LANES_PLANNER__VIRTUAL_LANES_PLANNER_HPP_
