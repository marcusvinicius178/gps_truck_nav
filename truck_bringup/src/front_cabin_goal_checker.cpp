#include "nav2_core/goal_checker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "angles/angles.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class FrontCabinGoalChecker : public nav2_core::GoalChecker
{
public:
    void initialize(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
      const std::string& plugin_name,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /* costmap_ros */) override
    {
        auto node = parent.lock();
        if (!node) {
            throw std::runtime_error("Failed to lock weak_ptr during goal checker initialization!");
        }

        node->get_parameter_or(plugin_name + ".xy_goal_tolerance", xy_goal_tolerance_, 1.5);
        node->get_parameter_or(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_, 0.25);
        node->get_parameter_or(plugin_name + ".front_offset", front_offset_, 5.0);

        RCLCPP_INFO(node->get_logger(), "Loaded xy_goal_tolerance: %f", xy_goal_tolerance_);
        RCLCPP_INFO(node->get_logger(), "Loaded yaw_goal_tolerance: %f", yaw_goal_tolerance_);
        RCLCPP_INFO(node->get_logger(), "Loaded front_offset: %f", front_offset_);
    }

    void reset() override
    {
        // Reset any internal state if necessary
    }

    bool isGoalReached(
      const geometry_msgs::msg::Pose& query_pose,
      const geometry_msgs::msg::Pose& goal_pose,
      const geometry_msgs::msg::Twist& /* velocity */) override
    {
        double dx = query_pose.position.x + front_offset_ * cos(tf2::getYaw(query_pose.orientation)) - goal_pose.position.x;
        double dy = query_pose.position.y + front_offset_ * sin(tf2::getYaw(query_pose.orientation)) - goal_pose.position.y;
        double distance = sqrt(dx * dx + dy * dy);

        double dyaw = angles::shortest_angular_distance(
          tf2::getYaw(query_pose.orientation),
          tf2::getYaw(goal_pose.orientation));

        return distance <= xy_goal_tolerance_ && fabs(dyaw) <= yaw_goal_tolerance_;
    }

    bool getTolerances(geometry_msgs::msg::Pose& pose, geometry_msgs::msg::Twist& twist) override
    {
        pose.position.x = xy_goal_tolerance_;
        pose.position.y = xy_goal_tolerance_;
        pose.position.z = 0.0; // Not used
        pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw_goal_tolerance_));

        twist.linear.x = 0.0; // Not used
        twist.angular.z = 0.0; // Not used
        return true;
    }

private:
    double front_offset_;
    double xy_goal_tolerance_;
    double yaw_goal_tolerance_;
};

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(FrontCabinGoalChecker, nav2_core::GoalChecker)
