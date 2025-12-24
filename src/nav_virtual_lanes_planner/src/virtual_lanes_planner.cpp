#include "nav_virtual_lanes_planner/virtual_lanes_planner.hpp"

#include <cmath>
#include <limits>

#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace nav_virtual_lanes_planner
{

namespace
{

double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::msg::Quaternion yawToQuat(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion q_msg;
  q_msg.x = q.x();
  q_msg.y = q.y();
  q_msg.z = q.z();
  q_msg.w = q.w();
  return q_msg;
}

inline bool isZeroStamp(const builtin_interfaces::msg::Time & t)
{
  return (t.sec == 0 && t.nanosec == 0u);
}

}  // namespace

VirtualLanesPlanner::VirtualLanesPlanner()
: logger_(rclcpp::get_logger("VirtualLanesPlanner")),
  resample_distance_(0.25),   // bem mais denso por default
  lane_id_(1)
{
}

void VirtualLanesPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  node_ = parent_.lock();
  if (!node_) {
    throw std::runtime_error("VirtualLanesPlanner: Lifecycle node is expired");
  }

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  global_frame_ = costmap_ros_->getGlobalFrameID();
  logger_ = node_->get_logger();

  RCLCPP_INFO(logger_, "VirtualLanesPlanner: configuring. global_frame='%s'", global_frame_.c_str());

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".lane_marker_topic",
    rclcpp::ParameterValue(std::string("visualization_marker")));

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".lane_marker_namespace",
    rclcpp::ParameterValue(std::string("lanes")));

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".resample_distance",
    rclcpp::ParameterValue(0.25));

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".lane_id",
    rclcpp::ParameterValue(1));

  node_->get_parameter(name_ + ".lane_marker_topic", lane_marker_topic_);
  node_->get_parameter(name_ + ".lane_marker_namespace", lane_marker_namespace_);
  node_->get_parameter(name_ + ".resample_distance", resample_distance_);
  node_->get_parameter(name_ + ".lane_id", lane_id_);

  RCLCPP_INFO(
    logger_,
    "VirtualLanesPlanner: topic='%s', ns='%s', lane_id=%d, resample_distance=%.3f",
    lane_marker_topic_.c_str(), lane_marker_namespace_.c_str(),
    lane_id_, resample_distance_);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  marker_sub_ = node_->create_subscription<visualization_msgs::msg::Marker>(
    lane_marker_topic_,
    qos,
    std::bind(&VirtualLanesPlanner::handleMarker, this, std::placeholders::_1));
}

void VirtualLanesPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "VirtualLanesPlanner: cleanup");
  marker_sub_.reset();
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    lane_points_.clear();
  }
}

void VirtualLanesPlanner::activate()
{
  RCLCPP_INFO(logger_, "VirtualLanesPlanner: activate");
}

void VirtualLanesPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "VirtualLanesPlanner: deactivate");
}

geometry_msgs::msg::PoseStamped VirtualLanesPlanner::transformToGlobalFrame(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  if (pose.header.frame_id.empty() || pose.header.frame_id == global_frame_) {
    return pose;
  }

  geometry_msgs::msg::PoseStamped out;
  try {
    // Se stamp == 0, TF usa o transform mais recente.
    tf_->transform(pose, out, global_frame_, tf2::durationFromSec(0.5));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger_, "VirtualLanesPlanner: TF error '%s'->'%s': %s",
      pose.header.frame_id.c_str(), global_frame_.c_str(), ex.what());
    out = pose;
    out.header.frame_id = global_frame_;
  }
  return out;
}

void VirtualLanesPlanner::handleMarker(const visualization_msgs::msg::Marker::SharedPtr msg)
{
  if (!lane_marker_namespace_.empty() && msg->ns != lane_marker_namespace_) {
    return;
  }
  if (msg->id != lane_id_) {
    return;
  }
  if (msg->type != visualization_msgs::msg::Marker::LINE_STRIP) {
    return;
  }
  if (msg->points.size() < 2) {
    return;
  }

  std::vector<geometry_msgs::msg::PoseStamped> converted;
  converted.reserve(msg->points.size());

  geometry_msgs::msg::PoseStamped base_pose;
  base_pose.header.frame_id = msg->header.frame_id;

  // Ponto chave: usar stamp=0 evita mismatch wall-time vs sim-time
  // e evita "Transform data too old" quando TF está em /clock.
  base_pose.header.stamp = rclcpp::Time(0);

  base_pose.pose = msg->pose;

  for (const auto & p : msg->points) {
    geometry_msgs::msg::PoseStamped lane_pose = base_pose;
    lane_pose.pose.position.x = p.x;
    lane_pose.pose.position.y = p.y;
    lane_pose.pose.position.z = p.z;

    converted.push_back(transformToGlobalFrame(lane_pose));
  }

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    lane_points_ = std::move(converted);
  }

  RCLCPP_INFO(
    logger_,
    "VirtualLanesPlanner: updated lane id=%d with %zu points (frame='%s', ns='%s')",
    lane_id_, lane_points_.size(), msg->header.frame_id.c_str(), msg->ns.c_str());
}

std::size_t VirtualLanesPlanner::nearestIndex(
  const geometry_msgs::msg::PoseStamped & ref,
  const std::vector<geometry_msgs::msg::PoseStamped> & pts) const
{
  if (pts.empty()) {
    return 0;
  }

  std::size_t best_idx = 0;
  double best_dist = std::numeric_limits<double>::max();

  const double rx = ref.pose.position.x;
  const double ry = ref.pose.position.y;

  for (std::size_t i = 0; i < pts.size(); ++i) {
    const double dx = pts[i].pose.position.x - rx;
    const double dy = pts[i].pose.position.y - ry;
    const double d2 = dx * dx + dy * dy;

    if (d2 < best_dist) {
      best_dist = d2;
      best_idx = i;
    }
  }

  return best_idx;
}

std::vector<geometry_msgs::msg::PoseStamped> VirtualLanesPlanner::resamplePath(
  const std::vector<geometry_msgs::msg::PoseStamped> & input,
  double ds) const
{
  if (input.size() < 2 || ds <= 0.0) {
    return input;
  }

  const std::size_t n = input.size();
  std::vector<double> s(n, 0.0);

  for (std::size_t i = 1; i < n; ++i) {
    const auto & p0 = input[i - 1].pose.position;
    const auto & p1 = input[i].pose.position;
    const double dx = p1.x - p0.x;
    const double dy = p1.y - p0.y;
    s[i] = s[i - 1] + std::sqrt(dx * dx + dy * dy);
  }

  const double total_length = s.back();
  if (total_length < ds) {
    return input;
  }

  std::vector<geometry_msgs::msg::PoseStamped> output;
  output.reserve(static_cast<std::size_t>(total_length / ds) + 3);

  output.push_back(input.front());

  double d = ds;
  while (d < total_length) {
    std::size_t i = 0;
    while (i + 1 < n && s[i + 1] < d) {
      ++i;
    }
    if (i + 1 >= n) {
      break;
    }

    const double seg_len = s[i + 1] - s[i];
    if (seg_len <= 1e-9) {
      d += ds;
      continue;
    }

    const double t = (d - s[i]) / seg_len;

    const auto & p0 = input[i].pose.position;
    const auto & p1 = input[i + 1].pose.position;

    geometry_msgs::msg::PoseStamped p;
    p.header = input[i].header;  // mantém frame, e stamp=0 se você já setou antes

    p.pose.position.x = p0.x + t * (p1.x - p0.x);
    p.pose.position.y = p0.y + t * (p1.y - p0.y);
    p.pose.position.z = p0.z + t * (p1.z - p0.z);

    const double yaw0 = yawFromQuat(input[i].pose.orientation);
    const double yaw1 = yawFromQuat(input[i + 1].pose.orientation);

    double dyaw = yaw1 - yaw0;
    while (dyaw > M_PI)  dyaw -= 2.0 * M_PI;
    while (dyaw < -M_PI) dyaw += 2.0 * M_PI;

    const double yaw_interp = yaw0 + t * dyaw;
    p.pose.orientation = yawToQuat(yaw_interp);

    output.push_back(p);
    d += ds;
  }

  output.push_back(input.back());
  return output;
}

nav_msgs::msg::Path VirtualLanesPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;

  // Mesma lógica: stamp=0 para evitar time-domain mismatch
  path.header.stamp = rclcpp::Time(0);

  auto start_in = start;
  auto goal_in = goal;

  // Zera os stamps se vierem com wall-time e seu TF estiver em sim-time.
  // Isso força TF a usar "latest".
  start_in.header.stamp = rclcpp::Time(0);
  goal_in.header.stamp  = rclcpp::Time(0);

  auto start_global = transformToGlobalFrame(start_in);
  auto goal_global  = transformToGlobalFrame(goal_in);

  std::vector<geometry_msgs::msg::PoseStamped> lane_copy;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    lane_copy = lane_points_;
  }

  if (lane_copy.empty()) {
    RCLCPP_WARN(logger_, "VirtualLanesPlanner: no lane points, using straight line");
    start_global.header.frame_id = global_frame_;
    goal_global.header.frame_id = global_frame_;
    start_global.header.stamp = rclcpp::Time(0);
    goal_global.header.stamp = rclcpp::Time(0);
    path.poses = {start_global, goal_global};
    return path;
  }

  std::size_t start_idx = nearestIndex(start_global, lane_copy);
  std::size_t goal_idx  = nearestIndex(goal_global, lane_copy);

  RCLCPP_INFO(
    logger_,
    "VirtualLanesPlanner: start_idx=%zu, goal_idx=%zu, lane_size=%zu",
    start_idx, goal_idx, lane_copy.size());

  std::vector<geometry_msgs::msg::PoseStamped> segment;
  if (start_idx <= goal_idx) {
    segment.reserve(goal_idx - start_idx + 1);
    for (std::size_t i = start_idx; i <= goal_idx; ++i) {
      segment.push_back(lane_copy[i]);
    }
  } else {
    segment.reserve(start_idx - goal_idx + 1);
    for (std::size_t i = start_idx;; --i) {
      segment.push_back(lane_copy[i]);
      if (i == goal_idx) {
        break;
      }
    }
  }

  if (segment.size() < 2) {
    start_global.header.stamp = rclcpp::Time(0);
    goal_global.header.stamp  = rclcpp::Time(0);
    path.poses = {start_global, goal_global};
    return path;
  }

  // Densifica
  auto dense_segment = resamplePath(segment, resample_distance_);
  path.poses = std::move(dense_segment);

  // Fix start/end e zera stamp
  if (!path.poses.empty()) {
    path.poses.front() = start_global;
    path.poses.back()  = goal_global;
  }

  // Garante frame e stamp coerentes
  for (auto & ps : path.poses) {
    ps.header.frame_id = global_frame_;
    ps.header.stamp = rclcpp::Time(0);
  }

  // Ajusta yaw seguindo a direção do caminho
  auto & poses = path.poses;
  if (poses.size() >= 2) {
    for (std::size_t i = 0; i + 1 < poses.size(); ++i) {
      const auto & p0 = poses[i].pose.position;
      const auto & p1 = poses[i + 1].pose.position;
      const double yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
      poses[i].pose.orientation = yawToQuat(yaw);
    }
    poses.back().pose.orientation = poses[poses.size() - 2].pose.orientation;
  }

  return path;
}

}  // namespace nav_virtual_lanes_planner

// Export tem que estar fora do namespace
PLUGINLIB_EXPORT_CLASS(nav_virtual_lanes_planner::VirtualLanesPlanner, nav2_core::GlobalPlanner)
