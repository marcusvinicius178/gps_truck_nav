// ROS 2 Iron wrapper around the deterministic commissioning guard.
// An independent supervisor must terminate Gazebo if this process hangs/dies.
#include "guard.hpp"

#include <chrono>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <sstream>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "self_test.hpp"
#include "lease_snapshot.hpp"

namespace
{
bool is_finite(double x) {return commissioning::finite(x);}
using Twist = geometry_msgs::msg::Twist;
constexpr const char * output_topic = "/actuation/steering_command";
constexpr const char * input_topic = "/commissioning/cmd_vel";

double monotonic()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string quote(const std::string & input)
{
  std::ostringstream out; out << '"';
  for (unsigned char c : input) {
    if (c == '"' || c == '\\') {out << '\\' << static_cast<char>(c);}
    else if (c < 32) {out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << unsigned(c) << std::dec;}
    else {out << static_cast<char>(c);}
  }
  out << '"'; return out.str();
}

std::string number(double x)
{
  if (!is_finite(x)) {return "null";}
  std::ostringstream out; out << std::setprecision(17) << x; return out.str();
}

template<typename Iterator> std::string gid_hex(Iterator first, Iterator last)
{
  std::ostringstream out;
  bool nonzero = false;
  for (auto i = first; i != last; ++i) {
    nonzero |= *i != 0; out << std::hex << std::setw(2) << std::setfill('0') << unsigned(*i);
  }
  return nonzero ? out.str() : "";
}

template<typename Info> std::string endpoint_gid(const Info & endpoint)
{
  const auto & id = endpoint.endpoint_gid(); return gid_hex(id.begin(), id.end());
}

template<typename Info> std::string node_name(const Info & endpoint)
{
  const auto ns = endpoint.node_namespace();
  return (ns == "/" ? "" : ns) + "/" + endpoint.node_name();
}

double stamp(const builtin_interfaces::msg::Time & time)
{
  if (time.sec < 0 || time.nanosec >= 1000000000) {return std::numeric_limits<double>::quiet_NaN();}
  return static_cast<double>(time.sec) + static_cast<double>(time.nanosec) / 1e9;
}

std::array<double, 6> components(const Twist & message)
{
  return {message.linear.x, message.linear.y, message.linear.z,
    message.angular.x, message.angular.y, message.angular.z};
}

std::string array_json(const std::array<double, 6> & values)
{
  std::string out = "[";
  for (size_t i = 0; i < values.size(); ++i) {out += (i ? "," : "") + number(values[i]);}
  return out + "]";
}

void write_all(int fd, const std::string & content)
{
  size_t offset = 0;
  while (offset < content.size()) {
    const auto count = ::write(fd, content.data() + offset, content.size() - offset);
    if (count < 0 && errno == EINTR) {continue;}
    if (count <= 0) {throw std::runtime_error("Evidence write failed");}
    offset += static_cast<size_t>(count);
  }
}

class Evidence
{
public:
  explicit Evidence(const std::string & directory)
  {
    directory_ = ::open(directory.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW);
    struct stat status{};
    if (directory_ < 0 || ::fstat(directory_, &status) != 0) {
      throw std::runtime_error("Evidence requires an existing real directory");
    }
    uid_ = status.st_uid; gid_ = status.st_gid;
    events_ = create("adapter_events.jsonl");
  }
  ~Evidence() {if (events_ >= 0) {::close(events_);} if (directory_ >= 0) {::close(directory_);}}
  Evidence(const Evidence &) = delete;
  Evidence & operator=(const Evidence &) = delete;
  void event(const std::string & json)
  {
    if (++events_count_ > 100000) {throw std::runtime_error("Evidence event limit reached");}
    write_all(events_, json + "\n");
  }
  void heartbeat(const std::string & json)
  {
    const int fd = create("adapter_heartbeat.tmp");
    try {write_all(fd, json + "\n");}
    catch (...) {::close(fd); throw;}
    if (::close(fd) != 0 || ::renameat(directory_, "adapter_heartbeat.tmp", directory_, "adapter_heartbeat.json") != 0) {
      throw std::runtime_error("Cannot commit adapter heartbeat");
    }
  }
private:
  int directory_ = -1, events_ = -1;
  uid_t uid_{}; gid_t gid_{};
  unsigned events_count_ = 0;
  int create(const char * name)
  {
    const int fd = ::openat(directory_, name, O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW, 0600);
    if (fd < 0) {throw std::runtime_error(std::string("Cannot create fresh evidence: ") + name);}
    struct stat existing{};
    if (::fstat(fd, &existing) != 0 ||
      ((existing.st_uid != uid_ || existing.st_gid != gid_) && ::fchown(fd, uid_, gid_) != 0) ||
      ::fchmod(fd, 0600) != 0) {
      ::close(fd); throw std::runtime_error("Cannot preserve host evidence ownership");
    }
    return fd;
  }
};

double read_lease(const std::string & path)
{
  const int fd = ::open(path.c_str(), O_RDONLY | O_NOFOLLOW | O_NONBLOCK);
  if (fd < 0) {return commissioning::absent;}
  struct stat info{};
  char buffer[129]{};
  const bool regular = ::fstat(fd, &info) == 0 && S_ISREG(info.st_mode) && info.st_size > 0 && info.st_size <= 128;
  const ssize_t n = regular ? ::read(fd, buffer, 128) : -1;
  ::close(fd);
  if (n <= 0 || n != info.st_size) {return commissioning::absent;}
  char * end = nullptr; errno = 0;
  const double value = std::strtod(buffer, &end);
  if (end == buffer || errno != 0 || !is_finite(value)) {return commissioning::absent;}
  while (*end && std::isspace(static_cast<unsigned char>(*end))) {++end;}
  return *end == 0 ? value : commissioning::absent;
}

bool identifier(const std::string & id, size_t size)
{
  return id.size() == size && std::all_of(id.begin(), id.end(), [](char c) {
    return ('a' <= c && c <= 'f') || ('0' <= c && c <= '9');});
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc == 2 && std::string(argv[1]) == "--offline-self-test") {
    const bool passed = commissioning::self_test();
    std::cout << "{\"status\":\"" << (passed ? "OFFLINE_CORE_PASS" : "OFFLINE_CORE_FAILED") <<
      "\",\"ros_initialized\":false,\"publishers_created\":0,\"dynamic_validated\":false}\n";
    return passed ? 0 : 2;
  }
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<Twist>::SharedPtr publisher;
  std::unique_ptr<Evidence> evidence;
  std::unique_ptr<commissioning::Guard> guard;
  std::string run_id, model_sha, lease_path;
  bool execute = false;
  unsigned long heartbeat_sequence = 0;
  auto heartbeat = [&]() {
      if (!evidence || !guard) {return;}
      const auto snapshot = commissioning::lease_snapshot([&]() {return read_lease(lease_path);}, monotonic);
      const double now = snapshot.now;
      ++heartbeat_sequence;
      evidence->heartbeat("{\"schema_version\":1,\"run_id\":" + quote(run_id) +
        ",\"seq\":" + std::to_string(heartbeat_sequence) + ",\"sequence\":" + std::to_string(heartbeat_sequence) +
        ",\"monotonic_s\":" + number(now) + ",\"monotonic_wall_s\":" + number(now) +
        ",\"state\":" + quote(guard->state) + ",\"fault\":" + quote(guard->fault_reason) +
        ",\"waiting_reason\":" + quote(guard->waiting_reason) +
        ",\"observations_ready\":" + (guard->observations_ready(now, snapshot.lease) ? "true" : "false") +
        ",\"standstill\":" + (guard->standstill_observed(now) ? "true" : "false") +
        ",\"publisher_created\":" + (publisher ? "true" : "false") +
        ",\"command_count\":" + std::to_string(guard->command_count) +
        ",\"rejected_count\":" + std::to_string(guard->rejected_count) +
        ",\"input_gid\":" + quote(guard->input_gid) + ",\"output_gid\":" + quote(guard->output_gid) +
        ",\"subscriber_gid\":" + quote(guard->subscriber_gid) +
        ",\"clock_s\":" + number(guard->clock.stamp) + ",\"odom_stamp_s\":" + number(guard->odom.stamp) +
        ",\"joint_stamp_s\":" + number(guard->joints.stamp) +
        ",\"observed_speed_mps\":" + number(guard->speed) +
        ",\"observed_rear_lateral_speed_mps\":" + number(guard->lateral_speed) +
        ",\"observed_yaw_rate_rps\":" + number(guard->yaw_rate) +
        ",\"left_angle_rad\":" + number(guard->left) + ",\"right_angle_rad\":" + number(guard->right) +
        ",\"travel_m\":" + number(guard->travel) + ",\"displacement_m\":" + number(guard->displacement) +
        ",\"model_sha256\":" + quote(model_sha) +
        ",\"dynamic_validated\":false,\"physical_stop_verified\":false,\"mission_ready\":false}");
    };
  try {
    node = rclcpp::Node::make_shared("phase_a_ackermann_guard");
    const auto mode = node->declare_parameter<std::string>("mode", "prepare");
    if (mode != "prepare" && mode != "execute") {throw std::runtime_error("Invalid mode");}
    execute = mode == "execute";
    const auto directory = node->declare_parameter<std::string>("evidence_dir", "");
    lease_path = node->declare_parameter<std::string>("supervisor_lease", "");
    run_id = node->declare_parameter<std::string>("run_id", "");
    model_sha = node->declare_parameter<std::string>("model_sha256", "");
    const auto authorization = node->declare_parameter<std::string>("authorization_id", "");
    const double duration = node->declare_parameter<double>("duration_wall_s", 120.0);
    if (!identifier(run_id, 32) || !identifier(model_sha, 64) ||
      !std::filesystem::path(directory).is_absolute() || !std::filesystem::path(lease_path).is_absolute() ||
      (execute && authorization.empty())) {throw std::runtime_error("Explicit identity, paths and execution authorization required");}
    auto required = [&](const char * name) {
        return node->declare_parameter<double>(name, std::numeric_limits<double>::quiet_NaN());
      };
    commissioning::Geometry geometry{
      required("wheelbase_m"), required("track_m"), required("rear_right_y_m"),
      required("plugin_max_speed_mps"), required("plugin_max_steer_rad"),
      required("left_lower_rad"), required("left_upper_rad"),
      required("right_lower_rad"), required("right_upper_rad")};
    const double rear_x_offset = required("base_to_rear_x_m");
    if (!is_finite(rear_x_offset) || rear_x_offset >= 0) {throw std::runtime_error("Explicit rear-axle frame offset required");}
    guard = std::make_unique<commissioning::Guard>(geometry, execute, monotonic(), duration);
    evidence = std::make_unique<Evidence>(directory);
    evidence->event("{\"event\":\"CONFIGURATION\",\"mode\":" + quote(mode) + ",\"run_id\":" + quote(run_id) +
      ",\"authorization_id\":" + quote(authorization) + ",\"model_sha256\":" + quote(model_sha) +
      ",\"input_topic\":" + quote(input_topic) + ",\"output_topic\":" + quote(output_topic) +
      ",\"input_reference\":\"rear_axle_nav\",\"limit_policy\":\"REJECT_NO_CLIPPING\","
      "\"producer_timestamp_verified\":false,\"receipt_clock\":\"CLOCK_MONOTONIC\","
      "\"geometry_consumed\":{\"wheelbase_m\":" + number(geometry.wheelbase) +
      ",\"track_m\":" + number(geometry.track) + ",\"rear_right_y_m\":" + number(geometry.rear_right_y) +
      ",\"plugin_max_speed_mps\":" + number(geometry.plugin_speed) +
      ",\"plugin_max_steer_rad\":" + number(geometry.plugin_steer) +
      ",\"left_lower_rad\":" + number(geometry.left_lower) + ",\"left_upper_rad\":" + number(geometry.left_upper) +
      ",\"right_lower_rad\":" + number(geometry.right_lower) + ",\"right_upper_rad\":" + number(geometry.right_upper) +
      ",\"base_to_rear_x_m\":" + number(rear_x_offset) +
      "},\"commissioning_limits\":{\"speed_mps\":0.5,\"yaw_rate_rps\":0.02,\"joint_target_rad\":0.20,"
      "\"observed_speed_mps\":0.7,\"observed_yaw_rate_rps\":0.05,\"observed_joint_rad\":0.25,"
      "\"command_timeout_s\":0.25,\"observation_timeout_s\":0.5,\"lease_timeout_s\":0.5,"
      "\"clock_stall_s\":1.0,\"clock_forward_jump_s\":0.25,\"sensor_future_window_s\":0.1,"
      "\"standstill_dwell_wall_s\":1.0,\"standstill_stamp_progress_s\":0.5,"
      "\"max_travel_m\":10.0,\"max_displacement_m\":10.0}}");
    if (execute) {
      publisher = node->create_publisher<Twist>(output_topic, rclcpp::QoS(1).reliable().durability_volatile());
      if (std::string(publisher->get_topic_name()) != output_topic) {throw std::runtime_error("Actuator remapping forbidden");}
    }
    auto output = [&]() {
        Twist message; message.linear.x = guard->last.plugin_speed; message.angular.z = guard->last.plugin_angle;
        publisher->publish(message);
      };
    auto input = node->create_subscription<Twist>(input_topic, rclcpp::QoS(1).reliable().durability_volatile(),
      [&](Twist::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
        const auto & raw = info.get_rmw_message_info().publisher_gid;
        const auto gid = gid_hex(std::begin(raw.data), std::end(raw.data));
        const auto values = components(*message);
        const auto snapshot = commissioning::lease_snapshot([&]() {return read_lease(lease_path);}, monotonic);
        const auto now = snapshot.now;
        const bool accepted = guard->command(values, gid, now, snapshot.lease);
        if (accepted && publisher) {output();}
        evidence->event("{\"event\":\"COMMAND\",\"monotonic_s\":" + number(now) +
          ",\"sim_s\":" + number(guard->clock.stamp) + ",\"source_gid\":" + quote(gid) +
          ",\"source_timestamp_raw\":" + std::to_string(info.get_rmw_message_info().source_timestamp) +
          ",\"requested_xyz_rpy\":" + array_json(values) + ",\"published\":" + (accepted ? "true" : "false") +
          ",\"state\":" + quote(guard->state) + ",\"status\":" + quote(guard->last.status) +
          ",\"reason\":" + quote(guard->state == "FAULT" ? guard->fault_reason : guard->last.reason) +
          ",\"rear_speed_mps\":" + number(guard->last.rear_speed) +
          ",\"yaw_rate_rps\":" + number(guard->last.yaw_rate) +
          ",\"plugin_linear_x_mps\":" + number(guard->last.plugin_speed) +
          ",\"plugin_angular_z_rad\":" + number(guard->last.plugin_angle) +
          ",\"left_target_rad\":" + number(guard->last.left_target) +
          ",\"right_target_rad\":" + number(guard->last.right_target) +
          ",\"curvature_inv_m\":" + number(guard->last.curvature) + "}");
      });
    if (std::string(input->get_topic_name()) != input_topic) {throw std::runtime_error("Input remapping forbidden");}
    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(10);
    auto clock = node->create_subscription<rosgraph_msgs::msg::Clock>("/clock", sensor_qos,
      [&](rosgraph_msgs::msg::Clock::ConstSharedPtr message) {
        guard->observe_clock(stamp(message->clock), monotonic());
      });
    auto odom = node->create_subscription<nav_msgs::msg::Odometry>("/odom", sensor_qos,
      [&](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        const auto & p = message->pose.pose.position;
        const auto & q = message->pose.pose.orientation;
        const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        if (message->header.frame_id != "odom" || message->child_frame_id != "base_footprint" ||
          !is_finite(norm) || std::abs(norm - 1) > 0.01 || !is_finite(p.z) ||
          !is_finite(message->twist.twist.linear.y) || !is_finite(message->twist.twist.linear.z) ||
          !is_finite(message->twist.twist.angular.x) || !is_finite(message->twist.twist.angular.y)) {
          guard->fault("ODOMETRY_FRAME_OR_POSE_INVALID"); return;
        }
        const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        if (std::hypot(message->twist.twist.linear.x, message->twist.twist.linear.y) >
          guard->limits.observed_speed) {guard->fault("OBSERVED_PLANAR_SPEED_ENVELOPE_EXCEEDED"); return;}
        guard->observe_odom(stamp(message->header.stamp), monotonic(), message->twist.twist.linear.x,
          message->twist.twist.angular.z, p.x + rear_x_offset * std::cos(yaw), p.y + rear_x_offset * std::sin(yaw),
          message->twist.twist.linear.y + message->twist.twist.angular.z * rear_x_offset);
      });
    auto joints = node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", sensor_qos,
      [&](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        if (message->name.size() != message->position.size()) {guard->fault("JOINT_STATE_SHAPE_INVALID"); return;}
        std::map<std::string, double> positions;
        for (size_t i = 0; i < message->name.size(); ++i) {
          if (!is_finite(message->position[i]) || !positions.emplace(message->name[i], message->position[i]).second) {
            guard->fault("JOINT_STATE_VALUE_OR_DUPLICATE_INVALID"); return;
          }
        }
        for (const char * name : {"front_left_steer_joint", "front_right_steer_joint", "first_wheel_left_joint",
            "first_wheel_right_joint", "fourth_wheel_left_joint", "fourth_wheel_right_joint"}) {
          if (!positions.count(name)) {guard->fault("REQUIRED_JOINT_MISSING"); return;}
        }
        guard->observe_joints(stamp(message->header.stamp), monotonic(),
          positions.at("front_left_steer_joint"), positions.at("front_right_steer_joint"));
      });
    std::vector<rclcpp::Subscription<Twist>::SharedPtr> quarantined;
    for (const char * topic : {"/actuation/nav2_twist", "/actuation/behavior_quarantine", "/cmd_vel", "/cmd_vel_nav"}) {
      quarantined.push_back(node->create_subscription<Twist>(topic, sensor_qos,
        [&, topic](Twist::ConstSharedPtr message) {
          const auto values = components(*message);
          if (std::any_of(values.begin(), values.end(), [](double x) {return !is_finite(x) || x != 0;})) {
            guard->fault(std::string("UNEXPECTED_NAVIGATION_COMMAND:") + topic);
          }
        }));
    }
    double next_graph = 0, next_heartbeat = 0;
    std::string prior_graph, prior_state;
    heartbeat();
    while (rclcpp::ok() && guard->state != "FAULT") {
      rclcpp::spin_some(node);
      const double now = monotonic();
      if (now >= next_graph) {
        const auto inputs = node->get_publishers_info_by_topic(input_topic);
        const auto outputs = node->get_publishers_info_by_topic(output_topic);
        const auto subscribers = node->get_subscriptions_info_by_topic(output_topic);
        const size_t expected_inputs = execute ? 1 : 0, expected_outputs = execute ? 1 : 0;
        bool transport_valid = outputs.size() == expected_outputs && subscribers.size() == 1;
        std::string source, actuator, destination, reason = "COMMAND_GRAPH_COUNT_OR_IDENTITY";
        if (subscribers.size() == 1) {
          destination = endpoint_gid(subscribers[0]);
          transport_valid &= node_name(subscribers[0]) == "/ackermann_drive" &&
            subscribers[0].topic_type() == "geometry_msgs/msg/Twist" && !destination.empty();
        }
        bool source_valid = inputs.size() == expected_inputs;
        if (execute && inputs.size() == 1) {
          source = endpoint_gid(inputs[0]);
          source_valid &= node_name(inputs[0]) == "/phase_a_commissioning_source" &&
            inputs[0].topic_type() == "geometry_msgs/msg/Twist" && !source.empty();
        }
        if (execute && outputs.size() == 1) {
          actuator = endpoint_gid(outputs[0]);
          const auto raw = publisher->get_gid();
          transport_valid &= node_name(outputs[0]) == "/phase_a_ackermann_guard" &&
            outputs[0].topic_type() == "geometry_msgs/msg/Twist" &&
            actuator == gid_hex(std::begin(raw.data), std::end(raw.data)) && !actuator.empty();
        }
        const bool valid = source_valid && transport_valid;
        if (inputs.size() > expected_inputs || outputs.size() > expected_outputs || subscribers.size() > 1 ||
          (inputs.size() == 1 && !source_valid) ||
          (subscribers.size() == 1 && (node_name(subscribers[0]) != "/ackermann_drive" ||
          subscribers[0].topic_type() != "geometry_msgs/msg/Twist")) ||
          (outputs.size() == 1 && execute && !transport_valid && subscribers.size() == 1)) {
          guard->fault("UNEXPECTED_OR_MULTIPLE_COMMAND_ENDPOINTS");
        }
        guard->observe_graph(now, valid, source, actuator, destination, reason,
          transport_valid && execute && inputs.empty());
        const auto graph_record = "{\"event\":\"GRAPH\",\"valid\":" + std::string(valid ? "true" : "false") +
          ",\"input_publishers\":" + std::to_string(inputs.size()) + ",\"output_publishers\":" + std::to_string(outputs.size()) +
          ",\"output_subscribers\":" + std::to_string(subscribers.size()) + ",\"input_gid\":" + quote(source) +
          ",\"output_gid\":" + quote(actuator) + ",\"subscriber_gid\":" + quote(destination) + "}";
        if (graph_record != prior_graph) {evidence->event(graph_record); prior_graph = graph_record;}
        next_graph = now + 0.1;
      }
      const auto snapshot = commissioning::lease_snapshot([&]() {return read_lease(lease_path);}, monotonic);
      guard->poll(snapshot.now, snapshot.lease);
      if (guard->state != prior_state) {
        evidence->event("{\"event\":\"STATE\",\"monotonic_s\":" + number(now) +
          ",\"state\":" + quote(guard->state) + ",\"reason\":" + quote(guard->fault_reason) + "}");
        prior_state = guard->state;
      }
      if (now >= next_heartbeat || guard->state == "FAULT") {heartbeat(); next_heartbeat = now + 0.05;}
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (guard->state != "FAULT") {guard->fault("ROS_CONTEXT_STOPPED");}
    // Best-effort zero cannot be relied on after process death; supervisor kill is mandatory.
    if (publisher && rclcpp::ok()) {publisher->publish(Twist{});}
    heartbeat();
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return 4;
  } catch (const std::exception & error) {
    std::cerr << "Commissioning adapter failed: " << error.what() << '\n';
    if (guard) {guard->fault(std::string("ADAPTER_EXCEPTION:") + error.what());}
    try {if (publisher && rclcpp::ok()) {publisher->publish(Twist{});} heartbeat();} catch (...) {}
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return 3;
  }
}
