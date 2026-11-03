// Read-only, bounded TF endpoint observer for the isolated Phase A container.
// Iron rclcpp 21.0.8 exposes publisher_gid via MessageInfo; rclpy does not.
// It sends no goals, transform broadcasts, service requests or velocity commands.
#include <cerrno>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "observations.hpp"

namespace
{
double unix_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

void write_exclusive(const std::string & path, const std::string & contents)
{
  const int descriptor = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0644);
  if (descriptor < 0) {
    throw std::runtime_error("Cannot create new evidence file: " + std::string(std::strerror(errno)));
  }
  size_t offset = 0;
  int failure = 0;
  while (offset < contents.size()) {
    const auto written = ::write(descriptor, contents.data() + offset, contents.size() - offset);
    if (written < 0 && errno == EINTR) {continue;}
    if (written <= 0) {failure = written < 0 ? errno : EIO; break;}
    offset += static_cast<size_t>(written);
  }
  if (!failure && ::fsync(descriptor) != 0) {failure = errno;}
  if (::close(descriptor) != 0 && !failure) {failure = errno;}
  if (failure) {
    throw std::runtime_error("Evidence write incomplete: " + std::string(std::strerror(failure)));
  }
}

void discover(const rclcpp::Node::SharedPtr & node, phase_a_tf::Observations & observations)
{
  for (const std::string topic : {"/tf", "/tf_static", "/clock"}) {
    try {
      for (const auto & publisher : node->get_publishers_info_by_topic(topic)) {
        const auto & gid = publisher.endpoint_gid();
        const auto qos = publisher.qos_profile().get_rmw_qos_profile();
        // Keep exact RMW enum values and depth; UNKNOWN is not silently replaced.
        const std::string offered_qos = "{\"reliability\":" + std::to_string(qos.reliability) +
          ",\"durability\":" + std::to_string(qos.durability) +
          ",\"history\":" + std::to_string(qos.history) + ",\"depth\":" +
          std::to_string(qos.depth) + "}";
        observations.endpoint(
          topic, phase_a_tf::gid_hex(gid.begin(), gid.end()),
          publisher.node_name(), publisher.node_namespace(), offered_qos);
      }
    } catch (const std::exception & error) {
      observations.discovery_error(topic + ": " + error.what());
    }
  }
}

rclcpp::SubscriptionOptions loss_options(
  const std::string & topic, phase_a_tf::Observations & observations)
{
  rclcpp::SubscriptionOptions options;
  options.event_callbacks.message_lost_callback =
    [&observations, topic](rclcpp::QOSMessageLostInfo & info) {
      observations.loss_event(topic, info.total_count, info.total_count_change);
    };
  return options;
}
}  // namespace

int main(int argc, char ** argv)
{
  std::string output_path;
  double requested_seconds = 30.0;
  bool duration_seen = false;
  try {
    for (int index = 1; index < argc; ++index) {
      const std::string argument = argv[index];
      if (argument == "--help" && argc == 2) {
        std::cout << "Usage: tf_authority_probe --output NEW_ABSOLUTE_JSON_PATH [--duration SECONDS]\n"
                  << "Duration must be between 1 and 120 wall seconds. Observations only; no goals.\n";
        return 0;
      }
      if (argument == "--output" && output_path.empty() && index + 1 < argc) {
        output_path = argv[++index];
      } else if (argument == "--duration" && !duration_seen && index + 1 < argc) {
        const std::string value = argv[++index];
        size_t consumed = 0;
        requested_seconds = std::stod(value, &consumed);
        duration_seen = true;
        if (consumed != value.size()) {throw std::runtime_error("Invalid duration");}
      } else {
        throw std::runtime_error("Unknown, incomplete or repeated argument: " + argument);
      }
    }
    const std::filesystem::path output(output_path);
    if (!output.is_absolute() || !std::filesystem::is_directory(output.parent_path())) {
      throw std::runtime_error("--output requires an absolute path in an existing directory");
    }
    if (std::filesystem::exists(output) || std::filesystem::is_symlink(output)) {
      throw std::runtime_error("Evidence path already exists; preserve it and choose a new output");
    }
    if (!std::isfinite(requested_seconds) || requested_seconds < 1.0 || requested_seconds > 120.0) {
      throw std::runtime_error("Duration must be between 1 and 120 wall seconds");
    }
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 2;
  }

  phase_a_tf::Observations observations;
  bool complete = false;
  std::string observation_error;
  auto started = std::chrono::steady_clock::now();
  double started_unix = unix_seconds();
  try {
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions()
      .use_global_arguments(false)
      .use_intra_process_comms(false)
      .enable_rosout(false)
      .enable_topic_statistics(false)
      .enable_logger_service(false)
      .start_parameter_services(false)
      .start_parameter_event_publisher(false)
      .append_parameter_override("start_type_description_service", false);
    auto node = std::make_shared<rclcpp::Node>(
      "nav2_tf_authority_probe_" + std::to_string(::getpid()), options);
    const auto wall_offset = [&started]() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
      };
    std::vector<rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr> subscriptions;
    for (const std::string topic : {"/tf", "/tf_static"}) {
      auto qos = rclcpp::QoS(rclcpp::KeepLast(100));
      if (topic == "/tf_static") {
        qos.reliable().transient_local();
      } else {
        // Compatible with both BEST_EFFORT and RELIABLE dynamic TF writers.
        qos.best_effort().durability_volatile();
      }
      const auto callback = [&observations, topic, &wall_offset](
          tf2_msgs::msg::TFMessage::ConstSharedPtr message, const rclcpp::MessageInfo & info)
        {
          const double received_wall = wall_offset();
          const auto & rmw_info = info.get_rmw_message_info();
          const auto & gid = rmw_info.publisher_gid;
          const auto identity = phase_a_tf::gid_hex(std::begin(gid.data), std::end(gid.data));
          for (const auto & transform : message->transforms) {
            const auto & translation = transform.transform.translation;
            const auto & rotation = transform.transform.rotation;
            observations.observe(
              topic, transform.header.frame_id, transform.child_frame_id, identity,
              {transform.header.stamp.sec, transform.header.stamp.nanosec},
              {translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z, rotation.w},
              received_wall, rmw_info.source_timestamp, rmw_info.received_timestamp);
          }
        };
      try {
        subscriptions.push_back(node->create_subscription<tf2_msgs::msg::TFMessage>(
          topic, qos, callback, loss_options(topic, observations)));
        observations.loss_event_support(topic, true);
      } catch (const rclcpp::UnsupportedEventTypeException & error) {
        observations.loss_event_support(topic, false, error.what());
        subscriptions.push_back(node->create_subscription<tf2_msgs::msg::TFMessage>(topic, qos, callback));
      }
    }
    const auto clock_callback = [&observations, &wall_offset](rosgraph_msgs::msg::Clock::ConstSharedPtr message) {
        observations.observe_clock({message->clock.sec, message->clock.nanosec}, wall_offset());
      };
    auto clock_qos = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort().durability_volatile();
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription;
    try {
      clock_subscription = node->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", clock_qos, clock_callback, loss_options("/clock", observations));
      observations.loss_event_support("/clock", true);
    } catch (const rclcpp::UnsupportedEventTypeException & error) {
      observations.loss_event_support("/clock", false, error.what());
      clock_subscription = node->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", clock_qos, clock_callback);
    }
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    started = std::chrono::steady_clock::now();
    started_unix = unix_seconds();
    const auto deadline = started + std::chrono::duration<double>(requested_seconds);
    auto next_discovery = started;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      // spin_some plus an unconditional 5 ms sleep throttled each ready
      // subscription to roughly 200 takes/s. One plugin emits separate wheel
      // TF messages in addition to odom. Wait only when no callback is ready.
      executor.spin_once(std::chrono::milliseconds(20));
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_discovery) {
        discover(node, observations);
        next_discovery = now + std::chrono::seconds(1);
      }
    }
    observations.finish_window(wall_offset());
    complete = rclcpp::ok() && std::chrono::steady_clock::now() >= deadline;
    if (complete) {
      discover(node, observations);
    } else {
      observation_error = "ROS context stopped before the observation window completed";
    }
    executor.remove_node(node);
  } catch (const std::exception & error) {
    observation_error = error.what();
    complete = false;
  }
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started).count();
  const double finished_unix = unix_seconds();
  if (rclcpp::ok()) {rclcpp::shutdown();}
  try {
    write_exclusive(output_path, observations.json(
      complete, requested_seconds, elapsed, started_unix, finished_unix, observation_error) + "\n");
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
  std::cout << "TF observations saved; observation_complete=" << (complete ? "true" : "false")
            << ". Gates remain NOT_YET_VERIFIED.\n";
  return complete ? 0 : 2;
}
