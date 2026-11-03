#ifndef NAV2_PHASE_A_TF_OBSERVATIONS_HPP
#define NAV2_PHASE_A_TF_OBSERVATIONS_HPP

// Dependency-free observation/serialization core. It does not certify TF health.
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include "timing.hpp"

namespace phase_a_tf
{
inline std::string quote(const std::string & value)
{
  std::ostringstream output;
  output << '"';
  for (const unsigned char character : value) {
    if (character == '"' || character == '\\') {
      output << '\\' << character;
    } else if (character < 0x20) {
      output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
             << static_cast<unsigned int>(character) << std::dec;
    } else {
      output << character;
    }
  }
  output << '"';
  return output.str();
}

template<typename Iterator>
std::string gid_hex(Iterator begin, Iterator end)
{
  std::ostringstream output;
  bool nonzero = false;
  for (auto iterator = begin; iterator != end; ++iterator) {
    const auto byte = static_cast<unsigned int>(*iterator);
    nonzero = nonzero || byte != 0;
    output << std::hex << std::setw(2) << std::setfill('0') << byte;
  }
  // An all-zero GID must never look like a verified unique authority.
  return nonzero ? output.str() : "";
}

struct Edge
{
  std::string topic;
  std::string parent;
  std::string child;
  std::set<std::string> publisher_gids;
  uint64_t publisher_gid_unknown_count = 0;
  uint64_t count = 0;
  uint64_t nonfinite_transform_count = 0;
  Stamp first_stamp;
  Stamp last_stamp;
  // Translation x,y,z then quaternion x,y,z,w. Values are observed, not repaired.
  std::array<double, 7> last_transform{};
  std::map<std::string, Samples> publisher_observations;

  std::string json() const
  {
    std::ostringstream output;
    output << "{\"parent\":" << quote(parent) << ",\"child\":" << quote(child)
           << ",\"topic\":" << quote(topic) << ",\"publisher_gids\":[";
    bool first = true;
    for (const auto & gid : publisher_gids) {
      if (!first) {output << ',';}
      first = false;
      output << quote(gid);
    }
    output << "],\"publisher_gid_unknown_count\":" << publisher_gid_unknown_count
           << ",\"count\":" << count
           << ",\"nonfinite_transform_count\":" << nonfinite_transform_count
           << ",\"first_stamp\":" << first_stamp.json()
           << ",\"last_stamp\":" << last_stamp.json()
           << ",\"last_transform\":{\"translation\":{\"x\":" << number(last_transform[0])
           << ",\"y\":" << number(last_transform[1]) << ",\"z\":" << number(last_transform[2])
           << "},\"rotation\":{\"x\":" << number(last_transform[3])
           << ",\"y\":" << number(last_transform[4]) << ",\"z\":" << number(last_transform[5])
           << ",\"w\":" << number(last_transform[6]) << "}}"
           << ",\"publisher_observations\":[";
    first = true;
    for (const auto & item : publisher_observations) {
      if (!first) {output << ',';}
      first = false;
      output << "{\"gid\":" << quote(item.first) << ",\"timing\":" << item.second.json() << '}';
    }
    output << "]}";
    return output.str();
  }
};

class Observations
{
public:
  void observe(
    const std::string & topic, const std::string & parent, const std::string & child,
    const std::string & gid, const Stamp & stamp, const std::array<double, 7> & transform,
    double received_wall_offset = std::numeric_limits<double>::quiet_NaN(),
    int64_t rmw_source_timestamp_ns = 0, int64_t rmw_received_timestamp_ns = 0)
  {
    auto & edge = edges_[{topic, parent, child}];
    edge.topic = topic;
    edge.parent = parent;
    edge.child = child;
    if (edge.count == 0) {edge.first_stamp = stamp;}
    ++edge.count;
    if (gid.empty()) {
      ++edge.publisher_gid_unknown_count;
    } else {
      edge.publisher_gids.insert(gid);
    }
    bool finite = true;
    for (const auto value : transform) {finite = finite && std::isfinite(value);}
    if (!finite) {++edge.nonfinite_transform_count;}
    edge.last_stamp = stamp;
    edge.last_transform = transform;
    edge.publisher_observations[gid].observe(
      stamp, received_wall_offset, rmw_source_timestamp_ns, rmw_received_timestamp_ns);
  }

  void observe_clock(const Stamp & stamp, double received_wall_offset)
  {
    clock_.observe(stamp, received_wall_offset);
  }

  void finish_window(double wall_offset) {window_end_wall_offset_ = wall_offset;}

  void endpoint(
    const std::string & topic, const std::string & gid,
    const std::string & node_name, const std::string & node_namespace,
    const std::string & offered_qos_json = "null")
  {
    publishers_.emplace(topic, gid, node_name, node_namespace, offered_qos_json);
  }

  void discovery_error(const std::string & error) {discovery_errors_.insert(error);}

  void loss_event_support(const std::string & topic, bool supported, const std::string & error = "")
  {
    auto & record = loss_events_[topic];
    record.supported = supported;
    record.error = error;
  }

  void loss_event(const std::string & topic, int32_t total, int32_t change)
  {
    auto & record = loss_events_[topic];
    ++record.callbacks;
    record.reported_total_count = total;
    record.reported_total_count_change = change;
  }

  std::string json(
    bool complete, double requested_seconds, double observed_seconds,
    double started_unix_seconds, double finished_unix_seconds,
    const std::string & error = "") const
  {
    std::ostringstream output;
    output << "{\"schema_version\":2,\"observation_complete\":" << (complete ? "true" : "false")
           << ",\"status\":\"OBSERVATIONS_ONLY\",\"gate0\":\"NOT_YET_VERIFIED\""
           << ",\"gate1\":\"NOT_YET_VERIFIED\",\"simulator_health_verified\":false"
           << ",\"requested_duration_seconds\":" << number(requested_seconds)
           << ",\"observed_duration_seconds\":" << number(observed_seconds)
           << ",\"started_at_unix_seconds\":" << number(started_unix_seconds)
           << ",\"finished_at_unix_seconds\":" << number(finished_unix_seconds)
           << ",\"observation_window_end_wall_offset\":" << number(window_end_wall_offset_)
           << ",\"clock_reference\":\"OBSERVER_LOCAL_CLOCK_SAME_WINDOW\""
           << ",\"clock\":" << clock_.json()
           << ",\"error\":" << (error.empty() ? "null" : quote(error))
           << ",\"subscriptions\":["
           << "{\"topic\":\"/tf\",\"reliability\":\"BEST_EFFORT\",\"durability\":\"VOLATILE\"},"
           << "{\"topic\":\"/tf_static\",\"reliability\":\"RELIABLE\",\"durability\":\"TRANSIENT_LOCAL\"},"
           << "{\"topic\":\"/clock\",\"reliability\":\"BEST_EFFORT\",\"durability\":\"VOLATILE\"}]"
           << ",\"edges\":[";
    bool first = true;
    for (const auto & item : edges_) {
      if (!first) {output << ',';}
      first = false;
      output << item.second.json();
    }
    output << "],\"publishers\":[";
    first = true;
    for (const auto & item : publishers_) {
      if (!first) {output << ',';}
      first = false;
      output << "{\"topic\":" << quote(std::get<0>(item))
             << ",\"gid\":" << quote(std::get<1>(item))
             << ",\"node_name\":" << quote(std::get<2>(item))
             << ",\"node_namespace\":" << quote(std::get<3>(item))
             << ",\"offered_qos\":" << std::get<4>(item) << '}';
    }
    output << "],\"message_loss_events\":[";
    first = true;
    for (const auto & item : loss_events_) {
      if (!first) {output << ',';}
      first = false;
      output << "{\"topic\":" << quote(item.first)
             << ",\"supported\":" << (item.second.supported ? "true" : "false")
             << ",\"callback_count\":" << item.second.callbacks
             << ",\"reported_total_count\":" << item.second.reported_total_count
             << ",\"last_total_count_change\":" << item.second.reported_total_count_change
             << ",\"error\":" << (item.second.error.empty() ? "null" : quote(item.second.error)) << '}';
    }
    output << "],\"endpoint_discovery_errors\":[";
    first = true;
    for (const auto & item : discovery_errors_) {
      if (!first) {output << ',';}
      first = false;
      output << quote(item);
    }
    output << "],\"limitations\":["
           << "\"One publisher GID per observed edge is a finite-window observation, not proof of unique authority for all time.\","
           << "\"GIDs identify publisher endpoints, not uniquely named processes; endpoint discovery may lag or miss departed publishers.\","
           << "\"Dynamic TF uses best-effort QoS to observe best-effort and reliable publishers; dropped or late-discovered samples can hide conflicts.\","
           << "\"Freshness uses only /clock observed inside this process and this window; a later observer clock must not be substituted.\","
           << "\"Message-lost callbacks are middleware reports, not end-to-end delivery accounting; zero events does not prove zero loss.\","
           << "\"Interval statistics describe received samples; they do not establish publisher cadence or transport latency without additional evidence.\","
           << "\"trace_truncated_count counts entries evicted from the bounded diagnostic trace, not messages lost in transport.\","
           << "\"RMW timestamps are retained as raw nanoseconds with implementation-dependent availability; zero is unknown and they are not simulation time.\","
           << "\"Static TF is transient-local; stamps are retained source timestamps and are not freshness evidence.\","
           << "\"Empty or zero GIDs, empty observations and discovery errors must not be treated as a healthy TF tree.\","
           << "\"Raw frame names and transforms are preserved; geometry, localization and mission readiness are not validated by this observer.\"]}";
    return output.str();
  }

private:
  struct LossEvents
  {
    bool supported = false;
    uint64_t callbacks = 0;
    int32_t reported_total_count = 0;
    int32_t reported_total_count_change = 0;
    std::string error;
  };
  std::map<std::tuple<std::string, std::string, std::string>, Edge> edges_;
  std::set<std::tuple<std::string, std::string, std::string, std::string, std::string>> publishers_;
  std::set<std::string> discovery_errors_;
  std::map<std::string, LossEvents> loss_events_;
  Samples clock_;
  double window_end_wall_offset_ = std::numeric_limits<double>::quiet_NaN();
};
}  // namespace phase_a_tf
#endif  // NAV2_PHASE_A_TF_OBSERVATIONS_HPP
