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

inline std::string number(double value)
{
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream output;
  output << std::setprecision(17) << value;
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

struct Stamp
{
  int32_t sec = 0;
  uint32_t nanosec = 0;

  std::string json() const
  {
    return "{\"sec\":" + std::to_string(sec) + ",\"nanosec\":" +
           std::to_string(nanosec) + "}";
  }
};

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
           << ",\"w\":" << number(last_transform[6]) << "}}}";
    return output.str();
  }
};

class Observations
{
public:
  void observe(
    const std::string & topic, const std::string & parent, const std::string & child,
    const std::string & gid, const Stamp & stamp, const std::array<double, 7> & transform)
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
  }

  void endpoint(
    const std::string & topic, const std::string & gid,
    const std::string & node_name, const std::string & node_namespace)
  {
    publishers_.emplace(topic, gid, node_name, node_namespace);
  }

  void discovery_error(const std::string & error) {discovery_errors_.insert(error);}

  std::string json(
    bool complete, double requested_seconds, double observed_seconds,
    double started_unix_seconds, double finished_unix_seconds,
    const std::string & error = "") const
  {
    std::ostringstream output;
    output << "{\"schema_version\":1,\"observation_complete\":" << (complete ? "true" : "false")
           << ",\"status\":\"OBSERVATIONS_ONLY\",\"gate0\":\"NOT_YET_VERIFIED\""
           << ",\"gate1\":\"NOT_YET_VERIFIED\",\"simulator_health_verified\":false"
           << ",\"requested_duration_seconds\":" << number(requested_seconds)
           << ",\"observed_duration_seconds\":" << number(observed_seconds)
           << ",\"started_at_unix_seconds\":" << number(started_unix_seconds)
           << ",\"finished_at_unix_seconds\":" << number(finished_unix_seconds)
           << ",\"error\":" << (error.empty() ? "null" : quote(error))
           << ",\"subscriptions\":["
           << "{\"topic\":\"/tf\",\"reliability\":\"BEST_EFFORT\",\"durability\":\"VOLATILE\"},"
           << "{\"topic\":\"/tf_static\",\"reliability\":\"RELIABLE\",\"durability\":\"TRANSIENT_LOCAL\"}]"
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
             << ",\"node_namespace\":" << quote(std::get<3>(item)) << '}';
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
           << "\"Static TF is transient-local; stamps are retained source timestamps and are not freshness evidence.\","
           << "\"Empty or zero GIDs, empty observations and discovery errors must not be treated as a healthy TF tree.\","
           << "\"Raw frame names and transforms are preserved; geometry, localization and mission readiness are not validated by this observer.\"]}";
    return output.str();
  }

private:
  std::map<std::tuple<std::string, std::string, std::string>, Edge> edges_;
  std::set<std::tuple<std::string, std::string, std::string, std::string>> publishers_;
  std::set<std::string> discovery_errors_;
};
}  // namespace phase_a_tf
#endif  // NAV2_PHASE_A_TF_OBSERVATIONS_HPP
