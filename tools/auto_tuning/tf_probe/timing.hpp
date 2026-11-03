#ifndef NAV2_PHASE_A_TF_TIMING_HPP
#define NAV2_PHASE_A_TF_TIMING_HPP

// ROS-independent bounded timing evidence. All wall offsets use one observer's
// steady clock; source stamps remain simulation time and are never retimed.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <sstream>
#include <string>

namespace phase_a_tf
{
inline std::string number(double value)
{
  if (!std::isfinite(value)) {return "null";}
  std::ostringstream output;
  output.precision(17);
  output << value;
  return output.str();
}

struct Stamp
{
  int32_t sec = 0;
  uint32_t nanosec = 0;
  double seconds() const {return sec + nanosec / 1e9;}
  std::string json() const
  {
    return "{\"sec\":" + std::to_string(sec) + ",\"nanosec\":" +
           std::to_string(nanosec) + "}";
  }
};

struct Intervals
{
  uint64_t count = 0;
  double minimum = std::numeric_limits<double>::infinity();
  double maximum = -std::numeric_limits<double>::infinity();
  double sum = 0.0;
  double squared_sum = 0.0;
  void add(double value)
  {
    if (!std::isfinite(value)) {return;}
    ++count;
    minimum = std::min(minimum, value);
    maximum = std::max(maximum, value);
    sum += value;
    squared_sum += value * value;
  }
  std::string json() const
  {
    const auto mean = count ? sum / count : std::numeric_limits<double>::quiet_NaN();
    const auto deviation = count ? std::sqrt(std::max(0.0, squared_sum / count - mean * mean)) : mean;
    return "{\"count\":" + std::to_string(count) + ",\"minimum\":" + number(minimum) +
      ",\"maximum\":" + number(maximum) + ",\"mean\":" + number(mean) +
      ",\"standard_deviation\":" + number(deviation) + "}";
  }
};

struct Sample
{
  Stamp stamp;
  double received_wall_offset;
};

struct Samples
{
  static constexpr size_t trace_capacity = 32;
  uint64_t count = 0;
  uint64_t forward_steps = 0;
  uint64_t backward_steps = 0;
  uint64_t repeated_steps = 0;
  Stamp first_stamp;
  Stamp last_stamp;
  double first_received_wall_offset = std::numeric_limits<double>::quiet_NaN();
  double last_received_wall_offset = std::numeric_limits<double>::quiet_NaN();
  double last_advance_wall_offset = std::numeric_limits<double>::quiet_NaN();
  int64_t last_rmw_source_timestamp_ns = 0;
  int64_t last_rmw_received_timestamp_ns = 0;
  Intervals source_intervals;
  Intervals reception_intervals;
  std::deque<Sample> trace;

  void observe(const Stamp & stamp, double received_wall_offset,
    int64_t rmw_source_timestamp_ns = 0, int64_t rmw_received_timestamp_ns = 0)
  {
    if (count) {
      const auto difference = stamp.seconds() - last_stamp.seconds();
      forward_steps += difference > 0;
      backward_steps += difference < 0;
      repeated_steps += difference == 0;
      if (difference > 0) {last_advance_wall_offset = received_wall_offset;}
      source_intervals.add(difference);
      reception_intervals.add(received_wall_offset - last_received_wall_offset);
    } else {
      first_stamp = stamp;
      first_received_wall_offset = received_wall_offset;
    }
    ++count;
    last_stamp = stamp;
    last_received_wall_offset = received_wall_offset;
    last_rmw_source_timestamp_ns = rmw_source_timestamp_ns;
    last_rmw_received_timestamp_ns = rmw_received_timestamp_ns;
    trace.push_back({stamp, received_wall_offset});
    if (trace.size() > trace_capacity) {trace.pop_front();}
  }

  std::string json() const
  {
    std::ostringstream out;
    out << "{\"count\":" << count << ",\"forward_steps\":" << forward_steps
        << ",\"backward_steps\":" << backward_steps << ",\"repeated_steps\":" << repeated_steps
        << ",\"first_stamp\":" << first_stamp.json() << ",\"last_stamp\":" << last_stamp.json()
        << ",\"first_received_wall_offset\":" << number(first_received_wall_offset)
        << ",\"last_received_wall_offset\":" << number(last_received_wall_offset)
        << ",\"last_advance_wall_offset\":" << number(last_advance_wall_offset)
        << ",\"last_rmw_source_timestamp_ns\":" << last_rmw_source_timestamp_ns
        << ",\"last_rmw_received_timestamp_ns\":" << last_rmw_received_timestamp_ns
        << ",\"source_interval_sim_seconds\":" << source_intervals.json()
        << ",\"reception_interval_wall_seconds\":" << reception_intervals.json()
        << ",\"trace_capacity\":" << trace_capacity
        << ",\"trace_truncated_count\":" << count - trace.size() << ",\"trace\":[";
    bool first = true;
    for (const auto & item : trace) {
      if (!first) {out << ',';}
      first = false;
      out << "{\"stamp\":" << item.stamp.json()
          << ",\"received_wall_offset\":" << number(item.received_wall_offset) << '}';
    }
    out << "]}";
    return out.str();
  }
};
}  // namespace phase_a_tf
#endif  // NAV2_PHASE_A_TF_TIMING_HPP
