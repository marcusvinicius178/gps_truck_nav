// Prospective low-speed commissioning guard. No ROS or I/O in this core.
// These engineering test limits do not change or validate the physical plant.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace commissioning
{
constexpr double absent = -std::numeric_limits<double>::infinity();
inline bool finite(double value) {return std::isfinite(value);}
inline bool fresh(double receipt, double now, double timeout)
{
  return finite(receipt) && finite(now) && 0 <= now - receipt && now - receipt < timeout;
}

struct Geometry
{
  double wheelbase{}, track{}, rear_right_y{}, plugin_speed{}, plugin_steer{};
  double left_lower{}, left_upper{}, right_lower{}, right_upper{};
  void validate() const
  {
    for (double x : {wheelbase, track, rear_right_y, plugin_speed, plugin_steer,
        left_lower, left_upper, right_lower, right_upper}) {
      if (!finite(x)) {throw std::invalid_argument("NONFINITE_GEOMETRY");}
    }
    if (!(wheelbase > 0 && track > 0 && rear_right_y < 0 && plugin_speed > 0 &&
      plugin_steer > 0 && plugin_steer < 1.5707963267948966 &&
      left_lower < 0 && left_upper > 0 && right_lower < 0 && right_upper > 0 &&
      left_lower > -1.5707963267948966 && right_lower > -1.5707963267948966 &&
      left_upper < 1.5707963267948966 && right_upper < 1.5707963267948966)) {
      throw std::invalid_argument("INVALID_GEOMETRY");
    }
    if (!finite(2 * wheelbase) || !finite(track / (2 * wheelbase))) {
      throw std::invalid_argument("GEOMETRY_ARITHMETIC_OVERFLOW");
    }
  }
};

struct Limits
{
  // Fixed commissioning bounds, not the 20 km/h navigation envelope.
  double speed = 0.5, yaw_rate = 0.02, joint_target = 0.20;
  double stop_band = 0.01, resume_speed = 0.02;
  double command_timeout = 0.25, observation_timeout = 0.5;
  double clock_stall = 1.0, lease_timeout = 0.5;
  double observed_speed = 0.7, observed_yaw = 0.05, observed_steer = 0.25;
  double standstill_dwell = 1.0, future_stamp_tolerance = 0.1;
  double max_travel = 10.0, max_displacement = 10.0;
};

struct Conversion
{
  std::string status = "REJECTED", reason;
  double rear_speed = 0, yaw_rate = 0, plugin_speed = 0, plugin_angle = 0;
  double virtual_angle = 0, left_target = 0, right_target = 0, curvature = 0;
};

inline Conversion convert(const std::array<double, 6> & twist, const Geometry & geometry,
  const Limits & limits = {}, bool held = false)
{
  geometry.validate();
  Conversion r;
  for (double x : twist) {if (!finite(x)) {r.reason = "NONFINITE_COMMAND"; return r;}}
  if (twist[1] != 0 || twist[2] != 0 || twist[3] != 0 || twist[4] != 0) {
    r.reason = "UNSUPPORTED_TWIST_COMPONENT"; return r;
  }
  const double v = twist[0], w = twist[5];
  if (v < 0) {r.reason = "REVERSE_DISABLED_FOR_FIRST_COMMISSIONING"; return r;}
  if (v <= limits.stop_band) {
    r.status = w == 0 ? "STOP" : "REJECTED";
    r.reason = w == 0 ? "STOP_BAND" : "UNREALIZABLE_NEAR_ZERO_YAW";
    return r;
  }
  if (v > limits.speed || std::abs(w) > limits.yaw_rate) {
    r.reason = "COMMISSIONING_SPEED_OR_YAW_LIMIT"; return r;
  }
  const double k = w / v, product = geometry.wheelbase * k;
  if (!finite(k) || !finite(product)) {r.reason = "NUMERIC_OVERFLOW"; return r;}
  const double angle = std::atan(product), ratio = geometry.track / (2 * geometry.wheelbase);
  const double tangent = std::tan(angle);
  const double left = std::atan2(tangent, 1 - ratio * tangent);
  const double right = std::atan2(tangent, 1 + ratio * tangent);
  const double vp = v - w * geometry.rear_right_y;
  if (!finite(vp) || !finite(left) || !finite(right) || !finite(ratio) ||
    vp <= 0 || vp > geometry.plugin_speed) {
    r.reason = "INVALID_OR_LIMITED_RIGHT_REAR_TARGET"; return r;
  }
  if (std::abs(angle) > geometry.plugin_steer || left < geometry.left_lower ||
    left > geometry.left_upper || right < geometry.right_lower || right > geometry.right_upper) {
    r.reason = "PHYSICAL_STEERING_ENVELOPE"; return r;
  }
  if (std::max(std::abs(left), std::abs(right)) > limits.joint_target) {
    r.reason = "COMMISSIONING_STEERING_ENVELOPE"; return r;
  }
  if (held && v < limits.resume_speed) {
    r.status = "HOLD"; r.reason = "STOP_HYSTERESIS"; return r;
  }
  r.status = "ACCEPTED"; r.reason = "UNCHANGED_CURVATURE_NO_CLIPPING";
  r.rear_speed = v; r.yaw_rate = w; r.plugin_speed = vp; r.plugin_angle = angle;
  r.virtual_angle = angle; r.left_target = left; r.right_target = right; r.curvature = k;
  return r;
}

struct Timed
{
  double stamp = absent, received = absent, progressed = absent;
  bool update(double next_stamp, double now)
  {
    if (!finite(next_stamp) || next_stamp < 0 || !finite(now) ||
      (finite(received) && now < received) || (finite(stamp) && next_stamp < stamp)) {return false;}
    if (!finite(stamp) || next_stamp > stamp) {progressed = now;}
    stamp = next_stamp; received = now; return true;
  }
};

// Callback-independent state machine; caller emits heartbeat only after poll.
// A supervisor in a separate process must kill the simulation on this loop's loss.
class Guard
{
public:
  Guard(Geometry g, bool execute, double started, double duration)
  : geometry(g), execute_(execute), started_(started), duration_(duration), last_now_(started)
  {
    geometry.validate();
    if (!finite(started) || !finite(duration) || duration < 1 || duration > 900) {
      throw std::invalid_argument("INVALID_WALL_BUDGET");
    }
  }

  Geometry geometry;
  Limits limits;
  std::string state = "ARMING", fault_reason, waiting_reason = "OBSERVATIONS_PENDING";
  std::string input_gid, output_gid, subscriber_gid;
  Timed clock, odom, joints;
  double speed = 0, lateral_speed = 0, yaw_rate = 0, left = 0, right = 0;
  double travel = 0, displacement = 0;
  unsigned long command_count = 0, rejected_count = 0, stop_count = 0;
  Conversion last;

  void fault(const std::string & reason)
  {
    if (state != "FAULT") {fault_reason = reason;}
    state = "FAULT";
    last = Conversion{}; last.reason = fault_reason;
  }

  void observe_clock(double sim, double now)
  {
    if (finite(clock.stamp) && sim - clock.stamp > 0.25) {
      fault("SIM_CLOCK_JUMP_EXCEEDS_COMMISSIONING_WINDOW"); return;
    }
    if (!clock.update(sim, now)) {fault("INVALID_OR_REGRESSED_SIM_CLOCK");}
  }

  void observe_odom(double stamp, double now, double v, double w, double rear_x, double rear_y,
    double rear_lateral_speed = 0)
  {
    if (!fresh(odom.received, now, limits.observation_timeout)) {
      standstill_since_ = absent; standstill_stamp_ = absent;
    }
    if (!odom.update(stamp, now) || !finite(v) || !finite(w) || !finite(rear_x) || !finite(rear_y) ||
      !finite(rear_lateral_speed)) {
      fault("INVALID_OR_REGRESSED_ODOMETRY"); return;
    }
    speed = v; lateral_speed = rear_lateral_speed; yaw_rate = w;
    if (std::hypot(v, lateral_speed) > limits.observed_speed || std::abs(w) > limits.observed_yaw) {
      fault("OBSERVED_MOTION_ENVELOPE_EXCEEDED"); return;
    }
    if (!have_origin_) {origin_x_ = rear_x; origin_y_ = rear_y; have_origin_ = true;}
    if (have_position_) {travel += std::hypot(rear_x - previous_x_, rear_y - previous_y_);}
    previous_x_ = rear_x; previous_y_ = rear_y; have_position_ = true;
    displacement = std::hypot(rear_x - origin_x_, rear_y - origin_y_);
    if (!finite(travel) || !finite(displacement)) {fault("ODOMETRY_DISTANCE_INVALID");}
    if (travel > limits.max_travel || displacement > limits.max_displacement) {
      fault("TRAVEL_OR_DISPLACEMENT_LIMIT");
    }
    if (std::hypot(v, lateral_speed) <= limits.stop_band && std::abs(w) <= 0.005) {
      if (!finite(standstill_since_)) {standstill_since_ = now; standstill_stamp_ = stamp;}
    } else {standstill_since_ = absent; standstill_stamp_ = absent;}
  }

  void observe_joints(double stamp, double now, double left_angle, double right_angle)
  {
    if (!joints.update(stamp, now) || !finite(left_angle) || !finite(right_angle)) {
      fault("INVALID_OR_REGRESSED_JOINT_STATES"); return;
    }
    left = left_angle; right = right_angle;
    if (left < geometry.left_lower || left > geometry.left_upper ||
      right < geometry.right_lower || right > geometry.right_upper ||
      std::max(std::abs(left), std::abs(right)) > limits.observed_steer) {
      fault("OBSERVED_STEERING_ENVELOPE_EXCEEDED");
    }
  }

  // Identity/type/count checks are performed against the ROS graph by the wrapper.
  // The selected GIDs are pinned; restarts require a new session, never auto-rearm.
  void observe_graph(double now, bool valid, const std::string & source,
    const std::string & actuator, const std::string & destination, const std::string & reason,
    bool transport_valid = false)
  {
    graph_at_ = now; graph_valid_ = valid; graph_observations_valid_ = valid || transport_valid;
    if (graph_observations_valid_) {
      if (have_destination_ && (actuator != output_gid || destination != subscriber_gid)) {
        fault("PINNED_DESTINATION_IDENTITY_CHANGED"); return;
      }
      have_destination_ = true; output_gid = actuator; subscriber_gid = destination;
    }
    if (!valid) {graph_reason_ = reason; if (state == "ARMED" || state == "READY_READ_ONLY") {fault(reason);} return;}
    if (have_graph_ && (source != input_gid || actuator != output_gid || destination != subscriber_gid)) {
      fault("PINNED_GRAPH_IDENTITY_CHANGED"); return;
    }
    have_graph_ = true; input_gid = source; output_gid = actuator; subscriber_gid = destination;
    graph_reason_.clear();
  }

  std::string health(double now, double lease, bool allow_waiting_source = false) const
  {
    if (!finite(now) || now < last_now_) {return "MONOTONIC_CLOCK_REGRESSED";}
    if (now - started_ >= duration_) {return "WALL_DURATION_LIMIT";}
    if (!fresh(lease, now, limits.lease_timeout)) {return "SUPERVISOR_LEASE_EXPIRED";}
    if (!fresh(clock.received, now, limits.observation_timeout)) {return "CLOCK_RECEIPT_STALE";}
    if (!fresh(clock.progressed, now, limits.clock_stall)) {return "SIM_CLOCK_STALLED";}
    if (!fresh(odom.received, now, limits.observation_timeout) ||
      !fresh(odom.progressed, now, limits.observation_timeout)) {return "ODOMETRY_STALE_OR_FROZEN";}
    if (!fresh(joints.received, now, limits.observation_timeout) ||
      !fresh(joints.progressed, now, limits.observation_timeout)) {return "JOINT_STATES_STALE_OR_FROZEN";}
    for (double stamp : {odom.stamp, joints.stamp}) {
      const double age = clock.stamp - stamp;
      if (!finite(age) || age < -limits.future_stamp_tolerance || age >= limits.observation_timeout) {
        return "SENSOR_STAMP_OUTSIDE_CLOCK_WINDOW";
      }
    }
    if (!fresh(graph_at_, now, limits.observation_timeout) ||
      !(graph_valid_ || (allow_waiting_source && graph_observations_valid_))) {
      return graph_reason_.empty() ? "COMMAND_GRAPH_STALE" : graph_reason_;
    }
    if (std::hypot(speed, lateral_speed) > limits.observed_speed || std::abs(yaw_rate) > limits.observed_yaw) {
      return "OBSERVED_MOTION_ENVELOPE_EXCEEDED";
    }
    if (left < geometry.left_lower || left > geometry.left_upper ||
      right < geometry.right_lower || right > geometry.right_upper ||
      std::max(std::abs(left), std::abs(right)) > limits.observed_steer) {
      return "OBSERVED_STEERING_ENVELOPE_EXCEEDED";
    }
    if (travel > limits.max_travel || displacement > limits.max_displacement) {
      return "TRAVEL_OR_DISPLACEMENT_LIMIT";
    }
    if (!execute_ && state == "READY_READ_ONLY" &&
      (std::hypot(speed, lateral_speed) > limits.stop_band || std::abs(yaw_rate) > 0.005)) {
      return "UNEXPECTED_MOVEMENT_IN_READ_ONLY_PREPARATION";
    }
    return {};
  }

  bool standstill_observed(double now) const
  {
    return fresh(odom.received, now, limits.observation_timeout) &&
      fresh(odom.progressed, now, limits.observation_timeout) &&
      finite(standstill_since_) && now - standstill_since_ >= limits.standstill_dwell &&
      odom.stamp - standstill_stamp_ >= 0.5 && std::hypot(speed, lateral_speed) <= limits.stop_band &&
      std::abs(yaw_rate) <= 0.005;
  }

  bool observations_ready(double now, double lease) const
  {
    return state != "FAULT" && health(now, lease, true).empty() && standstill_observed(now);
  }

  void poll(double now, double lease)
  {
    if (state == "FAULT") {return;}
    const auto reason = health(now, lease);
    last_now_ = now;
    if (!reason.empty()) {
      waiting_reason = reason;
      // A valid independent lease is mandatory even during discovery/warmup.
      if (reason == "SUPERVISOR_LEASE_EXPIRED" || reason == "WALL_DURATION_LIMIT" ||
        reason == "MONOTONIC_CLOCK_REGRESSED" || state == "ARMED" || state == "READY_READ_ONLY" ||
        reason == "OBSERVED_MOTION_ENVELOPE_EXCEEDED" ||
        reason == "OBSERVED_STEERING_ENVELOPE_EXCEEDED" ||
        reason == "TRAVEL_OR_DISPLACEMENT_LIMIT" ||
        now - started_ >= 45.0) {fault(reason);}
      return;
    }
    waiting_reason.clear();
    if (state == "ARMED") {
      if (!fresh(last_command_, now, limits.command_timeout)) {fault("COMMAND_WATCHDOG_EXPIRED");}
      return;
    }
    const bool standstill = standstill_observed(now);
    if (!standstill) {waiting_reason = "STANDSTILL_DWELL_REQUIRED"; return;}
    if (!execute_) {state = "READY_READ_ONLY"; return;}
    if (!fresh(last_zero_, now, limits.command_timeout)) {waiting_reason = "FRESH_AUTHORIZED_ZERO_REQUIRED"; return;}
    state = "ARMED"; last_command_ = last_zero_;
  }

  // Return true only when a new output may be published. Never repeat cached commands.
  bool command(const std::array<double, 6> & twist, const std::string & gid,
    double now, double lease)
  {
    ++command_count;
    if (state == "FAULT") {return false;}
    if (!execute_) {fault("UNEXPECTED_COMMAND_IN_READ_ONLY_PREPARATION"); return false;}
    const auto result = convert(twist, geometry, limits, held_);
    if (result.status == "REJECTED") {++rejected_count; fault(result.reason); return false;}
    const bool zero = std::all_of(twist.begin(), twist.end(), [](double x) {return x == 0;});
    if (!have_graph_) {
      if (!zero) {fault("NONZERO_BEFORE_AUTHORITY_ESTABLISHED");}
      return false;
    }
    if (gid.empty() || gid != input_gid) {fault("PER_MESSAGE_PUBLISHER_GID_MISMATCH"); return false;}
    if (state != "ARMED") {
      if (!zero) {fault("NONZERO_BEFORE_EXPLICIT_SESSION_ARMED"); return false;}
      last_zero_ = now;
      poll(now, lease);
      if (state != "ARMED") {return false;}
    }
    const auto reason = health(now, lease);
    if (!reason.empty()) {fault(reason); return false;}
    if (!fresh(last_command_, now, limits.command_timeout)) {fault("COMMAND_WATCHDOG_EXPIRED"); return false;}
    last_command_ = now; last = result;
    held_ = result.status == "STOP" || result.status == "HOLD";
    stop_count += held_;
    return true;
  }

private:
  bool execute_, have_graph_ = false, graph_valid_ = false, held_ = true;
  bool have_destination_ = false, graph_observations_valid_ = false;
  bool have_origin_ = false, have_position_ = false;
  double started_, duration_, last_now_, graph_at_ = absent, last_command_ = absent, last_zero_ = absent;
  double standstill_since_ = absent, standstill_stamp_ = absent;
  double origin_x_ = 0, origin_y_ = 0, previous_x_ = 0, previous_y_ = 0;
  std::string graph_reason_;
};
}  // namespace commissioning
