#pragma once
#include "guard.hpp"

namespace commissioning
{
// Image smoke entry: no ROS, filesystem, network, timing, hardware or publication.
// The full synthetic regression suite is tests/test_commissioning_adapter.py.
inline bool self_test()
{
  const Geometry g{6.816500073351341, 2.598120190224205, -1.299, 20,
    .4797896546711007, -.524, .524, -.524, .524};
  const auto straight = convert({.3, 0, 0, 0, 0, 0}, g);
  const auto curve = convert({.3, 0, 0, 0, 0, .003}, g);
  const auto reverse = convert({-.3, 0, 0, 0, 0, 0}, g);
  const auto spin = convert({0, 0, 0, 0, 0, .003}, g);
  const auto overspeed = convert({.51, 0, 0, 0, 0, 0}, g);
  Guard prepare(g, false, 0, 30);
  prepare.command({0, 0, 0, 0, 0, 0}, "unexpected", 0, 0);
  Guard expiry(g, true, 0, 30);
  expiry.poll(.5, 0);
  // Reproduce the reported mixed snapshot without ROS or waiting on a real clock.
  Guard sync(g, false, 3387, 30);
  sync.observe_graph(3388.99, true, "", "", "gazebo", "");
  sync.observe_clock(15.111822, 3388.99);
  sync.observe_odom(15.215145, 3388.992950571, 0, 0, 0, 0);
  sync.observe_joints(15.215145, 3388.993519428, 0, 0);
  sync.poll(3388.995064891, 3388.99);
  const bool pending = sync.clock_sync_pending && sync.state != "FAULT" &&
    !sync.observations_ready(3388.995064891, 3388.99);
  Guard persistent = sync;
  persistent.poll(3389.0931, 3389.09);
  sync.observe_clock(15.218478, 3388.995958977);
  sync.poll(3388.996, 3388.99);
  return straight.status == "ACCEPTED" && straight.plugin_speed == .3 &&
    curve.status == "ACCEPTED" && std::abs(curve.plugin_speed - .303897) < 1e-12 &&
    std::abs(std::tan(curve.plugin_angle) / g.wheelbase - .01) < 1e-12 &&
    reverse.status == "REJECTED" && spin.status == "REJECTED" && overspeed.status == "REJECTED" &&
    prepare.fault_reason == "UNEXPECTED_COMMAND_IN_READ_ONLY_PREPARATION" &&
    expiry.fault_reason == "SUPERVISOR_LEASE_EXPIRED" && pending &&
    persistent.fault_reason == "SENSOR_CLOCK_SYNC_TIMEOUT" &&
    sync.state != "FAULT" && !sync.clock_sync_pending && sync.clock_sync_recoveries == 1 &&
    !sync.observations_ready(3388.996, 3388.99) && sync.limits.future_stamp_tolerance == .1;
}
}  // namespace commissioning
