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
  return straight.status == "ACCEPTED" && straight.plugin_speed == .3 &&
    curve.status == "ACCEPTED" && std::abs(curve.plugin_speed - .303897) < 1e-12 &&
    std::abs(std::tan(curve.plugin_angle) / g.wheelbase - .01) < 1e-12 &&
    reverse.status == "REJECTED" && spin.status == "REJECTED" && overspeed.status == "REJECTED" &&
    prepare.fault_reason == "UNEXPECTED_COMMAND_IN_READ_ONLY_PREPARATION" &&
    expiry.fault_reason == "SUPERVISOR_LEASE_EXPIRED";
}
}  // namespace commissioning
