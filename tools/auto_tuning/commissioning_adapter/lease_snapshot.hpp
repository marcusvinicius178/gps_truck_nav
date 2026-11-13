#pragma once

namespace commissioning
{
struct LeaseSnapshot
{
  double lease, now;
};

// A supervisor may renew the file while it is being read. Sample CLOCK_MONOTONIC
// after that read, so a legitimately newer lease is not compared with an old now.
// Truly future timestamps remain invalid; no timing tolerance is introduced.
template<typename Reader, typename Clock>
LeaseSnapshot lease_snapshot(Reader read_lease, Clock monotonic)
{
  const double lease = read_lease();
  const double now = monotonic();
  return {lease, now};
}
}  // namespace commissioning
