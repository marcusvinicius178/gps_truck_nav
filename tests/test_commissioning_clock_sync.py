"""Exercise clock reconciliation in the actual ROS-independent C++ Guard.

All observations and commands are in-memory values. No ROS, DDS, Docker,
simulator, publisher, authorization or movement is created by these tests.
"""

from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools/auto_tuning"))
import commissioning_session as session

SOURCE = r'''
#include "guard.hpp"
#include <iostream>
#include <string>

using namespace commissioning;
#define CHECK(x) do {if (!(x)) {std::cerr << __LINE__ << ": " << #x << "\n"; return 1;}} while (0)

constexpr double wall = 3388.990000000;
constexpr double clock_stamp = 15.111822;
constexpr double sensor_stamp = 15.215145;
constexpr double odom_receipt = 3388.992950571;
constexpr double joint_receipt = 3388.993519428;
constexpr double fault_snapshot = 3388.995064891;
constexpr double recovered_clock = 15.218478;
constexpr double clock_receipt = 3388.995958977;

Geometry geometry()
{
  return {6.816500073351341, 2.598120190224205, -1.299, 20,
    .4797896546711007, -.524, .524, -.524, .524};
}

std::array<double, 6> command(double v = 0) {return {v, 0, 0, 0, 0, 0};}
bool near(double a, double b) {return std::abs(a - b) < 1e-10;}

void graph(Guard & g, double now, bool execute = false)
{
  g.observe_graph(now, true, execute ? "source" : "", execute ? "self" : "", "plugin", "");
}

void coherent(Guard & g, double now, double stamp, bool execute = false)
{
  g.observe_clock(stamp, now);
  g.observe_odom(stamp, now, 0, 0, 0, 0);
  g.observe_joints(stamp, now, 0, 0);
  graph(g, now, execute);
}

Guard warm(bool execute = false, bool arm = false)
{
  Guard g(geometry(), execute, wall - 1.1, 120);
  for (int i = 0; i <= 22; ++i) {
    const double offset = (i - 22) * .05;
    coherent(g, wall + offset, clock_stamp + offset, execute);
    if (arm) {g.command(command(), "source", wall + offset, wall + offset);}
    g.poll(wall + offset, wall + offset);
  }
  return g;
}

void observed_mismatch(Guard & g)
{
  g.observe_odom(sensor_stamp, odom_receipt, 0, 0, 0, 0);
  g.observe_joints(sensor_stamp, joint_receipt, 0, 0);
}

void recover(Guard & g)
{
  g.observe_clock(recovered_clock, clock_receipt);
  g.poll(clock_receipt, clock_receipt);
}

int main(int argc, char ** argv)
{
  if (argc != 2) {return 2;}
  const std::string test = argv[1];
  if (test == "observed_103323ms_is_pending_not_ready") {
    auto g = warm();
    CHECK(g.state == "READY_READ_ONLY");
    observed_mismatch(g);
    CHECK(near(g.clock.stamp - g.odom.stamp, -.103323));
    CHECK(g.health(fault_snapshot, fault_snapshot) == "SENSOR_STAMP_OUTSIDE_CLOCK_WINDOW");
    CHECK(!g.observations_ready(fault_snapshot, fault_snapshot));
    g.poll(fault_snapshot, fault_snapshot);
    CHECK(g.state == "READY_READ_ONLY" && g.fault_reason.empty());
    CHECK(g.clock_sync_pending && g.clock_sync_episodes == 1);
    CHECK(near(g.clock_sync_started, odom_receipt));
    CHECK(g.waiting_reason == "SENSOR_CLOCK_SYNC_PENDING");
    CHECK(!g.observations_ready(fault_snapshot, fault_snapshot));
    CHECK(!g.standstill_observed(fault_snapshot));
    CHECK(g.command_count == 0);
  } else if (test == "legitimate_clock_recovery_restarts_dwell") {
    auto g = warm(); observed_mismatch(g); g.poll(fault_snapshot, fault_snapshot);
    recover(g);
    CHECK(!g.clock_sync_pending && g.clock_sync_recoveries == 1);
    CHECK(g.fault_reason.empty() && g.state == "READY_READ_ONLY");
    CHECK(!g.observations_ready(clock_receipt, clock_receipt));
    for (int i = 0; i <= 21; ++i) {
      const double now = clock_receipt + .01 + i * .05;
      coherent(g, now, recovered_clock + .01 + i * .05);
      g.poll(now, now);
      if (i < 20) {CHECK(!g.observations_ready(now, now));}
      if (i == 21) {CHECK(g.observations_ready(now, now));}
      CHECK(g.fault_reason.empty());
    }
  } else if (test == "recovery_requires_simulation_dwell_too") {
    auto g = warm(); observed_mismatch(g); g.poll(fault_snapshot, fault_snapshot); recover(g);
    for (int i = 0; i <= 22; ++i) {
      const double now = clock_receipt + .01 + i * .05;
      coherent(g, now, recovered_clock + .001 + i * .01);
      g.poll(now, now);
      CHECK(!g.observations_ready(now, now));
      CHECK(g.fault_reason.empty());
    }
  } else if (test == "pending_samples_cannot_accrue_recovery_dwell") {
    auto g = warm(); observed_mismatch(g); g.poll(fault_snapshot, fault_snapshot);
    const double during = odom_receipt + .01;
    g.observe_odom(sensor_stamp + .001, during, 0, 0, 0, 0);
    g.observe_joints(sensor_stamp + .001, during, 0, 0); g.poll(during, during);
    CHECK(g.clock_sync_pending);
    const double recovered = odom_receipt + .09;
    g.observe_clock(recovered_clock, recovered); g.poll(recovered, recovered);
    CHECK(!g.clock_sync_pending && g.fault_reason.empty());
    for (int i = 0; i <= 21; ++i) {
      const double now = recovered + .001 + i * .05;
      coherent(g, now, recovered_clock + .001 + i * .05); g.poll(now, now);
      if (i < 20) {CHECK(!g.observations_ready(now, now));}
      if (i == 21) {CHECK(g.observations_ready(now, now));}
      CHECK(g.fault_reason.empty());
    }
  } else if (test == "fresh_callbacks_do_not_renew_pending_deadline") {
    auto g = warm(); observed_mismatch(g); g.poll(fault_snapshot, fault_snapshot);
    const double began = g.clock_sync_started;
    for (int i = 1; i <= 5; ++i) {
      const double now = fault_snapshot + i * .02;
      g.observe_clock(clock_stamp + i * .01, now);
      g.observe_odom(sensor_stamp + i * .01, now, 0, 0, 0, 0);
      g.observe_joints(sensor_stamp + i * .01, now, 0, 0);
      graph(g, now); g.poll(now, now);
      CHECK(near(g.clock_sync_started, began));
      if (i < 5) {CHECK(g.state == "READY_READ_ONLY" && g.clock_sync_pending);}
    }
    CHECK(g.state == "FAULT" && g.fault_reason == "SENSOR_CLOCK_SYNC_TIMEOUT");
    CHECK(g.clock_sync_episodes == 1 && g.clock_sync_recoveries == 0);
  } else if (test == "coherent_recovery_after_deadline_still_faults") {
    auto g = warm(); observed_mismatch(g); g.poll(fault_snapshot, fault_snapshot);
    const double now = odom_receipt + .100001;
    g.observe_clock(recovered_clock, now);
    CHECK(g.health(now, now).empty());
    g.poll(now, now);
    CHECK(g.fault_reason == "SENSOR_CLOCK_SYNC_TIMEOUT" && g.clock_sync_recoveries == 0);
  } else if (test == "callback_age_is_counted_before_first_poll") {
    auto g = warm(); observed_mismatch(g);
    const double now = odom_receipt + .100001;
    g.poll(now, now);
    CHECK(g.fault_reason == "SENSOR_CLOCK_SYNC_TIMEOUT");
    CHECK(near(g.clock_sync_started, odom_receipt));
  } else if (test == "timeout_boundary_is_exclusive") {
    Guard g(geometry(), false, 0, 120);
    g.observe_clock(10, 0); g.observe_odom(10.11, 0, 0, 0, 0, 0);
    g.observe_joints(10.11, 0, 0, 0); graph(g, 0); g.poll(0, 0);
    CHECK(g.clock_sync_pending);
    g.poll(.099, .099); CHECK(g.fault_reason.empty());
    g.poll(.1, .1); CHECK(g.fault_reason == "SENSOR_CLOCK_SYNC_TIMEOUT");
  } else if (test == "only_violating_sensor_anchors_deadline") {
    auto g = warm();
    const double now = wall + .2;
    g.observe_clock(clock_stamp, now);
    g.observe_odom(sensor_stamp, now, 0, 0, 0, 0);
    graph(g, now); g.poll(now, now);
    CHECK(g.clock_sync_pending && g.fault_reason.empty());
    CHECK(near(g.clock_sync_started, now));
  } else if (test == "joints_first_anchors_deadline") {
    auto g = warm();
    g.observe_joints(sensor_stamp, odom_receipt, 0, 0);
    g.observe_odom(sensor_stamp, joint_receipt, 0, 0, 0, 0);
    g.poll(fault_snapshot, fault_snapshot);
    CHECK(g.clock_sync_pending && near(g.clock_sync_started, odom_receipt));
  } else if (test == "clock_drop_during_pending_is_not_deferred") {
    auto g = warm(); const double now = wall + .49;
    g.observe_odom(sensor_stamp, now, 0, 0, 0, 0);
    g.observe_joints(sensor_stamp, now, 0, 0); graph(g, now); g.poll(now, now);
    CHECK(g.clock_sync_pending);
    g.poll(now + .02, now + .02);
    CHECK(g.fault_reason == "CLOCK_RECEIPT_STALE");
  } else if (test == "clock_frozen_during_pending_is_not_deferred") {
    auto g = warm(); const double now = wall + .99;
    g.observe_clock(clock_stamp, now);
    g.observe_odom(sensor_stamp, now, 0, 0, 0, 0);
    g.observe_joints(sensor_stamp, now, 0, 0); graph(g, now); g.poll(now, now);
    CHECK(g.clock_sync_pending);
    g.observe_clock(clock_stamp, now + .02); g.poll(now + .02, now + .02);
    CHECK(g.fault_reason == "SIM_CLOCK_STALLED");
  } else if (test == "frozen_odom_during_pending_is_not_deferred" ||
             test == "frozen_joints_during_pending_is_not_deferred") {
    auto g = warm(); const double now = wall + .49;
    const bool freeze_odom = test == "frozen_odom_during_pending_is_not_deferred";
    g.observe_clock(clock_stamp, now);
    g.observe_odom(freeze_odom ? clock_stamp : sensor_stamp, now, 0, 0, 0, 0);
    g.observe_joints(freeze_odom ? sensor_stamp : clock_stamp, now, 0, 0);
    graph(g, now); g.poll(now, now); CHECK(g.clock_sync_pending);
    g.observe_odom(g.odom.stamp, now + .02, 0, 0, 0, 0);
    g.observe_joints(g.joints.stamp, now + .02, 0, 0);
    g.poll(now + .02, now + .02);
    CHECK(g.fault_reason == (freeze_odom ? "ODOMETRY_STALE_OR_FROZEN" : "JOINT_STATES_STALE_OR_FROZEN"));
  } else if (test == "clock_regression_during_pending_faults") {
    auto g = warm(); observed_mismatch(g); g.poll(fault_snapshot, fault_snapshot);
    g.observe_clock(clock_stamp - .001, fault_snapshot + .001);
    CHECK(g.fault_reason == "INVALID_OR_REGRESSED_SIM_CLOCK");
  } else if (test == "monotonic_regression_during_pending_faults") {
    auto g = warm(); observed_mismatch(g); g.poll(fault_snapshot, fault_snapshot);
    g.poll(fault_snapshot - .001, fault_snapshot - .001);
    CHECK(g.fault_reason == "MONOTONIC_CLOCK_REGRESSED");
  } else if (test == "odom_regression_during_pending_faults") {
    auto g = warm(); observed_mismatch(g); g.poll(fault_snapshot, fault_snapshot);
    g.observe_odom(sensor_stamp - .001, fault_snapshot + .001, 0, 0, 0, 0);
    CHECK(g.fault_reason == "INVALID_OR_REGRESSED_ODOMETRY");
  } else if (test == "joint_regression_during_pending_faults") {
    auto g = warm(); observed_mismatch(g); g.poll(fault_snapshot, fault_snapshot);
    g.observe_joints(sensor_stamp - .001, fault_snapshot + .001, 0, 0);
    CHECK(g.fault_reason == "INVALID_OR_REGRESSED_JOINT_STATES");
  } else if (test == "lease_loss_during_pending_faults") {
    auto g = warm(); observed_mismatch(g); g.poll(fault_snapshot, fault_snapshot);
    g.poll(fault_snapshot + .001, fault_snapshot - .6);
    CHECK(g.fault_reason == "SUPERVISOR_LEASE_EXPIRED");
  } else if (test == "graph_violation_during_pending_faults") {
    auto g = warm(); observed_mismatch(g); g.poll(fault_snapshot, fault_snapshot);
    CHECK(g.state == "READY_READ_ONLY");
    g.observe_graph(fault_snapshot + .001, false, "", "", "plugin", "UNEXPECTED_ACTUATOR_PUBLISHER");
    CHECK(g.fault_reason == "UNEXPECTED_ACTUATOR_PUBLISHER");
  } else if (test == "graph_staleness_during_pending_faults") {
    auto g = warm(); const double now = wall + .49;
    g.observe_clock(clock_stamp, now);
    g.observe_odom(sensor_stamp, now, 0, 0, 0, 0);
    g.observe_joints(sensor_stamp, now, 0, 0); g.poll(now, now);
    CHECK(g.clock_sync_pending); g.poll(now + .02, now + .02);
    CHECK(g.fault_reason == "COMMAND_GRAPH_STALE");
  } else if (test == "destination_gid_change_during_pending_faults") {
    auto g = warm(); observed_mismatch(g); g.poll(fault_snapshot, fault_snapshot);
    g.observe_graph(fault_snapshot + .001, true, "", "", "replacement-plugin", "");
    CHECK(g.fault_reason == "PINNED_DESTINATION_IDENTITY_CHANGED");
  } else if (test == "source_gid_change_during_pending_faults") {
    auto g = warm(true); observed_mismatch(g); g.poll(fault_snapshot, fault_snapshot);
    CHECK(g.clock_sync_pending && g.state == "ARMING");
    g.observe_graph(fault_snapshot + .001, true, "replacement-source", "self", "plugin", "");
    CHECK(g.fault_reason == "PINNED_GRAPH_IDENTITY_CHANGED");
  } else if (test == "read_only_roll_during_pending_faults") {
    auto g = warm(); observed_mismatch(g); g.poll(fault_snapshot, fault_snapshot);
    g.observe_odom(sensor_stamp + .001, fault_snapshot + .001, .02, 0, 0, 0);
    g.poll(fault_snapshot + .001, fault_snapshot + .001);
    CHECK(g.fault_reason == "UNEXPECTED_MOVEMENT_IN_READ_ONLY_PREPARATION");
  } else if (test == "old_sensor_is_not_reconciliation_candidate") {
    auto g = warm();
    for (int i = 1; i <= 3; ++i) {g.observe_clock(clock_stamp + i * .2, wall + i * .01);}
    const double now = wall + .03;
    g.observe_odom(clock_stamp, now, 0, 0, 0, 0);
    g.observe_joints(clock_stamp + .6, now, 0, 0); graph(g, now); g.poll(now, now);
    CHECK(!g.clock_sync_pending && g.fault_reason == "SENSOR_STAMP_OUTSIDE_CLOCK_WINDOW");
  } else if (test == "future_sensor_cannot_mask_an_old_sensor") {
    auto g = warm();
    for (int i = 1; i <= 3; ++i) {g.observe_clock(clock_stamp + i * .2, wall + i * .01);}
    const double now = wall + .03;
    g.observe_odom(clock_stamp, now, 0, 0, 0, 0);
    g.observe_joints(clock_stamp + .703323, now, 0, 0); graph(g, now); g.poll(now, now);
    CHECK(!g.clock_sync_pending && g.fault_reason == "SENSOR_STAMP_OUTSIDE_CLOCK_WINDOW");
  } else if (test == "nonzero_cannot_arm_during_pending") {
    auto g = warm(true); observed_mismatch(g); g.poll(fault_snapshot, fault_snapshot);
    CHECK(g.clock_sync_pending);
    CHECK(!g.command(command(.3), "source", fault_snapshot + .001, fault_snapshot + .001));
    CHECK(g.state == "FAULT" && g.fault_reason == "NONZERO_BEFORE_EXPLICIT_SESSION_ARMED");
  } else if (test == "waiting_source_transport_still_enforces_sync_deadline") {
    Guard g(geometry(), true, wall - 1.1, 120);
    for (int i = 0; i <= 22; ++i) {
      const double offset = (i - 22) * .05;
      const double now = wall + offset, stamp = clock_stamp + offset;
      g.observe_clock(stamp, now); g.observe_odom(stamp, now, 0, 0, 0, 0);
      g.observe_joints(stamp, now, 0, 0);
      g.observe_graph(now, false, "", "self", "plugin", "WAITING_FOR_SOURCE", true);
      g.poll(now, now);
    }
    CHECK(g.state == "ARMING" && g.input_gid.empty());
    observed_mismatch(g); g.poll(fault_snapshot, fault_snapshot);
    CHECK(g.clock_sync_pending && g.fault_reason.empty());
    CHECK(!g.observations_ready(fault_snapshot, fault_snapshot));
    CHECK(!g.command(command(), "source", fault_snapshot + .0001, fault_snapshot + .0001));
    CHECK(g.state == "ARMING" && g.fault_reason.empty());
    const double now = odom_receipt + .100001;
    g.poll(now, now);
    CHECK(g.fault_reason == "SENSOR_CLOCK_SYNC_TIMEOUT");
  } else if (test == "unarmed_zero_is_suppressed_during_pending") {
    auto g = warm(true); observed_mismatch(g); g.poll(fault_snapshot, fault_snapshot);
    CHECK(!g.command(command(), "source", fault_snapshot + .0001, fault_snapshot + .0001));
    CHECK(g.state == "ARMING" && g.clock_sync_pending && g.fault_reason.empty());
    recover(g);
    CHECK(g.state == "ARMING" && !g.observations_ready(clock_receipt, clock_receipt));
    for (int i = 0; i <= 21; ++i) {
      const double now = clock_receipt + .01 + i * .05;
      coherent(g, now, recovered_clock + .01 + i * .05, true); g.poll(now, now);
    }
    CHECK(g.state == "ARMING");
    const double now = clock_receipt + 1.061;
    CHECK(g.observations_ready(now, now));
    CHECK(g.command(command(), "source", now, now));
    CHECK(g.state == "ARMED");
  } else if (test == "wrong_per_message_gid_during_pending_faults") {
    auto g = warm(true); observed_mismatch(g); g.poll(fault_snapshot, fault_snapshot);
    CHECK(!g.command(command(), "other-source", fault_snapshot + .001, fault_snapshot + .001));
    CHECK(g.fault_reason == "PER_MESSAGE_PUBLISHER_GID_MISMATCH");
  } else if (test == "prepare_zero_still_faults_during_pending") {
    auto g = warm(); observed_mismatch(g); g.poll(fault_snapshot, fault_snapshot);
    CHECK(!g.command(command(), "source", fault_snapshot + .001, fault_snapshot + .001));
    CHECK(g.fault_reason == "UNEXPECTED_COMMAND_IN_READ_ONLY_PREPARATION");
  } else if (test == "armed_poll_mismatch_faults_immediately") {
    auto g = warm(true, true); CHECK(g.state == "ARMED");
    observed_mismatch(g); g.poll(fault_snapshot, fault_snapshot);
    CHECK(g.fault_reason == "SENSOR_STAMP_OUTSIDE_CLOCK_WINDOW" && !g.clock_sync_pending);
  } else if (test == "armed_command_mismatch_faults_immediately") {
    auto g = warm(true, true); observed_mismatch(g);
    CHECK(!g.command(command(.3), "source", fault_snapshot, fault_snapshot));
    CHECK(g.fault_reason == "SENSOR_STAMP_OUTSIDE_CLOCK_WINDOW" && !g.clock_sync_pending);
  } else if (test == "tolerances_and_watchdogs_remain_unchanged") {
    auto g = warm();
    CHECK(g.limits.future_stamp_tolerance == .1 && g.limits.clock_sync_timeout == .1);
    CHECK(g.limits.observation_timeout == .5 && g.limits.clock_stall == 1.0);
    CHECK(g.limits.lease_timeout == .5 && g.limits.command_timeout == .25);
    CHECK(g.limits.standstill_dwell == 1.0 && g.limits.speed == .5);
    g.observe_odom(clock_stamp + .099, odom_receipt, 0, 0, 0, 0);
    g.observe_joints(clock_stamp + .099, joint_receipt, 0, 0);
    g.poll(fault_snapshot, fault_snapshot);
    CHECK(!g.clock_sync_pending && g.fault_reason.empty());
    g.observe_odom(clock_stamp + .100001, fault_snapshot + .001, 0, 0, 0, 0);
    g.poll(fault_snapshot + .001, fault_snapshot + .001);
    CHECK(g.clock_sync_pending && !g.observations_ready(fault_snapshot + .001, fault_snapshot + .001));
  } else {return 2;}
  std::cout << "PASS " << test << "\n";
  return 0;
}
'''

CASES = (
    "observed_103323ms_is_pending_not_ready",
    "legitimate_clock_recovery_restarts_dwell",
    "recovery_requires_simulation_dwell_too",
    "pending_samples_cannot_accrue_recovery_dwell",
    "fresh_callbacks_do_not_renew_pending_deadline",
    "coherent_recovery_after_deadline_still_faults",
    "callback_age_is_counted_before_first_poll",
    "timeout_boundary_is_exclusive",
    "only_violating_sensor_anchors_deadline",
    "joints_first_anchors_deadline",
    "clock_drop_during_pending_is_not_deferred",
    "clock_frozen_during_pending_is_not_deferred",
    "frozen_odom_during_pending_is_not_deferred",
    "frozen_joints_during_pending_is_not_deferred",
    "clock_regression_during_pending_faults",
    "monotonic_regression_during_pending_faults",
    "odom_regression_during_pending_faults",
    "joint_regression_during_pending_faults",
    "lease_loss_during_pending_faults",
    "graph_violation_during_pending_faults",
    "graph_staleness_during_pending_faults",
    "destination_gid_change_during_pending_faults",
    "source_gid_change_during_pending_faults",
    "read_only_roll_during_pending_faults",
    "old_sensor_is_not_reconciliation_candidate",
    "future_sensor_cannot_mask_an_old_sensor",
    "nonzero_cannot_arm_during_pending",
    "waiting_source_transport_still_enforces_sync_deadline",
    "unarmed_zero_is_suppressed_during_pending",
    "wrong_per_message_gid_during_pending_faults",
    "prepare_zero_still_faults_during_pending",
    "armed_poll_mismatch_faults_immediately",
    "armed_command_mismatch_faults_immediately",
    "tolerances_and_watchdogs_remain_unchanged",
)


class CommissioningClockSyncTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("g++")
        if compiler is None:
            raise unittest.SkipTest("g++ is required to exercise the actual C++ Guard")
        directory = tempfile.TemporaryDirectory(prefix="commissioning-clock-sync-")
        cls.addClassCleanup(directory.cleanup)
        source = Path(directory.name) / "clock_sync.cpp"
        cls.binary = Path(directory.name) / "clock_sync"
        source.write_text(SOURCE, encoding="utf-8")
        compiled = subprocess.run(
            [compiler, "-std=c++17", "-Wall", "-Wextra", "-Wpedantic", "-Werror",
             "-I", str(ROOT / "tools/auto_tuning/commissioning_adapter"),
             str(source), "-o", str(cls.binary)],
            capture_output=True, text=True, timeout=60, check=False,
        )
        if compiled.returncode != 0:
            raise AssertionError(compiled.stdout + compiled.stderr)


def _case(name):
    def test(self):
        completed = subprocess.run(
            [str(self.binary), name], capture_output=True, text=True, timeout=10, check=False,
        )
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
        self.assertEqual(completed.stdout.strip(), "PASS " + name)
    return test


for _name in CASES:
    setattr(CommissioningClockSyncTests, "test_" + _name, _case(_name))


class ReadOnlyClockCompletionTests(unittest.TestCase):
    @staticmethod
    def ready_record():
        return {
            "state": "READY_READ_ONLY", "observations_ready": True,
            "clock_sync_pending": False, "clock_sync_episodes": 1,
            "clock_sync_recoveries": 1,
        }

    def assert_rejected(self, record):
        self.assertFalse(session.read_only_adapter_ready(record))
        with self.assertRaisesRegex(RuntimeError, "PREPARATION_ENDED_WITHOUT_COHERENT_OBSERVATIONS"):
            session.require_read_only_completion(record)

    def test_coherent_requalified_record_can_finish(self):
        record = self.ready_record()
        self.assertTrue(session.read_only_adapter_ready(record))
        self.assertIsNone(session.require_read_only_completion(record))

    def test_ready_phase_cannot_mask_pending_clock(self):
        record = self.ready_record()
        record["clock_sync_pending"] = True
        self.assert_rejected(record)

    def test_clock_recovery_alone_cannot_bypass_standstill_requalification(self):
        record = self.ready_record()
        record["observations_ready"] = False
        self.assert_rejected(record)

    def test_missing_evidence_never_defaults_to_ready(self):
        self.assert_rejected({})
        for key in ("state", "observations_ready", "clock_sync_pending"):
            with self.subTest(missing=key):
                record = self.ready_record()
                del record[key]
                self.assert_rejected(record)

    def test_boolean_evidence_requires_actual_booleans(self):
        for key, values in (("observations_ready", (1, "true", None)),
                            ("clock_sync_pending", (0, "false", None))):
            for value in values:
                with self.subTest(key=key, value=value):
                    record = self.ready_record()
                    record[key] = value
                    self.assert_rejected(record)

    def test_other_adapter_states_cannot_finish_read_only(self):
        for state in ("ARMING", "ARMED", "FAULT", None):
            with self.subTest(state=state):
                record = self.ready_record()
                record["state"] = state
                self.assert_rejected(record)


if __name__ == "__main__":
    unittest.main()
