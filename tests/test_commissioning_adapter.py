"""Compile and execute the actual ROS-independent C++ commissioning guard.

No ROS, DDS, Docker, simulator, Twist publication or movement occurs here.
The Iron wrapper itself is compiled by the host's Docker image build.
"""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = r'''
#include "guard.hpp"
#include "self_test.hpp"
#include "lease_snapshot.hpp"
#include <iostream>
#include <string>
#include <limits>
using namespace commissioning;
#define CHECK(x) do {if (!(x)) {std::cerr << __LINE__ << ": " << #x << "\n"; return 1;}} while(0)

Geometry geometry() {
  return {6.816500073351341, 2.598120190224205, -1.299, 20,
    .4797896546711007, -.524, .524, -.524, .524};
}
std::array<double,6> command(double v=0, double w=0) {return {v,0,0,0,0,w};}
void feed(Guard & g, double now, bool execute=true, double v=0, double w=0) {
  g.observe_clock(10+now, now);
  g.observe_odom(10+now, now, v, w, 0, 0);
  g.observe_joints(10+now, now, 0, 0);
  g.observe_graph(now, true, execute?"source":"", execute?"self":"", "plugin", "");
}
Guard warm(bool execute=true) {
  Guard g(geometry(), execute, 0, 120);
  for (int i=0; i<=21; ++i) {
    const double t=i*.05;
    feed(g,t,execute);
    if(execute) {g.command(command(), "source", t, t);}
    g.poll(t,t);
  }
  return g;
}
bool near(double a,double b) {return std::abs(a-b)<1e-12;}
int main(int argc,char**argv) {
  if(argc!=2) return 2;
  const std::string test=argv[1];
  if(test=="straight") {
    auto r=convert(command(.3,0),geometry());
    CHECK(r.status=="ACCEPTED"); CHECK(near(r.plugin_speed,.3)); CHECK(r.plugin_angle==0);
  } else if(test=="left_right_curves") {
    for(double w:{-.003,.003}) {
      auto r=convert(command(.3,w),geometry());
      CHECK(r.status=="ACCEPTED"); CHECK(near(r.plugin_speed,.3-w*(-1.299)));
      CHECK(near(std::tan(r.plugin_angle)/geometry().wheelbase,w/.3));
      CHECK(near(r.curvature,w/.3)); CHECK(r.left_target*w>0 && r.right_target*w>0);
      if(w>0) CHECK(r.left_target>r.right_target); else CHECK(r.left_target>r.right_target);
    }
  } else if(test=="zero_and_negative_zero") {
    for(double z:{0.0,-0.0}) {
      auto r=convert(command(z,z),geometry()); CHECK(r.status=="STOP");
      CHECK(r.plugin_speed==0 && r.plugin_angle==0 && !std::signbit(r.plugin_speed));
    }
  } else if(test=="near_zero_yaw_rejected") {
    for(double v:{0.0,.001,.01}) CHECK(convert(command(v,.001),geometry()).reason=="UNREALIZABLE_NEAR_ZERO_YAW");
  } else if(test=="reverse_disabled") {
    for(double w:{-.003,0.0,.003}) CHECK(convert(command(-.3,w),geometry()).reason=="REVERSE_DISABLED_FOR_FIRST_COMMISSIONING");
  } else if(test=="finite_all_six_components") {
    for(int j=0;j<6;++j) {
      auto c=command(.3,0); c[j]=std::numeric_limits<double>::quiet_NaN();
      CHECK(convert(c,geometry()).reason=="NONFINITE_COMMAND");
      c[j]=std::numeric_limits<double>::infinity(); CHECK(convert(c,geometry()).reason=="NONFINITE_COMMAND");
    }
  } else if(test=="unsupported_components") {
    for(int j=1;j<5;++j) {auto c=command(.3,0); c[j]=.001;
      CHECK(convert(c,geometry()).reason=="UNSUPPORTED_TWIST_COMPONENT");}
  } else if(test=="overspeed_is_rejected_not_clipped") {
    auto r=convert(command(.50001,0),geometry());
    CHECK(r.status=="REJECTED"); CHECK(r.plugin_speed==0); CHECK(r.reason=="COMMISSIONING_SPEED_OR_YAW_LIMIT");
  } else if(test=="over_yaw_is_rejected") {
    CHECK(convert(command(.5,.020001),geometry()).reason=="COMMISSIONING_SPEED_OR_YAW_LIMIT");
  } else if(test=="curvature_is_rejected_not_projected") {
    CHECK(convert(command(.021,.02),geometry()).reason=="PHYSICAL_STEERING_ENVELOPE");
    CHECK(convert(command(.3,.01),geometry()).reason=="COMMISSIONING_STEERING_ENVELOPE");
  } else if(test=="right_rear_limit_independent") {
    auto g=geometry(); g.plugin_speed=.301;
    CHECK(convert(command(.3,.003),g).reason=="INVALID_OR_LIMITED_RIGHT_REAR_TARGET");
    CHECK(convert(command(.3,-.003),g).status=="ACCEPTED");
  } else if(test=="hysteresis") {
    CHECK(convert(command(.015,0),geometry(),Limits{},true).status=="HOLD");
    CHECK(convert(command(.02,0),geometry(),Limits{},true).status=="ACCEPTED");
    CHECK(convert(command(.015,0),geometry(),Limits{},false).status=="ACCEPTED");
  } else if(test=="invalid_geometry") {
    auto g=geometry(); g.wheelbase=0; bool threw=false;
    try {convert(command(.3,0),g);} catch(const std::invalid_argument&) {threw=true;} CHECK(threw);
    g=geometry(); g.rear_right_y=1; threw=false;
    try {Guard bad(g,false,0,120);} catch(const std::invalid_argument&) {threw=true;} CHECK(threw);
  } else if(test=="prepare_ready_has_no_commands") {
    auto g=warm(false); CHECK(g.state=="READY_READ_ONLY"); CHECK(g.command_count==0);
    CHECK(g.observations_ready(1.05,1.05)); CHECK(g.standstill_observed(1.05));
  } else if(test=="prepare_any_command_faults") {
    auto g=warm(false); CHECK(!g.command(command(),"source",1.1,1.1));
    CHECK(g.fault_reason=="UNEXPECTED_COMMAND_IN_READ_ONLY_PREPARATION");
  } else if(test=="standstill_dwell") {
    Guard g(geometry(),true,0,120); feed(g,0); g.command(command(),"source",0,0);
    CHECK(g.state=="ARMING"); CHECK(!g.standstill_observed(0));
    for(int i=1;i<20;++i) {double t=i*.05; feed(g,t);g.command(command(),"source",t,t);g.poll(t,t);}
    CHECK(g.state=="ARMING");
    feed(g,1.0); CHECK(g.command(command(),"source",1,1)); CHECK(g.state=="ARMED");
  } else if(test=="missing_source_ready_for_operator") {
    Guard g(geometry(),true,0,120);
    for(int i=0;i<=21;++i) {double t=i*.05;
      g.observe_clock(10+t,t);g.observe_odom(10+t,t,0,0,0,0);g.observe_joints(10+t,t,0,0);
      g.observe_graph(t,false,"","self","plugin","SOURCE_PENDING",true);g.poll(t,t);}
    CHECK(g.state=="ARMING"); CHECK(g.observations_ready(1.05,1.05));
    CHECK(!g.command(command(.3,0),"source",1.06,1.06)); CHECK(g.state=="FAULT");
  } else if(test=="nonzero_before_arm") {
    Guard g(geometry(),true,0,120);feed(g,0);
    CHECK(!g.command(command(.3,0),"source",0,0));
    CHECK(g.fault_reason=="NONZERO_BEFORE_EXPLICIT_SESSION_ARMED");
  } else if(test=="message_gid_must_match") {
    auto g=warm(); feed(g,1.1);
    CHECK(!g.command(command(.3,0),"rogue",1.1,1.1));
    CHECK(g.fault_reason=="PER_MESSAGE_PUBLISHER_GID_MISMATCH");
  } else if(test=="graph_restart_faults") {
    auto g=warm();g.observe_graph(1.1,true,"newsource","self","plugin","");
    CHECK(g.state=="FAULT"); CHECK(g.fault_reason=="PINNED_GRAPH_IDENTITY_CHANGED");
  } else if(test=="destination_restart_faults") {
    auto g=warm();g.observe_graph(1.1,true,"source","self","otherplugin","");
    CHECK(g.fault_reason=="PINNED_DESTINATION_IDENTITY_CHANGED");
  } else if(test=="graph_conflict_faults") {
    auto g=warm();g.observe_graph(1.1,false,"","","","EXTRA_PUBLISHER");
    CHECK(g.state=="FAULT"); CHECK(g.fault_reason=="EXTRA_PUBLISHER");
  } else if(test=="command_watchdog_boundary") {
    auto g=warm();
    for(int i=22;i<=26;++i) {double t=i*.05;feed(g,t);g.poll(t,t);}
    CHECK(g.state=="FAULT");CHECK(g.fault_reason=="COMMAND_WATCHDOG_EXPIRED");
  } else if(test=="late_command_cannot_revive") {
    auto g=warm();
    for(int i=22;i<=26;++i) {double t=i*.05;feed(g,t);}
    CHECK(!g.command(command(.3,0),"source",1.30,1.30));CHECK(g.state=="FAULT");
    feed(g,1.35);CHECK(!g.command(command(),"source",1.35,1.35));CHECK(g.state=="FAULT");
  } else if(test=="lease_boundary") {
    auto g=warm();feed(g,1.1);g.poll(1.1,.6);
    CHECK(g.state=="FAULT");CHECK(g.fault_reason=="SUPERVISOR_LEASE_EXPIRED");
  } else if(test=="future_lease") {
    auto g=warm();g.poll(1.1,1.1001);CHECK(g.fault_reason=="SUPERVISOR_LEASE_EXPIRED");
  } else if(test=="clock_regression") {
    auto g=warm();g.observe_clock(10,1.1);CHECK(g.fault_reason=="INVALID_OR_REGRESSED_SIM_CLOCK");
  } else if(test=="clock_forward_jump") {
    auto g=warm();g.observe_clock(20,1.1);CHECK(g.fault_reason=="SIM_CLOCK_JUMP_EXCEEDS_COMMISSIONING_WINDOW");
  } else if(test=="stale_observation_faults") {
    auto g=warm();g.poll(1.55,1.55);CHECK(g.state=="FAULT");CHECK(g.fault_reason=="CLOCK_RECEIPT_STALE");
  } else if(test=="frozen_odom_stamp_does_not_refresh") {
    auto g=warm();
    for(int i=22;i<=32 && g.state!="FAULT";++i) {double t=i*.05;
      g.observe_clock(10+t,t);g.observe_odom(11.05,t,0,0,0,0);g.observe_joints(10+t,t,0,0);
      g.observe_graph(t,true,"source","self","plugin","");g.command(command(),"source",t,t);g.poll(t,t);}
    CHECK(g.state=="FAULT");CHECK(g.fault_reason=="ODOMETRY_STALE_OR_FROZEN");
  } else if(test=="sensor_future_reorder_window") {
    auto g=warm();feed(g,1.1);g.observe_odom(11.19,1.1,0,0,0,0);
    CHECK(g.health(1.1,1.1).empty());
    g.observe_odom(11.201,1.1,0,0,0,0);
    CHECK(g.health(1.1,1.1)=="SENSOR_STAMP_OUTSIDE_CLOCK_WINDOW");
  } else if(test=="overspeed_immediate_during_startup") {
    Guard g(geometry(),true,0,120);g.observe_odom(10,0,.7001,0,0,0);
    CHECK(g.state=="FAULT");CHECK(g.fault_reason=="OBSERVED_MOTION_ENVELOPE_EXCEEDED");
  } else if(test=="steering_overrun_immediate") {
    Guard g(geometry(),true,0,120);g.observe_joints(10,0,.251,0);
    CHECK(g.state=="FAULT");CHECK(g.fault_reason=="OBSERVED_STEERING_ENVELOPE_EXCEEDED");
  } else if(test=="distance_bound") {
    auto g=warm();g.observe_odom(11.1,1.1,.3,0,10.01,0);
    CHECK(g.state=="FAULT");CHECK(g.fault_reason=="TRAVEL_OR_DISPLACEMENT_LIMIT");
  } else if(test=="monotonic_regression") {
    auto g=warm();g.poll(.99,.99);CHECK(g.fault_reason=="MONOTONIC_CLOCK_REGRESSED");
  } else if(test=="wall_bound") {
    auto g=warm();g.poll(120,120);CHECK(g.fault_reason=="WALL_DURATION_LIMIT");
  } else if(test=="observed_stop_requires_fresh_dwell") {
    auto g=warm();feed(g,1.1,true,.3,.003);
    CHECK(!g.standstill_observed(1.1));CHECK(g.command(command(),"source",1.1,1.1));
    for(int i=23;i<=41;++i) {double t=i*.05;feed(g,t);g.command(command(),"source",t,t);}
    CHECK(!g.standstill_observed(2.05));
    feed(g,2.2);g.command(command(),"source",2.2,2.2);
    CHECK(g.standstill_observed(2.2));CHECK(!g.standstill_observed(2.8));
  } else if(test=="standstill_gap_resets_dwell") {
    auto g=warm(false);g.observe_odom(12,2,0,0,0,0);
    CHECK(!g.standstill_observed(2));
  } else if(test=="read_only_unexpected_roll") {
    auto g=warm(false);feed(g,1.1,false,.02,0);g.poll(1.1,1.1);
    CHECK(g.fault_reason=="UNEXPECTED_MOVEMENT_IN_READ_ONLY_PREPARATION");
  } else if(test=="accepted_stop_does_not_claim_physical_stop") {
    auto g=warm();feed(g,1.1,true,.3,0);
    CHECK(g.command(command(.3,0),"source",1.1,1.1));
    feed(g,1.15,true,.3,0);CHECK(g.command(command(),"source",1.15,1.15));
    CHECK(g.last.status=="STOP");CHECK(g.last.plugin_speed==0);CHECK(!g.standstill_observed(1.15));
  } else if(test=="image_smoke_core") {
    CHECK(self_test());
  } else if(test=="lateral_slide_is_not_standstill") {
    auto g=warm();g.observe_odom(11.1,1.1,0,0,0,0,.02);
    CHECK(!g.standstill_observed(1.1));
    g.observe_odom(11.15,1.15,0,0,0,0,.71);
    CHECK(g.fault_reason=="OBSERVED_MOTION_ENVELOPE_EXCEEDED");
  } else if(test=="lease_renewal_during_read_is_not_future") {
    double clock=1.0;
    auto sample=lease_snapshot([&](){clock=1.1;return 1.1;},[&](){return clock;});
    CHECK(fresh(sample.lease,sample.now,.5));
    sample=lease_snapshot([&](){return 1.2;},[&](){return clock;});
    CHECK(!fresh(sample.lease,sample.now,.5));
  } else {std::cerr<<"Unknown case "<<test<<"\n";return 2;}
  std::cout<<"PASS "<<test<<"\n";return 0;
}
'''


CASES = (
    'straight', 'left_right_curves', 'zero_and_negative_zero',
    'near_zero_yaw_rejected', 'reverse_disabled', 'finite_all_six_components',
    'unsupported_components', 'overspeed_is_rejected_not_clipped',
    'over_yaw_is_rejected', 'curvature_is_rejected_not_projected',
    'right_rear_limit_independent', 'hysteresis', 'invalid_geometry',
    'prepare_ready_has_no_commands', 'prepare_any_command_faults',
    'standstill_dwell', 'missing_source_ready_for_operator', 'nonzero_before_arm',
    'message_gid_must_match', 'graph_restart_faults', 'destination_restart_faults',
    'graph_conflict_faults', 'command_watchdog_boundary', 'late_command_cannot_revive',
    'lease_boundary', 'future_lease', 'clock_regression', 'clock_forward_jump',
    'stale_observation_faults', 'frozen_odom_stamp_does_not_refresh',
    'sensor_future_reorder_window', 'overspeed_immediate_during_startup',
    'steering_overrun_immediate', 'distance_bound', 'monotonic_regression',
    'wall_bound', 'observed_stop_requires_fresh_dwell', 'standstill_gap_resets_dwell',
    'read_only_unexpected_roll', 'accepted_stop_does_not_claim_physical_stop',
    'image_smoke_core',
    'lateral_slide_is_not_standstill',
    'lease_renewal_during_read_is_not_future',
)


class CommissioningGuardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which('g++')
        if not compiler:
            raise unittest.SkipTest('g++ is required to execute the actual C++ core')
        cls.directory = tempfile.TemporaryDirectory(prefix='commissioning-core.')
        cls.addClassCleanup(cls.directory.cleanup)
        directory = Path(cls.directory.name)
        source = directory / 'test_guard.cpp'
        source.write_text(SOURCE)
        cls.executable = directory / 'test_guard'
        result = subprocess.run(
            [compiler, '-std=c++17', '-Wall', '-Wextra', '-Wpedantic', '-Werror',
             '-I', str(ROOT / 'tools/auto_tuning/commissioning_adapter'),
             str(source), '-o', str(cls.executable)],
            capture_output=True, text=True, timeout=60, check=False)
        if result.returncode:
            raise AssertionError(result.stdout + result.stderr)


def case(name):
    def run(self):
        result = subprocess.run([str(self.executable), name], capture_output=True,
                                text=True, timeout=10, check=False)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('PASS ' + name, result.stdout)
    return run


for _name in CASES:
    setattr(CommissioningGuardTests, 'test_' + _name, case(_name))


if __name__ == '__main__':
    unittest.main()
