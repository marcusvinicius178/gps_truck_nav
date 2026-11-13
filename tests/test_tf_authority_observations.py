"""Compile/test the C++ observation core without pretending to build rclcpp."""

import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT / 'tools/auto_tuning/tf_probe'
FIXTURE = r'''
#include <iostream>
#include <limits>
#include "observations.hpp"
int main() {
  using namespace phase_a_tf;
  Observations observations;
  const std::array<unsigned char, 4> gid_a{{1, 2, 15, 255}};
  const std::array<unsigned char, 4> gid_b{{2, 2, 15, 255}};
  const std::array<unsigned char, 4> missing{{0, 0, 0, 0}};
  const auto a = gid_hex(gid_a.begin(), gid_a.end());
  const auto b = gid_hex(gid_b.begin(), gid_b.end());
  const std::array<double, 7> identity{{1.25, -2.5, 3.0, 0.0, 0.0, 0.0, 1.0}};
  observations.observe_clock({10, 0}, 0.1);
  observations.observe_clock({11, 0}, 1.0);
  observations.observe_clock({12, 0}, 2.0);
  observations.observe("/tf", "map", "odom", a, {10, 1}, identity, 0.2, 101, 102);
  observations.observe("/tf", "map", "odom", a, {11, 2}, identity, 1.4, 201, 202);
  observations.observe("/tf", "map", "odom", b, {12, 3}, identity, 2.1, 301, 302);
  observations.observe("/tf", "other_parent", "odom", b, {12, 4}, identity);
  observations.observe("/tf_static", "map", "odom", b, {0, 0}, identity);
  observations.observe("/tf", "odom", "base_footprint",
    gid_hex(missing.begin(), missing.end()), {13, 4}, identity);
  auto nonfinite = identity;
  nonfinite[0] = std::numeric_limits<double>::infinity();
  nonfinite[6] = std::numeric_limits<double>::quiet_NaN();
  observations.observe("/tf_static", "base_footprint", "quoted\"child\n", a, {0, 0}, nonfinite);
  observations.endpoint("/tf", a, "ekf_global", "/", "{\"reliability\":1,\"durability\":2,\"history\":1,\"depth\":100}");
  observations.endpoint("/tf", a, "ekf_global", "/", "{\"reliability\":1,\"durability\":2,\"history\":1,\"depth\":100}");
  observations.endpoint("/tf", b, "another\npublisher", "/test");
  observations.discovery_error("discovery \"error\"\n");
  observations.loss_event_support("/tf", true);
  observations.loss_event("/tf", 4, 4);
  observations.loss_event_support("/clock", false, "unsupported");
  observations.finish_window(3.0);
  std::cout << observations.json(true, 30, 30.02, 100, 130.02) << '\n';
  std::cout << Observations().json(false, 30, 0.2, 100, 100.2, "interrupted") << '\n';
  Samples sequence;
  for (int i = 0; i < 40; ++i) {sequence.observe({i, 0}, i / 2.0);}
  sequence.observe({39, 0}, 20.0);
  sequence.observe({38, 0}, 20.5);
  std::cout << sequence.json() << '\n';
}
'''


class TfAuthorityObservationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which('g++')
        if not compiler:
            raise unittest.SkipTest('g++ unavailable; ROS-independent C++ core not compiled')
        cls.temp = tempfile.TemporaryDirectory()
        directory = Path(cls.temp.name)
        source = directory / 'fixture.cpp'
        binary = directory / 'fixture'
        source.write_text(FIXTURE)
        subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Wpedantic', '-Werror',
                        '-I', str(CORE), str(source), '-o', str(binary)],
                       check=True, capture_output=True, text=True, timeout=60)
        result = subprocess.run([str(binary)], check=True, capture_output=True, text=True, timeout=5)
        cls.complete, cls.interrupted, cls.sequence = [json.loads(line) for line in result.stdout.splitlines()]
        cls.edges = {(edge['topic'], edge['parent'], edge['child']): edge
                     for edge in cls.complete['edges']}

    @classmethod
    def tearDownClass(cls):
        if hasattr(cls, 'temp'):
            cls.temp.cleanup()

    def test_repeated_messages_count_samples_but_preserve_distinct_authorities(self):
        edge = self.edges['/tf', 'map', 'odom']
        self.assertEqual(edge['count'], 3)
        self.assertEqual(edge['publisher_gids'], ['01020fff', '02020fff'])
        self.assertEqual(edge['first_stamp'], {'sec': 10, 'nanosec': 1})
        self.assertEqual(edge['last_stamp'], {'sec': 12, 'nanosec': 3})
        self.assertEqual(edge['last_transform']['translation']['x'], 1.25)

    def test_parent_conflicts_and_static_dynamic_edges_are_not_merged_away(self):
        self.assertIn(('/tf', 'other_parent', 'odom'), self.edges)
        self.assertIn(('/tf_static', 'map', 'odom'), self.edges)
        self.assertEqual(self.edges['/tf_static', 'map', 'odom']['count'], 1)

    def test_zero_gid_is_unknown_not_a_unique_authority(self):
        edge = self.edges['/tf', 'odom', 'base_footprint']
        self.assertEqual(edge['publisher_gids'], [])
        self.assertEqual(edge['publisher_gid_unknown_count'], 1)

    def test_nonfinite_transform_is_retained_as_null_with_failure_evidence(self):
        edge = self.edges['/tf_static', 'base_footprint', 'quoted"child\n']
        self.assertEqual(edge['nonfinite_transform_count'], 1)
        self.assertIsNone(edge['last_transform']['translation']['x'])
        self.assertIsNone(edge['last_transform']['rotation']['w'])

    def test_endpoint_metadata_is_deduplicated_and_json_escaped(self):
        self.assertEqual(len(self.complete['publishers']), 2)
        self.assertIn('another\npublisher', [p['node_name'] for p in self.complete['publishers']])
        self.assertEqual(self.complete['endpoint_discovery_errors'], ['discovery "error"\n'])
        endpoints = {p['node_name']: p for p in self.complete['publishers']}
        self.assertEqual(endpoints['ekf_global']['offered_qos'],
                         {'reliability': 1, 'durability': 2, 'history': 1, 'depth': 100})
        self.assertIsNone(endpoints['another\npublisher']['offered_qos'])

    def test_completion_never_certifies_health_or_gates_and_interrupt_is_explicit(self):
        self.assertTrue(self.complete['observation_complete'])
        self.assertFalse(self.complete['simulator_health_verified'])
        self.assertEqual(self.complete['gate0'], 'NOT_YET_VERIFIED')
        self.assertEqual(self.complete['gate1'], 'NOT_YET_VERIFIED')
        self.assertFalse(self.interrupted['observation_complete'])
        self.assertEqual(self.interrupted['error'], 'interrupted')
        self.assertEqual(self.interrupted['edges'], [])
        self.assertEqual(self.interrupted['publishers'], [])

    def test_clock_and_window_reference_are_observed_inside_the_same_process(self):
        self.assertEqual(self.complete['schema_version'], 2)
        self.assertEqual(self.complete['clock_reference'], 'OBSERVER_LOCAL_CLOCK_SAME_WINDOW')
        self.assertEqual(self.complete['observation_window_end_wall_offset'], 3.0)
        clock = self.complete['clock']
        self.assertEqual((clock['count'], clock['forward_steps'], clock['backward_steps']), (3, 2, 0))
        self.assertEqual(clock['last_stamp'], {'sec': 12, 'nanosec': 0})
        self.assertEqual(clock['last_advance_wall_offset'], 2.0)

    def test_arrival_jitter_and_source_intervals_are_separate_for_each_gid(self):
        sequences = {record['gid']: record['timing'] for record in
                     self.edges['/tf', 'map', 'odom']['publisher_observations']}
        self.assertEqual(sequences['01020fff']['count'], 2)
        self.assertEqual(sequences['02020fff']['count'], 1)
        self.assertAlmostEqual(sequences['01020fff']['reception_interval_wall_seconds']['mean'], 1.2)
        self.assertAlmostEqual(sequences['01020fff']['source_interval_sim_seconds']['mean'], 1.000000001)
        self.assertEqual(sequences['01020fff']['last_rmw_source_timestamp_ns'], 201)
        self.assertEqual(sequences['01020fff']['last_rmw_received_timestamp_ns'], 202)

    def test_sequences_retain_regressions_duplicates_and_bounded_trace(self):
        self.assertEqual(self.sequence['count'], 42)
        self.assertEqual(self.sequence['backward_steps'], 1)
        self.assertEqual(self.sequence['repeated_steps'], 1)
        self.assertEqual(self.sequence['forward_steps'], 39)
        self.assertEqual(len(self.sequence['trace']), 32)
        self.assertEqual(self.sequence['trace_truncated_count'], 10)
        self.assertEqual(self.sequence['source_interval_sim_seconds']['minimum'], -1.0)
        self.assertEqual(self.sequence['last_advance_wall_offset'], 19.5)

    def test_loss_callbacks_and_unsupported_events_are_explicit_not_delivery_proof(self):
        events = {record['topic']: record for record in self.complete['message_loss_events']}
        self.assertTrue(events['/tf']['supported'])
        self.assertEqual(events['/tf']['reported_total_count'], 4)
        self.assertFalse(events['/clock']['supported'])
        self.assertEqual(events['/clock']['error'], 'unsupported')
        self.assertTrue(any('zero events does not prove zero loss' in item for item in self.complete['limitations']))


if __name__ == '__main__':
    unittest.main()
