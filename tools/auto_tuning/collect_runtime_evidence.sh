#!/usr/bin/env bash
# Read-only bounded probes. Run INSIDE the sourced experiment container.
# This collector does not approve readiness, start a mission or pass a gate.
set -Eeuo pipefail
if [[ $# -ne 1 ]]; then
  echo 'Usage: collect_runtime_evidence.sh NEW_OUTPUT_DIRECTORY' >&2
  exit 2
fi
for required in ros2 timeout python3; do
  command -v "$required" >/dev/null || { echo "Missing: $required" >&2; exit 2; }
done
output_dir=$1
mkdir "$output_dir"  # Fail on existing output; never replace earlier evidence.
probe() {
  local name=$1
  shift
  local status=0
  timeout --signal=INT --kill-after=3s 15s "$@" >"$output_dir/$name.log" 2>&1 || status=$?
  printf '%s\t%s\n' "$name" "$status" >>"$output_dir/exit_codes.tsv"
}
probe node_list ros2 node list --no-daemon
probe topic_list ros2 topic list -t
probe action_list ros2 action list -t
probe service_list ros2 service list -t
for node in bt_navigator planner_server controller_server behavior_server smoother_server velocity_smoother waypoint_follower; do
  probe "lifecycle_$node" ros2 lifecycle get "/$node"
  probe "parameters_$node" ros2 param dump "/$node"
done
probe parameters_global_costmap ros2 param dump /global_costmap/global_costmap
probe parameters_local_costmap ros2 param dump /local_costmap/local_costmap
probe clock ros2 topic echo /clock --once
probe gps ros2 topic echo /gps/fix --once
probe gps_alternative ros2 topic echo /gps/data --once
probe lidar ros2 topic echo /sick_scan_2D --once
probe odometry_global ros2 topic echo /odometry/global --once
probe odometry_gps ros2 topic echo /odometry/gps --once
probe odometry_local ros2 topic echo /odometry/local --once
probe odometry_raw ros2 topic echo /odom --once
probe imu_raw ros2 topic echo /imu/data --once
probe imu_remapped ros2 topic echo /imu --once
probe tf_static ros2 topic echo /tf_static --once --qos-durability transient_local --qos-reliability reliable
probe tf_map_base ros2 run tf2_ros tf2_echo map base_footprint
probe tf_odom_base ros2 run tf2_ros tf2_echo odom base_footprint
probe tf_base_cloud ros2 run tf2_ros tf2_echo base_link cloud
probe tf_publishers ros2 topic info /tf --verbose
probe tf_static_publishers ros2 topic info /tf_static --verbose
probe from_ll ros2 service type /fromLL
readiness_status=0
timeout --signal=INT --kill-after=5s 75s \
  python3 /opt/auto_tuning/tools/probe_readiness.py "$output_dir" --duration 30 \
  >"$output_dir/readiness_collector.log" 2>&1 || readiness_status=$?
printf 'readiness_collector\t%s\n' "$readiness_status" >>"$output_dir/exit_codes.tsv"
python3 - "$output_dir" <<'PY'
import datetime, json, os, pathlib, sys
out = pathlib.Path(sys.argv[1])
result = {
    'collected_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
    'evidence_status': 'OBSERVATIONS_ONLY_REQUIRES_REVIEW',
    'gate0': 'NOT_YET_VERIFIED', 'gate1': 'NOT_YET_VERIFIED',
    'readiness_report': 'readiness.json',
    'mission_ready': False,
    'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
    'ros_localhost_only': os.environ.get('ROS_LOCALHOST_ONLY'),
    'notes': [
        'tf2_echo probes normally reach the time limit; inspect their samples.',
        'A single TF message does not prove unique authority or freshness.',
        'Check repeated advancing clock, fresh sensor/TF timestamps, covariance, pose and all active lifecycle nodes before any mission.',
        'Parameter dumps show declared values; they do not alone prove plugin runtime effect.',
        'No mission, recording evaluation, reset test or gate approval is performed.'
    ]
}
(out / 'collection.json').write_text(json.dumps(result, indent=2) + '\n')
PY
printf 'Evidence collected in %s; readiness and gates are NOT verified.\n' "$output_dir"
exit "$readiness_status"
