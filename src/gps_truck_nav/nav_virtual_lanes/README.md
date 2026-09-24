# nav_virtual_lanes — experimental

Separate lane visualization, occupancy detection and costmap prototypes. Not
started by the full GPS Quick Start and not presented as the manuscript benchmark.

CMake installs `lane_costmap_generator.py`, `visualize_lanes_from_gps.py`,
`lane_occupancy_detector.py`, `visualize_lanes_from_gps_and_wps.py` and
`wall_costmap_generator.py`. `visualize_gps_waypoints.py` and `visualize_lanes.py`
are retained visualization variants, not installed commands.

See the [root usage map](../../../REPOSITORY_CLEANUP_AUDIT.md) and
[limitations](../../../docs/KNOWN_LIMITATIONS.md), especially the yaw-unit and
Marker/MarkerArray differences. No end-to-end lane workflow was validated.
