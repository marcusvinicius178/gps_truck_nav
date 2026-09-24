# truck_bringup

Vehicle models, Gazebo Classic worlds, GPS/localization support and Navigation2
launch/configuration artifacts for an 8×4 Ackermann research truck.

Use the [repository README](../../../README.md) for requirements, exact build
commands, validation limits and the GPS waypoint procedure.

| Launch | Role |
| --- | --- |
| `gps_waypoint_follower.launch.py` | Recommended full integration entry point: Gazebo once, dual EKF/navsat, Nav2, optional RViz/Mapviz; selects CTVI |
| `truck_simulation_launch.py` | Gazebo and robot state only; selects historical Sonoma empty ground; no EKF/Nav2 |
| `dual_ekf_navsat.launch.py` | Local/global EKF and `navsat_transform` |
| `navigation_launch.py` | Navigation2 servers/lifecycle; needs localization and a robot |
| `rviz_launch.py` | Optional `truck.rviz` visualization |
| `mapviz.launch.py` | Optional Mapviz overlays/local tile configuration |

The full launch already includes simulation; do not launch both independently.
CMake installs `logged_waypoint_follower.py`, `gps_waypoint_logger.py` and the
experimental `follow_dynamic_gps_wps_lanes.py` with their `.py` suffixes. The Python
module `truck_bringup` is installed from `scripts/`; there is no active setup.py.

The current waypoint file is CTVI-based despite its `empty_world_gps_wps.yaml`
name. The retained parameters are scientific snapshots, not complete run dumps.
Read the root runtime limitations before attempting navigation; no live simulation
success is claimed by this documentation.
