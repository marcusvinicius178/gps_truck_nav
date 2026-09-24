# Runtime and reproducibility limitations

This review used baseline `2eb40fd44b48baa31b31dcd1047ab89f8a128c3b`. No ROS 2
runtime, Gazebo session or completed Nav2 mission was observed. The full procedure
is source-checked. Do not represent these integration files as an exact runnable
reconstruction of all manuscript experiments.

## Retained runtime concerns requiring owner validation

| Location | Finding | Consequence / required check |
| --- | --- | --- |
| `truck_bringup/params/dual_ekf_navsat_params.yaml` | Both EKF nodes use `world_frame: map` and publish TF; the “odom” EKF consumes `/odom`, while the map EKF consumes `/odometry/local` | Check competing map→odom authorities and whether this snapshot produces a coherent tree. The usual independent odom/global split cannot be assumed. No frame or covariance retuning was made. |
| Same file, global EKF `odom1` | Input is `gps`, with velocity/yaw-rate selection; navsat launch publishes `odometry/gps` | No matching Odometry publisher on `gps` is established by the include graph. This is not proof of correct GNSS fusion. Review live types and intended historical wiring before changing the scientific snapshot. |
| `models/arocs_truck/model.sdf`, GPS plugin | Two `~/out` remaps, one to `fix` and one to `data`, within namespace `/gps` | Determine the actual resolved output. The README checks `/gps/fix`; if absent, inspect `/gps/data` and the plugin remap handling. The conflicting model remaps were preserved as model evidence. |
| `urdf/arocs_truck.urdf` and full launch | URDF `sick_joint` defines `base_link`→`cloud`; launch also broadcasts a static transform for that child, with its retained quaternion | Check duplicate TF publishers/alignment. Choosing one would alter sensor orientation, so neither was silently removed. |
| World, datum, model pose and sender | CTVI geographic origins differ; datum yaw differs from Gazebo pose yaw; sender adds a fixed +4.8025 m to map x | Same-site consistency is established, full frame/antenna/vehicle alignment is not. The translation was intentionally not changed into a rotated lever arm. |
| Nav2 obstacle sources | `/external_area_pointcloud` is configured, but the current full launch does not start a publisher | The retained `/sick_scan_2D` sensor exists. Check actual observation readiness; do not claim that the historical perception pipeline is complete. |
| Planner/controller snapshot | Active SMAC/MPPI settings include old/custom or potentially ignored parameters; private Nav2 fork is inaccessible | Upstream export names match by inspection, but parameter behavior/build compatibility has not been established experimentally. Do not drop parameters to obtain a convenient result. |
| `rviz/truck.rviz` | Retained visualization contains displays/topics from development | Some displays may have no publishers. This does not by itself mean the full Nav2 mission succeeded or failed. |
| Tk logger | Sensor subscriptions and initial default state need live-data verification | Confirm a real GPS fix and IMU update before writing waypoints. GUI functionality was not observed. |

Paths in this table are relative to `src/gps_truck_nav/`. These are deliberately
reported concerns, not assertions that every one was observed to cause a live
failure. Current ROS message/TF behavior must resolve them before this procedure
can be labeled runtime-verified.

## Explicit behavior-affecting infrastructure changes

- Full launch selects the retained `ctvi.world` to match canonical CTVI datum and
  waypoints. Simulation-only still selects `empty_ground.world`. No world file,
  spawn pose, heading, physics setting or waypoint coordinate was edited.
- The navsat launch remaps `imu/data` to `imu`, matching the model plugin and EKF
  sensor input. Previously it remapped the name to itself. This changes wiring,
  not filter parameters; it has not been used to recompute paper results.
- GUI processes are truly optional and the full launch creates one RViz and one
  Gazebo server. NVIDIA PRIME offload is opt-in. Container names and requested
  simulation time are propagated.
- The logged sender resolves its own package only when a default file is needed,
  converts once per waypoint, rejects an invalid/partial route and distinguishes
  failed/rejected/canceled actions from successful ones. The geometric offset
  and waypoint order are unchanged.
- Required C++ includes/dependencies and executable/module installation were
  repaired. These changes do not modify goal-checker equations or tolerances.

## Historical / experimental components

- Interactive click-to-drive waits for nonexistent `gps_localizer/get_state` and
  imports `nav2_gps_waypoint_follower_demo`; an older variant instead waits for
  AMCL. Neither is installed or advertised as current.
- The old farm launch needs private `offroad_sim`, external `laser_scan_integrator`
  and absent resources such as `set_datum.py` / `maps/infinite_map.yaml`.
- Segmentation references suffix-free commands that ament_cmake never installed;
  classifier code requires `ultralytics` and local model weights under an old
  `/home/rota_2024/...` path. It is not part of the core dependency recipe.
- `gps_datum_setter.py` only remembers a fix; it does not call the navsat datum
  service. The newer historical variant also uses unsupported printf-style
  positional logger arguments. It was not repurposed as a functioning datum setter.
- The declaration-only `WallInflationLayer` has no implementation or CMake target.
- Lane Python tools publish markers/costmap-related topics; the dynamic sender
  consumes `MarkerArray`, while the C++ planner consumes `Marker`. Several lane
  converters call `radians(wp['yaw'])`, unlike the logged sender's radians input.
  Combining these components requires explicit unit/topic verification.
- The experimental sender publishes `/cmd_vel` in addition to using Nav2. It must
  not compete with the logged sender in the core procedure.
- Other Gazebo worlds include unavailable external models, zero origins or, in
  `baylands.world`, duplicate spherical-coordinate elements (well-formed XML but
  not validated SDF semantics). They are retained research assets, not supported
  turnkey examples.

## Scientific limits

Configuration snapshots are not per-run runtime dumps. Seeds were not retained
for every simulation run, and this is not a seed-controlled Monte Carlo experiment.
Field N=1 per condition does not validate simulation rankings or planner-specific
safety. No numerical results, run inventories, thresholds, recordings or analysis
outputs were changed by this cleanup.
