# Owner review report — public research repository cleanup

## BRANCH

`paper-public-cleanup-2026-09`

Base: `2eb40fd44b48baa31b31dcd1047ab89f8a128c3b` on `master`.
No merge, force-push, history rewrite, benchmark recomputation or scientific
retuning was performed. The default branch was not edited. The cleanup should
not be described as an end-to-end validated navigation release.

## COMMITS

The substantive commits preceding this handoff document are:

```text
ea687e1c58c0748493218305afc6d894c687a611 security: remove committed map credentials and document history remediation
00747641ffe04ceb484fc0bb813babb12e4f87e5 refactor: establish canonical packages and isolate historical development variants
1c711de611c405aa10d8e167b43b2bb9c9179b5b docs: document reviewer workflow and full cleanup reachability audit
f2d249cc14df258a7102daf98e03bab2bfee0e25 docs: record license provenance, publication assets and validation limits
```

This report is added by the final `docs: add complete owner review handoff` commit.
Its immutable SHA is supplied in the completion message; the complete branch log
is available with `git log --reverse --oneline 2eb40fd..HEAD`.

## SECURITY

One Bing Maps credential value occurred in two baseline current-tree Mapviz
configs and in a historical plain key file. The first commit blanked both copies;
canonicalization then removed the redundant root config. The current canonical
Mapviz configurations contain empty values. Local settings/credentials are ignored.
Gitleaks 8.30.1 scanned the hydrated current tree (about 278.72 MB) and reported
zero findings. Independent filename/pattern review also covered the original
140 unique Git blobs. No secret value is included in these reports.

## HISTORY CLEANUP REQUIRED

The credential-bearing Mapviz blob occurs in all 20 baseline reachable commits,
under `src/truck_bringup/params/patio_mercedes.mvc`,
`truck_bringup/params/patio_mercedes.mvc` and
`src/gps_truck_nav/truck_bringup/params/patio_mercedes.mvc`.
`src/truck_bringup/bing_key.txt` occurs in initial commit
`43f74dc88c6f802344ada4470d34b1756f857efd` and contains the same credential value.
Gitleaks identified two introduction findings; the manual review additionally
identified the plain key file.

The default branch/history still contains the credential. Owner revocation or
rotation is required to invalidate old copies. Optional isolated `git-filter-repo`
instructions are in [SECURITY_AUDIT.md](SECURITY_AUDIT.md); no rewrite or remote
history replacement was executed.

## SIMULATION QUICK START

The exact README sequence is repeated here. It is source-checked and depends on
the installation steps in [README.md](../README.md). It has not been executed in
ROS/Gazebo in this environment.


Complete the native build above. All three terminals must use the same ROS
distribution, installed workspace and DDS domain. These are the exact source-checked
commands; live execution remains unverified. The full launch chooses **CTVI** to
match its retained datum and waypoint file. It includes Gazebo, both EKFs,
`navsat_transform`, Nav2 and optional displays. **Do not start
`truck_simulation_launch.py` in another terminal at the same time.**

**Terminal 1 — launch the complete system**

```bash
cd "$HOME/gps_truck_nav"
source /opt/ros/iron/setup.bash
source install/setup.bash
ros2 launch truck_bringup gps_waypoint_follower.launch.py \
  use_rviz:=true use_mapviz:=false use_gpu:=false
```

**Terminal 2 — inspect readiness before sending goals**

```bash
cd "$HOME/gps_truck_nav"
source /opt/ros/iron/setup.bash
source install/setup.bash
ros2 node list
ros2 action list -t
ros2 service type /fromLL
ros2 lifecycle get /bt_navigator
ros2 topic echo /gps/fix --once
ros2 topic echo /odometry/global --once
ros2 run tf2_ros tf2_echo map base_footprint
```

Expect `/navigate_through_poses` with `nav2_msgs/action/NavigateThroughPoses`,
`/fromLL` with `robot_localization/srv/FromLL`, an active navigator, GPS fixes
near latitude −22.6179 / longitude −47.5022, updating odometry and a coherent TF
chain. `tf2_echo` continues until **Ctrl+C**. If a command waits indefinitely,
check [Known limitations](docs/KNOWN_LIMITATIONS.md); do not interpret an active
process as successful localization. In particular, inspect `ros2 topic list` if
`/gps/fix` is absent: the retained model contains conflicting GPS output remaps.

**Terminal 3 — send the stored GPS trajectory**

```bash
cd "$HOME/gps_truck_nav"
source /opt/ros/iron/setup.bash
source install/setup.bash
ros2 run truck_bringup logged_waypoint_follower.py \
  "$(ros2 pkg prefix --share truck_bringup)/params/empty_world_gps_wps.yaml" \
  --ros-args -p use_sim_time:=true
```

Despite its name, the canonical `empty_world_gps_wps.yaml` contains **ten CTVI
waypoints**, not the old Sonoma example. Each is converted through `/fromLL` once
and the ordered route is submitted using `NavigateThroughPoses`. The sender
retains the historical **+4.8025 m map-x translation**. It does not rotate or retune
that offset. Failed conversions abort before any partial route is sent; rejected,
failed or canceled actions are not reported as successes.

With valid localization and an active stack, expected behavior is a global plan,
velocity commands and truck motion toward the route. **This behavior has not been
observed in this audit and is subject to the retained runtime issues.** Inspect
`/plan`, `/cmd_vel_nav`, `/cmd_vel`, `/sick_scan_2D`, `/tf` and `/tf_static` using
`ros2 topic list` / `ros2 topic echo`; an available topic alone does not validate
its content. A Nav2 action success is not the manuscript's success metric.

**Stop:** press Ctrl+C in Terminal 3 and then Terminal 1. Interrupting a sender
alone may leave an accepted action running; stopping Terminal 1 terminates Nav2
and Gazebo. Stop Terminal 2's `tf2_echo` too. These steps apply only to simulation.


## VALIDATION

| Result | Checks |
| --- | --- |
| PASS | Three unique colcon packages; active manifests; 14 YAML-family files with unique keys; 35 XML-family syntax checks; 42 Python/launch compilations; eight installed script paths/shebangs; seven sender regression tests; ten README bash blocks; ShellCheck; Docker Compose config; LFS fetch/fsck; current-tree Gitleaks; 51 scientific-content checks and seven mesh hashes |
| FAIL — environment | Actual `colcon build` reached CMake/GNU compiler checks and failed on absent `ament_cmake`; ROS 2 is not installed |
| SKIPPED | ROS launch-description execution, Docker image build/start, Gazebo/GUI and a complete GPS mission |
| UNRESOLVED | Runtime TF/GNSS wiring/frame alignment; historical private-fork equivalence; imagery/CAD/component license provenance |

[VALIDATION.md](VALIDATION.md) records scope and reproduction commands. Syntax,
mocked sender tests and same-site coordinates are not runtime navigation proof.

## PRIVATE DEPENDENCIES

- `navigation2-private`, branch `iron-truck`: inaccessible during audit (404),
  exact historical implementation unavailable to reviewers.
- `offroad_sim`, branch `master`: confirmed private.
- Historical farm/segmentation paths also reference external
  `laser_scan_integrator`, absent maps/scripts and local classifier weights.

The public Quick Start uses binary upstream Iron dependencies. The optional
public source manifest pins upstream Iron at
`022e7e18570d75a969b96ccd1c3d5c5c443b3f12`; this is explicitly not the private fork
and no historical identity is claimed. The original duplicate `bcr_teleop` key
was corrected in the retained historical manifest.

## BROKEN OR HISTORICAL COMPONENTS

See [KNOWN_LIMITATIONS.md](KNOWN_LIMITATIONS.md): duplicate/conflicting TF
publication; questionable EKF `world_frame` and `gps` input wiring; two GPS output
remaps; unmatched perception input; unresolved datum/pose/offset alignment;
broken interactive sender; absent private farm resources/weights; unbuilt wall
plugin; experimental lane unit/topic differences. These were not hidden by
changing scientific parameters or results. The default full launch now selects
CTVI explicitly; the simulation-only default remains the original Sonoma scene.

## LICENSE / PROVENANCE

Original Intel/OSRF Apache notices retained; current package maintainer fields
updated; full Apache license text supplied for applicable components. The root
LICENSE describes scope rather than imposing a blanket license. Legacy MIT versus
Apache metadata, GPS helper source rights, experimental planner licensing and
CAD/image ownership remain unresolved. Historical developer attribution/contact
metadata was preserved outside the build tree.

## PUBLICATION ASSETS

All unique worlds, images, heightmaps, vehicle models and meshes remain. The seven
mesh assets moved into the canonical package without changing LFS IDs. Eight
ordinary-Git images received narrow LFS attribute exceptions, with no content
migration. All originally referenced LFS backing objects were retrievable.

CTVI coordinates are not labeled confidential just because they identify the
publicly described site. Aerial imagery/heightmap rights and CAD/partner disclosure
permissions require independent owner confirmation. See the per-path
[PUBLICATION_ASSET_AUDIT.md](../PUBLICATION_ASSET_AUDIT.md).

## README

The root README now explains the integration versus analysis repository, the
raw-data DOI, build requirements, canonical source tree, CMake executable names,
single complete launch, GPS sender, expected topics/actions, stop procedure,
optional displays/logger, unsupported interactive mode, separate experimental
lane workflow and all requested scientific limitations. It makes the current
runtime verification gap explicit.

## FILES REMOVED AS DEVELOPMENT TRASH

There was no tracked generated build/cache/editor trash in the baseline tree.
The following **107 redundant paths** were removed after equivalence/replacement
checks. This is path de-duplication, not deletion of unique scientific evidence.
The detailed evidence and replacement for every path are in
[REPOSITORY_CLEANUP_AUDIT.md](../REPOSITORY_CLEANUP_AUDIT.md).

- `nav_virtual_lanes/CMakeLists.txt`
- `nav_virtual_lanes/nav_virtual_lanes/__init__.py`
- `nav_virtual_lanes/nav_virtual_lanes/lane_costmap_generator.py`
- `nav_virtual_lanes/nav_virtual_lanes/lane_occupancy_detector.py`
- `nav_virtual_lanes/nav_virtual_lanes/visualize_gps_waypoints.py`
- `nav_virtual_lanes/nav_virtual_lanes/visualize_lanes.py`
- `nav_virtual_lanes/nav_virtual_lanes/visualize_lanes_from_gps.py`
- `nav_virtual_lanes/nav_virtual_lanes/visualize_lanes_from_gps_and_wps.py`
- `nav_virtual_lanes/nav_virtual_lanes/wall_costmap_generator.py`
- `nav_virtual_lanes/package.xml`
- `nav_virtual_lanes/setup.cfg`
- `nav_virtual_lanes/setup.py`
- `nav_virtual_lanes/test/test_copyright.py`
- `nav_virtual_lanes/test/test_flake8.py`
- `nav_virtual_lanes/test/test_pep257.py`
- `nav_virtual_lanes/wall_inflation_layer.hpp`
- `nav_virtual_lanes/wall_inflation_layer.xml`
- `src/gps_truck_nav/.gitignore`
- `src/gps_truck_nav/nav2_gps.repos`
- `src/gps_truck_nav/truck_bringup/truck_bringup/__init__.py`
- `src/gps_truck_nav/truck_bringup/truck_bringup/camera_costmap_generator.py`
- `src/gps_truck_nav/truck_bringup/truck_bringup/gps_waypoint_logger.py`
- `src/gps_truck_nav/truck_bringup/truck_bringup/logged_waypoint_follower.py`
- `src/gps_truck_nav/truck_bringup/truck_bringup/ultrasonic_qos_bridge.py`
- `src/gps_truck_nav/truck_bringup/truck_bringup/utils/__init__.py`
- `src/gps_truck_nav/truck_bringup/truck_bringup/utils/gps_utils.py`
- `truck_bringup/CMakeLists.txt`
- `truck_bringup/README.md`
- `truck_bringup/front_cabin_plugin.xml`
- `truck_bringup/launch/camera_segmentation_costmap_generator_launch.py`
- `truck_bringup/launch/dual_ekf_navsat.launch.py`
- `truck_bringup/launch/farm_truck_simulation_launch.py`
- `truck_bringup/launch/gps_waypoint_follower.launch.py`
- `truck_bringup/launch/mapviz.launch.py`
- `truck_bringup/launch/navigation_launch.py`
- `truck_bringup/launch/rviz_launch.py`
- `truck_bringup/launch/truck_simulation_launch.py`
- `truck_bringup/models/Lencois_Farm/model.config`
- `truck_bringup/models/Lencois_Farm/model.sdf`
- `truck_bringup/models/Lencois_Farm/textures/Lencois_Farm_aerial.png`
- `truck_bringup/models/Lencois_Farm/textures/Lencois_Farm_heightmap.png`
- `truck_bringup/models/Lencois_Paulista_Farm/model.config`
- `truck_bringup/models/Lencois_Paulista_Farm/model.sdf`
- `truck_bringup/models/Lencois_Paulista_Farm/textures/Lencois_Paulista_Farm_aerial.png`
- `truck_bringup/models/Lencois_Paulista_Farm/textures/Lencois_Paulista_Farm_heightmap.png`
- `truck_bringup/models/arocs_truck/model.config`
- `truck_bringup/models/arocs_truck/model.sdf`
- `truck_bringup/models/ctvi/materials/scripts/MyTerrainMaterial.material`
- `truck_bringup/models/ctvi/materials/textures/ctvi_aerial.png`
- `truck_bringup/models/ctvi/materials/textures/ctvi_heightmap.png`
- `truck_bringup/models/ctvi/model.config`
- `truck_bringup/models/ctvi/model.sdf`
- `truck_bringup/models/ctvi_completo/model.config`
- `truck_bringup/models/ctvi_completo/model.sdf`
- `truck_bringup/models/ctvi_completo/textures/ctvi_aerial.png`
- `truck_bringup/models/ctvi_completo/textures/ctvi_heightmap.png`
- `truck_bringup/package.xml`
- `truck_bringup/params/gps_wpf_demo.mvc`
- `truck_bringup/params/navsat.yaml`
- `truck_bringup/params/patio_mercedes.mvc`
- `truck_bringup/params/virtual_lanes_gps_wps.yaml`
- `truck_bringup/rviz/truck.rviz`
- `truck_bringup/scripts/__init__.py`
- `truck_bringup/scripts/camera_costmap_generator.py`
- `truck_bringup/scripts/classificador.py`
- `truck_bringup/scripts/follow_dynamic_gps_wps_lanes.py`
- `truck_bringup/scripts/gps_datum_setter.py`
- `truck_bringup/scripts/gps_waypoint_logger.py`
- `truck_bringup/scripts/interactive_waypoint_follower.py`
- `truck_bringup/scripts/logged_waypoint_follower.py`
- `truck_bringup/scripts/topics_republisher_freq.py`
- `truck_bringup/scripts/ultrasonic_qos_bridge.py`
- `truck_bringup/scripts/utils/__init__.py`
- `truck_bringup/scripts/utils/gps_utils.py`
- `truck_bringup/setup.cfg`
- `truck_bringup/setup.py`
- `truck_bringup/src/front_cabin_goal_checker.cpp`
- `truck_bringup/truck_bringup/__init__.py`
- `truck_bringup/truck_bringup/broadcast_odom_base_footprint.py`
- `truck_bringup/truck_bringup/camera_costmap_generator.py`
- `truck_bringup/truck_bringup/classificador.py`
- `truck_bringup/truck_bringup/follow_dynamic_gps_wps_lanes.py`
- `truck_bringup/truck_bringup/gps_datum_setter.py`
- `truck_bringup/truck_bringup/gps_waypoint_logger.py`
- `truck_bringup/truck_bringup/interactive_waypoint_follower.py`
- `truck_bringup/truck_bringup/logged_waypoint_follower.py`
- `truck_bringup/truck_bringup/nmea_to_navsat_converter.py`
- `truck_bringup/truck_bringup/odom_to_base_broadcaster.py`
- `truck_bringup/truck_bringup/ultrasonic_qos_bridge.py`
- `truck_bringup/truck_bringup/utils/__init__.py`
- `truck_bringup/truck_bringup/utils/gps_utils.py`
- `truck_bringup/urdf/arocs_truck.sdf`
- `truck_bringup/urdf/arocs_truck.urdf`
- `truck_bringup/urdf/arocs_truck.xacro`
- `truck_bringup/urdf/model.sdf`
- `truck_bringup/worlds/Lencois_Paulista_Farm.world`
- `truck_bringup/worlds/ambulance_world.world`
- `truck_bringup/worlds/baylands.world`
- `truck_bringup/worlds/blank.world`
- `truck_bringup/worlds/custom_sonoma.world`
- `truck_bringup/worlds/empty.world`
- `truck_bringup/worlds/empty_ground.world`
- `truck_bringup/worlds/emptyfarm.world`
- `truck_bringup/worlds/farm_arocs.world`
- `truck_bringup/worlds/minimal_empty.world`
- `truck_bringup/worlds/truck.model`
- `truck_bringup/worlds/world_only.model`

## FILES RETAINED AS OPTIONAL/HISTORICAL

Seven required meshes were relocated into the canonical model directory.
The following **29 historical files/variants** were retained outside colcon
package discovery, with their differing contents and attribution intact:

- `historical/experiments/camera_costmap_generator.py`
- `historical/experiments/classificador.py`
- `historical/experiments/gps_datum_setter.py`
- `historical/experiments/interactive_waypoint_follower.py`
- `historical/experiments/topics_republisher_freq.py`
- `historical/experiments/ultrasonic_qos_bridge.py`
- `historical/launch/camera_segmentation_costmap_generator_launch.py`
- `historical/launch/farm_truck_simulation_launch.py`
- `historical/nav_virtual_lanes_planner/CMakeLists.txt`
- `historical/nav_virtual_lanes_planner/include/nav_virtual_lanes_planner/virtual_lanes_planner.hpp`
- `historical/nav_virtual_lanes_planner/package.xml`
- `historical/nav_virtual_lanes_planner/src/virtual_lanes_planner.cpp`
- `historical/nav_virtual_lanes_planner/virtual_lanes_planner.xml`
- `historical/packaging/nav_virtual_lanes/setup.cfg`
- `historical/packaging/nav_virtual_lanes/setup.py`
- `historical/packaging/truck_bringup/setup.cfg`
- `historical/packaging/truck_bringup/setup.py`
- `historical/python_variants/broadcast_odom_base_footprint.py`
- `historical/python_variants/classificador.py`
- `historical/python_variants/follow_dynamic_gps_wps_lanes.py`
- `historical/python_variants/gps_datum_setter.py`
- `historical/python_variants/interactive_waypoint_follower.py`
- `historical/python_variants/nmea_to_navsat_converter.py`
- `historical/python_variants/odom_to_base_broadcaster.py`
- `historical/root_snapshot/params/dual_ekf_navsat_params.yaml`
- `historical/root_snapshot/params/empty_world_gps_wps.yaml`
- `historical/root_snapshot/params/truck_nav2_params.yaml`
- `historical/unbuilt_plugins/wall_inflation_layer.hpp`
- `historical/unbuilt_plugins/wall_inflation_layer.xml`

Optional current material includes RViz, Mapviz/local configuration and the Tk GPS
logger. The current experimental lane packages/dynamic sender are documented
separately; no validated lane mission is claimed. Non-default worlds, URDF/model
variants, textures, heightmaps and waypoint/configuration snapshots remain with
individual classifications in the cleanup audit.

## FILES LEFT FOR OWNER DECISION

- Non-default worlds, alternate URDF/SDF/xacro models and `params/navsat.yaml`:
  exact manuscript/Supporting Information role cannot be established from the
  supplied repository and scientific context alone.
- Unique historical TF/NMEA helpers, classification/segmentation code and both
  lane-planner variants: retain or further archive after research provenance review.
- Aerial images, heightmaps and seven CAD-derived meshes: source license and
  partner permission evidence.
- GPS helper reuse, legacy MIT/Apache scope and experimental planner license.
- Whether/how to revoke the key and independently rewrite old remote history.
- Runtime resolution of the TF/GNSS and alignment findings, followed by a logged
  Iron/Gazebo integration test. No scientific snapshot was retuned to bypass them.

## FILES CHANGED — complete path/status list

`A` added, `M` modified, `D` removed duplicate path, `R` relocated. This list is
relative to the baseline and includes the handoff document itself. Git's rename
heuristic can pair equivalent old duplicates differently from the explicit
source-to-destination audit; content preservation is documented independently.

```text
A	.dockerignore
M	.gitattributes
M	.gitignore
M	Dockerfile
A	LICENSE
A	LICENSES/Apache-2.0.txt
A	PUBLICATION_ASSET_AUDIT.md
A	README.md
A	REPOSITORY_CLEANUP_AUDIT.md
A	THIRD_PARTY_NOTICES.md
M	docker-compose.yml
A	docs/KNOWN_LIMITATIONS.md
A	docs/SECURITY_AUDIT.md
A	docs/VALIDATION.md
M	entrypoint.sh
A	historical/COLCON_IGNORE
A	historical/README.md
R060	src/gps_truck_nav/nav2_gps.repos	historical/dependencies/nav2_gps.repos
R100	src/gps_truck_nav/truck_bringup/scripts/camera_costmap_generator.py	historical/experiments/camera_costmap_generator.py
R100	src/gps_truck_nav/truck_bringup/scripts/classificador.py	historical/experiments/classificador.py
R100	src/gps_truck_nav/truck_bringup/scripts/gps_datum_setter.py	historical/experiments/gps_datum_setter.py
R100	src/gps_truck_nav/truck_bringup/scripts/interactive_waypoint_follower.py	historical/experiments/interactive_waypoint_follower.py
R100	src/gps_truck_nav/truck_bringup/scripts/topics_republisher_freq.py	historical/experiments/topics_republisher_freq.py
R100	src/gps_truck_nav/truck_bringup/scripts/ultrasonic_qos_bridge.py	historical/experiments/ultrasonic_qos_bridge.py
R100	src/gps_truck_nav/truck_bringup/launch/camera_segmentation_costmap_generator_launch.py	historical/launch/camera_segmentation_costmap_generator_launch.py
R100	src/gps_truck_nav/truck_bringup/launch/farm_truck_simulation_launch.py	historical/launch/farm_truck_simulation_launch.py
R100	src/nav_virtual_lanes_planner/CMakeLists.txt	historical/nav_virtual_lanes_planner/CMakeLists.txt
R100	src/nav_virtual_lanes_planner/include/nav_virtual_lanes_planner/virtual_lanes_planner.hpp	historical/nav_virtual_lanes_planner/include/nav_virtual_lanes_planner/virtual_lanes_planner.hpp
R100	src/nav_virtual_lanes_planner/package.xml	historical/nav_virtual_lanes_planner/package.xml
R100	src/nav_virtual_lanes_planner/src/virtual_lanes_planner.cpp	historical/nav_virtual_lanes_planner/src/virtual_lanes_planner.cpp
R100	src/nav_virtual_lanes_planner/virtual_lanes_planner.xml	historical/nav_virtual_lanes_planner/virtual_lanes_planner.xml
R100	nav_virtual_lanes/setup.cfg	historical/packaging/nav_virtual_lanes/setup.cfg
R100	nav_virtual_lanes/setup.py	historical/packaging/nav_virtual_lanes/setup.py
R100	src/gps_truck_nav/truck_bringup/setup.cfg	historical/packaging/truck_bringup/setup.cfg
R100	src/gps_truck_nav/truck_bringup/setup.py	historical/packaging/truck_bringup/setup.py
R100	src/gps_truck_nav/truck_bringup/truck_bringup/broadcast_odom_base_footprint.py	historical/python_variants/broadcast_odom_base_footprint.py
R100	src/gps_truck_nav/truck_bringup/truck_bringup/classificador.py	historical/python_variants/classificador.py
R100	src/gps_truck_nav/truck_bringup/truck_bringup/follow_dynamic_gps_wps_lanes.py	historical/python_variants/follow_dynamic_gps_wps_lanes.py
R100	src/gps_truck_nav/truck_bringup/truck_bringup/gps_datum_setter.py	historical/python_variants/gps_datum_setter.py
R100	src/gps_truck_nav/truck_bringup/truck_bringup/interactive_waypoint_follower.py	historical/python_variants/interactive_waypoint_follower.py
R100	src/gps_truck_nav/truck_bringup/truck_bringup/nmea_to_navsat_converter.py	historical/python_variants/nmea_to_navsat_converter.py
R100	src/gps_truck_nav/truck_bringup/truck_bringup/odom_to_base_broadcaster.py	historical/python_variants/odom_to_base_broadcaster.py
R100	truck_bringup/params/dual_ekf_navsat_params.yaml	historical/root_snapshot/params/dual_ekf_navsat_params.yaml
R100	truck_bringup/params/empty_world_gps_wps.yaml	historical/root_snapshot/params/empty_world_gps_wps.yaml
R099	truck_bringup/params/truck_nav2_params.yaml	historical/root_snapshot/params/truck_nav2_params.yaml
R100	nav_virtual_lanes/wall_inflation_layer.hpp	historical/unbuilt_plugins/wall_inflation_layer.hpp
R100	nav_virtual_lanes/wall_inflation_layer.xml	historical/unbuilt_plugins/wall_inflation_layer.xml
M	nav2_gps.repos
D	nav_virtual_lanes/CMakeLists.txt
D	nav_virtual_lanes/nav_virtual_lanes/__init__.py
D	nav_virtual_lanes/nav_virtual_lanes/lane_costmap_generator.py
D	nav_virtual_lanes/nav_virtual_lanes/lane_occupancy_detector.py
D	nav_virtual_lanes/nav_virtual_lanes/visualize_gps_waypoints.py
D	nav_virtual_lanes/nav_virtual_lanes/visualize_lanes.py
D	nav_virtual_lanes/nav_virtual_lanes/visualize_lanes_from_gps.py
D	nav_virtual_lanes/nav_virtual_lanes/visualize_lanes_from_gps_and_wps.py
D	nav_virtual_lanes/nav_virtual_lanes/wall_costmap_generator.py
D	nav_virtual_lanes/package.xml
D	nav_virtual_lanes/test/test_copyright.py
D	nav_virtual_lanes/test/test_flake8.py
D	nav_virtual_lanes/test/test_pep257.py
D	src/gps_truck_nav/.gitignore
A	src/gps_truck_nav/nav_virtual_lanes/README.md
M	src/gps_truck_nav/nav_virtual_lanes/nav_virtual_lanes/visualize_lanes_from_gps.py
M	src/gps_truck_nav/nav_virtual_lanes/package.xml
D	src/gps_truck_nav/nav_virtual_lanes/setup.cfg
D	src/gps_truck_nav/nav_virtual_lanes/setup.py
D	src/gps_truck_nav/nav_virtual_lanes/wall_inflation_layer.hpp
D	src/gps_truck_nav/nav_virtual_lanes/wall_inflation_layer.xml
A	src/gps_truck_nav/nav_virtual_lanes_planner/README.md
M	src/gps_truck_nav/nav_virtual_lanes_planner/package.xml
M	src/gps_truck_nav/truck_bringup/CMakeLists.txt
M	src/gps_truck_nav/truck_bringup/README.md
M	src/gps_truck_nav/truck_bringup/front_cabin_plugin.xml
M	src/gps_truck_nav/truck_bringup/launch/dual_ekf_navsat.launch.py
M	src/gps_truck_nav/truck_bringup/launch/gps_waypoint_follower.launch.py
M	src/gps_truck_nav/truck_bringup/launch/mapviz.launch.py
M	src/gps_truck_nav/truck_bringup/launch/rviz_launch.py
M	src/gps_truck_nav/truck_bringup/launch/truck_simulation_launch.py
R100	truck_bringup/models/arocs_truck/meshes/cabine_chassi.dae	src/gps_truck_nav/truck_bringup/models/arocs_truck/meshes/cabine_chassi.dae
R100	truck_bringup/models/arocs_truck/meshes/roda_frontal.dae	src/gps_truck_nav/truck_bringup/models/arocs_truck/meshes/roda_frontal.dae
R100	truck_bringup/models/arocs_truck/meshes/roda_frontal_direita.dae	src/gps_truck_nav/truck_bringup/models/arocs_truck/meshes/roda_frontal_direita.dae
R100	truck_bringup/models/arocs_truck/meshes/roda_frontal_esquerda.dae	src/gps_truck_nav/truck_bringup/models/arocs_truck/meshes/roda_frontal_esquerda.dae
R100	truck_bringup/models/arocs_truck/meshes/roda_traseira.dae	src/gps_truck_nav/truck_bringup/models/arocs_truck/meshes/roda_traseira.dae
R100	truck_bringup/models/arocs_truck/meshes/roda_traseira_direita.dae	src/gps_truck_nav/truck_bringup/models/arocs_truck/meshes/roda_traseira_direita.dae
R100	truck_bringup/models/arocs_truck/meshes/roda_traseira_esquerda.dae	src/gps_truck_nav/truck_bringup/models/arocs_truck/meshes/roda_traseira_esquerda.dae
M	src/gps_truck_nav/truck_bringup/package.xml
M	src/gps_truck_nav/truck_bringup/params/patio_mercedes.mvc
M	src/gps_truck_nav/truck_bringup/params/truck_nav2_params.yaml
M	src/gps_truck_nav/truck_bringup/scripts/gps_waypoint_logger.py
M	src/gps_truck_nav/truck_bringup/scripts/logged_waypoint_follower.py
M	src/gps_truck_nav/truck_bringup/src/front_cabin_goal_checker.cpp
D	src/gps_truck_nav/truck_bringup/truck_bringup/__init__.py
D	src/gps_truck_nav/truck_bringup/truck_bringup/camera_costmap_generator.py
D	src/gps_truck_nav/truck_bringup/truck_bringup/gps_waypoint_logger.py
D	src/gps_truck_nav/truck_bringup/truck_bringup/logged_waypoint_follower.py
D	src/gps_truck_nav/truck_bringup/truck_bringup/ultrasonic_qos_bridge.py
D	src/gps_truck_nav/truck_bringup/truck_bringup/utils/__init__.py
D	src/gps_truck_nav/truck_bringup/truck_bringup/utils/gps_utils.py
A	tests/test_waypoint_sender.py
A	tools/audit_repository.py
D	truck_bringup/CMakeLists.txt
D	truck_bringup/README.md
D	truck_bringup/front_cabin_plugin.xml
D	truck_bringup/launch/camera_segmentation_costmap_generator_launch.py
D	truck_bringup/launch/dual_ekf_navsat.launch.py
D	truck_bringup/launch/farm_truck_simulation_launch.py
D	truck_bringup/launch/gps_waypoint_follower.launch.py
D	truck_bringup/launch/mapviz.launch.py
D	truck_bringup/launch/navigation_launch.py
D	truck_bringup/launch/rviz_launch.py
D	truck_bringup/launch/truck_simulation_launch.py
D	truck_bringup/models/Lencois_Farm/model.config
D	truck_bringup/models/Lencois_Farm/model.sdf
D	truck_bringup/models/Lencois_Farm/textures/Lencois_Farm_aerial.png
D	truck_bringup/models/Lencois_Farm/textures/Lencois_Farm_heightmap.png
D	truck_bringup/models/Lencois_Paulista_Farm/model.config
D	truck_bringup/models/Lencois_Paulista_Farm/model.sdf
D	truck_bringup/models/Lencois_Paulista_Farm/textures/Lencois_Paulista_Farm_aerial.png
D	truck_bringup/models/Lencois_Paulista_Farm/textures/Lencois_Paulista_Farm_heightmap.png
D	truck_bringup/models/arocs_truck/model.config
D	truck_bringup/models/arocs_truck/model.sdf
D	truck_bringup/models/ctvi/materials/scripts/MyTerrainMaterial.material
D	truck_bringup/models/ctvi/materials/textures/ctvi_aerial.png
D	truck_bringup/models/ctvi/materials/textures/ctvi_heightmap.png
D	truck_bringup/models/ctvi/model.config
D	truck_bringup/models/ctvi/model.sdf
D	truck_bringup/models/ctvi_completo/model.config
D	truck_bringup/models/ctvi_completo/model.sdf
D	truck_bringup/models/ctvi_completo/textures/ctvi_aerial.png
D	truck_bringup/models/ctvi_completo/textures/ctvi_heightmap.png
D	truck_bringup/package.xml
D	truck_bringup/params/gps_wpf_demo.mvc
D	truck_bringup/params/navsat.yaml
D	truck_bringup/params/patio_mercedes.mvc
D	truck_bringup/params/virtual_lanes_gps_wps.yaml
D	truck_bringup/rviz/truck.rviz
D	truck_bringup/scripts/__init__.py
D	truck_bringup/scripts/camera_costmap_generator.py
D	truck_bringup/scripts/classificador.py
D	truck_bringup/scripts/follow_dynamic_gps_wps_lanes.py
D	truck_bringup/scripts/gps_datum_setter.py
D	truck_bringup/scripts/gps_waypoint_logger.py
D	truck_bringup/scripts/interactive_waypoint_follower.py
D	truck_bringup/scripts/logged_waypoint_follower.py
D	truck_bringup/scripts/topics_republisher_freq.py
D	truck_bringup/scripts/ultrasonic_qos_bridge.py
D	truck_bringup/scripts/utils/__init__.py
D	truck_bringup/scripts/utils/gps_utils.py
D	truck_bringup/setup.cfg
D	truck_bringup/setup.py
D	truck_bringup/src/front_cabin_goal_checker.cpp
D	truck_bringup/truck_bringup/__init__.py
D	truck_bringup/truck_bringup/broadcast_odom_base_footprint.py
D	truck_bringup/truck_bringup/camera_costmap_generator.py
D	truck_bringup/truck_bringup/classificador.py
D	truck_bringup/truck_bringup/follow_dynamic_gps_wps_lanes.py
D	truck_bringup/truck_bringup/gps_datum_setter.py
D	truck_bringup/truck_bringup/gps_waypoint_logger.py
D	truck_bringup/truck_bringup/interactive_waypoint_follower.py
D	truck_bringup/truck_bringup/logged_waypoint_follower.py
D	truck_bringup/truck_bringup/nmea_to_navsat_converter.py
D	truck_bringup/truck_bringup/odom_to_base_broadcaster.py
D	truck_bringup/truck_bringup/ultrasonic_qos_bridge.py
D	truck_bringup/truck_bringup/utils/__init__.py
D	truck_bringup/truck_bringup/utils/gps_utils.py
D	truck_bringup/urdf/arocs_truck.sdf
D	truck_bringup/urdf/arocs_truck.urdf
D	truck_bringup/urdf/arocs_truck.xacro
D	truck_bringup/urdf/model.sdf
D	truck_bringup/worlds/Lencois_Paulista_Farm.world
D	truck_bringup/worlds/ambulance_world.world
D	truck_bringup/worlds/baylands.world
D	truck_bringup/worlds/blank.world
D	truck_bringup/worlds/custom_sonoma.world
D	truck_bringup/worlds/empty.world
D	truck_bringup/worlds/empty_ground.world
D	truck_bringup/worlds/emptyfarm.world
D	truck_bringup/worlds/farm_arocs.world
D	truck_bringup/worlds/minimal_empty.world
D	truck_bringup/worlds/truck.model
D	truck_bringup/worlds/world_only.model
A	docs/OWNER_REVIEW_REPORT.md
```
