# Heavy-Duty Agricultural Truck Navigation: Geometry-Dependent Planner–Controller Trade-offs in a Human-Referenced ROS 2 Navigation2 Benchmark

`gps_truck_nav` contains the vehicle-specific Navigation2 integration, truck
models, Gazebo worlds, launch files, localization/configuration snapshots and GPS
waypoint support associated with the manuscript submitted to **Computers and
Electronics in Agriculture**.

**Validation status:** the commands below have been checked against the package,
CMake, launch and waypoint sources. Package discovery, static checks, sender
regressions, Compose configuration and LFS retrieval were tested. **A successful
ROS/Gazebo build or moving-truck run has not been observed in this audit.** The
audit environment has no ROS 2 installation or Docker daemon. The retained
localization/TF configuration has unresolved runtime concerns described in
[Known limitations](docs/KNOWN_LIMITATIONS.md). This is a source-checked procedure
for integration testing, not a claim of an already validated end-to-end demo.

## Scientific scope and relationship to the paper

The research concerns a heavy-duty Mercedes-Benz/Grunner 8×4 Ackermann agricultural
truck; NavFn and SMAC global planners; MPPI and regulated pure pursuit (RPP)
controllers; GNSS, IMU and odometry localization; layered rolling costmaps; Gazebo
simulation; human-reference trajectories; and field obstacle-avoidance recordings.

| Resource | Role |
| --- | --- |
| This repository | Vehicle integration, models, launch/configuration artifacts and simulation support |
| [nav2_paper_scripts](https://github.com/marcusvinicius178/nav2_paper_scripts) | Processing, analysis, plotting and benchmark auditing |
| [Zenodo dataset: 10.5281/zenodo.22864167](https://doi.org/10.5281/zenodo.22864167) | Raw ROS 2 recordings; these are not distributed through this GitHub repository |

The dataset DOI was supplied by the authors. The record may still be completing
upload/publication preparation; availability and completeness were not verified
here. There is no paper DOI or acceptance claim in this repository.

## Repository structure

| Path | Purpose |
| --- | --- |
| `src/gps_truck_nav/truck_bringup/` | Canonical current integration package |
| `src/gps_truck_nav/nav_virtual_lanes/` | Separate experimental lane visualization/costmap package |
| `src/gps_truck_nav/nav_virtual_lanes_planner/` | Separate experimental C++ planner; license provenance unresolved |
| `historical/` | Different legacy parameter, Python and planner variants; excluded by `COLCON_IGNORE` |
| `tools/audit_repository.py`, `tests/` | Structural audit and ROS-free sender regression tests |
| `docs/` | Security, runtime limitations, validation and owner review report |

Build **`src/gps_truck_nav`**, or select `truck_bringup` for the core procedure.
There is now one discoverable copy of each package. The old root package copies
were compared by Git object IDs, hydrated SHA-256 hashes and diffs before removal.
The seven truck meshes were relocated without changing their LFS object IDs.
Different scientific configurations were retained under `historical/`.
See [REPOSITORY_CLEANUP_AUDIT.md](REPOSITORY_CLEANUP_AUDIT.md) for the full decisions
and usage map. The CMake packages do not use the historical setuptools entry points.

## Requirements

The repository's historical Docker/launch target is **Ubuntu 22.04 (Jammy), ROS 2
Iron, Gazebo Classic 11 and Navigation2 Iron**. No Humble, Jazzy, Ubuntu 24.04
native installation, modern Gazebo or CARLA compatibility is asserted here.
[ROS Iron](https://www.openrobotics.org/blog/2023/5/23/ros-2-iron-irwini-released)
and [Gazebo Classic](https://classic.gazebosim.org/) are end-of-life releases.
Their continued image/package availability is a separate installation constraint.

The package manifests declare the required ROS/Python dependencies, including
`robot_localization`, `nav2_simple_commander`, Gazebo ROS plugins,
`robot_state_publisher`, TF2, PyYAML and the build dependencies for the goal checker.
RViz and Mapviz are visualizations; Mapviz and Bing tiles are optional at runtime.
Git LFS is required for the truck meshes. A Linux graphics session is needed for
GUI windows; a headless invocation is provided below.

### Historical dependency note

The old manifest named `marcusvinicius178/navigation2-private` (`iron-truck`), which
was inaccessible during the audit, and the private `offroad_sim` repository.
It also used `bcr_teleop` twice as a YAML key. A corrected record is retained at
`historical/dependencies/nav2_gps.repos`, with `offroad_sim` under its own key and
HTTPS URLs. Do not import that manifest for the public Quick Start.

The public path uses released Iron binary packages. The root `nav2_gps.repos`
optionally pins **upstream** Navigation2 Iron to commit
`022e7e18570d75a969b96ccd1c3d5c5c443b3f12`; it is not the historical private fork.
Its plugin export names and commander interfaces were inspected, but a build and
runtime comparison were not possible. Neither upstream source nor the binary
packages are claimed to reproduce the private fork or its benchmark outcomes.
`offroad_sim`, `laser_scan_integrator`, custom perception plugins and classifier
weights used by historical experiments are not needed by the documented launch
include graph. Unresolved runtime behavior is listed separately.

## Installation

### Native Ubuntu 22.04

First install ROS 2 Iron Desktop and configure the ROS apt repository using the
[official Iron installation instructions](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html).
The following commands assume `/opt/ros/iron/setup.bash` exists. They have been
source-checked; package installation and a ROS build were not run on this host.

**Terminal 1 — clone, retrieve assets and build**

```bash
sudo apt-get update
sudo apt-get install -y git git-lfs build-essential cmake \
  python3-colcon-common-extensions python3-rosdep python3-yaml \
  ros-iron-navigation2 ros-iron-nav2-bringup ros-iron-gazebo-ros-pkgs

git lfs install
GIT_LFS_SKIP_SMUDGE=1 git clone --branch paper-public-cleanup-2026-09 \
  https://github.com/marcusvinicius178/gps_truck_nav.git "$HOME/gps_truck_nav"
cd "$HOME/gps_truck_nav"
git lfs pull
git lfs fsck
source /opt/ros/iron/setup.bash
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update --include-eol-distros
rosdep install --from-paths src/gps_truck_nav/truck_bringup \
  --ignore-src --rosdistro iron -y
colcon list --base-paths src/gps_truck_nav
colcon build --base-paths src/gps_truck_nav --packages-select truck_bringup --symlink-install
source install/setup.bash
ros2 pkg executables truck_bringup
```

The executable list should contain `logged_waypoint_follower.py`,
`gps_waypoint_logger.py` and `follow_dynamic_gps_wps_lanes.py`. **The `.py` suffix
is required:** CMake installs the files as programs to `lib/truck_bringup`.
Do not use `pip install .` or the old suffix-free setuptools names.

### Docker on Linux

Docker Engine and Compose must be installed on the host. Retrieve the repository
and LFS objects as above (host ROS is not needed), then:

```bash
cd "$HOME/gps_truck_nav"
docker compose config --quiet
docker compose build
docker compose up -d
docker compose exec nav2_sim bash -c 'source /opt/ros/iron/setup.bash && cd /ws && colcon build --base-paths /repo/src/gps_truck_nav --packages-select truck_bringup --symlink-install'
```

The full repository is mounted read-only at `/repo`; build/install/log directories
use container volumes. This replaces the old ambiguous `./src:/ws/src` mount and
never recursively changes ownership of the host checkout. No automatic dependency
installation is hidden in the entrypoint. Open each container terminal with:

```bash
docker compose exec nav2_sim bash
source /opt/ros/iron/setup.bash
source /ws/install/setup.bash
```

Then run the same `ros2` commands below. For a headless Docker test:

```bash
ros2 launch truck_bringup gps_waypoint_follower.launch.py \
  use_gzclient:=false use_rviz:=false use_mapviz:=false
```

For GUI access on a local X11/XWayland Linux session, allow the container's root
user with `xhost +si:localuser:root` before launching GUI processes and revoke it
with `xhost -si:localuser:root` afterwards. The Compose file forwards `DISPLAY`
and the X11 socket. GPU passthrough/driver setup is host-specific and has not been
validated; no NVIDIA configuration is assumed. Stop the container with
`docker compose down` (the build volumes remain).

## Quick Start: full GPS waypoint simulation

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

## Worlds and stored waypoint modes

The table distinguishes coordinate checks from live validation. “Verified
(source)” never means that truck motion was demonstrated.

| World | Waypoint file | Launch command / selection | Sender | Purpose / status |
| --- | --- | --- | --- | --- |
| `ctvi.world` | Canonical `params/empty_world_gps_wps.yaml` (10 CTVI points) | Full Quick Start; CTVI is the full launch default | `logged_waypoint_follower.py` | Verified (source: same geographic site); experimental runtime, full pose alignment not established |
| `empty_ground.world` | `historical/root_snapshot/params/empty_world_gps_wps.yaml` (1 Sonoma point) | Simulation-only command below; full GPS use would also need the historical Sonoma localization file | `logged_waypoint_follower.py` accepts its schema | Historical; do not combine with the canonical CTVI datum |
| `empty_ground.world` | Canonical `params/virtual_lanes_gps_wps.yaml` (7 Sonoma points) | Sonoma localization is required; no supported full-lane launch is claimed | `logged_waypoint_follower.py` accepts radians; experimental lane tools differ | Historical / experimental, not a CTVI route |
| `empty.world`, `custom_sonoma.world`, `ambulance_world.world`, `baylands.world` | No separately validated route | No reviewer Quick Start; some refer to external models and `baylands` has two spherical-coordinate blocks | None recommended | Historical, dependencies/alignment unresolved |
| `Lencois_Paulista_Farm.world`, `emptyfarm.world`, `farm_arocs.world`, `truck.model`, `world_only.model` | No compatible route established | Historical only | None recommended | Zero/missing geographic origin and/or external models |
| `blank.world`, `minimal_empty.world` | None | Empty scene utilities, not complete truck demonstrations | None | Historical, no truck included |

CTVI's world origin is `(-22.617911667, -47.502177333, 488 m)` with
`heading_deg=180`; the truck starts at `(8.416977, 9.692661, 0.38 m)` and yaw
`5.176624799 rad`. The retained navsat datum is
`[-22.617921667, -47.502277333, 2.035032145]` (latitude, longitude, yaw).
Those origins and headings are **not identical**. Site proximity was checked;
a calibrated world/map/antenna alignment was not proven or changed.

### Simulation only

```bash
source /opt/ros/iron/setup.bash
source "$HOME/gps_truck_nav/install/setup.bash"
ros2 launch truck_bringup truck_simulation_launch.py
```

This launches the truck in the historical Sonoma `empty_ground.world`, Gazebo and
`robot_state_publisher`. It does **not** launch either EKF, navsat or Nav2. Optional
arguments are `world:=/absolute/path/to/file.world`, `use_gzclient:=false` and
`use_rviz:=true`. World definitions, physics and vehicle pose values were preserved.

### Interactive operation and waypoint logging

The old interactive sender is **not supported**: it imports a missing demo
package and waits for `gps_localizer/get_state`, a service not created by the
current launch. It is retained under `historical/experiments/` and is not installed
as a runnable current executable. No click-to-drive command is advertised.

The separate Tk GUI logger is optional and has a corrected local utility import:

```bash
ros2 run truck_bringup gps_waypoint_logger.py "$HOME/gps_waypoints.yaml" \
  --ros-args -p use_sim_time:=true
```

It subscribes to `/gps/fix` and `/imu` and appends latitude, longitude and yaw in
radians. Its GUI and sensor QoS compatibility have not been runtime-verified;
confirm actual sensor updates before logging. Never overwrite retained research
waypoint files to make a test pass.

### Dynamic/virtual-lane mode

This is separate experimental material, not the primary benchmark recipe.
`follow_dynamic_gps_wps_lanes.py` consumes `visualization_marker_array`,
`/binary_state` and `/odometry/global`; it sends `NavigateThroughPoses` and also
publishes `/cmd_vel`. The `nav_virtual_lanes` CMake file installs five `.py` tools
for lane markers, occupancy and wall/costmap generation. Build those packages
separately if investigating this mode. Do not run the dynamic and logged senders
together. Several lane tools interpret YAML yaw as degrees, whereas the logged
sender uses radians; the experimental C++ planner consumes a `Marker` topic,
not the sender's `MarkerArray`. These are distinct pipelines, not interchangeable
steps. See the usage map and known limitations before attempting integration.

## Mapviz and Bing Maps

Mapviz is disabled by default. It can show ROS overlays without a shared Bing
key; optional Bing background tiles require the user's own valid service access.
No imagery redistribution rights are implied. Copy settings outside the checkout:

```bash
mkdir -p "$HOME/.config/gps_truck_nav"
cp "$(ros2 pkg prefix --share truck_bringup)/params/gps_wpf_demo.mvc" \
  "$HOME/.config/gps_truck_nav/mapviz.local.mvc"
```

Edit `bing_api_key` in that local file if needed, then add
`use_mapviz:=true mapviz_config:="$HOME/.config/gps_truck_nav/mapviz.local.mvc"`
to the full launch command. Never commit the key. See
[Security audit](docs/SECURITY_AUDIT.md) for historical exposure and optional owner
remediation. Core Gazebo/localization/Nav2 have no Bing credential dependency.

## Planner/controller configuration

The active snapshot is
`src/gps_truck_nav/truck_bringup/params/truck_nav2_params.yaml`. It selects SMAC
Hybrid-A* (`GridBased`) and MPPI (`FollowPath`) and retains a custom front-cabin
goal checker. The study also compared NavFn and RPP; this checkout is **not** a
complete matrix of preserved per-run configurations. A stale
`bt_navigator.default_controller: RegulatedPurePursuit` entry does not override
the configured controller server's MPPI plugin. Historical settings are retained
for comparison, not represented as run-specific ground truth.

No controller gains, planner costs, footprint, speed limits, scenario geometry,
waypoint coordinates, mission thresholds or result tables were retuned. One
identical repeated YAML `track_unknown_space: true` was removed from each of two
snapshots; the parsed parameter dictionaries are unchanged.

## Reproducibility and limitations

This repository supports inspection and reconstruction of the integration. The
analysis repository and available Zenodo recordings support the corresponding
analysis workflow. These resources do not establish exact replay of every
historical simulated run.

- Retained configuration files are snapshots, not complete historical per-run
  runtime parameter dumps.
- Random seeds were not retained for every run; this was not a seed-controlled
  Monte Carlo experiment.
- The field demonstration has **N=1 per condition**.
- Field observations do not validate simulation rankings or establish
  planner-specific safety performance.
- No autonomous deployment safety certification is claimed.
- Private dependencies, unresolved asset permissions and the absence of a live
  validation run remain explicit limitations.

See [Validation](docs/VALIDATION.md), [Known limitations](docs/KNOWN_LIMITATIONS.md),
[publication asset audit](PUBLICATION_ASSET_AUDIT.md) and
[cleanup audit](REPOSITORY_CLEANUP_AUDIT.md). The complete owner handoff is in
[OWNER_REVIEW_REPORT.md](docs/OWNER_REVIEW_REPORT.md).

## Citation, authors and contact

Submitted-manuscript citation placeholder (no paper DOI assigned here):

```bibtex
@unpublished{carvalho2026trucknavigation,
  title = {Heavy-Duty Agricultural Truck Navigation: Geometry-Dependent Planner--Controller Trade-offs in a Human-Referenced ROS 2 Navigation2 Benchmark},
  author = {Carvalho, Marcus Vin{\'i}cius Leal de and Yoshioka, Leopoldo Rideki and Justo, Jo{\~a}o Francisco and Silva, Antonio Marcos da},
  year = {2026},
  note = {Manuscript submitted to Computers and Electronics in Agriculture}
}
```

Authors: **Marcus Vinícius Leal de Carvalho**, **Leopoldo Rideki Yoshioka**,
**João Francisco Justo**, **Antonio Marcos da Silva**.

Corresponding author: Marcus Vinícius Leal de Carvalho —
[marcusvini178@usp.br](mailto:marcusvini178@usp.br).

## License, third-party components and disclaimer

This is a mixed-provenance repository. See [LICENSE](LICENSE) and
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md). Existing Apache notices are
preserved; conflicting legacy MIT metadata, some adapted utilities, the
experimental planner and external imagery/CAD provenance remain unresolved.
A software-package license does not establish rights to every image, terrain,
mesh, trademark or partner asset. No blanket relicensing was performed.

Research software; not safety certified. It is not intended for direct deployment
on a road vehicle without independent validation.
