# Heavy-Duty Agricultural Truck Navigation

Vehicle-specific ROS 2 Navigation2 integration used in the study:

**“Heavy-Duty Agricultural Truck Navigation: Geometry-Dependent Planner–Controller Trade-offs in a Human-Referenced ROS 2 Navigation2 Benchmark.”**

This repository contains the truck integration package, Gazebo models and worlds,
Navigation2 configuration, localization setup, GPS waypoint tools, and supporting
simulation resources for the heavy-duty Mercedes-Benz/Grunner 8×4 Ackermann
platform used in the project.

The study evaluates combinations of NavFn and SMAC global planners with MPPI and
Regulated Pure Pursuit (RPP) controllers using simulation, human-reference
trajectories, and field recordings.

## Related resources

| Resource | Purpose |
| --- | --- |
| This repository | Vehicle integration, simulation, launch/configuration files, localization and GPS waypoint support |
| [nav2_paper_scripts](https://github.com/marcusvinicius178/nav2_paper_scripts) | Processing, metrics, plots and benchmark analysis |
| [Zenodo dataset – 10.5281/zenodo.22864167](https://doi.org/10.5281/zenodo.22864167) | Raw ROS 2 recordings used for the study |

## Repository structure

```text
gps_truck_nav/
├── src/gps_truck_nav/
│   ├── truck_bringup/              # Main vehicle integration package
│   ├── nav_virtual_lanes/          # Virtual-lane utilities
│   └── nav_virtual_lanes_planner/  # Experimental planner package
├── historical/                     # Legacy/experimental variants kept for reference
├── tests/                          # Lightweight regression tests
├── Dockerfile
├── docker-compose.yml
└── nav2_gps.repos
```

The main package for the truck simulation is:

```text
src/gps_truck_nav/truck_bringup
```

## Main software environment

The project was developed around:

- Ubuntu 22.04
- ROS 2 Iron
- Navigation2
- Gazebo Classic 11
- `robot_localization`
- RViz2
- Mapviz (optional)
- Git LFS for large model assets

## Installation

### 1. Clone the repository

```bash
cd "$HOME"
git clone https://github.com/marcusvinicius178/gps_truck_nav.git
cd gps_truck_nav
```

### 2. Retrieve Git LFS assets

```bash
git lfs install
git lfs pull
```

### 3. Install ROS dependencies

Assuming ROS 2 Iron is already installed:

```bash
source /opt/ros/iron/setup.bash

sudo rosdep init 2>/dev/null || true
rosdep update --include-eol-distros

rosdep install   --from-paths src/gps_truck_nav/truck_bringup   --ignore-src   --rosdistro iron   -y
```

### 4. Build

```bash
source /opt/ros/iron/setup.bash

colcon build   --base-paths src/gps_truck_nav   --packages-select truck_bringup   --symlink-install

source install/setup.bash
```

You can confirm the installed executables with:

```bash
ros2 pkg executables truck_bringup
```

The main waypoint-related executables include:

- `logged_waypoint_follower.py`
- `gps_waypoint_logger.py`
- `follow_dynamic_gps_wps_lanes.py`

## Quick start: GPS waypoint simulation

The full launch starts the Gazebo simulation together with the localization and
Navigation2 stack used by the truck integration.

### Terminal 1 — launch the system

```bash
cd "$HOME/gps_truck_nav"

source /opt/ros/iron/setup.bash
source install/setup.bash

ros2 launch truck_bringup gps_waypoint_follower.launch.py   use_rviz:=true   use_mapviz:=false   use_gpu:=false
```

The default full launch uses the CTVI simulation environment and the corresponding
GPS waypoint set.

### Terminal 2 — send the stored GPS trajectory

```bash
cd "$HOME/gps_truck_nav"

source /opt/ros/iron/setup.bash
source install/setup.bash

ros2 run truck_bringup logged_waypoint_follower.py   "$(ros2 pkg prefix --share truck_bringup)/params/empty_world_gps_wps.yaml"   --ros-args -p use_sim_time:=true
```

The canonical waypoint file contains the CTVI route used by the current example.
Waypoints are converted through `robot_localization/fromLL` and submitted to
Navigation2 as an ordered `NavigateThroughPoses` mission.

Useful runtime topics include:

```text
/gps/fix
/odometry/global
/plan
/cmd_vel
/tf
/tf_static
```

Useful checks while the simulation is running:

```bash
ros2 node list
ros2 action list -t
ros2 service type /fromLL
ros2 lifecycle get /bt_navigator
ros2 topic echo /gps/fix --once
ros2 topic echo /odometry/global --once
```

## Simulation-only launch

To launch the truck model and Gazebo without the complete Navigation2/GPS stack:

```bash
cd "$HOME/gps_truck_nav"

source /opt/ros/iron/setup.bash
source install/setup.bash

ros2 launch truck_bringup truck_simulation_launch.py
```

Optional launch arguments include:

```text
world:=/absolute/path/to/world.world
use_gzclient:=false
use_rviz:=true
```

## Planner and controller configuration

The main Navigation2 parameter file is:

```text
src/gps_truck_nav/truck_bringup/params/truck_nav2_params.yaml
```

The repository includes the vehicle-specific setup used to work with the planner
and controller families evaluated in the study:

- NavFn
- SMAC
- MPPI
- Regulated Pure Pursuit (RPP)

The analysis repository and Zenodo recordings contain the benchmark outputs used
for the manuscript results.

## GPS waypoint tools

### Follow a stored trajectory

```bash
ros2 run truck_bringup logged_waypoint_follower.py   /path/to/waypoints.yaml   --ros-args -p use_sim_time:=true
```

Expected YAML structure:

```yaml
waypoints:
  - latitude: -22.6179
    longitude: -47.5022
    yaw: 0.0
```

### Record waypoints

```bash
ros2 run truck_bringup gps_waypoint_logger.py   "$HOME/gps_waypoints.yaml"   --ros-args -p use_sim_time:=true
```

## Mapviz

Mapviz is optional. The simulation can be used without it.

To use Bing background tiles, copy the Mapviz configuration to a local user
configuration and add your own API key there rather than committing credentials to
the repository.

```bash
mkdir -p "$HOME/.config/gps_truck_nav"

cp "$(ros2 pkg prefix --share truck_bringup)/params/gps_wpf_demo.mvc"   "$HOME/.config/gps_truck_nav/mapviz.local.mvc"
```

Then launch with:

```bash
ros2 launch truck_bringup gps_waypoint_follower.launch.py   use_mapviz:=true   mapviz_config:="$HOME/.config/gps_truck_nav/mapviz.local.mvc"
```

## Docker

A Docker/Compose setup is also provided.

```bash
cd "$HOME/gps_truck_nav"

docker compose build
docker compose up -d
```

Build the workspace inside the container with:

```bash
docker compose exec nav2_sim bash -c   'source /opt/ros/iron/setup.bash &&    cd /ws &&    colcon build      --base-paths /repo/src/gps_truck_nav      --packages-select truck_bringup      --symlink-install'
```

## Research data and reproducibility

The repository is intentionally split into three parts:

1. **Vehicle/simulation integration:** this repository.
2. **Analysis and plotting:** [nav2_paper_scripts](https://github.com/marcusvinicius178/nav2_paper_scripts).
3. **Raw ROS 2 recordings:** [Zenodo dataset](https://doi.org/10.5281/zenodo.22864167).

This separation keeps the Git repository practical while preserving the raw
recordings used for the scientific analysis.

## Historical material

Older development variants are kept under `historical/` for reference. They are
excluded from the normal workspace build through `COLCON_IGNORE`.

For normal use, start with:

```text
src/gps_truck_nav/truck_bringup
```

## Troubleshooting and issues

ROS/Gazebo installations can differ in package availability, graphics drivers,
DDS configuration and dependency versions. If you encounter a reproducible issue,
please open a GitHub issue and include:

- Ubuntu version
- ROS 2 distribution
- command used
- terminal output
- relevant ROS topic/node information

That makes it much easier to reproduce and fix environment-specific problems.

## Citation

```bibtex
@unpublished{carvalho2026trucknavigation,
  title = {Heavy-Duty Agricultural Truck Navigation: Geometry-Dependent Planner--Controller Trade-offs in a Human-Referenced ROS 2 Navigation2 Benchmark},
  author = {Carvalho, Marcus Vin{\'i}cius Leal de and Yoshioka, Leopoldo Rideki and Justo, Jo{\~a}o Francisco and Silva, Antonio Marcos da},
  year = {2026},
  note = {Manuscript submitted to Computers and Electronics in Agriculture}
}
```

### Authors

- Marcus Vinícius Leal de Carvalho
- Leopoldo Rideki Yoshioka
- João Francisco Justo
- Antonio Marcos da Silva

Corresponding author: **Marcus Vinícius Leal de Carvalho**  
Email: **marcusvini178@usp.br**

## License

See [LICENSE](LICENSE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for
license information and notices related to third-party components.
