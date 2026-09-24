# Historical and experimental material

This directory keeps earlier development variants and experimental material for
reference. It contains `COLCON_IGNORE`, so it is excluded from the normal ROS 2
workspace build.

| Directory | Contents |
| --- | --- |
| `root_snapshot/params/` | Earlier localization, waypoint and Navigation2 parameter snapshots |
| `nav_virtual_lanes_planner/` | Alternative planner implementation |
| `python_variants/` | Earlier sender, TF, NMEA and utility variants |
| `experiments/` | Development and prototype scripts |
| `launch/` | Earlier launch configurations |
| `packaging/` | Previous setup.py/setup.cfg packaging records |
| `unbuilt_plugins/` | Earlier plugin prototypes |
| `dependencies/` | Historical dependency manifest |

For the maintained integration, use the packages under:

```text
src/gps_truck_nav/
```

The historical directory is intentionally not part of the default build and can
be consulted when comparing earlier project variants.
