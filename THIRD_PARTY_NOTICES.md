# Third-party notices and unresolved provenance

This is an evidence-based inventory, not a blanket relicensing decision. Current
maintainership is distinct from authorship. Existing attribution was retained,
including in moved historical files.

| Component / paths | Evidence | Licensing/provenance status and action |
| --- | --- | --- |
| `src/gps_truck_nav/truck_bringup/launch/navigation_launch.py`, `rviz_launch.py`; `historical/launch/farm_truck_simulation_launch.py` | File headers credit Intel Corporation (2018), Apache-2.0; Navigation2-derived structure | Preserve original headers; Apache text in `LICENSES/Apache-2.0.txt`. No claim that all modifications are upstream code. |
| `src/gps_truck_nav/truck_bringup/launch/dual_ekf_navsat.launch.py` | Open Source Robotics Foundation (2018), Apache-2.0 header | Attribution retained; same license text supplied. |
| `src/gps_truck_nav/nav_virtual_lanes/test/` | OSRF template-test headers (2015/2017), Apache-2.0 | Retained once; the existing copyright test remains skipped, so it is not evidence of full license clearance. |
| Project integration, current `truck_bringup` / `nav_virtual_lanes` | Manifests already declared Apache-2.0 (different spellings normalized) | Maintainer updated to Marcus Vinícius Leal de Carvalho. Package declaration retained, not extended to unrelated assets or unidentified borrowed code. |
| `historical/packaging/truck_bringup/setup.py` | Original `maintainer='root'`, contact `pedro.gonzalez@eia.edu.co`, MIT metadata; missing resource marker and `models/rocs_truck` typo | Inactive packaging archived unchanged, including that upstream/development credit. Identity, origin and MIT/Apache scope require owner confirmation. Not used for public builds. |
| `historical/packaging/nav_virtual_lanes/` | Prior `rota_2024` maintainer identity and personal contact, Apache metadata | Preserved as historical authorship/maintenance evidence; current maintainer is in active package.xml. Contact metadata is not treated as a credential. |
| `src/gps_truck_nav/truck_bringup/scripts/utils/gps_utils.py` | `euler_from_quaternion` explicitly cites an Automatic Addison article | Source credit retained. A per-file reuse license has not been established; owner should confirm permission or replace with a suitably licensed implementation in a separately reviewed change. |
| GPS sender/logger lineage | APIs and structure resemble Nav2 GPS examples; no original per-file author/license headers found | Do not assign authorship from resemblance. Exact source revision and reuse terms require confirmation. No existing credit was erased. |
| Canonical and historical `nav_virtual_lanes_planner` | Original package metadata said `TODO: License declaration`; no source notice resolving it | Active manifest now explicitly says `LicenseRef-Provenance-Review-Required`. This is not permission to distribute under Apache or MIT. Owner decision required. |
| Arocs meshes and model/URDF geometry | `model.config` credits Marcus Vinícius; DAE export metadata says VCGLab / MeshLab | Exporter identity is not proof of CAD ownership or redistribution rights. Keep geometry unchanged and seek source/partner permission evidence. |
| CTVI / Lençóis textures and heightmaps | Model metadata includes `auto-gne` / “intelligent quads”; no imagery-provider license supplied | Generator metadata is not an imagery license. Satellite/aerial sources, terrain generation and downstream rights remain unresolved. |
| Bing/Mapviz | Configurations name Bing tile sources; no downloaded tile cache found in the tracked tree | Blank API keys only. Local display access does not establish imagery redistribution rights. |
| Mercedes-Benz, Grunner, Bosch, IPT identifiers | Truck/site/project association; no license or release instrument in tree | Do not infer endorsement or a partner asset release. See the asset audit. |

No root MIT/Apache blanket grant was invented. No partner materials were removed
on an assumption of confidentiality. The precise manuscript/Supporting Information
asset mapping and author agreements were not available; unresolved ownership and
reuse questions remain for the owner and relevant rights holders.

## External dependencies

- [Navigation2](https://github.com/ros-navigation/navigation2): public Iron source
  inspected at `022e7e18570d75a969b96ccd1c3d5c5c443b3f12`; its own license notices
  govern upstream components. The private research fork was not available.
- [Gazebo ROS packages](https://github.com/ros-simulation/gazebo_ros_pkgs),
  [robot_localization](https://github.com/cra-ros-pkg/robot_localization), and
  [Mapviz](https://github.com/swri-robotics/mapviz) are installed as dependencies;
  their source/assets are not relicensed by this repository.
- Historical `rviz2_plugin_ser2res` and `bcr_teleop` repositories were publicly
  accessible; the old `bcr_teleop` branch selection has not been runtime-tested.
  `offroad_sim` was confirmed private. These are not automatically imported.
