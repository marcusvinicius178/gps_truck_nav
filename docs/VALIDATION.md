# Validation record

Audit date: 2026-09-24. Baseline:
`2eb40fd44b48baa31b31dcd1047ab89f8a128c3b`.

The audit used a fresh Git clone with full reachable history and Git LFS retrieval.
It ran in a Linux environment without `/opt/ros/iron`, `ros2`, Gazebo or a Docker
Engine daemon. Auxiliary colcon, CMake, ShellCheck, Compose and Gitleaks tools were
installed in an isolated audit environment, not added as project runtime code.

| Check | Result | Evidence / scope |
| --- | --- | --- |
| Git branch / history | PASS | Work on `paper-public-cleanup-2026-09`; 20 baseline reachable commits inspected; no history rewrite or merge |
| Baseline duplicate discovery | FAIL at baseline; repaired | Six discovered package locations, three duplicated names; root CMake/setup/build type and content hashes examined |
| Final package discovery | PASS | `colcon list --base-paths .` and the selected canonical base find one each of truck_bringup, nav_virtual_lanes, nav_virtual_lanes_planner; historical tree ignored |
| Active manifests | PASS | `catkin_pkg` parse/validate for all three canonical package.xml files |
| YAML parse / duplicate keys | PASS | 14 YAML-family files checked, including repos/mvc/rviz; duplicate bcr_teleop mapping corrected; equal repeated costmap boolean removed without effective changes |
| XML parse | PASS (syntax only) | 35 XML/SDF/world/model/config/URDF/xacro files; no SDF schema or physics validation claimed |
| Python and launch syntax | PASS | 42 Python files compiled in memory; no ROS import or launch execution implied |
| Installed script paths/shebangs | PASS by CMake inspection | Three truck and five lane `.py` executables exist; module helper tree explicitly installed |
| Shell / entrypoint | PASS | `bash -n`; ShellCheck 0.11.0 on entrypoint |
| README shell blocks | PASS (syntax/source contracts) | Ten bash blocks parsed with `bash -n`; package/launch/executable/waypoint paths and sourcing order checked against sources |
| Sender regression tests | PASS | Seven unittest cases: single conversion/order, failed conversion without partial route, rejected goal, failed/canceled result, preserved 4.8025 m offset, invalid YAML, unchanged valid values |
| Docker Compose configuration | PASS | Standalone Docker Compose v5.5.1 `config --quiet`; volume paths, read-only repo mount and schema accepted; not a container startup |
| Docker image build / container execution | SKIPPED | Docker Engine/daemon unavailable; upstream image tag and apt package availability not tested by building an image |
| `colcon build ... --packages-select truck_bringup` | FAIL — environment prerequisite | After installing CMake, GNU C/C++ detection succeeded, then configuration stopped at missing `ament_cmake`; ROS 2 is not installed |
| Launch-description generation under ROS | SKIPPED | ROS launch/launch_ros/ament runtime unavailable; syntax and include graph inspected only |
| Gazebo / full GPS mission / GUI | SKIPPED | No compatible runtime, no observed truck motion or mission completion |
| Upstream Nav2 interface inspection | PASS (inspection only) | Iron commit `022e7e18570d75a969b96ccd1c3d5c5c443b3f12`: relevant plugin export strings, GoalChecker/GlobalPlanner API and BasicNavigator action/localization behavior inspected; not compiled |
| Geographic consistency | PASS for same-site check; alignment unresolved | CTVI world and ten canonical waypoints are nearby; original Sonoma/CTVI default mismatch identified; datum/orientation offsets preserved and documented |
| Scientific content preservation | PASS | 51 byte/effective-YAML checks plus seven mesh SHA-256 checks against baseline; no scenario, physics, waypoint, estimator/controller/planner numeric configuration or experimental planner code changed |
| Fresh-clone LFS retrieval | PASS | Initial remote `git lfs pull` completed for all 15 baseline pointer paths (13 unique objects), including required truck meshes; no unavailable backing object found |
| LFS pointer consistency | FAIL at baseline; repaired | Eight nested PNGs were real Git blobs covered by wildcard LFS rules. Narrow attribute exceptions preserve their exact bytes; no migration/history rewrite |
| Final LFS object/pointer checks | PASS | `git lfs fsck` on cleaned commit; `git lfs fsck --objects origin/master` also passes. Seven relocated mesh pointers retain original object IDs |
| Current-tree Gitleaks | PASS | Gitleaks 8.30.1 directory scan with 100% redaction: zero findings; backing asset files present |
| History Gitleaks | FINDINGS | Two generic-api-key introduction findings in old Mapviz configs; values withheld. Historical exposure intentionally not rewritten |
| Independent credential review | FINDINGS at baseline; sanitized current configs | Filename-only ripgrep plus manual patterns across all 140 original unique Git blobs / 20 commits found both Mapviz exposure and the old plain key file missed by the generic scanner |
| Broken symlinks / required mesh URIs | PASS | No broken link found; nine required model mesh references resolve to hydrated local assets |
| Licensing / asset permissions | UNRESOLVED | Missing CAD/imagery permissions, conflicting historical metadata and unresolved planner/utility provenance; see dedicated audits |

## Re-run the non-ROS checks

From the repository root with Python 3 and PyYAML installed:

```bash
python3 tools/audit_repository.py
python3 -m unittest discover -s tests -v
bash -n entrypoint.sh
shellcheck entrypoint.sh
colcon list --base-paths src/gps_truck_nav
docker compose config --quiet
git lfs fsck
git diff --check
```

ShellCheck, colcon, Docker Compose and Git LFS are separate command prerequisites.
`tools/audit_repository.py` is read-only and does not convert coordinates, tune
parameters, run ROS or rewrite files. The unittest harness uses ROS message/client
stubs: it tests sender control flow, not services, actions or navigation in ROS.

For owner-side runtime validation, follow the README on an Iron installation,
confirm actual topics/types/TF and resolve `KNOWN_LIMITATIONS.md` through explicit
review. Record the installed dependency versions and the launch output. A successful
build alone is not a successful mission, and a successful mission is not a
reproduction of the manuscript benchmark.
