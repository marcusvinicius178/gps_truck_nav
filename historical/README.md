# Historical and experimental material

This directory has `COLCON_IGNORE`. It is not imported by the current launch graph
and is not a second supported workspace. Original contents/attribution are
preserved except for the documented equal YAML duplicate removal and correction
of the duplicate key/SSH URL in the historical dependency manifest.

| Directory | Why retained |
| --- | --- |
| `root_snapshot/params/` | Different Sonoma-era localization, single waypoint and custom-plugin Nav2 configuration; meaningful scientific provenance |
| `nav_virtual_lanes_planner/` | Divergent planner source/header, not interchangeable with the canonical package |
| `python_variants/` | Distinct earlier sender, classifier, TF and NMEA code; potential real-data interpretation relevance |
| `experiments/` | Broken interactive service workflow, classifier requiring absent weights, costmap prototype, datum capture and topic/QoS bridges |
| `launch/` | Obsolete farm/private-dependency and segmentation launches |
| `packaging/` | Inactive setup.py/setup.cfg records, including original author/contact and inconsistent licensing metadata |
| `unbuilt_plugins/` | WallInflationLayer declarations without implementation or a build/export target |
| `dependencies/` | Correctly keyed historical private/public dependency names; not an installation recipe |

Do not run historical TF publishers alongside the current localization tree. Do
not use these parameter variants as substitutes for missing per-run dumps. Some
paths/imports intentionally remain historical and broken; migration into this
archive does not make these files runnable. The manuscript and Supporting
Information were not supplied for a file-by-file provenance cross-check, so unique
experimental material was retained for owner review rather than deleted.

See [REPOSITORY_CLEANUP_AUDIT.md](../REPOSITORY_CLEANUP_AUDIT.md) for individual
paths, references, replacements and decisions. Deleted exact duplicates are also
recoverable from baseline commit `2eb40fd44b48baa31b31dcd1047ab89f8a128c3b`.
