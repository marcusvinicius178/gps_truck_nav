# nav_virtual_lanes_planner — experimental

Plugin `nav_virtual_lanes_planner/VirtualLanesPlanner` implements the Nav2 global
planner interface and consumes lane `visualization_msgs/Marker` messages. It is
not selected by the retained SMAC/MPPI configuration or the reviewer Quick Start.

The distinct older implementation is retained under
`historical/nav_virtual_lanes_planner/`. No planner behavior was normalized across
the two variants. Licensing provenance is unresolved; the package's
`LicenseRef-Provenance-Review-Required` label is a status marker, not a license
grant. See [THIRD_PARTY_NOTICES.md](../../../THIRD_PARTY_NOTICES.md).
