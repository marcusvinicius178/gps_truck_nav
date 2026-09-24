#!/usr/bin/env bash
set -Eeuo pipefail
# ROS setup scripts can access unset variables. Restore nounset afterwards.
set +u
# shellcheck disable=SC1091
source /opt/ros/iron/setup.bash
if [[ -f /ws/install/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /ws/install/setup.bash
fi
set -u
exec "$@"
