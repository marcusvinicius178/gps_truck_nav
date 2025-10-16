#!/usr/bin/env bash
set -Eeuo pipefail
shopt -s inherit_errexit || true

export ROS_DISTRO="${ROS_DISTRO:-iron}"
export ROSDEP_AUTO_INSTALL="${ROSDEP_AUTO_INSTALL:-0}"
export BUILD_GZ_CLASSIC="${BUILD_GZ_CLASSIC:-0}"

mkdir -p /ws/src /ws/build /ws/install /ws/log

if [[ "$(id -u)" == "0" ]]; then
  chown -R 1000:1000 /ws || true
fi

# (Ambiente do ROS também é carregado por /etc/profile.d e bashrc)
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
fi

# >>>>> ACRÉSCIMO: também dar source do workspace instalado, se existir
if [[ -f "/ws/install/setup.bash" ]]; then
  set +u
  source "/ws/install/setup.bash"
  set -u
fi

# Paths do Gazebo "de fábrica"
if command -v gazebo >/dev/null 2>&1; then
  export GAZEBO_MODEL_DATABASE_URI="${GAZEBO_MODEL_DATABASE_URI:-}"
  export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-/usr/share/gazebo-11/models}"
  export GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH:-/usr/share/gazebo-11:/usr/share/gazebo-11/worlds}"
fi

# >>>>> ACRÉSCIMO: acrescentar modelos/worlds do pacote instalado (se existirem)
# Isso garante que 'model://arocs_truck' e seus assets sejam encontrados.
_add_if_dir() {
  local d="$1"; local var="$2"
  if [[ -d "$d" ]]; then
    # evita duplicar path
    if [[ ":${!var}:" != *":$d:"* ]]; then
      export "${var}=$d:${!var}"
    fi
  fi
}

_add_if_dir "/ws/install/truck_bringup/share/truck_bringup/models"   "GAZEBO_MODEL_PATH"
_add_if_dir "/ws/install/truck_bringup/share/truck_bringup/worlds"   "GAZEBO_RESOURCE_PATH"
_add_if_dir "/ws/install/truck_bringup/share/truck_bringup/media"    "GAZEBO_RESOURCE_PATH"
_add_if_dir "/ws/install/truck_bringup/share/truck_bringup/materials" "GAZEBO_RESOURCE_PATH"

# Estabilidade de GUI (opcional, honrando o que vier por docker-compose)
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"

export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-$(nproc)}"

if [[ "${ROSDEP_AUTO_INSTALL}" == "1" ]]; then
  if [[ -n "$(find /ws/src -mindepth 1 -maxdepth 1 2>/dev/null)" ]]; then
    apt-get update || true
    rosdep update || true
    rosdep install \
      --rosdistro "${ROS_DISTRO}" \
      --from-paths /ws/src \
      --ignore-src \
      --reinstall \
      -y || true
  fi
fi

exec "$@"
