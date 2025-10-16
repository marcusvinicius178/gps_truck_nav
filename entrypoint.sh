#!/usr/bin/env bash
set -Eeuo pipefail
shopt -s inherit_errexit || true

export ROS_DISTRO="${ROS_DISTRO:-iron}"
export ROSDEP_AUTO_INSTALL="${ROSDEP_AUTO_INSTALL:-0}"
export BUILD_GZ_CLASSIC="${BUILD_GZ_CLASSIC:-0}"

# Pastas do workspace
mkdir -p /ws/src /ws/build /ws/install /ws/log

# Ajuste de permissões na 1ª execução (quando rodando como root)
if [[ "$(id -u)" == "0" ]]; then
  chown -R 1000:1000 /ws || true
fi

# (Ambiente do ROS também é carregado por /etc/profile.d e bashrc)
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
fi

# Variáveis úteis do Gazebo Classic
if command -v gazebo >/dev/null 2>&1; then
  export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-/usr/share/gazebo-11/models}"
  export GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH:-/usr/share/gazebo-11:/usr/share/gazebo-11/worlds}"
fi

# Paralelismo
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-$(nproc)}"

# Auto-instalar dependências (inclui test_depend) de tudo em /ws/src
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
