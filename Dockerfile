# Base com ROS 2 Iron (desktop completo)
FROM osrf/ros:iron-desktop-full

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive \
    PIP_ROOT_USER_ACTION=ignore

# ------------------------------------------------------------------
# Ferramentas de build + colcon + vcstool + rosdep
# + Infra de testes (ament/pytest/gtest)
# + Gazebo Classic + gazebo_ros_pkgs (Nav2 system tests)
# + geographic_msgs, test_msgs, bond/bondcpp, robot_localization
# + GraphicsMagick++ e Ceres
# + ament_cmake_core
# + BehaviorTree (nav2_behavior_tree)
# + XTensor stack (nav2_mppi_controller)
# + OMPL (nav2_smac_planner)
# + MAPVIZ + plugins (para o teu launch)
# ------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl git \
      build-essential cmake gdb \
      python3-pip python3-colcon-common-extensions python3-vcstool python3-rosdep \
      ros-iron-ament-cmake ros-iron-ament-cmake-core \
      ros-iron-ament-cmake-gtest ros-iron-ament-cmake-pytest \
      ros-iron-launch ros-iron-launch-testing ros-iron-launch-testing-ament-cmake \
      ros-iron-test-msgs \
      gazebo \
      ros-iron-gazebo-ros-pkgs \
      ros-iron-gazebo-msgs \
      ros-iron-gazebo-plugins \
      ros-iron-geographic-msgs \
      ros-iron-bond ros-iron-bondcpp ros-iron-smclib \
      ros-iron-robot-localization \
      libgraphicsmagick++-dev \
      libceres-dev \
      libopenblas-dev \
      libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# BehaviorTree.CPP (preferir pacote ROS; senão, pacote do sistema)
RUN apt-get update && ( \
      apt-get install -y --no-install-recommends ros-iron-behaviortree-cpp-v3 \
      || apt-get install -y --no-install-recommends libbehaviortree-cpp-v3-dev \
    ) && rm -rf /var/lib/apt/lists/*

# XTensor stack (nomes corretos no Jammy; xtl varia por mirror → fallback)
RUN apt-get update && apt-get install -y --no-install-recommends \
      libxtensor-dev \
      libxtensor-blas-dev \
      libxsimd-dev \
    && (apt-get install -y --no-install-recommends xtl \
        || apt-get install -y --no-install-recommends libxtl-dev \
        || true) \
    && rm -rf /var/lib/apt/lists/*

# OMPL (resolve find_package(ompl) no nav2_smac_planner)
RUN apt-get update && ( \
      apt-get install -y --no-install-recommends ros-iron-ompl \
      || apt-get install -y --no-install-recommends libompl-dev \
    ) && rm -rf /var/lib/apt/lists/*

# MAPVIZ + plugins
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-iron-mapviz \
      ros-iron-mapviz-plugins \
      ros-iron-tile-map \
      ros-iron-swri-transform-util \
    && rm -rf /var/lib/apt/lists/*

# rosdep e auto-source
RUN rosdep init || true && rosdep update
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash 2>/dev/null || true' > /etc/profile.d/ros_setup.sh && \
    chmod +x /etc/profile.d/ros_setup.sh && \
    echo 'source /etc/profile.d/ros_setup.sh' >> /etc/bash.bashrc

# Usuário e workspace
RUN groupadd --gid 1000 ros || true && \
    useradd  --uid 1000 --gid 1000 --create-home --shell /bin/bash ros || true
WORKDIR /ws

# Entrypoint (fica dentro da imagem com +x)
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
