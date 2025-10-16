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
# + BehaviorTree (para nav2_behavior_tree)
# + XTensor stack (para nav2_mppi_controller)
# + OMPL (para nav2_smac_planner)
# ------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl git \
      build-essential cmake gdb \
      python3-pip python3-colcon-common-extensions python3-vcstool python3-rosdep \
      # infra de testes
      ros-iron-ament-cmake ros-iron-ament-cmake-core \
      ros-iron-ament-cmake-gtest ros-iron-ament-cmake-pytest \
      ros-iron-launch ros-iron-launch-testing ros-iron-launch-testing-ament-cmake \
      ros-iron-test-msgs \
      # Gazebo Classic + integração ROS
      gazebo \
      ros-iron-gazebo-ros-pkgs \
      ros-iron-gazebo-msgs \
      ros-iron-gazebo-plugins \
      # dependências adicionais
      ros-iron-geographic-msgs \
      ros-iron-bond ros-iron-bondcpp ros-iron-smclib \
      ros-iron-robot-localization \
      libgraphicsmagick++-dev \
      libceres-dev \
      # BLAS/Eigen p/ xtensor-blas
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

# Inicializa rosdep dentro da imagem (idempotente)
RUN rosdep init || true && rosdep update

# Auto-source do ROS para todo shell interativo e de login
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash 2>/dev/null || true' > /etc/profile.d/ros_setup.sh && \
    chmod +x /etc/profile.d/ros_setup.sh && \
    echo 'source /etc/profile.d/ros_setup.sh' >> /etc/bash.bashrc

# Usuário não-root (UID/GID 1000 para casar com host)
RUN groupadd --gid 1000 ros || true && \
    useradd  --uid 1000 --gid 1000 --create-home --shell /bin/bash ros || true

# Workspace
WORKDIR /ws

# Entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
