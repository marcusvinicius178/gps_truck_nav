# Discovery build for NEW prospective experiments; not the historical runtime.
# Supply an actual RepoDigest obtained by pulling the base image on the host.
ARG ROS_BASE
FROM ${ROS_BASE}
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git git-lfs \
    python3-colcon-common-extensions python3-rosdep python3-yaml python3-tk python3-numpy \
    ros-iron-ament-cmake-python ros-iron-ament-lint-auto ros-iron-ament-lint-common \
    ros-iron-ament-cmake-gtest ros-iron-ament-cmake-pytest ros-iron-launch-testing \
    ros-iron-navigation2 ros-iron-nav2-bringup \
    ros-iron-gazebo-ros-pkgs ros-iron-robot-localization \
    ros-iron-robot-state-publisher ros-iron-geographic-msgs \
    ros-iron-mapviz ros-iron-mapviz-plugins ros-iron-tile-map \
    ros-iron-swri-transform-util ros-iron-angles ros-iron-tf2-geometry-msgs \
    ros-iron-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*
ENV GAZEBO_MODEL_DATABASE_URI=""
ENV ROS_LOCALHOST_ONLY=1
ENV ROS_DOMAIN_ID=71
WORKDIR /ws
COPY src/gps_truck_nav/truck_bringup /ws/src/truck_bringup
RUN python3 -c "import sys; from pathlib import Path; meshes=list(Path('/ws/src/truck_bringup/models/arocs_truck/meshes').glob('*.dae')); pointers=[str(p) for p in meshes if p.open('rb').read(100).startswith(b'version https://git-lfs.github.com/spec/v1')]; sys.exit('Missing/unhydrated meshes: '+str(pointers) if not meshes or pointers else 0)"
RUN source /opt/ros/iron/setup.bash && \
    colcon build --base-paths src --packages-select truck_bringup \
      --executor sequential --cmake-args -DBUILD_TESTING=OFF && \
    dpkg-query -W > /ws/installed-packages.tsv
COPY config/auto_tuning /opt/auto_tuning/config
COPY tools/auto_tuning /opt/auto_tuning/tools
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
