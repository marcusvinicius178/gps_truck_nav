# Historical target: Ubuntu 22.04 / ROS 2 Iron / Gazebo Classic 11.
# Public integration environment, not an image of the private benchmark fork.
FROM osrf/ros:iron-desktop-full
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
    && rm -rf /var/lib/apt/lists/*
ENV GAZEBO_MODEL_DATABASE_URI=""
WORKDIR /ws
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
