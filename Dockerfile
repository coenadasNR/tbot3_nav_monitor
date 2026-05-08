# ══════════════════════════════════════════════════════════════════════════════
# Stage 1 — builder
#   Installs build tools, fetches external assets, compiles the ROS2 package.
#   Nothing from this stage reaches the final image except the build output.
# ══════════════════════════════════════════════════════════════════════════════
FROM osrf/ros:humble-desktop AS builder

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-cartographer-ros \
    ros-humble-turtlebot3-cartographer \
    ros-humble-rviz2 \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Fetch OSRF Gazebo models required by the house world (sparse clone — ~4 models)
RUN git clone --depth 1 --filter=blob:none --sparse \
        https://github.com/osrf/gazebo_models.git /tmp/gazebo_models \
    && cd /tmp/gazebo_models \
    && git sparse-checkout set cafe_table first_2015_trash_can mailbox table_marble \
    && cp -r cafe_table first_2015_trash_can mailbox table_marble \
              /usr/share/gazebo-11/models/ \
    && cd / && rm -rf /tmp/gazebo_models

COPY src/    /ros2_ws/src/
COPY worlds/ /ros2_ws/worlds/
COPY maps/   /ros2_ws/maps/

WORKDIR /ros2_ws
# colcon build (no --symlink-install) produces a self-contained install/ tree —
# no symlinks back to src/, so the runtime stage needs no source files at all.
RUN . /opt/ros/humble/setup.sh && colcon build


# ══════════════════════════════════════════════════════════════════════════════
# Stage 2 — runtime
#   Lean image: no build tools, no git, no colcon cache.
#   Only the compiled package, runtime ROS2 deps, and simulation assets.
# ══════════════════════════════════════════════════════════════════════════════
FROM osrf/ros:humble-desktop AS runtime

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    TURTLEBOT3_MODEL=burger \
    QT_X11_NO_MITSHM=1 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    MESA_GL_VERSION_OVERRIDE=4.5

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-cartographer-ros \
    ros-humble-turtlebot3-cartographer \
    ros-humble-rviz2 \
    python3-pip \
    x11-apps \
    libgl1-mesa-glx \
    libxrender1 \
    libsm6 \
    libxext6 \
    libxkbcommon-x11-0 \
    libxcb-xinerama0 \
    && pip3 install --no-cache-dir flask flask-cors pandas numpy \
    && rm -rf /var/lib/apt/lists/*

# Gazebo models fetched in builder — no git required at runtime
COPY --from=builder /usr/share/gazebo-11/models/cafe_table           /usr/share/gazebo-11/models/cafe_table
COPY --from=builder /usr/share/gazebo-11/models/first_2015_trash_can /usr/share/gazebo-11/models/first_2015_trash_can
COPY --from=builder /usr/share/gazebo-11/models/mailbox              /usr/share/gazebo-11/models/mailbox
COPY --from=builder /usr/share/gazebo-11/models/table_marble         /usr/share/gazebo-11/models/table_marble

# Compiled package (self-contained — no src/ needed)
COPY --from=builder /ros2_ws/install /ros2_ws/install
# Simulation assets
COPY --from=builder /ros2_ws/worlds  /ros2_ws/worlds
COPY --from=builder /ros2_ws/maps    /ros2_ws/maps

COPY scripts/ros_entrypoint.sh /ros_entrypoint.sh
COPY scripts/                  /ros2_ws/scripts/
RUN chmod +x /ros_entrypoint.sh /ros2_ws/scripts/*.sh

WORKDIR /ros2_ws
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/ros_entrypoint.sh"]
