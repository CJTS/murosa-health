ARG FROM_IMAGE=docker.io/library/ros:humble
ARG OVERLAY_WS=/opt/ros/overlay_ws


# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# clone overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY src/ src/

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
  find ./ -name "package.xml" | \
  xargs cp --parents -t /tmp/opt && \
  find ./ -name "COLCON_IGNORE" | \
  xargs cp --parents -t /tmp/opt || true

# multi-stage for building
FROM $FROM_IMAGE AS builder

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
  apt-get update && rosdep install -y -r -q --from-paths \
  --from-paths \
  src \
  --ignore-src \
  --rosdistro $ROS_DISTRO \
  && rm -rf /var/lib/apt/lists/*

RUN apt update && apt upgrade -y && apt install -y \
    ros-$ROS_DISTRO-plansys2-* \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-turtlebot3* \
    ros-$ROS_DISTRO-rosbridge-server \
    python3 \
    python3-pip \
    openjdk-17-jdk \
    netcat

# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
ARG OVERLAY_MIXINS="release"

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
  colcon build \
  --packages-select \
  interfaces \
  murosa_plan \
  --symlink-install \
  --mixin $OVERLAY_MIXINS

# source entrypoint setup
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
  '$isource "$OVERLAY_WS/install/setup.bash"' \
  /ros_entrypoint.sh

RUN pip3 install -r ./src/murosa_plan/requirements.txt

COPY jason/ jason/
COPY gradle-7.4/ /app/gradle-7.4

ENV GRADLE_HOME=/app/gradle-7.4
ENV PATH=$PATH:$GRADLE_HOME/bin
ENV PYTHONPATH "${PYTHONPATH}:./src/murosa_plan/murosa_plan"
ENV JAVA_HOME /usr/lib/jvm/java-17-openjdk-arm64/