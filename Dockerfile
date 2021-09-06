# Example build command:
# export FROM_IMAGE="ros:foxy"
# export OVERLAY_MIXINS="release ccache"
# docker build -t chapulina/dolly:foxy \
#   --build-arg FROM_IMAGE \
#   --build-arg OVERLAY_MIXINS \
#   -f Dockerfile ./
ARG FROM_IMAGE=ros:foxy
ARG OVERLAY_WS=/opt/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# clone underlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
COPY ./ ./chapulina/dolly

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# multi-stage for building
FROM $FROM_IMAGE AS builder
ARG DEBIAN_FRONTEND=noninteractive

# install CI dependencies
RUN apt-get update && apt-get install -q -y \
        ccache \
        lcov \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep update

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
ARG OVERLAY_MIXINS="release ccache"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --mixin $OVERLAY_MIXINS \
      --symlink-install

# source overlay entrypoint
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place \
      's|^source .*|source "$OVERLAY_WS/install/setup.bash"|' \
      /ros_entrypoint.sh

# test overlay build
ARG RUN_TESTS
ARG FAIL_ON_TEST_FAILURE=Ture
RUN if [ -n "$RUN_TESTS" ]; then \
        . $OVERLAY_WS/install/setup.sh && \
        colcon test \
        && colcon test-result \
          || ([ -z "$FAIL_ON_TEST_FAILURE" ] || exit 1) \
    fi
