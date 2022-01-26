#!/bin/bash
set -ev
set -x

# Configuration.
export COLCON_WS=~/ws
export COLCON_WS_SRC=${COLCON_WS}/src
export DEBIAN_FRONTEND=noninteractive
export ROS_PYTHON_VERSION=3

# For headless rendering
export DISPLAY=:1.0
export MESA_GL_VERSION_OVERRIDE=3.3

apt update -qq
apt install -qq -y lsb-release wget curl build-essential

# Tools and dependencies
echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
apt-get update -qq
apt-get install -y $IGN_DEPS \
                   python3-colcon-common-extensions \
                   python3-rosdep

rosdep init
rosdep update

# Create workspace and copy Dolly code there
mkdir -p $COLCON_WS_SRC
cp -r $GITHUB_WORKSPACE $COLCON_WS_SRC

# Install ROS dependencies
rosdep install --from-paths $COLCON_WS_SRC -i -y -r --rosdistro $ROS_DISTRO

# Build
source /opt/ros/$ROS_DISTRO/setup.bash
cd $COLCON_WS
colcon build --event-handlers console_direct+ --packages-up-to dolly

# Test
colcon test --event-handlers console_direct+ --packages-select-regex dolly
colcon test-result
