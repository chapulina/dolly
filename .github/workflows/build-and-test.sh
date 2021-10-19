#!/bin/bash
set -ev
set -x

# Configuration.
export COLCON_WS=~/ws
export COLCON_WS_SRC=${COLCON_WS}/src
export DEBIAN_FRONTEND=noninteractive
export ROS_PYTHON_VERSION=3

apt update -qq
apt install -qq -y lsb-release wget curl build-essential

# Fortres isn't on packages.ros.org yet, so we get it from packages.osrfoundation.org
# Once it's on packages.ros.org, it can be installed with rosdep below
if [ "$IGNITION_VERSION" == "fortress" ]; then
  echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list
  wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

  IGN_DEPS="ignition-fortress"
fi

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
colcon build --event-handlers console_direct+

# Test
colcon test --event-handlers console_direct+ --packages-select-regex dolly
colcon test-result
