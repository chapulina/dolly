# Dolly the robot

Packages for launching Dolly demo, which uses Gazebo and ROS 2.

## Install

Dolly needs Gazebo 9 + ROS 2 Crystal. Tested on Ubuntu Bionic.

1. Install ROS Crystal as instructed [here](https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians/).

1. Clone Dolly:

        mkdir -p ~/ws/src
        cd ~/ws/src
        git clone https://github.com/chapulina/dolly

1. Install ROS dependencies like `gazebo_ros_pkgs`, which also installs Gazebo:

        cd ~/ws
        rosdep install --from-paths src --ignore-src -r -y

1. Build and install:

        cd ~/ws
        colcon build

## Run

1. Setup environment variables:

        . ~/ws/install/setup.bash
        export GAZEBO_RESOURCE_PATH=/home/`whoami`/ws/src/dolly/dolly-gazebo/worlds:${GAZEBO_RESOURCE_PATH}
        export GAZEBO_MODEL_PATH=/home/`whoami`/ws/src/dolly/dolly-gazebo/models:${GAZEBO_MODEL_PATH}

1. Launch Dolly's world:

        ros2 launch dolly-gazebo dolly.launch.py world:=dolly.world

## Packages

This repository contains 2 packages:

* `dolly_follow`: Provides node with follow logic.
* `dolly_gazebo`: Robot model, simulation world and launch scripts.

# TODO

* Set Gazebo paths from launch file
* Make Dolly's model available to RViz

