[![Build Status](https://travis-ci.org/chapulina/dolly.svg?branch=master)](https://travis-ci.org/chapulina/dolly)

# Dolly the robot

_It's a sheep, it's a dolly, it's a following robot. Born to be cloned._

Packages for launching Dolly demo, which uses ROS 2 and either
[Gazebo](https://gazebosim.org) or [Ignition](https://ignitionrobotics.org).

Gazebo | Ignition
-- | --
![Dolly Gazebo](images/dolly.gif) | ![Dolly Ignition](images/dolly_ign.gif)

Dolly has been tested on:

* ROS 2 version:
    * ROS Crystal: `crystal` branch
    * ROS Dashing: `dashing` branch
    * ROS Eloquent: `master` branch
* Gazebo version:
    * Gazebo 9
* Ignition version:
    * Citadel
* Operating system:
    * Ubuntu Bionic
    * OSX Sierra (thanks, @Karsten1987 !)

## Install

Install instructions for Ubuntu Bionic.

1. Install at least one simulator,
   [Gazebo](http://gazebosim.org/tutorials?cat=install) or
   [Ignition](https://ignitionrobotics.org/docs/citadel/install)

1. Install the appropriate ROS 2 version as instructed
   [here](https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians/).

1. Clone Dolly:

        mkdir -p ~/ws/src
        cd ~/ws/src
        git clone https://github.com/chapulina/dolly

1. Ignition only: clone the bridge:

        git clone https://github.com/ignitionrobotics/ros_ign -b dashing

1. Install dependencies:

        cd ~/ws
        rosdep install --from-paths src --ignore-src -r -y \
            --skip-keys=ignition-math6 \
            --skip-keys=ignition-msgs5 \
            --skip-keys=ignition-transport8 \
            --skip-keys=ignition-gazebo3

1. Build and install:

        cd ~/ws
        colcon build

## Run

### Gazebo-classic

If you had Gazebo installed when compiling Dolly's packages, Gazebo support
should be enabled.

1. Setup environment variables (the order is important):

        . /usr/share/gazebo/setup.sh
        . ~/ws/install/setup.bash

    > *Tip*: If the command `ros2 pkg list | grep dolly_gazebo` comes up empty
      after setting up the environment, Gazebo support wasn't correctly setup.

1. Launch Dolly in a city (this will take some time to download models):

        ros2 launch dolly_gazebo dolly.launch.py world:=dolly_city.world

1. Launch Dolly in an empty world:

        ros2 launch dolly_gazebo dolly.launch.py world:=dolly_empty.world

### Ignition

1. Setup environment variables:

        . ~/ws/install/setup.bash

    > *Tip*: If the command `ros2 pkg list | grep dolly_ignition` comes up empty
      after setting up the environment, Ignition support wasn't correctly setup.

1. Launch Dolly in a station:

        ros2 launch dolly_ignition dolly.launch.py

## Packages

This repository contains the following packages:

* `dolly`: Metapackage which provides all other packages.
* `dolly_follow`: Provides node with follow logic.
* `dolly_gazebo`: Robot model, simulation world and launch scripts for Gazebo-classic.
* `dolly_ignition`: Robot model, simulation world and launch scripts for Ignition.

## Featured

* QConSF 2018
    * [Video](https://www.youtube.com/watch?v=Gwbk6Qf_TqY)
    * [Code](https://github.com/chapulina/simslides/tree/QConSF_Nov2018)
* [InfoQ: Open Source Robotics: Getting Started with Gazebo and ROS 2](https://www.infoq.com/articles/ros-2-gazebo-tutorial/)
* [ROS Developers LIVE Class #70: How to Control a Robot with ROS2 (Dashing)](https://www.youtube.com/watch?v=qB4SaP3TZog)
* ROSConJP 2019
    * [Video](https://vimeo.com/370247782)
    * [Code](https://github.com/chapulina/rosconjp_2019)

