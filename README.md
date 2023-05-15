# Introduction

ROS2 & Zenoh-flow project to carry out a cooperative object search between various robots conforming a robot swarm.

This repository contains a [ROS2 foxy](https://docs.ros.org/en/foxy/index.html) package to launch the simulation ([Gazebo](https://gazebosim.org/home)), RViz, and [Nav2](https://navigation.ros.org/) nodes appart from the [Zenoh-flow](https://zenoh.io/blog/2023-02-10-zenoh-flow/) nodes.

There's more information about this project and how to run it in the [getting started section](#getting-started).

All the nodes in this project were tested on Ubuntu 20.04.1 x86_64 GNU/Linux and ROS2.

# Requirements

 * ROS2:

     - [ROS2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation.html) distribution.

 * Simulator:

     - [Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)

 * Libraries and packages:

     - [OpenCV](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html) library (for python).

     - [cv_bridge](https://github.com/ros-perception/vision_opencv/blob/foxy/cv_bridge/README.md#installation) library (for python).

     - [Nav2](https://navigation.ros.org/getting_started/index.html#installation).

 * Zenoh and Zenoh-flow related:

     - [Zenoh-bridge-dds](https://github.com/eclipse-zenoh/zenoh-plugin-dds#readme)

     - [Zenoh router](https://zenoh.io/docs/getting-started/installation/)

     - [Zenoh-flow](https://github.com/eclipse-zenoh/zenoh-flow#readme)

     - [Zenoh-flow-python](https://github.com/eclipse-zenoh/zenoh-flow-python#readme)

# Getting started

All the nodes and programs of this package were tested using Ubuntu 20.04 x84_64 GNU/Linux and ROS2 Foxy Fitzroy distro.

More information about the process to launch the nodes in the [wiki](https://github.com/USanz/swarm_obj_finder/wiki).
