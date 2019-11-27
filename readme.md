# Turtlebot Navigator

[![Build Status](https://travis-ci.org/arp95/enpm808x_turtlebot_navigator.svg?branch=master)](https://travis-ci.org/arp95/enpm808x_turtlebot_navigator)
[![Coverage Status](https://coveralls.io/repos/github/arp95/enpm808x_turtlebot_navigator/badge.svg?branch=master)](https://coveralls.io/github/arp95/enpm808x_turtlebot_navigator?branch=master)
[![Packagist](https://img.shields.io/packagist/l/doctrine/orm.svg)](LICENSE.md)
---

## About the Authors

Shantam Bajpai: I am a first year graduate student pursuing my masters degree in Robotics from the University of Maryland, College Park. Prior to joining University of Maryland I completed my undergraduate in Electrical and Electronics Engineering from Vellore Institute of Technology, Vellore, India.

Arpit Aggarwal: I am a first year graduate student pursuing my masters degree in Robotics from the University of Maryland, College Park. Prior to joining University of Maryland I completed my undergraduate in Electrical Engineering from Delhi Technological University, Delhi, India.

## Overview of the project

Turtlebots are small robots that can drive around and sense their environment through a microsoft kinect sensor mounted on its body. This is an inspection robot for Acme Robotics which acts as an automated surveillance system providing security in a variety of environments. The robot navigates an unknown indoor environment and creates a real time 3-D map of the environment in the form of a point cloud using SLAM(Simultaneous Localization and Mapping). The robot uses the turtlebot package and the octomap package to develop the 3D map. 

## Dependencies

The following dependencies are required to run this package:

1. ROS kinetic
2. catkin (http://wiki.ros.org/catkin#Installing_catkin)
3. Ubuntu 16.04 For installing ROS (http://wiki.ros.org/kinetic/Installation)

## Standard install via command-line
```
cd ~/catkin_ws/src
git clone --recursive https://github.com/arp95/enpm808x_turtlebot_navigator
cd ..
catkin_make
```

## Agile Iterative Process
[![Solo Iterative Process](https://img.shields.io/badge/AIP-ClickHere-brightgreen.svg?style=flat)](https://docs.google.com/spreadsheets/d/1Gf2HPhlzFCxhdOP1XlNDx23QuuKKL7okn-5ru6TXR6c/edit?usp=sharing)
