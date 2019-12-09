# Turtlebot Navigator

[![Build Status](https://travis-ci.org/arp95/enpm808x_turtlebot_navigator.svg?branch=master)](https://travis-ci.org/arp95/enpm808x_turtlebot_navigator)
[![Coverage Status](https://coveralls.io/repos/github/arp95/enpm808x_turtlebot_navigator/badge.svg?branch=master)](https://coveralls.io/github/arp95/enpm808x_turtlebot_navigator?branch=master)
[![Packagist](https://img.shields.io/packagist/l/doctrine/orm.svg)](LICENSE.md)
---

## About the Authors

Shantam Bajpai: I am a first year graduate student pursuing my masters degree in Robotics from the University of Maryland, College Park. Prior to joining University of Maryland I completed my undergraduate in Electrical and Electronics Engineering from Vellore Institute of Technology, Vellore, India. 

Arpit Aggarwal: I am a first year graduate student pursuing my masters degree in Robotics from the University of Maryland, College Park. Prior to joining University of Maryland I completed my undergraduate in Electrical Engineering from Delhi Technological University, Delhi, India.

## Overview of the project

Turtlebots are small robots that can drive around and sense their environment through a microsoft kinect sensor mounted on its body. This is an inspection robot for Acme Robotics which acts as an automated surveillance system providing security in a variety of environments. The robot navigates an unknown indoor environment and creates a real time 3-D map of the environment in the form of a point cloud. The robot uses the turtlebot package and the octomap package to develop the 3D map. Using the 3-D map of the environment, the turtlebot learns the environment especially where are the obstacles located so that it can report those anomalies and rotate to further navigate the environment.

## License
```
MIT License

Copyright (c) 2019 Arpit Aggarwal Shantam Bajpai

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
```

## Dependencies

The following dependencies are required to run this package:

1. ROS Kinetic
2. catkin (http://wiki.ros.org/catkin#Installing_catkin)
3. Ubuntu 16.04 For installing ROS (http://wiki.ros.org/kinetic/Installation)
4. Octomap package (sudo apt-get install ros-kinetic-octomap)
5. Turtlebot Gazebo (sudo apt-get install ros-kinetic-turtlebot-gazebo)
6. Turtlebot apps (sudo apt-get install ros-kinetic-turtlebot-apps)
7. Turtlebot rviz launcher (sudo apt-get install ros-kinetic-turtlebot-rviz-launchers)
8. roscpp
9. rostest
10. rosbag
11. geometry_msgs
12. std_msgs
13. sensor_msgs
14. octovis (sudo apt-get install ros-kinetic-octovis)

## Standard install via command-line
```
cd ~/catkin_ws/src
git clone --recursive https://github.com/arp95/enpm808x_turtlebot_navigator
cd ..
catkin_make
```

## Run the demo

To run the demo and not record the bag file, follow the following steps:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch turtlebot_navigator demo.launch
```

To run the demo and record the bag file, follow the steps below:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch turtlebot_navigator demo.launch record:=true
```

While the demo.launch file is running, open another terminal for generating the map and type the following command:
```
rosrun octomap_server octomap_saver src/turtlebot_navigator/results/map.ot
```

To view the generated map, type the following command in the new terminal:
```
octovis src/turtlebot_navigator/results/map.ot
```

This will open up gazebo and rviz. The gazebo window will show the turtlebot navigating the environment and rotate, if an obstacle occurs. The rviz window shows the octomap generation as the turtlebot navigates the environment.

## Results

<p align="center">
  <img width="700" height="500" src="https://github.com/arp95/enpm808x_turtlebot_navigator/blob/master/results/demo.png">
</p>

## Presentation
The slides for the presentation can be found [here](https://docs.google.com/presentation/d/1AY0yj8zdrGQnezmRA_y8tVwdGh79-ZWhLBnZEmOYTUA/edit?usp=sharing)

## Running Tests
Unit tests have been written for each class to test the functionality and interface using gtest and rostest. To run rostest follow the given steps:
```
cd ~/catkin_ws/
source devel/setup.bash
catkin_make run_tests_turtlebot_navigator
```

Also, testing can be done through the test.launch file as follows:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch turtlebot_navigator test.launch
```

Another method of testing can be done by following the steps below:
```
cd ~/catkin_ws/
source devel/setup.bash
rostest turtlebot_navigator test.launch
```

## Print information in the bag file

To print the information of the bag file, follow the steps below:
```
cd ~/catkin_ws/
source devel/setup.bash
cd src/turtlebot_navigator/
rosbag info results/turtlebot_navigator.bag
```

## Run the bag file

To play the bag file, follow the steps below:
```
cd ~/catkin_ws/
source devel/setup.bash
cd src/turtlebot_navigator/
rosbag play results/turtlebot_navigator.bag
```

## Inspecting the bag file
To inspect the bag file, follow the steps below:
```
cd ~/catkin_ws/
source devel/setup.bash
cd src/turtlebot_navigator/
rqt_bag results/turtlebot_navigator.bag
```

## Doxygen Documentation
The doxygen generated documents have been added to the docs folder of the repository. A config file named 'Doxyfile' has been added to generate the documentation.
To generate the doxygen documentation, follow the steps below:
```
cd ~/catkin_ws/
source devel/setup.bash
cd src/turtlebot_navigator/
doxygen Doxyfile
```

## Code Coverage
The current code coverage for the package is 82%.
```
cd ~/catkin_ws/build
lcov --directory . --capture --output-file coverage.info
lcov --list coverage.info
```
This will output the coverage of each file in the terminal. To create an html file for the same, run the following command:
```
genhtml coverage.info --output-directory covout
```
This will store the index.html file in the folder covout.

## Agile Iterative Process
This package was developed following pair programming concepts and AIP. The estimated and completed tasks have been stored in the form of product backlog. Arpit Aggarwal and Shantam Bajpai worked together on this implementation and the commits were made by the driver while the other person acted as the navigator. The AIP can be accessed from the link below.
[![Solo Iterative Process](https://img.shields.io/badge/AIP-ClickHere-brightgreen.svg?style=flat)](https://docs.google.com/spreadsheets/d/1Gf2HPhlzFCxhdOP1XlNDx23QuuKKL7okn-5ru6TXR6c/edit?usp=sharing)

Sprint notes can be found [here](https://docs.google.com/document/d/1FklToC6_Twc2enWLIpCELDpGJxoGM9w2i3ojj_iZZNQ/edit?usp=sharing)

## Known bugs
The octomap uses the /camera/depth/points topic to create the 3D octomap of the environment using the point cloud. Use of ROSBAG for camera topics is inefficient as the bag file increases in size exponentially within a few seconds. Alternative ways need to be found out to record camera topics.
