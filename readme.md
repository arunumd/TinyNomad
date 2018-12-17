# TinyNomad
[![Build Status](https://travis-ci.org/arunumd/TinyNomad.svg?branch=Backup)](https://travis-ci.org/arunumd/TinyNomad)
[![Coverage Status](https://coveralls.io/repos/github/arunumd/TinyNomad/badge.svg?branch=Backup)](https://coveralls.io/github/arunumd/TinyNomad?branch=Backup)
---

## Overview

A frontier exploration project using Gazebo, ROS, CMake, C++, RViz and GMapping. The exploring agent is a turtlebot simulated in an environment with unknown obstacles and topology. The robot makes random moves and learns the environment from sensor scan data. Successive moves are made intelligently based on the posterior probabilities of obstacles and free space. Exploration terminates when there is no scope for further unexplored frontiers.

## License
This software is protected by MIT License. For more details refer [MIT License](LICENSE)

## Dependencies
The project depends on the following libraries and environments:
 - ROS Version Kinetic (Full Installation)
 - GCC Compiler - Version 5 or higher
 - ROS Gazebo
 - ROS Turtlebot (All related packages)
 - ROS GMapping
 - CMake

## Solo Iterative Process(SIP) Log
The following [link](https://drive.google.com/file/d/153M1aiCYegG0zRtJ9Sf43AjYE1P9QTQr/view?usp=sharing) contains the SIP log sheet with product backlog, iteration backlog and activity log.

## Sprint Planning Notes
The following [link](https://drive.google.com/file/d/1VqYlH_pGjYxlB-IP6AsRIp-IpPvnU8js/view?usp=sharing) contains the dynamic notes taken in respect to this project planning and execution.

## How to run the project ?
```
mkdir catkin_ws
cd catkin_ws
mkdir src
catkin_make
cd src
git clone https://github.com/arunumd/TinyNomad
cd ..
catkin_make
source devel/setup.bash
roslaunch tinynomad tinynomad.launch
```


