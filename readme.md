# TinyNomad
[![Build Status](https://travis-ci.org/arunumd/TinyNomad.svg?branch=master)](https://travis-ci.org/arunumd/TinyNomad)
[![Coverage Status](https://coveralls.io/repos/github/arunumd/TinyNomad/badge.svg?branch=master)](https://coveralls.io/github/arunumd/TinyNomad?branch=master)
---

## Overview

A frontier exploration project using Gazebo, ROS, CMake, C++, RViz and GMapping. The exploring agent is a turtlebot simulated in an environment with unknown obstacles and topology. The robot makes random moves and learns the environment from sensor scan data. Successive moves are made intelligently based on the posterior probabilities of obstacles and free space. Exploration terminates when there is no scope for further unexplored frontiers.

## License
This software is protected by MIT License. For more details refer [MIT License](LICENSE)

## How to run the project ?
```
git clone https://github.com/arunumd/TinyNomad
cd TinyNomad
mkdir build
cd build
cmake ..
make
Run tests: ./test/cpp-test
Run program: ./app/shell-app
```


