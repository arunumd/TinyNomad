/**************************************************************************************************
* @file      : main.cpp
* @author    : Arun Kumar Devarajulu
* @date      : December 9, 2018
* @copyright : 2018, Arun Kumar Devarajulu
* @license   : MIT License
*
*              Permission is hereby granted, free of charge, to any person obtaining a copy
*              of this software and associated documentation files (the "Software"), to deal
*              in the Software without restriction, including without limitation the rights
*              to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*              copies of the Software, and to permit persons to whom the Software is
*              furnished to do so, subject to the following conditions:
*
*              The above copyright notice and this permission notice shall be included in all
*              copies or substantial portions of the Software.
*
*              THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*              IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*              FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*              AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*              LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*              OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*              SOFTWARE.
*
* @brief     : The main.cpp file contains the implementation of the autonomous exploration of
*              unknown environment/ frontier detection project. This project is based on a gazebo
*              simulation of random exploration of an unknown environment. The turtlebot has a
*              laser scanner which continuously gives an environment feedback (distance detected
*              from obstacles present in the scene/ environment). Tha navigation is achieved by a
*              simple subscriber publisher implementation with the laserscan data and velocity
*              commands. The current scene is based on an outdoor environment in which there is a
*              restaurant and several driveways surrounding it. The robot starts at a random
*              location on the drive way and then starts to make moves based on laser feedback
*              about the surroundings/ obstacles. When there is an obstacle at a collision range
*              with the robot, the robot, stops and makes a rotational manuevre until its new
*              trajectory becomes obstacle free. Further to this, the robot also breaks
*              monotonicity in the straight movement by taking a random turn every 60 seconds.
*              The Gmapping package has been used to create dynamic SLAM and also create a
*              graphical visualization of the explored environment thus far.
****************************************************************************************************/
#include "Navigation.h"
#include "Turtlebot.h"
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "tinynomad");
    Turtlebot turtle = Turtlebot();
    ros::Duration(10).sleep();
    ros::Rate loop_rate(5);
    while (ros::ok()) {
        turtle.drive();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
