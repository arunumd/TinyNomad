/**************************************************************************************************
* @file      : Navigation.cpp
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
* @brief     : The Navigation.h file has class declarations for instructing turtlebot to move
*              straight when there are no obstacles while performing a random turn every 60 seconds;
*              and also make an obstacle evasion manuevre when there is an obstacle. The class uses
*              ros::Timer class and several callback methods for accomplishing its goals.
****************************************************************************************************/
#include "Navigation.h"
#include <random>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

Navigation::Navigation() : obstacleRange(0) {}

Navigation::~Navigation() {}

geometry_msgs::Twist Navigation::moveCommand() {
    drivePower.linear.x = 1.0;
    drivePower.linear.y = 0.0;
    drivePower.linear.z = 0.0;
    drivePower.angular.x = 0.0;
    drivePower.angular.y = 0.0;
    drivePower.angular.z = 0.0;
    return drivePower;
}

geometry_msgs::Twist Navigation::turnCommand() {
    std::random_device scramble;
    std::mt19937 num(scramble());
    std::uniform_int_distribution<int> dist(0, 360);
    float angle = dist(num);
    drivePower.linear.x = 0.0;
    drivePower.linear.y = 0.0;
    drivePower.linear.z = 0.0;
    drivePower.angular.x = 0.0;
    drivePower.angular.y = 0.0;
    drivePower.angular.z = angle * (3.14 / 180);
    return drivePower;
}

void Navigation::laserCallback(const sensor_msgs::LaserScan::ConstPtr& data) {
    float threshold = 25;
    for (const auto& dist : data->ranges) {
        ROS_INFO_STREAM("The actual range is : " << dist);
        if (threshold > dist) {
            threshold = dist;
        }
    }
    obstacleRange = threshold;
    ROS_INFO_STREAM("Approach distance to obstacle is : " << obstacleRange);
}

float Navigation::getObstacleRange() {
    return obstacleRange;
}
