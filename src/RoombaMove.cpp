/*
 * @file RoombaMove.cpp
 * @Copyright MIT license
 * Copyright (c) 2018 Bala Murali Manoghar Sai Sudhakar
 * @author Bala Murali Manoghar Sai Sudhakar
 * @brief This demonstrates simple sending of messages over the ROS system and depicts a listener node.
 */

/*
 * MIT License
 *
 * Copyright (c) 2018 Bala Murali Manoghar Sai Sudhakar
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <algorithm>
#include <vector>
#include <iterator>
#include <cmath>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "../include/RoombaMove.h"

RoombaMove::RoombaMove() {
  obstacle = false;
  minDist = 0;
}

void RoombaMove::nodeinit(int argc, char **argv) {
  ros::init(argc, argv, "simple_roomba");
  ROS_INFO_STREAM("Created Node node name : simple_roomba");
}

void RoombaMove::initScanSubscriber() {
  scanSubscriber = n.subscribe("scan", 100, &RoombaMove::scanCallBack, this);
  ROS_INFO_STREAM("Created subscriber to scan topic");
}

void RoombaMove::initTwistpublisher() {
  twistpublisher = n.advertise<geometry_msgs::Twist>(
      "/mobile_base/commands/velocity", 100);
  ROS_INFO_STREAM("Created publisher");
}

void RoombaMove::scanCallBack(
    const sensor_msgs::LaserScan::ConstPtr &scanData) {
  obstacle = false;
  minDist = *(scanData->ranges.begin());
  for (auto i : scanData->ranges) {
    if (i < minDist && !std::isnan(i)) {
      minDist = i;
    }
  }
  if (std::isnan(minDist)) {
    ROS_INFO_STREAM("No Obstacle in scan range");
  } else {
    ROS_INFO_STREAM("Min Distance : " << minDist);
  }

  if (minDist < scanData->range_min + 0.5 && !std::isnan(minDist)) {
    obstacle = true;
    ROS_WARN_STREAM("Obstacle Detected");
  }
  botMove(obstacle);
}

void RoombaMove::botMove(bool obstacle) {
  if (obstacle) {
    twist.linear.x = 0.0;
    twist.angular.z = 1.0;
  } else {
    twist.linear.x = 0.5;
    twist.angular.z = 0.0;
  }
  twistpublisher.publish(twist);
}

RoombaMove::~RoombaMove() {
}

