/*
 * @file RoombaMove.h
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

#ifndef SIMPLE_ROOMBA_SRC_ROOMBAMOVE_H_
#define SIMPLE_ROOMBA_SRC_ROOMBAMOVE_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

/**
 * @brief Roomba navigator class that drives the robot by avoiding obstacles
 */

class RoombaMove {
 private:
  // Node handle
  ros::NodeHandle n;
  // Subscriber for scan
  ros::Subscriber scanSubscriber;
  // Publisher to twist
  ros::Publisher twistpublisher;
  // Obstacle boolean that is true if there is obstacle
  bool obstacle;
  // Distance of nearest obstacle
  float minDist;
  // Twist message
  geometry_msgs::Twist twist;
 public:
  /**
   * @brief Roomba move constructor
   * @param None
   * @return None
   */
  RoombaMove();
  /**
   * @brief Initialize subscriber
   * @param None
   * @return None
   */
  void initScanSubscriber();
  /**
   * @brief Initialize publisher
   * @param None
   * @return None
   */
  void initTwistpublisher();
  /**
   * @brief Call back function for subscriber. It finds if there is
   * an obstacle in the path and correspondingly alters the path
   * @param scanData Pointer to message object
   * @return None
   */
  void scanCallBack(const sensor_msgs::LaserScan::ConstPtr &scanData);
  /**
   * @brief Checks if obstacle is present and moves the robot by publishing the values
   * @param scanData Pointer to message object
   * @return None
   */
  void botMove(bool obstacle);
  /**
   * @brief Roomba move destructor
   * @param None
   * @return None
   */
  virtual ~RoombaMove();
};

#endif /* SIMPLE_ROOMBA_SRC_ROOMBAMOVE_H_ */
