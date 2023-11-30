/**
 * @file walker_node.hpp
 * @author Abhishekh Reddy (areddy42@umd.edu)
 * @brief Header file for the Turtlebot walker node class that contains the
 *        declarations.
 * @version 1.0
 * @date 2023-11-29
 *
 * @copyright Copyright (c) 2023 Abhishekh Reddy
 *
 */
#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

/**
 * @brief RCLCPP node that walks the Turtlebot through a given world environment
 *
 */
class Walker : public rclcpp::Node {
 public:
 /**
  * @brief Constructor for the walker node
  *
  */
  /**
   * @brief Direction label for using the turn methods
   *
   */
  enum direction {left = 'L', right = 'R'};

 private:
  /**
   * @brief Main logic of the walker that runs at a regular period
   *
   */
  void timerCallback();

  /**
   * @brief Saves the scanned ranges from LiDAR to a private member
   *
   * @param data
   */
  void scannerCallback(const sensor_msgs::msg::LaserScan& data);

  /**
   * @brief Returns true if an obstacle is detected in the forward left region
   *        of the robot
   *
   * @return true
   * @return false
   */
  bool robotSenseLeft();

  /**
   * @brief Returns true if an obstacle is detected in the forward right region
   *        of the robot
   *
   * @return true
   * @return false
   */
  bool robotSenseRight();

  /**
   * @brief Method to turn the robot left or right when an obstacle is detected
   *
   * @param direction
   */
  void robotTurn(char direction);

  /**
   * @brief Publishes the velocities to move the Turtlebot
   *
   */
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr walker_;

  /**
   * @brief Subscribes to the scanned range messages from the LiDAR
   *
   */
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanner_;

  /**
   * @brief Runs the main logic of the walker at a regular interval
   *
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Data from the LiDAR
   *
   */
  sensor_msgs::msg::LaserScan scan_;
};
