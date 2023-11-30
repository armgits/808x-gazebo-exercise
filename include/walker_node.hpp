#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

class Walker : public rclcpp::Node {
 public:
  Walker();

  enum direction {left = 'L', right = 'R'};

 private:
  void timerCallback();
  void scannerCallback(const sensor_msgs::msg::LaserScan& data);
  bool robotSenseLeft();
  bool robotSenseRight();
  void robotTurn(char direction);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr walker_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanner_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::LaserScan scan_;
};
