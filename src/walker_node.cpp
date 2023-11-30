#include "walker_node.hpp"

using namespace std::chrono_literals;

Walker::Walker() : Node("walker") {
  walker_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  scanner_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10,
      std::bind(&Walker::scannerCallback, this, std::placeholders::_1)
  );

  timer_ = this->create_wall_timer(250ms, std::bind(&Walker::timerCallback, this));
}

void Walker::scannerCallback(const sensor_msgs::msg::LaserScan& data) {
  if (data.header.stamp.sec == 0)
    return;

  scan_ = data;
}

bool Walker::robotSenseLeft() {
  if (scan_.ranges.empty())
    return false;

  // Sense robot left
  for (auto d_i {scan_.ranges.begin()}; d_i < scan_.ranges.begin()+30; d_i++) {
    if (*d_i < 0.2 * scan_.range_max)
      return true;
  }

  return false;
}

bool Walker::robotSenseRight() {
  if (scan_.ranges.empty())
    return false;

  // Sense robot right
  for (auto d_i {scan_.ranges.rbegin()}; d_i < scan_.ranges.rbegin()+30; d_i++) {
    if (*d_i < 0.2 * scan_.range_max)
      return true;
  }

  return false;
}

void Walker::robotTurn(char direction) {
  auto stop_message {geometry_msgs::msg::Twist()};
  stop_message.linear.set__x(0);
  stop_message.linear.set__y(0);
  stop_message.linear.set__z(0);
  stop_message.angular.set__x(0);
  stop_message.angular.set__y(0);
  stop_message.angular.set__z(0);
  walker_->publish(stop_message);

  switch(direction) {
    case Walker::direction::left: {
      auto turn_message {geometry_msgs::msg::Twist()};
      turn_message.angular.set__z(-20 * M_PI/180);
      walker_->publish(turn_message);
      break;
    }

    case Walker::direction::right: {
      auto turn_message {geometry_msgs::msg::Twist()};
      turn_message.angular.set__z(-20 * M_PI/180);
      walker_->publish(turn_message);
      break;
    }
  }
}

void Walker::timerCallback() {
  auto forward_message {geometry_msgs::msg::Twist()};
  forward_message.linear.set__x(0.3);
  walker_->publish(forward_message);

  if (robotSenseLeft() && robotSenseRight())
    robotTurn(Walker::direction::right);

  if (robotSenseRight())
    robotTurn(Walker::direction::left);

  if (robotSenseLeft())
    robotTurn(Walker::direction::right);
}

// Main function
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_unique<Walker>());
  rclcpp::shutdown();

  return 0;
}
