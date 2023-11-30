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


  return 0;
}
