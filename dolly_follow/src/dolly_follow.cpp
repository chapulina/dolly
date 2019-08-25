// Copyright 2018 Louise Poubel.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <memory>
#include <utility>

class Follow : public rclcpp::Node
{
public:
  /// \brief Follow node, which subscribes to laser scan messages and publishes
  /// velocity commands.
  Follow()
  : Node("follow")
  {
    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    // Subscribe to sensor messages
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "laser_scan", default_qos,
      std::bind(&Follow::OnSensorMsg, this, std::placeholders::_1));

    // Advertise velocity commands
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", default_qos);
  }

private:
  /// \brief Callback for sensor message subscriber
  /// \param[in] _msg Laser scan message
  void OnSensorMsg(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
  {
    // Find closest hit
    float min_range = _msg->range_max + 1;
    int idx = -1;
    for (auto i = 0u; i < _msg->ranges.size(); ++i) {
      auto range = _msg->ranges[i];
      if (range > _msg->range_min && range < _msg->range_max && range < min_range) {
        min_range = range;
        idx = i;
      }
    }

    // Calculate desired yaw change
    double turn = _msg->angle_min + _msg->angle_increment * idx;

    // Populate command message, all weights have been calculated by trial and error
    auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();

    // Bad readings, stop
    if (idx < 0) {
      cmd_msg->linear.x = 0;
      cmd_msg->angular.z = 0;
    } else if (min_range <= min_dist_) {
      // Too close, just rotate
      cmd_msg->linear.x = 0;
      cmd_msg->angular.z = turn * angular_k_;
    } else {
      cmd_msg->linear.x = linear_k_ / abs(turn);
      cmd_msg->angular.z = turn * angular_k_;
    }

    cmd_pub_->publish(std::move(cmd_msg));
  }

  /// \brief Laser messages subscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

  /// \brief Velocity command publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  /// \brief Minimum allowed distance from target
  double min_dist_ = 1.0;

  /// \brief Scale linear velocity, chosen by trial and error
  double linear_k_ = 0.02;

  /// \brief Scale angular velocity, chosen by trial and error
  double angular_k_ = 0.08;
};

int main(int argc, char * argv[])
{
  // Forward command line arguments to ROS
  rclcpp::init(argc, argv);

  // Create a node
  auto node = std::make_shared<Follow>();

  // Run node until it's exited
  rclcpp::spin(node);

  // Clean up
  rclcpp::shutdown();
  return 0;
}
