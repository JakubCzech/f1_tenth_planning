// Copyright 2023 Jakub Czech
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef F1_PLANNING__F1_PLANNING_NODE_HPP_
#define F1_PLANNING__F1_PLANNING_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>

#include <cmath>
#include <string>
#include "f1_planning/f1_planning.hpp"

namespace f1_planning
{
using F1PlanningPtr = std::unique_ptr<f1_planning::F1Planning>;

class F1_PLANNING_PUBLIC F1PlanningNode : public rclcpp::Node
{
public:
  explicit F1PlanningNode(const rclcpp::NodeOptions & options);

private:
  F1PlanningPtr f1_planning_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _sub_laserscan;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _sub_odometry;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _pub_goal_pose;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pub_twist;
  rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr _pub_ackermann;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr _pub_gear;

  geometry_msgs::msg::PoseStamped _last_pose;
  geometry_msgs::msg::PoseStamped _goal_pose;

  int scan_counter = 0; 

  void create_sub(const std::string topic_scan, const std::string topic_odom);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publish_points(double x, double y);
  void publish_velocity(double angular_vel, double linear_vel);


  double max_distance = 5.0; // maksymalna odległość do śledzenia
  double linear_vel = 1.0; // prędkość liniowa (m/s)

  // add subscriber variable
};
}  // namespace f1_planning

#endif  // F1_PLANNING__F1_PLANNING_NODE_HPP_
