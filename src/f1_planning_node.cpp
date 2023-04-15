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

#include "f1_planning/f1_planning_node.hpp"
#include <string>
#include <cmath>

namespace f1_planning
{

F1PlanningNode::F1PlanningNode(const rclcpp::NodeOptions & options)
:  Node("f1_planning", options)
{
  f1_planning_ = std::make_unique<f1_planning::F1Planning>();
  const int64_t param_name = this->declare_parameter("param_name", 456);
  const std::string topic_scan = this->declare_parameter("topic_scan", "/sensing/lidar/scan");
  const std::string topic_odom = this->declare_parameter("topic_odom", "/localization/odometry");

  // Log parameters
  RCLCPP_INFO(this->get_logger(), "topic_scan: %s", topic_scan.c_str());
  RCLCPP_INFO(this->get_logger(), "topic_odom: %s", topic_odom.c_str());

  f1_planning_->setParameters(param_name);
  _pub_goal_pose = this->create_publisher<visualization_msgs::msg::Marker>("/planning/goal_pose", rclcpp::QoS{1}.transient_local()); 
  _pub_ackermann = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", rclcpp::QoS{1}.transient_local());
  _pub_gear = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", rclcpp::QoS{1}.transient_local());

  this->create_sub(topic_scan, topic_odom);

  // spin node
}

void F1PlanningNode::create_sub(const std::string topic_scan, const std::string topic_odom)
{
  auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
  _sub_laserscan = this->create_subscription<sensor_msgs::msg::LaserScan>(
    topic_scan, sensor_qos, std::bind(&F1PlanningNode::scan_callback, this, std::placeholders::_1));
  _sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
    topic_odom, sensor_qos, std::bind(&F1PlanningNode::odom_callback, this, std::placeholders::_1));
}

void F1PlanningNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{

    double angle = msg->angle_increment; // radians
    double angle_min = msg->angle_min; // radians


    double param = 0.275/2;
    int index_90_deg = (int)std::round((-(0.5+param) * M_PI - angle_min) / angle);
    int index_270_deg = (int)std::round(((0.5-param) * M_PI - angle_min) / angle);

    // int index_135_deg = (int)std::round((-0.75 * M_PI - angle_min) / angle);
    // int index_315_deg = (int)std::round((-.75 * M_PI - angle_min) / angle);
    
   // Global sterring 
    double distance_180_deg_left = 0;
    double distance_180_deg_right = 0;
    int index_180_deg = (int)msg->ranges.size() / 2;
    int count = 50;
    for (int i = -count; i < 0; i++) {
        distance_180_deg_left += msg->ranges[index_180_deg + i];
    }
    for (int i = 0; i < count; i++) {
        distance_180_deg_right += msg->ranges[index_180_deg + i];
    }

    distance_180_deg_left /= count;
    distance_180_deg_right /= count;

    double front_steering = 0;
    if (distance_180_deg_left - distance_180_deg_right > 0.1) {
        front_steering = distance_180_deg_right - distance_180_deg_left;
        front_steering /= 25;
        
        RCLCPP_INFO(this->get_logger(), "right diff = %f", distance_180_deg_left - distance_180_deg_right);
    } else if (distance_180_deg_left - distance_180_deg_right < -0.1) {
        front_steering = distance_180_deg_right - distance_180_deg_left;
        RCLCPP_INFO(this->get_logger(), "left diff = %f", distance_180_deg_left - distance_180_deg_right);
        front_steering /= 25;

    } else {

        RCLCPP_INFO(this->get_logger(), "straight diff = %f", distance_180_deg_left - distance_180_deg_right);
    }
    double distance_180_deg_mean = (distance_180_deg_left + distance_180_deg_right) / 2;
    if ((distance_180_deg_left - distance_180_deg_right) < 1.0 and distance_180_deg_mean < 2.0) {
        RCLCPP_INFO(this->get_logger(), "Big turn");
        front_steering = distance_180_deg_right - distance_180_deg_left;
        front_steering *= 200;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Small turn or straight diff = %f, mean = %f", distance_180_deg_left - distance_180_deg_right, distance_180_deg_mean) ;
    }



    // Local sterring
    count /= 2;
    double distance_90_mean = 0;
    for (int i = -count; i < count; i++) {
        distance_90_mean += msg->ranges[index_90_deg + i];
    }


    double distance_270_mean = 0;
    for (int i = -count; i < count; i++) {
        distance_270_mean += msg->ranges[index_270_deg + i];
    }


    distance_90_mean /= 2*count;
    distance_270_mean /= 2*count;
  

    double angular_vel = 0;
    // if (abs(front_steering) > 2){
    //     RCLCPP_INFO(this->get_logger(), "front steering = %f", front_steering);
        angular_vel = front_steering+ ( (distance_270_mean - distance_90_mean)/2.5);
    // }
    // else {
    //     RCLCPP_INFO(this->get_logger(), "side steering = %f", distance_270_mean - distance_90_mean);
    //     angular_vel = (distance_270_mean - distance_90_mean)/2.5;
    // }
  

    publish_velocity(angular_vel,distance_180_deg_mean);  

    RCLCPP_ERROR(this->get_logger(), "Left: [%f], right: [%f], front: [%f]", distance_270_mean,distance_90_mean, distance_180_deg_mean);
    double x = 10;
    double y = 10;
    publish_points(x,y);
}

void F1PlanningNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  _last_pose.pose.position.x = msg->pose.pose.position.x;
  _last_pose.pose.position.y = msg->pose.pose.position.y;
  _last_pose.pose.position.z = msg->pose.pose.position.z;
  _last_pose.pose.orientation.x = msg->pose.pose.orientation.x;
  _last_pose.pose.orientation.y = msg->pose.pose.orientation.y;
  _last_pose.pose.orientation.z = msg->pose.pose.orientation.z;
  _last_pose.pose.orientation.w = msg->pose.pose.orientation.w;
}

void F1PlanningNode::publish_velocity(double angular_vel, double distance)
{

  autoware_auto_control_msgs::msg::AckermannControlCommand cmd;
  cmd.stamp = cmd.lateral.stamp = cmd.longitudinal.stamp = get_clock()->now();
  autoware_auto_vehicle_msgs::msg::GearCommand gear_cmd;
  if (angular_vel > 0.1) {
  cmd.lateral.steering_tire_angle = std::min(angular_vel,3.0);
  }
  else if (angular_vel < -0.1) {
  cmd.lateral.steering_tire_angle = std::max(angular_vel,-3.0);
  }
  cmd.longitudinal.speed = std::max(distance /60 - angular_vel, 0.1);
  cmd.longitudinal.acceleration = std::max(distance /60 - angular_vel, 0.1);
  if (distance < 2){
    cmd.longitudinal.speed = 0.1;
    cmd.longitudinal.acceleration = 0.01;
  }
  {
    const double eps = 0.001;
    if (cmd.longitudinal.speed > eps) {
      gear_cmd.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE;
    } else if (cmd.longitudinal.speed < -eps ) {
      gear_cmd.command = autoware_auto_vehicle_msgs::msg::GearCommand::REVERSE;
      cmd.longitudinal.acceleration *= -1.0;
    } else {
      gear_cmd.command = autoware_auto_vehicle_msgs::msg::GearCommand::PARK;
    }
  }
  RCLCPP_ERROR(this->get_logger(), "Vel: [%f], ang: [%f], acc: [%f]", cmd.longitudinal.speed, angular_vel, cmd.longitudinal.acceleration);

  _pub_ackermann->publish(cmd);
  _pub_gear->publish(gear_cmd);

}

void F1PlanningNode::publish_points(double x, double y)
  {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->now();
    marker.ns = "points";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Add pointros2 launch f1tenth_launch f1tenth.launch.py map_path:=autoware_map/imolas to marker
    geometry_msgs::msg::Point point1;
    point1.x = x;
    point1.y = y;
    point1.z = 0.5;
    marker.points.push_back(point1);

    _pub_goal_pose->publish(marker);
  }

}  // namespace f1_planning
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(f1_planning::F1PlanningNode)
