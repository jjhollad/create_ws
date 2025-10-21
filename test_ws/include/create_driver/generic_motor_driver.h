/**
Software License Agreement (BSD)
\file      generic_motor_driver.h
\authors   Adapted from create_driver
\copyright Copyright (c) 2015, Autonomy Lab (Simon Fraser University), All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
 * Neither the name of Autonomy Lab nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef GENERIC_MOTOR_DRIVER__GENERIC_MOTOR_DRIVER_H_
#define GENERIC_MOTOR_DRIVER__GENERIC_MOTOR_DRIVER_H_

#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/transform_broadcaster.h"

static const double COVARIANCE[36] = {1e-5, 1e-5, 0.0,  0.0,  0.0,  1e-5,  // NOLINT(whitespace/braces)
                                      1e-5, 1e-5, 0.0,  0.0,  0.0,  1e-5,
                                      0.0,  0.0,  1e-5, 0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.0,  1e-5, 0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.0,  1e-5, 0.0,
                                      1e-5, 1e-5, 0.0,  0.0,  0.0,  1e-5};

struct MotorData {
  double total_encoder[4] = {0.0, 0.0, 0.0, 0.0};  // Total encoder counts for M1-M4
  double realtime_encoder[4] = {0.0, 0.0, 0.0, 0.0};  // Real-time encoder counts (10ms)
  double speed[4] = {0.0, 0.0, 0.0, 0.0};  // Speed for M1-M4
  rclcpp::Time timestamp;
};

class GenericMotorDriver : public rclcpp::Node
{
private:
  // Serial communication
  std::string dev_;
  int baud_;
  int serial_fd_;
  std::atomic<bool> connected_;
  std::mutex serial_mutex_;
  
  // Motor data
  MotorData motor_data_;
  std::mutex motor_data_mutex_;
  
  // Odometry calculation
  double wheel_base_;  // Distance between left and right wheels
  double wheel_radius_;  // Wheel radius
  double motor_gear_ratio_;  // Motor internal gear ratio (e.g., 50.0 for 1:50)
  double belt_drive_ratio_;  // Belt drive ratio (e.g., 2.0 for 1:2)
  double total_reduction_ratio_;  // Total reduction ratio
  double x_, y_, theta_;  // Robot pose
  double last_left_encoder_, last_right_encoder_;
  rclcpp::Time last_odom_time_;
  
  // ROS2 interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  
  rclcpp::TimerBase::SharedPtr loop_timer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  diagnostic_updater::Updater diagnostics_;
  
  // Messages
  nav_msgs::msg::Odometry odom_msg_;
  geometry_msgs::msg::TransformStamped tf_odom_;
  sensor_msgs::msg::JointState joint_state_msg_;
  rclcpp::Time last_cmd_vel_time_;
  std_msgs::msg::Float32 float32_msg_;
  
  // ROS parameters
  std::string base_frame_;
  std::string odom_frame_;
  rclcpp::Duration latch_duration_;
  double loop_hz_;
  bool publish_tf_;
  bool is_running_slowly_;
  
  // Serial communication methods
  bool connectSerial();
  void disconnectSerial();
  bool sendCommand(const std::string& command);
  void readSerialData();
  void parseMotorData(const std::string& data);
  
  // Motor control methods
  void sendMotorSpeeds(double m1, double m2, double m3, double m4);
  void updateOdometry();
  
  // ROS2 callbacks
  void cmdVelCallback(geometry_msgs::msg::Twist::UniquePtr msg);
  
  // Publishing methods
  void publishOdom();
  void publishJointState();
  bool update();
  
  // Diagnostic methods
  void updateSerialDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void updateDriverDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

public:
  GenericMotorDriver();
  ~GenericMotorDriver();
  virtual void spinOnce();
};

#endif  // GENERIC_MOTOR_DRIVER__GENERIC_MOTOR_DRIVER_H_
