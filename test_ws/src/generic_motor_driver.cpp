/**
Software License Agreement (BSD)
\file      generic_motor_driver.cpp
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

#include "create_driver/generic_motor_driver.h"

#include <chrono>
#include <memory>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <algorithm>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

GenericMotorDriver::GenericMotorDriver()
: Node("generic_motor_driver"),
  serial_fd_(-1),
  connected_(false),
  wheel_base_(0.3),  // Default 30cm wheelbase
  wheel_radius_(0.05),  // Default 5cm wheel radius
  motor_gear_ratio_(210.0),  // Default no gear reduction
  belt_drive_ratio_(6.4),  // Default no belt drive
  total_reduction_ratio_(1.0),  // Will be calculated
  x_(0.0), y_(0.0), theta_(0.0),
  last_left_encoder_(0.0), last_right_encoder_(0.0),
  last_odom_time_(now()),
  last_cmd_vel_time_(0),
  latch_duration_(std::chrono::nanoseconds{0}),
  is_running_slowly_(false),
  tf_broadcaster_(this),
  diagnostics_(this)
{
  // Get parameters
  dev_ = declare_parameter<std::string>("dev", "/dev/ttyUSB0");
  baud_ = declare_parameter<int>("baud", 115200);
  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  wheel_base_ = declare_parameter<double>("wheel_base", 0.3);
  wheel_radius_ = declare_parameter<double>("wheel_radius", 0.05);
  motor_gear_ratio_ = declare_parameter<double>("motor_gear_ratio", 1.0);
  belt_drive_ratio_ = declare_parameter<double>("belt_drive_ratio", 1.0);
  latch_duration_ = rclcpp::Duration::from_seconds(declare_parameter<double>("latch_cmd_duration", 0.2));
  loop_hz_ = declare_parameter<double>("loop_hz", 10.0);
  publish_tf_ = declare_parameter<bool>("publish_tf", true);
  
  // Calculate total reduction ratio
  total_reduction_ratio_ = motor_gear_ratio_ * belt_drive_ratio_;

  RCLCPP_INFO_STREAM(get_logger(), "[MOTOR] Using device: " << dev_ << " at " << baud_ << " baud");
  RCLCPP_INFO_STREAM(get_logger(), "[MOTOR] Motor gear ratio: " << motor_gear_ratio_);
  RCLCPP_INFO_STREAM(get_logger(), "[MOTOR] Belt drive ratio: " << belt_drive_ratio_);
  RCLCPP_INFO_STREAM(get_logger(), "[MOTOR] Total reduction ratio: " << total_reduction_ratio_);

  // Connect to serial device
  if (!connectSerial()) {
    RCLCPP_FATAL(get_logger(), "[MOTOR] Failed to establish serial connection.");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(get_logger(), "[MOTOR] Connection established.");

  // Set frame_id's
  tf_odom_.header.frame_id = odom_frame_;
  tf_odom_.child_frame_id = base_frame_;
  odom_msg_.header.frame_id = odom_frame_;
  odom_msg_.child_frame_id = base_frame_;
  
  // Setup joint state message
  joint_state_msg_.name.resize(4);
  joint_state_msg_.position.resize(4);
  joint_state_msg_.velocity.resize(4);
  joint_state_msg_.effort.resize(4);
  joint_state_msg_.name[0] = "motor_1_joint";
  joint_state_msg_.name[1] = "motor_2_joint";
  joint_state_msg_.name[2] = "motor_3_joint";
  joint_state_msg_.name[3] = "motor_4_joint";

  // Populate initial covariances
  for (int i = 0; i < 36; i++) {
    odom_msg_.pose.covariance[i] = COVARIANCE[i];
    odom_msg_.twist.covariance[i] = COVARIANCE[i];
  }

  // Setup subscribers
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 1, std::bind(&GenericMotorDriver::cmdVelCallback, this, std::placeholders::_1));

  // Setup publishers
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 30);
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  // Setup diagnostics
  diagnostics_.add("Serial Status", this, &GenericMotorDriver::updateSerialDiagnostics);
  diagnostics_.add("Driver Status", this, &GenericMotorDriver::updateDriverDiagnostics);
  diagnostics_.setHardwareID("Generic Motor Controller");

  // Setup update loop
  const auto loop_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / loop_hz_));
  loop_timer_ = create_wall_timer(loop_period, std::bind(&GenericMotorDriver::spinOnce, this));

  RCLCPP_INFO(get_logger(), "[MOTOR] Ready.");
}

GenericMotorDriver::~GenericMotorDriver()
{
  RCLCPP_INFO(get_logger(), "[MOTOR] Destruct sequence initiated.");
  disconnectSerial();
}

bool GenericMotorDriver::connectSerial()
{
  serial_fd_ = open(dev_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(get_logger(), "[MOTOR] Failed to open serial port: %s", dev_.c_str());
    return false;
  }

  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "[MOTOR] Failed to get serial attributes");
    return false;
  }

  // Set baud rate
  speed_t speed;
  switch (baud_) {
    case 115200: speed = B115200; break;
    case 57600:  speed = B57600;  break;
    case 38400:  speed = B38400;  break;
    case 19200:  speed = B19200;  break;
    case 9600:   speed = B9600;   break;
    default:
      RCLCPP_ERROR(get_logger(), "[MOTOR] Unsupported baud rate: %d", baud_);
      return false;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  // Configure serial port: 8N1, no flow control
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  tty.c_iflag &= ~IGNBRK;         // disable break processing
  tty.c_lflag = 0;                // no signaling chars, no echo, no canonical processing
  tty.c_oflag = 0;                // no remapping, no delays
  tty.c_cc[VMIN]  = 0;            // read doesn't block
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "[MOTOR] Failed to set serial attributes");
    return false;
  }

  connected_ = true;
  return true;
}

void GenericMotorDriver::disconnectSerial()
{
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
  connected_ = false;
}

bool GenericMotorDriver::sendCommand(const std::string& command)
{
  if (!connected_ || serial_fd_ < 0) {
    RCLCPP_WARN(get_logger(), "[MOTOR] Not connected, cannot send command: %s", command.c_str());
    return false;
  }

  std::lock_guard<std::mutex> lock(serial_mutex_);
  ssize_t written = write(serial_fd_, command.c_str(), command.length());
  bool success = written == static_cast<ssize_t>(command.length());
  
  if (success) {
    RCLCPP_INFO(get_logger(), "[MOTOR] Sent command: %s", command.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "[MOTOR] Failed to send command: %s (written: %ld, expected: %zu)", 
                command.c_str(), written, command.length());
  }
  
  return success;
}

void GenericMotorDriver::readSerialData()
{
  if (!connected_ || serial_fd_ < 0) {
    return;
  }

  char buffer[256];
  std::string data;
  
  std::lock_guard<std::mutex> lock(serial_mutex_);
  ssize_t n = read(serial_fd_, buffer, sizeof(buffer) - 1);
  if (n > 0) {
    buffer[n] = '\0';
    data = std::string(buffer);
    
    // Parse any complete messages in the buffer
    size_t start = 0;
    while (true) {
      size_t dollar_pos = data.find('$', start);
      if (dollar_pos == std::string::npos) break;
      
      size_t hash_pos = data.find('#', dollar_pos);
      if (hash_pos == std::string::npos) break;
      
      std::string message = data.substr(dollar_pos, hash_pos - dollar_pos + 1);
      parseMotorData(message);
      
      start = hash_pos + 1;
    }
  }
}

void GenericMotorDriver::parseMotorData(const std::string& data)
{
  if (data.length() < 3) return;
  
  RCLCPP_INFO(get_logger(), "[MOTOR] Received data: %s", data.c_str());
  
  std::string command = data.substr(1, 4);  // Extract command (e.g., "MAll", "MTEP", "MSPD")
  
  if (command == "MAll") {
    // Parse total encoder data: $MAll:M1,M2,M3,M4#
    size_t colon_pos = data.find(':');
    if (colon_pos != std::string::npos) {
      std::string values = data.substr(colon_pos + 1, data.length() - colon_pos - 2);
      std::stringstream ss(values);
      std::string item;
      int i = 0;
      
      std::lock_guard<std::mutex> lock(motor_data_mutex_);
      motor_data_.timestamp = now();
      while (std::getline(ss, item, ',') && i < 4) {
        motor_data_.total_encoder[i] = std::stod(item);
        i++;
      }
      RCLCPP_INFO(get_logger(), "[MOTOR] Updated total encoders: [%.1f, %.1f, %.1f, %.1f]", 
                   motor_data_.total_encoder[0], motor_data_.total_encoder[1], 
                   motor_data_.total_encoder[2], motor_data_.total_encoder[3]);
    }
  }
  else if (command == "MTEP") {
    // Parse real-time encoder data: $MTEP:M1,M2,M3,M4#
    size_t colon_pos = data.find(':');
    if (colon_pos != std::string::npos) {
      std::string values = data.substr(colon_pos + 1, data.length() - colon_pos - 2);
      std::stringstream ss(values);
      std::string item;
      int i = 0;
      
      std::lock_guard<std::mutex> lock(motor_data_mutex_);
      while (std::getline(ss, item, ',') && i < 4) {
        motor_data_.realtime_encoder[i] = std::stod(item);
        i++;
      }
    }
  }
  else if (command == "MSPD") {
    // Parse speed data: $MSPD:M1,M2,M3,M4#
    size_t colon_pos = data.find(':');
    if (colon_pos != std::string::npos) {
      std::string values = data.substr(colon_pos + 1, data.length() - colon_pos - 2);
      std::stringstream ss(values);
      std::string item;
      int i = 0;
      
      std::lock_guard<std::mutex> lock(motor_data_mutex_);
      while (std::getline(ss, item, ',') && i < 4) {
        motor_data_.speed[i] = std::stod(item);
        i++;
      }
    }
  }
}

void GenericMotorDriver::sendMotorSpeeds(double m1, double m2, double m3, double m4)
{
  std::stringstream ss;
  ss << "$spd:" << static_cast<int>(m1) << "," << static_cast<int>(m2) 
     << "," << static_cast<int>(m3) << "," << static_cast<int>(m4) << "#";
  
  if (!sendCommand(ss.str())) {
    RCLCPP_WARN(get_logger(), "[MOTOR] Failed to send motor command");
  }
}

void GenericMotorDriver::updateOdometry()
{
  std::lock_guard<std::mutex> lock(motor_data_mutex_);
  
  // Use motors 1 and 2 as left and right wheels for differential drive
  double left_encoder = motor_data_.total_encoder[0];
  double right_encoder = motor_data_.total_encoder[1];
  
  rclcpp::Time current_time = now();
  double dt = (current_time - last_odom_time_).seconds();
  
  if (dt > 0.0) {
    // Convert encoder counts to wheel rotations, accounting for gear ratios
    double left_wheel_rotations = (left_encoder - last_left_encoder_) / total_reduction_ratio_;
    double right_wheel_rotations = (right_encoder - last_right_encoder_) / total_reduction_ratio_;
    
    // Convert wheel rotations to linear distance
    double left_distance = left_wheel_rotations * (2.0 * M_PI * wheel_radius_);
    double right_distance = right_wheel_rotations * (2.0 * M_PI * wheel_radius_);
    
    double delta_distance = (left_distance + right_distance) / 2.0;
    double delta_theta = (right_distance - left_distance) / wheel_base_;
    
    x_ += delta_distance * cos(theta_);
    y_ += delta_distance * sin(theta_);
    theta_ += delta_theta;
    
    last_left_encoder_ = left_encoder;
    last_right_encoder_ = right_encoder;
    last_odom_time_ = current_time;
  }
}

void GenericMotorDriver::cmdVelCallback(geometry_msgs::msg::Twist::UniquePtr msg)
{
  // Convert twist to motor speeds
  // Assuming motors 1 and 2 are left and right wheels
  double linear = msg->linear.x;
  double angular = msg->angular.z;
  
  double left_speed = (linear - angular * wheel_base_ / 2.0) / wheel_radius_;
  double right_speed = (linear + angular * wheel_base_ / 2.0) / wheel_radius_;
  
  // Scale speeds to motor controller range (assuming -100 to 100)
  left_speed = std::max(-100.0, std::min(100.0, left_speed));
  right_speed = std::max(-100.0, std::min(100.0, right_speed));
  
  sendMotorSpeeds(left_speed, right_speed, 0.0, 0.0);
  last_cmd_vel_time_ = now();
}

void GenericMotorDriver::publishOdom()
{
  updateOdometry();
  
  // Populate position info
  tf2::Quaternion tf_quat;
  tf_quat.setRPY(0.0, 0.0, theta_);
  geometry_msgs::msg::Quaternion quat = tf2::toMsg(tf_quat);
  
  odom_msg_.header.stamp = now();
  odom_msg_.pose.pose.position.x = x_;
  odom_msg_.pose.pose.position.y = y_;
  odom_msg_.pose.pose.orientation = quat;
  
  // Calculate velocities from encoder data
  std::lock_guard<std::mutex> lock(motor_data_mutex_);
  // Convert motor speeds to wheel speeds accounting for gear ratios
  double left_motor_speed = motor_data_.speed[0] / total_reduction_ratio_;
  double right_motor_speed = motor_data_.speed[1] / total_reduction_ratio_;
  
  // Convert motor speeds to linear velocities
  double left_vel = left_motor_speed * wheel_radius_;
  double right_vel = right_motor_speed * wheel_radius_;
  
  double linear_vel = (left_vel + right_vel) / 2.0;
  double angular_vel = (right_vel - left_vel) / wheel_base_;
  
  odom_msg_.twist.twist.linear.x = linear_vel;
  odom_msg_.twist.twist.linear.y = 0.0;
  odom_msg_.twist.twist.angular.z = angular_vel;
  
  // Update covariances
  for (int i = 0; i < 36; i++) {
    odom_msg_.pose.covariance[i] = COVARIANCE[i];
    odom_msg_.twist.covariance[i] = COVARIANCE[i];
  }
  
  if (publish_tf_) {
    tf_odom_.header.stamp = now();
    tf_odom_.transform.translation.x = x_;
    tf_odom_.transform.translation.y = y_;
    tf_odom_.transform.rotation = quat;
    tf_broadcaster_.sendTransform(tf_odom_);
  }
  
  odom_pub_->publish(odom_msg_);
}

void GenericMotorDriver::publishJointState()
{
  std::lock_guard<std::mutex> lock(motor_data_mutex_);
  
  joint_state_msg_.header.stamp = now();
  for (int i = 0; i < 4; i++) {
    // Convert motor encoder counts to wheel rotations
    joint_state_msg_.position[i] = motor_data_.total_encoder[i] / total_reduction_ratio_;
    // Convert motor speeds to wheel speeds
    joint_state_msg_.velocity[i] = motor_data_.speed[i] / total_reduction_ratio_;
    joint_state_msg_.effort[i] = 0.0;  // Not available from this controller
  }
  
  joint_state_pub_->publish(joint_state_msg_);
}

void GenericMotorDriver::updateSerialDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (!connected_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Serial port not connected");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Serial connection is good");
  }
  
  stat.add("Device", dev_);
  stat.add("Baud Rate", baud_);
  stat.add("Connected", connected_);
}

void GenericMotorDriver::updateDriverDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (is_running_slowly_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Internal loop running slowly");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Maintaining loop frequency");
  }
}

bool GenericMotorDriver::update()
{
  // Request encoder data from motor controller
  sendCommand("$upload:1,1,1#");
  
  readSerialData();
  publishOdom();
  publishJointState();
  
  // If last velocity command was sent longer than latch duration, stop robot
  if (last_cmd_vel_time_.nanoseconds() == 0 || now() - last_cmd_vel_time_ >= latch_duration_) {
    sendMotorSpeeds(0.0, 0.0, 0.0, 0.0);
  }
  
  return true;
}

void GenericMotorDriver::spinOnce()
{
  const auto spin_start = now();
  
  update();
  diagnostics_.force_update();
  
  // Check if the spin took longer than the target loop period
  const auto spin_end = now();
  const auto elapsed = spin_end - spin_start;
  const double target_period = 1. / loop_hz_;
  is_running_slowly_ = elapsed.seconds() > target_period;
  
  if (is_running_slowly_) {
    RCLCPP_WARN(get_logger(), "[MOTOR] Loop running slowly.");
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto motor_driver = std::make_shared<GenericMotorDriver>();
  
  try {
    rclcpp::spin(motor_driver);
  } catch (std::runtime_error& ex) {
    RCLCPP_FATAL_STREAM(motor_driver->get_logger(), "[MOTOR] Runtime error: " << ex.what());
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
