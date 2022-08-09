//          Copyright Emil Fresk 2018.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE.md or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once

// Ros includes
#include <rclcpp/rclcpp.hpp>

// Messages
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
//#include <mav_msgs/default_topics.h>
#include <mav_msgs/msg/rate_thrust.hpp>
#include <mav_msgs/msg/roll_pitch_yawrate_thrust.hpp>
#include <mav_msgs/msg/status.hpp>
#include <mav_msgs/msg/actuators.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/altitude.hpp>

#include <Eigen/Dense>
#include <mavros/frame_tf.hpp>

#include <geometry_msgs/msg/quaternion.hpp>

class px4_communication : public rclcpp::Node
{
private:

  // mav_msgs side
    rclcpp::Subscription<mav_msgs::msg::RateThrust>::SharedPtr ratethrust_sub_;
    rclcpp::Subscription<mav_msgs::msg::RollPitchYawrateThrust>::SharedPtr rpyrt_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<mav_msgs::msg::Status>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr rc_pub_;

  // mavros_msgs side
  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<mavros_msgs::msg::Altitude>::SharedPtr altitude_sub_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr command_pub_;

  // Callback functions for subscribed messages
  void callback_roll_pitch_yawrate_thrust(
      const mav_msgs::msg::RollPitchYawrateThrust::SharedPtr msg);
  void callback_rollrate_pitchrate_yawrate_thrust(
      const mav_msgs::msg::RateThrust::SharedPtr msg);

  void callback_mavros_rc_value(const mavros_msgs::msg::RCIn::SharedPtr msg);
  void callback_mavros_state(const mavros_msgs::msg::State::SharedPtr msg);
  void callback_mavros_battery(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void callback_mavros_altitude(const mavros_msgs::msg::Altitude::SharedPtr msg);
  void callback_mavros_imu(const sensor_msgs::msg::Imu::SharedPtr msg);

  // Local variables
  mav_msgs::msg::Status status_msg_;
  double current_yaw;

  inline Eigen::Quaterniond quaternionFromMsg(const geometry_msgs::msg::Quaternion msg) {
      Eigen::Quaterniond quaternion(msg.w, msg.x, msg.y, msg.z);
      if (quaternion.norm() < std::numeric_limits<double>::epsilon()){
          quaternion.setIdentity();
      } else {
          quaternion.normalize();
      }
      return quaternion;
  }

public:

  px4_communication();


};
