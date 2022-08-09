//          Copyright Emil Fresk 2018.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE.md or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

// User includes
#include "px4_comm/px4_comm.hpp"
//#include "mav_msgs/msg/conversions.hpp"

// Callback functions for subscribed messages
void px4_communication::callback_roll_pitch_yawrate_thrust(
    const mav_msgs::msg::RollPitchYawrateThrust::SharedPtr msg)
{
  using namespace mavros;

  auto q = ftf::quaternion_from_rpy(msg->roll, msg->pitch, current_yaw);

  mavros_msgs::msg::AttitudeTarget target;

  target.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
                     mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE;

  target.orientation.x = q.x();
  target.orientation.y = q.y();
  target.orientation.z = q.z();
  target.orientation.w = q.w();

  target.body_rate.x = 0;
  target.body_rate.y = 0;
  target.body_rate.z = msg->yaw_rate;

  target.thrust = msg->thrust.z;

  target.header.stamp = this->get_clock()->now();
  command_pub_->publish(target);
}

void px4_communication::callback_rollrate_pitchrate_yawrate_thrust(
    const mav_msgs::msg::RateThrust::SharedPtr msg)
{
  RCLCPP_ERROR_ONCE(this->get_logger(),"Got Rate Thrust command, not implemented");

  mavros_msgs::msg::AttitudeTarget target;

  target.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ATTITUDE;

  target.orientation.x = 0;
  target.orientation.y = 0;
  target.orientation.z = 0;
  target.orientation.w = 1;

  target.body_rate.x = msg->angular_rates.x;
  target.body_rate.y = msg->angular_rates.y;
  target.body_rate.z = msg->angular_rates.z;

  target.thrust = msg->thrust.z;

  target.header.stamp = this->get_clock()->now();
  command_pub_->publish(target);
}

void px4_communication::callback_mavros_rc_value(
    const mavros_msgs::msg::RCIn::SharedPtr msg)
{
    sensor_msgs::msg::Joy joy;
  for (auto ch : msg->channels)
    joy.axes.push_back(ch);

  joy.header.stamp = this->get_clock()->now();
  rc_pub_->publish(joy);
}

void px4_communication::callback_mavros_state(
    const mavros_msgs::msg::State::SharedPtr msg)
{
  status_msg_.header.stamp = this->get_clock()->now();

  if (msg->mode == "OFFBOARD")
    status_msg_.command_interface_enabled = true;
  else
    status_msg_.command_interface_enabled = false;

  if (msg->armed == true)
  {
    status_msg_.in_air = true;
    status_msg_.motor_status = mav_msgs::msg::Status::MOTOR_STATUS_RUNNING;
  }
  else
  {
    status_msg_.in_air = false;
    status_msg_.motor_status = mav_msgs::msg::Status::MOTOR_STATUS_STOPPED;
  }

  status_pub_->publish(status_msg_);
}

void px4_communication::callback_mavros_battery(
    const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  status_msg_.battery_voltage = msg->voltage;
}

void px4_communication::callback_mavros_altitude(
    const mavros_msgs::msg::Altitude::SharedPtr msg)
{
  RCLCPP_ERROR_ONCE(this->get_logger(),"Got MAVROS Altitude, not implemented (50x)");
}

void px4_communication::callback_mavros_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  current_yaw = mavros::ftf::quaternion_get_yaw(
      quaternionFromMsg(msg->orientation));
}

px4_communication::px4_communication()
    : Node("uav_comm_node")
{
  //
  // Local variables
  //
  status_msg_.battery_voltage = 0;
  status_msg_.command_interface_enabled = false;
  status_msg_.cpu_load = 0;
  status_msg_.flight_time = 0;
  status_msg_.gps_num_satellites = 0;
  status_msg_.gps_status = mav_msgs::msg::Status::GPS_STATUS_NO_LOCK;
  status_msg_.in_air = false;
  status_msg_.motor_status = mav_msgs::msg::Status::MOTOR_STATUS_STOPPED;
  status_msg_.rc_command_mode = mav_msgs::msg::Status::RC_COMMAND_ATTITUDE;
  status_msg_.system_uptime = 0;
  status_msg_.vehicle_name = "no type";
  status_msg_.vehicle_type = "no name";
  status_msg_.header.stamp = this->get_clock()->now();

  current_yaw = 0;

  //
  // UAV comm topics
  //
  ratethrust_sub_ = this->create_subscription<mav_msgs::msg::RateThrust>(
      "command/rate_thrust", 10,
      std::bind(&px4_communication::callback_rollrate_pitchrate_yawrate_thrust, this, std::placeholders::_1));

  rpyrt_sub_ = this->create_subscription<mav_msgs::msg::RollPitchYawrateThrust>(
      "command/roll_pitch_yawrate_thrust", 5,
      std::bind(&px4_communication::callback_roll_pitch_yawrate_thrust, this, std::placeholders::_1));

  status_pub_ = this->create_publisher< mav_msgs::msg::Status >("status", 1);

  rc_pub_ = this->create_publisher< sensor_msgs::msg::Joy >("rc", 1);

  //
  // mavros topics
  //
  command_pub_ = this->create_publisher< mavros_msgs::msg::AttitudeTarget >(
      "mavros/setpoint_raw/attitude", 10);

  rc_sub_ = this->create_subscription<mavros_msgs::msg::RCIn>("mavros/rc/in", 5,
                                 std::bind(&px4_communication::callback_mavros_rc_value,
                                 this, std::placeholders::_1));

  state_sub_ = this->create_subscription<mavros_msgs::msg::State>("mavros/state", 5,
                                    std::bind(&px4_communication::callback_mavros_state, this, std::placeholders::_1));

  battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "mavros/battery", 5, std::bind(&px4_communication::callback_mavros_battery, this, std::placeholders::_1));

  altitude_sub_ = this->create_subscription<mavros_msgs::msg::Altitude>( 
      "mavros/altitude", 5, std::bind(&px4_communication::callback_mavros_altitude, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("mavros/imu/data", 5,
                                  std::bind(&px4_communication::callback_mavros_imu, this, std::placeholders::_1));
}

