#pragma once

// Ros includes
#include <ros/ros.h>

// Messages
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/BatteryState.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/RateThrust.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mav_msgs/Status.h>
#include <mav_msgs/Actuators.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/Altitude.h>

class px4_communication
{
private:
  ros::NodeHandle priv_nh_;
  ros::NodeHandle public_nh_;

  // mav_msgs side
  ros::Subscriber ratethrust_sub_;
  ros::Subscriber rpyrt_sub_;
  ros::Publisher status_pub_;
  ros::Publisher rc_pub_;

  // mavros_msgs side
  ros::Subscriber rc_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber altitude_sub_;
  ros::Publisher command_pub_;

  // Callback functions for subscribed messages
  void callback_roll_pitch_yawrate_thrust(
      const mav_msgs::RollPitchYawrateThrustConstPtr& msg);
  void callback_rollrate_pitchrate_yawrate_thrust(
      const mav_msgs::RateThrustConstPtr& msg);

  void callback_mavros_rc_value(const mavros_msgs::RCInConstPtr& msg);
  void callback_mavros_state(const mavros_msgs::StateConstPtr& msg);
  void callback_mavros_battery(const sensor_msgs::BatteryStateConstPtr& msg);
  void callback_mavros_altitude(const mavros_msgs::AltitudeConstPtr& msg);

  // Local variables
  mav_msgs::Status status_msg_;

public:

  px4_communication(ros::NodeHandle &pub_nh, ros::NodeHandle &priv_nh);

  void ros_loop();

};
