//          Copyright Emil Fresk 2018.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE.md or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include "px4_comm/px4_comm.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_comm_node");

  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  px4_communication comm(n, pn);

  comm.ros_loop();

  return 0;
}
