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
