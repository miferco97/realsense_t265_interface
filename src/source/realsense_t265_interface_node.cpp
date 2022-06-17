#include "realsense_t265_interface.hpp"
#include "ros/ros.h"
#include "ros_utils_lib/ros_utils.hpp"

int main(int argc, char **argv)
{

  ros::init(argc, argv, ros_utils_lib::getNodeName("rs_t265_interface_node"));
  std::cout << "Node starting "<< std::endl;
  RealsenseT265Interface rsT265_interface;
  rsT265_interface.setUp();
  rsT265_interface.start();
  ros::Rate r(250);
  while (ros::ok())
  {
    rsT265_interface.run();
    ros::spinOnce();
    r.sleep();
  }
  rsT265_interface.stop();
  
  return 0;
}
