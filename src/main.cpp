#include "mediation_layer.hpp"
#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mediation_layer");
  ros::NodeHandle nh;

  try
  {
    mediationLayer ml(nh);
    ros::spin();
  }
  catch(const std::exception &e)
  {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
    return 1;
  }
  return 0;
}

