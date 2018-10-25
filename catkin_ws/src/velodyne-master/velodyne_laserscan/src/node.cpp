#include <ros/ros.h>
#include "velodyne_laserscan/VelodyneLaserScan.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velodyne_laserscan_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // create VelodyneLaserScan class
  velodyne_laserscan::VelodyneLaserScan n(nh, nh_priv);

	      ROS_INFO("Bandera 2: Supere el main");

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
