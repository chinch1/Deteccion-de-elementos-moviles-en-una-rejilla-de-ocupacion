#include "ros/ros.h"
#include <string.h>
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"

using namespace ros;
using namespace std;
  /*
  La estructura de datos esta comprendida por los siguientes 15 valores, 
  divididos entre los tipos string, uint y float.
  
  uint32 seq
  time stamp       <------- Segundos, concatenados con nanosegundos 
  string frame_id

  time map_load_time
  float32 resolution
  uint32 width
  uint32 height

  float64 x
  float64 y
  float64 z

  float64 x
  float64 y
  float64 z
  float64 w
  
  int8[] data

  */

int main(int argc, char **argv)//-----------------------------------------------
{
  init(argc, argv, "OccupancyGrid");

  NodeHandle n;

  Publisher mapPublication = n.advertise<nav_msgs::OccupancyGrid>("mapPub", 1000);
  
  /*int p[] = {100, 100, 50, 0};
  vector<signed char> a(p, p+4);*/

  nav_msgs::OccupancyGrid map;

/*  map.info.resolution = 1.0;
  map.info.width = 2;
  map.info.height = 2;
  map.info.origin.position.x = 0.0;
  map.info.origin.position.y = 0.0;
  map.info.origin.position.z = 0.0;
  map.info.origin.orientation.x = 0.0;
  map.info.origin.orientation.y = 0.0;
  map.info.origin.orientation.z = 0.0;
  map.info.origin.orientation.w = 0.0;
  map.data = a;*/

  map.header.frame_id = "map";

  Rate loop_rate(1);

  while(ok())
  {
  mapPublication.publish(map);

  spinOnce();
  
  loop_rate.sleep();

  }

  return 0;
}
