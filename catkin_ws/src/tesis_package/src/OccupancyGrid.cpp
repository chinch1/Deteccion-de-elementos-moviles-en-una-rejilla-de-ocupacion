#include "ros/ros.h"
#include <string.h>
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include <stdlib.h>
# include <iostream>

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
  nav_msgs::OccupancyGrid map;

  int size;
  int i = 0; 
  

  map.info.resolution = 0.1;
  map.info.width = 100;
  map.info.height = 100;
  map.info.origin.position.x = 0.0;
  map.info.origin.position.y = 0.0;
  map.info.origin.position.z = 0.0;
  map.info.origin.orientation.x = 0.0;
  map.info.origin.orientation.y = 0.0;
  map.info.origin.orientation.z = 0.0;
  map.info.origin.orientation.w = 0.0;
  
  map.header.frame_id = "map";

  size = map.info.width*map.info.height;
  std::vector<signed char> mapProbs(size);

  Rate loop_rate(10);

  while(ok())
  {

  for(int i =0; i < size; i++)
  {
    mapProbs[i] = rand() % 100 + 1;
  }

  //std::fill(mapProbs.begin(), mapProbs.end(), a);
   
  map.data = mapProbs;

  mapPublication.publish(map);

  spinOnce();
  
  loop_rate.sleep();

  }

  return 0;
}
