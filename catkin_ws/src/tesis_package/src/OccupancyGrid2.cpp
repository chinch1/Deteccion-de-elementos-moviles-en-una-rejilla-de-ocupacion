#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"

using namespace ros;

Publisher map_pub;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){

  std_msgs::Header header = msg->header;
  nav_msgs::MapMetaData info = msg->info;
  nav_msgs::OccupancyGrid map;
  
  ROS_INFO("Got map %d %d", info.width, info.height);
  
  // Map map(info.width, info.height);
  
 /* for (unsigned int x = 0; x < info.width; x++)
    for (unsigned int y = 0; y < info.height; y++)
      map.Insert(Cell(x,y,info.width,msg->data[x+ info.width * y]));*/
  
 /* nav_msgs::OccupancyGrid* newGrid = map.Grid();
  newGrid->header = header;
  newGrid->info = info;
  map_pub.publish(*newGrid);
*/}

int main(int argc, char **argv){ 
  init(argc, argv, "grid"); //Inicio del nodo "grid" de ROS
  NodeHandle n; //

  map_pub = n.advertise<nav_msgs::OccupancyGrid>("map_out",10);
  Subscriber map_sub = n.subscribe("map",10,mapCallback);
  
  spin();
  return 0;
}