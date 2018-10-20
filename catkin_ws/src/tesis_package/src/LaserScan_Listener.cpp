#include "ros/ros.h"
#include <string.h>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Header.h"
#include <iostream>
#include <vector>
#include <cstdlib>
#include <sstream>

using namespace ros;
using namespace std;

struct myLaserscan {
  vector<char> myRanges;
  vector<char> vmyIntensities;

} ;

product apple;
product banana, melon
  
void laserScancallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

  sensor_msgs::LaserScan data = *msg; // La funcion callback de ROS guarda la data cruda en la memoria, por lo tanto para extraerla hay que apuntar a la direccion de memoria.

  /*
  La estructura de datos esta comprendida por los siguientes 12 valores, 
  divididos entre los tipos string, uint y float.
  
  uint32 seq
  time stamp     <------- Segundos, concatenados con nanosegundos 
  string frame_id

  float32 angle_min
  float32 angle_max
  float32 angle_increment
  float32 time_increment
  float32 scan_time
  float32 range_min
  float32 range_max
  float32[] ranges
  float32[] intensities
  */


  ROS_INFO(" \n\n I'm receiving from LaserScan: \n\n seq: %d \n stamp: %d \n frame_id: %s \n angle_min: %f \n angle_max: %f \n angle_increment: %f \n time_increment: %f \n scan_time: %f \n range_min: %f \n range_max: %f \n ", 
                                                                                       data.header.seq, data.header.stamp, data.header.frame_id.c_str(), data.angle_min, 
                                                                                       data.angle_max, data.angle_increment, data.time_increment, data.scan_time,
                                                                                       data.range_min, data.range_max);

  ROS_INFO("ranges: \n");

  for(int i = 0; i < data.ranges.size(); i++) {

    cout << data.ranges[i] << ", " ; 

  }

  ROS_INFO("intensities: \n");

  for(int j = 0; j < data.intensities.size(); j++) {
    cout << data.intensities[j] << ", " ;
  }   

  ROS_INFO("degrees: \n");

  // for(int j = 0; j < data.intensities.size(); j++)
  // {
  //   std::cout << data.intensities[j] << ", " ;
  // } 

}

int main(int argc, char **argv) {
  init(argc, argv, "LaserScan_Listener"); // Funcion de ROS que inicializa el nodo que vamos a lanzar

  NodeHandle n; // Declaracion del nodo a utilizar para la comunicacion

  Subscriber sub = n.subscribe("scan", 1000, laserScancallback);

  Rate loop_rate(1);

  spin();

  return 0;
}   