#include "ros/ros.h"
#include <string.h>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Header.h"

// #include <std_msgs/Int8.h>
// #include <std_msgs/Int16.h>
// #include <std_msgs/Int32.h>
// #include <std_msgs/Int64.h>

// #include <std_msgs/UInt8.h>
// #include <std_msgs/Uint16.h>
// #include <std_msgs/Uint32.h>

// #include <std_msgs/FLoat32.h>
// #include <std_msgs/Float64.h>

// #include <std_msgs/UInt8MultiArray.h>

// #include <std_msgs/Time.h>
// #include <std_msgs/Bool.h>
 
using namespace ros;

void laserScancallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

  sensor_msgs::LaserScan data = *msg; // La funcion callback de ROS guarda la data cruda en la memoria, por lo tanto para extraerla hay que apuntar a la direccion de memoria.

  /*
  La estructura de datos esta comprendida por los siguientes 12 valores, 
  divididos entre los tipos string, uint y float.
  
  uint32 seq
  time stamp       <------- Segundos, concatenados con nanosegundos 
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

// Ahora imprimimos la data para visualizarla
  ROS_INFO(" \n\n I'm receiving from LaserScan: \n\n seq: %d \n stamp: %d \n frame_id: %s \n angle_min: %f \n angle_max: %f \n angle_increment: %f \n time_increment: %f \n scan_time: %f \n range_min: %f \n range_max: %f \n ranges: %f \n intensities: %f", 
                                                                                       data.header.seq, data.header.stamp, data.header.frame_id.c_str(), data.angle_min, 
                                                                                       data.angle_max, data.angle_increment, data.time_increment, data.scan_time,
                                                                                       data.range_min, data.range_max, data.ranges, data.intensities);
}

/*void dataRestructuring(const sensor_msgs::LaserScan::ConstPtr& msg)//---------------------------------------------------
{
  sensor_msgs::PointCloud2 data = *msg;

}*/
                                                        
int main(int argc, char **argv)//--------------------------------------------
{
  init(argc, argv, "LaserScan_Listener"); // Funcion de ROS que inicializa el nodo que vamos a lanzar

  NodeHandle n; // Declaracion del nodo a utilizar para la comunicacion

  Subscriber sub = n.subscribe("scan", 1000, laserScancallback);

  Rate loop_rate(1);

  spin();

  return 0;
}   