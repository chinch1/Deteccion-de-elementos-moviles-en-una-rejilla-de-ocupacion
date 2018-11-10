#include "velodyne_laserscan/VelodyneLaserScan.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/String.h"
#include <std_msgs/Int8.h>
#include <vector>
#include <cstdlib>
#include <iostream>
#include <string>
#include <sstream> // for std::stringstream
#include <math.h>
#include <algorithm>

using namespace std;

//Codigo que construye el mensaje LaserScan (Nicola)
//---------------------------------------------------------------------------------------------------------------------
  namespace velodyne_laserscan {
      
    VelodyneLaserScan::VelodyneLaserScan(ros::NodeHandle &nh, ros::NodeHandle &nh_priv) : ring_count_(0), nh_(nh), srv_(nh_priv) {

      ros::SubscriberStatusCallback connect_cb = boost::bind(&VelodyneLaserScan::connectCb, this);

      pub_ = nh.advertise<sensor_msgs::LaserScan>("scan2", 10, connect_cb, connect_cb);
      
      ROS_INFO("Bandera 1: Voy a publicar en el topico /scan \n");

      srv_.setCallback(boost::bind(&VelodyneLaserScan::reconfig, this, _1, _2));
    }

    void VelodyneLaserScan::connectCb() { // Sirve para suscribir el nodo al topico "velodyne_points" en caso de que sub_ =  0.
    
      boost::lock_guard<boost::mutex> lock(connect_mutex_);

      if (!pub_.getNumSubscribers()) {

        sub_.shutdown();

      } else if (!sub_) {

        sub_ = nh_.subscribe("velodyne_points", 10, &VelodyneLaserScan::recvCallback, this);
        ROS_INFO("Bandera 3 : Estoy suscrito al topico velodyne_points \n");
      }
    }

    void VelodyneLaserScan::recvCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {  
      // Latch ring count
      //-----------------------------------------------------------------------------------------------------------
        if (!ring_count_) {

          // Check for PointCloud2 field 'ring'
          bool found = false;
          for (size_t i = 0; i < msg->fields.size(); i++) {
            if (msg->fields[i].datatype == sensor_msgs::PointField::UINT16) {
              if (msg->fields[i].name == "ring") {
                found = true;
                break;
              }
            }
          }
          
          if (!found) {
            ROS_ERROR("VelodyneLaserScan: Field 'ring' of type 'UINT16' not present in PointCloud2");
            return;
          }

          for (sensor_msgs::PointCloud2ConstIterator<uint16_t> it(*msg, "ring"); it != it.end(); ++it) {
            const uint16_t ring = *it;
            // if (ring + 1 > ring_count_) {
            //   ring_count_ = ring + 1;
            // ROS_INFO_ONCE("ring: %u \n", ring);
            }
          
          // if (ring_count_) {
          //   ROS_INFO("VelodyneLaserScan: Latched ring count of %u", ring_count_);

          // } else {
          //   ROS_ERROR("VelodyneLaserScan: Field 'ring' of type 'UINT16' not present in PointCloud2");
          //   return;
          // }
        }

      // Select ring to use
      //-----------------------------------------------------------------------------------------------------------
        uint16_t ring;
        if ((cfg_.ring < 0) || (cfg_.ring >= ring_count_)) {
          // Default to ring closest to being level for each known sensor
          if (ring_count_ > 32) {
            ring = 57; // HDL-64E
          } else if (ring_count_ > 16) {
            ring = 23; // HDL-32E
          } else {
            ring = 8; // VLP-16
          }
        } else {
          ring = cfg_.ring;
        }
          
        // ROS_INFO_ONCE("VelodyneLaserScan: Extracting ring %u \n", ring);

      // Load structure of PointCloud2
      //-----------------------------------------------------------------------------------------------------------
        int offset_x = -1;
        int offset_y = -1;
        int offset_z = -1;
        int offset_i = -1;
        int offset_r = -1;
        for (size_t i = 0; i < msg->fields.size(); i++) {
          if (msg->fields[i].datatype == sensor_msgs::PointField::FLOAT32) {
            if (msg->fields[i].name == "x") {
              offset_x = msg->fields[i].offset;
            } else if (msg->fields[i].name == "y") {
              offset_y = msg->fields[i].offset;
            } else if (msg->fields[i].name == "z") {
              offset_z = msg->fields[i].offset;
            } else if (msg->fields[i].name == "intensity") {
              offset_i = msg->fields[i].offset;
            }
        
          } else if (msg->fields[i].datatype == sensor_msgs::PointField::UINT16) {
            if (msg->fields[i].name == "ring") {
              offset_r = msg->fields[i].offset;
            }
          }
        }

      // Construct LaserScan message
      //-----------------------------------------------------------------------------------------------------------
      
      if ((offset_x >= 0) && (offset_y >= 0) && (offset_r >= 0)) {

        const float RESOLUTION = abs(cfg_.resolution);
        const size_t SIZE = 2.0 * M_PI / RESOLUTION; // SIZE = 897 if RESOLUTION = 0.007 rads
        
        const float z_velodyne = 1; //Adquisicion de la altura del velodyne por parametros (PENDIENTE)
        
        sensor_msgs::LaserScanPtr scan(new sensor_msgs::LaserScan());
        scan->header = msg->header;
        scan->angle_increment = RESOLUTION;
        scan->angle_min = -M_PI;
        scan->angle_max = M_PI;
        scan->range_min = 0.0;
        scan->range_max = 200.0;
        scan->time_increment = 0.0;
        scan->ranges.resize(SIZE, INFINITY);

        float W_hit[16][896];   
        float W_inc[16][896];
      
        if ((offset_x == 0) && (offset_y == 4) && (offset_z == 8) && (offset_i == 16) && (offset_r == 20)) {  
          scan->intensities.resize(SIZE);
          for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it) {

            const uint16_t r = *((const uint16_t*)(&it[5])); // ring // order: 0,8,1,9,2,10,3,11,4,12,5,13,6,14,7,15

            // if (r > 6 && r <9) { // Discard layers outside of w = (-3º, 3º)

            // if (r <= 7) {  // W_INC respecto al suelo // Desmarcar para todos los anillos
              if (r == 0) {

                // ROS_INFO("ring: %u \n", r);
                const float zero = 0.0;
                const float x_hit = it[0]; // x
                const float y_hit = it[1]; // y
                const float z_hit = it[2]; // z
                const float i = it[4]; // intensity
                const int sample = (atan2f(y_hit, x_hit) + (float)M_PI) / RESOLUTION; // Whole value that runs through each one of the samples made. (897 for each ring)
                // ROS_INFO("sample1: %d \n", sample);

                if ((sample >= 0) && (sample < (int)SIZE)) {
              
                  scan->ranges[sample] = sqrtf(x_hit * x_hit + y_hit * y_hit); // Distance to the hit, once the object was proyected in the 2D plane.  
                  // ROS_INFO_ONCE("range: %f", scan->ranges[sample]);           
           
                  scan->intensities[sample] = i;                          // Intensity of the hit 

                  const float w_angle = asin(z_hit/scan->ranges[sample]); // w angle through the rings (rads)
                  // ROS_INFO_ONCE("w: %f", w_angle*180/M_PI);

                  const float aux1 = (1 + (scan->ranges[sample]*tan(w_angle))/z_velodyne); 
                  // ROS_INFO_ONCE("aux1: %f", aux1);
                  // ROS_INFO_ONCE("tan(w_angle): %f", tan(w_angle));

                  const float RESOLUTION_DEGREES = RESOLUTION*180/M_PI; // alpha angle through the samples (degrees)

                  W_inc[r][sample] = max(aux1,zero);

                    // ROS_INFO("W_inc: %f", W_inc[sample]);            
                    // ROS_INFO("%d", sample);
                } 
              } /*else if (r == 1) {

                // ROS_INFO("ring: %u \n", r);
                const float zero = 0.0;
                const float x_hit = it[0]; // x
                const float y_hit = it[1]; // y
                const float z_hit = it[2]; // z
                const float i = it[4]; // intensity
                const int sample = (atan2f(y_hit, x_hit) + (float)M_PI) / RESOLUTION; // Whole value that runs through each one of the samples made. (897 for each ring)
                // ROS_INFO("sample1: %d \n", sample);

                if ((sample >= 0) && (sample < (int)SIZE)) {
              
                  scan->ranges[sample] = sqrtf(x_hit * x_hit + y_hit * y_hit); // Distance to the hit, once the object was proyected in the 2D plane.  
                  // ROS_INFO_ONCE("range: %f", scan->ranges[sample]);           
           
                  scan->intensities[sample] = i;                          // Intensity of the hit 

                  const float w_angle = asin(z_hit/scan->ranges[sample]); // w angle through the rings (rads)
                  // ROS_INFO_ONCE("w: %f", w_angle*180/M_PI);

                  const float aux1 = (1 + (scan->ranges[sample]*tan(w_angle))/z_velodyne); 
                  // ROS_INFO_ONCE("aux1: %f", aux1);
                  ROS_INFO_ONCE("tan(w_angle): %f", tan(w_angle));

                  const float RESOLUTION_DEGREES = RESOLUTION*180/M_PI; // alpha angle through the samples (degrees)

                  W_inc[r][sample] = max(aux1,zero);

                    // ROS_INFO("W_inc: %f", W_inc[sample]);            
                    // ROS_INFO("%d", sample);
                } 
              } else if (r == 2) {

                // ROS_INFO("ring: %u \n", r);
                const float zero = 0.0;
                const float x_hit = it[0]; // x
                const float y_hit = it[1]; // y
                const float z_hit = it[2]; // z

                const float i = it[4]; // intensity
                const int sample = (atan2f(y_hit, x_hit) + (float)M_PI) / RESOLUTION; // Whole value that runs through each one of the samples made. (897 for each ring)
                // ROS_INFO("sample1: %d \n", sample);

                if ((sample >= 0) && (sample < (int)SIZE)) {
              
                  scan->ranges[sample] = sqrtf(x_hit * x_hit + y_hit * y_hit); // Distance to the hit, once the object was proyected in the 2D plane.  
                  // ROS_INFO_ONCE("range: %f", scan->ranges[sample]);           

                  scan->intensities[sample] = i;                          // Intensity of the hit 

                  const float w_angle = asin(z_hit/scan->ranges[sample]); // w angle through the rings (rads)
                  // ROS_INFO_ONCE("w: %f", w_angle*180/M_PI);

                  const float aux1 = (1 + (scan->ranges[sample]*abs(tan(w_angle)))/z_velodyne); 
                  // ROS_INFO_ONCE("aux1: %f", aux1);
                  // ROS_INFO_ONCE("tan(w_angle): %f", tan(w_angle));

                  const float RESOLUTION_DEGREES = RESOLUTION*180/M_PI; // Alpha angle through the samples (degrees)

                  W_inc[r][sample] = max(aux1,zero);
                  // ROS_INFO("W_inc: %f", W_inc[sample]);            
                  // ROS_INFO("%d", sample);

                } 
              } else if (r == 3) {

                // ROS_INFO("ring: %u \n", r);
                const float zero = 0.0;
                const float x_hit = it[0]; // x
                const float y_hit = it[1]; // y
                const float z_hit = it[2]; // z
                const float i = it[4]; // intensity
                const int sample = (atan2f(y_hit, x_hit) + (float)M_PI) / RESOLUTION; // Whole value that runs through each one of the samples made. (897 for each ring)
                // ROS_INFO("sample1: %d \n", sample);

                if ((sample >= 0) && (sample < (int)SIZE)) {
              
                  scan->ranges[sample] = sqrtf(x_hit * x_hit + y_hit * y_hit); // Distance to the hit, once the object was proyected in the 2D plane.  
                  // ROS_INFO_ONCE("range: %f", scan->ranges[sample]);           
           
                  scan->intensities[sample] = i;                          // Intensity of the hit 

                  const float w_angle = asin(z_hit/scan->ranges[sample]); // w angle through the rings (rads)
                  // ROS_INFO_ONCE("w: %f", w_angle*180/M_PI);

                  const float aux1 = (1 + (scan->ranges[sample]*tan(w_angle))/z_velodyne); 
                  // ROS_INFO_ONCE("aux1: %f", aux1);
                  // ROS_INFO_ONCE("tan(w_angle): %f", tan(w_angle));

                  const float RESOLUTION_DEGREES = RESOLUTION*180/M_PI; // alpha angle through the samples (degrees)

                  W_inc[r][sample] = max(aux1,zero);
                  // ROS_INFO("W_inc: %f", W_inc[sample]);            
                  // ROS_INFO("%d", sample);

                } 
              } else if (r == 4) {

                // ROS_INFO("ring: %u \n", r);
                const float zero = 0.0;
                const float x_hit = it[0]; // x
                const float y_hit = it[1]; // y
                const float z_hit = it[2]; // z
                const float i = it[4]; // intensity
                const int sample = (atan2f(y_hit, x_hit) + (float)M_PI) / RESOLUTION; // Whole value that runs through each one of the samples made. (897 for each ring)
                // ROS_INFO("sample1: %d \n", sample);

                if ((sample >= 0) && (sample < (int)SIZE)) {
              
                  scan->ranges[sample] = sqrtf(x_hit * x_hit + y_hit * y_hit); // Distance to the hit, once the object was proyected in the 2D plane.  
                  // ROS_INFO_ONCE("range: %f", scan->ranges[sample]);           
           
                  scan->intensities[sample] = i;                          // Intensity of the hit 

                  const float w_angle = asin(z_hit/scan->ranges[sample]); // w angle through the rings (rads)
                  // ROS_INFO_ONCE("w: %f", w_angle*180/M_PI);

                  const float aux1 = (1 + (scan->ranges[sample]*tan(w_angle))/z_velodyne); 
                  // ROS_INFO_ONCE("aux1: %f", aux1);
                  // ROS_INFO_ONCE("tan(w_angle): %f", tan(w_angle));

                  const float RESOLUTION_DEGREES = RESOLUTION*180/M_PI; // alpha angle through the samples (degrees)

                  W_inc[r][sample] = max(aux1,zero);

                    // ROS_INFO("W_inc: %f", W_inc[sample]);            
                    // ROS_INFO("%d", sample);
                } 
              } else if (r == 5) {

                // ROS_INFO("ring: %u \n", r);
                const float zero = 0.0;
                const float x_hit = it[0]; // x
                const float y_hit = it[1]; // y
                const float z_hit = it[2]; // z
                const float i = it[4]; // intensity
                const int sample = (atan2f(y_hit, x_hit) + (float)M_PI) / RESOLUTION; // Whole value that runs through each one of the samples made. (897 for each ring)
                // ROS_INFO("sample1: %d \n", sample);

                if ((sample >= 0) && (sample < (int)SIZE)) {
              
                  scan->ranges[sample] = sqrtf(x_hit * x_hit + y_hit * y_hit); // Distance to the hit, once the object was proyected in the 2D plane.  
                  // ROS_INFO_ONCE("range: %f", scan->ranges[sample]);           
           
                  scan->intensities[sample] = i;                          // Intensity of the hit 

                  const float w_angle = asin(z_hit/scan->ranges[sample]); // w angle through the rings (rads)
                  // ROS_INFO_ONCE("w: %f", w_angle*180/M_PI);

                  const float aux1 = (1 + (scan->ranges[sample]*tan(w_angle))/z_velodyne); 
                  // ROS_INFO_ONCE("aux1: %f", aux1);
                  // ROS_INFO_ONCE("tan(w_angle): %f", tan(w_angle));

                  const float RESOLUTION_DEGREES = RESOLUTION*180/M_PI; // alpha angle through the samples (degrees)

                  W_inc[r][sample] = max(aux1,zero);

                    // ROS_INFO("W_inc: %f", W_inc[sample]);            
                    // ROS_INFO("%d", sample);
                } 
              } else if (r == 6) {

                // ROS_INFO("ring: %u \n", r);
                const float zero = 0.0;
                const float x_hit = it[0]; // x
                const float y_hit = it[1]; // y
                const float z_hit = it[2]; // z
                const float i = it[4]; // intensity
                const int sample = (atan2f(y_hit, x_hit) + (float)M_PI) / RESOLUTION; // Whole value that runs through each one of the samples made. (897 for each ring)
                // ROS_INFO("sample1: %d \n", sample);

                if ((sample >= 0) && (sample < (int)SIZE)) {
              
                  scan->ranges[sample] = sqrtf(x_hit * x_hit + y_hit * y_hit); // Distance to the hit, once the object was proyected in the 2D plane.  
                  // ROS_INFO_ONCE("range: %f", scan->ranges[sample]);           
           
                  scan->intensities[sample] = i;                          // Intensity of the hit 

                  const float w_angle = asin(z_hit/scan->ranges[sample]); // w angle through the rings (rads)
                  // ROS_INFO_ONCE("w: %f", w_angle*180/M_PI);

                  const float aux1 = (1 + (scan->ranges[sample]*tan(w_angle))/z_velodyne); 
                  ROS_INFO_ONCE("aux1: %f", aux1);
                  // ROS_INFO_ONCE("tan(w_angle): %f", tan(w_angle));

                  const float RESOLUTION_DEGREES = RESOLUTION*180/M_PI; // alpha angle through the samples (degrees)

                  W_inc[r][sample] = max(aux1,zero);

                    // ROS_INFO("W_inc: %f", W_inc[sample]);            
                    // ROS_INFO("%d", sample);
                } 
              } else {

                // ROS_INFO("ring: %u \n", r);
                const float zero = 0.0;
                const float x_hit = it[0]; // x
                const float y_hit = it[1]; // y
                const float z_hit = it[2]; // z
                const float i = it[4]; // intensity
                const int sample = (atan2f(y_hit, x_hit) + (float)M_PI) / RESOLUTION; // Whole value that runs through each one of the samples made. (897 for each ring)
                // ROS_INFO("sample1: %d \n", sample);

                if ((sample >= 0) && (sample < (int)SIZE)) {
              
                  scan->ranges[sample] = sqrtf(x_hit * x_hit + y_hit * y_hit); // Distance to the hit, once the object was proyected in the 2D plane.  
                  // ROS_INFO_ONCE("range: %f", scan->ranges[sample]);           
           
                  scan->intensities[sample] = i;                          // Intensity of the hit 

                  const float w_angle = asin(z_hit/scan->ranges[sample]); // w angle through the rings (rads)
                  // ROS_INFO_ONCE("w: %f", w_angle*180/M_PI);

                  const float aux1 = (1 + (scan->ranges[sample]*tan(w_angle))/z_velodyne); 
                  // ROS_INFO_ONCE("aux1: %f", aux1);
                  // ROS_INFO_ONCE("tan(w_angle): %f", tan(w_angle));

                  const float RESOLUTION_DEGREES = RESOLUTION*180/M_PI; // alpha angle through the samples (degrees)

                  W_inc[r][sample] = max(aux1,zero);

                    // ROS_INFO("W_inc: %f", W_inc[sample]);            
                    // ROS_INFO("%d", sample);
                } 
              } 
            } else {

              if (r == 8) {

                // ROS_INFO("ring: %u \n", r);
                const float zero = 0.0;
                const float x_hit = it[0]; // x
                const float y_hit = it[1]; // y
                const float z_hit = it[2]; // z
                const float i = it[4]; // intensity
                const int sample = (atan2f(y_hit, x_hit) + (float)M_PI) / RESOLUTION; // Whole value that runs through each one of the samples made. (897 for each ring)
                // ROS_INFO("sample1: %d \n", sample);

                if ((sample >= 0) && (sample < (int)SIZE)) {
              
                  scan->ranges[sample] = sqrtf(x_hit * x_hit + y_hit * y_hit); // Distance to the hit, once the object was proyected in the 2D plane.  
                  // ROS_INFO_ONCE("range: %f", scan->ranges[sample]);           
           
                  scan->intensities[sample] = i;                          // Intensity of the hit 

                  const float w_angle = asin(z_hit/scan->ranges[sample]); // w angle through the rings (rads)
                  // ROS_INFO_ONCE("w: %f", w_angle*180/M_PI);

                  const float aux1 = (1 + (z_hit*tan(w_angle))/z_velodyne); 
                  // ROS_INFO_ONCE("aux1: %f", aux1);
                  // ROS_INFO_ONCE("tan(w_angle): %f", tan(w_angle));

                  const float RESOLUTION_DEGREES = RESOLUTION*180/M_PI; // alpha angle through the samples (degrees)

                  W_inc[r][sample] = max(aux1,zero);

                    // ROS_INFO("W_inc: %f", W_inc[sample]);            
                    // ROS_INFO("%d", sample);
                } 
              } else if (r == 9) {

                // ROS_INFO("ring: %u \n", r);
                const float zero = 0.0;
                const float x_hit = it[0]; // x
                const float y_hit = it[1]; // y
                const float z_hit = it[2]; // z
                const float i = it[4]; // intensity
                const int sample = (atan2f(y_hit, x_hit) + (float)M_PI) / RESOLUTION; // Whole value that runs through each one of the samples made. (897 for each ring)
                // ROS_INFO("sample1: %d \n", sample);

                if ((sample >= 0) && (sample < (int)SIZE)) {
              
                  scan->ranges[sample] = sqrtf(x_hit * x_hit + y_hit * y_hit); // Distance to the hit, once the object was proyected in the 2D plane.  
                  // ROS_INFO_ONCE("range: %f", scan->ranges[sample]);           
           
                  scan->intensities[sample] = i;                          // Intensity of the hit 

                  const float w_angle = asin(z_hit/scan->ranges[sample]); // w angle through the rings (rads)
                  // ROS_INFO_ONCE("w: %f", w_angle*180/M_PI);

                  const float aux1 = (1 + (z_hit*tan(w_angle))/z_velodyne); 
                  // ROS_INFO_ONCE("aux1: %f", aux1);
                  // ROS_INFO_ONCE("tan(w_angle): %f", tan(w_angle));

                  const float RESOLUTION_DEGREES = RESOLUTION*180/M_PI; // alpha angle through the samples (degrees)

                  W_inc[r][sample] = max(aux1,zero);

                    // ROS_INFO("W_inc: %f", W_inc[sample]);            
                    // ROS_INFO("%d", sample);
                } 
              } else if (r == 10) {

                // ROS_INFO("ring: %u \n", r);
                const float zero = 0.0;
                const float x_hit = it[0]; // x
                const float y_hit = it[1]; // y
                const float z_hit = it[2]; // z
                const float i = it[4]; // intensity
                const int sample = (atan2f(y_hit, x_hit) + (float)M_PI) / RESOLUTION; // Whole value that runs through each one of the samples made. (897 for each ring)
                // ROS_INFO("sample1: %d \n", sample);

                if ((sample >= 0) && (sample < (int)SIZE)) {
              
                  scan->ranges[sample] = sqrtf(x_hit * x_hit + y_hit * y_hit); // Distance to the hit, once the object was proyected in the 2D plane.  
                  // ROS_INFO_ONCE("range: %f", scan->ranges[sample]);           
           
                  scan->intensities[sample] = i;                          // Intensity of the hit 

                  const float w_angle = asin(z_hit/scan->ranges[sample]); // w angle through the rings (rads)
                  // ROS_INFO_ONCE("w: %f", w_angle*180/M_PI);

                  const float aux1 = (1 + (z_hit*tan(w_angle))/z_velodyne); 
                  // ROS_INFO_ONCE("aux1: %f", aux1);
                  // ROS_INFO_ONCE("tan(w_angle): %f", tan(w_angle));

                  const float RESOLUTION_DEGREES = RESOLUTION*180/M_PI; // alpha angle through the samples (degrees)

                  W_inc[r][sample] = max(aux1,zero);

                    // ROS_INFO("W_inc: %f", W_inc[sample]);            
                    // ROS_INFO("%d", sample);
                } 
              } else if (r == 11) {

                // ROS_INFO("ring: %u \n", r);
                const float zero = 0.0;
                const float x_hit = it[0]; // x
                const float y_hit = it[1]; // y
                const float z_hit = it[2]; // z
                const float i = it[4]; // intensity
                const int sample = (atan2f(y_hit, x_hit) + (float)M_PI) / RESOLUTION; // Whole value that runs through each one of the samples made. (897 for each ring)
                // ROS_INFO("sample1: %d \n", sample);

                if ((sample >= 0) && (sample < (int)SIZE)) {
              
                  scan->ranges[sample] = sqrtf(x_hit * x_hit + y_hit * y_hit); // Distance to the hit, once the object was proyected in the 2D plane.  
                  // ROS_INFO_ONCE("range: %f", scan->ranges[sample]);           
           
                  scan->intensities[sample] = i;                          // Intensity of the hit 

                  const float w_angle = asin(z_hit/scan->ranges[sample]); // w angle through the rings (rads)
                  // ROS_INFO_ONCE("w: %f", w_angle*180/M_PI);

                  const float aux1 = (1 + (z_hit*tan(w_angle))/z_velodyne); 
                  // ROS_INFO_ONCE("aux1: %f", aux1);
                  // ROS_INFO_ONCE("tan(w_angle): %f", tan(w_angle));

                  const float RESOLUTION_DEGREES = RESOLUTION*180/M_PI; // alpha angle through the samples (degrees)

                  W_inc[r][sample] = max(aux1,zero);

                    // ROS_INFO("W_inc: %f", W_inc[sample]);            
                    // ROS_INFO("%d", sample);
                } 
              } else if (r == 12) {

                // ROS_INFO("ring: %u \n", r);
                const float zero = 0.0;
                const float x_hit = it[0]; // x
                const float y_hit = it[1]; // y
                const float z_hit = it[2]; // z
                const float i = it[4]; // intensity
                const int sample = (atan2f(y_hit, x_hit) + (float)M_PI) / RESOLUTION; // Whole value that runs through each one of the samples made. (897 for each ring)
                // ROS_INFO("sample1: %d \n", sample);

                if ((sample >= 0) && (sample < (int)SIZE)) {
              
                  scan->ranges[sample] = sqrtf(x_hit * x_hit + y_hit * y_hit); // Distance to the hit, once the object was proyected in the 2D plane.  
                  // ROS_INFO_ONCE("range: %f", scan->ranges[sample]);           
           
                  scan->intensities[sample] = i;                          // Intensity of the hit 

                  const float w_angle = asin(z_hit/scan->ranges[sample]); // w angle through the rings (rads)
                  // ROS_INFO_ONCE("w: %f", w_angle*180/M_PI);

                  const float aux1 = (1 + (z_hit*tan(w_angle))/z_velodyne); 
                  // ROS_INFO_ONCE("aux1: %f", aux1);
                  // ROS_INFO_ONCE("tan(w_angle): %f", tan(w_angle));

                  const float RESOLUTION_DEGREES = RESOLUTION*180/M_PI; // alpha angle through the samples (degrees)

                  W_inc[r][sample] = max(aux1,zero);

                    // ROS_INFO("W_inc: %f", W_inc[sample]);            
                    // ROS_INFO("%d", sample);
                } 
              } else if (r == 13) {

                // ROS_INFO("ring: %u \n", r);
                const float zero = 0.0;
                const float x_hit = it[0]; // x
                const float y_hit = it[1]; // y
                const float z_hit = it[2]; // z
                const float i = it[4]; // intensity
                const int sample = (atan2f(y_hit, x_hit) + (float)M_PI) / RESOLUTION; // Whole value that runs through each one of the samples made. (897 for each ring)
                // ROS_INFO("sample1: %d \n", sample);

                if ((sample >= 0) && (sample < (int)SIZE)) {
              
                  scan->ranges[sample] = sqrtf(x_hit * x_hit + y_hit * y_hit); // Distance to the hit, once the object was proyected in the 2D plane.  
                  // ROS_INFO_ONCE("range: %f", scan->ranges[sample]);           
           
                  scan->intensities[sample] = i;                          // Intensity of the hit 

                  const float w_angle = asin(z_hit/scan->ranges[sample]); // w angle through the rings (rads)
                  // ROS_INFO_ONCE("w: %f", w_angle*180/M_PI);

                  const float aux1 = (1 + (z_hit*tan(w_angle))/z_velodyne); 
                  // ROS_INFO_ONCE("aux1: %f", aux1);
                  // ROS_INFO_ONCE("tan(w_angle): %f", tan(w_angle));

                  const float RESOLUTION_DEGREES = RESOLUTION*180/M_PI; // alpha angle through the samples (degrees)

                  W_inc[r][sample] = max(aux1,zero);

                    // ROS_INFO("W_inc: %f", W_inc[sample]);            
                    // ROS_INFO("%d", sample);
                } 
              } else if (r == 14) {

                // ROS_INFO("ring: %u \n", r);
                const float zero = 0.0;
                const float x_hit = it[0]; // x
                const float y_hit = it[1]; // y
                const float z_hit = it[2]; // z
                const float i = it[4]; // intensity
                const int sample = (atan2f(y_hit, x_hit) + (float)M_PI) / RESOLUTION; // Whole value that runs through each one of the samples made. (897 for each ring)
                // ROS_INFO("sample1: %d \n", sample);

                if ((sample >= 0) && (sample < (int)SIZE)) {
              
                  scan->ranges[sample] = sqrtf(x_hit * x_hit + y_hit * y_hit); // Distance to the hit, once the object was proyected in the 2D plane.  
                  // ROS_INFO_ONCE("range: %f", scan->ranges[sample]);           
           
                  scan->intensities[sample] = i;                          // Intensity of the hit 

                  const float w_angle = asin(z_hit/scan->ranges[sample]); // w angle through the rings (rads)
                  // ROS_INFO_ONCE("w: %f", w_angle*180/M_PI);

                  const float aux1 = (1 + (z_hit*tan(w_angle))/z_velodyne); 
                  // ROS_INFO_ONCE("aux1: %f", aux1);
                  // ROS_INFO_ONCE("tan(w_angle): %f", tan(w_angle));

                  const float RESOLUTION_DEGREES = RESOLUTION*180/M_PI; // alpha angle through the samples (degrees)

                  W_inc[r][sample] = max(aux1,zero);

                    // ROS_INFO("W_inc: %f", W_inc[sample]);            
                    // ROS_INFO("%d", sample);
                } 
              } else {// r = 15

                // ROS_INFO("ring: %u \n", r);
                const float zero = 0.0;
                const float x_hit = it[0]; // x
                const float y_hit = it[1]; // y
                const float z_hit = it[2]; // z
                const float i = it[4]; // intensity
                const int sample = (atan2f(y_hit, x_hit) + (float)M_PI) / RESOLUTION; // Whole value that runs through each one of the samples made. (897 for each ring)
                // ROS_INFO("sample1: %d \n", sample);

                if ((sample >= 0) && (sample < (int)SIZE)) {
              
                  scan->ranges[sample] = sqrtf(x_hit * x_hit + y_hit * y_hit); // Distance to the hit, once the object was proyected in the 2D plane.  
                  // ROS_INFO_ONCE("range: %f", scan->ranges[sample]);           
           
                  scan->intensities[sample] = i;                          // Intensity of the hit 

                  const float w_angle = asin(z_hit/scan->ranges[sample]); // w angle through the rings (rads)
                  // ROS_INFO_ONCE("w: %f", w_angle*180/M_PI);

                  const float aux1 = (1 + (z_hit*tan(w_angle))/z_velodyne); 
                  // ROS_INFO_ONCE("aux1: %f", aux1);
                  // ROS_INFO_ONCE("tan(w_angle): %f", tan(w_angle));

                  const float RESOLUTION_DEGREES = RESOLUTION*180/M_PI; // alpha angle through the samples (degrees)

                  W_inc[r][sample] = max(aux1,zero);

                    // ROS_INFO("W_inc: %f", W_inc[sample]);            
                    // ROS_INFO("%d", sample);
                }
              }
            }*/  
          }
        } /*else {
          ROS_WARN_ONCE("VelodyneLaserScan: PointCloud2 fields in unexpected order. Using slower generic method.");
          if (offset_i >= 0) {
            scan->intensities.resize(SIZE);
            sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_r(*msg, "ring");
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_i(*msg, "intensity");
            for ( ; iter_r != iter_r.end(); ++iter_x, ++iter_y, ++iter_r, ++iter_i) {
              const uint16_t r = *iter_r; // ring
              if (r == 0) {
                const float x = *iter_x; // x
                const float y = *iter_y; // y
                const float i = *iter_i; // intensity
                const int sample = (atan2f(y, x) + (float)M_PI) / RESOLUTION;
                if ((sample >= 0) && (sample < (int)SIZE)) {
                  scan->ranges[sample] = sqrtf(x * x + y * y);
                  scan->intensities[sample] = i;
                }
              }
            }
          } else {
            sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_r(*msg, "ring");
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
            for ( ; iter_r != iter_r.end(); ++iter_x, ++iter_y, ++iter_r) {
              const uint16_t r = *iter_r; // ring
              if (r == ring) {
                const float x = *iter_x; // x
                const float y = *iter_y; // y
                const int sample = (atan2f(y, x) + (float)M_PI) / RESOLUTION;
                if ((sample >= 0) && (sample < (int)SIZE)) {
                  scan->ranges[sample] = sqrtf(x * x + y * y);
                }
              }
            }
          }
        }*/
        ROS_INFO_ONCE("Valor aleatorio del vector: %f", W_inc[2][100]);

        // pub_.publish(scan);
      } else {
        ROS_ERROR("VelodyneLaserScan: PointCloud2 missing one or more required fields! (x,y,ring)");
      }
    }

    void VelodyneLaserScan::reconfig(VelodyneLaserScanConfig& config, uint32_t level) {

      cfg_ = config;
    }
  }
//---------------------------------------------------------------------------------------------------------------------

//Codigo que dibuja el mapa (Eladio)
//-------------------------------------------------------------------------------------------------------------------

  ros::Publisher map_pub;
  ros::Subscriber sub;

  nav_msgs::OccupancyGrid gridMap;

  void laserScancallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  int assign_points(uint width, uint height, int fila, int columna);
  void bresenhamLine(int x0, int y0, int x1, int y1, std::vector<int8_t> &dataProb);

  nav_msgs::OccupancyGrid generate_Grid_Map(nav_msgs::OccupancyGrid gridMap, std::vector<int8_t> dataProb);

  void laserScancallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

    sensor_msgs::LaserScan data = *msg; // La funcion callback de ROS guarda la data cruda en la memoria, por lo tanto para extraerla hay que apuntar a la direccion de memoria.

    nav_msgs::OccupancyGrid gridMap_pub;

    double angle_rad = data.angle_min; //angulo minimo en radianes del velodyne (desde donde parte el movimiento de escaneo)
    double angle_degrees = 0.0, dist_x_m = 0.0, dist_y_m = 0.0;

    int posX = 0, posY = 0, pos = 0, len = gridMap.info.height*gridMap.info.width, coord_x = 0, coord_y = 0;
    int center_x = gridMap.info.width/2;
    int center_y = gridMap.info.height/2;
    
    vector<int8_t> dataProb;
    dataProb.assign(len,50);

    /*
       *La estructura de datos esta comprendida por los siguientes 12 valores, 
       *divididos entre los tipos string, uint y float.
      
       *uint32 seq
       *time stamp     <------- Segundos, concatenados con nanosegundos 
       *string frame_id

       *float32 angle_min
       *float32 angle_max
       *float32 angle_increment
       *float32 time_increment
       *float32 scan_time
       *float32 range_min
       *float32 range_max
       *float32[] ranges
       *float32[] intensities
      
        ROS_INFO(" \n\n I'm receiving from LaserScan: \n\n seq: %d \n stamp: %d \n frame_id: %s \n angle_min: %f \n angle_max: %f \n angle_increment: %f \n time_increment: %f \n scan_time: %f \n range_min: %f \n range_max: %f \n ", 
                                                                                           data.header.seq, data.header.stamp, data.header.frame_id.c_str(), data.angle_min, 
                                                                                           data.angle_max, data.angle_increment, data.time_increment, data.scan_time,
                                                                                           data.range_min, data.range_max);
      
        *ROS_INFO("ranges: \n");

        for(int i = 0; i < data.ranges.size(); i++)  
        {

          std::cout << data.ranges[i] << ", " ; 
        }

        ROS_INFO("intensities: \n");

        for(int i = 0; i < data.intensities.size(); i++)
        {

          std::cout << data.intensities[j] << ", " ;
        }
       
        ROS_INFO("angles: \n");

        for(int i = 0; i < data.ranges.size(); i++)
        {
          angle_rad = angle_rad + data.angle_increment;
          angle_degrees = angle_rad*180/M_PI;
          std::cout << angle_degrees << ", " ;
        }

        angle_rad = data.angle_min;
        std::cout <<"\n" ;
    */

    for(int i = 0; i < data.ranges.size(); i++) {

      if(i>0) { 

        angle_rad = angle_rad + data.angle_increment;
      }

      if(data.ranges[i] <= 200) {// 200 m es la maxima distancia que nos provee el VLP-16 
      
        dist_x_m = data.ranges[i] * cos(angle_rad);
        dist_y_m = data.ranges[i] * sin(angle_rad);

        coord_x = dist_x_m/gridMap.info.resolution;
        coord_y = dist_y_m/gridMap.info.resolution;
      
      } else {//Si hay una distancia que tiende a inf, esta se asigna a 250 m que esta fuera del rango del velodyne

        dist_x_m = 250 * cos(angle_rad);
        dist_y_m = 250 * sin(angle_rad);
        coord_x = dist_x_m/gridMap.info.resolution;
        coord_y = dist_y_m/gridMap.info.resolution;
      
      }
      
      angle_degrees = angle_rad*180/M_PI;    

      posX = center_x + coord_x;
      posY = center_y + coord_y;
      
      cout << "center = [" << center_x << ", " <<center_y << "] \n\n";    
      cout << "Hola1 coord_x = " << coord_x << "  coord_y = " << coord_y << "  posX = " << posX << "  posy = " << posY << "  distancia = " << data.ranges[i] << "  angulo = " << angle_degrees << "  pos = " << pos << "\n\n";

      bresenhamLine(center_x, center_y, posX, posY, dataProb);
      
      cout << "Hola2 coord_x = " << coord_x << "  coord_y = " << coord_y << "  posX = " << posX << "  posy = " << posY << "  distancia = " << data.ranges[i] << "  angulo = " << angle_degrees << "  pos = " << pos << "\n\n";
        
      if ((abs(coord_x) < gridMap.info.width/2) && (abs(coord_y) < gridMap.info.height/2)) {

        cout << "Chao1 coord_x = " << coord_x << "  coord_y = " << coord_y << "  posX = " << posX << "  posy = " << posY << "  distancia = " << data.ranges[i] << "  angulo = " << angle_degrees << "  pos = " << pos << "\n\n";
        
        pos = assign_points(gridMap.info.width, gridMap.info.height, posX, posY);      
        dataProb.at(pos) = 100;
        
        cout << "Chao2 coord_x = " << coord_x << "  coord_y = " << coord_y << "  posX = " << posX << "  posy = " << posY << "  distancia = " << data.ranges[i] << "  angulo = " << angle_degrees << "  pos = " << pos << "\n\n";
      }
    }

    gridMap_pub = generate_Grid_Map(gridMap, dataProb);
    map_pub.publish(gridMap_pub);
  }
  //-------------------------------------------------------------

  int assign_points(uint width, uint height, int columna, int fila) {
    int position = width * (fila) + columna;
    return position;
  }

  //-------------------------------------------------------------
  void bresenhamLine(int x0, int y0, int x1, int y1, std::vector<int8_t> &dataProb) {
    int dy = y1-y0; 
    int dx = x1-x0;
    int incXi, incYi, incXr, incYr;
    
    int xini = x0, yini = y0, xfin = x1, yfin = y1;
    int pos;

    // 1 - Incrementos para las secciones con avance inclinado
    if(dy >= 0) {

      incYi = 1;

    } else {

      incYi = -1;
      dy = -dy;
    }

    if(dx >= 0) { 

      incXi = 1;

    } else {

      incXi = -1;
      dx = -dx;
    }

    // 2 - Incrementos para las secciones con avance recto:
    if (dx >= dy) {

      incYr = 0;
      incXr = incXi;

    } else {

      incYr = incYi;
      incXr = 0;

      // Cuando dy es mayor que dx, se intercambian, para reutilizar el mismo bucle.
      // ver octantes blancos en la imagen encima del código
      int k = dx; dx = dy; dy = k;
    }

    // 3  - Inicializar valores (y de error).
    int x = x0, y = y0;
    int avR = (2 * dy);
    int av = (avR - dx);
    int avI = (av - dx);

    // 4  - Bucle para el trazado de las línea.
    do {

      pos = assign_points(gridMap.info.width, gridMap.info.height, x, y);

      if(dataProb.at(pos) != 100) dataProb.at(pos) = 0;

      if (av >= 0) {

        x = (x + incXi);     // X aumenta en inclinado.
        y = (y + incYi);     // Y aumenta en inclinado.
        av = (av + avI);     // Avance Inclinado

      } else {

        x = (x + incXr);     // X aumenta en recto.
        y = (y + incYr);     // Y aumenta en recto.
        av = (av + avR);     // Avance Recto
      }

    } while(((x != x1) || (y != y1)) && (x != gridMap.info.width) && (y != gridMap.info.height) && (x >= 0) && (y >= 0));
  }

  //-------------------------------------------------------------
  nav_msgs::OccupancyGrid generate_Grid_Map(nav_msgs::OccupancyGrid gridMap, std::vector<int8_t> dataProb) {

    gridMap.data = dataProb; 
    return gridMap;
  }

  int main(int argc, char *argv[])
  {
   
    ros::init(argc, argv, "Occupancy_Grid");

    ROS_INFO("Nodo Occupancy_Grid \n");
    
    gridMap.info.origin.position.x = 0;//ver como se asigna un point
    gridMap.info.origin.position.y = 0;
    gridMap.info.origin.position.z = 0;
    gridMap.info.resolution = 0.01; // resolution is in meters
    gridMap.info.width = 10;
    gridMap.info.height = 10;
    gridMap.header.frame_id = "map";

    std::string arg;

    for (int i = 1; i<argc; i++) {
      
      arg = argv[i];
      
      if (arg == "-h") {
        
        std::cout << "Este nodo genera la rejilla probabilistica de ocupacion \n";
        std::cout << "  Parametros: \n";
        std::cout << "\t--r\t'resolution' \n";
        std::cout << "\t--w\t'width' \n";
        std::cout << "\t--h\t'height' \n";
        std::cout << "\t--f\t'frame_id' \n";
        std::cout << "\n \n";
        std::cout << "Por default se genera una rejilla con:\n\n";
        std::cout << "\tresolution\t= 0.01 \n";
        std::cout << "\twidth\t\t= 10 \n";
        std::cout << "\theight\t\t= 10 \n";
        std::cout << "\tframe_id\t= map \n";
        std::cout << "\n\n";
        exit(1);
      
      } else if(argc > (i+1)) { 
          
          if (arg == "--r") {

            std::stringstream convert(argv[i+1]);

            if (!(convert >> gridMap.info.resolution)) gridMap.info.resolution = 0;
            ROS_INFO("resolution = %f \n",gridMap.info.resolution);
            
          } else if(arg == "--w") {

              std::stringstream convert(argv[i+1]);

              if (!(convert >> gridMap.info.width)) gridMap.info.width = 0;
              gridMap.info.origin.position.x = gridMap.info.width*gridMap.info.resolution/2;
              gridMap.info.origin.position.x = -gridMap.info.origin.position.x;
              ROS_INFO("width = %u \n", gridMap.info.width);
            
          } else if(arg == "--h") {
              
              std::stringstream convert(argv[i+1]);

              if (!(convert >> gridMap.info.height)) gridMap.info.height = 0; 
              gridMap.info.origin.position.y = gridMap.info.height*gridMap.info.resolution/2;
              gridMap.info.origin.position.y = -gridMap.info.origin.position.y;
              ROS_INFO("height = %u \n", gridMap.info.height); 
          
          } else if(arg == "--f") {
              gridMap.header.frame_id = argv[i+1];
              ROS_INFO("frame_id = %s \n",argv[i+1]); 

          }

        i++;
        } else {
          ROS_INFO("Debe ingresar el valor del argumento \n");
          exit(1);
        }
    }

    ros::NodeHandle n;

    map_pub = n.advertise<nav_msgs::OccupancyGrid>("map",10); //PUBLICAR
    
    sub = n.subscribe("scan", 1000, laserScancallback); //SUBSCRIBIRSE


    ros::spin();

    return 0;
  }
//---------------------------------------------------------------------------------------------------------------------

 