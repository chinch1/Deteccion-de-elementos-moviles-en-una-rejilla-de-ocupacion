#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int64.h>
          
using namespace ros;

void chatterCallback(const std_msgs::Int64::ConstPtr& msg)
{
  ROS_INFO("I heard u talker: %d", msg->data);
}

int main(int argc, char **argv)//--------------------------------------------
{

  init(argc, argv, "listener");

  NodeHandle n;

  Publisher chatter_pub = n.advertise<std_msgs::Int64>("chatterBack", 1000);
  Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  Rate loop_rate(1);

  int count = 1;

  while(ok())
  {
  std_msgs::Int64 msg;
  msg.data = count;
  ROS_INFO("talker, te enviare: %d", count);
  chatter_pub.publish(msg);

  spinOnce();
  loop_rate.sleep();
  count = count + 2;
  }
    
  return 0;
}  