#include "ros/ros.h"
#include "std_msgs/Bool.h"

void ypresserCallback (const std_msgs::Bool msg)
{
  ROS_INFO("I am hearing: %d\n", msg.data);
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("ypresser",1000, ypresserCallback);
  ros::spin();
  return 0;
}
