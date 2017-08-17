#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <iostream>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Bool>("ypresser", 1000);
  while (ros::ok())
  {
    bool inputBool;
    std::cout << "True/False?";
    std::cin >> inputBool;
    std_msgs::Bool msg;
    msg.data = inputBool;
    ROS_INFO("You are saying: %d\n", msg.data);
    pub.publish(msg);
    ros::spinOnce();
  }
  
  return 0;
}
