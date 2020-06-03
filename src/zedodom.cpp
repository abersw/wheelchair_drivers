#include "ros/ros.h"
//#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"


void pointCallback(const nav_msgs::Odometry  &msg)
{
          nav_msgs::Odometry pub;
          ros::NodeHandle r;
          ros::Publisher base_link_pub = r.advertise<nav_msgs::Odometry>("odom", 50);
          base_link_pub.publish(pub);
}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "odom_converter");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("zed/zed_node/odom", 50, pointCallback);
  ros::spin();

  return 0;
} 