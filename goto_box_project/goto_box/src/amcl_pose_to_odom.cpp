#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher pub;
nav_msgs::Odometry odom;

void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  odom.header = msg->header;
  odom.pose.pose = msg->pose;
  pub.publish(odom);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "converter");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/pose_stamped", 1, poseCallback);
  pub = nh.advertise<nav_msgs::Odometry>("/amcl_odom", 1);

  ros::spin();
  return 0;
}

