#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher pub,pub2;
geometry_msgs::PoseStamped odom;
sensor_msgs::LaserScan scanout;

void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  odom.header = msg->header;
  odom.pose = msg->pose;
  pub.publish(odom);
}

void oseCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
  if(scanout.ranges.size() != msg->ranges.size()) scanout.ranges.resize(msg->ranges.size());
  for(size_t i=0; i<scanout.ranges.size(); i++)
  {
	scanout.ranges[i] = msg->ranges[i];
  }

  scanout.time_increment = msg->time_increment;
  scanout.angle_min = msg->angle_min; scanout.angle_max = msg->angle_max;
  scanout.angle_increment = msg->angle_increment; scanout.time_increment = msg->time_increment;
  scanout.scan_time = msg->scan_time;
  scanout.range_min = msg->range_min; scanout.range_max = msg->range_max; 

  scanout.header.seq = msg->header.seq;
  scanout.header.frame_id = "laser";
  scanout.header.stamp = ros::Time::now();
  pub2.publish(scanout);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "converter");
  ros::NodeHandle nh;  
  ros::Subscriber sub = nh.subscribe("/slam_out_pose2", 1, poseCallback);
  ros::Subscriber sub2 = nh.subscribe("/scan", 1, oseCallback);
  pub = nh.advertise<geometry_msgs::PoseStamped>("/odom_hector", 1);
  pub2 = nh.advertise<sensor_msgs::LaserScan>("/scan2", 1);

  ros::spin();
  return 0;
}
