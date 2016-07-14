#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "tf_to_odom");
    ros::NodeHandle nh;

    ros::Publisher odom = nh.advertise<nav_msgs::Odometry>("odom2",10);

    tf::TransformListener listener;
    ros::Rate rate(10.0);
    while(nh.ok()){
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "odom";
        odom_msg.pose.pose.position.x = transform.getOrigin().x();
        odom_msg.pose.pose.position.y = transform.getOrigin().y();
        odom_msg.pose.pose.position.z = transform.getOrigin().z();
        odom_msg.pose.pose.orientation.x = transform.getRotation().x();
        odom_msg.pose.pose.orientation.y = transform.getRotation().y();
        odom_msg.pose.pose.orientation.z = transform.getRotation().z();
        odom_msg.pose.pose.orientation.w = transform.getRotation().w();
        odom.publish(odom_msg);

        rate.sleep();
    }
    return 0;
}
