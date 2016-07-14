#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "tf_to_odom");
    ros::NodeHandle nh;

    ros::Publisher amcl_pose = nh.advertise<nav_msgs::Odometry>("amcl_ground_truth",10);
    ros::Publisher ground_truth = nh.advertise<nav_msgs::Odometry>("laser_ground_truth",10);

    tf::TransformListener listener;
    tf::TransformListener listener2;
    tf::TransformListener listener3;
    ros::Rate rate(10.0);

    while(nh.ok()){
        tf::StampedTransform transform;
        tf::StampedTransform transform_s;
        tf::StampedTransform transform_d;
        tf::Transform transform_r;

        try{
            listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        try{
            listener2.lookupTransform("/map", "/laser_head", ros::Time(0), transform_s);
        }
        catch(tf::TransformException &ex2){
            ROS_ERROR("%s",ex2.what());
            ros::Duration(1.0).sleep();
        }

        try{
            listener3.lookupTransform("/odom_wc", "/static_link_laser_head", ros::Time(0), transform_d);
        }
        catch(tf::TransformException &ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        transform_r = transform_s*transform_d;

        nav_msgs::Odometry odom_truth;
        odom_truth.header.stamp = ros::Time::now();
        odom_truth.header.frame_id = "ground_truth";
        odom_truth.pose.pose.position.x = transform_r.getOrigin().x();
        odom_truth.pose.pose.position.y = transform_r.getOrigin().y();
        odom_truth.pose.pose.position.z = transform_r.getOrigin().z();
        odom_truth.pose.pose.orientation.x = transform_r.getRotation().x();
        odom_truth.pose.pose.orientation.y = transform_r.getRotation().y();
        odom_truth.pose.pose.orientation.z = transform_r.getRotation().z();
        odom_truth.pose.pose.orientation.w = transform_r.getRotation().w();
        ground_truth.publish(odom_truth);

        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "amcl_pose";
        odom_msg.pose.pose.position.x = transform.getOrigin().x();
        odom_msg.pose.pose.position.y = transform.getOrigin().y();
        odom_msg.pose.pose.position.z = transform.getOrigin().z();
        odom_msg.pose.pose.orientation.x = transform.getRotation().x();
        odom_msg.pose.pose.orientation.y = transform.getRotation().y();
        odom_msg.pose.pose.orientation.z = transform.getRotation().z();
        odom_msg.pose.pose.orientation.w = transform.getRotation().w();
        amcl_pose.publish(odom_msg);

        rate.sleep();
    }
    return 0;
}
