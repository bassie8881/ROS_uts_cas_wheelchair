#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

std::string robot_pose;

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
    static tf::TransformBroadcaster transformer2;
    geometry_msgs::TransformStamped odom_trans2;
    odom_trans2.header.frame_id = "base_footprint";
    odom_trans2.child_frame_id = "base_link";
    odom_trans2.transform.translation.x = 0;
    odom_trans2.transform.translation.y = 0;
    odom_trans2.transform.translation.z = 0;
    odom_trans2.transform.rotation.x = 0;
    odom_trans2.transform.rotation.y = 0;
    odom_trans2.transform.rotation.z = 0;
    odom_trans2.transform.rotation.w = 1;
    transformer2.sendTransform(odom_trans2);

    static tf::TransformBroadcaster transformer3;
    geometry_msgs::TransformStamped odom_trans3;
    odom_trans3.header.frame_id = "base_link";
    odom_trans3.child_frame_id = "base_laser_link";
    odom_trans3.transform.translation.x = -0.17;
    odom_trans3.transform.translation.y = -0.2;
    odom_trans3.transform.translation.z = 1.0;
    odom_trans3.transform.rotation.x = 0;
    odom_trans3.transform.rotation.y = 0;
    odom_trans3.transform.rotation.z = 0;
    odom_trans3.transform.rotation.w = 1.0;
    transformer3.sendTransform(odom_trans3);

    static tf::TransformBroadcaster transformer;
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    odom_trans.transform.translation.x = msg->pose.pose.position.x;
    odom_trans.transform.translation.y = msg->pose.pose.position.y;
    odom_trans.transform.translation.z = msg->pose.pose.position.z;
    odom_trans.transform.rotation.x = msg->pose.pose.orientation.x;
    odom_trans.transform.rotation.y = msg->pose.pose.orientation.y;
    odom_trans.transform.rotation.z = msg->pose.pose.orientation.z;
    odom_trans.transform.rotation.w = msg->pose.pose.orientation.w;
    transformer.sendTransform(odom_trans);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "odom_to_tf");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/odom",1,poseCallback);
    ros::spin();
    return 0;}
