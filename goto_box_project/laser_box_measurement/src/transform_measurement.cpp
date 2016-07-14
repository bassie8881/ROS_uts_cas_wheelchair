#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Point.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "tf_measurement_to_map");
    ros::NodeHandle nh;

    tf::TransformListener listener2;
    tf::TransformListener listener3;
    tf::TransformBroadcaster tfbc_;
    tf::StampedTransform stf_;
    stf_.frame_id_ = "/map";
    stf_.child_frame_id_ = "/pole";

    ros::Rate rate(10.0);

    ros::Publisher pub = nh.advertise<geometry_msgs::Point>("/eyoo", 1);
    geometry_msgs::Point pointmsg_;

    while(nh.ok()){
        tf::StampedTransform transform_s;

        try{
            listener2.lookupTransform("/map", "/pole", ros::Time(0), transform_s);
        }
        catch(tf::TransformException &ex2){
            ROS_ERROR("%s",ex2.what());
            ros::Duration(1.0).sleep();
        }

        stf_.setData(transform_s);
        stf_.stamp_ = ros::Time::now(); tfbc_.sendTransform(stf_);
        pointmsg_.x = stf_.getOrigin().x(); pointmsg_.y = stf_.getOrigin().y();
        pub.publish(pointmsg_);

        rate.sleep();
    }
    return 0;
}
