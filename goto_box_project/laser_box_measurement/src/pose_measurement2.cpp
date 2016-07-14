#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <math.h>

sensor_msgs::LaserScan scanmsg;
ros::Publisher pub;
nav_msgs::Odometry odom;

double rangefinder = 0.5;
double search_angle_min, search_angle_max, local_min_angle_y, local_max_angle_y, local_min_angle_x, local_max_angle_x = 0;

void laserCallback(const sensor_msgs::LaserScan& msg){
    scanmsg = msg;
}

void copy_odom_msg_to_stamped_transform(geometry_msgs::TransformStamped& stf, const nav_msgs::Odometry& odom){
    stf.header.stamp = odom.header.stamp;
    stf.transform.translation.x = odom.pose.pose.position.x;
    stf.transform.translation.y = odom.pose.pose.position.y;
    stf.transform.rotation = odom.pose.pose.orientation;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_tracking");
    ros::NodeHandle nh("~");

    nh.getParam("start_x", odom.pose.pose.position.x);
    nh.getParam("start_y", odom.pose.pose.position.y);

    local_min_angle_y = atan((odom.pose.pose.position.y+rangefinder)/odom.pose.pose.position.x);
    ROS_INFO("Local min angle y = %f", local_min_angle_y);
    local_max_angle_y = atan((odom.pose.pose.position.y-rangefinder)/odom.pose.pose.position.x);
    ROS_INFO("Local max angle y = %f", local_max_angle_y);
    local_min_angle_x = atan(odom.pose.pose.position.y/(odom.pose.pose.position.x+rangefinder));
    ROS_INFO("Local min angle x = %f", local_min_angle_x);
    local_max_angle_x = atan(odom.pose.pose.position.y/(odom.pose.pose.position.x-rangefinder));
    ROS_INFO("Local max angle x = %f", local_max_angle_x);

    if(local_min_angle_y<=local_min_angle_x) search_angle_min = local_min_angle_y; else search_angle_min = local_min_angle_x;
    ROS_INFO("Search angle min = %f", search_angle_min); ROS_INFO("Search angle max = %f", search_angle_max);

    ros::Subscriber sub = nh.subscribe("/head_scan",1,laserCallback);
    pub = nh.advertise<nav_msgs::Odometry>("/odom2", 1);
    tf::TransformBroadcaster tfbc_; geometry_msgs::TransformStamped stf_;
    odom.header.frame_id = stf_.header.frame_id = "/odom_wc";
    odom.child_frame_id = stf_.child_frame_id = "/static_link_laser_head";
    odom.pose.pose.position.z = stf_.transform.translation.z = 0;

    while(ros::ok()){
        ros::spinOnce();

        odom.header.stamp = ros::Time::now();
        bool set1, set2 = true;

        std::vector<float> x_distance(scanmsg.ranges.size()), y_distance(scanmsg.ranges.size());
        float h = scanmsg.angle_min;
        for(size_t i=0; i<scanmsg.ranges.size(); i++){
            x_distance[i] = scanmsg.ranges[i] * cos(h);
            y_distance[i] = scanmsg.ranges[i] * sin(h);
            h += scanmsg.angle_increment;
            if((search_angle_min<=h)&&(set1==true))search_angle_min = h; set1 = false;
            if((search_angle_max<=h)&&(set2==true))search_angle_max = h; set2 = false;
        }
        ROS_INFO("Search angle min update = %f", search_angle_min); ROS_INFO("Search angle max update = %f", search_angle_max);
        /*ROS_INFO("start angle = %f", scanmsg.ranges[0]);*/

        for(size_t i=(scanmsg.angle_increment/search_angle_min); i<(scanmsg.angle_increment/search_angle_max);i++){
            if((x_distance[i] <= (odom.pose.pose.position.x+rangefinder)) && (x_distance[i] >= (odom.pose.pose.position.x-rangefinder))){
                if((y_distance[i] <= (odom.pose.pose.position.y+rangefinder)) && (y_distance[i] >= (odom.pose.pose.position.y-rangefinder))){
                    if(scanmsg.ranges[i]<=scanmsg.ranges[i-1]){
                        odom.pose.pose.position.x = x_distance[i];
                        odom.pose.pose.position.y = y_distance[i];
                    }
                }
            }
        }
        copy_odom_msg_to_stamped_transform(stf_, odom);
        pub.publish(odom); tfbc_.sendTransform(stf_);

        local_min_angle_y = atan((odom.pose.pose.position.y+rangefinder)/odom.pose.pose.position.x);
        local_max_angle_y = atan((odom.pose.pose.position.y-rangefinder)/odom.pose.pose.position.x);
        local_min_angle_x = atan(odom.pose.pose.position.y/(odom.pose.pose.position.x+rangefinder));
        local_max_angle_x = atan(odom.pose.pose.position.y/(odom.pose.pose.position.x-rangefinder));

        if(local_min_angle_y<=local_min_angle_x) search_angle_min = local_min_angle_y; else search_angle_min = local_min_angle_x;
    }
    return 0;
}

