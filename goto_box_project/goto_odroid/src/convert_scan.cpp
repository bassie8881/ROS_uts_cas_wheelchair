#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

sensor_msgs::LaserScan head_laser_msg, wc_laser_msg;

void laser_cb1(const sensor_msgs::LaserScan& msg) { wc_laser_msg = msg; }
void laser_cb2(const sensor_msgs::LaserScan& msg) { head_laser_msg = msg; }

int main(int argc, char **argv){
    ros::init(argc, argv, "convert_scan");
    ros::NodeHandle nh("~");

    ros::Subscriber sub2 = nh.subscribe("/scan",1,laser_cb1);
    ros::Subscriber sub3 = nh.subscribe("/base_scan",1,laser_cb2);
    ros::Publisher pub1 = nh.advertise<sensor_msgs::LaserScan>("/head_scan", 1);
    ros::Publisher pub2 = nh.advertise<sensor_msgs::LaserScan>("/wc_scan", 1);

    while(ros::ok()){
        ros::spinOnce();

        head_laser_msg.header.stamp = wc_laser_msg.header.stamp = ros::Time::now();
        head_laser_msg.header.frame_id = "/map";
        pub1.publish(head_laser_msg);
        pub2.publish(wc_laser_msg);

        ros::Duration(0.05).sleep();
    }
    return 0;
}
