#include <ros/ros.h>
//#include <ros/time.h>
#include <std_srvs/Empty.h>

ros::Time InitialTime;
ros::Time ComparingTimeStamp;

int main(int argc, char **argv){
    ros::init(argc, argv, "loc");
    ros::NodeHandle nh;
    ros::Duration(3).sleep();
    ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/global_localization");
    std_srvs::Empty empty_srv_; client.call(empty_srv_);
    ROS_INFO("LOCALISATION SERVICE CALLED. EXITING");

    return 0;
}
