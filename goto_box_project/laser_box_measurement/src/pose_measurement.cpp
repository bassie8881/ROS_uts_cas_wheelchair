#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <math.h>

sensor_msgs::LaserScan scanmsg;
ros::Publisher pub; nav_msgs::Odometry odom;

void laserCallback(const sensor_msgs::LaserScan& msg){
    scanmsg = msg;
}

void filter_scan(std::vector<float>& scandata, sensor_msgs::LaserScan& scanin){
    int index = (scanin.ranges.size()-1);

    if(scanin.ranges.size()>=3){
        scandata[0] = ((scanin.ranges[0]+scanin.ranges[1]+scanin.ranges[2])/3);
        scandata[index] = ((scanin.ranges[index-2]+scanin.ranges[index-1]+scanin.ranges[index])/3);

        for(std::size_t i=1; i<scanin.ranges.size()-2; i++){
            scandata[i] = (scanin.ranges[i-1]+scanin.ranges[i]+scanin.ranges[i+1])/3;
        }
    }
    else{
        if(index==1) scandata[0] = scanin.ranges[0];
        if(index==2){ scandata[0] = scanin.ranges[0]; scandata[1] = scanin.ranges[1];}
    }
}

void find_pole(double& x, double& y,
  const std::vector<float>& px, const std::vector<float>& py){
    int Q = px.size();
    if(Q>=3){
        float totx, toty = 0;
        for(std::size_t i=0; i<(Q-1); i++){
            totx = totx + px[i];
            toty = toty + py[i];
        }
        x = totx/Q;
        y = toty/Q;
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
    }
    if(Q==2){
        x = px[0]+px[1];
        y = py[0]+py[1];
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
    }
    if(Q==1){
        x = px[0];
        y = py[0];
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
    }

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
    odom.pose.pose.orientation.z = 1;
    double search_radius_; nh.getParam("sradius", search_radius_);

    ros::Subscriber sub = nh.subscribe("/head_scan",1,laserCallback);
    pub = nh.advertise<nav_msgs::Odometry>("/odom2", 1);
    tf::TransformBroadcaster tfbc_; geometry_msgs::TransformStamped stf_;
    odom.header.frame_id = stf_.header.frame_id = "/static_laser";
    odom.child_frame_id = stf_.child_frame_id = "/odom_wc";
    odom.pose.pose.position.z = stf_.transform.translation.z = 0;

    while(ros::ok()){
        ros::spinOnce();

        odom.header.stamp = scanmsg.header.stamp;

        double sumx_ = 0, sumy_ = 0; int c_ = 0;
        if(!scanmsg.ranges.empty()){
            float h = scanmsg.angle_min;
            for(size_t i=0; i<scanmsg.ranges.size(); i++){
                float x_distance = scanmsg.ranges[i] * cos(h);
                float y_distance = scanmsg.ranges[i] * sin(h);
                h += scanmsg.angle_increment;

                float dx_ = x_distance - odom.pose.pose.position.x,
                  dy_ = y_distance - odom.pose.pose.position.y;
                if(sqrt((dx_*dx_)+(dy_*dy_)) <= search_radius_)
                {
                    sumx_ += x_distance; sumy_ += y_distance; c_++;
                }
            }
        }
        if(c_ > 0)
        {
            odom.pose.pose.position.x = sumx_ / c_;
            odom.pose.pose.position.y = sumy_ / c_;
            ROS_INFO("%f %f", odom.pose.pose.position.x,odom.pose.pose.position.y);

            copy_odom_msg_to_stamped_transform(stf_, odom);
            pub.publish(odom); tfbc_.sendTransform(stf_);
        }
        else ROS_WARN("Pole not found");

        ros::Duration(0.05).sleep();
    }
    return 0;
}
