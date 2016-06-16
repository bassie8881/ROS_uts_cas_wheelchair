#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>

#define STEP 0.1
#define LPAUSE 4.0
#define SPAUSE 2.0
#define OAVG 100

double v_,w_; bool newo_;

void odom_cb(const nav_msgs::Odometry& msg)
{
  v_ = msg.twist.twist.linear.x;
  w_ = msg.twist.twist.angular.z;
  newo_ = 1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jt_profile_node");
  ros::NodeHandle nh("~");
  
  ros::Publisher pub_ = nh.advertise<sensor_msgs::Joy>("/wc_joy", 1);
  sensor_msgs::Joy jmsg_; jmsg_.axes.resize(2);
  ros::Subscriber sub_ = nh.subscribe("/odom", 1, &odom_cb);
  
  FILE* fp_; fp_ = fopen("jt_profile.csv", "w");
  
  ROS_INFO("Pausing (10 s). Please reset hand controller on mode II");
  ros::Duration(10).sleep(); ROS_INFO("Starting ...");

  jmsg_.axes[0] = -1.0;
  while(jmsg_.axes[0] < 1.0+STEP/2 && ros::ok())
  {
    jmsg_.axes[1] = -1.0;
    while(jmsg_.axes[1] < 1.0+STEP/2 && ros::ok())
    {
      pub_.publish(jmsg_);
      if(jmsg_.axes[1] == -1.0) ros::Duration(LPAUSE).sleep();
      else ros::Duration(SPAUSE).sleep();
      int count_ = 0; double vagg_ = 0, wagg_ = 0;
      while(count_<OAVG && ros::ok())
      {
        newo_ = 0; ros::spinOnce();
        if(newo_) { vagg_ += v_; wagg_ += w_; count_++; fprintf(stderr, "."); }
      }
      vagg_ /= OAVG; wagg_ /= OAVG; fprintf(stderr, "\n");
      fprintf(fp_, "%f,%f,%f,%f\n", jmsg_.axes[0],jmsg_.axes[1], vagg_,wagg_);
      ROS_INFO("%f,%f -> %f,%f", jmsg_.axes[0],jmsg_.axes[1], vagg_,wagg_);
      jmsg_.axes[1] += STEP;
    }
    jmsg_.axes[0] += STEP;
  }
  
  fclose(fp_);

  jmsg_.axes[0] = jmsg_.axes[1] = 0; pub_.publish(jmsg_); ros::Duration(1).sleep();

  return 0;
}
