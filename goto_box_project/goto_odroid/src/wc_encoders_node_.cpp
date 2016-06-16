#include <ros/ros.h>
// serial
#include "goto_odroid/defs_.h"
#include "goto_odroid/WheelEncoder_.h"
// ROS
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

double ang_fix(double in)
{
  double out_ = in;
  while(out_ >= M_PI) out_ -= 2*M_PI;
  while(out_ <= -M_PI) out_ += 2*M_PI;
  return out_;
}

void copy_odom_msg_to_stamped_transform(
  geometry_msgs::TransformStamped& stf, const nav_msgs::Odometry& odom)
{
  stf.header.stamp = odom.header.stamp;
  stf.transform.translation.x = odom.pose.pose.position.x;
  stf.transform.translation.y = odom.pose.pose.position.y;
  stf.transform.rotation = odom.pose.pose.orientation;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wc_enc_node");
  ros::NodeHandle nh("~");
  
  // serial
  char lenc_port_[MAX_FILENAME_SIZE], renc_port_[MAX_FILENAME_SIZE];
  strncpy(lenc_port_, Left_Encoder_Port, sizeof(lenc_port_));
  strncpy(renc_port_, Right_Encoder_Port, sizeof(renc_port_));
  WheelEncoder lenc_(lenc_port_, Encoder_Baud_Rate),
    renc_(renc_port_, Encoder_Baud_Rate);
  fflush(stdout);
  
  // ROS
  ros::Publisher pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 1);
  nav_msgs::Odometry odom_; double yaw_ = 0;
  tf::TransformBroadcaster tfbc_; geometry_msgs::TransformStamped stf_;
  odom_.header.frame_id = stf_.header.frame_id = "/odom";
  odom_.child_frame_id = stf_.child_frame_id = "/base_link";
  odom_.pose.pose.position.z = stf_.transform.translation.z = 0;
  
  int lticksl_ = lenc_.GetTicks(), rticksl_ = renc_.GetTicks();
  ros::Time tl_ = ros::Time::now();
  while(ros::ok())
  {
    // wheel/time delta
    int lticksc_ = lenc_.GetTicks(), rticksc_ = renc_.GetTicks();
    odom_.header.stamp = ros::Time::now(); double dt_ = (odom_.header.stamp-tl_).toSec();
    int dl_ = lticksc_ - lticksl_, dr_ = rticksc_ - rticksl_;
    if(dl_ > 1e6)
    {
      if(lticksc_ < 1e6) dl_ = lticksc_ + (MAX_ENCODER_VALUE - lticksl_);
      else dl_ = lticksl_ + (MAX_ENCODER_VALUE - lticksc_);
    }
    dl_ *= -1; // left wheel encoder spins CCW
    if(dr_ > 1e6)
    {
      if(rticksc_ < 1e6) dr_ = rticksc_ + (MAX_ENCODER_VALUE - rticksl_);
      else dr_ = rticksl_ + (MAX_ENCODER_VALUE - rticksc_);
    }
    lticksl_ = lticksc_; rticksl_ = rticksc_; tl_ = odom_.header.stamp;
    
    // wheel/platform movement
    double ml_ = dl_ * M_PER_TICK, mr_ = dr_ * M_PER_TICK;
    double dpos_ = (ml_+mr_) / 2, dyaw_ = (mr_-ml_) / AXLE_LENGTH;
    // pose update
    if(dyaw_ == 0)
    {
      odom_.pose.pose.position.x += dpos_ * cos(yaw_);
      odom_.pose.pose.position.y += dpos_ * sin(yaw_);
    }
    else
    {
      double radius_ = dpos_ / dyaw_;
      odom_.pose.pose.position.x += radius_ * (sin(dyaw_+yaw_)-sin(yaw_));
      odom_.pose.pose.position.y -= radius_ * (cos(dyaw_+yaw_)-cos(yaw_));
      yaw_ = ang_fix(yaw_ + dyaw_);
      odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_);
    }
    // twist update
    odom_.twist.twist.linear.x = dpos_ / dt_;
    odom_.twist.twist.angular.z = dyaw_ / dt_;
    // publish odom msg, broadcast odom TF
    copy_odom_msg_to_stamped_transform(stf_, odom_);
    pub_.publish(odom_); tfbc_.sendTransform(stf_);
  }

  return 0;
}
