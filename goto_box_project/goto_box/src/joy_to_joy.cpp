#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Joy.h>

ros::Publisher joy;
sensor_msgs::Joy joystick;

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
    joystick.header.stamp = ros::Time::now();
    joystick.header.frame_id = "''";
    joystick.axes.clear();
    joystick.axes.push_back(msg->axes[0]);
    joystick.axes.push_back(msg->axes[1]);
    joystick.axes.push_back(0);
    joystick.axes.push_back(0);
    joystick.axes.push_back(0);
    joystick.buttons.clear();
    joystick.buttons.push_back(0);
    joystick.buttons.push_back(0);
    joystick.buttons.push_back(0);
    joystick.buttons.push_back(0);
    joystick.axes[0] = msg->axes[0];
    joystick.axes[1] = msg->axes[1];
    joy.publish(joystick);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "joy_talker");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/wc_joy_", 1, joyCallback);
  joy = nh.advertise<sensor_msgs::Joy>("/joy", 1);
  ros::spin();
  return 0;
}
