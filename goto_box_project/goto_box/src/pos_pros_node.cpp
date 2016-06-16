#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Int16MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <rosgraph_msgs/Log.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>

bool button0, button1, last_reading, goal_reached, set_second = false;
bool buttons_in[2];
bool set_time, set_time2, set_error_level = true;
int status_planner, status_error, last_reading_status_planner, quantity = 0;
int ERROR = 15;
const unsigned int data_sz=2;
const unsigned int data_sz2=1;
const unsigned int data_sz3=1;
double duration, duration2;
double debounce_delay = 20.0;
double debounce_delay2 = 2.0;
ros::Time Initialtime;
ros::Time Initialtime2;

ros::Publisher pub_LED;
std_msgs::Int16MultiArray led_msg;
std_msgs::MultiArrayDimension msg_dim;

ros::Publisher pub_buzzer;
std_msgs::Int16MultiArray buzzer_msg;
std_msgs::MultiArrayDimension msg2_dim;

ros::Publisher pub_LEDRED;
std_msgs::Int16MultiArray redled_msg;
std_msgs::MultiArrayDimension msg3_dim;

ros::Publisher pos, cancel_pub_;
geometry_msgs::PoseStamped odom;

void errorCallback(const rosgraph_msgs::Log::ConstPtr& msg){
    status_error = msg->level;
}

void planner_statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
    if(!msg->status_list.empty()){
        status_planner = (msg->status_list.back().status);
    }
}

void buttonCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    buttons_in[0] = (msg->data[0] == 1);
    buttons_in[1] = (msg->data[1] == 1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "positionProviderNode");
    ros::NodeHandle nh;

    msg_dim.label = "greenled";
    msg_dim.size = data_sz;
    msg_dim.stride = 2;
    led_msg.layout.dim.clear();
    led_msg.layout.dim.push_back(msg_dim);
    led_msg.data.clear();
    led_msg.data.push_back(1);
    led_msg.data.push_back(0);

    msg2_dim.label = "buzzer";
    msg2_dim.size = data_sz2;
    msg2_dim.stride = 1;
    buzzer_msg.layout.dim.clear();
    buzzer_msg.layout.dim.push_back(msg2_dim);
    buzzer_msg.data.clear();
    buzzer_msg.data.push_back(0);

    msg3_dim.label = "redled";
    msg3_dim.size = data_sz3;
    msg3_dim.stride = 1;
    redled_msg.layout.dim.clear();
    redled_msg.layout.dim.push_back(msg3_dim);
    redled_msg.data.clear();
    redled_msg.data.push_back(0);

    ros::Subscriber sub = nh.subscribe("/pushed",1,buttonCallback);
    ros::Subscriber sub1 = nh.subscribe("move_base/status",1,planner_statusCallback);
    ros::Subscriber sub2 = nh.subscribe("/rosout_agg",1,errorCallback);
    pub_LED = nh.advertise<std_msgs::Int16MultiArray>("/LED",1);
    pub_LEDRED = nh.advertise<std_msgs::Int16MultiArray>("/LED2",1);
    pub_buzzer = nh.advertise<std_msgs::Int16MultiArray>("/Buzzer",1);
    pos = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);
    cancel_pub_ = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel",1);

    if(set_time == true){
        set_time == false;
        Initialtime = ros::Time::now();
    }

    buttons_in[0] = buttons_in[1] = 0;
    while(ros::ok())
    {
        duration = (ros::Time::now()-Initialtime).toSec();
        if(set_error_level==true && (duration >= debounce_delay)){
            set_error_level = false;
            ERROR = 8;
        }

        if(status_error == ERROR){
        if(set_time2 == true){
        set_time2 == false;
        set_second == true;
        Initialtime2 = ros::Time::now();
        }

        if(set_second ==true){
        duration2 = (ros::Time::now()-Initialtime2).toSec();
        Initialtime2 = ros::Time::now();
        }

        if(duration2 >= debounce_delay2){
        quantity = quantity + 1;
        }

        if(quantity >= 30){
        redled_msg.data[0] = 1;
            pub_LEDRED.publish(redled_msg);
            led_msg.data[0]=0;
                led_msg.data[1]=0;
                pub_LED.publish(led_msg);
            ROS_INFO("AN ERROR HAS BEEN OCCURED");
                return 0;
        }
        }

        last_reading_status_planner = status_planner;

        ros::spinOnce();

        if(buttons_in[0] == buttons_in[1]) last_reading = false;
        if((buttons_in[0] != buttons_in[1])&&(last_reading==false)){
            if((buttons_in[0]==1)&&(buttons_in[1]==0)&&(button1==0)){
                button0 = 1;
                odom.header.stamp = ros::Time::now();
                odom.header.frame_id = "/map";
                odom.pose.position.x = 5.9909;
                odom.pose.position.y = 6.6703;
                odom.pose.position.z = 0.0;
                odom.pose.orientation.x = 0.0;
                odom.pose.orientation.y = 0.0;
                odom.pose.orientation.z = 0.43275224201;
                odom.pose.orientation.w = 0.90152586226;
                pos.publish(odom);
                ROS_INFO("SENT GOAL 1");
            }
            if((buttons_in[0]==0)&&(buttons_in[1]==1)&&(button0==0)){
                button1 = 1;
                odom.header.stamp = ros::Time::now();
                odom.header.frame_id = "/map";
                odom.pose.position.x = -7.98388957977;
                odom.pose.position.y = 31.673789978;
                odom.pose.position.z = 0.0;
                odom.pose.orientation.x = 0.0;
                odom.pose.orientation.y = 0.0;
                odom.pose.orientation.z = 0.546743102512;
                odom.pose.orientation.w = 0.837300411952;
                pos.publish(odom);
                ROS_INFO("SENT GOAL 2");
            }
            if((buttons_in[0]==1)&&(buttons_in[1]==1)&&(button0==0)&&(button1==0)) button0 = button1 = 0;
            if((buttons_in[0]==1)&&(button1==1)){
                button1 = 0;
                actionlib_msgs::GoalID empty_goal_;
                cancel_pub_.publish(empty_goal_);
                led_msg.data[0]=1;
                led_msg.data[1]=0;
                buzzer_msg.data[0]=1;
                pub_LED.publish(led_msg);
                pub_buzzer.publish(buzzer_msg);
                ROS_INFO("CANCEL COMMAND SENT");
            }
            if((buttons_in[1]==1)&&(button0==1)){
                button0 = 0;
                actionlib_msgs::GoalID empty_goal_;
                cancel_pub_.publish(empty_goal_);
                led_msg.data[0]=1;
                led_msg.data[1]=0;
                buzzer_msg.data[0]=1;
                pub_LED.publish(led_msg);
                pub_buzzer.publish(buzzer_msg);
                ROS_INFO("CANCEL COMMAND SENT");
            }
            last_reading = true;
        }
        if((status_planner==1)&&(last_reading_status_planner != 1)){ // A
            led_msg.data[0]=1;
            led_msg.data[1]=1;
            buzzer_msg.data[0]=0;
            pub_LED.publish(led_msg);
            pub_buzzer.publish(buzzer_msg);
            ROS_INFO("ROS FLASH GREEN COMMAND SENT");
        }
        if((status_planner==3)&&(last_reading_status_planner != 3)){ // B
            button0, button1 = 0;
            led_msg.data[0]=1;
            led_msg.data[1]=0;
            buzzer_msg.data[0]=1;
            pub_LED.publish(led_msg);
            pub_buzzer.publish(buzzer_msg);
            ROS_INFO("ROS CONSTANT GREEN COMMAND SENT");
        }
        ros::Duration(0.01).sleep();
    }
    return 0;
}
