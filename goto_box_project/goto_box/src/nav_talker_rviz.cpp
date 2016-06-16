#include <ros/ros.h>
#include <boost/thread.hpp>
#include <nav_msgs/Path.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#define PRINT 1
#define MIN_SPACING 0.25
#define hypotenuse(dx,dy) sqrt(dx*dx + dy*dy)

nav_msgs::Path leader_path; int path_write_step = -1;
double gap_x=0,gap_y=0;

void mouse_cb(const geometry_msgs::PoseStamped& msg)
{
  if(path_write_step>-1)
  {
	gap_x = msg.pose.position.x - leader_path.poses[path_write_step].pose.position.x;
	gap_y = msg.pose.position.y - leader_path.poses[path_write_step].pose.position.y;
  }

  if(hypotenuse(gap_x,gap_y) > MIN_SPACING || path_write_step==-1)
  {
	leader_path.poses.push_back(msg); path_write_step++;
	ROS_INFO("Recieved goal: %f,%f", msg.pose.position.x,msg.pose.position.y);
  }
}


int action_lib(void)
{
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
  while(ros::ok() && !ac.waitForServer(ros::Duration(5.0))) ROS_INFO("Waiting for actionlib server....");
  move_base_msgs::MoveBaseGoal goal; size_t path_r=0;
  
  while(ros::ok())
  {
	if(path_r < leader_path.poses.size())
	{
		goal.target_pose = leader_path.poses[path_r];
		goal.target_pose.header.stamp = ros::Time::now();

		ac.sendGoal(goal); ac.waitForResult();
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			path_r++;
			if(PRINT) ROS_INFO("Goal successfully reached! Moving to next goal...");
		}
	}
  }
  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nav_stack_talker");
  ros::NodeHandle nh;

  boost::thread path(boost::bind(action_lib));
  ros::Subscriber sub = nh.subscribe("move_base_simple/goal", 1, mouse_cb);
  ros::spin();

  path.join();
  return 0;
}
