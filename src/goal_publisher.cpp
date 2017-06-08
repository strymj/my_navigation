// #include /*{{{*/
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <actionlib/client/simple_action_client.h>
#include <sylib/Point.h>
#include <sylib/MoveBaseGoal.h>
/*}}}*/

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{/*{{{*/
	ros::init(argc, argv, "goal_publisher");
	ros::NodeHandle node_("~");
	ros::Rate looprate(10);

	string csv_path_;
	double reach_tolerance_;
	bool smooth_path_;
	node_.param("csv_path", csv_path_, string("../goal.csv"));
	node_.param("smooth_path", smooth_path_, true);
	node_.param("reach_tolerance", reach_tolerance_, 0.5);

	sy::MoveBaseGoal mbg(csv_path_);

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	tf::TransformListener listener;
	tf::StampedTransform map2base;

	while (ros::ok())
	{/*{{{*/

		move_base_msgs::MoveBaseGoal goal;
		mbg.makeNextGoal(goal);
		ac.sendGoal(goal);
		ROS_INFO("Sending goal : %s", mbg.getGoalID().c_str());

		if (smooth_path_)
		{/*{{{*/
			while (ros::ok())
			{
				try
				{
					listener.lookupTransform("/map", "/base_link", ros::Time(0), map2base);
					double dist = mbg.getGoalDistance(map2base);
					if (dist < reach_tolerance_)
						break;
				}
				catch (tf::TransformException ex)
				{
					ROS_ERROR("%s",ex.what());
				}
				looprate.sleep();
			}
		}/*}}}*/
		else
			ac.waitForResult();

	}/*}}}*/
	return 0;
}/*}}}*/
