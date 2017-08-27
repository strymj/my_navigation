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
	ros::Rate looprate(1);

	string csv_path;
	bool smooth_path;
	double reach_tolerance, send_interval;
	node_.param("csv_path", csv_path, string("../goal.csv"));
	node_.param("smooth_path", smooth_path, true);
	node_.param("reach_tolerance", reach_tolerance, 0.5);
	node_.param("send_interval", send_interval, 3.0);

	sy::MoveBaseGoal mbg(csv_path);

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
		if (!mbg.makeNextGoal(goal))
			break;
		ac.sendGoal(goal);
		ROS_INFO("Sending goal : %s", mbg.getGoalID().c_str());
		ros::Duration(send_interval).sleep();

		if (smooth_path)
		{/*{{{*/
			while (ros::ok())
			{
				try
				{
					listener.lookupTransform("/map", "/base_link", ros::Time(0), map2base);
					double dist = mbg.getGoalDistance(map2base);
					if (dist < reach_tolerance)
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
