// #include /*{{{*/
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <mylib/Pt.h>
/*}}}*/

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;

static double GOAL_OFFSET = -0.45;

int petnum = 0;
bool petnum_subscribed = false;
void petPointCallback(const sensor_msgs::PointCloud& msg)
{/*{{{*/
	petnum = msg.points.size();
	petnum_subscribed = true;
}/*}}}*/

double lefttime = 0.0;
bool lefttime_subscribed = false;
void ojmStatusCallback(const std_msgs::Float32& msg)
{/*{{{*/
	lefttime = msg.data;
	lefttime_subscribed = true;
}/*}}}*/

struct Goal
{/*{{{*/
	string frame_id;
	string goal_id;
	geometry_msgs::Pose2D pose2d;

	Goal(){}

	Goal(string tag, string frame, double x, double y, double theta)
	{
		goal_id = tag;
		frame_id = frame;
		pose2d.x = x;
		pose2d.y = y;
		pose2d.theta = theta;
	}
};/*}}}*/

move_base_msgs::MoveBaseGoal GoalMaker(Goal goal)
{/*{{{*/
	move_base_msgs::MoveBaseGoal move_base_goal;
	move_base_goal.target_pose.header.frame_id = goal.frame_id;
	move_base_goal.target_pose.header.stamp = ros::Time::now();
	move_base_goal.target_pose.pose.position.x = goal.pose2d.x;
	move_base_goal.target_pose.pose.position.y = goal.pose2d.y;
	move_base_goal.target_pose.pose.position.z = 0.0;
	geometry_msgs::Quaternion quaternion;
	quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, goal.pose2d.theta);
	move_base_goal.target_pose.pose.orientation = quaternion;

	return move_base_goal;
}/*}}}*/

void SetGoal(vector<Goal>& goals, string tag, string frame, double x, double y, double theta)
{/*{{{*/
	Goal goal;
	goal.frame_id = frame;
	goal.goal_id = tag;
	goal.pose2d.x = x;
	goal.pose2d.y = y;
	goal.pose2d.theta = theta;
	goals.push_back(goal);
}/*}}}*/

int main(int argc, char **argv)
{/*{{{*/
	ros::init(argc, argv, "goal_publisher");
	ros::NodeHandle node_("~");
	ros::Rate looprate(30);

	bool SMOOTH_PATH;
	double REACH_TOLERANCE, REACH_TOLERANCE_NARROW = 0.12;
	node_.param("smooth_path", SMOOTH_PATH, true);
	node_.param("reach_tolerance", REACH_TOLERANCE, 0.5);
	node_.param("reach_tolerance_narrow", REACH_TOLERANCE_NARROW, 0.15);

	ros::Subscriber petPointSub = node_.subscribe("/pet_point", 10, petPointCallback);
	ros::Subscriber ojmStatus1Sub = node_.subscribe("/left_time", 10, ojmStatusCallback);
	ros::Publisher goal_id_pub = node_.advertise<std_msgs::String>("/goal_id", 10);

	vector<Goal> goals;
	goals.push_back( Goal("aisle_end", "map", -0.4, -1.8, -M_PI/2) );
	goals.push_back( Goal("explore_start", "map", -0.35, -3.1, -M_PI/2) );
	goals.push_back( Goal("explore_middle1", "map", -0.7, -3.75, -M_PI) );
	goals.push_back( Goal("explore_middle2", "map", -1.3, -3.75, -M_PI) );
	goals.push_back( Goal("explore_end", "map", -1.65, -3.1, M_PI/2) );
	goals.push_back( Goal("pole_front", "map", -1.6, -2.3 -0.4, M_PI/2) );
	goals.push_back( Goal("pole_back", "map", -1.6, -1.5, M_PI/2) );
	goals.push_back( Goal("carboxA_front1", "map", -1.6, -0.4 -0.9, M_PI/2) );
	goals.push_back( Goal("carboxA_front2", "map", -1.6, -0.4 -0.7, M_PI/2) );
	goals.push_back( Goal("carboxA_inside", "map", -1.6, -0.4 -0.24, M_PI/2) );

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	tf::TransformListener listener;
	tf::StampedTransform map2base;

	int GNum = 0;
	double reach_tolerance = REACH_TOLERANCE;
	std_msgs::String msg;

	while (ros::ok() && goals.size()!=0)
	{/*{{{*/

		cout<<"Next Goal Number : "<<GNum<<endl;

		if (goals[GNum].goal_id == "explore_start")
		{/*{{{*/
			while (ros::ok()) {
				cout<<"lefttime = "<<lefttime<<endl;
				if (lefttime_subscribed && 0.6<lefttime && lefttime<1.1)
				{
					lefttime_subscribed = false;
					break;
				}
				ros::spinOnce();
				looprate.sleep();
			}
		}/*}}}*/

		if (goals[GNum].goal_id == "pole_back")
		{/*{{{*/
			while (ros::ok()) {
				// cout<<"lefttime = "<<lefttime<<endl;
				if (lefttime_subscribed && 1.5<lefttime && lefttime<2.0)
				{
					lefttime_subscribed = false;
					break;
				}
				ros::spinOnce();
				looprate.sleep();
			}
		}/*}}}*/

		if (goals[GNum].goal_id == "pole_front")
		{/*{{{*/
			while (ros::ok()) {
				if (petnum_subscribed)
					break;
				ros::spinOnce();
				looprate.sleep();
			}
			if (3<petnum) petnum = 3;
			cout<<"petnum = "<<petnum<<endl;
		}/*}}}*/

		if (goals[GNum].goal_id == "pole_back") goals[GNum].pose2d.theta += M_PI/3 * (petnum%2);
		if (goals[GNum].goal_id == "carboxA_front1") reach_tolerance = REACH_TOLERANCE_NARROW;
		if (goals[GNum].goal_id == "carboxA_front1") goals[GNum].pose2d.x += GOAL_OFFSET * (petnum%2);
		if (goals[GNum].goal_id == "carboxA_front2") goals[GNum].pose2d.x += GOAL_OFFSET * (petnum%2);
		if (goals[GNum].goal_id == "carboxA_inside") goals[GNum].pose2d.x += GOAL_OFFSET * (petnum%2);

		ROS_INFO("Sending goal");
		ac.sendGoal(GoalMaker(goals[GNum]));

		msg.data = goals[GNum].goal_id;
		goal_id_pub.publish(msg);

		if (SMOOTH_PATH && GNum+1 < goals.size() && goals[GNum].goal_id != "pole_front")
		{/*{{{*/
			while (ros::ok())
			{
				try
				{
					listener.lookupTransform("/map", "/base_link", ros::Time(0), map2base);
					double dist = Pt::distance(
							Pt::Pt2d(map2base.getOrigin().getX(), map2base.getOrigin().getY()),
							Pt::Pt2d(goals[GNum].pose2d.x, goals[GNum].pose2d.y) );
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
		{/*{{{*/
			ac.waitForResult();
		}/*}}}*/

		msg.data = goals[GNum].goal_id + "_reached";
		goal_id_pub.publish(msg);

		++GNum;
		if (goals.size() <= GNum)
		{/*{{{*/
			ROS_INFO("Traveled all point!!!");
			break;
		}/*}}}*/

	}/*}}}*/
	return 0;
}/*}}}*/
