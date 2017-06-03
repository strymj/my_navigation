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
#include <fstream>
#include <sstream>
/*}}}*/

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;

static double GOAL_OFFSET = -0.45;


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
	node_.param("goal_csv", goal_csv_, string("../catkin_ws/src/my_navigation/goal/sample.csv"));
	node_.param("smooth_path", SMOOTH_PATH, true);
	node_.param("reach_tolerance", REACH_TOLERANCE, 0.5);
	node_.param("reach_tolerance_narrow", REACH_TOLERANCE_NARROW, 0.15);

	ros::Subscriber petPointSub = node_.subscribe("/pet_point", 10, petPointCallback);
	ros::Subscriber ojmStatus1Sub = node_.subscribe("/left_time", 10, ojmStatusCallback);
	ros::Publisher goal_id_pub = node_.advertise<std_msgs::String>("/goal_id", 10);

	vector<Goal> goals;

	ifstream ifs(goal_csv_);
	if(!ifs)
	{
		ROS_INFO("Cannot load %s.", goal_csv_.c_str());
		return -1;
	}

	//csvファイルを1行ずつ読み込む
	string str;
	while(getline(ifs,str)){
		string token;
		istringstream stream(str);

		//1行のうち、文字列とコンマを分割する
		while(getline(stream,token,',')){
			//すべて文字列として読み込まれるため
			//数値は変換が必要
			int temp=stof(token); //stof(string str) : stringをfloatに変換
			cout<<temp<<",";
		}
		cout<<endl;
	}


	// goals.push_back( Goal("carboxA_inside", "map", -1.6, -0.4 -0.24, M_PI/2) );

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
