// #include{{{
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <mylib/Lidar2D.h>
/*}}}*/

using namespace std;
Lidar2D ld2d;

struct Region
{/*{{{*/
	Pt::Pt2d min;
	Pt::Pt2d max;
};/*}}}*/

sensor_msgs::LaserScan::ConstPtr scan;
bool scan_subscribed = false;
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{/*{{{*/
	scan = msg;
	ld2d.scanPtrRegister(msg);
	scan_subscribed = true;
}/*}}}*/

string goal_id = "";
bool goal_id_subscribed = false;
void goalIDCallback(const std_msgs::String& msg)
{/*{{{*/
	goal_id = msg.data;
	goal_id_subscribed = true;
}/*}}}*/

bool isOjmExist(Region& region, tf::StampedTransform& transform)
{/*{{{*/
	double laser_x = transform.getOrigin().getX();
	double laser_y = transform.getOrigin().getY();
	double laser_yaw = tf::getYaw(transform.getRotation());

	ld2d.clustering(0.04, 10);

	for (int i=0; i<ld2d.clusterlist.size(); ++i) {
		Pt::Pt2d grav = ld2d.clusterlist[i].grav;
		Pt::Pt2d obstacle;
		obstacle.x = laser_x + grav.x*cos(laser_yaw) - grav.y*sin(laser_yaw);
		obstacle.y = laser_y + grav.x*sin(laser_yaw) + grav.y*cos(laser_yaw);
		if (region.min.x<obstacle.x && obstacle.x<region.max.x
				&& region.min.y<obstacle.y && obstacle.y<region.max.y) {
			return true;
		}
	}
	return false;
}/*}}}*/

int main(int argc, char **argv)
{/*{{{*/
	ros::init(argc, argv, "ojm_detector");
	ros::NodeHandle node_("~");
	int loop_rate = 10;
	ros::Rate looprate(loop_rate);
	ros::Subscriber scan_sub = node_.subscribe("/scan", 1, scanCallback);
	ros::Subscriber goal_id_sub = node_.subscribe("/goal_id", 1, goalIDCallback);
	ros::Publisher lefttime_pub = node_.advertise<std_msgs::Float32>("/left_time", 1);

	tf::TransformListener listener;
	tf::StampedTransform transform;
	bool tf_listened = false;

	Region region1;
	region1.min.x = -0.7;
	region1.max.x = -0.1;
	region1.min.y = -2.6;
	region1.max.y = -2.0;
	Region region2;
	region2.min.x = -2.8;
	region2.max.x = -1.3;
	region2.min.y = -2.6;
	region2.max.y = -2.0;

	Region region = region1;
	double time = 100.0;

	while (ros::ok()) {

		if (goal_id_subscribed)
		{
			time = 100.0;
			if (goal_id == "aisle_end")
				region = region1;
			if (goal_id == "pole_front")
				region = region2;
		}

		if (scan_subscribed && tf_listened && isOjmExist(region, transform))
			time = 0.0;
		else
			time += 1.0/loop_rate;

		std_msgs::Float32 msg;
		msg.data = time;
		lefttime_pub.publish(msg);

		try
		{/*{{{*/
			listener.lookupTransform("/map", "/laser", ros::Time(0), transform);
			tf_listened = true;
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			tf_listened = false;
		}/*}}}*/

		goal_id_subscribed = false;
		scan_subscribed = false;
		ros::spinOnce();
		looprate.sleep();
	}

	return 0;
}/*}}}*/

