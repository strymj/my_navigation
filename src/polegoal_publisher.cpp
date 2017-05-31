#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <lib2dl/core.h>

using namespace std;

#define DMIN 0.08
#define DMAX 0.13
#define SMIN 0.4
#define SMAX 1.2
#define AMIN -M_PI/3
#define AMAX M_PI/3
#define WMIN 0.7
#define WMAX 0.9


bool ScanFlag = false;
sensor_msgs::LaserScan scan;
void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	scan = *msg;
	ScanFlag = true;
}

bool FlagString = false;
void FlagStringCallback(const std_msgs::String& msg)
{
	if (msg.data == "PoleGoalCalc") {
		FlagString = true;
	}
}

Lib2dl lib2dl;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "goal_publisher");
	ros::NodeHandle node_("~");
	ros::Rate looprate(30);
	ros::Subscriber FlagStringSub = node_.subscribe("/FlagString", 1, FlagStringCallback);
	ros::Subscriber ScanSub = node_.subscribe("/scan", 1, ScanCallback);
	ros::Publisher PoleGoalPub = node_.advertise<geometry_msgs::Pose2D>("/PoleGoal", 1);

	tf::TransformListener listener;
	tf::StampedTransform transform;
	
	vector<Lib2dl::Cluster> polelist;
	lib2dl.SetLidarFrame(0.15, 0.0);

	while (ros::ok()) {
		if (FlagString && ScanFlag) {
			lib2dl.ScanRegister(scan);
			lib2dl.Clustering(0.05, 0.01, 10);
			polelist.clear();
			for (int i=0; i<lib2dl.clusterlist.size(); ++i) {
				if (DMIN<lib2dl.clusterlist[i].width && lib2dl.clusterlist[i].width<DMAX 
						&& SMIN<lib2dl.clusterlist[i].dist && lib2dl.clusterlist[i].dist<SMAX
						&& AMIN<lib2dl.clusterlist[i].angle && lib2dl.clusterlist[i].angle<AMAX) {
					polelist.push_back(lib2dl.clusterlist[i]);
					cout<<lib2dl.clusterlist[i].dist<<"  , "<<lib2dl.clusterlist[i].width<<"  , "<<lib2dl.clusterlist[i].angle<<endl;
				}
			}
			cout<<"polelist.size() = "<<polelist.size()<<endl;
			cout<<endl;
			
			try{
				listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				// ros::Duration(1.0).sleep();
			}

			// int rm_num = -1;
			// if (polelist.size() == 3) {
			// 	for (int i=0; i<polelist.size(); ++i) {
			// 		for (int j=i+1; j<polelist.size(); j++) {
			// 			double diff_ang = polelist[i].angle - polelist[j].angle;
			// 			if (fabs(diff_ang) < M_PI/5) {
			// 				if (polelist[i].dist < polelist[j].dist) {
			// 					rm_num = j;
			// 				}
			// 				else {
			// 					rm_num = i;
			// 				}
			// 			}
			// 		}
			// 	}
			// }

			if (polelist.size() == 2) {
				double gatewidth = sqrt(
						pow(polelist[0].grav.x-polelist[1].grav.x, 2) +
						pow(polelist[0].grav.y-polelist[1].grav.y, 2) );
				if (WMIN<gatewidth && gatewidth<WMAX) break;
			}
		}
		ros::spinOnce();
		looprate.sleep();
	}

	double robot_x = transform.getOrigin().getX();
	double robot_y = transform.getOrigin().getY();
	double robot_yaw = tf::getYaw(transform.getRotation());

	double goal_x = (polelist[0].grav.x + polelist[1].grav.x) /2;
	double goal_y = (polelist[0].grav.y + polelist[1].grav.y) /2;

	double norm = sqrt(goal_x*goal_x + goal_y*goal_y);

	cout<<"polegoal_x = "<<goal_x<<endl;
	cout<<"polegoal_y = "<<goal_y<<endl;

	cout<<"robot_x = "<<robot_x<<endl;
	cout<<"robot_y = "<<robot_y<<endl;
	cout<<"robot_t = "<<robot_yaw<<endl;

	geometry_msgs::Pose2D msg;
	msg.x = robot_x + norm * cos(robot_yaw);
	msg.y = robot_y + norm * sin(robot_yaw);
	msg.theta = M_PI/2.0;

	PoleGoalPub.publish(msg);

	cout<<"PoleGole Published."<<endl;
	cout<<"x : "<<msg.x<<endl;
	cout<<"y : "<<msg.y<<endl;
	cout<<"t : "<<msg.theta<<endl;

	return 0;
}
