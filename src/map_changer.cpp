#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <tf/transform_listener.h>
#include "lib2dl.h"

using namespace std;

nav_msgs::OccupancyGrid map_0;
nav_msgs::OccupancyGrid map_m5;
nav_msgs::OccupancyGrid map_m10;
nav_msgs::OccupancyGrid map_m15;
nav_msgs::OccupancyGrid map_m20;
nav_msgs::OccupancyGrid map_m25;
nav_msgs::OccupancyGrid map_m30;
nav_msgs::OccupancyGrid map_p5;
nav_msgs::OccupancyGrid map_p10;
nav_msgs::OccupancyGrid map_p15;
nav_msgs::OccupancyGrid map_p20;
nav_msgs::OccupancyGrid map_p25;
nav_msgs::OccupancyGrid map_p30;
int subnum = 0;

void Map_0Callback  (const nav_msgs::OccupancyGrid& msg) {subnum++;map_0   = msg;}
void Map_m5Callback (const nav_msgs::OccupancyGrid& msg) {subnum++;map_m5  = msg;}
void Map_m10Callback(const nav_msgs::OccupancyGrid& msg) {subnum++;map_m10 = msg;}
void Map_m15Callback(const nav_msgs::OccupancyGrid& msg) {subnum++;map_m15 = msg;}
void Map_m20Callback(const nav_msgs::OccupancyGrid& msg) {subnum++;map_m20 = msg;}
void Map_m25Callback(const nav_msgs::OccupancyGrid& msg) {subnum++;map_m25 = msg;}
void Map_m30Callback(const nav_msgs::OccupancyGrid& msg) {subnum++;map_m30 = msg;}
void Map_p5Callback (const nav_msgs::OccupancyGrid& msg) {subnum++;map_p5  = msg;}
void Map_p10Callback(const nav_msgs::OccupancyGrid& msg) {subnum++;map_p10 = msg;}
void Map_p15Callback(const nav_msgs::OccupancyGrid& msg) {subnum++;map_p15 = msg;}
void Map_p20Callback(const nav_msgs::OccupancyGrid& msg) {subnum++;map_p20 = msg;}
void Map_p25Callback(const nav_msgs::OccupancyGrid& msg) {subnum++;map_p25 = msg;}
void Map_p30Callback(const nav_msgs::OccupancyGrid& msg) {subnum++;map_p30 = msg;}

void globalCostmapCallback(const nav_msgs::OccupancyGrid& msg) {cout<<"global costmap subscribed"<<endl;}

sensor_msgs::LaserScan scan;
bool scanflag = false;
void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	scan = *msg;
	scanflag = true;
}

bool MapChangeFlag = false;
void MapChangeFlagCallback(const std_msgs::String& msg)
{
	if (msg.data == "MapChange") {
		MapChangeFlag = true;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_canger");
	ros::NodeHandle node_("~");

	ros::Rate looprate(30);
	ros::Subscriber Map_0Sub = node_.subscribe("/map_0", 1, Map_0Callback);
	ros::Subscriber Map_m5Sub = node_.subscribe("/map_m5", 1, Map_m5Callback);
	ros::Subscriber Map_m10Sub = node_.subscribe("/map_m10", 1, Map_m10Callback);
	ros::Subscriber Map_m15Sub = node_.subscribe("/map_m15", 1, Map_m15Callback);
	ros::Subscriber Map_m20Sub = node_.subscribe("/map_m20", 1, Map_m20Callback);
	ros::Subscriber Map_m25Sub = node_.subscribe("/map_m25", 1, Map_m25Callback);
	ros::Subscriber Map_m30Sub = node_.subscribe("/map_m30", 1, Map_m30Callback);
	ros::Subscriber Map_p5Sub = node_.subscribe("/map_p5", 1, Map_p5Callback);
	ros::Subscriber Map_p10Sub = node_.subscribe("/map_p10", 1, Map_p10Callback);
	ros::Subscriber Map_p15Sub = node_.subscribe("/map_p15", 1, Map_p15Callback);
	ros::Subscriber Map_p20Sub = node_.subscribe("/map_p20", 1, Map_p20Callback);
	ros::Subscriber Map_p25Sub = node_.subscribe("/map_p25", 1, Map_p25Callback);
	ros::Subscriber Map_p30Sub = node_.subscribe("/map_p30", 1, Map_p30Callback);
	
	ros::Subscriber GlobalCostmapSub = node_.subscribe("/move_base/global_costmap/costmap", 1, globalCostmapCallback);

	// ros::Subscriber MovebaseResultSub = node_.subscribe("/move_base/result", 1, MoveBaseResultCallback);
	ros::Subscriber ScanSub = node_.subscribe("/scan", 1, ScanCallback);
	ros::Subscriber MapChangeFlagSub = node_.subscribe("/FlagString", 1, MapChangeFlagCallback);
	ros::Publisher MapPub = node_.advertise<nav_msgs::OccupancyGrid>("/map", 1);
	ros::Publisher CarboxTiltPub = node_.advertise<std_msgs::Int32>("/CarboxTilt", 1);

	Lib2dl lib2dl;
	lib2dl.SetLidarFrame(0.15, 0.0);

	tf::TransformListener listener;
	tf::StampedTransform transform;


	vector<Lib2dl::LineData> linelist;
	vector<Lib2dl::LineData> carbox_end;
	while (ros::ok()) {
		// if (scanflag) {
		if (subnum == 13 && MapChangeFlag && scanflag) {
			cout<<"##### searching carbox endline #####"<<endl;
			linelist.clear();
			carbox_end.clear();
			lib2dl.ScanRegister(scan);
			lib2dl.Clustering(0.05, 0.01, 10);
			lib2dl.GetLineList(linelist, 0.006, false);
			for (int i=0; i<linelist.size(); ++i) {
				bool reversed = linelist[i].reversed;
				double theta = atan(linelist[i].a);
				double dist = linelist[i].dist;
				if (reversed && -M_PI/4<theta && theta<M_PI/4 && 0.4<dist && dist<0.8) {
					carbox_end.push_back(linelist[i]);
				}
			}

			try{
				listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				// ros::Duration(1.0).sleep();
			}


			if (carbox_end.size() == 1) {
				break;
			}
		}
		scanflag = false;
		ros::spinOnce();
		looprate.sleep();
	}

	double robot_yaw = tf::getYaw(transform.getRotation());
	double carbox_yaw = -atan(carbox_end[0].a);
	double deg = fmod((robot_yaw+M_PI/2.0 + carbox_yaw) * 180/M_PI, 360.0);
	cout<<"deg = "<<deg<<endl;

	int step = (int)(deg/5.0 + 100.5) - 100;   // 94.5 ~ 106.5 -> 94 ~ 106 -> -6 ~ 6
	nav_msgs::OccupancyGrid msg;
	switch (step) {
		case 6:
			msg = map_p30;
			break;
		case 5:
			msg = map_p25;
			break;
		case 4:
			msg = map_p20;
			break;
		case 3:
			msg = map_p15;
			break;
		case 2:
			msg = map_p10;
			break;
		case 1:
			msg = map_p5;
			break;
		case -6:
			msg = map_m30;
			break;
		case -5:
			msg = map_m25;
			break;
		case -4:
			msg = map_m20;
			break;
		case -3:
			msg = map_m15;
			break;
		case -2:
			msg = map_m10;
			break;
		case -1:
			msg = map_m5;
			break;
		case 0:
			msg = map_0;
			break;
		default:
			msg = map_0;
			cout<<"error : out of range."<<endl;
			break;
	}
	MapPub.publish(msg);

	std_msgs::Int32 CarboxTiltMsg;
	CarboxTiltMsg.data = step;
	CarboxTiltPub.publish(CarboxTiltMsg);

	cout<<"map changed.  Tilt : "<<step*5<<"[deg]"<<endl;

	return 0;
	}
