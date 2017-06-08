#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <sylib/Lidar2D.h>
#include <sylib/Point.h>

using namespace std;

sy::Point2D explore_center(-1.0, -3.1);
static double samepet_tolerance = 0.13;
static double pet_d = 0.065;
static double pet_d_tolerance = 0.05;
static double explore_radius = 0.4;
static double explore_tolerance = pet_d + 0.03;

sy::Lidar2D ld2d;

sensor_msgs::LaserScan::ConstPtr scan;
bool scan_subscribed = false;
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{/*{{{*/
	// scan = *msg;
	scan = msg;
	ld2d.scanPtrRegister(msg);
	scan_subscribed = true;
}/*}}}*/

vector<sy::Point2D> getPetList(tf::StampedTransform& transform)
{/*{{{*/
	double laser_x = transform.getOrigin().getX();
	double laser_y = transform.getOrigin().getY();
	double laser_yaw = tf::getYaw(transform.getRotation());

	vector<sy::Point2D> list;
	ld2d.clustering(0.05, 10);

	for (int i=0; i<ld2d.clusterlist.size(); ++i) {
		// check cluster width
		if (pet_d - pet_d_tolerance < ld2d.clusterlist[i].width
				&& ld2d.clusterlist[i].width < pet_d + pet_d_tolerance) {
			// calclate pet (x,y) in map frame
			int scan_number = (ld2d.clusterlist[i].N_bgn + ld2d.clusterlist[i].N_end) /2;
			double norm = scan->ranges[scan_number] + pet_d/2; 
			double angle = scan->angle_min + scan->angle_increment * scan_number;
			sy::Point2D pet;
			pet.x = laser_x + norm * cos(laser_yaw + angle);
			pet.y = laser_y + norm * sin(laser_yaw + angle);
			// if in the explore area
			if (sy::distance(pet, explore_center) < explore_radius+explore_tolerance) {
				list.push_back(pet);
			}
		}
	}

	return list;
}/*}}}*/

void publishPointCloud(ros::Publisher& publisher, vector<sy::Point2D>& list)
{/*{{{*/
	static int seq = 0;
	sensor_msgs::PointCloud msg;
	msg.header.stamp = ros::Time::now();
	msg.header.seq = ++seq;
	msg.header.frame_id = "map";
	for (int i=0; i<list.size(); ++i) {
		geometry_msgs::Point32 petpoint;
		petpoint.x = list[i].x;
		petpoint.y = list[i].y;
		petpoint.z = 0.0;
		msg.points.push_back(petpoint);
	}
	publisher.publish(msg);
}/*}}}*/

int main(int argc, char **argv)
{/*{{{*/
	// ros init
	ros::init(argc, argv, "goal_publisher");
	ros::NodeHandle node_("~");
	ros::Rate looprate(10);
	ros::Subscriber scan_sub = node_.subscribe("/scan", 1, scanCallback);
	ros::Publisher petpoint_pub = node_.advertise<sensor_msgs::PointCloud>("/pet_point", 1);

	tf::TransformListener listener;
	tf::StampedTransform transform;
	bool tf_listened = false;

	// petlist.push_back(explore_center);

	vector<sy::Point2D> petlist;
	vector<sy::Point2D> newpetlist;

	while (ros::ok()) {

		if (scan_subscribed && tf_listened) {
			// cout<<"looping"<<endl;
			newpetlist = getPetList(transform);

			if (petlist.size() < newpetlist.size()) {
				petlist = newpetlist;
			}
		}

		publishPointCloud(petpoint_pub, petlist);

		try{
			listener.lookupTransform("/map", "/laser", ros::Time(0), transform);
			tf_listened = true;
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			tf_listened = false;
		}

		scan_subscribed = false;
		ros::spinOnce();
		looprate.sleep();
	}

	return 0;
}/*}}}*/

