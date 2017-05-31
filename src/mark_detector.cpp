// #include{{{
#include <ros/ros.h>
#include <iostream>
#include <string>
/*}}}*/

int main (int argc, char **argv)
{/*{{{*/
	ros::init (argc, argv, "mark_detector");
	ros::NodeHandle nh("~");
	ros::Rate looprate (30);

	double time = 0;
	double vel = 0.0;

	while (ros::ok())
	{/*{{{*/
		if (5<time) 
		{
			nh.setParam("/move_base/TrajectoryPlannerROS/max_vel_x", 0.5);
			// nh.setParam("/hogehoge/piyopiyo", 0.5);
			ROS_INFO("changed param!");
			time = 0.0;
			nh.getParam("/move_base/TrajectoryPlannerROS/max_vel_x", vel);
			ROS_INFO("max_vel_x : %f", vel);
		}
		ros::spinOnce();
		looprate.sleep();
		time += 1.0/30;
	}/*}}}*/

	return 0;
}/*}}}*/


