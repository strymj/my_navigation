// #include{{{
#include <ros/ros.h>
#include <iostream>
#include <string>
/*}}}*/

int main (int argc, char **argv)
{/*{{{*/
	ros::init (argc, argv, "mark_sample");
	ros::NodeHandle nh("~");
	ros::Rate looprate (30);

	double time = 0.0;
	double hogepiyo = 0.0;
	nh.param("/hogehoge/piyopiyo", hogepiyo, 0.111);

	while (ros::ok())
	{/*{{{*/
		if (5<time) 
		{
			nh.getParam("/hogehoge/piyopiyo", hogepiyo);
			ROS_INFO("hpgepiyo : %f", hogepiyo);
			time = 0.0;
			// nh.getParam("/move_base/TrajectoryPlannerROS/max_vel_x", vel);
			// std::cout<<vel<<std::endl;
		}
		ros::spinOnce();
		looprate.sleep();
		time += 1.0/30;
	}/*}}}*/

	return 0;
}/*}}}*/


