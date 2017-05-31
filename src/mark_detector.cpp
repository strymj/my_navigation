// #include{{{
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
/*}}}*/

int main (int argc, char **argv)
{/*{{{*/
	ros::init (argc, argv, "mark_detector");
	ros::NodeHandle nh("~");
	ros::Rate looprate (30);

	int cnt = 0;
	double vel = 0.3;

	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;
	dynamic_reconfigure::DoubleParameter double_param;
	dynamic_reconfigure::Config conf;

	double_param.name = "max_vel_x";
	double_param.value = vel;
	conf.doubles.push_back(double_param);
	srv_req.config = conf;

	while (ros::ok())
	{/*{{{*/
		if (10*30 < cnt)
		{
			vel = 0.3;
			conf.doubles[0].value = vel;
			srv_req.config = conf;
			ros::service::call("/move_base/TrajectoryPlannerROS/set_parameters", srv_req, srv_resp);
			ROS_INFO("changed param   max_vel_x = %f", vel);
			cnt = 0;

		}
		else if (cnt == 5*30) 
		{
			vel = 0.9;
			conf.doubles[0].value = vel;
			srv_req.config = conf;
			ros::service::call("/move_base/TrajectoryPlannerROS/set_parameters", srv_req, srv_resp);
			ROS_INFO("changed param   max_vel_x = %f", vel);
		}
		ros::spinOnce();
		looprate.sleep();
		++cnt;
	}/*}}}*/

	return 0;
}/*}}}*/


