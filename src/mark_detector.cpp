// #include{{{
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <mylib/ImageProc.h>
using namespace std;
using namespace cv;
/*}}}*/

int main (int argc, char **argv)
{/*{{{*/
	ros::init (argc, argv, "mark_detector");
	ros::NodeHandle nh("~");
	ros::Rate looprate (10);

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

	// string filepath = "/home/strymj/Pictures/sokudo.jpg";
	// string filepath = "/home/strymj/Pictures/so.png";
	string filepath = "/home/strymj/Pictures/vel20.jpg";
	bool picture = true;
	cv::Mat frame;

	if (picture) 
	{
		frame = cv::imread(filepath);
		if (!frame.data)
		{
			ROS_ERROR("cannot read image");
			return -1;
		}
	}
	cv::VideoCapture cap(1);
	if(!cap.isOpened())
		return -1;



	my::Image::HSV hsv(160,40, 100,255, 190,255);
	Mat extract;
	Mat cut;


	while (ros::ok())
	{/*{{{*/

		if (!picture)
			cap >> frame;
		my::Image::colorExtract(frame, extract, hsv, 0);
		vector<my::Image::Regiondata> regionlist;
		my::Image::labeling(extract, regionlist);

		int max_num = -1;
		int pix = 0;
		for (int i=0; i<regionlist.size(); ++i)
		{
			if (pix<regionlist[i].pixels)
			{
				pix = regionlist[i].pixels;
				max_num = i;
			}
		}
		if (max_num != -1)
		{
			Mat cut_img(frame,Rect(regionlist[max_num].min.x,regionlist[max_num].min.y,regionlist[max_num].size.x,regionlist[max_num].size.y));
			cv::imshow("cut", cut_img);
		}
		cv::imshow("webcam", frame);
		cv::imshow("extracted", extract);
		cv::waitKey(1);

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


