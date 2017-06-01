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
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <mylib/ImageProc.h>
#include <mylib/TextDetection.h>
using namespace std;
using namespace cv;
/*}}}*/

int main (int argc, char **argv)
{/*{{{*/
	ros::init (argc, argv, "mark_detector");
	ros::NodeHandle nh("~");
	ros::Rate looprate (10);

	string tessdata_path_, language_;
	nh.param("tessdata_path", tessdata_path_, string("/home/yamaji-s"));
	nh.param("language", language_, string("eng"));

	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;
	dynamic_reconfigure::DoubleParameter double_param;
	dynamic_reconfigure::Config conf;

	double_param.name = "max_vel_x";
	double_param.value = 0.0;
	conf.doubles.push_back(double_param);
	srv_req.config = conf;

	string filepath = "/home/yamaji-s/Pictures/sokudo30.jpg";
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

	// cv::VideoCapture cap(0);
	// if(!cap.isOpened())
	// {
	// 	ROS_ERROR("cannot open camera");
	// 	return -1;
	// }

	sy::TextDetection td(tessdata_path_, language_);

	sy::Image::HSV hsv(160,40, 100,255, 140,255);
	Mat extract;
	Mat cut;
	const double cut_ratio_x = 0.55;
	const double cut_ratio_y = 0.45;

	int now_mark_vel = 0;

	while (ros::ok())
	{/*{{{*/
		// if (!picture)
		// cap >> frame;
		// resize(frame, frame, cv::Size(), 0.75, 0.75);
		sy::Image::colorExtract(frame, extract, hsv, 0);
		vector<sy::Image::Regiondata> llist;
		sy::Image::labeling(extract, llist);

		int max_num = -1;
		int pix = 0;
		for (int i=0; i<llist.size(); ++i)
		{
			if (pix<llist[i].pixels)
			{
				pix = llist[i].pixels;
				max_num = i;
			}
		}
		if (max_num != -1)
		{
			int size_x = llist[max_num].size.x * cut_ratio_x;
			int size_y = llist[max_num].size.y * cut_ratio_y;
			int rec_x = llist[max_num].min.x + llist[max_num].size.x*(1.0-cut_ratio_x)/2;
			int rec_y = llist[max_num].min.y + llist[max_num].size.y*(1.0-cut_ratio_y)/2;
			Mat cut_img(frame,Rect(rec_x, rec_y, size_x, size_y));
			cv::imshow("cut", cut_img);

			string sokudo = td.textDetection(cut_img, true);

			for (int i=0; i<sokudo.size(); ++i)
			{
				if(sokudo[i] == '\n' || sokudo[i] == ' ')
				{
					sokudo.erase(sokudo.begin()+i,sokudo.end());
					break;
				}
			}

			try {
				int mark_vel = stoi(sokudo);
				double max_vel = mark_vel / 100.0;
				if (0.1 < max_vel && max_vel < 1.2 && mark_vel != now_mark_vel)
				{
					conf.doubles[0].value = max_vel;
					srv_req.config = conf;
					ros::service::call("/move_base/TrajectoryPlannerROS/set_parameters", srv_req, srv_resp);
					ROS_INFO("set max_vel_x : %f", max_vel);
					now_mark_vel = mark_vel;
				}
			}
			catch (std::invalid_argument e) {}
			catch (std::out_of_range e) {}

		}
		cv::imshow("webcam", frame);
		cv::imshow("extracted", extract);
		cv::waitKey(1);

		ros::spinOnce();
		looprate.sleep();
	}/*}}}*/

	return 0;
}/*}}}*/


