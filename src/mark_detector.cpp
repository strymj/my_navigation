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

string trimText(string text)
{/*{{{*/
	for (int i=0; i<text.size(); ++i)
	{
		if(text[i] == '\n' || text[i] == ' ')
		{
			text.erase(text.begin()+i);
			--i;
		}
	}

	return text;
}/*}}}*/

int main (int argc, char **argv)
{/*{{{*/
	ros::init (argc, argv, "mark_detector");
	ros::NodeHandle nh("~");
	ros::Rate looprate (5);

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

	cv::Mat frame;

	string filepath = "/home/yamaji-s/Pictures/sokudo30.jpg";
	// string filepath = "/home/yamaji-s/Pictures/tomare.jpg";
	// string filepath = "/home/yamaji-s/Pictures/zyoko.jpg";
	// string filepath = "/home/yamaji-s/Pictures/zyoko.png";

	frame = cv::imread(filepath);
	if (!frame.data)
	{
		ROS_ERROR("cannot read image");
		return -1;
	}

	// cv::VideoCapture cap(0);
	// if(!cap.isOpened())
	// {
	// 	ROS_ERROR("cannot open camera");
	// 	return -1;
	// }

	language_ = "jpn";
	sy::TextDetection td(tessdata_path_, language_);

	sy::Image::HSV hsv(160,40, 100,255, 140,255);
	Mat extract;
	Mat cut;
	const double circle_cut_ratio_x = 0.60;
	const double circle_cut_ratio_y = 0.45;
	const double triangle_cut_ratio_x = 0.70;
	const double triangle_cut_ratio_y = 0.25;

	int now_mark_vel = 0;

	while (ros::ok())
	{/*{{{*/
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
			int circle_size_x = llist[max_num].size.x * circle_cut_ratio_x;
			int circle_size_y = llist[max_num].size.y * circle_cut_ratio_y;
			int circle_rec_x = llist[max_num].min.x + llist[max_num].size.x*(1.0-circle_cut_ratio_x)/2;
			int circle_rec_y = llist[max_num].min.y + llist[max_num].size.y*(1.0-circle_cut_ratio_y)/2;
			Mat circle_img(frame, Rect(circle_rec_x, circle_rec_y, circle_size_x, circle_size_y));

			int triangle_size_x = llist[max_num].size.x * triangle_cut_ratio_x;
			int triangle_size_y = llist[max_num].size.y * triangle_cut_ratio_y;
			int triangle_rec_x = llist[max_num].min.x + llist[max_num].size.x*(1.0-triangle_cut_ratio_x)/2;
			int triangle_rec_y = llist[max_num].min.y + llist[max_num].size.y*0.16;
			Mat triangle_img(frame, Rect(triangle_rec_x, triangle_rec_y, triangle_size_x, triangle_size_y));

			cv::imshow("cut_circle", circle_img);
			cv::imshow("cut_triangle", triangle_img);

			string text_circle = trimText(td.textDetection(circle_img, true));
			string text_triangle = trimText(td.textDetection(triangle_img, true));

			ROS_INFO("text1 \"%s\"", text_circle.c_str());
			ROS_INFO("text2 \"%s\"", text_triangle.c_str());

			try {
				int mark_vel = stoi(text_circle);
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
		cv::imshow("extracted", extract);
		cv::waitKey(1);

		ros::spinOnce();
		looprate.sleep();
	}/*}}}*/

	return 0;
}/*}}}*/


