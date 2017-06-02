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
	string ans = "";
	for (int i=0; i<text.size(); ++i)
	{
		if(text[i] == '\n' || text[i] == ' ')
		{
			text.erase(text.begin()+i);
			--i;
		}
		else
		{
			ans.push_back(text[i]);
		}
	}

	return ans;
}/*}}}*/

int main (int argc, char **argv)
{/*{{{*/
	ros::init (argc, argv, "mark_detector");
	ros::NodeHandle nh("~");
	ros::Rate looprate (5);

	int video_num_;
	string tessdata_path_, language_;
	nh.param("tessdata_path", tessdata_path_, string("/home/yamaji-s"));
	nh.param("language", language_, string("jpn"));
	nh.param("video_num", video_num_, 0);

	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;
	dynamic_reconfigure::DoubleParameter double_param;
	dynamic_reconfigure::Config conf;

	double_param.name = "max_vel_x";
	double_param.value = 0.0;
	conf.doubles.push_back(double_param);
	srv_req.config = conf;

	cv::Mat frame;

	// string filepath = "/home/yamaji-s/Pictures/hyousiki/sokudo30.jpg";
	// string filepath = "/home/yamaji-s/Pictures/hyousiki/zyoko.png";
	// string filepath = "/home/yamaji-s/Pictures/hyousiki/tomare.jpg";

	// frame = cv::imread(filepath);
	// if (!frame.data)
	// {
	// 	ROS_ERROR("cannot read image");
	// 	return -1;
	// }

	cv::VideoCapture cap(video_num_);
	if(!cap.isOpened())
	{
		ROS_ERROR("cannot open camera");
		return -1;
	}

	sy::TextDetection td(tessdata_path_, language_);

	sy::Image::HSV hsv(160,40, 100,255, 140,255);
	Mat extract;
	Mat cut;
	const double circle_cut_ratio_x = 0.60;
	const double circle_cut_ratio_y = 0.45;
	const double triangle_cut_ratio_x = 0.70;
	// const double triangle_cut_ratio_x = 0.50;
	const double triangle_cut_ratio_y = 0.3;

	int now_mark_vel = 0;
	int mark_vel = 30;

	conf.doubles[0].value = mark_vel/100.0;
	srv_req.config = conf;
	ros::service::call("/move_base/TrajectoryPlannerROS/set_parameters", srv_req, srv_resp);
	ROS_INFO("set max_vel_x : %f", mark_vel/100.0);
	now_mark_vel = mark_vel;

	while (ros::ok())
	{/*{{{*/
		cap >> frame;
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
			int triangle_rec_y = llist[max_num].min.y + llist[max_num].size.y*0.12;
			Mat triangle_img(frame, Rect(triangle_rec_x, triangle_rec_y, triangle_size_x, triangle_size_y));

			cv::imshow("cut_circle", circle_img);
			cv::imshow("cut_triangle", triangle_img);

			string text_c = trimText(td.textDetection(circle_img, true));
			string text_t = trimText(td.textDetection(triangle_img, true));
			// text_t = td.convertEncording(text_t, "UTF-8", "iso-2022-jp");

			// ROS_INFO("text_c \"%s\"", text_c.c_str());
			// ROS_INFO("text_t \"%s\"", text_t.c_str());

			string zyoko("徐行"), tomare("止まれ");
			zyoko = td.convertEncording(zyoko, "iso-2022-jp", "UTF-8");
			tomare = td.convertEncording(tomare, "iso-2022-jp", "UTF-8");

			// char nihongo[128];
			// strcpy(nihongo, zyoko.c_str());
			// cout<<"<zyoko>"<<endl;
			// for (int i=0; i<128; ++i)
			// {
			// 	if (nihongo[i] == '\0')
			// 		break;
			// 	cout<<nihongo[i];
			// }
			//
			// cout<<endl;
			// char nihongo2[128];
			// strcpy(nihongo2, text_t.c_str());
			// cout<<"<text_t>"<<endl;
			// for (int i=0; i<128; ++i)
			// {
			// 	if (nihongo2[i] == '\0')
			// 		break;
			// 	cout<<nihongo2[i];
			// }		
			// cout<<endl;
			//
			// cout<<strcmp(text_t.c_str(), zyoko.c_str())<<endl;

			// if (text_t == zyoko)
			if (text_t.find(tomare) != string::npos)
				mark_vel = 0;
			else if (text_t.find(zyoko) != string::npos)
				mark_vel = 10;
			else {
				try {
					mark_vel = stoi(text_c);
					double max_vel = mark_vel / 100.0;
					if (mark_vel%10 != 0 || mark_vel < 20 || 80 < mark_vel)
						mark_vel = now_mark_vel;
				}
				catch (std::invalid_argument e) {}
				catch (std::out_of_range e) {}
			}
		}

		if (mark_vel == 0)
		{
			conf.doubles[0].value = 0.0;
			srv_req.config = conf;
			ros::service::call("/move_base/TrajectoryPlannerROS/set_parameters", srv_req, srv_resp);
			ROS_INFO("set max_vel_x : %f", 0.0);
			sleep(5);
			mark_vel = now_mark_vel;
			now_mark_vel = 0;
		}

		if (mark_vel != now_mark_vel)
		{
			conf.doubles[0].value = mark_vel/100.0;
			srv_req.config = conf;
			ros::service::call("/move_base/TrajectoryPlannerROS/set_parameters", srv_req, srv_resp);
			ROS_INFO("set max_vel_x : %f", mark_vel/100.0);
			now_mark_vel = mark_vel;
		}

		cv::imshow("extracted", extract);
		cv::waitKey(1);

		ros::spinOnce();
		looprate.sleep();
	}/*}}}*/

	return 0;
}/*}}}*/


