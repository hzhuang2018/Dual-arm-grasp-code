// Copyright (c) 2021 Yipeng Li @ BICE (yipengli.bice@gmail.com)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//


// demo_tracking.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <cartesian_space_track_ur.h>
#include <camera.h>
#include <ur_sdk.h>
#include <conio.h>
#include <opencv2\opencv.hpp>
#include <vector>
#define _USE_MATH_DEFINES
#include<math.h>

#define UR_CYCLIC_TIME_MS 2
#define CONTROL_PERIOD_MS 40

// variable used in the control thread of UR
boost::condition_variable cRun;
boost::mutex mUpdate;
bool running;

std::vector<cv::Point2f> left_points, right_points;
// position of the target
Eigen::Vector3d pos_target;

using namespace Robot;

std::vector<cv::Point2f> detect_points(const cv::Mat& img_)
{
	std::vector<cv::Point2f> points;
	cv::Mat img = img_;
	cv::blur(img, img, cv::Size(3, 3));

	cv::Mat binary_img(img.size(), CV_8UC1);
	cv::threshold(img, binary_img, 100, 150, cv::THRESH_OTSU || cv::THRESH_BINARY_INV);
	vector<vector<cv::Point>> contour, contour_candidate;
	vector<cv::Vec4i> hierarchy;
	cv::findContours(binary_img, contour, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));

	int c = 0, ic = 0, area = 0;
	int parent_id = -1;
	for (int i = 0; i < contour.size(); i++)
	{
		if (hierarchy[i][2] != -1 && ic == 0)
		{
			parent_id = i;
			ic++;
		}
		else if (hierarchy[i][2] != -1)
		{
			ic++;
		}
		//最外面的清0
		else if (hierarchy[i][2] == -1)
		{
			ic = 0;
			parent_id = -1;
		}
		//找到定位点信息
		if (ic >= 2)
		{
			contour_candidate.push_back(contour[parent_id]);
			ic = 0;
			parent_id = -1;
		}
	}

	for (int i = 0; i < contour_candidate.size(); i++)
	{
		cv::Moments mn = cv::moments(cv::Mat(contour_candidate[i]));
		points.push_back(cv::Point2f(mn.m10/mn.m00, mn.m01/mn.m00));
		//std::cout << cv::Point2f(mn.m10 / mn.m00, mn.m01 / mn.m00) << std::endl;
	}

	return points;
}

cv::Point2f ball_tracking(cv::Mat& img)
{
	cv::Point2f center;

	return center;
}

void control_ur(CartesianSpaceTrackUR* controller_ur)
{
	boost::mutex::scoped_lock sl(mUpdate);
	cRun.wait(sl);
	boost::posix_time::ptime start_time = boost::get_system_time();
	controller_ur->start(start_time);

	while (running)
	{
		// the desired position specified by user or maybe the motion planner
		Eigen::Vector3d x_des;
		Eigen::Vector3d xd_des;
		Eigen::Vector3d xdd_des;
		x_des << 0.3, 0.0, 0.3;
		xd_des << 0.0, 0.0, 0.0;
		xdd_des << 0.0, 0.0, 0.0;

		Eigen::Matrix3d R_des;
		Eigen::Vector3d w_des;
		Eigen::Vector3d wd_des;
		R_des.setIdentity();
		w_des.setZero();
		wd_des.setZero();

		controller_ur->set_desired_position(x_des, xd_des, xdd_des);
		controller_ur->set_desired_orientation(R_des, w_des, wd_des);

		controller_ur->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
		for (int i = 0; i < CONTROL_PERIOD_MS / UR_CYCLIC_TIME_MS; i++)
		{
			controller_ur->specify_qcd((double)UR_CYCLIC_TIME_MS *(i + 1) / CONTROL_PERIOD_MS);
			cRun.wait(sl);
		}
	}
	boost::posix_time::ptime stop_time = boost::get_system_time();
	controller_ur->stop(stop_time);
}
// detect target from the stereo images and calculate the position
void detect_target(BaslerStereoCamera* stereo_camera)
{
	while (running)
	{
		cv::Mat left_img, right_img;
		left_img = stereo_camera->get_left_image();
		right_img = stereo_camera->get_right_image();
		if (left_img.data)
		{
			cv::Mat view(left_img.rows / 4, left_img.cols / 4, CV_8UC1);
			cv::resize(left_img, view, cv::Size(), 0.25, 0.25);
			left_points = detect_points(view);
		}
		if (right_img.data)
		{
			cv::Mat view(left_img.rows / 4, left_img.cols / 4, CV_8UC1);
			cv::resize(right_img, view, cv::Size(), 0.25, 0.25);
			right_points = detect_points(view);
		}
	}
}

int main(int argc, char* argv[])
{
	SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS);

	if (argc < 5)
	{
		printf("Please run this program by: demo_tracking leftcam_sn rightcam_sn <ip_ur> <port_ur>");
		return -1;
	}
	// camera
	std::string left_sn(argv[1]);
	std::string right_sn(argv[2]);
	BaslerStereoCamera stereo_camera;
	stereo_camera.set_exposure(15000, 15000);
	stereo_camera.setup(left_sn, right_sn);
	// start the loop thread for grabing images
	stereo_camera.start();

	// variables for connecting ur
	boost::asio::io_service io_service;
	boost::asio::io_service::work work(io_service);
	ur_sdk::client client(io_service, cRun, argv[3], argv[4]);

	CartesianSpaceTrackUR controller_ur(client);
	controller_ur.controller_type = CartesianSpaceTrackUR::CONTROLLER_TYPE::FULL_POSE;
	controller_ur.Kp = 2.0;

	running = true;
	// start the threads
	boost::thread thread_io_service(boost::bind(&boost::asio::io_service::run, &io_service));
	boost::thread thread_control_ur(boost::bind(&control_ur, &controller_ur));
	boost::thread thread_detect_target(boost::bind(&detect_target, &stereo_camera));
	//boost::thread thread_show(boost::bind(&show_stereo, &stereo_camera));

	SetThreadPriority(thread_io_service.native_handle(), THREAD_PRIORITY_TIME_CRITICAL);
	SetThreadPriority(thread_control_ur.native_handle(), THREAD_PRIORITY_TIME_CRITICAL);

	cv::Mat view_total;
	cv::Mat left_img, right_img;
	cv::Mat last1, last2;
	bool first_run = true;
	while (true)
	{
		left_img = stereo_camera.get_left_image();
		right_img = stereo_camera.get_right_image();

		
		if (!left_img.data) continue;
		if (!right_img.data) continue;

		cv::Mat view(left_img.rows / 4, left_img.cols / 4, CV_8UC1);
		cv::resize(left_img, view, cv::Size(), 0.25, 0.25);
		cv::Mat view_rgb(left_img.rows / 4, left_img.cols / 4, CV_8UC3);
		cv::resize(left_img, view, cv::Size(), 0.25, 0.25);
		cv::cvtColor(view, view_rgb, cv::COLOR_GRAY2RGB);

		for (int i = 0;i < left_points.size(); i++)
		{
			cv::circle(view_rgb, left_points[i], 5, cv::Scalar(0, 0, 255), 3);
		}

		cv::Mat view1(left_img.rows / 4, left_img.cols / 4, CV_8UC1);
		cv::resize(right_img, view1, cv::Size(), 0.25, 0.25);
		cv::Mat view_rgb1(left_img.rows / 4, left_img.cols / 4, CV_8UC3);
		cv::resize(right_img, view1, cv::Size(), 0.25, 0.25);
		cv::cvtColor(view1, view_rgb1, cv::COLOR_GRAY2RGB);

		for (int i = 0;i < right_points.size(); i++)
		{
			cv::circle(view_rgb1, right_points[i], 5, cv::Scalar(0, 0, 255), 3);
		}
		if (first_run)
		{
			last1 = view_rgb;
			last2 = view_rgb1;
			first_run = false;
		}


		//stereo_calib.draw_pattern(view, view_rgb);
		//stereo_calib.draw_pattern(view1, view_rgb1);
		cv::Mat ret1, ret2;
		cv::absdiff(view_rgb, last1, ret1);
		cv::absdiff(view_rgb1, last2, ret2);
		cv::hconcat(view_rgb, view_rgb1, view_total);
		cv::imshow("Stereo view", view_total);
		last1 = view_rgb;
		last2 = view_rgb1;
		int key = cv::waitKey(30);
		if (key == 'q')
		{
			cv::destroyAllWindows();
			stereo_camera.stop();
			
			break;
		}
	}
	
	//_getch();
	running = false;

	io_service.stop();
	thread_io_service.join();
	thread_control_ur.join();
	thread_detect_target.join();
	//thread_show.join();

	return -1;
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
