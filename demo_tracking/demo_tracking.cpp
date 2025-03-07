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
#include "realsense_track.h"

#define UR_CYCLIC_TIME_MS 2
#define CONTROL_PERIOD_MS 40

// variable used in the control thread of UR
boost::condition_variable cRun;
boost::mutex mUpdate;
bool running;
double im3_actual_center[12];//3点图像实际像素位置，像素平面
double im3_actual_radius[4];//3点图像实际像素位置
std::vector<cv::Point2f> left_points, right_points;
// position of the target
Eigen::Vector3d pos_target;
extern double im3_dirsed_center[8];//3点图像期望像素位置
using namespace Robot;
Eigen::Matrix4d external_M,test_exM;
Eigen::Matrix<double, 3, 3> intenal_M;

Point3f getWorldPoints2(cv::Mat rotationMatrix_, cv::Mat cameraMatrix_, cv::Mat tvec_, Point2f inPoints)
{
	double zConst = 0;//实际坐标系的距离，若工作平面与相机距离固定可设置为0

	//计算参数	   
	double s;
	//获取图像坐标
	cv::Mat imagePoint = cv::Mat::ones(3, 1, cv::DataType<double>::type); //u,v,1
	cv::Mat camaPoint = cv::Mat::ones(3, 1, cv::DataType<double>::type); //u,v,1
	imagePoint.at<double>(0, 0) = inPoints.x;
	imagePoint.at<double>(1, 0) = inPoints.y;

	//计算比例参数S
	cv::Mat tempMat, tempMat2;
	tempMat = rotationMatrix_.inv() * cameraMatrix_.inv() * imagePoint;
	//cout << "camara base :=" << endl << 0.64 * cameraMatrix_.inv() * imagePoint << endl;
	//camaPoint = 0.64 * cameraMatrix_.inv() * imagePoint ;
	cv::Mat CamaraPoint = cv::Mat::ones(4, 1, cv::DataType<double>::type); //u,v,1
	//CamaraPoint.at<double>(0, 0) = (0.64 * cameraMatrix_.inv() * imagePoint);
	//cout << "robot base :=" << endl << 0.64 * cameraMatrix_.inv() * imagePoint << endl;
	tempMat2 = rotationMatrix_.inv() * tvec_;
	s = zConst + tempMat2.at<double>(2, 0);
	s /= tempMat.at<double>(2, 0);
	cout << "s:=" << s << endl;
	cout << "camara caculate s:=" << endl<< s* cameraMatrix_.inv()* imagePoint << endl;
	camaPoint = s * cameraMatrix_.inv() * imagePoint;
	//计算世界坐标
	//camara to word
	//0.64 * (cameraMatrix_.inv() * imagePoint).at<double>(0, 0);
	Eigen::Vector4d p_c_q{ camaPoint.at<double>(0,0),camaPoint.at<double>(1,0),camaPoint.at<double>(2,0),1 };
	Eigen::Vector4d p_w_q = test_exM* p_c_q;
	cout << "p_w_q:=" << p_w_q.transpose() << endl;
	//return Vector3d{ p_w_q(0,0),p_w_q(1,0),p_w_q(2,0) };


	Mat wcPoint = rotationMatrix_* (s * cameraMatrix_.inv() * imagePoint)+ tvec_;
	Point3f worldPoint(wcPoint.at<double>(0, 0), wcPoint.at<double>(1, 0), wcPoint.at<double>(2, 0));
	return worldPoint;
}

Point3f getWorldPoints(Eigen::Matrix3d external_R, Eigen::Matrix3d intrisci_M, Eigen::Vector3d external_t,Point2d inPoints)
{
	//at cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 摄像机内参数矩阵 */
	//Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
	double zConst = 0;//实际坐标系的距离，若工作平面与相机距离固定可设置为0

	double s;
	//获取图像坐标
	//cv::Mat imagePoint = cv::Mat::ones(3, 1, cv::DataType<double>::type); //u,v,1
	Eigen::Vector3d imagePoint;
	imagePoint(0) = inPoints.x;
	imagePoint(1) = inPoints.y;
	imagePoint(2) = 1;
	//计算比例参数S
	//cv::Mat tempMat, tempMat2;
	//Eigen::Matrix3d tempMat;
	Eigen::Vector3d	tempMat,tempMat2;
	tempMat = external_R.inverse() * intrisci_M.inverse() * imagePoint;
	tempMat2 = external_R.inverse() * external_t;
	s = zConst + tempMat2(2, 0);
	s /= tempMat(2, 0);
	cout << "s=" << s << endl;
	//计算世界坐标
	Eigen::Vector3d wcPoint = external_R.inverse() * (s * intrisci_M.inverse() * imagePoint) + external_t;
	Point3f worldPoint(wcPoint(0, 0), wcPoint(1, 0), wcPoint(2, 0));
	//Point3f worldPoint;
	return worldPoint;
}

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
		points.push_back(cv::Point2f(mn.m10 / mn.m00, mn.m01 / mn.m00));
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
	//controller_ur->useIBVS = true;
	external_M << 9.9999182276709764e-01, -2.5015706196520101e-03, -3.1775058414804393e-03, -3.6770868325372012e-02,
				2.4858905285792821e-03, 9.9998476195823494e-01, -4.9291175287571806e-03, -7.5690552499139668e-02,
				3.1897879581044689e-03, 4.9211782905393027e-03, 9.9998280348064750e-01, -2.5302551302136194e-02,
		        0,0,0,1;

	intenal_M << 8.8884850236407863e+02, 0., 6.4873567892875235e+02,//637.8,0,637.8,
		         0., 8.8769039996290348e+02, 3.6643649723707466e+02,
		         0., 0., 1.;
				
	while (running)
	{
		Eigen::Vector4d Pos_target;
		Eigen::Vector3d point_target;//u v zC
		point_target << im3_actual_center[0], im3_actual_center[1], im3_actual_center[2];
		//Pos_target = external_M * intenal_M * point_target;
		cout << "pixel pos and Zc:" << im3_actual_center[0]<<" "<< im3_actual_center[1]<<" "<< im3_actual_center[2] << endl;
		//Point3fcout << "Pos_target:" << Pos_target << endl;
		Eigen::Vector3d external_T = (controller_ur->Ttool_to_base *external_M).block<3, 1>(0, 3);
		//Eigen::Vector3d external_T;
		//external_T << -3.6770868325372012e-02, -7.5690552499139668e-02, -2.5302551302136194e-02;
		Eigen::Matrix3d external_R = (controller_ur->Ttool_to_base * external_M).block<3, 3>(0, 0);
		//cout << "controller_ur->Ttool_to_base:=" << controller_ur->Ttool_to_base << endl;
		//cout << " (controller_ur->Ttool_to_base * external_M)" << (controller_ur->Ttool_to_base * external_M) << endl;
		test_exM = controller_ur->Ttool_to_base * external_M;
		cv::Point2d  Pixel_point;
		Pixel_point.x = im3_actual_center[0];
		Pixel_point.y = im3_actual_center[1];
		//cv::Point3d Word_point;
		cv::Point3d Word_point=  getWorldPoints(external_R, intenal_M, external_T, Pixel_point);
		cv::Mat cv_intenal_M  = Mat(3, 3, CV_64FC1, Scalar::all(0));
		cv::Mat cv_external_R = Mat(3, 3, CV_64FC1, Scalar::all(0));
		cv::Mat cv_external_T = Mat(3, 1, CV_64FC1, Scalar::all(0));
		for(int i=0;i<3;i++)
			for (int j = 0; j < 3; j++)
			{
				cv_intenal_M.at<double>(i, j)  = intenal_M(i, j);
				cv_external_R.at<double>(i, j) = external_R(i, j);
				cv_external_T.at<double>(i, 0) = external_T(i, 0);

			}
		//cout <<"cv_external_R:="<<endl<< cv_external_R << endl;
		//cout << "cv_intenal_M:=" << endl << cv_intenal_M << endl;
		//cout << "cv_external_T:=" << endl << cv_external_T << endl;
		//cv::Point3d Word_point = getWorldPoints2(cv_external_R, cv_intenal_M, cv_external_T, Pixel_point);
		cout <<"Word_point="<< Word_point.x <<" "<< Word_point.y <<" " << Word_point.z << endl;
		//// the desired position specified by user or maybe the motion planner
		//Eigen::Vector3d x_des;
		//Eigen::Vector3d xd_des;
		//Eigen::Vector3d xdd_des;
		//x_des << 0.3, 0.0, 0.3;
		//xd_des << 0.0, 0.0, 0.0;
		//xdd_des << 0.0, 0.0, 0.0;

		//Eigen::Matrix3d R_des;
		//Eigen::Vector3d w_des;
		//Eigen::Vector3d wd_des;
		//R_des.setIdentity();
		//w_des.setZero();
		//wd_des.setZero();

		//controller_ur->set_desired_position(x_des, xd_des, xdd_des);
		//controller_ur->set_desired_orientation(R_des, w_des, wd_des);
		//set 3 point desired position
		controller_ur->im3point_des[0] = im3_dirsed_center[0]; controller_ur->im3point_des[1] = im3_dirsed_center[1];//大圆心
		controller_ur->im3point_des[2] = im3_dirsed_center[2]; controller_ur->im3point_des[3] = im3_dirsed_center[3]; //中圆心
		controller_ur->im3point_des[4] = im3_dirsed_center[4]; controller_ur->im3point_des[5] = im3_dirsed_center[5];//小圆心
		controller_ur->im3point_des[6] = im3_dirsed_center[6]; controller_ur->im3point_des[7] = im3_dirsed_center[7];//小圆心
		//set 3 point actual
		controller_ur->im3point_act[0] = im3_actual_center[0]; controller_ur->im3point_act[1] = im3_actual_center[1]; controller_ur->im3point_act[2] = 0.45;// im3_actual_center[2];
		controller_ur->im3point_act[3] = im3_actual_center[3]; controller_ur->im3point_act[4] = im3_actual_center[4]; controller_ur->im3point_act[5] = 0.45;// im3_actual_center[5];
		controller_ur->im3point_act[6] = im3_actual_center[6]; controller_ur->im3point_act[7] = im3_actual_center[7]; controller_ur->im3point_act[8] = 0.45;// im3_actual_center[8];
		controller_ur->im3point_act[9] = im3_actual_center[9]; controller_ur->im3point_act[10] = im3_actual_center[10]; controller_ur->im3point_act[11] = 0.45;// im3_actual_center[8];
		//cout << "center:" << im3_actual_center[0] << "  " << im3_actual_center[1] << "  " << im3_actual_center[3] << "  " << im3_actual_center[4]
			        // <<" "<< im3_actual_center[6] <<"  "  <<im3_actual_center[7] << " "   << im3_actual_center[9] << " " << im3_actual_center[10] << endl;
		//controller_ur->im3point_act[0] = 499.5; controller_ur->im3point_act[1] = 178.5+10; controller_ur->im3point_act[2] = 5.4;// im3_actual_center[2];
		//controller_ur->im3point_act[3] = 332.5; controller_ur->im3point_act[4] = 179.5+10; controller_ur->im3point_act[5] = 5.4;// im3_actual_center[5];
		//controller_ur->im3point_act[6] = 340.5; controller_ur->im3point_act[7] = 63.5+10; controller_ur->im3point_act[8] = 5.4;// im3_actual_center[8];
		if (im3_actual_center[0] == 0 || im3_actual_center[3] ==0|| im3_actual_center[6] ==0 || im3_actual_center[9] == 0)
		{
			cout << "not detected " << endl;
			controller_ur->specify_zero_qcd();
			continue;
		}
		controller_ur->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
		for (int i = 0; i < CONTROL_PERIOD_MS / UR_CYCLIC_TIME_MS; i++)
		{
			controller_ur->specify_qcd((double)UR_CYCLIC_TIME_MS * (i + 1) / CONTROL_PERIOD_MS);
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
	// variables for connecting ur
	boost::asio::io_service io_service;
	boost::asio::io_service::work work(io_service);
	ur_sdk::client client(io_service, cRun, argv[3], argv[4]);

	CartesianSpaceTrackUR controller_ur(client);
	controller_ur.controller_type = CartesianSpaceTrackUR::CONTROLLER_TYPE::FULL_POSE;
	controller_ur.Kp = 1.0;

	running = true;
	// start the threads
	boost::thread thread_io_service(boost::bind(&boost::asio::io_service::run, &io_service));
	boost::thread thread_control_ur(boost::bind(&control_ur, &controller_ur));
	//boost::thread thread_detect_target(boost::bind(&detect_target, &stereo_camera));
	//boost::thread thread_show(boost::bind(&show_stereo, &stereo_camera));
	boost::thread thread_track(&track);
	SetThreadPriority(thread_io_service.native_handle(), THREAD_PRIORITY_TIME_CRITICAL);
	SetThreadPriority(thread_control_ur.native_handle(), THREAD_PRIORITY_TIME_CRITICAL);
	
	_getch();
	running = false;

	io_service.stop();
	thread_io_service.join();
	thread_control_ur.join();
	//thread_detect_target.join();
	//thread_show.join();
	thread_track.join();
	return -1;
	
	//track();
	/*
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

		for (int i = 0; i < left_points.size(); i++)
		{
			cv::circle(view_rgb, left_points[i], 5, cv::Scalar(0, 0, 255), 3);
		}

		cv::Mat view1(left_img.rows / 4, left_img.cols / 4, CV_8UC1);
		cv::resize(right_img, view1, cv::Size(), 0.25, 0.25);
		cv::Mat view_rgb1(left_img.rows / 4, left_img.cols / 4, CV_8UC3);
		cv::resize(right_img, view1, cv::Size(), 0.25, 0.25);
		cv::cvtColor(view1, view_rgb1, cv::COLOR_GRAY2RGB);

		for (int i = 0; i < right_points.size(); i++)
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
	*/
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
