// ur_romote_control.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include <iostream>

//#include <chrono>
#include <fstream>
#include <cartesian_space_track_ur.h>
#include <camera.h>
#include <ur_sdk.h>
#include <conio.h>
#include <opencv2\opencv.hpp>
#include <vector>
#include <algorithm>
#define _USE_MATH_DEFINES
//#define ARM_calib
//#define dua_arm
#define single_arm
#include<math.h>
#include <opencv2/core/eigen.hpp>
#include <iomanip>
#include <algorithm>
#include <ctime>
#include "..\include\alleprohand\myAllegroHand.h"
#include "..\include\touch_sensor\ControlCAN.h"
//#include "..\include\fuzzy_controller.h"
//#include "..\src\touch_sensor\CANTest.cpp"
// Collision, Distance 
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>
// Distance Request & Result
#include <fcl/narrowphase/distance_request.h>
#include <fcl/narrowphase/distance_result.h>
//#include"ur_js_cal_and_run.h"
#include<chrono>
#include "collision_det.h"
#include "planner_joint.h"
#include "path_generator.h"
#include "planner_cartisian.h"
#include "realsense_position.h"
#include "omp.h"
using namespace std::chrono;
using namespace Robot;  

//extern double im3_actual_center[12];//3点图像实际像素位置
//extern double im3_actual_radius[4];//3点图像实际像素半径
#define UR_CYCLIC_TIME_MS 8
#define CONTROL_PERIOD_MS 8   //40

// variable used in the control thread of UR
boost::condition_variable cRun, cRun2;
boost::mutex mUpdate,mUpdate2;
bool running;

std::vector<cv::Point2f> left_points, right_points;
 //position of the target
Eigen::Vector3d pos_target;
int inputnum = 0;

//using namespace std::chrono;
extern bool bCrash ;
extern int touch_sensor_read_data();
extern int touch_sensor_read_data2();
void   Delay(int   time)//time*1000为秒数 
{
	clock_t   now = clock();

	while (clock() - now < time);
}

const double ZERO_THRESH = 0.00000001;
int SIGN(double x) {
	return (x > 0) - (x < 0);
}
//UR3
//const double d1 = 0.1519;
//const double L2 = -0.24365;
//const double L3 = -0.21325;
//const double d4 = 0.11235;
//const double d5 = 0.08535;
//const double d6 = 0.0819;
//UR10
//const double d1 = 0.1273;
//const double L2 = -0.612;
//const double L3 = -0.5723;
//const double d4 = 0.163941;
//const double d5 = 0.1157;
//const double d6 = 0.0922;


void control_ur(CartesianSpaceTrackUR* controller_ur)
{
	std::ofstream for_loop_time("for_loop_time.txt", std::ios::trunc);//trunc
	boost::mutex::scoped_lock sl(mUpdate);
	cRun.wait(sl);
	boost::posix_time::ptime start_time = boost::get_system_time();
	controller_ur->start(start_time);
	double d_ms;
	float scale = 0.4;
	// the desired position specified by user or maybe the motion planner
	Eigen::Vector3d x_des;
	Eigen::Vector3d xd_des;
	Eigen::Vector3d xdd_des;

	double centerX = 0.35;
	double centerY = -0.1;
	double centerZ = 0.2;
	double radius = 0.1;
	boost::posix_time::ptime cur_time;
	boost::posix_time::time_duration d_time;
	//Eigen::AngleAxisd Rxyz_rotate_vector;
	Eigen::Matrix3d R_des;
	Eigen::Vector3d w_des;
	Eigen::Vector3d wd_des;

	double df_pos_orient[6];//目标位置和姿态XYZRPY
	while (running)
	{
		cur_time = boost::get_system_time();
		d_time = cur_time - start_time;
		d_ms = (double)d_time.total_milliseconds() / 1000;
		printf("time: %f\n", d_ms);

		//sep target position
		x_des(0) = -0.700;
		x_des(1) = -0.500;
		x_des(2) = 0.4;

		xd_des << 0, 0, 0;

		//xd_des << -scale * M_PI * radius * sin(scale * M_PI * d_ms), scale* M_PI* radius* cos(scale * M_PI * d_ms), 0;
		xdd_des << 0.0, 0.0, 0.0;

		//set target oritention
		df_pos_orient[3] = -3.099;//R ORIENT  -1.5708
		df_pos_orient[4] = -0.098;//P ORIENT   0.0
		df_pos_orient[5] = -0.535;//Y ORIENT  -3.1416
		//Eigen::Vector3d eulerAngle_start_point = Rc.eulerAngles(2, 1, 0);
		Eigen::Vector3d eulerAngle(df_pos_orient[3], df_pos_orient[4], df_pos_orient[5]);
		Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
		Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
		Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
		Eigen::AngleAxisd rotation_vector;
		rotation_vector = yawAngle * pitchAngle * rollAngle;//ZYX
		Eigen::Matrix3d orient_rotation_matrix = Eigen::Matrix3d::Identity();
		orient_rotation_matrix = rotation_vector.matrix();
		//li bo begen
		//Eigen::AngleAxisd Rxyz_rotate_vector(M_PI / 4, Eigen::Vector3d(0, 1, 0));
		//R_des.setIdentity();
		//R_des = Rxyz_rotate_vector.matrix();
		w_des.setZero();
		wd_des.setZero();
		//li bo end
		// 
		//设置笛卡儿坐标系x,y,z位置，速度，加速度
		controller_ur->set_desired_position(x_des, xd_des, xdd_des);
		//controller_ur2->set_desired_position(x_des, xd_des, xdd_des);
		//设置笛卡儿坐标系姿态位置，速度，加速度
		controller_ur->set_desired_orientation(orient_rotation_matrix, w_des, wd_des);
		//controller_ur->set_desired_orientation(R_des, w_des, wd_des);
		//update():读关节实际位置，关节实际速度；读关节目标位置
		//计算末端笛卡儿位姿，计算雅可比矩阵，求笛卡儿空间速度和姿态速度
		//最后求qcd（关节速度）

		controller_ur->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));

		//cur_time = boost::get_system_time();
		//d_time = cur_time - start_time;
		//d_ms = (double)d_time.total_milliseconds() / 1000;
		//for_loop_time << d_ms<<"   ";
		for (int i = 0; i < CONTROL_PERIOD_MS / UR_CYCLIC_TIME_MS; i++)
		{
			//设置关节速度和关节加速度
			controller_ur->specify_qcd((double)UR_CYCLIC_TIME_MS * (i + 1) / CONTROL_PERIOD_MS);
			cRun.wait(sl);
		}
		Eigen::VectorXd actualpose= controller_ur->get_tool_pose();
		if (abs(x_des(0)- actualpose(3))<0.003 && abs(x_des(1) - actualpose(4)) < 0.003 && abs(x_des(2) - actualpose(5)) < 0.003)
		inputnum = 51;

	}

	boost::posix_time::ptime stop_time = boost::get_system_time();
	controller_ur->stop(stop_time);
	for_loop_time.close();
}

void control_ur2(CartesianSpaceTrackUR* controller_ur, CartesianSpaceTrackUR* controller_ur2)
{
	std::ofstream for_loop_time("for_loop_time.txt", std::ios::trunc);//trunc
	bool after_contact = false;
	double duration;//以下预设值重要
	controller_ur->Spring_k = 0;//默认0.01
	controller_ur2->Spring_k = 0.001;// 0.003;//默认0.01
	controller_ur->Kp = 0.0;
	controller_ur2->Kp =1.0;
	controller_ur->force_controle_mode = false;
	controller_ur2->force_controle_mode = true;
	controller_ur2->tool_force_preset_(0) = 50;//50N,记得改回来
	controller_ur2->tool_force_preset_(1) = 0;
	controller_ur2->tool_force_preset_(2) = 0;
	std::ofstream pos_record("pos_record.txt", std::ios::trunc);//trunc
	std::ofstream R_force("R_force.txt", std::ios::trunc);//trunc
	std::ofstream L_force("L_force.txt", std::ios::trunc);//trunc
	boost::mutex::scoped_lock sl(mUpdate), s2(mUpdate2);
	cRun.wait(sl);
	cRun2.wait(s2);
	//time_point<high_resolution_clock> _start ;
	boost::posix_time::ptime start_time =  boost::posix_time::microsec_clock::local_time();// boost::get_system_time();
	//cout << start_time << endl;
	controller_ur->start(start_time);
	double d_ms, cirtime_ms;
	float scale = 0.4;
	// the desired position specified by user or maybe the motion planner
	Eigen::Vector3d x_des, x_des2;
	Eigen::Vector3d xd_des, xd_des2;
	Eigen::Vector3d xdd_des,xdd_des2;

	time_point<high_resolution_clock> chrono_begin;
	boost::posix_time::ptime cur_time,circle_end_time;
	boost::posix_time::time_duration d_time,circle_time;
	//Eigen::AngleAxisd Rxyz_rotate_vector;
	Eigen::Matrix3d R_des,		R_des2;
	Eigen::Vector3d w_des,		w_des2;
	Eigen::Vector3d wd_des,		wd_des2;
	Eigen::Matrix<double, 4, 4>  rob_left_base_to_rob_right_base;//左臂相对于右臂的旋转平移矩阵
	Eigen::Matrix<double, 4, 4>  target_center_to_rob_right_tool;//目标中心相对于右臂的旋转平移矩阵
	Eigen::Matrix<double, 4, 4>  target_center_to_rob_left_tool;//目标中心相对于右臂的旋转平移矩阵
	Eigen::Matrix<double, 4, 4>  T_right_rob_tool_to_base;//左臂相对于右臂的旋转平移矩阵
	//rob_left_base_to_rob_right_base <<  cos(-M_PI/13), -sin(-M_PI / 13), 0, -2,//-1时，物体沿着基坐标系X轴正向搬运。-2，物体沿着基坐标系Y轴负向搬运
	//									sin(-M_PI / 13), cos(-M_PI / 13), 0, 0.2,
	//									0, 0, 1, 0,
	//									0, 0, 0, 1;

	rob_left_base_to_rob_right_base << 0.999738, 0.0128716, 0.0189068, -2.00533,
		- 0.0129267, 0.999913, 0.0027971, -0.0797245,
		- 0.0188692, -0.00304077, 0.999817, 0.0170871,
		0, 0, 0, 1;

	//0.999738   0.0128716   0.0189068 - 2.00533
	//	- 0.0129267    0.999913   0.0027971 - 0.0797245
	//	- 0.0188692 - 0.00304077    0.999817   0.0170871
	//	0           0           0           1
	
	//cout << rob_left_base_to_rob_right_base(0, 3);
	target_center_to_rob_right_tool <<  1, 0, 0, 0,
										0, 1, 0, 0,
										0, 0, 1, 0.155,//0, 0, 1, 1
										0, 0, 0, 1;
	target_center_to_rob_left_tool <<   -1, 0, 0, 0,
										0, 1, 0, 0,
										0, 0, -1, 0.155,//0, 0, 1, 1
										0, 0, 0, 1;
	//cout << rob_left_base_to_rob_right_base << endl;
	double df_pos_orient[6];//目标位置和姿态XYZRPY

	controller_ur->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
	controller_ur2->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
	//Sleep(100);
	while (running)
	{
#ifdef	ARM_calib
		{
			Eigen::Matrix<double, 4, 4> M_Trans = controller_ur->Ttool_to_base;//A
			Eigen::Matrix<double, 4, 4> S_Trans = controller_ur2->Ttool_to_base;//B
			Eigen::Matrix<double, 4, 4> Tool_Trans;//tool B  to tool A
			Tool_Trans << -1.0, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1;//C
			Eigen::Matrix<double, 4, 4> M_TO_S_Trans;
			M_TO_S_Trans = S_Trans * Tool_Trans * M_Trans.inverse();
			cout << controller_ur->Ttool_to_base << endl;
			cout << controller_ur2->Ttool_to_base << endl;
			cout << M_TO_S_Trans << endl;
		}
#endif
		//_start = high_resolution_clock::now();
		cur_time = boost::posix_time::microsec_clock::local_time();
		//cur_time = boost::get_system_time();
		d_time = cur_time - start_time;
		d_ms = (double)d_time.total_milliseconds() / 1000;
		printf("time: %f\n", d_ms);
		for_loop_time << d_ms << endl;
		duration = 0.0;
		if (controller_ur2->tool_force_[0] - controller_ur2->tool_force_preset_(0) >0.0)
			after_contact = true;
		//工具坐标系下力传感器力位混采用speedL指令时增益设置
		//if (abs(controller_ur2->tcp_speed_[1]) > 0.005 and after_contact)//(controller_ur2->tool_force_preset_(0) -controller_ur2->tool_force_[0] ) > 5
		//{
		//	if ((controller_ur2->tool_force_preset_(0) - controller_ur2->tool_force_[0]) > 3)
		//		controller_ur2->Spring_k = 0.003;//0.005
		//	else if ((controller_ur2->tool_force_[0] - controller_ur2->tool_force_preset_(0) ) > 3)
		//		controller_ur2->Spring_k = 0.003;//0.005
		//	else                              //0.005
		//		controller_ur2->Spring_k = min(0.0001, abs((controller_ur2->tool_force_preset_(0) - controller_ur2->tool_force_[0])) * 0.001);
		//}
		//else
		//	controller_ur2->Spring_k = 0.0001;//0.0005   0.0015


		//基座系下力传感器力位混采用speedj指令时增益设置
		if (abs(controller_ur2->tcp_speed_[1]) > 0.005 and after_contact)//(controller_ur2->tool_force_preset_(0) -controller_ur2->tool_force_[0] ) > 5
		{
			if ((controller_ur2->tool_force_preset_(0) - controller_ur2->tool_force_[0]) > 3)
				controller_ur2->Spring_k = 0.005;////0.005
			else if ((controller_ur2->tool_force_[0] - controller_ur2->tool_force_preset_(0)) > 3)
				controller_ur2->Spring_k = 0.005;//0.005
			else                              
				controller_ur2->Spring_k = min(0.003, abs((controller_ur2->tool_force_preset_(0) - controller_ur2->tool_force_[0])) * 0.001);
		}
		else
			controller_ur2->Spring_k = 0.0005;//0.0005
		{
			if (d_ms < 10)//原来是10，调试可以改大10000用于测试
			{
				x_des(0) = 0.8;//9楼0.7 ，间距1900；2楼0.8,间距2000
				x_des(1) = 0.5;//0.5 
				x_des(2) = 0.1;//0.1
			}
			else if (d_ms >= 10 && d_ms < 14)//提升4S
			{
				x_des(2) = min(0.15, x_des(2) + 0.0001);
			}
			else if (d_ms >= 14 && d_ms < 26)//平移12S
			{
				x_des(1) = max(-0.20, x_des(1) - 0.00015 * (!bCrash));//*bCrash
			}
			else  if (d_ms >= 26 && d_ms < 31)//降落5S
			{
				x_des(2) = max(0.10, x_des(2) - 0.0001);
			}

			else if (d_ms >= 31 && d_ms < 36)//提升5S
			{
				x_des(2) = min(0.15, x_des(2) + 0.0001);
			}
			else if (d_ms >= 36 && d_ms < 48)//平移12S
			{
				x_des(1) = min(0.5, x_des(1) + 0.00015);
			}
			else if (d_ms >= 48 && d_ms < 53)//降落5S
			{
				x_des(2) = max(0.10, x_des(2) - 0.0001);
			}
			else
				start_time = cur_time ;
	/*		boost::posix_time::to_simple_string();
			boost::posix_time::time_from_string();*/
		}

		//IP 98为左臂,实际机器人
		T_right_rob_tool_to_base= rob_left_base_to_rob_right_base* controller_ur->Ttool_to_base* target_center_to_rob_left_tool*
			target_center_to_rob_right_tool.inverse();//34
		//cout << controller_ur2->Ttool_to_base << endl;
		//实际计算：右臂相对于右臂基座系的位姿关系
		x_des2(0) = T_right_rob_tool_to_base(0, 3);
		x_des2(1) = T_right_rob_tool_to_base(1, 3);
		x_des2(2) = T_right_rob_tool_to_base(2, 3);
		//cout << "x_des2(0):"<<x_des2(0) <<"   x_des2(1):" << x_des2(1) << "   x_des2(2):" << x_des2(2) << endl;
		xd_des << 0, 0, 0;
		//xd_des2 << 0, 0, 0;
		xd_des2 << controller_ur->tcp_speed_[0], controller_ur->tcp_speed_[1], controller_ur->tcp_speed_[2];
		//xd_des << -scale * M_PI * radius * sin(scale * M_PI * d_ms), scale* M_PI* radius* cos(scale * M_PI * d_ms), 0;
		xdd_des << 0.0, 0.0, 0.0;
		xdd_des2 << 0.0, 0.0, 0.0;
		
		//set target oritention
		df_pos_orient[3] = 1.55;	//R ORIENT   3.141593     1.569
		df_pos_orient[4] = -0.013;			//P ORIENT   0.0		0
		df_pos_orient[5] = 1.574;	//Y ORIENT	 3.141593      1.883
		//Eigen::Vector3d eulerAngle_start_point = Rc.eulerAngles(2, 1, 0);
		Eigen::Vector3d eulerAngle(df_pos_orient[3], df_pos_orient[4], df_pos_orient[5]);
		Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
		Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
		Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
		Eigen::AngleAxisd rotation_vector;
		rotation_vector = yawAngle * pitchAngle * rollAngle;//ZYX
		//cout << "rotation_vector=" << rotation_vector(0,0) << endl;
		Eigen::Matrix3d orient_rotation_matrix = Eigen::Matrix3d::Identity();
		orient_rotation_matrix = rotation_vector.matrix();
	
		//cout << "eulerAngle1" << orient_rotation_matrix.eulerAngles(2, 1, 0) << endl;

		//初始化从机欧拉角(Z - Y - X，即RPY)
		//Eigen::Vector3d eulerAngle2(1.777,-0.011, -1.45);//yaw, pitch, roll
		//// 欧拉角转旋转向量
		//Eigen::AngleAxisd rollAngle2(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitX()));
		//Eigen::AngleAxisd pitchAngle2(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
		//Eigen::AngleAxisd yawAngle2(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitZ()));
		//Eigen::AngleAxisd rotation_vector2; 
		//rotation_vector2 = yawAngle2 * pitchAngle2 * rollAngle2;

		////欧拉角转旋转矩阵
		//Eigen::Matrix3d orient_rotation_matrix2; 
		//orient_rotation_matrix2 = rotation_vector2.matrix();
		////Eigen::Vector3d eulerAngle3 = orient_rotation_matrix2.eulerAngles(2, 1, 0);


		Eigen::Matrix3d orient_rotation_matrix2= T_right_rob_tool_to_base.block<3,3>(0,0);//不能删
		//Eigen::Matrix3d orient_rotation_matrix2 << 0.271011, 0.141474, -0.952123, -0.86605, -0.395889, -0.305335, -0.420132, 0.907335, 0.0152332;
		Eigen::Vector3d eulerAngle3 = orient_rotation_matrix2.eulerAngles(2, 1, 0);
		//cout << "orient_rotation_matrix2" << endl;

		w_des.setZero();
		wd_des.setZero();
		//li bo end
		// 
		//设置笛卡儿坐标系x,y,z位置，速度，加速度
		controller_ur->set_desired_position(x_des, xd_des, xdd_des);
		controller_ur2->set_desired_position(x_des2, xd_des2, xdd_des2);
		//controller_ur2->set_desired_position(x_des, xd_des, xdd_des);
		//设置笛卡儿坐标系姿态位置，速度，加速度
		controller_ur->set_desired_orientation(orient_rotation_matrix, w_des, wd_des);
		controller_ur2->set_desired_orientation(orient_rotation_matrix2, w_des, wd_des);
		//controller_ur->set_desired_orientation(R_des, w_des, wd_des);
		//update():读关节实际位置，关节实际速度；读关节目标位置
		//计算末端笛卡儿位姿，计算雅可比矩阵，求笛卡儿空间速度和姿态速度
		//最后求qcd（关节速度）

		controller_ur->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
		controller_ur2->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
		pos_record << controller_ur->x.transpose() <<"   " << ((rob_left_base_to_rob_right_base.inverse() * controller_ur2->Ttool_to_base).block<3, 1>(0, 3)).transpose() << endl;
		//cout	   << controller_ur->x.transpose() <<"   " << ((rob_left_base_to_rob_right_base.inverse() * controller_ur2->Ttool_to_base).block<3, 1>(0, 3)).transpose() << endl;
		std::vector<double> tcp_force_torque =controller_ur2->get_tcp_force_tool(controller_ur2->tool_force_);
		for (int mk = 0; mk < 6; mk++)
		{
			R_force << controller_ur2->tool_force_[mk]<< "  " << tcp_force_torque[mk] << " ";
			L_force << controller_ur->tool_force_[mk] << "  ";
		}
		R_force << endl;
		L_force << endl;
		
		
		//cur_time = boost::posix_time::microsec_clock::local_time();
		//d_time = cur_time - start_time;
		//d_ms = (double)d_time.total_milliseconds() / 1000;
		//cout << "next time:= " << d_ms << endl;


		for (int i = 0; i < CONTROL_PERIOD_MS / UR_CYCLIC_TIME_MS; i++)
		{
			//设置关节速度和关节加速度
			controller_ur->specify_qcd((double)UR_CYCLIC_TIME_MS * (i + 1) / CONTROL_PERIOD_MS);
			controller_ur2->specify_qcd((double)UR_CYCLIC_TIME_MS * (i + 1) / CONTROL_PERIOD_MS);
			cRun.wait(sl);
			//cRun2.wait(s2);
		}
		do
		{
			circle_end_time = boost::posix_time::microsec_clock::local_time(); //boost::get_system_time();
			circle_time = circle_end_time - cur_time;
			cirtime_ms = (double)circle_time.total_milliseconds() / 1000;
			//cout << cirtime_ms << endl;
		} while (cirtime_ms < 0.001);//0.007  从循环开始到这里0.056S，

		//do
		//{
		//	duration =duration_cast<microseconds>(high_resolution_clock::now() - _start).count()*0.001;
		//} while (duration < 5.90);
		//cout <<"duration:="<< duration << endl;
		//for_loop_time << duration << endl;
	}

	boost::posix_time::ptime stop_time =  boost::get_system_time();
	controller_ur->stop(stop_time);
	pos_record.close();
	for_loop_time.close();
}



void control_ur_pcontrol(CartesianSpaceTrackUR* controller_ur)
{
	boost::mutex::scoped_lock sl(mUpdate);
	cRun.wait(sl);
	boost::posix_time::ptime start_time = boost::get_system_time();
	controller_ur->start(start_time);
	double d_ms, cirtime_ms;
	float scale = 0.4;
	// the desired position specified by user or maybe the motion planner
	Eigen::Vector3d x_des;
	Eigen::Vector3d xd_des;
	Eigen::Vector3d xdd_des;

	boost::posix_time::ptime cur_time, circle_end_time;
	boost::posix_time::time_duration d_time, circle_time;
	//Eigen::AngleAxisd Rxyz_rotate_vector;
	Eigen::Matrix3d R_des;
	Eigen::Vector3d w_des;
	Eigen::Vector3d wd_des;

	double df_pos_orient[6];//目标位置和姿态XYZRPY
	bool have_solution = false, run_robot=false;
	double desire_q[6];
	x_des(0) = 0.5;
	x_des(1) = 0.5;
	x_des(2) = 0.5;
	//x_des(0) = 0.5;
	//x_des(1) = -0.5;
	//x_des(2) = 0.42;
	while (running)
	{
		cur_time = boost::get_system_time();
		d_time = cur_time - start_time;
		d_ms = (double)d_time.total_milliseconds() / 1000;
		printf("time: %f\n", d_ms);

		//sep target position
		//if ((d_ms > 1.552 and d_ms < 1.570) || (d_ms > 2.552 and d_ms < 2.570) ||(d_ms > 3.552 and d_ms < 3.570)
		//	|| (d_ms > 4.552 and d_ms < 4.570) || (d_ms > 5.552 and d_ms < 5.570) || (d_ms > 6.552 and d_ms < 6.570)
		//	|| (d_ms > 7.552 and d_ms < 7.570) )
		//{
		//	x_des(0) -= 0.1;
		//	cout << "*****************************************"<< endl;
		//	run_robot = false;
		//}
		Eigen::VectorXd target = controller_ur->get_tool_pose();
		x_des(0) = target(3);
		x_des(1) = target(4);
		x_des(2) = target(5);


		xd_des << 0, 0, 0;
		//xd_des << -scale * M_PI * radius * sin(scale * M_PI * d_ms), scale* M_PI* radius* cos(scale * M_PI * d_ms), 0;
		xdd_des << 0.0, 0.0, 0.0;

		//set target oritention
		df_pos_orient[3] = 3.14159;//R ORIENT  -1.5708    -3.099
		df_pos_orient[4] = 0.0;//P ORIENT   0.0   -0.098
		df_pos_orient[5] = 1.57;//Y ORIENT  -3.1416   -0.535
		//Eigen::Vector3d eulerAngle_start_point = Rc.eulerAngles(2, 1, 0);
		Eigen::Vector3d eulerAngle(df_pos_orient[3], df_pos_orient[4], df_pos_orient[5]);
		Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
		Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
		Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
		Eigen::AngleAxisd rotation_vector;
		rotation_vector = yawAngle * pitchAngle * rollAngle;//ZYX
		Eigen::Matrix3d orient_rotation_matrix = Eigen::Matrix3d::Identity();
		orient_rotation_matrix = rotation_vector.matrix();
		Eigen::Matrix<double, 4, 4> des_Matrix= Eigen::Matrix4d::Identity();
		des_Matrix.block<3, 3>(0, 0) = orient_rotation_matrix;
		des_Matrix.block<3, 1>(0, 3) = x_des;
		cout << des_Matrix << endl;
		have_solution = controller_ur->ik_with_q(des_Matrix, desire_q);
		if (!have_solution)
		{
			cout << "无逆解..............." << endl;
			continue;
		}
			
		w_des.setZero();
		wd_des.setZero();

		for (int i = 0; i <6; i++)
		{
			controller_ur->q_pos_target[i] = desire_q[i];
			//controller_ur->q_pos_target[i] = x_des[i];
			//controller_ur->q_pos_target[3+i] = df_pos_orient[3+i];
			cout << setw(12)<< controller_ur->q_pos_target[i];
			//cout << setw(12) << controller_ur->q_pos_target[3+i];
		}
		cout  << endl;
		//设置目标关节位置
		controller_ur->set_Qdesired_position(desire_q);
		
		//li bo end
		//ik_with_q(des_Matrix);
		//设置笛卡儿坐标系x,y,z位置，速度，加速度
		
		//controller_ur->set_desired_position(x_des, xd_des, xdd_des);
		//设置笛卡儿坐标系姿态位置，速度，加速度
		//controller_ur->set_desired_orientation(orient_rotation_matrix, w_des, wd_des);
		//controller_ur->set_desired_orientation(R_des, w_des, wd_des);
		//update():读关节实际位置，关节实际速度；读关节目标位置
		//计算末端笛卡儿位姿，计算雅可比矩阵，求笛卡儿空间速度和姿态速度
		//最后求qcd（关节速度）
		controller_ur->update_p(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
		
		for (int i = 0; i < CONTROL_PERIOD_MS / UR_CYCLIC_TIME_MS; i++)
		//if (!run_robot)
		{
			//设置关节速度和关节加速度
			controller_ur->specify_qcd( (double)UR_CYCLIC_TIME_MS * (i + 1) / CONTROL_PERIOD_MS);
			cRun.wait(sl);
			run_robot = true;
		}
		//do
		//{
		//	circle_end_time = boost::get_system_time();
		//	circle_time = circle_end_time - cur_time;
		//	cirtime_ms = (double)circle_time.total_milliseconds() / 1000;
		//} while (cirtime_ms < 0.107);//0.007
		//run_robot = false;
	}

	boost::posix_time::ptime stop_time = boost::get_system_time();
	controller_ur->stop(stop_time);
}


void cartesion_control_ur(CartesianSpaceTrackUR* controller_ur)
{
	double qa[6] = { 0,-M_PI / 2,0,-M_PI / 2, 0, 0 }, qb[6] = { -M_PI / 3,-M_PI / 3,M_PI / 3,-M_PI / 2, 0,0 }, qc[6], qd[6];//q为关节角度
	double q_soltemp[6], q_last_circle[6] = { 0 };
	double ta[4][4], tb[4][4], tc[4][4], td[4][4];

	double* Qa = &qa[0], * Qb = &qb[0], * Qc = &qc[0], * Qd = &qd[0];// = &qc[0]; //* Ta = &ta[0][0], * Tb = &tb[0][0];

	Eigen::Matrix<double, 4, 4>  Ta, Tb, Tc, Td;
	Eigen::Matrix<double, 3, 3>  Ra, Rb, Rc, Rd;
	Eigen::Vector3d  Va, Vb, Vc, Vd;

	double T_temp[4][4]; double* T_Temp = &T_temp[0][0];
	double* test_temp;
	double* test_temp_old;
	double qq = 0.0;// double* Q = &q[0];
	double s = 0.1;
	Eigen::Matrix3d R5;
	double p1[3], p2[3], p_temp[3];
	double q_sols[48], q_deleate_sols[48];
	int num_sols, num_remained_sols;

	boost::mutex::scoped_lock sl(mUpdate);
	cRun.wait(sl);
	boost::posix_time::ptime start_time = boost::get_system_time();
	controller_ur->start(start_time);
	double d_ms,cirtime_ms;
	float scale = 0.4;
	// the desired position specified by user or maybe the motion planner
	Eigen::Vector3d x_des;
	Eigen::Vector3d xd_des;
	Eigen::Vector3d xdd_des;


	boost::posix_time::ptime cur_time,circle_end_time;
	boost::posix_time::time_duration d_time,circle_time;
	//Eigen::AngleAxisd Rxyz_rotate_vector;
	Eigen::Matrix3d R_des;
	Eigen::Vector3d w_des;
	Eigen::Vector3d wd_des;
	int itotaltime = 0;//10s finish motion ,40MS一个周期 400
	int icounter = 0;
	float fincrease = 0.0;
	double time_increase = 0.0;
	double df_run_total_time = 5.0;//单次规划运动时间
	//double df_i_run_total_time = 4;//单次规划运动时间
	bool b_running = false;
	double df_array_q_act_post[6];
	double df_array_q_choosed_pos[6];
	double df_temp;

	int i_input = 0;
	Eigen::VectorXd vct_cur_q;
	double df_two_point_distance = 0;
	double df_run_speed = 0.4;
	double df_two_point_ori_distance = 0;
	double df_run_ori_speed = 2.0;

	double df_rnd_x = 0.0;
	double df_rnd_y = 0.0;
	double df_rnd_z = 0.5;
	double df_rnd_min = 0.4;
	double df_rnd_max = 1.0;
	const int point_per_second = 125;
	const int  N = 100; //精度为小数点后面2位
	double df_pos_orient[6];//目标位置和姿态XYZRPY
	double df_previous_pos[3] = { 0.0 };//0:x,1:y,2:z
	//Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0, 0, 1));
	for (int i = 0; i < 6; i++)
	{
		qd[i] = qb[i];
	}
	//df_two_point_distance = sqrt(pow(abs(Vc[0] - Vd[0]), 2) + pow(abs(Vc[1] - Vd[1]), 2)
	//	+ pow(abs(Vc[2] - Vd[2]), 2));
	//itotaltime = (df_two_point_distance / df_run_speed) * 125;//距离除以速度=时间,每秒有125次循环(点),时间乘125,等于点数
	double* q_pos_sequence = nullptr;// = new double[6 * itotaltime + 6];
	double* q_speed_sequence = nullptr;
	double* q_acc_sequence = nullptr;
	while (running)
	{
		//i_input++;
		//关节插值结束
		controller_ur->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
		vct_cur_q = controller_ur->get_q();
		//cout << "vct_cur_q= " << vct_cur_q<< endl;
		//cout << "qc" << endl;
		qc[0] = controller_ur->q1;
		qc[1] = controller_ur->q2;
		qc[2] = controller_ur->q3;
		qc[3] = controller_ur->q4;
		qc[4] = controller_ur->q5;
		qc[5] = controller_ur->q6;

		//cout << "-------------------------------------------" << endl;
		//cout << controller_ur->get_tool_pose() << endl;
		//for (int i = 0; i < 6; i++)
		//{
		//	//qc[i] = vct_cur_q[i];
		//	cout << setw(12) << qc[i];
		//}
		//cout << endl;
		for (int i = 0; i < 6; i++)
		{
			q_last_circle[i] = qc[i];
			//qd[i] = qb[i];
		}


		controller_ur->fkRobot(Qa, Ta, Ra, Va);
		controller_ur->fkRobot(Qb, Tb, Rb, Vb);
		controller_ur->fkRobot(Qc, Tc, Rc, Vc);
		controller_ur->fkRobot(Qd, Td, Rd, Vd);

		ofstream read_pos("read_pos.txt", ios::trunc);
		ofstream set_pos("set_pos.txt", ios::app);
		ofstream total_point("total_point.txt", ios::trunc);
		//ofstream set_pos("set_pos.txt", ios::trunc);
		std::ofstream act_xyz_pos("act_xyz_pos.csv", std::ios::trunc);//trunc
		cout << "input the b_running" << endl;
		srand(time(NULL));//设置随机数种子，使每次产生的随机序列不同
		df_rnd_x = df_rnd_min + (df_rnd_max - df_rnd_min) * (rand() % N) / N;
		df_rnd_y = df_rnd_min + (df_rnd_max - df_rnd_min) * (rand() % N) / N ;//-0.5
		//df_rnd_x = 0;
		//df_rnd_y = -0.19425;
		df_pos_orient[0] = df_rnd_x;//X POS -0.275
		df_pos_orient[1] = df_rnd_y;//Y POS -0.2625
		df_pos_orient[2] = df_rnd_z;//Z POS
		df_pos_orient[3] = -1.5708;//R ORIENT  -1.5708
		df_pos_orient[4] = 0.0;//P ORIENT   0.0
		df_pos_orient[5] = -3.1416;//Y ORIENT  -3.1416
		Eigen::Vector3d eulerAngle_start_point = Rc.eulerAngles(2,1,0);
		Eigen::Vector3d eulerAngle(df_pos_orient[3], df_pos_orient[4], df_pos_orient[5]);
		Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
		Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
		Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
		Eigen::AngleAxisd rotation_vector;
		rotation_vector = yawAngle * pitchAngle * rollAngle;//ZYX
		Eigen::Matrix3d orient_rotation_matrix = Eigen::Matrix3d::Identity();
		orient_rotation_matrix = rotation_vector.matrix();
		Td.setIdentity();
		Td.block<3, 3>(0, 0) = orient_rotation_matrix;
		Td(0, 3) = df_pos_orient[0];//x POS
		Td(1, 3) = df_pos_orient[1];//y POS
		Td(2, 3) = df_pos_orient[2];//z POS
		//cout << "tc= " << endl
		//	<< Tc << endl;
		//cout << "td= " << endl
		//	<< Td << endl;
		//cout<< eulerAngle_start_point(0) <<endl;
		//cout << eulerAngle_start_point(1) << endl;
		//cout << eulerAngle_start_point(2) << endl;

		if (!b_running)// || (sqrt(pow(abs(df_pos_orient[0] - df_previous_pos[0]), 2) + pow(abs(df_pos_orient[1] - df_previous_pos[1]), 2)>0.2)))//
		{
			Delay(1 * 2000);   //延时5秒  
			df_two_point_ori_distance = (max(abs(eulerAngle_start_point(0) - eulerAngle(0)), abs(eulerAngle_start_point(1) - eulerAngle(1))), abs(eulerAngle_start_point(2) - eulerAngle(2)));
			df_two_point_distance = sqrt(pow(abs(df_pos_orient[0] - Vd[0]), 2) + pow(abs(df_pos_orient[1] - Vd[1]), 2)
				+ pow(abs(Vc[2] - Vd[2]), 2));
			//itotaltime = 500;
			itotaltime = max((df_two_point_distance / df_run_speed), df_two_point_ori_distance/ df_run_ori_speed) * point_per_second;//距离除以速度=时间,每秒有125次循环(点),时间乘125,等于点数
			cout << "itotaltime= " << itotaltime << endl;
			total_point << setw(12) << itotaltime << endl;
			q_pos_sequence = new double[6 * itotaltime + 6];
			q_speed_sequence = new double[6 * itotaltime + 6];
			q_acc_sequence = new double[6 * itotaltime + 6];
			//if (df_two_point_distance >= 0.05)
			//{
			//bool b_generate_success = controller_ur->joint_gen_point_sequence(Tc, Td, q_pos_sequence, q_speed_sequence, q_acc_sequence, itotaltime);

			bool b_generate_success = controller_ur->Ctraj_gen_point_sequence(Tc, Td, q_pos_sequence, itotaltime);
			if (!b_generate_success)
			{
				cout << "生成轨迹失败!" << endl;
				itotaltime = -1;
			}
			else
			{
				icounter = 0;//0
				for(int mk =0;mk<3;mk++)
					df_previous_pos[mk] = df_pos_orient[mk];
			}
				
		}

		while ((icounter < itotaltime + 1))
		{
			b_running = true;
			cur_time = boost::get_system_time();
			d_time = cur_time - start_time;
			d_ms = (double)d_time.total_milliseconds() / 1000;
			printf("time: %f\n", d_ms);
			//fincrease =(float)icounter / itotaltime;
			fincrease = 1.0 <= ((float)icounter / itotaltime) ? 1.0 : ((float)icounter / itotaltime);
			time_increase = d_ms <= df_run_total_time ? d_ms : df_run_total_time;
			//cout << "q actul speed" << endl;
			//Eigen::VectorXd vceActQd;
			//vceActQd.resize(6);
			//vceActQd = controller_ur->get_qd();
			//cout << vceActQd << endl;
			//for (int mkk = 0; mkk < 6; mkk++)
			//{
			//	cout << vceActQd(mkk) << endl;
			//}

			//Eigen::VectorXd vxd_q_act_post = controller_ur->get_q();
			//double* df_q_act_post = vxd_q_act_post.transpose().data();

			//df_q_act_post[0] = controller_ur->q1;
			//df_q_act_post[1] = controller_ur->q2;
			//df_q_act_post[2] = controller_ur->q3;
			//df_q_act_post[3] = controller_ur->q4;
			//df_q_act_post[4] = controller_ur->q5;
			//df_q_act_post[5] = controller_ur->q6;
			//for (int mk = 0; mk < 6; mk++)
			//{
			//	qa[mk] = df_q_act_post[mk];
			//}
			//controller_ur->fkRobot(Qa, Ta, Ra, Va);
			//for (int mkk = 0; mkk < 3; mkk++)
			//{
			//	act_xyz_pos<<Va[mkk]<<",";
			//}
			//act_xyz_pos << endl;
			//controller_ur->solution_choose_total_diff(num_remained_sols, q_deleate_sols, q_last_circle, df_array_q_choosed_pos); // q_last_circle  df_q_act_post
			//controller_ur->solution_choose_sigle_axis_diff(num_remained_sols, q_deleate_sols, df_q_act_post, df_array_q_choosed_pos);
			//cout << "The choosed solutions ";
			for (int im = 0; im < 6; im++)
			{
				
				controller_ur->q_pos_target[im] = q_pos_sequence[im + icounter * 6];
				controller_ur->qcd[im] = q_speed_sequence[im + icounter * 6];
				if (abs(controller_ur->q_pos_target[im]) < ZERO_THRESH)
					controller_ur->q_pos_target[im] = 0;
				//cout << setw(10) << controller_ur->q_pos_target[im] << "  ";
			}

			//cout << endl;


			//将数据输出至out.txt文件中
			//for (int i = 0; i < 6; i++)
			//{
			//	set_pos << setw(12) << controller_ur->q_pos_target[i];
			//	read_pos << setw(12) << df_q_act_post[i];
			//};
			//set_pos << std::endl;
			//for (int i = 0; i < 6; i++)
			//{
			//	//set_pos << setw(12) << controller_ur->q_pos_target[i];
			//	set_pos << setw(12) << df_q_act_post[i];
			//};
			//set_pos << std::endl;
			//read_pos << std::endl;


			//cout << "The last solutions ";
			//for (int i = 0; i < 6; i++)
			//{
			//	cout << setw(10) << q_last_circle[i] << "  ";
			//}
			//cout << endl;

			//cout << "The actual solutions ";
			//for (int i = 0; i < 6; i++)
			//{
			//	cout << setw(10) << df_q_act_post[i] << "  ";
			//}
			//cout << endl;
			x_des(0) = p_temp[0];
			x_des(1) = p_temp[1];
			x_des(2) = p_temp[2];

			xd_des << 0, 0, 0;

			//xd_des << -scale * M_PI * radius * sin(scale * M_PI * d_ms), scale* M_PI* radius* cos(scale * M_PI * d_ms), 0;
			xdd_des << 0.0, 0.0, 0.0;

			//Eigen::AngleAxisd Rxyz_rotate_vector(M_PI / 4, Eigen::Vector3d(0, 1, 0));
			R_des.setIdentity();
			//R_des = R5.matrix();
			w_des.setZero();
			wd_des.setZero();
			Eigen::VectorXd q_received_data;

			q_received_data = controller_ur->get_tool_pose();
			//设置笛卡儿坐标系x,y,z位置，速度，加速度
			//controller_ur->set_desired_position(x_des, xd_des, xdd_des);
			//设置笛卡儿坐标系姿态位置，速度，加速度
			//controller_ur->set_desired_orientation(R_des, w_des, wd_des);
			//update():读关节实际位置，关节实际速度；读关节目标位置
			//计算末端笛卡儿位姿，计算雅可比矩阵，求笛卡儿空间速度和姿态速度
			//最后求要下发的的qcd（关节速度）
			controller_ur->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
			//for (int i = 0; i < CONTROL_PERIOD_MS / UR_CYCLIC_TIME_MS; i++)
			//{
					//设置关节速度和关节加速度
			controller_ur->specify_qcd((double)UR_CYCLIC_TIME_MS * (0 + 1) / CONTROL_PERIOD_MS);
			cRun.wait(sl);
			//}
			
			icounter++;
			for (int i = 0; i < 6; i++)
			{
				q_last_circle[i] = df_array_q_choosed_pos[i];
			}
			//cRun.wait(sl);
			do
			{
				circle_end_time = boost::get_system_time();
				circle_time = circle_end_time - cur_time;
				cirtime_ms = (double)circle_time.total_milliseconds() / 1000;
			} while (cirtime_ms < 0.007);//0.007

		}

		b_running = false;
		set_pos.close();
		read_pos.close();
		total_point.close();
		act_xyz_pos.close();
		//i_input++;
	}
	delete[] q_pos_sequence;
	boost::posix_time::ptime stop_time = boost::get_system_time();
	controller_ur->stop(stop_time);
}


void joint_space_control_ur(CartesianSpaceTrackUR* controller_ur)
{
	double qa[6] = { 0,-M_PI / 2, 0,-M_PI / 2, 0, 0 }, qb[6] = { M_PI / 3,-M_PI * 4 / 5, -M_PI * 1 / 3,-M_PI / 4,0, 0 };//q为关节角度
	double q_soltemp[6] = { 0 };
	double ta[4][4], tb[4][4];

	double* Qa = &qa[0], * Qb = &qb[0]; //* Ta = &ta[0][0], * Tb = &tb[0][0];

	Eigen::Matrix<double, 4, 4>  Ta, Tb;
	Eigen::Matrix<double, 3, 3>  Ra, Rb;
	Eigen::Vector3d  Va, Vb;
	controller_ur->fkRobot(Qa, Ta, Ra, Va); controller_ur->fkRobot(Qb, Tb, Rb, Vb);

	double T_temp[4][4]; double* T_Temp = &T_temp[0][0];
	double qq = 0.0;// double* Q = &q[0];
	double s = 0.1;
	Eigen::Matrix3d R5;
	double p1[3], p2[3], p_temp[3];
	double q_sols[48]; int num_sols;

	Eigen::Quaterniond quat1(Ra), quat2(Rb), quat_temp;

	boost::mutex::scoped_lock sl(mUpdate);
	cRun.wait(sl);
	boost::posix_time::ptime start_time = boost::get_system_time();
	controller_ur->start(start_time);
	double d_ms;
	float scale = 0.4;
	// the desired position specified by user or maybe the motion planner
	Eigen::Vector3d x_des;
	Eigen::Vector3d xd_des;
	Eigen::Vector3d xdd_des;

	boost::posix_time::ptime cur_time;
	boost::posix_time::time_duration d_time;
	//Eigen::AngleAxisd Rxyz_rotate_vector;
	Eigen::Matrix3d R_des;
	Eigen::Vector3d w_des;
	Eigen::Vector3d wd_des;
	int itotaltime = 500;//10s finish motion ,40MS一个周期
	int icounter = 0;
	double fincrease = 0.0;
	double time_increase = 0.0;
	double df_run_total_time = 5.0;//单次规划运动时间
	bool b_first_run = true;
	double df_array_q_act_post[6];
	double df_array_q_choosed_pos[6];
	//关节插值开始
	//1.关节空间只是插值，就用QA，QB分别设置位置，后插值，赋值给机器人。
	//2.关节空间先正解，再反解，再插值，赋值给机器人
	//关节空间插值时，反解一次
	//for(int)
	double dfArray[16];
	Eigen::Matrix<double, 4, 4> tb_trans;

	tb_trans = (Tb.transpose());
	T_Temp = tb_trans.data();//T_Temp

	//cout << "Tb= " << Tb << endl;
	//cout << "tb_trans=" << tb_trans << endl;
	num_sols = controller_ur->inverse(T_Temp, q_sols, qq);
	cout << "solutions=" << num_sols << endl;
	for (int i = 0; i < 48; i++)
	{
		if (0 == i % 6) cout << endl;
		cout << setw(12) << q_sols[i];
	}
	cout << endl;
	controller_ur->solution_choose_total_diff(num_sols, q_sols, qa, df_array_q_choosed_pos);
	cout << "start poing=";
	for (int i = 0; i < 6; ++i)
	{
		cout << setw(16) << qa[i];
	}
	cout << endl;
	cout << "input=";
	for (int i = 0; i < 6; ++i)
	{
		cout << setw(16) << qb[i];
	}
	cout << endl;
	cout << "output=";
	for (int i = 0; i < 6; ++i)
	{
		cout << setw(16) << df_array_q_choosed_pos[i];
	}
	cout << endl;
	//itotaltime =(int) (df_i_run_total_time / 0.008);
	//关节插值结束
	while (running)
	{
		cur_time = boost::get_system_time();
		d_time = cur_time - start_time;
		d_ms = (double)d_time.total_milliseconds() / 1000;
		printf("time: %f\n", d_ms);
		//fincrease =(float)icounter / itotaltime;
		fincrease = 1.0 <= ((double)icounter / itotaltime) ? 1.0 : ((double)icounter / itotaltime);
		time_increase = d_ms <= df_run_total_time ? d_ms : df_run_total_time;
		Eigen::VectorXd vxd_q_act_post = controller_ur->get_q();
		double* df_q_act_post = vxd_q_act_post.transpose().data();

		//关节空间插值	
		double* q_temp;

		for (int im = 0; im < 6; im++)
		{
			if (df_array_q_choosed_pos[im] <= M_PI)
				df_array_q_choosed_pos[im] = df_array_q_choosed_pos[im];
			else
				df_array_q_choosed_pos[im] = df_array_q_choosed_pos[im] - 2 * M_PI;
			//cout << df_array_q_choosed_pos[im] << endl;
		}
		//q_temp = controller_ur->joint_traj(&qa[0], &df_array_q_choosed_pos[0], fincrease);
		//q_temp = controller_ur->joint_space_3traj(&qa[0], &df_array_q_choosed_pos[0], time_increase, df_run_total_time);
		for (int i = 0; i < 6; i++)
		{
			q_soltemp[i] = q_temp[i];
			//cout << "The test q" << q_soltemp[i] << endl;
		}

		for (int im = 0; im < 6; im++)
		{
			controller_ur->q_pos_target[im] = q_soltemp[im];
		}

		//xd_des << -scale * M_PI * radius * sin(scale * M_PI * d_ms), scale* M_PI* radius* cos(scale * M_PI * d_ms), 0;
		xdd_des << 0.0, 0.0, 0.0;

		//Eigen::AngleAxisd Rxyz_rotate_vector(M_PI / 4, Eigen::Vector3d(0, 1, 0));
		R_des.setIdentity();
		//R_des = R5.matrix();
		w_des.setZero();
		wd_des.setZero();
		controller_ur->get_tool_pose();
		//设置笛卡儿坐标系x,y,z位置，速度，加速度
		controller_ur->set_desired_position(x_des, xd_des, xdd_des);
		//设置笛卡儿坐标系姿态位置，速度，加速度
		controller_ur->set_desired_orientation(R_des, w_des, wd_des);
		//update():读关节实际位置，关节实际速度；读关节目标位置
		//计算末端笛卡儿位姿，计算雅可比矩阵，求笛卡儿空间速度和姿态速度
		//最后求要下发的的qcd（关节速度）
		controller_ur->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
		//for (int i = 0; i < CONTROL_PERIOD_MS / UR_CYCLIC_TIME_MS; i++)
		//{
		//	if (abs(controller_ur->x(0) - Vb(0) > 0.001) || abs(controller_ur->x(1) - Vb(1) > 0.001) || abs(controller_ur->x(2) - Vb(2) > 0.001))
		//	{

				//设置关节速度和关节加速度
		controller_ur->specify_qcd((double)UR_CYCLIC_TIME_MS * (0 + 1) / CONTROL_PERIOD_MS);
		//	}
		cRun.wait(sl);
		//}
			//cRun.wait(sl);
		icounter++;

	}

	boost::posix_time::ptime stop_time = boost::get_system_time();
	controller_ur->stop(stop_time);
}


void ur_cal_and_run(CartesianSpaceTrackUR* controller_ur)
{
	double qa[6] = { 0,-M_PI / 2,0,-M_PI / 2, 0, 0 }, qb[6] = { -M_PI / 3,-M_PI / 3,M_PI / 3,-M_PI / 2, 0,0 }, qc[6], qd[6];//q为关节角度
	double q_soltemp[6], q_last_circle[6] = { 0 };
	double ta[4][4], tb[4][4], tc[4][4], td[4][4];

	double* Qa = &qa[0], * Qb = &qb[0], * Qc = &qc[0], * Qd = &qd[0];// = &qc[0]; //* Ta = &ta[0][0], * Tb = &tb[0][0];

	Eigen::Matrix<double, 4, 4>  Ta, Tb, Tc, Td;
	Eigen::Matrix<double, 3, 3>  Ra, Rb, Rc, Rd;
	Eigen::Vector3d  Va, Vb, Vc, Vd;

	double T_temp[4][4]; double* T_Temp = &T_temp[0][0];
	double* test_temp;
	double* test_temp_old;
	double qq = 0.0;// double* Q = &q[0];
	double s = 0.1;
	Eigen::Matrix3d R5;
	double p1[3], p2[3], p_temp[3];
	double q_sols[48], q_deleate_sols[48];
	int num_sols, num_remained_sols;


	boost::mutex::scoped_lock sl(mUpdate);
	cRun.wait(sl);
	boost::posix_time::ptime start_time = boost::get_system_time();
	controller_ur->start(start_time);
	double d_ms, cirtime_ms;
	float scale = 0.4;
	// the desired position specified by user or maybe the motion planner
	Eigen::Vector3d x_des;
	Eigen::Vector3d xd_des;
	Eigen::Vector3d xdd_des;

	boost::posix_time::ptime cur_time, circle_end_time;
	boost::posix_time::time_duration d_time, circle_time;

	//Eigen::AngleAxisd Rxyz_rotate_vector;
	Eigen::Matrix3d R_des;
	Eigen::Vector3d w_des;
	Eigen::Vector3d wd_des;
	int itotaltime = 500;//10s finish motion ,40MS一个周期 400
	int icounter = 0;
	float fincrease = 0.0;
	double time_increase = 0.0;
	double df_run_total_time = 5.0;//单次规划运动时间
	//double df_i_run_total_time = 4;//单次规划运动时间
	bool b_running = false;
	double df_array_q_act_post[6];
	double df_array_q_choosed_pos[6];
	double df_temp;

	int i_input = 0;
	Eigen::VectorXd vct_cur_q;
	double df_two_point_distance = 0;
	double df_run_speed = 0.3;
	double df_two_point_ori_distance = 0;
	double df_run_ori_speed = 2.0;

	double df_rnd_x = 0.5;
	double df_rnd_y = 0.5;
	double df_rnd_z = 0.5;
	double df_rnd_min = 0.4;
	double df_rnd_max = 0.8;
	#define N  60 //精度为小数点后面2位
	double df_pos_orient[6];//目标位置和姿态XYZRPY
	double df_previous_pos[3] = { 0.0 };//0:x,1:y,2:z
	//Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0, 0, 1));
	for (int i = 0; i < 6; i++)
	{
		qd[i] = qb[i];
	}

	double* q_pos_sequence = nullptr;// = new double[6 * itotaltime + 6];
	double* q_speed_sequence = nullptr;
	double* q_acc_sequence = nullptr;
	double a0_3[6], a1_3[6], a2_3[6], a3_3[6];//多项式的4个系数
	while (running)
	{
		cur_time = boost::get_system_time();
		d_time = cur_time - start_time;
		d_ms = (double)d_time.total_milliseconds() / 1000;
		printf("time: %f\n", d_ms);

		//set target position
		if ((d_ms > 1.552 and d_ms < 1.570) || (d_ms > 2.552 and d_ms < 2.570) ||(d_ms > 3.552 and d_ms < 3.570)
			|| (d_ms > 4.552 and d_ms < 4.570) || (d_ms > 5.552 and d_ms < 5.570) || (d_ms > 6.552 and d_ms < 6.570)
			|| (d_ms > 7.552 and d_ms < 7.570)  ||  (d_ms < 0.0170) )
		{
			df_rnd_x = df_rnd_min + (df_rnd_max - df_rnd_min) * (rand() % N) / N;
			df_rnd_y = df_rnd_min + (df_rnd_max - df_rnd_min) * (rand() % N) / N;//-0.5
			cout << "*****************************************"<< endl;
			//run_robot = false;
		}

		//i_input++;
		//关节插值结束
		controller_ur->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));

		//controller_ur->fkRobot(Qa, Ta, Ra, Va);
		//controller_ur->fkRobot(Qb, Tb, Rb, Vb);
		controller_ur->fkRobot(Qc, Tc, Rc, Vc);
		controller_ur->fkRobot(Qd, Td, Rd, Vd);

		ofstream read_pos("read_pos.txt", ios::trunc);
		//ofstream set_pos("set_pos.txt", ios::app);
		ofstream total_point("total_point.txt", ios::trunc);
		ofstream set_pos("set_pos.txt", ios::trunc);
		std::ofstream act_xyz_pos("act_xyz_pos.csv", std::ios::trunc);//trunc
		//cout << "input the b_running" << endl;
		srand(time(NULL));//设置随机数种子，使每次产生的随机序列不同
		//df_rnd_x = df_rnd_min + (df_rnd_max - df_rnd_min) * (rand() % N) / N;
		//df_rnd_y = df_rnd_min + (df_rnd_max - df_rnd_min) * (rand() % N) / N;//-0.5
		//df_rnd_x = 0.575;
		//df_rnd_y = 0.5625;
		set_pos << "df_rnd_x= " << df_rnd_x << endl;
		set_pos << "df_rnd_y= " << df_rnd_y << endl;
		df_pos_orient[0] = df_rnd_x;//X POS -0.275
		df_pos_orient[1] = df_rnd_y;//Y POS -0.2625
		df_pos_orient[2] = df_rnd_z;//Z POS
		df_pos_orient[3] = -1.5708;//R ORIENT   -1.5708
		df_pos_orient[4] = 0;//P ORIENT
		df_pos_orient[5] = -3.1416;//Y ORIENT
		Eigen::Vector3d eulerAngle_start_point = Rc.eulerAngles(2, 1, 0);
		Eigen::Vector3d eulerAngle(df_pos_orient[3], df_pos_orient[4], df_pos_orient[5]);
		Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
		Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
		Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
		Eigen::AngleAxisd rotation_vector;
		rotation_vector = yawAngle * pitchAngle * rollAngle;//ZYX
		Eigen::Matrix3d orient_rotation_matrix = Eigen::Matrix3d::Identity();
		orient_rotation_matrix = rotation_vector.matrix();

		Td.setIdentity();
		Td.block<3, 3>(0, 0) = orient_rotation_matrix;
		Td(0, 3) = df_pos_orient[0];//x POS
		Td(1, 3) = df_pos_orient[1];//y POS
		Td(2, 3) = df_pos_orient[2];//z POS
		for (int i = 0; i < 3; i++)
		{
			Vb(i) = (df_pos_orient[i] + Vc(i)) / 2;
		}
		Tb.setIdentity();
		Tb.block<3, 3>(0, 0) = orient_rotation_matrix;
		Tb(0, 3) = Vb(0);//x POS
		Tb(1, 3) = Vb(1);//y POS
		Tb(2, 3) = Vb(2);//z POS
		if ((df_pos_orient[0] != df_previous_pos[0]) || (df_pos_orient[1] != df_previous_pos[1]))//
		//if (/*!b_running || */(sqrt(pow(abs(df_pos_orient[0] - df_previous_pos[0]), 2) + pow(abs(df_pos_orient[1] - df_previous_pos[1]), 2) > 0.00001)))//
		{
			//Delay(1 * 4000);   //延时5秒  
			df_two_point_ori_distance = (max(abs(eulerAngle_start_point(0) - eulerAngle(0)), abs(eulerAngle_start_point(1) - eulerAngle(1))), abs(eulerAngle_start_point(2) - eulerAngle(2)));
			df_two_point_distance = sqrt(pow(abs(df_pos_orient[0] - Vd[0]), 2) + pow(abs(df_pos_orient[1] - Vd[1]), 2)
				+ pow(abs(Vc[2] - Vd[2]), 2));
			//itotaltime = 500;
			itotaltime = max((df_two_point_distance / df_run_speed), df_two_point_ori_distance / df_run_ori_speed) * 125;//距离除以速度=时间,每秒有125次循环(点),时间乘125,等于点数
			//cout << "itotaltime= " << itotaltime << endl;
			//total_point << setw(12) << itotaltime << endl;
			q_pos_sequence = new double[6 * itotaltime + 6];
			q_speed_sequence = new double[6 * itotaltime + 6];
			q_acc_sequence = new double[6 * itotaltime + 6];
			
			bool b_generate_success = controller_ur->joint_gen_point_sequence(Tc, Td, q_pos_sequence, q_speed_sequence, q_acc_sequence, itotaltime);
			//bool b_generate_success = controller_ur->Ctraj_gen_point_sequence(Tc, Td, q_pos_sequence, itotaltime);

			 cout << "***********************************************************" << endl;

			if (!b_generate_success)
			{
				cout << "生成轨迹失败!" << endl;
				itotaltime = -1;
			}
			else
			{
				icounter = 0;
				cout << "更新轨迹了" << endl;
				for (int mk = 0; mk < 3; mk++)
					df_previous_pos[mk] = df_pos_orient[mk];
			}
			cout << "***********************************************************" << endl;
		}

		//while ((icounter < itotaltime + 1))
		{
			//cout << "run circle!" << endl;
			b_running = true;
			//Eigen::VectorXd vxd_q_act_post = controller_ur->get_q();
			for (int im = 0; im < 6; im++)
			{
				//controller_ur->q_pos_target[im] = df_pos_orient[im];
				controller_ur->q_pos_target[im] = q_pos_sequence[im + icounter * 6];
				controller_ur->qcd[im] = q_speed_sequence[im + icounter * 6];
				if (abs(controller_ur->q_pos_target[im]) < ZERO_THRESH)
					controller_ur->q_pos_target[im] = 0;
			}
			controller_ur->specify_qcd((double)UR_CYCLIC_TIME_MS * (0 + 1) / CONTROL_PERIOD_MS);
			//cRun.wait(sl);
			icounter++;
			if (icounter >= itotaltime)
			{
				icounter = itotaltime;
				b_running = false;
			}
				
			for (int i = 0; i < 6; i++)
			{
				q_last_circle[i] = df_array_q_choosed_pos[i];
			}
			do
			{
				circle_end_time = boost::get_system_time();
				circle_time = circle_end_time - cur_time;
				cirtime_ms = (double)circle_time.total_milliseconds() / 1000;
				//cout << cirtime_ms << endl;
			} while (cirtime_ms < 0.007);//0.007  从循环开始到这里0.056S，
			cRun.wait(sl);
		}

		//b_running = false;
		set_pos.close();
		read_pos.close();
		total_point.close();
		act_xyz_pos.close();
		//i_input++;
	}
	delete[] q_pos_sequence;
	boost::posix_time::ptime stop_time = boost::get_system_time();
	controller_ur->stop(stop_time);
}



void cartesion_ur_cal_and_run(CartesianSpaceTrackUR* controller_ur)
{
	double qa[6] = { 0,-M_PI / 2,0,-M_PI / 2, 0, 0 }, qb[6] = { -M_PI / 3,-M_PI / 3,M_PI / 3,-M_PI / 2, 0,0 }, qc[6], qd[6];//q为关节角度
	double q_soltemp[6], q_last_circle[6] = { 0 };
	double ta[4][4], tb[4][4], tc[4][4], td[4][4];

	double* Qa = &qa[0], * Qb = &qb[0], * Qc = &qc[0], * Qd = &qd[0];// = &qc[0]; //* Ta = &ta[0][0], * Tb = &tb[0][0];

	Eigen::Matrix<double, 4, 4>  Ta, Tb, Tc, Td;
	Eigen::Matrix<double, 3, 3>  Ra, Rb, Rc, Rd;
	Eigen::Vector3d  Va, Vb, Vc, Vd;

	double T_temp[4][4]; double* T_Temp = &T_temp[0][0];
	double* test_temp;
	double* test_temp_old;
	double qq = 0.0;// double* Q = &q[0];
	double s = 0.1;
	Eigen::Matrix3d R5;
	double p1[3], p2[3], p_temp[3];
	double q_sols[48], q_deleate_sols[48];
	int num_sols, num_remained_sols;
	ofstream time_elapse("time_elapse.txt", ios::app);

	boost::mutex::scoped_lock sl(mUpdate);
	cRun.wait(sl);
	boost::posix_time::ptime start_time = boost::get_system_time();
	controller_ur->start(start_time);
	double d_ms, cirtime_ms;
	float scale = 0.4;
	// the desired position specified by user or maybe the motion planner
	Eigen::Vector3d x_des;
	Eigen::Vector3d xd_des;
	Eigen::Vector3d xdd_des;

	boost::posix_time::ptime cur_time, circle_end_time;
	boost::posix_time::time_duration d_time, circle_time;

	//Eigen::AngleAxisd Rxyz_rotate_vector;
	Eigen::Matrix3d R_des;
	Eigen::Vector3d w_des;
	Eigen::Vector3d wd_des;
	int itotaltime = 500;//10s finish motion ,40MS一个周期 400
	int icounter = 0;
	float fincrease = 0.0;
	double time_increase = 0.0,total_run_time=0.0;
	double df_run_total_time = 5.0;//单次规划运动时间
	//double df_i_run_total_time = 4;//单次规划运动时间
	bool b_running = false;
	double df_array_q_act_post[6];
	double df_array_q_choosed_pos[6];
	double df_temp;

	int i_input = 0;
	Eigen::VectorXd vct_cur_q;
	double df_two_point_distance = 0;
	double df_run_speed = 0.3;
	double df_two_point_ori_distance = 0;
	double df_run_ori_speed = 2.0;

	double df_rnd_x = 0.8;
	double df_rnd_y = -0.3;
	double df_rnd_z = 0.4;
	double df_rnd_min = 0.4;
	double df_rnd_max = 1.2;
	double send_time = 0.0;
	const int  nN = 100; //精度为小数点后面2位//精度为小数点后面2位
	double df_pos_orient[6];//目标位置和姿态XYZRPY
	double df_previous_pos[3] = { 0.0 };//0:x,1:y,2:z
	//Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0, 0, 1));
	for (int i = 0; i < 6; i++)
	{
		qd[i] = qb[i];
	}

	double* q_pos_sequence = nullptr;// = new double[6 * itotaltime + 6];
	double* q_speed_sequence = nullptr;
	double* q_acc_sequence = nullptr;
	double a0_3[6], a1_3[6], a2_3[6], a3_3[6];//多项式的4个系数
	icounter = 1;
	while (running)
	{
		//cur_time = boost::get_system_time();
		//d_time = cur_time - start_time;
		//d_ms = (double)d_time.total_milliseconds() / 1000;
		//printf("time: %f\n", d_ms);

		srand(time(NULL));//设置随机数种子，使每次产生的随机序列不同
		//set target position
		//if (d_ms > 1.5 )
		{
			//start_time = cur_time;
			df_rnd_x = df_rnd_min + (df_rnd_max - df_rnd_min) * (rand() % N) / N;
			df_rnd_y = df_rnd_min + (df_rnd_max - df_rnd_min) * (rand() % N) / N;//-0.5
			//cout << "*****************************************" << endl;
			//run_robot = false;
		}

		//i_input++;
		//关节插值结束
		controller_ur->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
		qc[0] = controller_ur->q1;
		qc[1] = controller_ur->q2;
		qc[2] = controller_ur->q3;
		qc[3] = controller_ur->q4;
		qc[4] = controller_ur->q5;
		qc[5] = controller_ur->q6;

		controller_ur->fkRobot(Qc, Tc, Rc, Vc);
		//controller_ur->fkRobot(Qd, Td, Rd, Vd);
		//std::cout << "tool_force :";
		for (int i = 0; i < 6; i++)
		{
			time_elapse << controller_ur->tool_force_[i] << "   ";
			//std::cout  << controller_ur->tool_force_[i] << endl;
		}
		//std::cout << endl;
		
		ofstream read_pos("read_pos.txt", ios::trunc);
		//ofstream set_pos("set_pos.txt", ios::app);
		ofstream total_point("total_point.txt", ios::trunc);
		ofstream set_pos("set_pos.txt", ios::trunc);
		std::ofstream act_xyz_pos("act_xyz_pos.txt", std::ios::trunc);//trunc
		//cout << "input the b_running" << endl;
		//srand(time(NULL));//设置随机数种子，使每次产生的随机序列不同
		//df_rnd_x = df_rnd_min + (df_rnd_max - df_rnd_min) * (rand() % nN) / nN;
		//df_rnd_y = df_rnd_min + (df_rnd_max - df_rnd_min) * (rand() % nN) / nN;//-0.5
		//df_rnd_x = 0.575;
		//df_rnd_y = 0.5625;
		set_pos << "df_rnd_x= " << df_rnd_x << endl;
		set_pos << "df_rnd_y= " << df_rnd_y << endl;
		df_pos_orient[0] = 0.5;//X POS -0.275
		df_pos_orient[1] = -0.5;//Y POS -0.2625
		df_pos_orient[2] = df_rnd_z;//Z POS
		df_pos_orient[3] = 2.9;//R ORIENT   -1.5708
		df_pos_orient[4] = 0;//P ORIENT
		df_pos_orient[5] = -1.2;//Y ORIENT
		Eigen::Vector3d eulerAngle_start_point = Rc.eulerAngles(2, 1, 0);
		Eigen::Vector3d eulerAngle(df_pos_orient[3], df_pos_orient[4], df_pos_orient[5]);
		Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
		Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
		Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
		Eigen::AngleAxisd rotation_vector;
		rotation_vector = yawAngle * pitchAngle * rollAngle;//ZYX
		Eigen::Matrix3d orient_rotation_matrix = Eigen::Matrix3d::Identity();
		orient_rotation_matrix = rotation_vector.matrix();

		Td.setIdentity();
		Td.block<3, 3>(0, 0) = orient_rotation_matrix;
		Td(0, 3) = df_pos_orient[0];//x POS
		Td(1, 3) = df_pos_orient[1];//y POS
		Td(2, 3) = df_pos_orient[2];//z POS
		cout << "++++++++++++++++++++++++++++++++++++++" << endl;
		for (int i = 0; i < 3; i++)
		{
			Vb(i) = (df_pos_orient[i] + Vc(i)) / 2;
		}
		Tb.setIdentity();
		Tb.block<3, 3>(0, 0) = orient_rotation_matrix;
		Tb(0, 3) = Vb(0);//x POS
		Tb(1, 3) = Vb(1);//y POS
		Tb(2, 3) = Vb(2);//z POS
		if ((df_pos_orient[0] != df_previous_pos[0]) || (df_pos_orient[1] != df_previous_pos[1]))//
		//if (/*!b_running || */(sqrt(pow(abs(df_pos_orient[0] - df_previous_pos[0]), 2) + pow(abs(df_pos_orient[1] - df_previous_pos[1]), 2) > 0.00001)))//
		{
			//Delay(1 * 4000);   //延时5秒  
			df_two_point_ori_distance = 0; //(max(abs(eulerAngle_start_point(0) - eulerAngle(0)), abs(eulerAngle_start_point(1) - eulerAngle(1))), abs(eulerAngle_start_point(2) - eulerAngle(2)));
			df_two_point_distance = sqrt(pow(abs(df_pos_orient[0] - controller_ur->x(0)), 2) + pow(abs(df_pos_orient[1] - controller_ur->x(1)), 2)
				+ pow(abs(df_pos_orient[2] - controller_ur->x(2)), 2));
			//itotaltime = 500;
		}
			itotaltime = max((df_two_point_distance / df_run_speed), df_two_point_ori_distance / df_run_ori_speed) * 500;//距离除以速度=时间,每秒有125次循环(点),时间乘125,等于点数
			//total_run_time = max((df_two_point_distance / df_run_speed), df_two_point_ori_distance / df_run_ori_speed) ;//距离除以速度=时间,每秒有125次循环(点),时间乘125,等于点数
			total_run_time = 3;
			
			//cout << "itotaltime= " << itotaltime << endl;
			//cout << "total_run_time= " << total_run_time << endl;
			//total_point << setw(12) << itotaltime << endl;
		
			q_pos_sequence = new double[6 * itotaltime + 6];
			q_speed_sequence = new double[6 * itotaltime + 6];
			q_acc_sequence = new double[6 * itotaltime + 6];

			cur_time = boost::get_system_time();
			d_time = cur_time - start_time;
			d_ms = (double)d_time.total_milliseconds() / 1000;
			printf("time: %f\n", d_ms);
			time_elapse << d_ms << endl;
			send_time = d_ms /total_run_time;
			if (send_time > 1.0)
				send_time = 1.0;
			//cout <<"send_time:=" <<send_time << endl;
			//bool b_generate_success = controller_ur->joint_gen_point_sequence(Tc, Td, q_pos_sequence, q_speed_sequence, q_acc_sequence, itotaltime);
			bool b_generate_success = controller_ur->Ctraj_gen_step_point_sequence(Tc, Td, q_pos_sequence, icounter, itotaltime, send_time);
			//bool b_generate_success = controller_ur->Ctraj_gen_point_sequence(Tc, Td, q_pos_sequence, itotaltime);
			//cout << "***********************************************************" << endl;

			if (!b_generate_success)
			{
				cout << "生成轨迹失败!" << endl;
				itotaltime = -1;
			}
			else
			{
				//icounter = 0;
				cout << "更新轨迹了" << endl;
				for (int mk = 0; mk < 3; mk++)
					df_previous_pos[mk] = df_pos_orient[mk];
			}
			cout << "***********************************************************" << endl;
		

		//while ((icounter < itotaltime + 1))
		{
			//cout << "run circle!" << endl;
			b_running = true;
			//Eigen::VectorXd vxd_q_act_post = controller_ur->get_q();
			//AbstractControllerUR->getPosition();
			for (int im = 0; im < 6; im++)
			{
				//controller_ur->q_pos_target[im] = df_pos_orient[im];
				controller_ur->q_pos_target[im] = q_pos_sequence[im + icounter * 6];
				controller_ur->qcd[im] = q_speed_sequence[im + icounter * 6];
				if (abs(controller_ur->q_pos_target[im]) < ZERO_THRESH)
					controller_ur->q_pos_target[im] = 0;
			}
			for(int ik=5;ik<6;ik++)
				controller_ur->specify_qcd((double)ik / 5);
			//cRun.wait(sl);
			icounter++;
			if (icounter >= itotaltime)
			{
				icounter = itotaltime;
				b_running = false;
				inputnum = 52;
			}

			for (int i = 0; i < 6; i++)
			{
				q_last_circle[i] = df_array_q_choosed_pos[i];
			}
			do
			{
				circle_end_time = boost::get_system_time();
				circle_time = circle_end_time - cur_time;
				cirtime_ms = (double)circle_time.total_milliseconds() / 1000;
				//cout << cirtime_ms << endl;
			} while (cirtime_ms < 0.0001);//0.007  从循环开始到这里0.056S，
			cRun.wait(sl);
		}

		//b_running = false;
		set_pos.close();
		read_pos.close();
		total_point.close();
		act_xyz_pos.close();
		
		//i_input++;
	}
	time_elapse.close();
	delete[] q_pos_sequence;
	boost::posix_time::ptime stop_time = boost::get_system_time();
	controller_ur->stop(stop_time);
}

int main(int argc, char* argv[])
{

#ifdef single_arm
	// realsense
	//RealsenseCamera rs;
	////rs.set_frame_size(640, 480);
	//rs.read_handeye_params("handeye_results_realsense.xml");//外参
	//rs.read_handeye_intrinsic_params("intrinsic_params_realsense.xml");//内参
	//rs.start();

	 

	// variables for connecting ur
	boost::asio::io_service io_service;// , io_service2;
	boost::asio::io_service::work work(io_service);// , work2(io_service2);
	ur_sdk::client client(io_service, cRun, argv[3], argv[4]);
	//ur_sdk::client client2(io_service2, cRun2, "192.168.2.100", argv[4]);//99-100之间换，这里设置从机ip:99虚拟机,100:中间UR,98:主机，靠窗UR
	//这里记得交换回来
	CartesianSpaceTrackUR controller_ur(client);// , controller_ur2(client2);//这里记得交换回来
	controller_ur.controller_type = CartesianSpaceTrackUR::CONTROLLER_TYPE::FULL_POSE;//FULL_POSE   POSITION_ONLY
	//controller_ur2.controller_type = CartesianSpaceTrackUR::CONTROLLER_TYPE::FULL_POSE;//FULL_POSE   POSITION_ONLY

	running = true;
	// start the threads
	boost::thread thread_io_service(boost::bind(&boost::asio::io_service::run, &io_service));
	//boost::thread thread_io_service2(boost::bind(&boost::asio::io_service::run, &io_service2));//path_generate_ur_pcontrol
	//boost::thread thread_control_ur(boost::bind(&path_generate_ur_pcontrol, &controller_ur));//control_ur   controller_ur  control_ur_pcontrol  path_generate_ur_pcontrol
	//boost::thread thread_realsense_position(&track, &controller_ur);//
    boost::thread thread_control_ur(boost::bind(&ur_ibvs_control, &controller_ur));//ur_ibvs_control ur_redbox_control
	//boost::thread thread_realsense_position(&track_ibvs, &controller_ur);//track_ibvs  track_chessboard  track_redblue_box
	//boost::thread thread_realsense_position(&gen_pointcloud,&rs,&controller_ur);
	//boost::thread thread_control_ur(boost::bind(&control_ur, &controller_ur));//joint_space_control_ur cartesion_control_ur  control_ur  ur_cal_and_run  control_ur_pcontrol  cartesion_ur_cal_and_run
	//boost::thread thread_control_ur2(boost::bind(&control_ur2, &controller_ur ,&controller_ur2));
	//boost::thread thread_collision_detection(boost::bind(&collision_detection, &controller_ur, &controller_ur2));
    //boost::thread thread_detect_target(boost::bind(&detect_target, &stereo_camera));
	//boost::thread thread_show(boost::bind(&show_stereo, &stereo_camera));
	//boost::thread thread_allepro_hand(&test_tmain);
	//boost::thread thread_touch_sensor1(&touch_sensor_read_data);

	SetThreadPriority(thread_io_service.native_handle(), THREAD_PRIORITY_TIME_CRITICAL);
	SetThreadPriority(thread_control_ur.native_handle(), THREAD_PRIORITY_TIME_CRITICAL);
	//SetThreadPriority(thread_realsense_position.native_handle(), THREAD_PRIORITY_TIME_CRITICAL);
	//SetThreadPriority(thread_io_service2.native_handle(), THREAD_PRIORITY_TIME_CRITICAL);
	
	//SetThreadPriority(thread_control_ur2.native_handle(), THREAD_PRIORITY_TIME_CRITICAL);
	//SetThreadPriority(thread_collision_detection.native_handle(), THREAD_PRIORITY_TIME_CRITICAL);
	//SetThreadPriority(thread_allepro_hand.native_handle(), THREAD_PRIORITY_TIME_CRITICAL);
	//SetThreadPriority(thread_touch_sensor1.native_handle(), THREAD_PRIORITY_TIME_CRITICAL);

	_getch();
	running = false;

	io_service.stop();
	//io_service2.stop();
	thread_io_service.join();
	//thread_io_service2.join();
	thread_control_ur.join();
	//thread_realsense_position.join();
	//thread_control_ur2.join();
	//thread_collision_detection.join();
	//thread_allepro_hand.join();
	//thread_touch_sensor1.join();
	//thread_detect_target.join();
	//thread_show.join();
#endif 


	//dua_arm
#ifdef  dua_arm
// variables for connecting ur
boost::asio::io_service io_service , io_service2;
boost::asio::io_service::work work(io_service) , work2(io_service2);
ur_sdk::client client(io_service, cRun, argv[3], argv[4]);
ur_sdk::client client2(io_service2, cRun2, "192.168.2.100", argv[4]);//99-100之间换，这里设置从机ip:99虚拟机,100:中间UR,98:主机，靠窗UR
//这里记得交换回来
CartesianSpaceTrackUR controller_ur(client), controller_ur2(client2);//这里记得交换回来
controller_ur.controller_type = CartesianSpaceTrackUR::CONTROLLER_TYPE::FULL_POSE;//FULL_POSE   POSITION_ONLY
controller_ur2.controller_type = CartesianSpaceTrackUR::CONTROLLER_TYPE::FULL_POSE;//FULL_POSE   POSITION_ONLY

running = true;
// start the threads
boost::thread thread_io_service(boost::bind(&boost::asio::io_service::run, &io_service));
boost::thread thread_io_service2(boost::bind(&boost::asio::io_service::run, &io_service2));
//boost::thread thread_control_ur(boost::bind(&path_generate_ur_pcontrol, &controller_ur));//control_ur   controller_ur

//boost::thread thread_control_ur(boost::bind(&control_ur, &controller_ur));//joint_space_control_ur cartesion_control_ur  control_ur  ur_cal_and_run  control_ur_pcontrol  cartesion_ur_cal_and_run
boost::thread thread_control_ur2(boost::bind(&control_ur2, &controller_ur ,&controller_ur2));
//boost::thread thread_control_ur2(boost::bind(&ompl_path_control_ur, &controller_ur, &controller_ur2));
//boost::thread thread_realsense_position(&track, &controller_ur);
//boost::thread thread_collision_detection(boost::bind(&collision_detection, &controller_ur, &controller_ur2));
//boost::thread thread_detect_target(boost::bind(&detect_target, &stereo_camera));
//boost::thread thread_show(boost::bind(&show_stereo, &stereo_camera));
//boost::thread thread_allepro_hand(&test_tmain);
//boost::thread thread_touch_sensor1(&touch_sensor_read_data);

SetThreadPriority(thread_io_service.native_handle(), THREAD_PRIORITY_TIME_CRITICAL);
SetThreadPriority(thread_io_service2.native_handle(), THREAD_PRIORITY_TIME_CRITICAL);
//SetThreadPriority(thread_control_ur.native_handle(), THREAD_PRIORITY_TIME_CRITICAL);
SetThreadPriority(thread_control_ur2.native_handle(), THREAD_PRIORITY_TIME_CRITICAL);
//SetThreadPriority(thread_realsense_position.native_handle(), THREAD_PRIORITY_TIME_CRITICAL);
//SetThreadPriority(thread_collision_detection.native_handle(), THREAD_PRIORITY_TIME_CRITICAL);
//SetThreadPriority(thread_allepro_hand.native_handle(), THREAD_PRIORITY_TIME_CRITICAL);
//SetThreadPriority(thread_touch_sensor1.native_handle(), THREAD_PRIORITY_TIME_CRITICAL);

_getch();
running = false;

io_service.stop();
io_service2.stop();
thread_io_service.join();
thread_io_service2.join();
//thread_control_ur.join();
thread_control_ur2.join();
//thread_realsense_position.join();
//thread_collision_detection.join();
//thread_allepro_hand.join();
//thread_touch_sensor1.join();
//thread_detect_target.join();
//thread_show.join();
#endif
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
