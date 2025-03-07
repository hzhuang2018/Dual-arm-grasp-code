#pragma once

// ur_romote_control.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include <iostream>

#include <fstream>
#include <cartesian_space_track_ur.h>
#include <camera.h>
#include <ur_sdk.h>
#include <conio.h>
#include <opencv2\opencv.hpp>
#include <vector>
#include <algorithm>
#define _USE_MATH_DEFINES
#include<math.h>

#include <iomanip>
#include <algorithm>
#include <ctime>

#define UR_CYCLIC_TIME_MS 8
#define CONTROL_PERIOD_MS 40

// variable used in the control thread of UR
boost::condition_variable cRun;
boost::mutex mUpdate;
bool running;

std::vector<cv::Point2f> left_points, right_points;
// position of the target
Eigen::Vector3d pos_target;

using namespace Robot;
//void ur_cal_and_run(CartesianSpaceTrackUR* controller_ur);