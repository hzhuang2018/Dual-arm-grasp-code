#pragma once
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
#include<math.h>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

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
#include "planner_joint_dualarm.h"
#include "planner_cartisian.h"
using namespace std::chrono;
using namespace Robot;
void path_generate_ur_pcontrol(CartesianSpaceTrackUR* controller_ur);
void ur_ibvs_control(CartesianSpaceTrackUR* controller_ur);
void ur_redbox_control(CartesianSpaceTrackUR* controller_ur);
void ompl_path_control_ur(CartesianSpaceTrackUR* controller_ur, CartesianSpaceTrackUR* controller_ur2);