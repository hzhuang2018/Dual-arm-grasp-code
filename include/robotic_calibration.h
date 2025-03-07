//
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
#pragma once
#include<opencv2/opencv.hpp>
#include<opencv2/calib3d.hpp>
#include<opencv2/imgproc.hpp>
#include<iostream>
#include<vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include<Eigen/Dense>
#include<utils.h>
#include "dll_export.h"

using namespace std;
namespace Robot {
	// class for calibrating mono camera
	class ROBOTIC_GRASP_DLL MonoCameraCalibration
	{
	public:
		MonoCameraCalibration();
		~MonoCameraCalibration();

		void read_config(const string& file_);
		void read_images(const string& filedir_);

		void detect_corners();
		void run_calib();
		void save_param();
		void save_param_xml(const string& filename_intrinsic_);
		void save_pose();
		vector<Rot> getRot();
		vector<Vec> getVec();

		cv::Size board_size;   // calibration board
		float square_size;  // size of square
	private:
		vector<cv::Mat> images;  // all acquired images
		cv::Size image_size;
		vector<int> valid_indexes; // indexes of all valid images
		vector<vector<cv::Point2f>> corners; // all detected corners
		vector<vector<cv::Point3f>> coordinates; // 3d position of corners
		int count;  // count of all images

		vector<cv::Mat> rots;  // all rotation matrix
		vector<cv::Mat> trans; // all translation 
		cv::Mat dist_coeff;    // distortion parameter
		cv::Mat K;             // intrinsic parameter

		
	};
	// class for calibrating stereo camera
	class ROBOTIC_GRASP_DLL StereoCalibration {
	public:
		enum PATTERN_TYPE{
			CHESSBORD,
			CIRCLEGRID
		};
	public:
		StereoCalibration();
		~StereoCalibration();
		/**********************************************/
		//找角点
		//设置类成员变量：left_corners，right_corners
		void add_images(cv::Mat& left_img, cv::Mat& right_img);
		void draw_pattern(const cv::Mat& img_, cv::Mat& imgc_);
		/************************************************************/
		//设置世界坐标系下的各角点XYZ坐标，其中Z均为0.坐标系原点：左上角点
		//设置类成员变量：obj_point
		//输入：
		//row_:行点数
		//col_:列点数
		//grid_length:棋盘格子的宽
		void set_pattern(int row_, int col_, double grid_length_);
		void run_calibration();
		void save_parameters(const string& filename_intrinsic_, const string& filename_extrinsic_);
		int get_image_count();

		PATTERN_TYPE pattern_type;
		cv::Size winSize;
		cv::Size imageSize;
	private:
		vector<cv::Mat> left_imgs;
		vector<cv::Mat> right_imgs;

		// camera matrix
		cv::Mat K_left;
		cv::Mat K_right;
		// distorsition coeffs
		cv::Mat dist_coeffs_left;
		cv::Mat dist_coeffs_right;
		// extrinsic parameters
		cv::Mat R;		// the rotation from the left camera to the right one, i.e., R = Rr * Rl^T
		cv::Mat t;		// the translation from the left camera to the right one, i.e., t = tr - R * tl
		cv::Mat E;
		cv::Mat F;
		cv::Mat R1;
		cv::Mat R2;
		cv::Mat P1;
		cv::Mat P2;
		cv::Mat Q;
		cv::Rect validRoi_left;
		cv::Rect validRoi_right;

		vector<vector<cv::Point2f>> left_corners;
		vector<vector<cv::Point2f>> right_corners;

		vector<cv::Point3f> obj_point;

		int rows_points;
		int cols_points;
		double grid_length;
		cv::Size pattern_size;
	};

	// class for calibrating eye-in/to-hand camera and robotic manipulator
	class ROBOTIC_GRASP_DLL RobotCameraCalibration {
	public:
		enum ROBOT_CAMERA_TYPE {
			EYE_IN_HAND,
			EYE_TO_HAND
		};
	public:
		RobotCameraCalibration();
		~RobotCameraCalibration();

		void run_calib();
		// add poses
		void add_pose_pairs(const Rot& R_robot_, const Vec& t_robot_, const Rot& R_camera_, const Vec& t_camera_);
		// calib the mono camera
		void calib_camera(const string& image_dir_, cv::Size pattern_size_, double square_size_);
		void calib_camera(const string& image_dir_, cv::Size pattern_size_, double square_size_, const string& filename_intrinsic_);
		// load the camera poses from the file.
		void load_camera_poses(const string& filename_);
		void load_robot_poses(const string& filename_);
		void save_results(const string& filename_);


		/* If calibratig the eye-in-hand camera, [Rcg, tcg] denotes the rotation and translation from the camera to the gripper. */
		/* If calibratig the eye-to-hand camera, [Rcg, tcg] denotes the rotation and translation from the camera to the base frame. */
		Rot get_Rcg();
		Vec get_tcg();

		ROBOT_CAMERA_TYPE type;
		
	private:
		MonoCameraCalibration calib4camera;  // calibration for camera

		vector<Rot> Rg;     // rotation matrices of hand
		vector<Vec> tg;     // translation vectors of hand

		vector<Rot> Rc;     // rotation matrices of camera
		vector<Vec> tc;     // translation vectors of camera

		// observation pairs, unused for now
		vector<Rot> Rgij;
		vector<Vec> tgij;
		vector<Rot> Rcij;
		vector<Vec> tcij;

		// calculated from calibration
		Rot Rcg;            // rotation matrix from camera to hand
		Vec tcg;            // translation from camera to hand


		// function for find eigenvector with eigenvalue 1
		Vec calEigenvector(const Rot& R_);

	};
}