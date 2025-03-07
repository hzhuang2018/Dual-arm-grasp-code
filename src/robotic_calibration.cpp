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

#include<robotic_calibration.h>
#include<opencv2/imgproc.hpp>

namespace Robot {
	using namespace cv;
	// mono camera calibration
	MonoCameraCalibration::MonoCameraCalibration()
	{

	}
	MonoCameraCalibration::~MonoCameraCalibration()
	{

	}

	void MonoCameraCalibration::detect_corners()
	{
		if (images.empty())
			return;

		cv::namedWindow("calibration", 0);
		for (size_t i = 0; i < images.size(); i++)
		{
			if (i == 0)
			{
				image_size.width = images[i].cols;
				image_size.height = images[i].rows;
				cout << "image size: " << image_size.width << " " << image_size.height << endl;
			}
			vector<Point2f> corner;
			//cout << "image size: " << image_size.width << " " << image_size.height << endl;
			int res = findChessboardCorners(images[i], board_size, corner);
			if (res == 0)
			{
				cout << "No chessboard corners found! Drop this image!" << endl;
				continue;
			}
			else
			{
				find4QuadCornerSubpix(images[i], corner, Size(11, 11));
				// save subpixel corners;
				corners.push_back(corner);
				valid_indexes.push_back((int)i);

				/* show corner */
				Mat view;
				//view = images[i].clone();
				cv::cvtColor(images[i], view, COLOR_GRAY2BGR);
				cv::drawChessboardCorners(view, board_size, corner, true);
				Mat view_down;
				cv::resize(view, view_down, Size(), 0.25, 0.25);
				cv::imshow("calibration", view_down);
				cv::waitKey(500);

				cout << "found " << corner.size() << " corners." << endl;
			}
		}

		cout << "Corners detection have been finished!" << endl;

		// set coordinates of calibration target
		for (size_t i = 0; i < valid_indexes.size(); i++)
		{
			vector<Point3f> point;
			for (int m = 0; m < board_size.height; m++)
			{
				for (int n = 0; n < board_size.width; n++)
				{
					Point3f pt;
					pt.x = n * square_size;
					pt.y = m * square_size;
					pt.z = 0.0f;
					point.push_back(pt);
				}
			}
			coordinates.push_back(point);
		}
	}
	void MonoCameraCalibration::run_calib()
	{
		K = Mat(3, 3, CV_32FC1, Scalar::all(0));
		dist_coeff = Mat(1, 5, CV_32FC1, Scalar::all(0));
		int flag = 0;
		flag |= CALIB_FIX_K5;// CV_CALIB_FIX_K5;
		cout << "Start calibration..." << endl;
		calibrateCamera(coordinates, corners, image_size, K, dist_coeff, rots, trans, flag);
		cout << "Cabliration finished!" << endl;

		cout << "K: \n" << K << endl;
		cout << "dist: \n" << dist_coeff << endl;
		cout << "pose with respect to chessboard..." << endl;
		for (int i = 0; i < rots.size(); i++)
		{
			cout << "image " << i << " :" << endl;
			cout << rots[i] << "\n" << trans[i] << endl;
		}

		// report the cablibration result
		cout << "Evaluating the calibration result..." << endl;
		double err = 0.0;
		vector<Point2f> corner_reprj; // reprojected corner
		for (size_t i = 0; i < valid_indexes.size(); i++)
		{
			int index = valid_indexes[i];
			projectPoints(coordinates[i], rots[i], trans[i], K, dist_coeff, corner_reprj);
			double res = 0.0;
			for (int j = 0; j < corners[index].size(); j++)
			{
				double err_x = corners[index][j].x - corner_reprj[j].x;
				double err_y = corners[index][j].y - corner_reprj[j].y;
				res += sqrt(err_x * err_x + err_y * err_y);
			}

			res /= corners[index].size();
			err += res;
			cout << "mean reprojected error of image " << index << " : " << res << endl;
		}
		cout << "<<------------------------------------------->>" << endl;
		cout << "mean reprojected error of all images: " << err / valid_indexes.size() << endl;
		cout << "<<------------------------------------------->>" << endl;
	}

	void MonoCameraCalibration::read_config(const string& file_)
	{

		ifstream in(file_);
		char s[1024] = { 0 };
		int width;
		int height;
		double x;
		in.getline(s, sizeof(s));
		stringstream data(s);
		data >> width;
		data >> height;
		data >> x;

		board_size = Size(width, height);
		square_size = x;
		cout << "size of chessboard: " << board_size.width << " " << board_size.height << endl;
		cout << "size of square: " << square_size << endl;
	}

	void MonoCameraCalibration::read_images(const string& filedir_)
	{
		ifstream in(filedir_ + "file.txt");
		string filename;
		char line[1024] = { 0 };
		//cv::namedWindow("image", 0);
		while (in.getline(line, sizeof(line)))
		{
			stringstream ss(line);
			ss >> filename;
			if (filename == "") continue;
			cout << "reading image file: " << filedir_ + filename << endl;
			Mat img = imread(filedir_ + filename, 0);
			images.push_back(img);
			cv::imshow("image",img);
			cv::waitKey(500);
		}
	}
	void MonoCameraCalibration::save_param()
	{
		ofstream write("calibration_result.txt");
		write << "# calibration result of camera" << endl;
		write << "# fx, fy, cx, xy" << endl;
		write << K.at<double>(0, 0) << " " << K.at<double>(1, 1) << " " << K.at<double>(0, 2) << " " << K.at<double>(1, 2) << endl;
		write << "# k1, k2, p1, p2, k3" << endl;
		write << dist_coeff << endl;
		write.close();
	}
	void MonoCameraCalibration::save_param_xml(const string& filename_intrinsic_)
	{
		cout << "save intrinsic parameters for handeye camera" << endl;
		cv::FileStorage fs(filename_intrinsic_.c_str(), cv::FileStorage::WRITE);
		if (fs.isOpened())
		{
			fs << "K" << K << "dist" << dist_coeff;
			fs.release();
		}
		else
		{
			printf("Cannot save the intrinsic parameters for handeye camera");
		}
	}
	void MonoCameraCalibration::save_pose()
	{
		ofstream write("calibration_pose.txt");
		write << "# tx ty tz theta_x theta_y theta_z" << endl;
		for (size_t i = 0; i < rots.size(); i++)
		{
			cv::Mat Rc(3, 3, CV_32FC1);
			cv::Rodrigues(rots[i], Rc);
			// cout << "cv: " << Rc << endl;
			// cout << "eigne: " << vec2rot(rots[i].at<double>(0,0), rots[i].at<double>(1,0), rots[i].at<double>(2,0)) << endl;
			write << trans[i].at<double>(0, 0) << " " << trans[i].at<double>(1, 0) << " " << trans[i].at<double>(2, 0) << " ";
			write << rots[i].at<double>(0, 0) << " " << rots[i].at<double>(1, 0) << " " << rots[i].at<double>(2, 0) << endl;
		}
		write.close();
	}

	vector<Rot> MonoCameraCalibration::getRot()
	{
		vector<Rot> R;

		for (size_t i = 0; i < rots.size(); i++)
		{
			cv::Mat rtmp(3, 3, CV_32FC1);
			cv::Rodrigues(rots[i], rtmp);
			R.push_back(Mat2Rot(rtmp));
		}

		return R;
	}
	vector<Vec> MonoCameraCalibration::getVec()
	{
		vector<Vec> t;
		for (size_t i = 0; i < trans.size(); i++)
			t.push_back(Mat2Vec(trans[i]));

		return t;
	}

	// stereo calibration
	StereoCalibration::StereoCalibration()
	{
		pattern_type = PATTERN_TYPE::CHESSBORD;
		winSize.height = 11;
		winSize.width = 11;
	}
	StereoCalibration::~StereoCalibration()
	{

	}
	void StereoCalibration::add_images(cv::Mat& left_img, cv::Mat& right_img)
	{
		if (pattern_type == PATTERN_TYPE::CHESSBORD)
		{
			/*
			cv::Mat tmp1, tmp2;
			if (imageSize.height >= 2048)
			{
				tmp1 = cv::Mat(imageSize / 4, CV_8UC3);
				tmp2 = cv::Mat(imageSize / 4, CV_8UC3);
				cv::resize(left_img, tmp1, cv::Size(), 0.25, 0.25);
				cv::resize(right_img, tmp2, cv::Size(), 0.25, 0.25);
			}
			else
			{
				tmp1 = left_img;
				tmp2 = right_img;
			}
			*/
			cv::Mat left_gray_img(left_img.size(),CV_8UC1), right_gray_img(left_img.size(),CV_8UC1);
			
			if (left_img.channels() == 1)
			{
				left_gray_img = left_img;
			}
			else
			{
				cv::cvtColor(left_img, left_gray_img, cv::COLOR_BGR2GRAY);
			}
			if (right_img.channels() == 1)
			{
				right_gray_img = right_img;
			}
			else
			{
				cv::cvtColor(right_img, right_gray_img, cv::COLOR_BGR2GRAY);
			}
			//cv::GaussianBlur(left_gray_img, left_gray_img, cv::Size(15, 15), 0.5, 0.5);
			//cv::GaussianBlur(right_gray_img, right_gray_img, cv::Size(15, 15), 0.5, 0.5);

			vector<cv::Point2f> left_corner, right_corner;
			bool flag1 = cv::findChessboardCorners(left_gray_img, pattern_size, left_corner, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
			bool flag2 = cv::findChessboardCorners(right_gray_img, pattern_size, right_corner, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
			
			if (flag1 && flag2)
			{
				/*
				if (imageSize.height >= 2048)
				{
					for (int i = 0;i < left_corner.size();i++)
					{
						left_corner[i].x = left_corner[i].x * 4;
						left_corner[i].y = left_corner[i].y * 4;
						right_corner[i].x = right_corner[i].x * 4;
						right_corner[i].y = right_corner[i].y * 4;
					}

				}
				cv::Mat tmp1(left_img.size(), CV_8UC1);
				cv::Mat tmp2(left_img.size(), CV_8UC1);
				cv::cvtColor(left_img, tmp1, cv::COLOR_RGB2GRAY);
				cv::cvtColor(right_img, tmp2, cv::COLOR_RGB2GRAY);
				*/
				cv::cornerSubPix(right_gray_img, left_corner, winSize, cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 0.0001));
				cv::cornerSubPix(right_gray_img, right_corner, winSize, cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 0.0001));

				left_corners.push_back(left_corner);
				right_corners.push_back(right_corner);
				if (left_img.channels() == 3)
					cv::drawChessboardCorners(left_img, pattern_size, cv::Mat(left_corner), flag1);
				if (right_img.channels() == 3)
					cv::drawChessboardCorners(right_img, pattern_size, cv::Mat(right_corner), flag2);

				left_imgs.push_back(left_gray_img);
				right_imgs.push_back(right_gray_img);
			}
			else
			{
				printf("No valid pattern found!\n");
				return;
			}
		}
		else {
			printf("Please use chessbord!\n");
		}
	}
	int StereoCalibration::get_image_count()
	{
		return left_imgs.size();
	}
	void StereoCalibration::draw_pattern(const cv::Mat& img_, cv::Mat& imgc_)
	{
		vector<cv::Point2f> corners;
		bool flag = cv::findChessboardCorners(img_, pattern_size, corners, 3);
		cv::drawChessboardCorners(imgc_, pattern_size, cv::Mat(corners), flag);
	}
	
	
	void StereoCalibration::set_pattern(int row_, int col_, double grid_length_)
	{
		if (pattern_type == PATTERN_TYPE::CHESSBORD)
		{
			rows_points = row_;
			cols_points = col_;
			grid_length = grid_length_;
			pattern_size.height = rows_points;
			pattern_size.width = cols_points;

			for (int i = 0;i < pattern_size.height;++i)
				for (int j = 0;j < pattern_size.width;++j)
					obj_point.push_back(cv::Point3f(j * grid_length, i * grid_length, 0.0));
		}
		
	}
	void StereoCalibration::run_calibration()
	{
		if (obj_point.size() == 0)
		{
			printf("The pattern of the object should be set first!\n");
			return;
		}
		// calibrate stereo
		printf("Running the stereo calibration ...\n");
		vector<vector<cv::Point3f>> obj_points;
		for (size_t i = 0;i < left_corners.size();i++)
		{
			obj_points.push_back(obj_point);
		}
		K_left = cv::initCameraMatrix2D(obj_points, left_corners, imageSize, 0);
		K_right = cv::initCameraMatrix2D(obj_points, right_corners, imageSize, 0);

		double rms = cv::stereoCalibrate(obj_points, left_corners, right_corners, K_left, dist_coeffs_left, K_right, dist_coeffs_right, imageSize, R, t, E, F,
			cv::CALIB_FIX_ASPECT_RATIO +
			cv::CALIB_ZERO_TANGENT_DIST +
			cv::CALIB_USE_INTRINSIC_GUESS +
			cv::CALIB_SAME_FOCAL_LENGTH +
			cv::CALIB_RATIONAL_MODEL +
			cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5,
			cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5));

		printf("The RMS error = %f.\n", rms);

		// check the quality
		printf("Checking the calibration quality ...\n");
		double err = 0;
		int npoints = 0;
		vector<cv::Vec3f> lines_left, lines_right;
		for (size_t i = 0;i < left_imgs.size();i++)
		{
			int npt = (int)left_corners[i].size();
			cv::Mat imgpt_left, imgpt_right;
			imgpt_left = cv::Mat(left_corners[i]);
			imgpt_right = cv::Mat(right_corners[i]);

			cv::undistortPoints(imgpt_left, imgpt_left, K_left, dist_coeffs_left, cv::Mat(), K_left);
			cv::undistortPoints(imgpt_right, imgpt_right, K_right, dist_coeffs_right, cv::Mat(), K_right);

			cv::computeCorrespondEpilines(imgpt_left, 1, F, lines_left);
			cv::computeCorrespondEpilines(imgpt_right, 2, F, lines_right);
			for (int j = 0;j < npt;j++)
			{
				double err_ij = fabs(imgpt_left.at<float>(j,0) * lines_right[j][0] + imgpt_left.at<float>(j, 1) * lines_right[j][1] + lines_right[j][2]) +
					fabs(imgpt_right.at<float>(j, 0) * lines_left[j][0] + imgpt_right.at<float>(j, 1) * lines_left[j][1] + lines_left[j][2]);
				//std::cout << "point: " << imgpt_left.at<float>(j, 0) << "," << imgpt_left.at<float>(j, 1) << std::endl;
				err += err_ij;
			}
			npoints += npt;
		}
		// printf("Total points: %d\n", npoints);
		printf("The average epipolar error = %f.\n", err / npoints / 2);
	}
	void StereoCalibration::save_parameters(const string& filename_intrinsic_, const string& filename_extrinsic_)
	{
		printf("Saving the intrinsic and extrinsic parameters of the stereo camera ...\n");
		cv::FileStorage fs(filename_intrinsic_.c_str(), cv::FileStorage::WRITE);
		if (fs.isOpened())
		{
			fs << "K_left" << K_left << "dist_left" << dist_coeffs_left << "K_right" << K_right << "dist_right" << dist_coeffs_right;
			fs.release();
		}
		else
		{
			printf("Cannot save the intrinsic parameters!\n");
		}
		cv::stereoRectify(K_left, dist_coeffs_left, K_right, dist_coeffs_right,
						  imageSize, R, t, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY,
						  1, imageSize, &validRoi_left, &validRoi_right);

		fs.open(filename_extrinsic_.c_str(), cv::FileStorage::WRITE);
		if (fs.isOpened())
		{
			fs << "R" << R << "t" << t << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
			fs.release();
		}
		else
		{
			printf("Cannot save the extrinsic parameters!\n");
		}
	}

	// roobt camera calibration
	RobotCameraCalibration::RobotCameraCalibration()
	{
		type = ROBOT_CAMERA_TYPE::EYE_IN_HAND;
	}
	RobotCameraCalibration::~RobotCameraCalibration()
	{

	}
	void RobotCameraCalibration::run_calib()
	{
		// trans to cv::Mat

		vector<cv::Mat> Rg_1, Rc_1;
		vector<cv::Mat> tg_1, tc_1;
		for (size_t i = 0; i < Rg.size(); i++)
		{
			cv::Mat tmp1(3,3,CV_64FC1), tmp2(3,3,CV_64FC1), tmp3(3,1,CV_64FC1), tmp4(3,1,CV_64FC1);
			for (int j = 0;j < 3;j++)
			{
				for (int k = 0;k < 3;k++)
				{
					tmp1.at<double>(j, k) = Rg[i](j, k);
					tmp2.at<double>(j, k) = Rc[i](j, k);
				}
				tmp3.at<double>(j, 0) = tg[i](j);
				tmp4.at<double>(j, 0) = tc[i](j);
			}
			Rg_1.push_back(tmp1);
			tg_1.push_back(tmp3);
			Rc_1.push_back(tmp2);
			tc_1.push_back(tmp4);
		}
		// calibration
		if (type == ROBOT_CAMERA_TYPE::EYE_IN_HAND) // eye-in-hand
		{
			printf("Run calibration for eye-in-hand camera ...\n");
			cv::Mat R, t;
			cv::calibrateHandEye(Rg_1, tg_1, Rc_1, tc_1, R, t, CALIB_HAND_EYE_TSAI);
			for (int i = 0;i < 3;i++)
			{
				for (int j = 0;j < 3;j++)
				{
					Rcg(i, j) = R.at<double>(i, j);
				}
				tcg(i) = t.at<double>(i);
			}
		}
		else // eye-to-hand
		{
			printf("Run calibration for eye-to-hand camera ...\n");
			// In this case, the rotation and translation from the target to the gripper is obtained.
			for (size_t i = 0; i < Rc_1.size();i++)
			{
				Rc_1[i] = Rc_1[i].inv();
				tc_1[i] = -1.0 * Rc_1[i] * tc_1[i];
			}
			cv::Mat R, t;
			cv::calibrateHandEye(Rg_1, tg_1, Rc_1, tc_1, R, t, CALIB_HAND_EYE_TSAI);
			Rot R_1;
			Vec t_1;
			for (int i = 0;i < 3;i++)
			{
				for (int j = 0;j < 3;j++)
				{
					R_1(i, j) = R.at<double>(i, j);
				}
				t_1(i) = t.at<double>(i);
			}
			// Calculating the rotation and translation from the camera to the base frame.
			Vec theta_m, trans_m;
			theta_m.setZero();
			trans_m.setZero();
			for (size_t i = 0; i < Rc.size(); i++)
			{
				Rot tmp1 = Rg[i] * R_1 * Rc[i].inverse();
				Vec tmp2 = tg[i] + Rg[i] * (t_1 - R_1 * Rc[i].inverse() * tc[i]);

				theta_m = theta_m + rot2vec(tmp1);
				trans_m = trans_m + tmp2;
			}
			theta_m = theta_m / Rc.size();
			Rcg = vec2rot(theta_m(0), theta_m(1), theta_m(2));
			tcg = trans_m / Rc.size();
		}
	}
	void RobotCameraCalibration::add_pose_pairs(const Rot& R_robot_, const Vec& t_robot_, const Rot& R_camera_, const Vec& t_camera_)
	{
		Rg.push_back(R_robot_);
		tg.push_back(t_robot_);
		Rc.push_back(R_camera_);
		tc.push_back(t_camera_);
	}
	void RobotCameraCalibration::calib_camera(const string& image_dir_, cv::Size pattern_size_, double square_size_)
	{
		calib4camera.read_images(image_dir_);
		calib4camera.board_size = pattern_size_;
		calib4camera.square_size = square_size_;
		calib4camera.run_calib();
		Rc = calib4camera.getRot();
		tc = calib4camera.getVec();
	}
	void RobotCameraCalibration::calib_camera(const string& image_dir_, cv::Size pattern_size_, double square_size_, const string& filename_intrinsic_)
	{
		calib4camera.read_images(image_dir_);
		calib4camera.board_size = pattern_size_;
		calib4camera.square_size = square_size_;
		calib4camera.detect_corners();
		calib4camera.run_calib();
		Rc = calib4camera.getRot();
		tc = calib4camera.getVec();
		calib4camera.save_param_xml(filename_intrinsic_);
	}
	Vec RobotCameraCalibration::calEigenvector(const Rot& R_)
	{
		Eigen::EigenSolver<Rot> es(R_);
		Eigen::Matrix<complex<double>, 3, 1> lambda = es.eigenvalues();
		Eigen::Matrix<complex<double>, 3, 3> vecs = es.eigenvectors();

		Vec vec;
		for (int i = 0; i < 3; i++)
		{

			if (fabs(lambda(i).real() - 1.0f) < 1e-4 && fabs(lambda(i).imag()) < 1e-4)
			{
				vec(0, 0) = vecs.col(i)(0, 0).real();
				vec(1, 0) = vecs.col(i)(1, 0).real();
				vec(2, 0) = vecs.col(i)(2, 0).real();
			}

		}
		return vec;
	}

	Rot RobotCameraCalibration::get_Rcg()
	{
		return Rcg;
	}

	Vec RobotCameraCalibration::get_tcg()
	{
		return tcg;
	}


	void RobotCameraCalibration::load_camera_poses(const string& filename_)
	{
		ifstream in(filename_);
		char s[1024] = { 0 };
		// negelect the first line
		in.getline(s, sizeof(s));
		while (in.getline(s, sizeof(s)))
		{
			double tx, ty, tz;
			double theta_x, theta_y, theta_z;
			stringstream data(s);
			data >> tx >> ty >> tz >> theta_x >> theta_y >> theta_z;
			Vec t;
			t << tx, ty, tz;
			tc.push_back(t);
			Rc.push_back(vec2rot(theta_x, theta_y, theta_z));
		}
	}

	void RobotCameraCalibration::load_robot_poses(const string& filename_)
	{
		ifstream in(filename_);
		char s[1024] = { 0 };
		// negelect the first line
		in.getline(s, sizeof(s));
		while (in.getline(s, sizeof(s)))
		{
			double tx, ty, tz;
			double theta_x, theta_y, theta_z;
			stringstream data(s);
			data >> theta_x >> theta_y >> theta_z >> tx >> ty >> tz;
			Vec t;
			t << tx, ty, tz;
			tg.push_back(t);
			Rg.push_back(vec2rot(theta_x, theta_y, theta_z));
		}
	}

	void RobotCameraCalibration::save_results(const string& filename_)
	{
		printf("saving the results of handeye calibration...\n");
		cv::FileStorage fs(filename_, cv::FileStorage::WRITE);

		cv::Mat Rcg_mat(3, 3, CV_64FC1), tcg_mat(3, 1, CV_64FC1);
		for (int j = 0; j < 3; j++)
		{
			for (int k = 0; k < 3; k++)
			{
				Rcg_mat.at<double>(j, k) = Rcg(j, k);
			}
			tcg_mat.at<double>(j, 0) = tcg(j);
		}

		if (fs.isOpened())
		{
			fs << "Rcg" << Rcg_mat << "tcg" << tcg_mat;
			fs.release();
		}
		else
		{
			printf("can't save the results of handeye calibration.../n");
		}
	}
}