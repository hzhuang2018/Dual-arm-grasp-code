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

// robotcamera_calibration.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include<ur_sdk.h>
#include<iostream>
#include<camera.h>
#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
#include<string>
#include<fstream>
#include<sstream>
#include<cartesian_space_track_ur.h>
#include<robotic_calibration.h>
#include<utils.h>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

//#define HANDEYE_CALIB_OFFLINE
#define HANDEYE_CALIB_ONLINE



boost::condition_variable cRun;
boost::mutex mUpdate;

#define UR_CYCLIC_TIME_MS 2
using namespace std;
int main(int argc, char* argv[])
{
	//cv::Mat myvec(3, 1, CV_64FC1) ;
	//cv::Mat myresult(3, 1, CV_64FC1);
	//myvec.at<double>(0, 0) = 1.118;
	//myvec.at<double>(1, 0) = 0.991;
	//myvec.at<double>(2, 0) = 3.416;
	//cv::Mat Rccc(3, 3, CV_64FC1);
	//cv::Mat Rccc1(3, 3, CV_64FC1);
	//cv::Rodrigues(myvec, Rccc);
	//cv::Rodrigues(Rccc, myresult);
	//cv::Rodrigues(myresult, Rccc1);
	//cout << Rccc << endl;
	//cout << myresult << endl;
	//cout << Rccc1 << endl;


#ifdef HANDEYE_CALIB_OFFLINE
	/*
	cv::Mat_<double> CamPose = (cv::Mat_<double>(13, 6) <<
		-0.072944147641399, -0.06687830562048944, 0.4340418493881254, -0.2207496117519063, 0.0256862005614321, 0.1926014162476009,
		-0.01969337858360518, -0.05095294728651902, 0.3671266719105768, 0.1552099329677287, -0.5763323472739464, 0.09956130526058841,
		0.1358164536530692, -0.1110802522656379, 0.4001396735998251, -0.04486168331242635, -0.004268942058870162, 0.05290073845562016,
		0.1360676260120161, -0.002373036366121294, 0.3951670952829301, -0.4359637938379769, 0.00807193982932386, 0.06162504121755787,
		-0.1047666520352697, -0.01377729010376614, 0.4570029374109721, -0.612072103513551, -0.04939465180949879, -0.1075464055169537,
		0.02866460103085085, -0.1043911269729344, 0.3879127305077527, 0.3137563103168434, -0.02113958397023016, 0.1311397970432597,
		0.1122741829392126, 0.001044006395747612, 0.3686697279333643, 0.1607160803445018, 0.2468677059920437, 0.1035103912091547,
		-0.06079521129779342, -0.02815190820828123, 0.4451740202390909, 0.1280935541917056, -0.2674407142401368, 0.1633865613363686,
		-0.02475533256363622, -0.06950841248698086, 0.2939836207787282, 0.1260629671933584, -0.2637748974005461, 0.1634102148863728,
		0.1128618887222624, 0.00117877722121125, 0.3362496409334229, 0.1049541359309871, -0.2754352318773509, 0.4251492928748009,
		0.1510545750008333, -0.0725019944548204, 0.3369908269102371, 0.2615745097093249, -0.1295598776133405, 0.6974394284203849,
		0.04885313290076512, -0.06488755216394324, 0.2441532410787161, 0.1998243391807502, -0.04919417529483511, -0.05133193756053007,
		0.08816140480523708, -0.05549965109057759, 0.3164905645998022, 0.164693654482863, 0.1153894876338608, 0.01455551646362294);

	//机械臂末端13组位姿,x,y,z,rx,ry,rz
	cv::Mat_<double> ToolPose = (cv::Mat_<double>(13, 6) <<
		-0.3969707, -0.460018, 0.3899877, 90.2261, -168.2015, 89.7748,
		-0.1870185, -0.6207147, 0.2851157, 57.2636, -190.2034, 80.7958,
		-0.1569776, -0.510021, 0.3899923, 90.225, -178.2038, 81.7772,
		-0.1569787, -0.5100215, 0.3299975, 90.2252, -156.205, 81.7762,
		-0.3369613, -0.4100348, 0.3299969, 90.2264, -146.2071, 71.778,
		-0.2869552, -0.6100449, 0.4299998, 90.2271, -199.2048, 86.7806,
		-0.2869478, -0.6600489, 0.4299948, 105.2274, -189.2053, 86.7814,
		-0.286938, -0.6300559, 0.4299997, 75.2279, -189.2056, 86.783,
		-0.2869343, -0.5700635, 0.2800084, 75.2291, -189.2055, 86.7835,
		-0.1669241, -0.5700796, 0.280015, 75.2292, -189.205, 101.7845,
		-0.236909, -0.4700997, 0.3600046, 87.2295, -196.2063, 118.7868,
		-0.2369118, -0.6201035, 0.2600001, 87.2297, -192.2087, 75.7896,
		-0.2468983, -0.620112, 0.359992, 97.2299, -190.2082, 80.7908);

	Robot::RobotCameraCalibration handeye_calib;
	handeye_calib.type = Robot::RobotCameraCalibration::ROBOT_CAMERA_TYPE::EYE_IN_HAND;

	for (int i = 0;i < 16;i++)
	{
		Robot::Rot R1 = Robot::vec2rot(CamPose.at<double>(i, 3), CamPose.at<double>(i, 4), CamPose.at<double>(i, 5));
		Robot::Vec t1;
		t1 << CamPose.at<double>(i, 0), CamPose.at<double>(i, 1), CamPose.at<double>(i, 2);
		Robot::Rot R2 = Robot::vec2rot(ToolPose.at<double>(i, 3)/57.3, ToolPose.at<double>(i, 4) / 57.3, ToolPose.at<double>(i, 5) / 57.3);
		Robot::Vec t2;
		t2 << ToolPose.at<double>(i, 0), ToolPose.at<double>(i, 1), ToolPose.at<double>(i, 2);

		handeye_calib.add_pose_pairs(R2, t2, R1, t1);
	}

	handeye_calib.run_calib();
	*/

	Robot::RobotCameraCalibration robot_camera_calib;
	if (atoi(argv[5]) == 1)
		robot_camera_calib.type = Robot::RobotCameraCalibration::ROBOT_CAMERA_TYPE::EYE_IN_HAND;
	if (atoi(argv[5]) == 2)
		robot_camera_calib.type = Robot::RobotCameraCalibration::ROBOT_CAMERA_TYPE::EYE_TO_HAND;
	//robot_camera_calib.calib_camera("./handeye_calib/", cv::Size(7, 7), 0.025);
	robot_camera_calib.load_robot_poses("./handeye_calib/robot_pose.txt");
	//for_each(vector<Rot>iterator it.)
	robot_camera_calib.run_calib();

	cout << "Rcg: " << endl << robot_camera_calib.get_Rcg() << endl;
	cout << "tcg: " << robot_camera_calib.get_tcg() << endl;
#endif // HANDEYE_CALIB_OFFLINE


#ifdef HANDEYE_CALIB_ONLINE
	if (argc < 6)
	{
		printf("Please run this program by: robotcamera_calibration leftcam_sn rightcam_sn <ip_ur> <port_ur> calib_type(1 for eye-in-hand, 2 for eye-to-hand)\n");
		return -1;
	}
	/*	
	std::string left_sn(argv[1]);
	std::string right_sn(argv[2]);
	Robot::BaslerStereoCamera stereo_camera;
	stereo_camera.setup(left_sn, right_sn);
	// start the loop thread for grabing images
	stereo_camera.start();
	*/

	//set realsense
	rs2::config cfg;

	///配置彩色图像流：分辨率640*480，图像格式：BGR， 帧率：30帧/秒
	cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);//1280 *720
	///配置深度图像流：分辨率640*480，图像格式：Z16， 帧率：30帧/秒
	cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);

	// Create a Pipeline - this serves as a top-level API for streaming and processing frames
	rs2::pipeline p;

	// Configure and start the pipeline
	p.start(cfg);
	//p.start();
	//set realsense end

	boost::asio::io_service io_service;
	boost::asio::io_service::work work(io_service);
	ur_sdk::client client(io_service, cRun, argv[3], argv[4]);

	Robot::CartesianSpaceTrackUR controller(client);

	boost::thread thread_io_service(boost::bind(&boost::asio::io_service::run, &io_service));

	string filename = "./handeye_calib/image_calib_";
	int counter = 0;


	bool saved = false;

	std::cout << "show image!" << endl;

	ofstream write;
	write.open("./handeye_calib/robot_pose_realsense.txt");
	write << "# recorded poses of robot manipulator" << endl;

	ofstream file;
	file.open("./handeye_calib/file.txt");

	bool running = true;

	while (running)
	{
		// read latest grabbed image 
		//cv::Mat img;
		//img = stereo_camera.get_right_image();
		
		/*realsense get pic*/
		rs2::frameset frames = p.wait_for_frames();
		rs2::video_frame color = frames.get_color_frame(); ///获取彩色图像数据
		rs2::video_frame depth = frames.get_depth_frame(); ///获取彩色图像数据
		int color_width = color.get_width();
		int color_height = color.get_height();
		int depth_width = depth.get_width();
		int depth_height = depth.get_height();
		if (!color ) break; ///如果获取不到数据则退出
		cv::Mat img(cv::Size(color_width, color_height), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
		/*realsense get pic finish*/

		if (!img.data) continue;

		//cv::Mat view(img.rows / 2, img.cols / 2, CV_8UC1);
		//cv::resize(img, view, cv::Size(), 0.5, 0.5);
		cv::imshow("image", img);

		char key = cv::waitKey(30);
		int key_int = (int)key;
		// cout << "press key: " << (int)key << endl;
		switch (key_int) {
		case 27:
		{
			cv::destroyAllWindows();
			//stereo_camera.stop();
			running = false;
			
			break;
		}
		case 13:
		{
			// save images and joint angles
			std::cout << "saving image..." << std::endl;
			string sn;
			stringstream ss;
			ss << counter;
			ss >> sn;
			cv::imwrite(filename + sn + ".bmp", img);
			file << "image_calib_" + sn + ".bmp" << endl;
			counter++;

			controller.update(boost::get_system_time(), boost::posix_time::milliseconds(UR_CYCLIC_TIME_MS));

			Eigen::VectorXd pose = controller.get_tool_pose();
			//vector<double> pose = client.getToolPose();
			write << pose(0) << " " << pose(1) << " " << pose(2) << " " << pose(3) << " " << pose(4) << " " << pose(5) << endl;
			std::cout << "pose: " << pose(0) << " " << pose(1) << " " << pose(2) << " " << pose(3) << " " << pose(4) << " " << pose(5) << std::endl;
			break;
		}
		default:
			break;
		}
	}
	io_service.stop();
	thread_io_service.join();
	write.close();
//#endif // HANDEYE_CALIB_OFFLINE
	// run the handeye calibration
	printf("Run handeye calibration ...\n");
	Robot::RobotCameraCalibration robot_camera_calib;
	if (atoi(argv[5]) == 1)
		robot_camera_calib.type = Robot::RobotCameraCalibration::ROBOT_CAMERA_TYPE::EYE_IN_HAND;
	if (atoi(argv[5]) == 2)
		robot_camera_calib.type = Robot::RobotCameraCalibration::ROBOT_CAMERA_TYPE::EYE_TO_HAND;
	robot_camera_calib.calib_camera("./handeye_calib/", cv::Size(11, 8), 0.05, "intrinsic_params_realsense.xml");
	robot_camera_calib.load_robot_poses("./handeye_calib/robot_pose_realsense.txt");
	robot_camera_calib.run_calib();
	robot_camera_calib.save_results("handeye_results_realsense.xml");
#endif

	return 0;
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
