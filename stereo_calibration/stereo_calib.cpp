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


// stereo_calibration.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include<robotic_calibration.h>
#include<camera.h>
#include<opencv2/opencv.hpp>
#include<vector>

//#define STEREO_CALIB_TEST
#define STEREO_IMAGE_GRAB

using namespace std;

int main(int argc, char* argv[])
{
#ifdef STEREO_CALIB_TEST

	vector<string> left_img_list;
	vector<string> right_img_list;
	
	left_img_list.push_back("./stereo_calib/left1.bmp");
	left_img_list.push_back("./stereo_calib/left2.bmp");
	left_img_list.push_back("./stereo_calib/left3.bmp");
	left_img_list.push_back("./stereo_calib/left4.bmp");
	left_img_list.push_back("./stereo_calib/left5.bmp");
	left_img_list.push_back("./stereo_calib/left6.bmp");
	left_img_list.push_back("./stereo_calib/left7.bmp");
	left_img_list.push_back("./stereo_calib/left8.bmp");
	left_img_list.push_back("./stereo_calib/left9.bmp");
	left_img_list.push_back("./stereo_calib/left10.bmp");
	left_img_list.push_back("./stereo_calib/left11.bmp");
	left_img_list.push_back("./stereo_calib/left12.bmp");
	left_img_list.push_back("./stereo_calib/left13.bmp");
	left_img_list.push_back("./stereo_calib/left14.bmp");

	right_img_list.push_back("./stereo_calib/right1.bmp");
	right_img_list.push_back("./stereo_calib/right2.bmp");
	right_img_list.push_back("./stereo_calib/right3.bmp");
	right_img_list.push_back("./stereo_calib/right4.bmp");
	right_img_list.push_back("./stereo_calib/right5.bmp");
	right_img_list.push_back("./stereo_calib/right6.bmp");
	right_img_list.push_back("./stereo_calib/right7.bmp");
	right_img_list.push_back("./stereo_calib/right8.bmp");
	right_img_list.push_back("./stereo_calib/right9.bmp");
	right_img_list.push_back("./stereo_calib/right10.bmp");
	right_img_list.push_back("./stereo_calib/right11.bmp");
	right_img_list.push_back("./stereo_calib/right12.bmp");
	right_img_list.push_back("./stereo_calib/right13.bmp");
	right_img_list.push_back("./stereo_calib/right14.bmp");
	/*
	left_img_list.push_back("../data/left01.jpg");
	left_img_list.push_back("../data/left02.jpg");
	left_img_list.push_back("../data/left03.jpg");
	left_img_list.push_back("../data/left04.jpg");
	left_img_list.push_back("../data/left05.jpg");
	left_img_list.push_back("../data/left06.jpg");
	left_img_list.push_back("../data/left07.jpg");
	left_img_list.push_back("../data/left08.jpg");
	left_img_list.push_back("../data/left09.jpg");
	left_img_list.push_back("../data/left10.jpg");
	left_img_list.push_back("../data/left11.jpg");
	left_img_list.push_back("../data/left12.jpg");
	left_img_list.push_back("../data/left13.jpg");
	left_img_list.push_back("../data/left14.jpg");

	right_img_list.push_back("../data/right01.jpg");
	right_img_list.push_back("../data/right02.jpg");
	right_img_list.push_back("../data/right03.jpg");
	right_img_list.push_back("../data/right04.jpg");
	right_img_list.push_back("../data/right05.jpg");
	right_img_list.push_back("../data/right06.jpg");
	right_img_list.push_back("../data/right07.jpg");
	right_img_list.push_back("../data/right08.jpg");
	right_img_list.push_back("../data/right09.jpg");
	right_img_list.push_back("../data/right10.jpg");
	right_img_list.push_back("../data/right11.jpg");
	right_img_list.push_back("../data/right12.jpg");
	right_img_list.push_back("../data/right13.jpg");
	right_img_list.push_back("../data/right14.jpg");
	*/
	Robot::StereoCalibration stereo_calib;
	stereo_calib.set_pattern(8, 11, 0.04);
	stereo_calib.imageSize = cv::Size(2048, 2048);
	//stereo_calib.set_pattern(6, 9, 0.04);
	//stereo_calib.imageSize = cv::Size(640, 480);
	stereo_calib.winSize = cv::Size(1, 1);

	for (int i = 0;i < 14;i++)
	{
		cv::Mat tmp1 = cv::imread(left_img_list[i]);
		cv::Mat tmp2 = cv::imread(right_img_list[i]);
		cv::Mat left_img, right_img;
		//std::cout << tmp1.channels() << " " << tmp2.channels() << std::endl;
		if (tmp1.channels() == 3)
		{
			left_img = tmp1;
		}
		else
		{
			//continue;
			left_img = cv::Mat(tmp1.size(), CV_8UC3);
			cv::cvtColor(tmp1, left_img, cv::COLOR_GRAY2RGB);
		}
		if (tmp2.channels() == 3)
		{
			right_img = tmp2;
		}
		else
		{
			//continue;
			right_img = cv::Mat(tmp2.size(), CV_8UC3);
			cv::cvtColor(tmp2, right_img, cv::COLOR_GRAY2RGB);
		}

		
		
		stereo_calib.add_images(left_img, right_img);

		cv::Mat view_rgb(left_img.rows / 4, left_img.cols / 4, CV_8UC3);
		cv::resize(left_img, view_rgb, cv::Size(), 0.25, 0.25);
		cv::Mat view_rgb1(right_img.rows / 4, right_img.cols / 4, CV_8UC3);
		cv::resize(right_img, view_rgb1, cv::Size(), 0.25, 0.25);
		cv::imshow("left", view_rgb);
		cv::imshow("right", view_rgb1);

		cv::imwrite("left_calib"+std::to_string(i)+".bmp", left_img);
		cv::imwrite("right_calib" + std::to_string(i) + ".bmp", right_img);

		cv::waitKey(30);
	}
	stereo_calib.run_calibration();
	stereo_calib.save_parameters("intrinsic.xml", "extrinsic.xml");

#endif // STEREO_CALIB_TEST

#ifdef STEREO_IMAGE_GRAB

	if (argc < 3)
	{
		printf("Please run this program like: stereo_calibration leftcam_sn rightcam_sn\n");
		return -1;
	}

	std::string left_sn(argv[1]);
	std::string right_sn(argv[2]);
	Robot::BaslerStereoCamera stereo_camera;
	stereo_camera.set_exposure(20000, 20000);
	stereo_camera.setup(left_sn, right_sn);
	// start the loop thread for grabing images
	stereo_camera.start();

	Robot::StereoCalibration stereo_calib;
	stereo_calib.set_pattern(8, 11, 0.04);
	stereo_calib.imageSize = cv::Size(2048, 2048);
	stereo_calib.winSize = cv::Size(1, 1);
	int counter = 0;
	std::vector<cv::Mat> left_imgs, right_imgs;
	std::string name_left = "./stereo_calib/left";
	std::string name_right = "./stereo_calib/right";
	cv::Mat view_total;
	while (true)
	{
		cv::Mat left_img, right_img;
		left_img = stereo_camera.get_left_image();
		right_img = stereo_camera.get_right_image();

		if (!left_img.data) continue;
		if (!right_img.data) continue;

		cv::Mat view(left_img.rows / 4, left_img.cols / 4, CV_8UC1);
		cv::resize(left_img, view, cv::Size(), 0.25, 0.25);
		cv::Mat view_rgb(left_img.rows / 4, left_img.cols / 4, CV_8UC3);
		cv::resize(left_img, view, cv::Size(), 0.25, 0.25);
		cv::cvtColor(view, view_rgb, cv::COLOR_GRAY2RGB);

		cv::Mat view1(left_img.rows / 4, left_img.cols / 4, CV_8UC1);
		cv::resize(right_img, view1, cv::Size(), 0.25, 0.25);
		cv::Mat view_rgb1(left_img.rows / 4, left_img.cols / 4, CV_8UC3);
		cv::resize(right_img, view1, cv::Size(), 0.25, 0.25);
		cv::cvtColor(view1, view_rgb1, cv::COLOR_GRAY2RGB);

		//stereo_calib.draw_pattern(view, view_rgb);
		//stereo_calib.draw_pattern(view1, view_rgb1);
		
		cv::hconcat(view_rgb, view_rgb1, view_total);
		cv::imshow("Stereo view", view_total);

		int key = cv::waitKey(100);
		
		if (key == 'q')
		{
			//cv::destroyAllWindows();
			stereo_camera.stop();
			break;
		}
		else if (key == 13)
		{
			printf("Add the %d pair of images ...\n", counter+1);
			//stereo_calib.add_images(left_img, right_img);
			//counter = stereo_calib.get_image_count();
			left_imgs.push_back(left_img);
			right_imgs.push_back(right_img);
			
			std::string num = std::to_string(counter);
			counter++;
			cv::imwrite(name_left + num + ".bmp", left_img);
			cv::imwrite(name_right + num + ".bmp", right_img);

			continue;
		}
		else
		{
			//printf("No respond ...\n");
		}
	}
	printf("Start calibration ...\n");
	for (int i = 0;i < left_imgs.size();i++)
	{
		stereo_calib.add_images(left_imgs[i], right_imgs[i]);
	}
	stereo_calib.run_calibration();
	stereo_calib.save_parameters("intrinsic.xml", "extrinsic.xml");
	
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
