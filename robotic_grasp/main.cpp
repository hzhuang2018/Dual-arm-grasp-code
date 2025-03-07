#include<iostream>
#include<camera.h>
#include<opencv2\opencv.hpp>

//#define TEST_MONO_CAMERA
#define TEST_STEREO_CAMERA

int main(int argc, char* argv[])
{
#ifdef TEST_MONO_CAMERA

	Robot::BaslerCamera camera;
	camera.setup();
	camera.continuous_grabing();
	while (true)
	{
		cv::Mat img;
		camera.get_latest_image(img);
		if (!img.data) continue;
		cv::Mat view(img.rows / 4, img.cols / 4, CV_8UC1);
		cv::resize(img, view, cv::Size(), 0.25, 0.25);
		cv::Mat view_rgb(img.rows / 4, img.cols / 4, CV_8UC3);
		cv::resize(img, view, cv::Size(), 0.25, 0.25);
		cv::cvtColor(view, view_rgb, cv::COLOR_GRAY2RGB);
		cv::imshow("Test mono camera", view_rgb);

		int key = cv::waitKey(30);
		if (key == 27)
		{
			break;
		}
	}

#endif // TEST_MONO_CAMERA

#ifdef TEST_STEREO_CAMERA
	std::string left_sn("21876818");
	std::string right_sn("21876808");
	Robot::BaslerStereoCamera stereo_camera;
	stereo_camera.setup(left_sn, right_sn);
	// start the loop thread for grabing images
	stereo_camera.start();

	
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

		cv::imshow("Left", view_rgb);
		cv::imshow("Right", view_rgb1);

		int key = cv::waitKey(30);
		if (key == 27)
		{
			break;
		}
	}
#endif // TEST_STEREO_CAMERA

	
	return 0;
}