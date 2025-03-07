#pragma once
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>             // for cout
#include <opencv2\opencv.hpp>
#include <conio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <cartesian_space_track_ur.h>
#include "realsense_camera.h";
#include "algorithm";
using namespace Robot;
using namespace cv;
using namespace std;
void track(CartesianSpaceTrackUR* controller_ur);
void track_ibvs(CartesianSpaceTrackUR* controller_ur);
void track_redblue_box(CartesianSpaceTrackUR* controller_ur);
void track_chessboard(CartesianSpaceTrackUR* controller_ur);
void gen_pointcloud(RealsenseCamera* rs_, CartesianSpaceTrackUR* controller_ur);

