#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include "dll_export.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <queue>
#include <unordered_set>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>

using pixel = std::pair<int, int>;

namespace Robot {

	class ROBOTIC_GRASP_DLL RealsenseCamera
	{
	public:
		RealsenseCamera();
		~RealsenseCamera();
		
		void start();
		cv::Mat get_data_rgb(cv::Mat& depthmat);
		cv::Point3f get_point_3d(pixel u);
		void set_frame_size(int width_ = 1280, int height_ = 720);
		void read_handeye_params(const std::string& filename_handeye);
		void read_handeye_intrinsic_params(const std::string& filename_handeye);
		//void read_handeye_API_intrinsic_params();
		cv::Mat Rcg, tcg,intrisic_K;
		int width ,height;//image w *h
	private:
		rs2::colorizer color_map;
		rs2::decimation_filter dec;
		rs2::disparity_transform depth2disparity;
		rs2::disparity_transform disparity2depth;
		rs2::spatial_filter spat;
		rs2::temporal_filter temp;
		rs2::align align_to_depth;
		rs2::align align_to_color;
		rs2::pipeline pipe;
		rs2::config cfg;
		rs2::frameset data;
		//rs2::depth_frame depth;
	};
}
