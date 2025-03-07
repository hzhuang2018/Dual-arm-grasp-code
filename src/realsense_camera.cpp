#include <realsense_camera.h>

namespace Robot {

    RealsenseCamera::RealsenseCamera(): disparity2depth(false), align_to_color(RS2_STREAM_COLOR)
        , align_to_depth(RS2_STREAM_DEPTH)
	{
        //set picture size
        width = 1280;
        height = 720;

        //configute camera
        //配置彩色图像流：分辨率640*480，图像格式：BGR， 帧率：30帧/秒
        cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 30);//1280 *720
        //配置深度图像流：分辨率640*480，图像格式：Z16， 帧率：30帧/秒
        cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);

        spat.set_option(RS2_OPTION_HOLES_FILL, 5);
        color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.f);
        dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
	}

    RealsenseCamera::~RealsenseCamera() {

	}

    void RealsenseCamera::set_frame_size(int width_ , int height_ )
    {
        width = width_;
        height = height_;
    }

	void RealsenseCamera::start() {

		auto profile = pipe.start(cfg);
		auto sensor = profile.get_device().first<rs2::depth_sensor>();
		if (sensor && sensor.is<rs2::depth_stereo_sensor>())
		{
			sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
		}
	}

    cv::Point3f RealsenseCamera::get_point_3d(pixel u)
    {
        auto frame = data.get_depth_frame();

        float upixel[2];
        float upoint[3];
        upixel[0] = static_cast<float>(u.first);
        upixel[1] = static_cast<float>(u.second);

        auto udist = frame.get_distance(static_cast<int>(upixel[0]), static_cast<int>(upixel[1]));
        rs2_intrinsics intr = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
        rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);

        cv::Point3f point_3d;
        point_3d.x = upoint[0];
        point_3d.y = upoint[1];
        point_3d.z = upoint[2];
        return point_3d;
    }

	cv::Mat RealsenseCamera::get_data_rgb(cv::Mat & depthmat) {

        // Block program until frames arrive
        data = pipe.wait_for_frames();
        // Align to depth 
        data = align_to_color.process(data);

        // Try to get a frame of a depth image
        rs2::depth_frame depth = data.get_depth_frame(); ///获取深度图像数据
        rs2::frame color = data.get_color_frame(); ///获取彩色图像数据

        // if (!color || !depth) break; ///如果获取不到数据则退出

      ///将彩色图像和深度图像转换为Opencv格式
        cv::Mat image_gray;
        cv::Mat image(cv::Size(width, height), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        //cv::initUndistortRectifyMap();
        // depthmat.size(width, height);
        cv::Mat depth_temp(cv::Size(width, height), CV_16U, (void*)depth.get_data(), cv::Mat::AUTO_STEP);//CV_16U
        depth_temp.copyTo(depthmat);
        //namedWindow("Display Image", WINDOW_AUTOSIZE);
        cv::imshow("Display 1 Image", image);
        cv::waitKey(1);
        //namedWindow("Display depth", WINDOW_AUTOSIZE);
        cv::imshow("Display depth", depthmat*15);
        cv::waitKey(1);
        return image;








        /*
        //rs2::frameset data;
        data = pipe.wait_for_frames();

        // First make the frames spatially aligned
        data = data.apply_filter(align_to);

        // Decimation will reduce the resultion of the depth image,
        // closing small holes and speeding-up the algorithm
        //data = data.apply_filter(dec); //此操作会使depth.get_distance()提取深度大概率失败

        // To make sure far-away objects are filtered proportionally
        // we try to switch to disparity domain
        data = data.apply_filter(depth2disparity);

        // Apply spatial filtering
        data = data.apply_filter(spat);

        // Apply temporal filtering
        data = data.apply_filter(temp);

        // If we are in disparity domain, switch back to depth
        data = data.apply_filter(disparity2depth);

        //// Apply color map for visualization of depth
        data = data.apply_filter(color_map);

        // Send resulting frames for visualization in the main thread
        //postprocessed_frames.enqueue(data);

        auto depth = data.get_depth_frame();
        auto color = data.get_color_frame();
        //auto colorized_depth = data.first(RS2_STREAM_DEPTH, RS2_FORMAT_RGB8);

        auto w = color.get_width();
        auto h = color.get_height();
        cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        //cv::imshow("image", image);

        return image;
        */
	}

    void RealsenseCamera::read_handeye_params(const std::string& filename_handeye) {
        cv::FileStorage fs;
        fs.open(filename_handeye, cv::FileStorage::READ);
        if (fs.isOpened())
        {
            Rcg = fs["Rcg"].mat();
            tcg = fs["tcg"].mat();
            fs.release();
        }
        else
        {
            printf("can't read handeye parameters.../n");
        }
    }
    void RealsenseCamera::read_handeye_intrinsic_params(const std::string& filename_handeye) {
        cv::FileStorage fs;
        fs.open(filename_handeye, cv::FileStorage::READ);
        if (fs.isOpened())
        {
            intrisic_K = fs["K"].mat();
            fs.release();
        }
        else
        {
            printf("can't read handeye parameters.../n");
        }
    }
  //  void RealsenseCamera::read_handeye_API_intrinsic_params()
}