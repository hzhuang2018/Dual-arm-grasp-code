#include "realsense_track.h"
using namespace rs2;
cv::Mat image_gray;
extern double im3_actual_center[12];//3点图像实际像素位置
extern double im3_actual_radius[4];//3点图像实际像素半径
double im3_dirsed_center[8];//3点图像期望像素位置
//#define in_para;

void track(void)
{
    //set realsense
    rs2::config cfg;
    colorizer color_map;
    ///配置彩色图像流：分辨率640*480，图像格式：BGR， 帧率：30帧/秒
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);//1280 *720
    ///配置深度图像流：分辨率640*480，图像格式：Z16， 帧率：30帧/秒
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    // Configure and start the pipeline
    p.start(cfg);


    //rs2::align align_to_depth(RS2_STREAM_DEPTH);
    //rs2::align align_to_color(RS2_STREAM_COLOR);

    im3_dirsed_center[0] = 470; im3_dirsed_center[1] = 430;
    im3_dirsed_center[2] = 456; im3_dirsed_center[3] = 262;
    im3_dirsed_center[4] = 679; im3_dirsed_center[5] = 275;
    im3_dirsed_center[6] = 674; im3_dirsed_center[7] = 430;
    while (true)
    {
        //boost::posix_time::ptime start_time;// = boost::get_system_time();
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();

        // Try to get a frame of a depth image
        rs2::depth_frame depth = frames.get_depth_frame(); ///获取深度图像数据
        //frame depth_frame = frames.get_depth_frame().apply_filter(color_map);
        rs2::video_frame color = frames.get_color_frame(); ///获取彩色图像数据

      //  rs2::frameset frameset = pipe.wait_for_frames();



            // Align all frames to depth viewport
            //frames = align_to_depth.process(frames);


            // Align all frames to color viewport
            //frames = align_to_color.process(frames);
     

        // With the aligned frameset we proceed as usual
        //auto depth = frames.get_depth_frame();
        //auto color = frames.get_color_frame();
        //rs2::colorizer c;
        //auto colorized_depth = c.colorize(depth);



        int color_width = color.get_width();
        int color_height = color.get_height();
        int depth_width = depth.get_width();
        int depth_height = depth.get_height();


        // Get the depth frame's dimensions
        auto width = depth.get_width();
        auto height = depth.get_height();

        //获得内参
#ifdef  in_para
        for (int i = 1; i < 20; i++)
            p.wait_for_frames();
        frames = p.wait_for_frames();

        auto color1 = frames.get_color_frame();
        //auto depth = frames.get_depth_frame();

        stream_profile color_profile = color1.get_profile();

        auto rvsprofile = video_stream_profile(color_profile);
        //auto dvsprofile = video_stream_profile(depth_profile);

        rs2_intrinsics a = rvsprofile.get_intrinsics();
        //rs2_intrinsics a = dvsprofile.get_intrinsics();

        cout << "fx:\t" << a.fx << endl;
        cout << "fy:\t" << a.fy << endl;
        cout << "ppx:\t" << a.ppx << endl;
        cout << "ppy:\t" << a.ppy << endl;
#endif //  in_para


        if (!color || !depth) break; ///如果获取不到数据则退出

///将彩色图像和深度图像转换为Opencv格式

        cv::Mat image(cv::Size(color_width, color_height), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        //cv::initUndistortRectifyMap();
        //cv::Mat depthmat(cv::Size(depth_width, depth_height), CV_16U, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
        if (!image.data) continue;
        //使用灰度图像进行角点检测
        //cv::Mat image_gray;
        cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);//彩色图转灰度图

        vector<Vec3f> circles;//存储圆的容器
        HoughCircles(image_gray, circles, HOUGH_GRADIENT, 1, 80, 150, 40, 5, 80);//进行霍夫圆检测
        Scalar circleColor = Scalar(255, 0, 0);//圆形的边缘颜色
        Scalar centerColor = Scalar(0, 0, 255);//圆心的颜色
        Scalar desireColor = Scalar(0, 255, 0);//圆心的颜色
        if (circles.size() != 4)
            continue;
        for (int i = 0; i < circles.size(); i++) {
            Vec3f c = circles[i];
            float dist_to_center = depth.get_distance(c[0], c[1]);
            im3_actual_center[i*3 + 0] = c[0];
            im3_actual_center[i*3 + 1] = c[1];
            im3_actual_radius[i] = c[2];
            im3_actual_center[i*3 + 2] = dist_to_center;
           // cout << "center:" << c[0] << "  " << c[1] << "  "<<c[2]<<"  " << "distance:=" << dist_to_center << endl;
            circle(image, Point(c[0], c[1]), c[2], circleColor, 2, LINE_AA);//画边缘
            circle(image, Point(c[0], c[1]), 2, centerColor, 2, LINE_AA);//画圆心
            circle(image, Point(im3_dirsed_center[0], im3_dirsed_center[1]), 2, desireColor, 2, LINE_AA);//画期望圆心一
            circle(image, Point(im3_dirsed_center[2], im3_dirsed_center[3]), 2, desireColor, 2, LINE_AA);//画期望圆心二
            circle(image, Point(im3_dirsed_center[4], im3_dirsed_center[5]), 2, desireColor, 2, LINE_AA);//画期望圆心三
            circle(image, Point(im3_dirsed_center[6], im3_dirsed_center[7]), 2, desireColor, 2, LINE_AA);//画期望圆心三

        }
        //圆半径排序
        for(int m=0;m<3;m++)
            for (int n = m+1; n < 4; n++)
            {
                if (im3_actual_radius[m] < im3_actual_radius[n])
                {
                    double  tempX = im3_actual_center[m*3];
                    double  tempY = im3_actual_center[m*3+1];
                    im3_actual_center[m*3] = im3_actual_center[n*3];
                    im3_actual_center[m*3 + 1]= im3_actual_center[n * 3+1];
                    im3_actual_center[n * 3] = tempX;
                    im3_actual_center[n * 3 + 1] = tempY;
                }
            }
        //im3_actual_center[2] =
        //im3_actual_center[0], im3_actual_center[1]
        //设置中心点附近的一个区域,用来求距离
        for (int jk = 0; jk < 4; jk++)
        {
            int x_min, x_max, y_min, y_max;
            if ((im3_actual_center[jk*3+0] - 10) > 0)
                x_min = (int)(im3_actual_center[jk * 3 + 0] - 10);
            else
                x_min = 0;
            if ((im3_actual_center[jk * 3 + 0] + 10) < 1279)
                x_max = (int)(im3_actual_center[jk * 3 + 0] + 10);
            else
                x_max = 1279;

            if ((im3_actual_center[jk * 3 + 1] - 10) > 0)
                y_min = (int)(im3_actual_center[jk * 3 + 1] - 10);
            else
                y_min = 0;
            if ((im3_actual_center[jk * 3 + 1] + 10) < 719)
                y_max = (int)(im3_actual_center[jk * 3 + 1] + 10);
            else
                y_max = 719;

            for (int i = x_min; i < x_max; i++)
                for(int j = y_min;j<y_max;j++)
                {
                if( im3_actual_center[jk * 3 + 2]< depth.get_distance(i, j))
                im3_actual_center[jk * 3 + 2] = depth.get_distance(i, j);
                }

        }
        imshow("dst", image);//显示处理后的图像
        waitKey(1);
    }

}

void captual_frames()
{
}