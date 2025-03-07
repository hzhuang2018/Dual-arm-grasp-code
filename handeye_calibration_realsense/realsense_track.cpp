#include "realsense_track.h"
using namespace rs2;
cv::Mat image_gray;
extern double im3_actual_center[9];//3点图像实际像素位置
extern double im3_actual_radius[3];//3点图像实际像素半径
double im3_dirsed_center[6];//3点图像期望像素位置
//#define in_para;
void track(void)
{
    rs2::config cfg;

    ///配置彩色图像流：分辨率640*480，图像格式：BGR， 帧率：30帧/秒

    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);//1280 *720

    ///配置深度图像流：分辨率640*480，图像格式：Z16， 帧率：30帧/秒

    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    //const int width = 1280;
    //const int height = 720;
    //const int fps = 30;

    //rs2::config pipe_config;
    //pipe_config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
    //pipe_config.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
    //pipe_config.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
    //pipe_config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);


    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    // Configure and start the pipeline
    p.start(cfg);
    im3_dirsed_center[0] = 423.5; im3_dirsed_center[1] = 368.5;
    im3_dirsed_center[2] = 285; im3_dirsed_center[3] = 371.5;
    im3_dirsed_center[4] = 290.5; im3_dirsed_center[5] = 281.5;
    while (true)
    {
        //boost::posix_time::ptime start_time;// = boost::get_system_time();
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();

        // Try to get a frame of a depth image
        rs2::depth_frame depth = frames.get_depth_frame(); ///获取深度图像数据

        rs2::video_frame color = frames.get_color_frame(); ///获取彩色图像数据

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

        cv::Mat depthmat(cv::Size(depth_width, depth_height), CV_16U, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

        //使用灰度图像进行角点检测
        //cv::Mat image_gray;
        cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);//彩色图转灰度图

        vector<Vec3f> circles;//存储圆的容器
        HoughCircles(image_gray, circles, HOUGH_GRADIENT, 1, 10, 100, 30, 5, 50);//进行霍夫圆检测
        Scalar circleColor = Scalar(255, 0, 0);//圆形的边缘颜色
        Scalar centerColor = Scalar(0, 0, 255);//圆心的颜色
        Scalar desireColor = Scalar(0, 255, 0);//圆心的颜色
        if (circles.size() != 3)
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

        }
        for(int m=0;m<2;m++)
            for (int n = m+1; n < 3; n++)
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

        imshow("dst", image);//显示处理后的图像
        waitKey(1);
    }

}

void captual_frames()
{
}