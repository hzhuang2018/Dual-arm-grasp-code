#include "realsense_position.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <opencv2/core/eigen.hpp>
#include <omp.h> // OpenMP编程需要包含的头文件    
using namespace rs2;
double im3_actual_center_temp[12];//临时存储3点图像实际像素位置
double im3_actual_center[12];//3点图像实际像素位置
double im3_actual_radius[4];//3点图像实际像素半径
double im3_dirsed_center[8];//3点图像期望像素位置
double redbox_des_center[6];//矩形中心x/y/z,rx,ry,rz
cv::Point3d rectangel_point[4];//存储的4个3D矩形顶点
cv::RotatedRect rotatedrect;
float api_camara[3];
//#define in_para
#define red_box_
//#define blue_box
cv::Mat depthmatR;
bool cam_ivbs_error=true;
bool cam_redbox_error  =true;
extern octomap::OcTree envitree;  // create empty tree with resolution 0.1
Eigen::Matrix4d external_Mat;
Eigen::Matrix<double, 3, 3> intenal_Mat;
Eigen::Matrix3d m_rot;
Eigen::Matrix4d Mobj_cama;
extern bool running;
using namespace octomap;
void LargestConnecttedComponent(Mat srcImage, Mat& dstImage)
{
    Mat temp;
    Mat labels;
    srcImage.copyTo(temp);

    //1. 标记连通域
    int n_comps = connectedComponents(temp, labels, 4, CV_16U);
    vector<int> histogram_of_labels;
    for (int i = 0; i < n_comps; i++)//初始化labels的个数为0
    {
        histogram_of_labels.push_back(0);
    }

    int rows = labels.rows;
    int cols = labels.cols;
    for (int row = 0; row < rows; row++) //计算每个labels的个数
    {
        for (int col = 0; col < cols; col++)
        {
            histogram_of_labels.at(labels.at<unsigned short>(row, col)) += 1;
        }
    }
    histogram_of_labels.at(0) = 0; //将背景的labels个数设置为0

    //2. 计算最大的连通域labels索引
    int maximum = 0;
    int max_idx = 0;
    for (int i = 0; i < n_comps; i++)
    {
        if (histogram_of_labels.at(i) > maximum)
        {
            maximum = histogram_of_labels.at(i);
            max_idx = i;
        }
    }

    //3. 将最大连通域标记为1
    for (int row = 0; row < rows; row++)
    {
        for (int col = 0; col < cols; col++)
        {
            if (labels.at<unsigned short>(row, col) == max_idx)
            {
                labels.at<unsigned short>(row, col) = 255;
            }
            else
            {
                labels.at<unsigned short>(row, col) = 0;
            }
        }
    }

    //4. 将图像更改为CV_8U格式
    labels.convertTo(dstImage, CV_8U);
}


void cameraPointCloud(Eigen::Matrix4d external_T, Eigen::Matrix3d intrisci_M, const  cv::Mat depth_mat, octomap::OcTree& tree)
{

    double camera_factor = 1000.0;
    octomap::point3d p, p2;
    octomap::Pointcloud octPointCloud;
    Eigen::Vector3d p_temp;
    Eigen::Vector4d p_temp2(0,0,0,1);
    double intric_m_0_0 = intrisci_M(0, 0);
    double intric_m_1_1 = intrisci_M(1, 1);
    double intric_m_0_2 = intrisci_M(0, 2);
    double intric_m_1_2 = intrisci_M(1, 2);

    Eigen::Matrix3d ex_R = external_T.block<3, 3>(0, 0);
    Eigen::Vector3d eulerAngle1 = ex_R.eulerAngles(2, 1, 0);
    Eigen::Vector3d ex_T = external_T.block<3, 1>(0, 3);
    for (int m = 0; m < depth_mat.rows; m++) {
        for (int n = 0; n < depth_mat.cols; n++) {
            // 获取深度图中(m,n)处的值
            ushort d = depth_mat.ptr<ushort>(m)[n];  //float d = depth_pic.ptr<float>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d < 200 or d > 1500)
                continue;
            // d 存在值，则向点云增加一个点

            // 计算这个点的空间坐标
            p_temp2(2) = double(d) / camera_factor;
            p_temp2(0) = (n - intric_m_0_2) * p_temp2.z() / intric_m_0_0;
            p_temp2(1) = (m - intric_m_1_2) * p_temp2.z() / intric_m_1_1;
            //p_temp2(3) = 1;
            //p_temp2(0) = p_temp(0); p_temp2(1) = p_temp(1); p_temp2(2) = p_temp(2); p_temp2(3) = 1;
           // Eigen::Vector3d p_cama;
           // p_cama(0) = p.x(); p_cama(1) = p.y(); p_cama(2) = p.z();
          // Eigen::Vector3d APIwcPoint1 = ex_R * (p_temp)+ex_T;//api_ip
          // Eigen::Vector4d APIwcPoint2 = external_T * p_temp2;

          // p2.x() = APIwcPoint1(0); p2.y() = APIwcPoint1(1); p2.z() = APIwcPoint1(2);
           //p2.x() = APIwcPoint2(0); p2.y() = APIwcPoint2(1); p2.z() = APIwcPoint2(2);
            p2.x() = p_temp2(0); p2.y() = p_temp2(1); p2.z() = p_temp2(2);
            //if (p2.z() < -0.2)
            //    continue;
            //tree.updateNode(p2, true); // integrate 'occupied' measurement
            //tree.updateInnerOccupancy();
            //tree.insertRay(octomap::point3d(0, 0, 0), p2);
            
             //octomap::point3d origin(0.0, 0.0, 0.0);//应用相机原点
          
          octPointCloud.push_back(p2);

        }
    }
    octomap::point3d origin(0.0, 0.0, 0.0);//应用相机原点
    octomap::pose6d frame_origin(external_T(0,3), external_T(1, 3), external_T(2, 3), eulerAngle1(0), eulerAngle1(1), eulerAngle1(2));//应用相机原点
    //origin.x() = external_T(0, 3); origin.y() = external_T(1, 3); origin.z() = external_T(2, 3);
    tree.clear();
    tree.insertPointCloud(octPointCloud, origin, frame_origin,2.5,true,true);
    tree.updateInnerOccupancy();
    envitree.writeBinary("envitree_total.bt");
    envitree.write("envitree_total.ot");
}

void track(CartesianSpaceTrackUR* controller_ur)
{
    boost::posix_time::ptime start_time1 = boost::get_system_time();
    double d_ms1;
    boost::posix_time::ptime cur_time1, circle_end_time1;
    boost::posix_time::time_duration d_time1, circle_time1;
    external_Mat << 9.9999182276709764e-01, -2.5015706196520101e-03, -3.1775058414804393e-03, -3.6770868325372012e-02,
        2.4858905285792821e-03, 9.9998476195823494e-01, -4.9291175287571806e-03, -7.5690552499139668e-02,
        3.1897879581044689e-03, 4.9211782905393027e-03, 9.9998280348064750e-01, -2.5302551302136194e-02,
        0, 0, 0, 1;

    intenal_Mat << 8.8884850236407863e+02, 0., 6.4873567892875235e+02,//637.8,0,637.8,
                   0., 8.8769039996290348e+02, 3.6643649723707466e+02,
                   0., 0., 1.;

    rs2::colorizer color_map;
    rs2::decimation_filter dec;
    rs2::disparity_transform depth2disparity;
    rs2::disparity_transform disparity2depth;
    rs2::spatial_filter spat;
    rs2::temporal_filter temp;
    rs2::align align_to(RS2_STREAM_COLOR);
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::frameset data;
    //set realsense
   // rs::texture depth_image, color_image;     // Helpers for renderig images
    rs2::colorizer c;         // Helper to colorize depth images
    color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.f);
    dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    spat.set_option(RS2_OPTION_HOLES_FILL, 5);
   // cfg.enable_stream(RS2_STREAM_DEPTH);
    //cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGB8);
    disparity2depth = false;
    int width = 1280;
    int height = 720;
    ///配置彩色图像流：分辨率640*480，图像格式：BGR， 帧率：30帧/秒
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 30);//1280 *720
    ///配置深度图像流：分辨率640*480，图像格式：Z16， 帧率：30帧/秒
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    // Configure and start the pipeline
    p.start(cfg);

    while (true)
    {
        //boost::posix_time::ptime start_time;// = boost::get_system_time();
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();
        // Align to depth 
        //rs2::align align_to_depth(RS2_STREAM_DEPTH);
        rs2::align align_to_depth(RS2_STREAM_DEPTH);
        rs2::align align_to_color(RS2_STREAM_COLOR);
       // frames = align_to_depth.process(frames);

        frames = align_to_color.process(frames);

        // Try to get a frame of a depth image
        rs2::depth_frame depth = frames.get_depth_frame(); ///获取深度图像数据
        //frame depth_frame = frames.get_depth_frame().apply_filter(color_map);
        rs2::frame color = frames.get_color_frame(); ///获取彩色图像数据


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

       // if (!color || !depth) break; ///如果获取不到数据则退出

///将彩色图像和深度图像转换为Opencv格式
        cv::Mat image_gray;
        cv::Mat image(cv::Size(width, height), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        //cv::initUndistortRectifyMap();
        cv::Mat depthmat(cv::Size(width, height), CV_16U, (void*)depth.get_data(), cv::Mat::AUTO_STEP);//CV_16U
        if (!image.data ) continue;
        if (!depthmat.data) continue;
       // cout << "size:=" << depthmat.size() << endl;;
        controller_ur->update_p(boost::get_system_time(), boost::posix_time::milliseconds(8));
        depthmat.copyTo(depthmatR);
        cur_time1 = boost::get_system_time();
        d_time1 = cur_time1 - start_time1;
        d_ms1 = (double)d_time1.total_milliseconds() / 1000;
        //printf("function begin time: %f\n", d_ms1);
        //cameraPointCloud(slave_robot_tool_to_base * external_Mat, intenal_Mat, depthmatR, envitree);
        //cameraPointCloud(controller_ur->Ttool_to_base * external_Mat, intenal_Mat, depthmatR, envitree);
        cur_time1 = boost::get_system_time();
        d_time1 = cur_time1 - start_time1;
        d_ms1 = (double)d_time1.total_milliseconds() / 1000;
        //printf("function engd time: %f\n", d_ms1);

        envitree.writeBinary("envitree_total.bt");
        envitree.write("envitree_total.ot");
        //将深度图归一化显示
        vector<cv::Point> target_points;

        // Display in a GUI
        //namedWindow("Display Image", WINDOW_AUTOSIZE);
        //cv::imshow("Display 1 Image", image);
        //cv::waitKey(1);
        //cv::imshow("Display depth", depthmat*15);
        //cv::waitKey(1);
        continue;

        //for (int i = 0; i < depthmatR.rows; i++)
        //{
        //    for (int j = 780; j < depthmatR.cols; j++)
        //    {
        //        if (depthmatR.at<uint>(i, j) <670 && depthmatR.at<uint>(i, j) > 600)
        //        {
        //            //取出灰度图像中i行j列的点
        //            //depthmatR.at<int>(i, j) = 200;
        //            cv::Point p;
        //            p.x = i;
        //            p.y = j;
        //            target_points.push_back(p);
        //            cout << "depth:=" << depthmatR.at<uint>(i, j) << i << " "<<j << endl;
        //        }

        //    }
        //}
       

        //    //相关操作
        //    //(*it)[0] = 255;//(*it)[0] * 255 / 1000 / 3;//除1000换算到米,再除3归一化到255
        //    //(*it)[1] = 0;
        //    //(*it)[2] = 0;
        //    it++;
        //}

        //使用灰度图像进行角点检测
        //cv::Mat red_image;
        int roi_startX = 780;
        Mat image_back = image;
        Mat imageROI = image(Rect(roi_startX, 0, 500, 720));
        Mat depth_imageROI = depthmatR(Rect(roi_startX, 0, 500, 720));

        //计算中心
        vector<Mat> channels;
        Mat grayROI;
        Mat imageBlueChannel;
        Mat imageGreenChannel;
        Mat imageRedChannel;
        split(imageROI, channels);//分离色彩通道
        imageBlueChannel = channels.at(0);
        imageGreenChannel = channels.at(1);
        imageRedChannel = channels.at(2);
       // grayROI = imageROI.
        Mat binaryImage, binaryImage1;
        threshold(imageRedChannel, binaryImage, 150, 255, THRESH_TOZERO);//THRESH_BINARY | THRESH_OTSU
        threshold(binaryImage, binaryImage1, 230, 255, THRESH_TOZERO_INV);//THRESH_BINARY | THRESH_OTSU
        //获取自定义核
        Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));//MORPH_RECT
        Mat erode_out,dilate_out;

        //进行腐蚀操作
        erode(binaryImage1, erode_out, element);
        Mat erode_out1, erode_out2;
       // LargestConnecttedComponent(erode_out, erode_out1);
       //LargestConnecttedComponent(erode_out1, erode_out2);
        //膨胀操作
        dilate(erode_out, dilate_out, element);
        //imshow("erode_out", erode_out);//显示处理后的图像


        //找联通域
        Mat labels = Mat::zeros(image.size(), CV_32S);
        Mat stats, centroids;
        int num_labels = connectedComponentsWithStats(erode_out, labels, stats, centroids, 8, 4);
        Vec2d pt;
        int num_checked;
        int x0, y0, x1, y1, w, h,area;
        if (num_labels > 2)
            num_checked = 4;
        for (int k = 0; k < 1; k++) {

                x0 = centroids.at<double>(k, 0);
                y0 = centroids.at<double>(k, 1);
                x1 = stats.at<int>(k, 0);
                y1 = stats.at<int>(k, 1);
                w = stats.at<int>(k, 2);
                h = stats.at<int>(k, 3);
                area = stats.at<int>(k, 4);
               // printf("Comp %2d: (%3d×%3d) from (%3d,%3d) 质心(%3d,%3d)\n", k, w, h, x1, y1, x0, y0);
     /*           if (area > 200)
                    continue;*/

            //stats
            pt = centroids.at<Vec2d>(k, 0);
            im3_actual_center[k * 3 + 0] = pt[0] + roi_startX;
            im3_actual_center[k * 3 + 1] = pt[1];
            im3_actual_radius[k] = 10;
            //im3_actual_center[k * 3 + 2] =  depth.get_distance(c[0], c[1]);;
            // cout << "center:" << c[0] << "  " << c[1] << "  "<<c[2]<<"  " << "distance:=" << dist_to_center << endl;
           // circle(image, Point(pt[0] + roi_startX, pt[1]), 10, (0,0,255), 2, LINE_AA);//画边缘
           // circle(image, Point(pt[0] + roi_startX, pt[1]), 2, (0, 255, 0), 2, LINE_AA);//画圆心
        }
        Mat bImage;
        cv::cvtColor(imageROI, image_gray, cv::COLOR_BGR2GRAY);//彩色图转灰度图
        threshold(image_gray, bImage, 120, 255, THRESH_BINARY );//| THRESH_OTSU
        vector<Vec3f> circles;//存储圆的容器
       // HoughCircles(image_back, circles, HOUGH_GRADIENT, 1, 10, 100, 50, 5, 50);//进行霍夫圆检测
         adaptiveThreshold(imageRedChannel, binaryImage,255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 25, 10);   

        cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);//彩色图转灰度图       
        //vector<Vec3f> circles;//存储圆的容器
        HoughCircles(erode_out, circles, HOUGH_GRADIENT, 1.5, 10, 100, 300, 20, 50);//进行霍夫圆检测
        Scalar circleColor = Scalar(255, 0, 0);//圆形的边缘颜色
        Scalar centerColor = Scalar(255, 0, 0);//圆心的颜色
        Scalar desireColor = Scalar(255, 0, 0);//圆心的颜色
        //if (circles.size() !=1)
        //    continue;
        //for (auto per_point = target_points.begin(); per_point != target_points.end(); per_point++)
        //{
        //    image.at<Vec3b>(per_point->x, per_point->y)[1] = 255;
        //}
        rs2_intrinsics intr;// = color.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
        intr = { 1280,720 ,6.4873567892875235e+02 ,3.6643649723707466e+02 ,
        8.8884850236407863e+02 ,8.8769039996290348e+02 ,RS2_DISTORTION_NONE,{1.6477064591350837e-01, - 5.5084560380708281e-01,
         2.1098851537153015e-03, - 8.3430941555631818e-04,    5.3144730038638299e-01} };
        float pixelpos[2], wordpos[3];
        //pixelpos[0] = 832; pixelpos[1] = 446;
        //rs2_deproject_pixel_to_point(api_camara, &intr, pixelpos, dist_to_center);//dist_to_center
       
      //  cout << "Api get the camera corrdinate:" << wordpos[0] << wordpos[1] << wordpos[2] << endl;
        //if (circles.size() > 3)
           continue;
        double centerP[3] = { 0,0,0 };
        uint16_t dist_to_center =0;
        for (int i = 0; i < circles.size(); i++) {
            Vec3f c = circles[i];
           float dist_to_center = depth.get_distance(c[0]+ roi_startX, c[1]);//depth.get_distance
            //  uint16_t depth_value=depth.at<uint16_t>(row,col);
            //dist_to_center = depth.get_distance()
            im3_actual_center[i*3 + 0] = c[0]+ roi_startX;
            im3_actual_center[i*3 + 1] = c[1];
            im3_actual_center[i*3 + 2] = dist_to_center;
            //im3_actual_radius[i] = dist_to_center;
          //  im3_actual_center[i*3 + 2] = dist_to_center;
           //cout << "center:" << c[0] << "  " << c[1] << "  "<<c[2]<<"  " << "distance:=" << dist_to_center << endl;
           circle(image_back, Point(c[0]+ roi_startX, c[1]), c[2], circleColor, 2, LINE_AA);//画边缘
           circle(image_back, Point(c[0]+ roi_startX, c[1]), 2, centerColor, 2, LINE_AA);//画圆心
            //circle(image, Point(im3_dirsed_center[0], im3_dirsed_center[1]), 2, desireColor, 2, LINE_AA);//画期望圆心一
            // circle(image, Point(im3_dirsed_center[2], im3_dirsed_center[3]), 2, desireColor, 2, LINE_AA);//画期望圆心二
            //circle(image, Point(im3_dirsed_center[4], im3_dirsed_center[5]), 2, desireColor, 2, LINE_AA);//画期望圆心三
            //circle(image, Point(im3_dirsed_center[6], im3_dirsed_center[7]), 2, desireColor, 2, LINE_AA);//画期望圆心三
           //rs2_intrinsics intr = color.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
           //float pixelpos[2], wordpos[3];
            pixelpos[0] = im3_actual_center[0]; pixelpos[1] = im3_actual_center[1];
          
            //设置中心点附近的一个区域,用来求距离
            {


                int x_min, x_max, y_min, y_max;
                if ((im3_actual_center[0] - 10) > 0)
                    x_min = (int)(im3_actual_center[0] - 10);
                else
                    x_min = 0;
                if ((im3_actual_center[0] + 10) < 1279)
                    x_max = (int)(im3_actual_center[0] + 10);
                else
                    x_max = 1279;

                if ((im3_actual_center[1] - 10) > 0)
                    y_min = (int)(im3_actual_center[1] - 10);
                else
                    y_min = 0;
                if ((im3_actual_center[1] + 10) < 719)
                    y_max = (int)(im3_actual_center[1] + 10);
                else
                    y_max = 719;

                for (int i = x_min; i < x_max; i++)
                    for (int j = y_min; j < y_max; j++)
                    {
                        
                        if ((im3_actual_center[2] < depth.get_distance(i, j)) && (im3_actual_center[2] > 20.0))
                          im3_actual_center[2] = depth.get_distance(i, j);
                    }
            }
           // cout << "center:" << pixelpos[0] << "  " << pixelpos[1] << "  " << c[2] << "  " << "distance:=" << dist_to_center << endl;

           //cout <<"Api get the camera corrdinate:" <<wordpos[0] << wordpos[1] << wordpos[2] << endl;
        }

        rs2_deproject_pixel_to_point(api_camara, &intr, pixelpos, im3_actual_center[2]);


        cv::imshow("dst1", image_back);//显示处理后的图像
        //imshow("dst2", depthmat);
        
        waitKey(1);
    }
    
}

void gen_pointcloud(RealsenseCamera *rs_, CartesianSpaceTrackUR* controller_ur)
{
    Eigen::Matrix4d slave_robot_tool_to_base, rob_left_base_to_rob_right_base, slave_base_to_master_base;
    slave_robot_tool_to_base << 0.15576, 0.830617, -0.534617, -0.208531,
        0.987755, -0.135816, 0.0767681, -0.127554,
        -0.0088448, -0.540028, -0.841601, 1.237,
        0, 0, 0, 1;
    rob_left_base_to_rob_right_base << 0.981456, 0.190843, 0.0179473, -1.90731,
        -0.190929, 0.981599, 0.00321324, 0.208629,
        -0.0170039, -0.00658033, 0.999834, 0.0112472,
        0, 0, 0, 1;
    slave_base_to_master_base = rob_left_base_to_rob_right_base.inverse();
    //cout << slave_base_to_master_base << endl;


    Eigen::Matrix3d external_R;
    Eigen::Vector3d external_T;
    external_Mat= Eigen::Matrix4d::Identity();
    cv2eigen(rs_->intrisic_K, intenal_Mat);
    cv2eigen(rs_->Rcg, external_R);
    cv2eigen(rs_->tcg, external_T);
    external_Mat.block<3, 3>(0, 0) = external_R;
    external_Mat.block<3, 1>(0, 3) = external_T;
    boost::posix_time::ptime start_time1 = boost::get_system_time();
    while (running)
    {
        rs_->get_data_rgb(depthmatR);
        double d_ms1;
        boost::posix_time::ptime cur_time1, circle_end_time1;
        boost::posix_time::time_duration d_time1, circle_time1;
        cur_time1 = boost::get_system_time();
        d_time1 = cur_time1 - start_time1;
        d_ms1 = (double)d_time1.total_milliseconds() / 1000;
        printf("function begin time: %f\n", d_ms1);
        controller_ur->update_p(boost::get_system_time(), boost::posix_time::milliseconds(8));
        //cout << slave_base_to_master_base * slave_robot_tool_to_base * external_Mat << endl;
        //cameraPointCloud(slave_base_to_master_base* slave_robot_tool_to_base * external_Mat, intenal_Mat, depthmatR, envitree);
        //cameraPointCloud(controller_ur->Ttool_to_base * external_Mat, intenal_Mat, depthmatR, envitree);
        cur_time1 = boost::get_system_time();
        d_time1 = cur_time1 - start_time1;
        d_ms1 = (double)d_time1.total_milliseconds() / 1000;
        printf("function engd time: %f\n", d_ms1);
    }

}


///////
void track_ibvs(CartesianSpaceTrackUR* controller_ur)
{
    boost::posix_time::ptime start_time1 = boost::get_system_time();
    double d_ms1;
    boost::posix_time::ptime cur_time1, circle_end_time1;
    boost::posix_time::time_duration d_time1, circle_time1;

    intenal_Mat << 8.8884850236407863e+02, 0., 6.4873567892875235e+02,//637.8,0,637.8,
        0., 8.8769039996290348e+02, 3.6643649723707466e+02,
        0., 0., 1.;


    rs2::colorizer color_map;
    rs2::decimation_filter dec;
    rs2::disparity_transform depth2disparity;
    rs2::disparity_transform disparity2depth;
    rs2::spatial_filter spat;
    rs2::temporal_filter temp;
    rs2::align align_to(RS2_STREAM_COLOR);
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::frameset data;
    //set realsense
   // rs::texture depth_image, color_image;     // Helpers for renderig images
    rs2::colorizer c;         // Helper to colorize depth images
    color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.f);
    dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    spat.set_option(RS2_OPTION_HOLES_FILL, 5);
    // cfg.enable_stream(RS2_STREAM_DEPTH);
     //cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGB8);
    disparity2depth = false;
    int width = 1280;
    int height = 720;
    ///配置彩色图像流：分辨率640*480，图像格式：BGR， 帧率：30帧/秒
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 30);//1280 *720
    ///配置深度图像流：分辨率640*480，图像格式：Z16， 帧率：30帧/秒
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    // Configure and start the pipeline
    p.start(cfg);

    while (true)
    {
        start_time1 = boost::get_system_time();
        //boost::posix_time::ptime start_time;// = boost::get_system_time();
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();
        // Align to depth 
        //rs2::align align_to_depth(RS2_STREAM_DEPTH);
        rs2::align align_to_depth(RS2_STREAM_DEPTH);
        rs2::align align_to_color(RS2_STREAM_COLOR);
        // frames = align_to_depth.process(frames);

        frames = align_to_color.process(frames);

        // Try to get a frame of a depth image
        rs2::depth_frame depth = frames.get_depth_frame(); ///获取深度图像数据
        //frame depth_frame = frames.get_depth_frame().apply_filter(color_map);
        rs2::frame color = frames.get_color_frame(); ///获取彩色图像数据


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
        cv::Mat image_gray,image_canny;
        cv::Mat image(cv::Size(width, height), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        //cv::initUndistortRectifyMap();
        cv::Mat depthmat(cv::Size(width, height), CV_16U, (void*)depth.get_data(), cv::Mat::AUTO_STEP);//CV_16U
        if (!image.data) continue;
        if (!depthmat.data) continue;
        // cout << "size:=" << depthmat.size() << endl;;
        controller_ur->update_p(boost::get_system_time(), boost::posix_time::milliseconds(8));
        depthmat.copyTo(depthmatR);


        //将深度图归一化显示
        vector<cv::Point> target_points;

        // Display in a GUI
        //namedWindow("Display Image", WINDOW_AUTOSIZE);
        //cv::imshow("Display 1 Image", image);
        //cv::waitKey(1);
        //cv::imshow("Display depth", depthmat*15);
        //cv::waitKey(1);
        cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);//彩色图转灰度图

        //equalizeHist(image_gray, image_gray1);
        //adaptiveThreshold(image_gray, image_canny, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 25, 0);
        //adaptiveThreshold(image_canny, image_canny, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 25, 0);
        threshold(image_gray, image_canny, 100, 255, THRESH_TOZERO);//THRESH_BINARY | THRESH_OTSU  THRESH_TOZERO_INV
        threshold(image_canny, image_canny, 80, 255, THRESH_BINARY);//THRESH_BINARY | THRESH_OTSU
        //LargestConnecttedComponent(image_canny, image_canny);
         //获取自定义核
        Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));//MORPH_RECT MORPH_CROSS

        //进行腐蚀操作
        erode(image_canny, image_canny, element);


        //【1】轮廓发现--找出最大轮廓--轮廓外点置0，轮廓内点保留
        vector<vector<Point>> contours;
        vector<Vec4i> her;
        findContours(image_canny, contours, her, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        Mat resultImage = Mat::zeros(image_canny.size(), CV_8UC3);
        RNG rng(12345);
        double area = 0.0;
        int max_contours_index;
        Point pRadius;
        int  circle_num = 0, circle_num_temp=0;
        //找出最大轮廓
        for (size_t i = 0; i < contours.size(); i++) {
            //cout << contourArea(contours[i], false) << endl;
            if (contourArea(contours[i], false) > area)
            {
                area = contourArea(contours[i], false);
                max_contours_index = i;
            }
        }
        //轮廓外点置0，轮廓内点保留
        Mat hole(image_canny.size(), CV_8U, Scalar(0)); //遮罩图层  
        cv::drawContours(hole, contours,max_contours_index, Scalar(255,255,255), -1); //在遮罩图层上，用白色像素填充轮廓  
        //namedWindow("My hole");
        //cv::imshow("My hole", hole);
        Mat crop(image_canny.rows, image_canny.cols, CV_8UC3);
        image_canny.copyTo(crop, hole);//将原图像拷贝进遮罩图层  

        //for (int j = 0; j < image.size().height - 1; j++)
        //    for (int i = 0; i < image.size().width - 1; i++)
        //    {
        //        cv::Point siglePoint(i, j);
        //        double dis= cv::pointPolygonTest(contours[max_contours_index], siglePoint, false);
        //        if (dis < 0)
        //            image_canny.at<uchar>(j, i) = 0;
        //    }

         //【2】根据面积及纵横比过滤轮廓
        findContours(crop, contours, her, RETR_TREE, CHAIN_APPROX_SIMPLE);
        for (size_t i = 0; i < contours.size(); i++)
        {
            double area = contourArea(contours[i], false);
            if (area > 100) {
                Rect rect = boundingRect(contours[i]);
                float scale = float(rect.width) / float(rect.height);
                float radius = (float(rect.width) + float(rect.height)) / 4;
                if (scale < 1.1 && scale>0.9 && radius > 5 && radius < 150) {
                    circle_num++;
                }
            }
        }
        if (circle_num == 4)
        {
            cam_ivbs_error = false;
            for (size_t i = 0; i < contours.size(); i++) {
                double area = contourArea(contours[i], false);
                //【2】根据面积及纵横比过滤轮廓
                if (area > 200) {

                    Rect rect = boundingRect(contours[i]);
                    float scale = float(rect.width) / float(rect.height);
                    float radius = (float(rect.width) + float(rect.height)) / 4;
                    if (scale < 1.1 && scale>0.9 && radius > 5 && radius < 150) {
                        drawContours(resultImage, contours, i, Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), -1);
                        int x = rect.width / 2;
                        int y = rect.height / 2;
                        //【3】找出圆心并绘制--求距离
                        pRadius = Point(rect.x + x, rect.y + y);
                        im3_actual_radius[circle_num_temp] = (x+y)/2;
                        im3_actual_center_temp[circle_num_temp * 3] = rect.x + x;
                        im3_actual_center_temp[circle_num_temp * 3 + 1] = rect.y + y;
                        //设置中心点附近的一个区域,用来求距离
                        {
                            int x_min, x_max, y_min, y_max;
                            if ((im3_actual_center_temp[0] - 10) > 0)
                                x_min = (int)(im3_actual_center_temp[0] - 10);
                            else
                                x_min = 0;
                            if ((im3_actual_center_temp[0] + 10) < 1279)
                                x_max = (int)(im3_actual_center_temp[0] + 10);
                            else
                                x_max = 1279;

                            if ((im3_actual_center_temp[1] - 10) > 0)
                                y_min = (int)(im3_actual_center_temp[1] - 10);
                            else
                                y_min = 0;
                            if ((im3_actual_center_temp[1] + 10) < 719)
                                y_max = (int)(im3_actual_center_temp[1] + 10);
                            else
                                y_max = 719;
                            im3_actual_center_temp[circle_num_temp * 3 + 2] = depth.get_distance(rect.x + x, rect.y + y);
                            for (int i = x_min; i < x_max; i++)
                                for (int j = y_min; j < y_max; j++)
                                {

                                    if ((im3_actual_center_temp[circle_num_temp * 3 + 2] < depth.get_distance(i, j)) && (depth.get_distance(i, j) > 20.0))
                                        im3_actual_center_temp[circle_num_temp * 3 + 2] = depth.get_distance(i, j);
                                }
                        }
                        circle(resultImage, pRadius, 2, Scalar(0, 0, 255), 2);
                        circle_num_temp++;
                        circle(image, pRadius, 2, Scalar(0, 0, 255), 2);
                    }
                }

            }
        }
        else
        {
            cam_ivbs_error = true;
            cv::imshow("异常图像", image);
            cv::waitKey(1);
            continue;
        }
            
        //cv::imshow("resultImage", resultImage);
        //cv::waitKey(1);
        //【4】在原图上绘制圆心，这一步要不要都行，因为坐标都找出来了，可以随便标注
       // circle(image, pRadius, 2, Scalar(0, 0, 255), 2);
        circle(image, Point(controller_ur->im3point_des[0], controller_ur->im3point_des[1]), 2, Scalar(0, 255, 0), 2);
        circle(image, Point(controller_ur->im3point_des[2], controller_ur->im3point_des[3]), 2, Scalar(0, 255, 0), 2);
        circle(image, Point(controller_ur->im3point_des[4], controller_ur->im3point_des[5]), 2, Scalar(0, 255, 0), 2);
        circle(image, Point(controller_ur->im3point_des[6], controller_ur->im3point_des[7]), 2, Scalar(0, 255, 0), 2);
        cv::imshow("src_clone", image);
        cv::waitKey(1);

        //按半径排序
        for (int i = 0; i < 4 - 1; i++)
        {
            for (int j = 0; j < 4 - 1- i; j++)
            {
                if (im3_actual_radius[j] > im3_actual_radius[j + 1]) {
                    double temp = im3_actual_radius[j];
                    im3_actual_radius[j] = im3_actual_radius[j + 1];
                    im3_actual_radius[j + 1] = temp;
                    for (int m = 0; m < 3; m++)
                    {
                        double temp_center[3];
                        temp_center[m] = im3_actual_center_temp[j*3 + m];
                        im3_actual_center_temp[j*3 + m] = im3_actual_center_temp[(j+1)*3 + m];
                        im3_actual_center_temp[(j + 1) * 3 + m] = temp_center[m];
                    }
                    
                    //im3_actual_center_temp   im3_actual_center_temp[circle_num_temp * 3]
                }
                
            }
        }
        for (int i = 0; i < 12; i++)
        {
            im3_actual_center[i]=im3_actual_center_temp[i];
        }
        cur_time1 = boost::get_system_time();
        d_time1 = cur_time1 - start_time1;
        d_ms1 = (double)d_time1.total_milliseconds() / 1000;
        //printf("function begin time: %f\n", d_ms1);
    }
}


///////
void track_redblue_box(CartesianSpaceTrackUR* controller_ur)
{
    boost::posix_time::ptime start_time1 = boost::get_system_time();
    double d_ms1;
    boost::posix_time::ptime cur_time1, circle_end_time1;
    boost::posix_time::time_duration d_time1, circle_time1;

    intenal_Mat << 8.8884850236407863e+02, 0., 6.4873567892875235e+02,//637.8,0,637.8,
        0., 8.8769039996290348e+02, 3.6643649723707466e+02,
        0., 0., 1.;


    rs2::colorizer color_map;
    rs2::decimation_filter dec;
    rs2::disparity_transform depth2disparity;
    rs2::disparity_transform disparity2depth;
    rs2::spatial_filter spat;
    rs2::temporal_filter temp;
    rs2::align align_to(RS2_STREAM_COLOR);
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::frameset data;
    //set realsense
   // rs::texture depth_image, color_image;     // Helpers for renderig images
    rs2::colorizer c;         // Helper to colorize depth images
    color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.f);
    dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    spat.set_option(RS2_OPTION_HOLES_FILL, 5);
    // cfg.enable_stream(RS2_STREAM_DEPTH);
     //cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGB8);
    disparity2depth = false;
    int width = 1280;
    int height = 720;
    ///配置彩色图像流：分辨率640*480，图像格式：BGR， 帧率：30帧/秒
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 30);//1280 *720
    ///配置深度图像流：分辨率640*480，图像格式：Z16， 帧率：30帧/秒
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    // Configure and start the pipeline
    p.start(cfg);

    while (true)
    {
        start_time1 = boost::get_system_time();
        //boost::posix_time::ptime start_time;// = boost::get_system_time();
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();
        // Align to depth 
        //rs2::align align_to_depth(RS2_STREAM_DEPTH);
        rs2::align align_to_depth(RS2_STREAM_DEPTH);
        rs2::align align_to_color(RS2_STREAM_COLOR);
        // frames = align_to_depth.process(frames);

        frames = align_to_color.process(frames);

        // Try to get a frame of a depth image
        rs2::depth_frame depth = frames.get_depth_frame(); ///获取深度图像数据
        //frame depth_frame = frames.get_depth_frame().apply_filter(color_map);
        rs2::frame color = frames.get_color_frame(); ///获取彩色图像数据


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
        cv::Mat image_gray, image_hsv, image_canny,img_mask0,img_mask1, img_mask2,img_mask, img_mask4;
        cv::Mat image(cv::Size(width, height), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        //cv::initUndistortRectifyMap();
        cv::Mat depthmat(cv::Size(width, height), CV_16U, (void*)depth.get_data(), cv::Mat::AUTO_STEP);//CV_16U
        if (!image.data) continue;
        if (!depthmat.data) continue;
        // cout << "size:=" << depthmat.size() << endl;;
        controller_ur->update_p(boost::get_system_time(), boost::posix_time::milliseconds(8));
        depthmat.copyTo(depthmatR);
        cv::Mat depth_mask, image_hsv1;
        //将深度图归一化显示
        vector<cv::Point> target_points;
      
        //cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);//彩色图转灰度图
        cv::cvtColor(image, image_hsv, cv::COLOR_BGR2HSV);//彩色图转HSV

#ifdef  blue_box
        cv::inRange(depthmatR, 200, 600, depth_mask);
        //dilate operation
        Mat element_2 = getStructuringElement(MORPH_RECT, Size(15, 15));
        cv::dilate(depth_mask, depth_mask, element_2);
        cv::dilate(depth_mask, depth_mask, element_2);
           // masked_img = cv2.bitwise_and(raw_img, raw_img, mask = mask)
        cv::bitwise_and(image_hsv, image_hsv, image_hsv1, depth_mask);

        //cv::inRange(image_hsv, cv::Scalar(0, 80, 50), cv::Scalar(12, 255, 255), img_mask2);
        //cv::inRange(image_hsv, cv::Scalar(170, 80, 50), cv::Scalar(185, 255, 255), img_mask4);
        cv::inRange(image_hsv1, cv::Scalar(90, 43, 46), cv::Scalar(124, 255, 255), img_mask0);
#endif
#ifdef  red_box_
        cv::inRange(image_hsv, cv::Scalar(0, 80, 50), cv::Scalar(12, 255, 255), img_mask2);
        cv::inRange(image_hsv, cv::Scalar(170, 80, 50), cv::Scalar(185, 255, 255), img_mask4);
        img_mask0 = img_mask2 + img_mask4;
#endif
        
        //定义核
        Mat element1 = getStructuringElement(MORPH_RECT, Size(15, 15));

        //进行形态学开操作
        morphologyEx(img_mask0, img_mask1, MORPH_CLOSE, element1);
        morphologyEx(img_mask1, img_mask, MORPH_CLOSE, element1);
        //进行形态学闭操作
       // morphologyEx(img_mask1, img_mask, MORPH_OPEN, element1);

        //【1】轮廓发现--找出最大轮廓--轮廓外点置0，轮廓内点保留
        vector<vector<Point>> contours;
        vector<Vec4i> her;
        findContours(img_mask, contours, her, RETR_EXTERNAL, CHAIN_APPROX_NONE);
        if (contours.size() < 1)
            {
                cam_redbox_error = true;
                cout << "detect image  contour error " << endl;
                continue;
            }
        Mat resultImage = Mat::zeros(img_mask.size(), CV_8UC3);
        RNG rng(12345);
        double area = 0.0;
        double area_level = 15000.0;
        int max_contours_index;
        Point pRadius;
        int  circle_num = 0, circle_num_temp = 0;
        //找出最大轮廓
        for (size_t i = 0; i < contours.size(); i++) {
            //cout << contourArea(contours[i], false) << endl;
            if (contourArea(contours[i], false) > area)
            {
                area = contourArea(contours[i], false);
                max_contours_index = i;
            }
        }

        //如果最大轮廓面积太少则重新检测
        if (contourArea(contours[max_contours_index], false)< area_level)
        {
            cam_redbox_error = true;
            cout << "detect image  contour area error " << endl;
            continue;
        }
        //轮廓外点置0，轮廓内点保留
        Mat hole(img_mask.size(), CV_8U, Scalar(0)); //遮罩图层  
        cv::drawContours(hole, contours, max_contours_index, Scalar(255, 0, 0), -1); //在遮罩图层上，用白色像素填充轮廓  
        rotatedrect  = cv::minAreaRect(contours[max_contours_index]);
        
        //求各顶点的深度值，采用中间值
        Point2f box_points[4];
        rotatedrect.points(box_points);
        //根据长宽比进行点的排序
        double dist_p1_p2 = sqrt(pow((box_points[1].x - box_points[0].x), 2) + pow((box_points[1].y - box_points[0].y), 2));
        double dist_p2_p3 = sqrt(pow((box_points[2].x - box_points[1].x), 2) + pow((box_points[2].y - box_points[1].y), 2));
        if (dist_p2_p3 / dist_p1_p2 > 1.0)
        {
            //点1,2,3,4其中点2 作为直接边的顶点，边32为X边，12为Y边，Z朝里，整体和相机是一致
            redbox_des_center[5] = rotatedrect.angle + 90;
        }
        else
        {
            Point2f point_temp;
            point_temp = box_points[3];
            for (int i = 3; i >0; i--)
            {
                box_points[i] = box_points[i - 1];
            }
            box_points[0] = point_temp;
            redbox_des_center[5] = rotatedrect.angle;
        }

        int N = 20;

        vector<double > single_depth_vec;
        for (int m = 0; m < 4; m++)
        {
            for (int i = (int)box_points[m].x - N; i < (int)box_points[m].x + N; i++)
            {
                for (int j = (int)box_points[m].y - N; j < (int)box_points[m].y + N; j++)
                {
                    //cout << depthmatR.at<ushort>(j, i) << endl;
                    if (j > 719 || j < 0 || i>1279 || i < 0)
                        continue;
                    if (depthmatR.at<ushort>(j, i) > 250 && depthmatR.at<ushort>(j, i) < 1000)
                    {
                        //cout << depthmatR.at<uint>(i, j) << endl;
                        single_depth_vec.push_back(depthmatR.at<ushort>(j, i));
                    }
                }
            }
            if (single_depth_vec.size() == 0)
            {
                cam_redbox_error = true;
                cout << "detect image depth error " << endl;
                break;
            }
            std::sort(single_depth_vec.begin(), single_depth_vec.end());
            //深度值赋值    
            if (single_depth_vec.size() % 2 == 0)
                rectangel_point[m].z = single_depth_vec[single_depth_vec.size() / 2 - 1];
            else
                rectangel_point[m].z = single_depth_vec[single_depth_vec.size() / 2];
           // cout << rectangel_point[m].z<<"   ";
            //x y赋值
            rectangel_point[m].x = box_points[m].x;
            rectangel_point[m].y = box_points[m].y;
            cam_redbox_error = false;
        }
       // cout << endl;
       // cout << "first point " << rectangel_point[0].x << "  " << rectangel_point[0].y << endl;
        
        //旋转角度赋值
        //double dist_p1_p2 = sqrt(pow((box_points[1].x - box_points[0].x), 2) + pow((box_points[1].y - box_points[0].y), 2));
        //double dist_p2_p3 = sqrt(pow((box_points[2].x - box_points[1].x), 2) + pow((box_points[2].y - box_points[1].y), 2));
        //if(dist_p2_p3/ dist_p1_p2>1.0)
        //    redbox_des_center[5] = rotatedrect.angle+90;
        //else
        //    redbox_des_center[5] = rotatedrect.angle;




       // Point2f box_points[4];//像平面点
        vector<Point2d> box_p;//基坐标系下的三维点
        Point3d point_base_rob, point_base_rob1;//基坐标系下的点
        vector<Point3d> objP;//基坐标系下的三维点
        
        // 计算这个点的空间3D坐标
        for (int i = 0; i < 4; i++)
        {
            //point_base_rob.z = rectangel_point[i].z / camera_factor;
            //point_base_rob.x = (rectangel_point[i].x - intenal_M(0, 2)) * point_base_rob.z / intenal_M(0, 0);
            //point_base_rob.y = (rectangel_point[i].y - intenal_M(1, 2)) * point_base_rob.z / intenal_M(1, 1);
            //Eigen::Vector3d tempP(point_base_rob.x, point_base_rob.y, point_base_rob.z);
            //Eigen::Vector3d APIwcPoint1 = (controller_ur->Ttool_to_base * external_M).block<3, 3>(0, 0) * (tempP)+
            //    (controller_ur->Ttool_to_base * external_M).block<3, 1>(0, 3);//api_ip
            //point_base_rob1.x = APIwcPoint1(0); point_base_rob1.y = APIwcPoint1(1); point_base_rob1.z = APIwcPoint1(2);
            //objP.push_back(point_base_rob1);
           // box_points[i].x = rectangel_point[i].x;
            //box_points[i].y = rectangel_point[i].y;
            box_p.push_back(box_points[i]);
        }
        objP.push_back(cv::Point3d(0,0.145,0));
        objP.push_back(cv::Point3d(0, 0, 0));
        objP.push_back(cv::Point3d(0.210, 0 , 0));
        objP.push_back(cv::Point3d(0.210, 0.145 , 0));
        cv::Mat cama_M(3, 3, CV_64FC1);
        Eigen::Matrix3d intenal_M;
        intenal_M<< 8.8884850236407863e+02, 0., 6.4873567892875235e+02,//637.8,0,637.8,
            0., 8.8769039996290348e+02, 3.6643649723707466e+02,
            0., 0., 1.;
        //cama_M =cv::Mat::zeros(3, 3, CV_64FC1);   // intrinsic camera parameters
        cv::eigen2cv(intenal_M, cama_M);
        cv::Mat distort_coeffs = cv::Mat::zeros(1, 4, CV_64FC1); //畸变系数矩阵 顺序是[k1, k2, p1, p2, k3]
        distort_coeffs.at<double>(0, 0) = 1.6477064591350837e-01;
        distort_coeffs.at<double>(0, 1) = -5.5084560380708281e-01;
        distort_coeffs.at<double>(0, 2) = 2.1098851537153015e-03;
        distort_coeffs.at<double>(0, 3) = -8.3430941555631818e-04;
        //distort_coeffs.at<float>(0, 4) = 5.3144730038638299e-01; 

        //使用pnp解算求出相机矩阵和畸变系数矩阵
        //Rodrigues(rotM, rvec);  //将旋转矩阵变换成旋转向量
        //cv::Mat_<float>(3, 1)
        cv::Mat rvec, tvec, rotM;
        //solvePnP(objP, Mat(box_p), cama_M, distort_coeffs, rvec, tvec,false, cv::SOLVEPNP_UPNP);
        solvePnPRansac(objP, Mat(box_p), cama_M, distort_coeffs, rvec, tvec, false, 100, 8.f, 0.99, cv::noArray(), cv::SOLVEPNP_ITERATIVE);
        Rodrigues(rvec, rotM);  //将旋转向量变换成旋转矩阵
        //Rodrigues(tvec, rotT);
        //cout << "rvec" << endl;
        //cout << rvec<<endl;
        //cout << "rotM" << endl;
        //cout << rotM << endl;
        //cout << "tvec" << endl;
        //cout << tvec << endl;
        Eigen::Matrix3d rot_obj_cama;
        Eigen::Vector3d trans_obj_cama;
        cv2eigen(rotM, rot_obj_cama);
        cv2eigen(tvec, trans_obj_cama);
        //Eigen::Matrix4d Mobj_cama;
        Mobj_cama.setIdentity();
        Mobj_cama.block<3, 3>(0, 0) =  rot_obj_cama;
        Mobj_cama.block<3, 1>(0, 3) = trans_obj_cama;
       // cout << Mobj_cama << endl<<endl;


        std::vector<cv::Point3f> objectPoints;
        objectPoints.push_back(cv::Point3f(0, 0, 0));
        objectPoints.push_back(cv::Point3f(0.1, 0, 0));
        objectPoints.push_back(cv::Point3f(0, 0.1, 0));
        objectPoints.push_back(cv::Point3f(0, 0, 0.3));
        
        std::vector<cv::Point2f> projectedPoints;
        cv::projectPoints(objectPoints, rvec, tvec, cama_M, distort_coeffs, projectedPoints);
        cv::line(image, projectedPoints[0], projectedPoints[1], cv::Scalar(255, 0, 0), 5);
        cv::line(image, projectedPoints[0], projectedPoints[2], cv::Scalar(0, 255, 0), 5);
        cv::line(image, projectedPoints[0], projectedPoints[3], cv::Scalar(0, 255, 255), 5);

        //cout << rotM.inv() * tvec << endl;

        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> R_n;
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> T_n;
        
        cv2eigen(rotM, R_n);
        cv2eigen(rotM, m_rot);
        //cout << "R_n" << endl<< R_n << endl;
        //cout << "ouler" << endl;
        //m_rot.normalize();

        //Eigen::Vector3d eulerAngle = m_rot.eulerAngles(2, 1, 0);
        //cout << eulerAngle << endl;
        cv2eigen(tvec, T_n);
        Eigen::Vector3f P_oc;

        //P_oc = R_n.eulerAngles(0, 1, 2);

        //P_oc = -R_n.inverse() * T_n;
        //cout << "P_oc" << endl;
        //cout << P_oc << endl;
        //Pc = R * Po + T //通过基系下的一个3D点求相机坐标系的3D点
        Eigen::Vector3f P_word, P_cama_co, P_pixel;
        P_word << objP[0].x, objP[0].y, objP[0].z;
        P_cama_co = R_n * P_word + T_n;//通过基系下的一个3D点求相机坐标系的3D点
        //cout << "P_cama_co" << endl;
        //cout << P_cama_co << endl;

        Eigen::Vector3d p1, p2, p3;
        p2 << objP[0].x, objP[0].y, objP[0].z;
        p1 << objP[1].x, objP[1].y, objP[1].z;
        p3 << objP[2].x, objP[2].y, objP[2].z;
        //Eigen::AngleAxisd yawAngle_detect(Eigen::AngleAxisd((0, 0, (90 - redbox_des_center[5]) * 3.141 / 180), Eigen::Vector3d::UnitZ()));
       // R_des = Point3_to_Plane(p1, p2, p3);
        //cout << "R_des" << endl;
        //cout << R_des << endl;
        Eigen::Vector3d euler_angles = m_rot.eulerAngles(2, 1, 0);
        //cout << m_rot << endl;
        double yaw = euler_angles[0] * 180 / 3.14159;
        double picth = euler_angles[1] * 180 / 3.14159;
        double roll = euler_angles[2] * 180 / 3.14159;
        //cout << "roll  " << roll << " picth " << picth << "yaw  " << yaw << endl;

        //cout << "controller_ur->Ttool_to_base.block<3, 3>(0, 0)" << endl;
        euler_angles = controller_ur->Ttool_to_base.block<3, 3>(0, 0).eulerAngles(2, 1, 0);
        yaw = euler_angles[0] * 180 / 3.14159;
        picth = euler_angles[1] * 180 / 3.14159;
        roll = euler_angles[2] * 180 / 3.14159;
       // cout << "roll_rot  " << roll << " picth_rob " << picth << "yaw_rob  " << yaw << endl;
        //x_des = controller_ur->x;
        //求世界系下的点 通过相机系下在3D点，求基系下的3D点
        //float p_z = 0.445;
        //P_pixel << rotatedrect.center.x, rotatedrect.center.y, 1;
        //P_cama_co = (intenal_M.inverse()).cast<float>() * p_z * P_pixel;
        //cout << "P_cama_co" << endl;
        //cout << P_cama_co << endl;
        //P_word = (P_cama_co - T_n) * R_n.inverse();
        //cout << "P_word" << endl;
        //cout << P_word << endl;





        //cv::projectPoints(objectPoints, rVec, tVec, intrisicMat, distCoeffs, projectedPoints);
        ////画点
        pRadius = rotatedrect.center;
        circle(image, pRadius, 4, Scalar(0, 255, 0), 7);
        imshow("color", image);
        cv::waitKey(1);
        imshow("depth", depthmatR * 15);
        cv::waitKey(1);
        continue;

        Mat crop(image_canny.rows, image_canny.cols, CV_8UC3);
        image_canny.copyTo(crop, hole);//将原图像拷贝进遮罩图层  



         //【2】根据面积及纵横比过滤轮廓
        findContours(crop, contours, her, RETR_TREE, CHAIN_APPROX_SIMPLE);
        for (size_t i = 0; i < contours.size(); i++)
        {
            double area = contourArea(contours[i], false);
            if (area > 100) {
                Rect rect = boundingRect(contours[i]);
                float scale = float(rect.width) / float(rect.height);
                float radius = (float(rect.width) + float(rect.height)) / 4;
                if (scale < 1.1 && scale>0.9 && radius > 5 && radius < 150) {
                    circle_num++;
                }
            }
        }
        if (circle_num == 4)
        {
            cam_ivbs_error = false;
            for (size_t i = 0; i < contours.size(); i++) {
                double area = contourArea(contours[i], false);
                //【2】根据面积及纵横比过滤轮廓
                if (area > 200) {

                    Rect rect = boundingRect(contours[i]);
                    float scale = float(rect.width) / float(rect.height);
                    float radius = (float(rect.width) + float(rect.height)) / 4;
                    if (scale < 1.1 && scale>0.9 && radius > 5 && radius < 150) {
                        drawContours(resultImage, contours, i, Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), -1);
                        int x = rect.width / 2;
                        int y = rect.height / 2;
                        //【3】找出圆心并绘制--求距离
                        pRadius = Point(rect.x + x, rect.y + y);
                        im3_actual_radius[circle_num_temp] = (x + y) / 2;
                        im3_actual_center_temp[circle_num_temp * 3] = rect.x + x;
                        im3_actual_center_temp[circle_num_temp * 3 + 1] = rect.y + y;
                        //设置中心点附近的一个区域,用来求距离
                        {
                            int x_min, x_max, y_min, y_max;
                            if ((im3_actual_center_temp[0] - 10) > 0)
                                x_min = (int)(im3_actual_center_temp[0] - 10);
                            else
                                x_min = 0;
                            if ((im3_actual_center_temp[0] + 10) < 1279)
                                x_max = (int)(im3_actual_center_temp[0] + 10);
                            else
                                x_max = 1279;

                            if ((im3_actual_center_temp[1] - 10) > 0)
                                y_min = (int)(im3_actual_center_temp[1] - 10);
                            else
                                y_min = 0;
                            if ((im3_actual_center_temp[1] + 10) < 719)
                                y_max = (int)(im3_actual_center_temp[1] + 10);
                            else
                                y_max = 719;
                            im3_actual_center_temp[circle_num_temp * 3 + 2] = depth.get_distance(rect.x + x, rect.y + y);
                            for (int i = x_min; i < x_max; i++)
                                for (int j = y_min; j < y_max; j++)
                                {

                                    if ((im3_actual_center_temp[circle_num_temp * 3 + 2] < depth.get_distance(i, j)) && (depth.get_distance(i, j) > 20.0))
                                        im3_actual_center_temp[circle_num_temp * 3 + 2] = depth.get_distance(i, j);
                                }
                        }
                        circle(resultImage, pRadius, 2, Scalar(0, 0, 255), 2);
                        circle_num_temp++;
                        circle(image, pRadius, 2, Scalar(0, 0, 255), 2);
                    }
                }

            }
        }
        else
        {
            cam_ivbs_error = true;
            cv::imshow("异常图像", image);
            cv::waitKey(1);
            continue;
        }

        //cv::imshow("resultImage", resultImage);
        //cv::waitKey(1);
        //【4】在原图上绘制圆心，这一步要不要都行，因为坐标都找出来了，可以随便标注
       // circle(image, pRadius, 2, Scalar(0, 0, 255), 2);
        circle(image, Point(controller_ur->im3point_des[0], controller_ur->im3point_des[1]), 2, Scalar(0, 255, 0), 2);
        circle(image, Point(controller_ur->im3point_des[2], controller_ur->im3point_des[3]), 2, Scalar(0, 255, 0), 2);
        circle(image, Point(controller_ur->im3point_des[4], controller_ur->im3point_des[5]), 2, Scalar(0, 255, 0), 2);
        circle(image, Point(controller_ur->im3point_des[6], controller_ur->im3point_des[7]), 2, Scalar(0, 255, 0), 2);
        cv::imshow("src_clone", image);
        cv::waitKey(1);

        //按半径排序
        for (int i = 0; i < 4 - 1; i++)
        {
            for (int j = 0; j < 4 - 1 - i; j++)
            {
                if (im3_actual_radius[j] > im3_actual_radius[j + 1]) {
                    double temp = im3_actual_radius[j];
                    im3_actual_radius[j] = im3_actual_radius[j + 1];
                    im3_actual_radius[j + 1] = temp;
                    for (int m = 0; m < 3; m++)
                    {
                        double temp_center[3];
                        temp_center[m] = im3_actual_center_temp[j * 3 + m];
                        im3_actual_center_temp[j * 3 + m] = im3_actual_center_temp[(j + 1) * 3 + m];
                        im3_actual_center_temp[(j + 1) * 3 + m] = temp_center[m];
                    }
                    //im3_actual_center_temp   im3_actual_center_temp[circle_num_temp * 3]
                }
            }
        }
        for (int i = 0; i < 12; i++)
        {
            im3_actual_center[i] = im3_actual_center_temp[i];
        }
        cur_time1 = boost::get_system_time();
        d_time1 = cur_time1 - start_time1;
        d_ms1 = (double)d_time1.total_milliseconds() / 1000;
        //printf("function begin time: %f\n", d_ms1);
    }
}

///////
void track_chessboard(CartesianSpaceTrackUR* controller_ur)
{
    boost::posix_time::ptime start_time1 = boost::get_system_time();
    double d_ms1;
    boost::posix_time::ptime cur_time1, circle_end_time1;
    boost::posix_time::time_duration d_time1, circle_time1;

    intenal_Mat << 8.8884850236407863e+02, 0., 6.4873567892875235e+02,//637.8,0,637.8,
        0., 8.8769039996290348e+02, 3.6643649723707466e+02,
        0., 0., 1.;


    rs2::colorizer color_map;
    rs2::decimation_filter dec;
    rs2::disparity_transform depth2disparity;
    rs2::disparity_transform disparity2depth;
    rs2::spatial_filter spat;
    rs2::temporal_filter temp;
    rs2::align align_to(RS2_STREAM_COLOR);
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::frameset data;
    //set realsense
   // rs::texture depth_image, color_image;     // Helpers for renderig images
    rs2::colorizer c;         // Helper to colorize depth images
    color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.f);
    dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    spat.set_option(RS2_OPTION_HOLES_FILL, 5);
    // cfg.enable_stream(RS2_STREAM_DEPTH);
     //cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGB8);
    disparity2depth = false;
    int width = 1280;
    int height = 720;
    ///配置彩色图像流：分辨率640*480，图像格式：BGR， 帧率：30帧/秒
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 30);//1280 *720
    ///配置深度图像流：分辨率640*480，图像格式：Z16， 帧率：30帧/秒
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;
    //#define board_11_8;
    #define board_9_6
    // Configure and start the pipeline
    p.start(cfg);

    while (true)
    {
        start_time1 = boost::get_system_time();
        //boost::posix_time::ptime start_time;// = boost::get_system_time();
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();
        // Align to depth 
        //rs2::align align_to_depth(RS2_STREAM_DEPTH);
        rs2::align align_to_depth(RS2_STREAM_DEPTH);
        rs2::align align_to_color(RS2_STREAM_COLOR);
        // frames = align_to_depth.process(frames);

        frames = align_to_color.process(frames);

        // Try to get a frame of a depth image
        rs2::depth_frame depth = frames.get_depth_frame(); ///获取深度图像数据
        //frame depth_frame = frames.get_depth_frame().apply_filter(color_map);
        rs2::frame color = frames.get_color_frame(); ///获取彩色图像数据


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
        cv::Mat image_gray, image_backup, image_canny, img_mask0, img_mask1, img_mask2, img_mask, img_mask4;
        cv::Mat image(cv::Size(width, height), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        //cv::initUndistortRectifyMap();
        cv::Mat depthmat(cv::Size(width, height), CV_16U, (void*)depth.get_data(), cv::Mat::AUTO_STEP);//CV_16U
        if (!image.data) continue;
        if (!depthmat.data) continue;
        // cout << "size:=" << depthmat.size() << endl;;
        controller_ur->update_p(boost::get_system_time(), boost::posix_time::milliseconds(8));
        depthmat.copyTo(depthmatR);
        image.copyTo(image_backup);
        cv::Mat depth_mask, image_hsv1;
        vector<Point2f> corner;
        //cout << "image size: " << image_size.width << " " << image_size.height << endl;
        int res = findChessboardCorners(image_backup, Size(9, 6), corner);//11,8
        Point2f box_points[4];//像平面点
        vector<Point2d> box_p;//基坐标系下的三维点
        vector<Point3d> objP;//基坐标系下的三维点
        Point3d point_base_rob, point_base_rob1;//基坐标系下的点
        if (res == 0)
        {
            cout << "No chessboard corners found! Drop this image!" << endl;
            cam_redbox_error = true;
            continue;
        }
        else
        {
            /* show corner */
            Mat view;
            view = image_backup.clone();
            //cv::drawChessboardCorners(view, Size(11, 8), corner, true);
            //cv::imshow("calibration", view);
           // cv::waitKey(1);
          //  cout << "found " << corner.size() << " corners." << endl;
#ifdef board_11_8
            //11*8
            {
                box_p.push_back(corner[77]);
                box_p.push_back(corner[66]);
                box_p.push_back(corner[55]);
                box_p.push_back(corner[44]);
                box_p.push_back(corner[33]);
                box_p.push_back(corner[22]);
                box_p.push_back(corner[11]);
                box_p.push_back(corner[0]);
                box_p.push_back(corner[1]);
                box_p.push_back(corner[2]);
                box_p.push_back(corner[3]);
                box_p.push_back(corner[4]);
                box_p.push_back(corner[5]);
                box_p.push_back(corner[6]);
                box_p.push_back(corner[7]);
                box_p.push_back(corner[8]);
                box_p.push_back(corner[9]);
                box_p.push_back(corner[10]);
                box_p.push_back(corner[21]);
                box_p.push_back(corner[32]);
                box_p.push_back(corner[43]);
                box_p.push_back(corner[54]);
                box_p.push_back(corner[65]);
                box_p.push_back(corner[76]);
                box_p.push_back(corner[87]);
                box_p.push_back(corner[77]);
                box_p.push_back(corner[78]);
                box_p.push_back(corner[79]);
                box_p.push_back(corner[80]);
                box_p.push_back(corner[81]);
                box_p.push_back(corner[82]);
                box_p.push_back(corner[83]);
                box_p.push_back(corner[84]);
                box_p.push_back(corner[85]);
                box_p.push_back(corner[86]);

                objP.push_back(cv::Point3d(0, 0.35, 0));
                objP.push_back(cv::Point3d(0, 0.3, 0));
                objP.push_back(cv::Point3d(0, 0.25, 0));
                objP.push_back(cv::Point3d(0, 0.2, 0));
                objP.push_back(cv::Point3d(0, 0.15, 0));
                objP.push_back(cv::Point3d(0, 0.1, 0));
                objP.push_back(cv::Point3d(0, 0.05, 0));
                objP.push_back(cv::Point3d(0, 0, 0));
                objP.push_back(cv::Point3d(0.05, 0, 0));
                objP.push_back(cv::Point3d(0.1, 0, 0));
                objP.push_back(cv::Point3d(0.15, 0, 0));
                objP.push_back(cv::Point3d(0.2, 0, 0));
                objP.push_back(cv::Point3d(0.25, 0, 0));
                objP.push_back(cv::Point3d(0.3, 0, 0));
                objP.push_back(cv::Point3d(0.35, 0, 0));
                objP.push_back(cv::Point3d(0.4, 0, 0));
                objP.push_back(cv::Point3d(0.45, 0, 0));
                objP.push_back(cv::Point3d(0.5, 0, 0));
                objP.push_back(cv::Point3d(0.5, 0.5, 0));
                objP.push_back(cv::Point3d(0.5, 0.1, 0));
                objP.push_back(cv::Point3d(0.5, 0.15, 0));
                objP.push_back(cv::Point3d(0.5, 0.2, 0));
                objP.push_back(cv::Point3d(0.5, 0.25, 0));
                objP.push_back(cv::Point3d(0.5, 0.3, 0));
                objP.push_back(cv::Point3d(0.5, 0.35, 0));
                objP.push_back(cv::Point3d(0.35, 0.05, 0));
                objP.push_back(cv::Point3d(0.35, 0.1, 0));
                objP.push_back(cv::Point3d(0.35, 0.15, 0));
                objP.push_back(cv::Point3d(0.35, 0.2, 0));
                objP.push_back(cv::Point3d(0.35, 0.25, 0));
                objP.push_back(cv::Point3d(0.35, 0.3, 0));
                objP.push_back(cv::Point3d(0.35, 0.35, 0));
                objP.push_back(cv::Point3d(0.35, 0.4, 0));
                objP.push_back(cv::Point3d(0.35, 0.45, 0));
                objP.push_back(cv::Point3d(0.35, 0.5, 0));
            }
#endif

#ifdef board_9_6
            //11*8
            {
                //Y axis 
                box_p.push_back(corner[45]);
                box_p.push_back(corner[36]);
                box_p.push_back(corner[27]);
                box_p.push_back(corner[18]);
                box_p.push_back(corner[9]);
                box_p.push_back(corner[0]);

                //X axis
                box_p.push_back(corner[1]);
                box_p.push_back(corner[2]);
                box_p.push_back(corner[3]);
                box_p.push_back(corner[4]);
                box_p.push_back(corner[5]);
                box_p.push_back(corner[6]);
                box_p.push_back(corner[7]);
                box_p.push_back(corner[8]);

                // y parallel edge
                box_p.push_back(corner[17]);
                box_p.push_back(corner[26]);
                box_p.push_back(corner[35]);
                box_p.push_back(corner[44]);
                box_p.push_back(corner[53]);
                
                //x parallel edge
                box_p.push_back(corner[46]);
                box_p.push_back(corner[47]);
                box_p.push_back(corner[48]);
                box_p.push_back(corner[49]);
                box_p.push_back(corner[50]);
                box_p.push_back(corner[51]);
                box_p.push_back(corner[52]);
                box_p.push_back(corner[53]);


                
                objP.push_back(cv::Point3d(0, 0.0214*5, 0));
                objP.push_back(cv::Point3d(0, 0.0214*4, 0));
                objP.push_back(cv::Point3d(0, 0.0214*3, 0));
                objP.push_back(cv::Point3d(0, 0.0214*2, 0));
                objP.push_back(cv::Point3d(0, 0.0214*1, 0));
                objP.push_back(cv::Point3d(0, 0, 0));

                objP.push_back(cv::Point3d(0.0214 * 1, 0, 0));
                objP.push_back(cv::Point3d(0.0214 * 2, 0, 0));
                objP.push_back(cv::Point3d(0.0214 * 3, 0, 0));
                objP.push_back(cv::Point3d(0.0214 * 4, 0, 0));
                objP.push_back(cv::Point3d(0.0214 * 5, 0, 0));
                objP.push_back(cv::Point3d(0.0214 * 6, 0, 0));
                objP.push_back(cv::Point3d(0.0214 * 7, 0, 0));
                objP.push_back(cv::Point3d(0.0214 * 8, 0, 0));

                objP.push_back(cv::Point3d(0.0214 * 8, 0.0214 * 1, 0));
                objP.push_back(cv::Point3d(0.0214 * 8, 0.0214 * 2, 0));
                objP.push_back(cv::Point3d(0.0214 * 8, 0.0214 * 3, 0));
                objP.push_back(cv::Point3d(0.0214 * 8, 0.0214 * 4, 0));
                objP.push_back(cv::Point3d(0.0214 * 8, 0.0214 * 5, 0));

                objP.push_back(cv::Point3d(0.0214 * 5, 0.0214 * 1, 0));
                objP.push_back(cv::Point3d(0.0214 * 5, 0.0214 * 2, 0));
                objP.push_back(cv::Point3d(0.0214 * 5, 0.0214 * 3, 0));
                objP.push_back(cv::Point3d(0.0214 * 5, 0.0214 * 4, 0));
                objP.push_back(cv::Point3d(0.0214 * 5, 0.0214 * 5, 0));
                objP.push_back(cv::Point3d(0.0214 * 5, 0.0214 * 6, 0));
                objP.push_back(cv::Point3d(0.0214 * 5, 0.0214 * 7, 0));
                objP.push_back(cv::Point3d(0.0214 * 5, 0.0214 * 8, 0));

            }
#endif
                cv::Mat cama_M(3, 3, CV_64FC1);
            Eigen::Matrix3d intenal_M;
            intenal_M << 8.8884850236407863e+02, 0., 6.4873567892875235e+02,//637.8,0,637.8,
                0., 8.8769039996290348e+02, 3.6643649723707466e+02,
                0., 0., 1.;
            //cama_M =cv::Mat::zeros(3, 3, CV_64FC1);   // intrinsic camera parameters
            cv::eigen2cv(intenal_M, cama_M);
            cv::Mat distort_coeffs = cv::Mat::zeros(1, 4, CV_64FC1); //畸变系数矩阵 顺序是[k1, k2, p1, p2, k3]
            distort_coeffs.at<double>(0, 0) = 1.6477064591350837e-01;
            distort_coeffs.at<double>(0, 1) = -5.5084560380708281e-01;
            distort_coeffs.at<double>(0, 2) = 2.1098851537153015e-03;
            distort_coeffs.at<double>(0, 3) = -8.3430941555631818e-04;
            //distort_coeffs.at<float>(0, 4) = 5.3144730038638299e-01; 

            //使用pnp解算求出相机矩阵和畸变系数矩阵
            //Rodrigues(rotM, rvec);  //将旋转矩阵变换成旋转向量
            //cv::Mat_<float>(3, 1)
            cv::Mat rvec, tvec, rotM;
           //solvePnP(objP, Mat(box_p), cama_M, distort_coeffs, rvec, tvec,false, cv::SOLVEPNP_UPNP);
            solvePnPRansac(objP, Mat(box_p), cama_M, distort_coeffs, rvec, tvec, false, 30, 8.f, 0.99, cv::noArray(), cv::SOLVEPNP_ITERATIVE);
            Rodrigues(rvec, rotM);  //将旋转向量变换成旋转矩阵
            //Rodrigues(tvec, rotT);
            //cout << "rvec  " << rvec << endl<< endl;
            //cout << rvec<<endl;
            //cout << "rotM" << endl;
            //cout << rotM << endl;
            //cout << "tvec" << endl;
            //cout << tvec << endl;
            Eigen::Matrix3d rot_obj_cama;
            Eigen::Vector3d trans_obj_cama;
            cv2eigen(rotM, rot_obj_cama);
            cv2eigen(tvec, trans_obj_cama);
            //Eigen::Matrix4d Mobj_cama;
            Mobj_cama.setIdentity();
            Mobj_cama.block<3, 3>(0, 0) = rot_obj_cama;
            Mobj_cama.block<3, 1>(0, 3) = trans_obj_cama;
            // cout << Mobj_cama << endl<<endl;

            std::vector<cv::Point3f> objectPoints;
            objectPoints.push_back(cv::Point3f(0, 0, 0));
            objectPoints.push_back(cv::Point3f(0.1, 0, 0));
            objectPoints.push_back(cv::Point3f(0, 0.1, 0));
            objectPoints.push_back(cv::Point3f(0, 0, 0.3));

            std::vector<cv::Point2f> projectedPoints;
            cv::projectPoints(objectPoints, rvec, tvec, cama_M, distort_coeffs, projectedPoints);
            cv::line(view, projectedPoints[0], projectedPoints[1], cv::Scalar(255, 0, 0), 5);
            cv::line(view, projectedPoints[0], projectedPoints[2], cv::Scalar(0, 255, 0), 5);
            cv::line(view, projectedPoints[0], projectedPoints[3], cv::Scalar(0, 255, 255), 5);
            //cout << rotM.inv() * tvec << endl;

            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> R_n;
            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> T_n;

            cv2eigen(rotM, R_n);
            cv2eigen(rotM, m_rot);
            //cout << "R_n" << endl<< R_n << endl;
            //cout << "ouler" << endl;
            //m_rot.normalize();

            Eigen::Vector3d eulerAngle = m_rot.eulerAngles(2, 1, 0);
            cout << eulerAngle.transpose() <<endl<< endl;
            cv2eigen(tvec, T_n);
            Eigen::Vector3f P_oc;

            //P_oc = R_n.eulerAngles(0, 1, 2);

            //P_oc = -R_n.inverse() * T_n;
            //cout << "P_oc" << endl;
            //cout << P_oc << endl;
            //Pc = R * Po + T //通过基系下的一个3D点求相机坐标系的3D点
            Eigen::Vector3f P_word, P_cama_co, P_pixel;
            P_word << objP[0].x, objP[0].y, objP[0].z;
            P_cama_co = R_n * P_word + T_n;//通过基系下的一个3D点求相机坐标系的3D点
            //cout << "P_cama_co" << endl;
            //cout << P_cama_co << endl;

            Eigen::Vector3d euler_angles = m_rot.eulerAngles(2, 1, 0);
            //cout << m_rot << endl;
            double yaw = euler_angles[0] * 180 / 3.14159;
            double picth = euler_angles[1] * 180 / 3.14159;
            double roll = euler_angles[2] * 180 / 3.14159;
            //cout << "roll  " << roll << " picth " << picth << "yaw  " << yaw << endl;
            
            //debug ibvs use begin
                im3_actual_center[0] = corner[0].x; im3_actual_center[1] = corner[0].y;
                im3_actual_center[3] = corner[8].x; im3_actual_center[4] = corner[8].y;
                im3_actual_center[6] = corner[45].x; im3_actual_center[7] = corner[45].y;
                im3_actual_center[9] = corner[53].x; im3_actual_center[10] = corner[53].y;

                //im3_actual_center
                //求Z向深度
                int N = 10;
                for (int m = 0; m < 4; m++)
                {
                    vector<double > single_depth_vec;
                    for (int i = (int)im3_actual_center[m * 3 + 0] - N; i < (int)im3_actual_center[m * 3 + 0] + N; i++)
                    {
                        for (int j = (int)im3_actual_center[m * 3 + 1] - N; j < (int)im3_actual_center[m * 3 + 1] + N; j++)
                        {
                            //cout << depthmatR.at<ushort>(j, i) << endl;
                            if (j > 719 || j < 0 || i>1279 || i < 0)
                                continue;
                            if (depth.get_distance(i, j) > 0.250 && depth.get_distance(i, j) < 1)
                            {
                                //cout << depthmatR.at<uint>(i, j) << endl;
                                single_depth_vec.push_back(depth.get_distance(i, j));
                            }
                        }
                    }
                    if (single_depth_vec.size() == 0)
                    {
                        cam_redbox_error = true;
                        cout << "detect image depth error " << endl;
                        break;
                    }
                    std::sort(single_depth_vec.begin(), single_depth_vec.end());
                    //深度值赋值    
                    if (single_depth_vec.size() % 2 == 0)
                        im3_actual_center[m * 3 + 2] = single_depth_vec[single_depth_vec.size() / 2 - 1];
                    else
                        im3_actual_center[m * 3 + 2] = single_depth_vec[single_depth_vec.size() / 2];
                }

                //draw actual circle center
                circle(view, Point(im3_actual_center[0], im3_actual_center[1]), 2, Scalar(0, 255, 0), 2);
                circle(view, Point(im3_actual_center[3], im3_actual_center[4]), 2, Scalar(0, 255, 0), 2);
                circle(view, Point(im3_actual_center[6], im3_actual_center[7]), 2, Scalar(0, 255, 0), 2);
                circle(view, Point(im3_actual_center[9], im3_actual_center[10]), 2, Scalar(0, 255, 0), 2);

                //draw desired circle center
                circle(view, Point(controller_ur->im3point_des[0], controller_ur->im3point_des[1]), 2, Scalar(0, 0,255), 2);
                circle(view, Point(controller_ur->im3point_des[2], controller_ur->im3point_des[3]), 2, Scalar(0, 0,255), 2);
                circle(view, Point(controller_ur->im3point_des[4], controller_ur->im3point_des[5]), 2, Scalar(0, 0,255), 2);
                circle(view, Point(controller_ur->im3point_des[6], controller_ur->im3point_des[7]), 2, Scalar(0, 0,255), 2);
            //debug ibvs use begin

            imshow("color", view);  
            cv::waitKey(1);
            //imshow("depth", depthmatR * 15);
            //cv::waitKey(1);
            cam_redbox_error = false;
        }
           
    }
}