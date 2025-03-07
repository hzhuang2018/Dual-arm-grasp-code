#include "path_generator.h"
#include "realsense_position.h"
#include <deque>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include "omp.h"
using namespace Eigen;


#define UR_CYCLIC_TIME_MS 8
#define CONTROL_PERIOD_MS 8 //40
using namespace std;

// variable used in the control thread of UR
extern boost::condition_variable cRun, cRun2;
boost::mutex mUpdate_ompl, mUpdate2_ompl;
bool running_ompl;
double x_rnd_min = 0.4;
double x_rnd_max = 1.0;
double y_rnd_min = -1.0;
double y_rnd_max = 1.0;
const int  N_ompl = 100; //精度为小数点后面2位
Eigen::Matrix4d external_M, test_exM;
Eigen::Matrix<double, 3, 3> intenal_M;
extern double im3_actual_center[12];//3点图像实际像素位置
extern double im3_actual_radius[4];//3点图像实际像素位置
extern float api_camara[3];
extern cv::Mat depthmatR;
extern bool cam_ivbs_error;
extern bool cam_redbox_error;
extern double redbox_des_center[6];//矩形中心x/y/z,rx,ry,rz
extern cv::RotatedRect rotatedrect;
extern cv::Point3d rectangel_point[4];//存储的4个3D矩形顶点
extern Eigen::Matrix3d m_rot;
extern Eigen::Matrix4d Mobj_cama;
OcTree Bigtree(0.02);  // create empty tree with resolution 0.1
octomap::OcTree* octTree = new octomap::OcTree(0.02);
octomap::OcTree envitree(0.02);  // create empty tree with resolution 0.1
planner_joint_dualarm  myplanner;//planner_joint   planner_cartisian
//#define pi 3.1415926

Eigen::Matrix3d Point3_to_Plane(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C)
{
	//AB=(x2-x1,y2-y1,z2-z1),AC=(x3-x1,y3-y1,z3-z1)
	Eigen::Vector3d AB;
	AB << B(0) - A(0), B(1) - A(1), B(2) - A(2);
	//AB = AB.normalized();
	//AC
	Eigen::Vector3d AC;
	AC << C(0) - A(0), C(1) - A(1), C(2) - A(2);
	//AC = AC.normalized();
	 //ABXAC  a=(y2-y1)(z3-z1)-(z2-z1)(y3-y1)
			//b=(z2-z1)(x3-x1)-(z3-z1)(x2-x1)
			//c=(x2-x1)(y3-y1)-(x3-x1)(y2-y1)

	//AB_cross_AC <<
	//	(p2(1) - p1(1)) * (p3(2) - p1(2)) - (p2(2) - p1(2)) * (p3(1) - p1(1)),
	//	(p2(2) - p1(2)) * (p3(0) - p1(0)) - (p3(2) - p1(2)) * (p2(0) - p1(0)),
	//	(p2(0) - p1(0)) * (p3(1) - p1(1)) - (p3(0) - p1(0)) * (p2(1) - p1(1));
	Eigen::Matrix3d rotMatrix = Eigen::Quaterniond::FromTwoVectors(AB, AC).toRotationMatrix();

	return rotMatrix;
}


void cameraToBase(const Eigen::Matrix4d & external_T, const Eigen::Matrix3d & intrisci_M, const  cv::Mat & depth_mat, OcTree &tree)
{
	double camera_factor = 1000.0;
	
	octomap::Pointcloud octPointCloud;
	omp_set_num_threads(4);
#pragma omp parallel
	{
		octomap::point3d p, p2;
		for (int m = 0; m < depth_mat.rows; m++) {
			for (int n = 0; n < depth_mat.cols; n++) {
				// 获取深度图中(m,n)处的值
				ushort d = depth_mat.ptr<ushort>(m)[n];  //float d = depth_pic.ptr<float>(m)[n];
				// d 可能没有值，若如此，跳过此点
				if (d == 0 or d > 2000)
					continue;
				// d 存在值，则向点云增加一个点

				// 计算这个点的空间坐标
				p.z() = double(d) / camera_factor;
				p.x() = (n - intrisci_M(0, 2)) * p.z() / intrisci_M(0, 0);
				p.y() = (m - intrisci_M(1, 2)) * p.z() / intrisci_M(1, 1);

				Eigen::Vector3d p_cama;
				p_cama(0) = p.x(); p_cama(1) = p.y(); p_cama(2) = p.z();
				Eigen::Vector3d APIwcPoint1 = external_T.block<3, 3>(0, 0) * (p_cama)+external_T.block<3, 1>(0, 3);//api_ip
				p2.x() = APIwcPoint1(0); p2.y() = APIwcPoint1(1); p2.z() = APIwcPoint1(2);
				if (p2.z() < -0.2)
					continue;
				tree.updateNode(p2, true); // integrate 'occupied' measurement
				//tree.updateInnerOccupancy();
				//tree.insertRay(octomap::point3d(0, 0, 0), p2);
				//octPointCloud.push_back(p2);
			}
		}
		tree.updateInnerOccupancy();
	}
	
	//tree.prune();
	//tree.writeBinary("simple_tree.bt");//ot文件包括概率

	//这里应该是机器人基坐标系
	//octomap::point3d origin(0.0, 0.0, 0.0);//应用相机原点
	////origin.x() = external_T(0, 3);origin.y() = external_T(1, 3);origin.z() = external_T(2, 3);
	//octTree->insertPointCloud(octPointCloud, origin);
	//octTree->updateInnerOccupancy();
	//octTree->writeBinary("static_occ.bt");
	//octTree->write("static_occ.ot");

}
void cameraToWorld(InputArray cameraMatrix, InputArray rV, InputArray tV, vector<Point2f> imgPoints, vector<Point3f>& worldPoints)
{
	Mat invK64, invK;
	invK64 = cameraMatrix.getMat().inv();
	invK64.convertTo(invK, CV_32F);
	Mat r, t, rMat;
	rV.getMat().convertTo(r, CV_32F);
	tV.getMat().convertTo(t, CV_32F);//外参平移
	Rodrigues(r, rMat);//外参旋转

	//计算 invR * T
	Mat invR = rMat.inv();
	//cout << "invR\n" << invR << endl;
	//cout << "t\n" << t << t.t() << endl;
	Mat transPlaneToCam;
	if (t.size() == Size(1, 3)) {
		transPlaneToCam = invR * t;//t.t();
	}
	else if (t.size() == Size(3, 1)) {
		transPlaneToCam = invR * t.t();
	}
	else {
		return;
	}
	//cout << "transPlaneToCam\n" << transPlaneToCam << endl;

	int npoints = (int)imgPoints.size();
	//cout << "npoints\n" << npoints << endl;
	for (int j = 0; j < npoints; ++j) {
		Mat coords(3, 1, CV_32F);
		Point3f pt;
		coords.at<float>(0, 0) = imgPoints[j].x;
		coords.at<float>(1, 0) = imgPoints[j].y;
		coords.at<float>(2, 0) = 1.0f;
		//[x,y,z] = invK * [u,v,1]
		Mat worldPtCam = invK * coords;
		//cout << "worldPtCam:" << worldPtCam << endl;
		//[x,y,1] * invR
		Mat worldPtPlane = invR * worldPtCam;
		//cout << "worldPtPlane:" << worldPtPlane << endl;
		//zc 
		float scale = transPlaneToCam.at<float>(2) / worldPtPlane.at<float>(2);
		//cout << "scale:" << scale << endl;
		Mat scale_worldPtPlane(3, 1, CV_32F);
		//scale_worldPtPlane.at<float>(0, 0) = worldPtPlane.at<float>(0, 0) * scale;
		//zc * [x,y,1] * invR
		scale_worldPtPlane = scale * worldPtPlane;
		//cout << "scale_worldPtPlane:" << scale_worldPtPlane << endl;
		//[X,Y,Z]=zc*[x,y,1]*invR - invR*T
		Mat worldPtPlaneReproject = scale_worldPtPlane - transPlaneToCam;
		//cout << "worldPtPlaneReproject:" << worldPtPlaneReproject << endl;
		pt.x = worldPtPlaneReproject.at<float>(0);
		pt.y = worldPtPlaneReproject.at<float>(1);
		//pt.z = worldPtPlaneReproject.at<float>(2);
		pt.z = 1.0f;
		worldPoints.push_back(pt);
	}
}

Point3f getWorldPoints(Eigen::Matrix3d external_R, Eigen::Matrix3d intrisci_M, Eigen::Vector3d external_t, Point2d inPoints,double z_lengh)
{
	double zConst = 0;//实际坐标系的距离，若工作平面与相机距离固定可设置为0

	double s,s2;
	//获取图像坐标
	Eigen::Vector3d imagePoint,api_ip;
	float p[2],wordP[3];
	imagePoint(0) = inPoints.x;  //832
	imagePoint(1) = inPoints.y;  //446
	imagePoint(2) = 1; //z_lengh; 0.78  //1

	
	api_ip(0) = api_camara[0];
	api_ip(1) = api_camara[1];
	api_ip(2) = api_camara[2];
	//计算比例参数S
	//cv::Mat tempMat, tempMat2;
	//Eigen::Matrix3d tempMat;
	Eigen::Vector3d	tempMat, tempMat2, tempMat3, tempMat4, tempMat5;
	tempMat = external_R.inverse() * intrisci_M.inverse() * imagePoint;
	tempMat2 = external_R.inverse() * external_t;
	//s = zConst + tempMat2(2, 0);
	//s /= tempMat(2, 0);

	s = (external_R.inverse() * external_t)(2) / (external_R.inverse() * intrisci_M.inverse() * imagePoint)(2);

	//cout << "Own func get camara corrdinate pos:" << intrisci_M.inverse() * imagePoint << endl;
	Eigen::Vector3d test = z_lengh * intrisci_M.inverse()* imagePoint;//转到相机坐标系下
	//计算世界坐标
	Eigen::Vector3d wcPoint =  external_R.inverse() * (s  * intrisci_M.inverse() * imagePoint) + external_t;
	//Eigen::Vector3d wcPoint2 = external_R.inverse() * (s2 * intrisci_M.inverse() * imagePoint) + external_t;


	Eigen::Vector3d APIwcPoint = external_R.inverse() * (api_ip) + external_t;//api_ip
	Eigen::Vector3d APIwcPoint1 = external_R * (api_ip)+external_t;//api_ip

	Point3f worldPoint(wcPoint(0, 0), wcPoint(1, 0), wcPoint(2, 0));
	Point3f worldPoint1(APIwcPoint(0, 0), APIwcPoint(1, 0), APIwcPoint(2, 0));
	Point3f worldPoint2(APIwcPoint1(0, 0), APIwcPoint1(1, 0), APIwcPoint1(2, 0)+0.050);
	return worldPoint2;
}

//single robot plan and control
void path_generate_ur_pcontrol(CartesianSpaceTrackUR* controller_ur)
{
	//test begin

	//test end

	planner_joint  myplanner;//planner_joint   planner_cartisian
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

	  external_M << 9.9999182276709764e-01, -2.5015706196520101e-03, -3.1775058414804393e-03, -3.6770868325372012e-02,
		2.4858905285792821e-03, 9.9998476195823494e-01, -4.9291175287571806e-03, -7.5690552499139668e-02,
		3.1897879581044689e-03, 4.9211782905393027e-03, 9.9998280348064750e-01, -2.5302551302136194e-02,
		0, 0, 0, 1;

	intenal_M << 8.8884850236407863e+02, 0., 6.4873567892875235e+02,//637.8,0,637.8,
		0., 8.8769039996290348e+02, 3.6643649723707466e+02,
		0., 0., 1.;


	octomap::point3d pmin(0.2, -1.3, 0.2);
	octomap::point3d pmax(1.3, 1.3, 1.2);
	Bigtree.setBBXMin(pmin);
	Bigtree.setBBXMax(pmax);
	Bigtree.useBBXLimit(true);
	Bigtree.enableChangeDetection(true);

	//octTree->setBBXMin(pmin);
	//octTree->setBBXMax(pmax);
	////octTree->useBBXLimit(true);
	//octTree->enableChangeDetection(true);


	//intenal_M << 9.09775146e+02, 0., 6.52678345e+02,//637.8,0,637.8,
	//	0., 9.09666321e+02, 3.64545776e+02,
	//	0., 0., 1.;

	std::ofstream pos_record("q_pos_record.txt", std::ios::trunc);//trunc
	boost::mutex::scoped_lock sl(mUpdate_ompl);
	cRun.wait(sl);
	boost::posix_time::ptime start_time = boost::get_system_time();
	controller_ur->start(start_time);
	double d_ms, cirtime_ms;
	float scale = 0.4;
	// the desired position specified by user or maybe the motion planner
	Eigen::Vector3d x_des;
	Eigen::Vector3d xd_des;
	Eigen::Vector3d xdd_des;

	boost::posix_time::ptime cur_time, circle_end_time;
	boost::posix_time::time_duration d_time, circle_time;
	//Eigen::AngleAxisd Rxyz_rotate_vector;
	Eigen::Matrix3d R_des;
	Eigen::Vector3d w_des;
	Eigen::Vector3d wd_des;

	double df_pos_orient[6];//目标位置和姿态XYZRPY
	bool have_solution = false, run_robot = false;
	double desire_q[6];
	x_des(0) = 0.5;
	x_des(1) = 0.5;
	x_des(2) = 0.5;
	int counter = 0;
	cv::Point3d Word_point_pre, Word_point, Word_point1,  camaraPoint;
	Vec6d targePoint_L, targePoint_R, camaP[4];//前后左右四个点
	targePoint_L(0) = 0.9;		targePoint_L(1) = 0.4;		targePoint_L(2) = 0.15;//0.25  0.15
	targePoint_L(3) = 2.224;		targePoint_L(4) = -2.222;		targePoint_L(5) = 0.0;
	targePoint_R(0) = 0.9;		targePoint_R(1) = -0.5;		targePoint_R(2) = 0.15;//0.25  0.15
	targePoint_R(3) = 2.224;		targePoint_R(4) = -2.222;		targePoint_R(5) = 0.0;
	camaraPoint.x = 0.6895;	camaraPoint.y = -0.1735;	camaraPoint.z = 0.7+0.2;
	//前后左右四个点
	camaP[0](0) = camaraPoint.x + 0.4-0.2; camaP[0](1) = camaraPoint.y;		  camaP[0](2) = camaraPoint.z;
	camaP[0](3) = 2.224; camaP[0](4) =-2.222;		  camaP[0](5) = 0;
	camaP[1](0) = camaraPoint.x - 0.2; camaP[1](1) = camaraPoint.y;		  camaP[1](2) = camaraPoint.z;
	camaP[1](3) = 2.224; camaP[1](4) = -2.222;		  camaP[1](5) = 0.;
	camaP[2](0) = camaraPoint.x ;	   camaP[2](1) = camaraPoint.y + 0.5; camaP[2](2) = camaraPoint.z;
	camaP[2](3) = 2.224; camaP[2](4) = -2.222;		  camaP[2](5) = 0.;
	camaP[3](0) = camaraPoint.x ;	   camaP[3](1) = camaraPoint.y - 0.1; camaP[3](2) = camaraPoint.z;
	camaP[3](3) = 2.224; camaP[3](4) = -2.222;		  camaP[3](5) = 0.;

	deque<Vec6d> goals;
	bool first = true;
	Sleep(1000);
	while (1)
	{
		cur_time = boost::get_system_time();
		d_time = cur_time - start_time;
		d_ms = (double)d_time.total_milliseconds() / 1000;
		printf("time: %f\n", d_ms);
		//controller_ur->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
		//实时计算三维点信息
		if (false and counter < 2000)
		{
			controller_ur->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
			Eigen::Vector4d Pos_target;
			Eigen::Vector3d point_target;//u v zC
			point_target << im3_actual_center[0], im3_actual_center[1], im3_actual_center[2];

			cout << "pixel pos and Zc:" << im3_actual_center[0] << " " << im3_actual_center[1] << " " << im3_actual_center[2] << endl;
			//Point3fcout << "Pos_target:" << Pos_target << endl;
			Eigen::Vector3d external_T = (controller_ur->Ttool_to_base * external_M).block<3, 1>(0, 3);
			Eigen::Matrix3d external_R = (controller_ur->Ttool_to_base * external_M).block<3, 3>(0, 0);

			cv::Point2d  Pixel_point;
			//if (im3_actual_center[0] == 0 or im3_actual_center[1] == 0 or im3_actual_center[2] == 0)
			//	continue;
			Pixel_point.x = im3_actual_center[0];
			Pixel_point.y = im3_actual_center[1];
			//cv::Point3d Word_point;
			Word_point = getWorldPoints(external_R, intenal_M, external_T, Pixel_point, im3_actual_center[2]);
			cout << "Word_point" << Word_point << im3_actual_center[2]<<endl;

		}
		if (false and pow((Word_point.x - Word_point_pre.x),2)+ pow((Word_point.y - Word_point_pre.y), 2)<0.04*0.04)
		{
			counter =counter + 1;
			//cout << "counter" << counter << endl;

		}
		
		//if (false and counter < 2000)
		//{
		//	Word_point_pre = Word_point;
		//	continue;
		//}
		//else
		if (first)//第一次运行至各个图像采集点  
		{
			//counter = 0;

			//Word_point.z = 0.25;
			goals.push_back(camaP[0]);//Word_point
			goals.push_back(camaP[1]);//targePoint
			goals.push_back(camaP[2]);//camaraPoint
			goals.push_back(camaP[3]);
		}
		else   //至两个目标点
		{

			double min_limit[3], min_limit2[3];
			double max_limit[3], max_limit2[3];
			Bigtree.getMetricMin(min_limit[0], min_limit[1], min_limit[2]);
			Bigtree.getMetricMax(max_limit[0], max_limit[1], max_limit[2]);	
			Bigtree.writeBinary("Bigtree_total.bt");
			Bigtree.write("Bigtree_total.ot");

			//Bigtree.read("static_occ06221728.ot");//simple_tree_total06131449
			//Bigtree.readBinary("Bigtree_total.bt");//simple_tree_total06131449
			cout << Bigtree.getResolution()<<endl;
			//Bigtree.setResolution(0.05);
			cout << Bigtree.getResolution() << endl;

			//double min_limit[3], min_limit2[3],M_size[3];
			//double max_limit[3], max_limit2[3];
			//Bigtree.getMetricMin(min_limit[0], min_limit[1], min_limit[2]);
			//Bigtree.getMetricMax(max_limit[0], max_limit[1], max_limit[2]);	

			//Bigtree.getMetricSize(M_size[0], M_size[1], M_size[2]);
			//point3d test =Bigtree.getBBXBounds();
			
			goals.push_back(targePoint_R);
			goals.push_back(targePoint_L);
			std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
		}

		//for(int k=0;k< goals.size();k++)
		while ( !goals.empty())
		{
			xd_des << 0, 0, 0;
			//xd_des << -scale * M_PI * radius * sin(scale * M_PI * d_ms), scale* M_PI* radius* cos(scale * M_PI * d_ms), 0;
			xdd_des << 0.0, 0.0, 0.0;
			srand(time(NULL));//设置随机数种子，使每次产生的随机序列不同
			df_pos_orient[0] = x_rnd_min + (x_rnd_max - x_rnd_min) * (rand() % N_ompl) / N_ompl;//X POS 
			df_pos_orient[1] = y_rnd_min + (y_rnd_max - y_rnd_min) * (rand() % N_ompl) / N_ompl;//Y POS
			df_pos_orient[2] = 0.2;// df_rnd_min + (df_rnd_max - df_rnd_min) * (rand() % N) / N;

			//set target oritention
			df_pos_orient[3] = 3.14159;//R ORIENT  -1.5708    -3.099
			df_pos_orient[4] = 0.0;//P ORIENT   0.0   -0.098
			df_pos_orient[5] = 1.57;//Y ORIENT  -3.1416   -0.535
			//Eigen::Vector3d eulerAngle_start_point = Rc.eulerAngles(2, 1, 0);
			Eigen::Vector3d eulerAngle(df_pos_orient[3], df_pos_orient[4], df_pos_orient[5]);
			Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
			Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
			Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
			Eigen::AngleAxisd rotation_vector;
			rotation_vector = yawAngle * pitchAngle * rollAngle;//ZYX
			Eigen::Matrix3d orient_rotation_matrix = Eigen::Matrix3d::Identity();
			orient_rotation_matrix = rotation_vector.matrix();
			Eigen::Matrix<double, 4, 4> des_Matrix = Eigen::Matrix4d::Identity();
			Eigen::Matrix<double, 4, 4> start_Matrix = Eigen::Matrix4d::Identity();
			start_Matrix.block<3, 3>(0, 0) = orient_rotation_matrix;
			//start_Matrix.block<3, 1>(0, 3) = x_des;
			des_Matrix.block<3, 3>(0, 0) = orient_rotation_matrix;
			des_Matrix.block<3, 1>(0, 3) = x_des;
			//cout << des_Matrix << endl;
			//由OMPL生成路径点
			deque<double> path_point;
			  myplanner._ur = controller_ur;//planner_joint   planner_cartisian
			 
			if (!first)
			{

	/*			auto fcl_octree = std::make_shared<fcl::OcTree<double>>(&Bigtree);
				std::shared_ptr<fcl::CollisionGeometry<double>> fcl_geometry = fcl_octree;*/
				//std::make_shared<fcl::CollisionObject>(fcl_geometry);

				fcl::OcTree<double>* tree = new fcl::OcTree<double>(std::shared_ptr<const octomap::OcTree>(&Bigtree));//Bigtree
				//tree->computeLocalAABB();
				//tree->toBoxes();
				std::shared_ptr<fcl::CollisionGeometry<double>>	tree_obj_geo = std::shared_ptr<fcl::CollisionGeometry<double>>(tree);
				//cout << tree_obj_geo->aabb_center << endl;
				//cout << tree_obj_geo->aabb_local.max_ << endl;
				//cout << tree_obj_geo->aabb_local.min_ << endl;
				//tree_obj_geo->computeLocalAABB();
				//tree_obj_geo->
				myplanner.updateMap(tree_obj_geo);
			}
				
			Eigen::VectorXd robot_pose,rob_cur_q;
			Vec6d goal_pos;
			//cv:Point3d p_target;
			goal_pos = goals[0];
			//goals.pop_back();
			//goal_pos[0] = p_target.x; goal_pos[1] = p_target.y; goal_pos[2] = p_target.z;
			robot_pose.resize(6);
			rob_cur_q.resize(6);
			controller_ur->update_p(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
			robot_pose = controller_ur->get_tool_pose();
			rob_cur_q(0) = controller_ur->q1;rob_cur_q(1) = controller_ur->q2;rob_cur_q(2) = controller_ur->q3;
			rob_cur_q(3) = controller_ur->q4;rob_cur_q(4) = controller_ur->q5;rob_cur_q(5) = controller_ur->q6;
			//cout << "robot_pose" << robot_pose << endl;
			//start_Matrix.block<3, 1>(0, 3) = robot_pose;
			myplanner.setStart(rob_cur_q);// 
			//myplanner.setStart(goal_pos[0], goal_pos[1], goal_pos[2], robot_pose(0), robot_pose(1), robot_pose(2));
			//if (abs(0.4 - robot_pose(4)) < 0.1)
			//{
			//	goal_pos[0] = 0.9; goal_pos[1] = -0.5; goal_pos[2] = 0.25;
			//}
			//else if (abs(-0.5 - robot_pose(4)) < 0.1)
			//{
			//	goal_pos[0] = 0.9; goal_pos[1] = 0.4; goal_pos[2] = 0.25;
			//}
			//else if (0.4-robot_pose(4) >abs(-0.5- robot_pose(4)))
			//{
			//	goal_pos[0] = 0.9; goal_pos[1] = -0.5; goal_pos[2] = 0.25;
			//}
			//else if (0.4 - robot_pose(4) < abs(-0.5 - robot_pose(4)))
			//{
			//	goal_pos[0] = 0.9; goal_pos[1] = 0.4; goal_pos[2] = 0.25;
			//}
			myplanner.setGoal(goal_pos[0], goal_pos[1], goal_pos[2], goal_pos[3], goal_pos[4], goal_pos[5]);

			myplanner.plan(path_point);
			if (path_point.empty())
				continue;
			if (myplanner.is_cartisian_planner)
			{
				if (pow((path_point[path_point.size() - 1] - goal_pos[0]), 2) + pow((path_point[path_point.size() - 2] - goal_pos[1]), 2)
					+ pow((path_point[path_point.size() - 3] - goal_pos[2]), 2) > 0.01 * 0.01 * 0.01)
				{
					path_point.clear();
					myplanner.replan(path_point);
				}
			}
			else
			{
				double q[6];
				Eigen::Matrix4d T;
				Eigen::Matrix3d R;
				Eigen::Vector3d V;
				q[0] = path_point[path_point.size() - 6];
				q[1] = path_point[path_point.size() - 5];
				q[2] = path_point[path_point.size() - 4];
				q[3] = path_point[path_point.size() - 3];
				q[4] = path_point[path_point.size() - 2];
				q[5] = path_point[path_point.size() - 1];
				controller_ur->fkRobot(q, T, R, V);
				double distace = sqrt(pow((V[0] - goal_pos[0]), 2) + pow((V[1] - goal_pos[1]), 2)
					+ pow((V[2] - goal_pos[2]), 2));
				if (distace > 0.05)
				{
					path_point.clear();
					myplanner.replan(path_point);
				}
			}

			while (!path_point.empty())
			{
				if (!myplanner.is_cartisian_planner)
				{
					for (int i = 0; i < 6; i++)
					{
						desire_q[i] = path_point.front();
						path_point.pop_front();
						cout<<desire_q[i] << setw(12);
					}
					cout << endl;
				}
				else
				{
					des_Matrix(0, 3) = path_point.front();
					path_point.pop_front();
					des_Matrix(1, 3) = path_point.front();
					path_point.pop_front();
					des_Matrix(2, 3) = path_point.front();
					path_point.pop_front();

					have_solution = controller_ur->ik_with_q(des_Matrix, desire_q);
					//if (!have_solution)
					//{
					//	cout << "无逆解..............." << endl;
					//	continue;
					//}
				}

				w_des.setZero();
				wd_des.setZero();

				for (int i = 0; i < 6; i++)
				{
					controller_ur->q_pos_target[i] = desire_q[i];
					//cout << setw(12) << controller_ur->q_pos_target[i];
					pos_record << setw(12) << controller_ur->q_pos_target[i];
				}
				//cout << endl;
				pos_record << endl;
				//设置目标关节位置
				controller_ur->set_Qdesired_position(desire_q);

				while (1)
				{
					//求qcd（关节速度）
					controller_ur->update_p(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));

					//下发关节速度
					for (int i = 0; i < CONTROL_PERIOD_MS / UR_CYCLIC_TIME_MS; i++)
						//if (!run_robot)
					{
						//设置关节速度和关节加速度
						controller_ur->specify_qcd((double)UR_CYCLIC_TIME_MS * (i + 1) / CONTROL_PERIOD_MS);
						cRun.wait(sl);
						run_robot = true;
					}

					if (abs(desire_q[0] - controller_ur->q1) < 0.02 &&
						abs(desire_q[1] - controller_ur->q2) < 0.02 &&
						abs(desire_q[2] - controller_ur->q3) < 0.02 &&
						abs(desire_q[3] - controller_ur->q4) < 0.02 &&
						abs(desire_q[4] - controller_ur->q5) < 0.02 &&
						abs(desire_q[5] - controller_ur->q6) < 0.02)
					{
						if (path_point.empty())
						{
							controller_ur->specify_zero_qcd();
							//增加采集图像
							if    ((abs(controller_ur->x(0) - camaP[0](0)) < 0.033 and abs(controller_ur->x(1) - camaP[0](1)) <0.033 and abs(controller_ur->x(2) - camaP[0](2)) < 0.033)
								or (abs(controller_ur->x(0) - camaP[1](0)) < 0.033 and abs(controller_ur->x(1) - camaP[1](1)) <0.033 and abs(controller_ur->x(2) - camaP[1](2)) < 0.033)
								or (abs(controller_ur->x(0) - camaP[2](0)) < 0.033 and abs(controller_ur->x(1) - camaP[2](1)) <0.033 and abs(controller_ur->x(2) - camaP[2](2)) < 0.033)
								or (abs(controller_ur->x(0) - camaP[3](0)) < 0.033 and abs(controller_ur->x(1) - camaP[3](1)) <0.033 and abs(controller_ur->x(2) - camaP[3](2)) < 0.033))
							{
								cur_time = boost::get_system_time();
								d_time = cur_time - start_time;
								d_ms = (double)d_time.total_milliseconds() / 1000;
								printf("function begin time: %f\n", d_ms);

								//cameraToBase(slave_base_to_master_base* slave_robot_tool_to_base * external_M, intenal_M, depthmatR, Bigtree);
								cameraToBase(controller_ur->Ttool_to_base* external_M, intenal_M, depthmatR, Bigtree);
								//Sleep(1000);
								cur_time = boost::get_system_time();
								d_time = cur_time - start_time;
								d_ms = (double)d_time.total_milliseconds() / 1000;
								printf("function engd time: %f\n", d_ms);
							}
							goals.pop_front();
							
						}

						break;
					}
				}
			}
		}
		first = false;
		//break;
		//counter = 0;
	}
	pos_record.close();
	boost::posix_time::ptime stop_time = boost::get_system_time();
	controller_ur->stop(stop_time);
}

//two robot plan and control
void ompl_path_control_ur(CartesianSpaceTrackUR* controller_ur, CartesianSpaceTrackUR* controller_ur2)
{
	
	//planner_joint_dualarm  myplanner;//planner_joint   planner_cartisian
	Eigen::Matrix4d rob_left_base_to_rob_right_base, slave_base_to_master_base;

	rob_left_base_to_rob_right_base << 
		0.982936, 0.183593, 0.0114527, -1.90168,
		-0.18375, 0.982864, 0.0146152, 0.199745,
		-0.0085732, -0.0164702, 0.999828, 0.00565541,
		0, 0, 0, 1;
	slave_base_to_master_base = rob_left_base_to_rob_right_base.inverse();
	//camara external para
	external_M << 9.9999182276709764e-01, -2.5015706196520101e-03, -3.1775058414804393e-03, -3.6770868325372012e-02,
		2.4858905285792821e-03, 9.9998476195823494e-01, -4.9291175287571806e-03, -7.5690552499139668e-02,
		3.1897879581044689e-03, 4.9211782905393027e-03, 9.9998280348064750e-01, -2.5302551302136194e-02,
		0, 0, 0, 1;
	//camara internal para
	intenal_M << 8.8884850236407863e+02, 0., 6.4873567892875235e+02,//637.8,0,637.8,
		0., 8.8769039996290348e+02, 3.6643649723707466e+02,
		0., 0., 1.;


	octomap::point3d pmin(0.2, -1.3, 0.2);
	octomap::point3d pmax(1.3, 1.3, 1.2);
	Bigtree.setBBXMin(pmin);
	Bigtree.setBBXMax(pmax);
	Bigtree.useBBXLimit(true);
	Bigtree.enableChangeDetection(true);

	std::ofstream pos_record("q_pos_record.txt", std::ios::trunc);//trunc
	boost::mutex::scoped_lock sl(mUpdate_ompl);
	boost::mutex::scoped_lock s2(mUpdate2_ompl);
	cRun.wait(sl);
	cRun2.wait(s2);
	boost::posix_time::ptime start_time = boost::get_system_time();
	controller_ur->start(start_time);
	controller_ur2->start(start_time);
	//cRun.wait(sl);
	//cRun2.wait(sl);
	double d_ms, cirtime_ms;
	float scale = 0.4;
	// the desired position specified by user or maybe the motion planner
	Eigen::Vector3d x_des;
	Eigen::Vector3d xd_des;
	Eigen::Vector3d xdd_des;

	boost::posix_time::ptime cur_time, circle_end_time;
	boost::posix_time::time_duration d_time, circle_time;
	//Eigen::AngleAxisd Rxyz_rotate_vector;
	Eigen::Matrix3d R_des;
	Eigen::Vector3d w_des;
	Eigen::Vector3d wd_des;

	double df_pos_orient[6];//目标位置和姿态XYZRPY
	bool have_solution = false, run_robot = false;
	double desire_q[6], desire2_q[6];

	int counter = 0;
	cv::Point3d Word_point_pre, Word_point, Word_point1,camaraPoint;
	Vec<double, 13> MasterL_slaveR, MasterR_slaveL;
	Vec<double, 13> camaP[4];
	MasterL_slaveR << 0.9,  0.4, 0.15, 2.224, -2.222, 0, 0.9, -0.5, 0.15, 2.224, 2.222, 0,1;
	MasterR_slaveL << 0.9, -0.5, 0.15, 2.224, -2.222, 0, 0.9, 0.4, 0.15, 2.224, 2.222, 0, 1;
	//Vec6d targePoint_L, targePoint_R, camaP[4];//前后左右四个点--拍深度图用
	//targePoint_L(0) = 0.9;		targePoint_L(1) = 0.4;		targePoint_L(2) = 0.15;//0.25  0.15
	//targePoint_L(3) = 2.224;		targePoint_L(4) = -2.222;		targePoint_L(5) = 0.0;
	//targePoint_R(0) = 0.9;		targePoint_R(1) = -0.5;		targePoint_R(2) = 0.15;//0.25  0.15
	//targePoint_R(3) = 2.224;		targePoint_R(4) = -2.222;		targePoint_R(5) = 0.0;
	camaraPoint.x = 0.6895;	camaraPoint.y = -0.1735;	camaraPoint.z = 0.7 + 0.2;
	//Sleep(200);
	
	//前后左右四个点
	controller_ur->update_p(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
	controller_ur2->update_p(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
	
	cRun.wait(sl);
	cRun2.wait(s2);
	
	Eigen::VectorXd rob_pose(6),rob2_pose(6);
	//cout << controller_ur->q1 << endl;
	//cout << controller_ur2->q1 << endl;
	//rob_pose = controller_ur->get_tool_pose();
	//cout << rob_pose << endl;
	rob2_pose= controller_ur2->get_tool_pose();

	cout << rob2_pose << endl;
	camaP[0](0) = camaraPoint.x + 0.4 - 0.2; camaP[0](1) = camaraPoint.y;		  camaP[0](2) = camaraPoint.z;
	camaP[0](3) = 2.224; camaP[0](4) = -2.222;		  camaP[0](5) = 0;
	camaP[1](0) = camaraPoint.x - 0.2; camaP[1](1) = camaraPoint.y;		  camaP[1](2) = camaraPoint.z;
	camaP[1](3) = 2.224; camaP[1](4) = -2.222;		  camaP[1](5) = 0.;
	camaP[2](0) = camaraPoint.x;	   camaP[2](1) = camaraPoint.y + 0.5; camaP[2](2) = camaraPoint.z;
	camaP[2](3) = 2.224; camaP[2](4) = -2.222;		  camaP[2](5) = 0.;
	camaP[3](0) = camaraPoint.x;	   camaP[3](1) = camaraPoint.y - 0.1; camaP[3](2) = camaraPoint.z;
	camaP[3](3) = 2.224; camaP[3](4) = -2.222;		  camaP[3](5) = 0.;
	for (int i = 0; i < 4; i++)
	{
		camaP[i](6) = rob2_pose(3); camaP[i](7)  = rob2_pose(4); camaP[i](8)  = rob2_pose(5);
		camaP[i](9) = rob2_pose(0); camaP[i](10) = rob2_pose(1); camaP[i](11) = rob2_pose(2);
		camaP[i](12) = 0;
	}
	
	deque< Vec<double, 13>> goals;
	bool first = true;
	Sleep(500);
	while (1)
	{
		cur_time = boost::get_system_time();
		d_time = cur_time - start_time;
		d_ms = (double)d_time.total_milliseconds() / 1000;
		//printf("time: %f\n", d_ms);
		//controller_ur->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));

		if (first)//第一次运行至各个图像采集点  
		{
			goals.push_back(camaP[0]);//Word_point
			goals.push_back(camaP[1]);//targePoint
			goals.push_back(camaP[2]);//camaraPoint
			goals.push_back(camaP[3]);
		}
		else   //至两个目标点
		{

			double min_limit[3], min_limit2[3];
			double max_limit[3], max_limit2[3];
			Bigtree.getMetricMin(min_limit[0], min_limit[1], min_limit[2]);
			Bigtree.getMetricMax(max_limit[0], max_limit[1], max_limit[2]);
			Bigtree.writeBinary("Bigtree_total.bt");
			Bigtree.write("Bigtree_total.ot");

			//Bigtree.read("static_occ06221728.ot");//simple_tree_total06131449
			//Bigtree.readBinary("Bigtree_total.bt");//simple_tree_total06131449


			goals.push_back(MasterR_slaveL);
			goals.push_back(MasterL_slaveR);
			std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
		}

		//for(int k=0;k< goals.size();k++)
		while (!goals.empty())
		{
			xd_des << 0, 0, 0;
			//xd_des << -scale * M_PI * radius * sin(scale * M_PI * d_ms), scale* M_PI* radius* cos(scale * M_PI * d_ms), 0;
			xdd_des << 0.0, 0.0, 0.0;
			srand(time(NULL));//设置随机数种子，使每次产生的随机序列不同
			df_pos_orient[0] = x_rnd_min + (x_rnd_max - x_rnd_min) * (rand() % N_ompl) / N_ompl;//X POS 
			df_pos_orient[1] = y_rnd_min + (y_rnd_max - y_rnd_min) * (rand() % N_ompl) / N_ompl;//Y POS
			df_pos_orient[2] = 0.2;// df_rnd_min + (df_rnd_max - df_rnd_min) * (rand() % N) / N;

			//set target oritention
			df_pos_orient[3] = 3.14159;//R ORIENT  -1.5708    -3.099
			df_pos_orient[4] = 0.0;//P ORIENT   0.0   -0.098
			df_pos_orient[5] = 1.57;//Y ORIENT  -3.1416   -0.535
			//Eigen::Vector3d eulerAngle_start_point = Rc.eulerAngles(2, 1, 0);
			Eigen::Vector3d eulerAngle(df_pos_orient[3], df_pos_orient[4], df_pos_orient[5]);
			Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
			Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
			Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
			Eigen::AngleAxisd rotation_vector;
			rotation_vector = yawAngle * pitchAngle * rollAngle;//ZYX
			Eigen::Matrix3d orient_rotation_matrix = Eigen::Matrix3d::Identity();
			orient_rotation_matrix = rotation_vector.matrix();
			Eigen::Matrix<double, 4, 4> des_Matrix = Eigen::Matrix4d::Identity();
			Eigen::Matrix<double, 4, 4> start_Matrix = Eigen::Matrix4d::Identity();
			start_Matrix.block<3, 3>(0, 0) = orient_rotation_matrix;
			//start_Matrix.block<3, 1>(0, 3) = x_des;
			des_Matrix.block<3, 3>(0, 0) = orient_rotation_matrix;
			des_Matrix.block<3, 1>(0, 3) = x_des;
			//cout << des_Matrix << endl;
			//由OMPL生成路径点
			deque<double> path_point;
			myplanner._ur = controller_ur;//planner_joint   planner_cartisian
			myplanner._ur2 = controller_ur2;//planner_joint   planner_cartisian
			if (!first)
			{
				fcl::OcTree<double>* tree = new fcl::OcTree<double>(std::shared_ptr<const octomap::OcTree>(&Bigtree));//Bigtree
				std::shared_ptr<fcl::CollisionGeometry<double>>	tree_obj_geo = std::shared_ptr<fcl::CollisionGeometry<double>>(tree);
				myplanner.updateMap(tree_obj_geo);
				//delete tree;//这里新增的，报错记得删除
			}

			Eigen::VectorXd robot_pose, rob_cur_q, rob2_cur_q;
			Vec<double, 13> goal_pos;
			//cv:Point3d p_target;
			goal_pos = goals[0];
			//goals.pop_back();
			//goal_pos[0] = p_target.x; goal_pos[1] = p_target.y; goal_pos[2] = p_target.z;
			//robot_pose.resize(6);
			rob_cur_q.resize(6);
			rob2_cur_q.resize(6);
			controller_ur->update_p(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
			controller_ur2->update_p(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
			cRun.wait(sl);
			cRun2.wait(s2);
			//robot_pose = controller_ur->get_tool_pose();
			rob_cur_q(0) = controller_ur->q1; rob_cur_q(1) = controller_ur->q2; rob_cur_q(2) = controller_ur->q3;
			rob_cur_q(3) = controller_ur->q4; rob_cur_q(4) = controller_ur->q5; rob_cur_q(5) = controller_ur->q6;

			rob2_cur_q(0) = controller_ur2->q1; rob2_cur_q(1) = controller_ur2->q2; rob2_cur_q(2) = controller_ur2->q3;
			rob2_cur_q(3) = controller_ur2->q4; rob2_cur_q(4) = controller_ur2->q5; rob2_cur_q(5) = controller_ur2->q6;
			cout << "robot_pose" << goal_pos << endl;
			//start_Matrix.block<3, 1>(0, 3) = robot_pose;
			myplanner.setStart(rob_cur_q, rob2_cur_q);// 
			myplanner.setGoal(goal_pos[0], goal_pos[1], goal_pos[2], goal_pos[3], goal_pos[4], goal_pos[5],
							  goal_pos[6], goal_pos[7], goal_pos[8], goal_pos[9], goal_pos[10], goal_pos[11], goal_pos[12]);

			myplanner.plan(path_point);
			if (path_point.empty() )
				continue;
			if (path_point.size()<70*12 &&(!first)&& controller_ur->x(2)<0.3)
				continue;
			if (myplanner.is_cartisian_planner)
			{
				if (pow((path_point[path_point.size() - 1] - goal_pos[0]), 2) + pow((path_point[path_point.size() - 2] - goal_pos[1]), 2)
					+ pow((path_point[path_point.size() - 3] - goal_pos[2]), 2) > 0.01 * 0.01 * 0.01)
				{
					path_point.clear();
					myplanner.replan(path_point);
				}

			}
			else
			{
				double q[6];
				Eigen::Matrix4d T;
				Eigen::Matrix3d R;
				Eigen::Vector3d V;
				q[0] = path_point[path_point.size() - 6-6];
				q[1] = path_point[path_point.size() - 5-6];
				q[2] = path_point[path_point.size() - 4-6];
				q[3] = path_point[path_point.size() - 3-6];
				q[4] = path_point[path_point.size() - 2-6];
				q[5] = path_point[path_point.size() - 1-6];
				controller_ur->fkRobot(q, T, R, V);
				double distace = sqrt(pow((V[0] - goal_pos[0]), 2) + pow((V[1] - goal_pos[1]), 2)
					+ pow((V[2] - goal_pos[2]), 2));
				if (distace > 0.05)
				{
					path_point.clear();
					continue;
					//myplanner.replan(path_point);
				}
			}

			while (!path_point.empty())
			{
				if (!myplanner.is_cartisian_planner)
				{
					//for rob1
					for (int i = 0; i < 6; i++)
					{
						desire_q[i] = path_point.front();
						path_point.pop_front();
						cout << desire_q[i] << setw(12);
					}
					//for rob2
					for (int i = 0; i < 6; i++)
					{
						desire2_q[i] = path_point.front();
						path_point.pop_front();
						cout << desire2_q[i] << setw(12);
					}
					cout << endl;
				}
				else
				{
					des_Matrix(0, 3) = path_point.front();
					path_point.pop_front();
					des_Matrix(1, 3) = path_point.front();
					path_point.pop_front();
					des_Matrix(2, 3) = path_point.front();
					path_point.pop_front();

					have_solution = controller_ur->ik_with_q(des_Matrix, desire_q);
					//if (!have_solution)
					//{
					//	cout << "无逆解..............." << endl;
					//	continue;
					//}
				}

				w_des.setZero();
				wd_des.setZero();

				for (int i = 0; i < 6; i++)
				{
					controller_ur->q_pos_target[i] = desire_q[i];
					controller_ur2->q_pos_target[i] = desire2_q[i];
					//cout << setw(12) << controller_ur->q_pos_target[i];
					//pos_record << setw(12) << controller_ur->q_pos_target[i]; //record rob pos
				}
				//cout << endl;
				pos_record << endl;
				//设置目标关节位置
				controller_ur->set_Qdesired_position(desire_q);
				controller_ur2->set_Qdesired_position(desire2_q);
				while (1)
				{
					//求qcd（关节速度）
					controller_ur->update_p(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
					controller_ur2->update_p(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
					//下发关节速度
					for (int i = 0; i < CONTROL_PERIOD_MS / UR_CYCLIC_TIME_MS; i++)
						//if (!run_robot)
					{
						//设置关节速度和关节加速度
						controller_ur->specify_qcd((double)UR_CYCLIC_TIME_MS * (i + 1) / CONTROL_PERIOD_MS);
						controller_ur2->specify_qcd((double)UR_CYCLIC_TIME_MS * (i + 1) / CONTROL_PERIOD_MS);
						cRun.wait(sl);
						cRun2.wait(s2);
						run_robot = true;
					}

					if (abs(desire_q[0] - controller_ur->q1) < 0.02 &&
						abs(desire_q[1] - controller_ur->q2) < 0.02 &&
						abs(desire_q[2] - controller_ur->q3) < 0.02 &&
						abs(desire_q[3] - controller_ur->q4) < 0.04 &&
						abs(desire_q[4] - controller_ur->q5) < 0.04 &&
						abs(desire_q[5] - controller_ur->q6) < 0.04 &&//以下是2号机器人位置差值
						abs(desire2_q[0] - controller_ur2->q1) < 0.02 &&
						abs(desire2_q[1] - controller_ur2->q2) < 0.02 &&
						abs(desire2_q[2] - controller_ur2->q3) < 0.02 &&
						abs(desire2_q[3] - controller_ur2->q4) < 0.04 &&
						abs(desire2_q[4] - controller_ur2->q5) < 0.04 &&
						abs(desire2_q[5] - controller_ur2->q6) < 0.04)
					{
						if (path_point.empty())
						{
							controller_ur->specify_zero_qcd();
							controller_ur2->specify_zero_qcd();
							//增加采集图像
							if    ((abs(controller_ur->x(0) - camaP[0](0)) < 0.033 and abs(controller_ur->x(1) - camaP[0](1)) < 0.033 and abs(controller_ur->x(2) - camaP[0](2)) < 0.033)
								or (abs(controller_ur->x(0) - camaP[1](0)) < 0.033 and abs(controller_ur->x(1) - camaP[1](1)) < 0.033 and abs(controller_ur->x(2) - camaP[1](2)) < 0.033)
or (abs(controller_ur->x(0) - camaP[2](0)) < 0.033 and abs(controller_ur->x(1) - camaP[2](1)) < 0.033 and abs(controller_ur->x(2) - camaP[2](2)) < 0.033)
	or (abs(controller_ur->x(0) - camaP[3](0)) < 0.033 and abs(controller_ur->x(1) - camaP[3](1)) < 0.033 and abs(controller_ur->x(2) - camaP[3](2)) < 0.033))
							{
							cur_time = boost::get_system_time();
							d_time = cur_time - start_time;
							d_ms = (double)d_time.total_milliseconds() / 1000;
							//printf("function begin time: %f\n", d_ms);

							//cameraToBase(slave_base_to_master_base * slave_robot_tool_to_base * external_M, intenal_M, depthmatR, Bigtree);
							cameraToBase(controller_ur->Ttool_to_base* external_M, intenal_M, depthmatR, Bigtree);
							Sleep(1000);
							cur_time = boost::get_system_time();
							d_time = cur_time - start_time;
							d_ms = (double)d_time.total_milliseconds() / 1000;
							//printf("function engd time: %f\n", d_ms);
							}
goals.pop_front();

						}

						break;
					}
				}


			}
		}
		first = false;
	}
	pos_record.close();
	boost::posix_time::ptime stop_time = boost::get_system_time();
	controller_ur->stop(stop_time);
}
void ur_ibvs_control(CartesianSpaceTrackUR* controller_ur)
{
	intenal_M << 8.8884850236407863e+02, 0., 6.4873567892875235e+02,//637.8,0,637.8,
		0., 8.8769039996290348e+02, 3.6643649723707466e+02,
		0., 0., 1.;

	std::ofstream pos_record("q_pos_record.txt", std::ios::trunc);//trunc
	boost::mutex::scoped_lock sl(mUpdate_ompl);
	cRun.wait(sl);
	boost::posix_time::ptime start_time = boost::get_system_time();
	controller_ur->start(start_time);
	double d_ms, cirtime_ms;
	float scale = 0.4;
	// the desired position specified by user or maybe the motion planner
	Eigen::Vector3d x_des;
	Eigen::Vector3d xd_des;
	Eigen::Vector3d xdd_des;

	boost::posix_time::ptime cur_time, circle_end_time;
	boost::posix_time::time_duration d_time, circle_time;
	//Eigen::AngleAxisd Rxyz_rotate_vector;
	Eigen::Matrix3d R_des;
	Eigen::Vector3d w_des;
	Eigen::Vector3d wd_des;
	Sleep(1000);

	cur_time = boost::get_system_time();
	d_time = cur_time - start_time;
	d_ms = (double)d_time.total_milliseconds() / 1000;
	printf("time: %f\n", d_ms);
	{
		//controller_ur->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
		// im3_actual_center[0], im3_actual_center[1], im3_actual_center[2];
		controller_ur->im3point_act[2] = 0.5;
		controller_ur->im3point_act[5] = 0.5;
		controller_ur->im3point_act[8] = 0.5;
		controller_ur->im3point_act[11] = 0.5;
		//874         178           0         673         160           0
		//641         450           0         864         445           0
		//期望点位置
		//controller_ur->im3point_des[0] = 475.7; controller_ur->im3point_des[1] = 254.8;
		//controller_ur->im3point_des[2] = 783;   controller_ur->im3point_des[3] = 242.3;
		//controller_ur->im3point_des[4] = 484.6; controller_ur->im3point_des[5] = 444.5;
		//controller_ur->im3point_des[6] = 789.4; controller_ur->im3point_des[7] = 432.3;
		//matlab data test
		//426.5229  426.5229  870.9471  870.9471
		//144.2239  588.6481  588.6481  144.2239
		controller_ur->im3point_des[0] = 426.5229; controller_ur->im3point_des[1] = 144.2239;
		controller_ur->im3point_des[2] = 426.5229; controller_ur->im3point_des[3] = 588.6481;
		controller_ur->im3point_des[4] = 870.9471; controller_ur->im3point_des[5] = 588.6481;
		controller_ur->im3point_des[6] = 870.9471; controller_ur->im3point_des[7] = 144.2239;

	}

	while (1)
	{
		//每次固定在0.5，注释则用实际测量值
		//im3_actual_center[2] = 0.5;
		//im3_actual_center[5] = 0.5;
		//im3_actual_center[8] = 0.5;
		//im3_actual_center[11]= 0.5;
		if ((im3_actual_center[0] == 0 && im3_actual_center[1] == 0) || cam_ivbs_error)
		{
			for (int i = 0; i < 4; i++)
			{
				controller_ur->im3point_act[i*3] = controller_ur->im3point_des[i*2];
				controller_ur->im3point_act[i*3+1] = controller_ur->im3point_des[i*2+1];
				controller_ur->im3point_act[i*3+2] = 2.2;//0.5
			}
			controller_ur->specify_zero_qcd();
			cRun.wait(sl);
//			continue;
		}
		else
		{
			for (int i = 0; i < 12; i++)
				{
				controller_ur->im3point_act[i] =im3_actual_center[i];
				}
		}
		//可用于无传感器测试
		//for (int i = 0; i < 4; i++)
		//{
		//	controller_ur->im3point_act[i * 3] = controller_ur->im3point_des[i * 2]-100;
		//	controller_ur->im3point_act[i * 3 + 1] = controller_ur->im3point_des[i * 2 + 1]-100;
		//	controller_ur->im3point_act[i * 3 + 2] = 0.5;
		//}
			//求qcd（关节速度）
		controller_ur->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));

			//下发关节速度
		for (int i = 0; i < CONTROL_PERIOD_MS / UR_CYCLIC_TIME_MS; i++)
			{
				//设置关节速度和关节加速度
				controller_ur->specify_qcd((double)UR_CYCLIC_TIME_MS * (i + 1) / CONTROL_PERIOD_MS);
				cRun.wait(sl);
			}
		}
	pos_record.close();
	boost::posix_time::ptime stop_time = boost::get_system_time();
	controller_ur->stop(stop_time);
}

void ur_redbox_control(CartesianSpaceTrackUR* controller_ur)
{
	double camera_factor = 1000.0;
	intenal_M << 8.8884850236407863e+02, 0., 6.4873567892875235e+02,//637.8,0,637.8,
		0., 8.8769039996290348e+02, 3.6643649723707466e+02,
		0., 0., 1.;
	external_M << 9.9999182276709764e-01, -2.5015706196520101e-03, -3.1775058414804393e-03, -3.6770868325372012e-02,
		2.4858905285792821e-03, 9.9998476195823494e-01, -4.9291175287571806e-03, -7.5690552499139668e-02,
		3.1897879581044689e-03, 4.9211782905393027e-03, 9.9998280348064750e-01, -2.5302551302136194e-02,
		0, 0, 0, 1;
	std::ofstream pos_record("q_pos_record.txt", std::ios::trunc);//trunc
	boost::mutex::scoped_lock sl(mUpdate_ompl);
	cRun.wait(sl);
	boost::posix_time::ptime start_time = boost::get_system_time();
	controller_ur->start(start_time);
	double d_ms, cirtime_ms;
	float scale = 0.4;
	Eigen::Vector3d p;
	// the desired position specified by user or maybe the motion planner
	Eigen::Vector3d x_des;
	Eigen::Vector3d xd_des;
	Eigen::Vector3d xdd_des;

	boost::posix_time::ptime cur_time, circle_end_time;
	boost::posix_time::time_duration d_time, circle_time;
	//Eigen::AngleAxisd Rxyz_rotate_vector;
	Eigen::Matrix3d R_des;
	Eigen::Vector3d w_des;
	Eigen::Vector3d wd_des;
	//Sleep(1000);
	controller_ur->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
	cur_time = boost::get_system_time();
	d_time = cur_time - start_time;
	d_ms = (double)d_time.total_milliseconds() / 1000;
	printf("time: %f\n", d_ms);

	while (1)
	{
		Eigen::Matrix3d orient_rotation_matrix;
		//Eigen::Matrix3d m_rot;
		if (cam_redbox_error )
		{
			x_des = controller_ur->x;
			orient_rotation_matrix = controller_ur->Ttool_to_base.block<3, 3>(0, 0);
		}
		else
		{

			Eigen::Matrix4d Mobj2base;
			Eigen::Matrix4d M_trans;
			M_trans.setIdentity();
			M_trans(0, 3) = -Mobj_cama(0, 3);
			M_trans(1, 3) = -Mobj_cama(1, 3);
			M_trans(2, 3) = -Mobj_cama(2,3);
			Mobj2base = controller_ur->Ttool_to_base*Mobj_cama* M_trans;
			//cout << Mobj2base << endl << endl;

			Eigen::Vector3d euler_angles = m_rot.eulerAngles(2, 1, 0);
			Eigen::AngleAxisd yaw, roll, pitch;

			if (euler_angles[0]<1.57/2)
				yaw =(Eigen::AngleAxisd((0, 0, -euler_angles[0]), Eigen::Vector3d::UnitZ()));
			else
				yaw = (Eigen::AngleAxisd((0, 0, (3.14159-euler_angles[0])), Eigen::Vector3d::UnitZ()));
			if (euler_angles[1] > 3.14159 / 2)
				pitch = (Eigen::AngleAxisd(3.14159 -euler_angles[1], Eigen::Vector3d::UnitY()));
			else if (euler_angles[1] < -3.14159 / 2)
				pitch = (Eigen::AngleAxisd(-(3.14159 +euler_angles[1]), Eigen::Vector3d::UnitY()));
			else 
				pitch = (Eigen::AngleAxisd( euler_angles[1], Eigen::Vector3d::UnitY()));
			if(euler_angles[2] > 3.14159/2)
				roll=(Eigen::AngleAxisd(((0,0,-(-3.14159  + euler_angles[2]))), Eigen::Vector3d::UnitX()));
			else if (euler_angles[2] <- 3.14159 / 2)
				roll=(Eigen::AngleAxisd(((0,0,-(3.14159 + euler_angles[2]))), Eigen::Vector3d::UnitX()));
			else
				roll = (Eigen::AngleAxisd(((0, 0, -euler_angles[2])), Eigen::Vector3d::UnitX()));
			//cout << euler_angles[2] << endl;//-3.14159+euler_angles[1]
			orient_rotation_matrix = yaw * pitch*roll *  controller_ur->Ttool_to_base.block<3, 3>(0, 0);
			//Eigen::AngleAxisd rotation_vector;
			//rotation_vector = yaw * pitch * roll;//ZYX
			//Mobj_cama
			Eigen::Matrix4d M_tran;
			M_tran.setIdentity();
			M_tran(0, 3) = 0.08+Mobj_cama(0, 3);//0.2
			M_tran(1, 3) = 0.08+Mobj_cama(1, 3);//0.15
			M_tran(2, 3) =  Mobj_cama(2, 3)-0.5;
			x_des = (controller_ur->Ttool_to_base* M_tran).block<3,1>(0,3);
		}

		//cout << orient_rotation_matrix << endl;
		xd_des << 0, 0, 0;
		xdd_des << 0.0, 0.0, 0.0;
		//设置笛卡儿坐标系x,y,z位置，速度，加速度
		controller_ur->set_desired_position(x_des, xd_des, xdd_des);
		//controller_ur2->set_desired_position(x_des, xd_des, xdd_des);
		//设置笛卡儿坐标系姿态位置，速度，加速度
		controller_ur->set_desired_orientation(orient_rotation_matrix, w_des, wd_des);
		//controller_ur->set_desired_orientation(R_des, w_des, wd_des);
		//update():读关节实际位置，关节实际速度；读关节目标位置
		//计算末端笛卡儿位姿，计算雅可比矩阵，求笛卡儿空间速度和姿态速度
		//最后求qcd（关节速度）
			//求qcd（关节速度）
		controller_ur->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));

		//下发关节速度
		for (int i = 0; i < CONTROL_PERIOD_MS / UR_CYCLIC_TIME_MS; i++)
		{
			//设置关节速度和关节加速度
			controller_ur->specify_qcd((double)UR_CYCLIC_TIME_MS * (i + 1) / CONTROL_PERIOD_MS);
			cRun.wait(sl);
		}
	}
	pos_record.close();
	boost::posix_time::ptime stop_time = boost::get_system_time();
	controller_ur->stop(stop_time);
}