// ur_romote_control.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include <iostream>

#include <fstream>
#include <cartesian_space_track_ur.h>
#include <camera.h>
#include <ur_sdk.h>
#include <conio.h>
#include <opencv2\opencv.hpp>
#include <vector>
#define _USE_MATH_DEFINES
#include<math.h>

#include <iomanip>
#include <algorithm>


#define UR_CYCLIC_TIME_MS 8
#define CONTROL_PERIOD_MS 40

// variable used in the control thread of UR
boost::condition_variable cRun;
boost::mutex mUpdate;
bool running;

std::vector<cv::Point2f> left_points, right_points;
// position of the target
Eigen::Vector3d pos_target;

using namespace Robot;

const double ZERO_THRESH = 0.00000001;
int SIGN(double x) {
	return (x > 0) - (x < 0);
}
//UR3
const double d1 = 0.1519;
const double L2 = -0.24365;
const double L3 = -0.21325;
const double d4 = 0.11235;
const double d5 = 0.08535;
const double d6 = 0.0819;
//UR10
//const double d1 = 0.1273;
//const double L2 = -0.612;
//const double L3 = -0.5723;
//const double d4 = 0.163941;
//const double d5 = 0.1157;
//const double d6 = 0.0922;


void control_ur(CartesianSpaceTrackUR* controller_ur)
{

	/*
		double qa[6] = { 0.1 ,0.2,0.3,0.4,0.5,0.6 }, qb[6] = { 0.7 ,0.8,0.9,1.0,1.1,1.2 };//q为关节角度
	double ta[4][4], tb[4][4];
	double* Qa = &qa[0], * Qb = &qb[0], * Ta = &ta[0][0], * Tb = &tb[0][0];
	CartesianSpaceTrackUR::cal_pose(Qa, Ta);  CartesianSpaceTrackUR::cal_pose(Qb, Tb);
	cout << "初始矩阵Ta： " << endl;  CartesianSpaceTrackUR::OutputT(ta); cout << "终止矩阵Tb： " << endl;  CartesianSpaceTrackUR::OutputT(tb);
	int t = 10;
	CartesianSpaceTrackUR::Ctraj(ta, tb, t);
	*/






	boost::mutex::scoped_lock sl(mUpdate);
	cRun.wait(sl);
	boost::posix_time::ptime start_time = boost::get_system_time();
	controller_ur->start(start_time);
	double d_ms;
	float scale = 0.4;
	// the desired position specified by user or maybe the motion planner
	Eigen::Vector3d x_des;
	Eigen::Vector3d xd_des;
	Eigen::Vector3d xdd_des;

	double centerX = 0.35;
	double centerY = -0.1;
	double centerZ = 0.2;
	double radius = 0.1;
	boost::posix_time::ptime cur_time;
	boost::posix_time::time_duration d_time;
	//Eigen::AngleAxisd Rxyz_rotate_vector;
	Eigen::Matrix3d R_des;
	Eigen::Vector3d w_des;
	Eigen::Vector3d wd_des;
	while (running)
	{

		cur_time = boost::get_system_time();
		d_time = cur_time - start_time;
		d_ms = (double)d_time.total_milliseconds() / 1000;
		printf("time: %f\n", d_ms);


		x_des(0) = centerX + radius * (cos(scale * M_PI * d_ms));
		x_des(1) = centerY + radius * sin(scale * M_PI * d_ms);
		x_des(2) = centerZ;



		/*
			x_des(0) =0.0;
			x_des(1) = -0.194;
			x_des(2) = 0.694;
		*/


		//xd_des << 0, 0, 0;

		xd_des << -scale * M_PI * radius * sin(scale * M_PI * d_ms), scale* M_PI* radius* cos(scale * M_PI * d_ms), 0;
		xdd_des << 0.0, 0.0, 0.0;

		Eigen::AngleAxisd Rxyz_rotate_vector(M_PI / 4, Eigen::Vector3d(0, 1, 0));
		R_des.setIdentity();
		R_des = Rxyz_rotate_vector.matrix();
		w_des.setZero();
		wd_des.setZero();
		//设置笛卡儿坐标系x,y,z位置，速度，加速度
		controller_ur->set_desired_position(x_des, xd_des, xdd_des);
		//设置笛卡儿坐标系姿态位置，速度，加速度
		controller_ur->set_desired_orientation(R_des, w_des, wd_des);
		//update():读关节实际位置，关节实际速度；读关节目标位置
		//计算末端笛卡儿位姿，计算雅可比矩阵，求笛卡儿空间速度和姿态速度
		//最后求qcd（关节速度）
		controller_ur->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
		for (int i = 0; i < CONTROL_PERIOD_MS / UR_CYCLIC_TIME_MS; i++)
		{
			//设置关节速度和关节加速度
			controller_ur->specify_qcd((double)UR_CYCLIC_TIME_MS * (i + 1) / CONTROL_PERIOD_MS);
			cRun.wait(sl);
		}

	}

	boost::posix_time::ptime stop_time = boost::get_system_time();
	controller_ur->stop(stop_time);
}


void cartesion_control_ur(CartesianSpaceTrackUR* controller_ur)
{
	double qa[6] = {    0,-M_PI / 3,M_PI / 2,-M_PI / 2, 0, 0 }, qb[6] = { 0,-M_PI / 2,0,-M_PI / 2, 0, 0 };//q为关节角度
	double q_soltemp[6],q_last_circle[6] = { 0 };
	double ta[4][4], tb[4][4];

	double* Qa = &qa[0], * Qb = &qb[0]; //* Ta = &ta[0][0], * Tb = &tb[0][0];

	Eigen::Matrix<double, 4, 4>  Ta, Tb;
	Eigen::Matrix<double, 3, 3>  Ra, Rb;
	Eigen::Vector3d  Va, Vb;
	
	controller_ur->fkRobot(Qa, Ta, Ra, Va); controller_ur->fkRobot(Qb, Tb, Rb, Vb);

	double T_temp[4][4]; double* T_Temp = &T_temp[0][0];
	double* test_temp;
	double* test_temp_old;
	double qq = 0.0;// double* Q = &q[0];
	double s = 0.1;
	Eigen::Matrix3d R5;
	double p1[3], p2[3], p_temp[3];
	double q_sols[48], q_deleate_sols[48] ;
	int num_sols,num_remained_sols;
	

	
	boost::mutex::scoped_lock sl(mUpdate);
	cRun.wait(sl);
	boost::posix_time::ptime start_time = boost::get_system_time();
	controller_ur->start(start_time);
	double d_ms;
	float scale = 0.4;
	// the desired position specified by user or maybe the motion planner
	Eigen::Vector3d x_des;
	Eigen::Vector3d xd_des;
	Eigen::Vector3d xdd_des;


	boost::posix_time::ptime cur_time;
	boost::posix_time::time_duration d_time;
	//Eigen::AngleAxisd Rxyz_rotate_vector;
	Eigen::Matrix3d R_des;
	Eigen::Vector3d w_des;
	Eigen::Vector3d wd_des;
	int itotaltime = 400;//10s finish motion ,40MS一个周期
	int icounter = 0;
	float fincrease = 0.0;
	double time_increase = 0.0;
	double df_run_total_time = 5.0;//单次规划运动时间
	//double df_i_run_total_time = 4;//单次规划运动时间
	bool b_first_run = true;
	double df_array_q_act_post[6];
	double df_array_q_choosed_pos[6];
	double df_temp;
	double* q_pos_sequence = new double[6 * itotaltime];

	//关节插值结束
	controller_ur->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
	//if (controller_ur->q1 < M_PI)
	//	q_last_circle[0] = controller_ur->q1;
	//else
	//	q_last_circle[0] = controller_ur->q1 -2*M_PI;

	//if (controller_ur->q2 < M_PI)
	//	q_last_circle[1] = controller_ur->q2;
	//else
	//	q_last_circle[1] = controller_ur->q2 - 2 * M_PI;

	//if (controller_ur->q3 < M_PI)
	//	q_last_circle[2] = controller_ur->q3;
	//else
	//	q_last_circle[2] = controller_ur->q3 - 2 * M_PI;

	//if (controller_ur->q4 < M_PI)
	//	q_last_circle[3] = controller_ur->q4;
	//else
	//	q_last_circle[3] = controller_ur->q4 - 2 * M_PI;

	//if (controller_ur->q5 < M_PI)
	//	q_last_circle[4] = controller_ur->q5;
	//else
	//	q_last_circle[4] = controller_ur->q5 - 2 * M_PI;

	//if (controller_ur->q6 < M_PI)
	//	q_last_circle[5] = controller_ur->q6;
	//else
	//	q_last_circle[5] = controller_ur->q6 - 2 * M_PI;


	for (int i = 0; i < 6; i++)
	{
		q_last_circle[i] = qa[i];
	}
	ofstream read_pos("read_pos.txt", ios::trunc);
	ofstream set_pos("set_pos.txt", ios::trunc);
	//controller_ur->Ctraj_gen_point_sequence(Ta, Tb, q_pos_sequence, itotaltime);
	while (running)
	{

		cur_time = boost::get_system_time();
		d_time = cur_time - start_time;
		d_ms = (double)d_time.total_milliseconds() / 1000;
		printf("time: %f\n", d_ms);
		//fincrease =(float)icounter / itotaltime;
		fincrease = 1.0 <= ((float)icounter / itotaltime) ? 1.0 : ((float)icounter / itotaltime);
		time_increase = d_ms <= df_run_total_time ? d_ms : df_run_total_time;
		Eigen::VectorXd vxd_q_act_post = controller_ur->get_q();
		double* df_q_act_post = vxd_q_act_post.transpose().data();
		//test_temp = controller_ur->Ctraj_cubic_poly(Ta, Tb, time_increase, df_run_total_time);
		test_temp = controller_ur->Ctraj(Ta, Tb, fincrease);
		//cout << endl;
		//for (int mk = 0; mk < 16; mk++)
		//	{	
		//		if (0 == mk % 4)
		//			cout << endl;
		//		cout << setw(14) << test_temp[mk]<< setw(14) << test_temp[mk];
	
		//	}
		//cout << endl;

		num_sols = controller_ur->inverse(test_temp, q_sols, qq);

		num_remained_sols=controller_ur->deleate_cannot_reach(num_sols,q_sols, q_deleate_sols);
		for (int mk = 0; mk < 48; mk++)
		{	
			if (0 == mk % 6)
				cout << endl;
			cout << setw(9) << q_sols[mk];
			
		}
		cout << endl;
		for (int mk = 0; mk < 48; mk++)
		{
			if (0 == mk % 6)
				cout << endl;
			cout << setw(9) << q_deleate_sols[mk];

		}
		cout << endl;

		cout << "num_sols="<<num_sols << endl;
		cout << "num_remained_sols=" << num_remained_sols << endl;

		df_q_act_post[0] = controller_ur->q1;
		df_q_act_post[1] = controller_ur->q2;
		df_q_act_post[2] = controller_ur->q3;
		df_q_act_post[3] = controller_ur->q4;
		df_q_act_post[4] = controller_ur->q5;
		df_q_act_post[5] = controller_ur->q6;

		//controller_ur->solution_choose_total_diff(num_remained_sols, q_deleate_sols, q_last_circle, df_array_q_choosed_pos); // q_last_circle  df_q_act_post
		controller_ur->solution_choose_sigle_axis_diff(num_remained_sols, q_deleate_sols, df_q_act_post, df_array_q_choosed_pos);
		cout << "The choosed solutions ";
		for (int im = 0; im < 6; im++)
		{
			//if (std::isnan(df_array_q_choosed_pos[im]) or std::isinf(df_array_q_choosed_pos[im]))//or std::isfinite()
			//{
			//	controller_ur->q_pos_target[im] = df_q_act_post[im];
			//	continue;
			//}
			if (abs(df_array_q_choosed_pos[im]) < ZERO_THRESH)
				controller_ur->q_pos_target[im] = 0;
			else if (df_array_q_choosed_pos[im] <= M_PI and df_array_q_choosed_pos[im] >0.0)
				controller_ur->q_pos_target[im] = df_array_q_choosed_pos[im];
			else if (df_array_q_choosed_pos[im] < 2*M_PI and df_array_q_choosed_pos[im] >= M_PI)
				controller_ur->q_pos_target[im] = df_array_q_choosed_pos[im]-2 * M_PI;
			else
				controller_ur->q_pos_target[im] = df_q_act_post[im];
			
			cout <<setw(10)<< controller_ur->q_pos_target[im] <<"  ";
		}
		cout << endl;


		//将数据输出至out.txt文件中
		for (int i = 0; i < 6; i++)
		{
			set_pos <<setw(12)<<controller_ur->q_pos_target[i];
			read_pos << setw(12) << df_q_act_post[i];
		};
		set_pos << std::endl;
		read_pos << std::endl;


		cout << "The last solutions ";
		for (int i = 0; i < 6; i++)
		{
			cout << setw(10) << q_last_circle[i] << "  ";
		}
		cout << endl;

		cout << "The actual solutions ";
		for (int i = 0; i < 6; i++)
		{
			cout << setw(10) << df_q_act_post[i] << "  ";
		}
		cout << endl;
		x_des(0) = p_temp[0];
		x_des(1) = p_temp[1];
		x_des(2) = p_temp[2];

		xd_des << 0, 0, 0;

		//xd_des << -scale * M_PI * radius * sin(scale * M_PI * d_ms), scale* M_PI* radius* cos(scale * M_PI * d_ms), 0;
		xdd_des << 0.0, 0.0, 0.0;

		//Eigen::AngleAxisd Rxyz_rotate_vector(M_PI / 4, Eigen::Vector3d(0, 1, 0));
		R_des.setIdentity();
		//R_des = R5.matrix();
		w_des.setZero();
		wd_des.setZero();
		Eigen::VectorXd q_received_data;

		q_received_data =controller_ur->get_tool_pose();
		//设置笛卡儿坐标系x,y,z位置，速度，加速度
		controller_ur->set_desired_position(x_des, xd_des, xdd_des);
		//设置笛卡儿坐标系姿态位置，速度，加速度
		controller_ur->set_desired_orientation(R_des, w_des, wd_des);
		//update():读关节实际位置，关节实际速度；读关节目标位置
		//计算末端笛卡儿位姿，计算雅可比矩阵，求笛卡儿空间速度和姿态速度
		//最后求要下发的的qcd（关节速度）
		controller_ur->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
		//for (int i = 0; i < CONTROL_PERIOD_MS / UR_CYCLIC_TIME_MS; i++)
		//{
		//	if (abs(controller_ur->x(0) - Vb(0) > 0.001) || abs(controller_ur->x(1) - Vb(1) > 0.001) || abs(controller_ur->x(2) - Vb(2) > 0.001))
		//	{

				//设置关节速度和关节加速度
		controller_ur->specify_qcd((double)UR_CYCLIC_TIME_MS * (0 + 1) / CONTROL_PERIOD_MS);
		//	}
		cRun.wait(sl);
		//}
			//cRun.wait(sl);
		icounter++;
		for (int i = 0; i < 6; i++)
		{
			q_last_circle[i] = df_array_q_choosed_pos[i];
		}
		b_first_run = false;
	}
	set_pos.close();
	read_pos.close();
	boost::posix_time::ptime stop_time = boost::get_system_time();
	controller_ur->stop(stop_time);
}


void joint_space_control_ur(CartesianSpaceTrackUR* controller_ur)
{
	double qa[6] = {  0,-M_PI / 2, 0,-M_PI / 2, 0, 0 }, qb[6] = { M_PI / 3,-M_PI * 4 / 5, -M_PI * 1 / 3,-M_PI / 4,0, 0 };//q为关节角度
	double q_soltemp[6] = { 0 };
	double ta[4][4], tb[4][4];

	double* Qa = &qa[0], * Qb = &qb[0]; //* Ta = &ta[0][0], * Tb = &tb[0][0];

	Eigen::Matrix<double, 4, 4>  Ta, Tb;
	Eigen::Matrix<double, 3, 3>  Ra, Rb;
	Eigen::Vector3d  Va, Vb;
	controller_ur->fkRobot(Qa, Ta, Ra, Va); controller_ur->fkRobot(Qb, Tb, Rb, Vb);

	double T_temp[4][4]; double* T_Temp = &T_temp[0][0];
	double qq = 0.0;// double* Q = &q[0];
	double s = 0.1;
	Eigen::Matrix3d R5;
	double p1[3], p2[3], p_temp[3];
	double q_sols[48]; int num_sols;

	Eigen::Quaterniond quat1(Ra), quat2(Rb), quat_temp;

	boost::mutex::scoped_lock sl(mUpdate);
	cRun.wait(sl);
	boost::posix_time::ptime start_time = boost::get_system_time();
	controller_ur->start(start_time);
	double d_ms;
	float scale = 0.4;
	// the desired position specified by user or maybe the motion planner
	Eigen::Vector3d x_des;
	Eigen::Vector3d xd_des;
	Eigen::Vector3d xdd_des;

	boost::posix_time::ptime cur_time;
	boost::posix_time::time_duration d_time;
	//Eigen::AngleAxisd Rxyz_rotate_vector;
	Eigen::Matrix3d R_des;
	Eigen::Vector3d w_des;
	Eigen::Vector3d wd_des;
	int itotaltime = 500;//10s finish motion ,40MS一个周期
	int icounter = 0;
	double fincrease = 0.0;
	double time_increase = 0.0;
	double df_run_total_time = 5.0;//单次规划运动时间
	bool b_first_run = true;
	double df_array_q_act_post[6];
	double df_array_q_choosed_pos[6];
	//关节插值开始
	//1.关节空间只是插值，就用QA，QB分别设置位置，后插值，赋值给机器人。
	//2.关节空间先正解，再反解，再插值，赋值给机器人
	//关节空间插值时，反解一次
	//for(int)
	double dfArray[16];
	Eigen::Matrix<double, 4, 4> tb_trans;

	tb_trans = (Tb.transpose());
	T_Temp = tb_trans.data();//T_Temp

	//cout << "Tb= " << Tb << endl;
	//cout << "tb_trans=" << tb_trans << endl;
	num_sols = controller_ur->inverse(T_Temp, q_sols, qq);
	cout << "solutions="<<num_sols << endl;
	for (int i = 0; i < 48; i++)
	{
		if (0 == i % 6) cout << endl;
		cout << setw(12) << q_sols[i];
	}
	cout << endl;
	controller_ur->solution_choose_total_diff(num_sols, q_sols, qa, df_array_q_choosed_pos);
	cout << "start poing=";
	for (int i = 0; i < 6; ++i)
	{
		cout << setw(16) << qa[i];
	}
	cout << endl;
	cout << "input=";
	for (int i = 0; i < 6; ++i)
	{
		cout << setw(16) << qb[i];
	}
	cout << endl;
	cout << "output=";
	for (int i = 0; i < 6; ++i)
	{
		cout << setw(16) << df_array_q_choosed_pos[i];
	}
	cout << endl;
	//itotaltime =(int) (df_i_run_total_time / 0.008);
	//关节插值结束
	while (running)
	{
		cur_time = boost::get_system_time();
		d_time = cur_time - start_time;
		d_ms = (double)d_time.total_milliseconds() / 1000;
		printf("time: %f\n", d_ms);
		//fincrease =(float)icounter / itotaltime;
		fincrease = 1.0 <= ((double)icounter / itotaltime) ? 1.0 : ((double)icounter / itotaltime);
		time_increase = d_ms <= df_run_total_time ? d_ms : df_run_total_time;
		Eigen::VectorXd vxd_q_act_post = controller_ur->get_q();
		double* df_q_act_post = vxd_q_act_post.transpose().data();

		//关节空间插值	
		double* q_temp;

		for (int im = 0; im < 6; im++)
		{
			if (df_array_q_choosed_pos[im] <= M_PI)
				df_array_q_choosed_pos[im] = df_array_q_choosed_pos[im];
			else
				df_array_q_choosed_pos[im] = df_array_q_choosed_pos[im] - 2 * M_PI;
			//cout << df_array_q_choosed_pos[im] << endl;
		}
			   //q_temp = controller_ur->joint_traj(&qa[0], &df_array_q_choosed_pos[0], fincrease);
		  q_temp = controller_ur->joint_space_3traj(&qa[0], &df_array_q_choosed_pos[0], time_increase,df_run_total_time);
		for (int i = 0; i < 6; i++)
		{
			q_soltemp[i] = q_temp[i]; 
			//cout << "The test q" << q_soltemp[i] << endl;
		}

		for (int im = 0; im < 6; im++)
		{
			controller_ur->q_pos_target[im] = q_soltemp[im];
		}

		//xd_des << -scale * M_PI * radius * sin(scale * M_PI * d_ms), scale* M_PI* radius* cos(scale * M_PI * d_ms), 0;
		xdd_des << 0.0, 0.0, 0.0;

		//Eigen::AngleAxisd Rxyz_rotate_vector(M_PI / 4, Eigen::Vector3d(0, 1, 0));
		R_des.setIdentity();
		//R_des = R5.matrix();
		w_des.setZero();
		wd_des.setZero();
		controller_ur->get_tool_pose();
		//设置笛卡儿坐标系x,y,z位置，速度，加速度
		controller_ur->set_desired_position(x_des, xd_des, xdd_des);
		//设置笛卡儿坐标系姿态位置，速度，加速度
		controller_ur->set_desired_orientation(R_des, w_des, wd_des);
		//update():读关节实际位置，关节实际速度；读关节目标位置
		//计算末端笛卡儿位姿，计算雅可比矩阵，求笛卡儿空间速度和姿态速度
		//最后求要下发的的qcd（关节速度）
		controller_ur->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
		//for (int i = 0; i < CONTROL_PERIOD_MS / UR_CYCLIC_TIME_MS; i++)
		//{
		//	if (abs(controller_ur->x(0) - Vb(0) > 0.001) || abs(controller_ur->x(1) - Vb(1) > 0.001) || abs(controller_ur->x(2) - Vb(2) > 0.001))
		//	{

				//设置关节速度和关节加速度
		controller_ur->specify_qcd((double)UR_CYCLIC_TIME_MS * (0 + 1) / CONTROL_PERIOD_MS);
		//	}
		cRun.wait(sl);
		//}
			//cRun.wait(sl);
		icounter++;

	}

	boost::posix_time::ptime stop_time = boost::get_system_time();
	controller_ur->stop(stop_time);
}


int main(int argc, char* argv[])
{



// variables for connecting ur
	boost::asio::io_service io_service;
	boost::asio::io_service::work work(io_service);
	ur_sdk::client client(io_service, cRun, argv[3], argv[4]);

	CartesianSpaceTrackUR controller_ur(client);
	controller_ur.controller_type = CartesianSpaceTrackUR::CONTROLLER_TYPE::FULL_POSE;
	controller_ur.Kp = 2.0;


	running = true;
	// start the threads
	boost::thread thread_io_service(boost::bind(&boost::asio::io_service::run, &io_service));
	//boost::thread thread_control_ur(boost::bind(&control_ur, &controller_ur));
	boost::thread thread_control_ur(boost::bind(&cartesion_control_ur, &controller_ur));//joint_space_control_ur cartesion_control_ur
	//boost::thread thread_detect_target(boost::bind(&detect_target, &stereo_camera));
	//boost::thread thread_show(boost::bind(&show_stereo, &stereo_camera));

	SetThreadPriority(thread_io_service.native_handle(), THREAD_PRIORITY_TIME_CRITICAL);
	SetThreadPriority(thread_control_ur.native_handle(), THREAD_PRIORITY_TIME_CRITICAL);



	_getch();
	running = false;

	io_service.stop();
	thread_io_service.join();
	thread_control_ur.join();
	//thread_detect_target.join();
	//thread_show.join();

	return -1;
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
