#include "path_generator.h"
#define UR_CYCLIC_TIME_MS 8
#define CONTROL_PERIOD_MS 8   //40

// variable used in the control thread of UR
extern boost::condition_variable cRun, cRun2;
boost::mutex mUpdate_ompl, mUpdate2_ompl;
bool running_ompl;
double x_rnd_min = 0.4;
double x_rnd_max = 1.0;
double y_rnd_min = -1.0;
double y_rnd_max = 1.0;
const int  N_ompl = 100; //精度为小数点后面2位
void path_generate_ur_pcontrol(CartesianSpaceTrackUR* controller_ur)
{
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
	controller_ur->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
	while (1)
	{
		cur_time = boost::get_system_time();
		d_time = cur_time - start_time;
		d_ms = (double)d_time.total_milliseconds() / 1000;
		printf("time: %f\n", d_ms);


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
		des_Matrix.block<3, 3>(0, 0) = orient_rotation_matrix;
		des_Matrix.block<3, 1>(0, 3) = x_des;
		cout << des_Matrix << endl;
		//由OMPL生成路径点
		deque<double> path_point;
		planner  myplanner(controller_ur);
		Eigen::VectorXd robot_pose;
		robot_pose.resize(6);
		robot_pose = controller_ur->get_tool_pose();
		myplanner.setStart(robot_pose(3), robot_pose(4), robot_pose(5));
		if (abs(0.4- robot_pose(4))  < 0.1)
			myplanner.setGoal(0.9, -0.5, 0.25);//0.84, -0.5, 0.15
		else if (abs(-0.5 - robot_pose(4)) < 0.1)
			myplanner.setGoal(0.9, 0.4, 0.25);//0.84, 0.4, 0.15
		else if (0.4-robot_pose(4) >abs(-0.5- robot_pose(4)))
			myplanner.setGoal(0.9, -0.5, 0.25);//
		else if (0.4 - robot_pose(4) < abs(-0.5 - robot_pose(4)))
			myplanner.setGoal(0.9, 0.4, 0.25);
		//myplanner.setGoal(df_pos_orient[0], df_pos_orient[1], 0.2);
		//myplanner.setStart(0.7, 0.4, 0.2);//(0.4, 0.4, 0.2);  0.76, 0.34, 0.3
		//myplanner.setGoal(0.7, -0.4, 0.2);
		myplanner.plan(path_point);
		while (!path_point.empty())
		{
			des_Matrix(0, 3) = path_point.front();
			path_point.pop_front();
			des_Matrix(1, 3) = path_point.front();
			path_point.pop_front();
			des_Matrix(2, 3) = path_point.front();
			path_point.pop_front();

			have_solution = controller_ur->ik_with_q(des_Matrix, desire_q);
			if (!have_solution)
			{
				cout << "无逆解..............." << endl;
				continue;
			}

			w_des.setZero();
			wd_des.setZero();

			for (int i = 0; i < 6; i++)
			{
				controller_ur->q_pos_target[i] = desire_q[i];
				cout << setw(12) << controller_ur->q_pos_target[i];
				pos_record << setw(12) << controller_ur->q_pos_target[i];
			}
			cout << endl;
			pos_record << endl;
			//设置目标关节位置
			controller_ur->set_Qdesired_position(desire_q);

			while ( 1 )
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

				if (abs(desire_q[0] - controller_ur->q1) < 0.01	&&
					abs(desire_q[1] - controller_ur->q2) < 0.01 &&
					abs(desire_q[2] - controller_ur->q3) < 0.01 &&
					abs(desire_q[3] - controller_ur->q4) < 0.01 &&
					abs(desire_q[4] - controller_ur->q5) < 0.01 &&
					abs(desire_q[5] - controller_ur->q6) < 0.01)
				{
					if (path_point.empty())
						controller_ur->specify_zero_qcd();
					break;
				}
				

			}


		}

	}
	pos_record.close();
	boost::posix_time::ptime stop_time = boost::get_system_time();
	controller_ur->stop(stop_time);
}


void ompl_path_control_ur(CartesianSpaceTrackUR* controller_ur)
{
	std::ofstream for_loop_time("for_loop_time.txt", std::ios::trunc);//trunc
	boost::mutex::scoped_lock sl(mUpdate_ompl);
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

	double df_pos_orient[6];//目标位置和姿态XYZRPY
	while (running_ompl)
	{
		cur_time = boost::get_system_time();
		d_time = cur_time - start_time;
		d_ms = (double)d_time.total_milliseconds() / 1000;
		printf("time: %f\n", d_ms);


		//sep target position
		x_des(0) = -0.700;
		x_des(1) = -0.500;
		x_des(2) = 0.4;

		xd_des << 0, 0, 0;

		//xd_des << -scale * M_PI * radius * sin(scale * M_PI * d_ms), scale* M_PI* radius* cos(scale * M_PI * d_ms), 0;
		xdd_des << 0.0, 0.0, 0.0;

		//set target oritention
		df_pos_orient[3] = -3.099;//R ORIENT  -1.5708
		df_pos_orient[4] = -0.098;//P ORIENT   0.0
		df_pos_orient[5] = -0.535;//Y ORIENT  -3.1416
		//Eigen::Vector3d eulerAngle_start_point = Rc.eulerAngles(2, 1, 0);
		Eigen::Vector3d eulerAngle(df_pos_orient[3], df_pos_orient[4], df_pos_orient[5]);
		Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
		Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
		Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
		Eigen::AngleAxisd rotation_vector;
		rotation_vector = yawAngle * pitchAngle * rollAngle;//ZYX
		Eigen::Matrix3d orient_rotation_matrix = Eigen::Matrix3d::Identity();
		orient_rotation_matrix = rotation_vector.matrix();
		
		w_des.setZero();
		wd_des.setZero();
		//li bo end
		// 
		//设置笛卡儿坐标系x,y,z位置，速度，加速度
		controller_ur->set_desired_position(x_des, xd_des, xdd_des);
		//设置笛卡儿坐标系姿态位置，速度，加速度
		controller_ur->set_desired_orientation(orient_rotation_matrix, w_des, wd_des);
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
	for_loop_time.close();
}