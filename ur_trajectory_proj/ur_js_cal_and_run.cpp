//#include"ur_js_cal_and_run.h"

void   Delay(int   time)//time*1000为秒数 
{
	clock_t   now = clock();

	while (clock() - now < time);
}

const double ZERO_THRESH = 0.00000001;
int SIGN(double x) {
	return (x > 0) - (x < 0);
}

void ur_cal_and_run(CartesianSpaceTrackUR* controller_ur)
{
	double qa[6] = { 0,-M_PI / 2,0,-M_PI / 2, 0, 0 }, qb[6] = { -M_PI / 3,-M_PI / 3,M_PI / 3,-M_PI / 2, 0,0 }, qc[6], qd[6];//q为关节角度
	double q_soltemp[6], q_last_circle[6] = { 0 };
	double ta[4][4], tb[4][4], tc[4][4], td[4][4];

	double* Qa = &qa[0], * Qb = &qb[0], * Qc = &qc[0], * Qd = &qd[0];// = &qc[0]; //* Ta = &ta[0][0], * Tb = &tb[0][0];

	Eigen::Matrix<double, 4, 4>  Ta, Tb, Tc, Td;
	Eigen::Matrix<double, 3, 3>  Ra, Rb, Rc, Rd;
	Eigen::Vector3d  Va, Vb, Vc, Vd;

	double T_temp[4][4]; double* T_Temp = &T_temp[0][0];
	double* test_temp;
	double* test_temp_old;
	double qq = 0.0;// double* Q = &q[0];
	double s = 0.1;
	Eigen::Matrix3d R5;
	double p1[3], p2[3], p_temp[3];
	double q_sols[48], q_deleate_sols[48];
	int num_sols, num_remained_sols;



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
	int itotaltime = 500;//10s finish motion ,40MS一个周期 400
	int icounter = 0;
	float fincrease = 0.0;
	double time_increase = 0.0;
	double df_run_total_time = 5.0;//单次规划运动时间
	//double df_i_run_total_time = 4;//单次规划运动时间
	bool b_running = false;
	double df_array_q_act_post[6];
	double df_array_q_choosed_pos[6];
	double df_temp;

	int i_input = 0;
	Eigen::VectorXd vct_cur_q;
	double df_two_point_distance = 0;
	double df_run_speed = 0.15;
	double df_two_point_ori_distance = 0;
	double df_run_ori_speed = 2.0;

	double df_rnd_x = 0.0;
	double df_rnd_y = 0.0;
	double df_rnd_z = 0.3;
	double df_rnd_min = 0.15;
	double df_rnd_max = 0.4;
	#define N  40 //精度为小数点后面2位
	double df_pos_orient[6];//目标位置和姿态XYZRPY
	double df_previous_pos[3] = { 0.0 };//0:x,1:y,2:z
	//Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0, 0, 1));
	for (int i = 0; i < 6; i++)
	{
		qd[i] = qb[i];
	}

	double* q_pos_sequence = nullptr;// = new double[6 * itotaltime + 6];
	double* q_speed_sequence = nullptr;
	double* q_acc_sequence = nullptr;
	while (running)
	{
		//i_input++;
		//关节插值结束
		controller_ur->update(boost::get_system_time(), boost::posix_time::milliseconds(CONTROL_PERIOD_MS));
		vct_cur_q = controller_ur->get_q();

		controller_ur->fkRobot(Qa, Ta, Ra, Va);
		controller_ur->fkRobot(Qb, Tb, Rb, Vb);
		controller_ur->fkRobot(Qc, Tc, Rc, Vc);
		controller_ur->fkRobot(Qd, Td, Rd, Vd);

		ofstream read_pos("read_pos.txt", ios::trunc);
		ofstream set_pos("set_pos.txt", ios::app);
		ofstream total_point("total_point.txt", ios::trunc);
		//ofstream set_pos("set_pos.txt", ios::trunc);
		std::ofstream act_xyz_pos("act_xyz_pos.csv", std::ios::trunc);//trunc
		cout << "input the b_running" << endl;
		srand(time(NULL));//设置随机数种子，使每次产生的随机序列不同
		df_rnd_x = df_rnd_min + (df_rnd_max - df_rnd_min) * (rand() % N) / N;
		df_rnd_y = df_rnd_min + (df_rnd_max - df_rnd_min) * (rand() % N) / N;//-0.5
		//df_rnd_x = 0;
		//df_rnd_y = -0.19425;
		cout << "df_rnd_x= " << df_rnd_x << endl;
		cout << "df_rnd_y= " << df_rnd_y << endl;
		df_pos_orient[0] = -0.275;//X POS -0.275
		df_pos_orient[1] = -0.2625;//Y POS -0.2625
		df_pos_orient[2] = df_rnd_z;//Z POS
		df_pos_orient[3] = -1.5708;//R ORIENT
		df_pos_orient[4] = 0;//P ORIENT
		df_pos_orient[5] = -3.1416;//Y ORIENT
		Eigen::Vector3d eulerAngle_start_point = Rc.eulerAngles(2, 1, 0);
		Eigen::Vector3d eulerAngle(df_pos_orient[3], df_pos_orient[4], df_pos_orient[5]);
		Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
		Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
		Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
		Eigen::AngleAxisd rotation_vector;
		rotation_vector = yawAngle * pitchAngle * rollAngle;//ZYX
		Eigen::Matrix3d orient_rotation_matrix = Eigen::Matrix3d::Identity();
		orient_rotation_matrix = rotation_vector.matrix();
		Td.setIdentity();
		Td.block<3, 3>(0, 0) = orient_rotation_matrix;
		Td(0, 3) = df_pos_orient[0];//x POS
		Td(1, 3) = df_pos_orient[1];//y POS
		Td(2, 3) = df_pos_orient[2];//z POS

		if (!b_running || (sqrt(pow(abs(df_pos_orient[0] - df_previous_pos[0]), 2) + pow(abs(df_pos_orient[1] - df_previous_pos[1]), 2) > 0.2)))//
		{
			//Delay(1 * 4000);   //延时5秒  
			df_two_point_ori_distance = (max(abs(eulerAngle_start_point(0) - eulerAngle(0)), abs(eulerAngle_start_point(1) - eulerAngle(1))), abs(eulerAngle_start_point(2) - eulerAngle(2)));
			df_two_point_distance = sqrt(pow(abs(df_pos_orient[0] - Vd[0]), 2) + pow(abs(df_pos_orient[1] - Vd[1]), 2)
				+ pow(abs(Vc[2] - Vd[2]), 2));
			//itotaltime = 500;
			itotaltime = max((df_two_point_distance / df_run_speed), df_two_point_ori_distance / df_run_ori_speed) * 125;//距离除以速度=时间,每秒有125次循环(点),时间乘125,等于点数
			cout << "itotaltime= " << itotaltime << endl;
			total_point << setw(12) << itotaltime << endl;
			q_pos_sequence = new double[6 * itotaltime + 6];
			q_speed_sequence = new double[6 * itotaltime + 6];
			q_acc_sequence = new double[6 * itotaltime + 6];

			bool b_generate_success = controller_ur->joint_gen_point_sequence(Tc, Td, q_pos_sequence, q_speed_sequence, q_acc_sequence, itotaltime);
			//bool b_generate_success = controller_ur->Ctraj_gen_point_sequence(Tc, Td, q_pos_sequence, itotaltime);
			if (!b_generate_success)
			{
				cout << "生成轨迹失败!" << endl;
				itotaltime = -1;
			}
			else
			{
				icounter = 0;
				for (int mk = 0; mk < 3; mk++)
					df_previous_pos[mk] = df_pos_orient[mk];
			}
		}

		//while ((icounter < itotaltime + 1))
		{
			//b_running = true;
			Eigen::VectorXd vxd_q_act_post = controller_ur->get_q();
			for (int im = 0; im < 6; im++)
			{
				controller_ur->q_pos_target[im] = q_pos_sequence[im + icounter * 6];
				controller_ur->qcd[im] = q_speed_sequence[im + icounter * 6];
				if (abs(controller_ur->q_pos_target[im]) < ZERO_THRESH)
					controller_ur->q_pos_target[im] = 0;
			}
			controller_ur->specify_qcd((double)UR_CYCLIC_TIME_MS * (0 + 1) / CONTROL_PERIOD_MS);
			cRun.wait(sl);
			icounter++;
			if (icounter >= itotaltime)
				icounter = itotaltime;
			for (int i = 0; i < 6; i++)
			{
				q_last_circle[i] = df_array_q_choosed_pos[i];
			}
		}
		b_running = false;
		set_pos.close();
		read_pos.close();
		total_point.close();
		act_xyz_pos.close();
		//i_input++;
	}
	delete[] q_pos_sequence;
	boost::posix_time::ptime stop_time = boost::get_system_time();
	controller_ur->stop(stop_time);
}