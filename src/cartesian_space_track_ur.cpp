// Copyright (c) 2021 Yipeng Li @ BICE (yipengli.bice@gmail.com)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#include<cartesian_space_track_ur.h>
#include<utils.h>
#include<iomanip>
#include<Eigen/Eigenvalues>
#include <Eigen/Dense>
#include<cmath>
#include <fstream>
#include <algorithm>

namespace Robot {

	CartesianSpaceTrackUR::CartesianSpaceTrackUR(ur_sdk::client& client_) : AbstractControllerUR(client_)
	{
		Kp = 1.0;//1.00  0.5
		Ki = 0.0001;//1.00
		Kd = 0.00;
		Spring_k = 0.01;
		useIBVS = true;
		//elapsed_time_pre = 0.0;
		//jKp[0] = 5;//0.7  //2.7   3.7   5(7s~7.1S)  8(7.4S会过冲)  7(7.56S会过冲)  6(7.5没明显过冲) 5.5(7.1~7.3S)
		for (int i = 0; i < 6; i++)
		{
			last_last_deviation[i] = 0;
			last_deviation[i] = 0;
			tatal_deviation[i] = 0;
			impedance_control_D[i] = 500.0;
			impedance_control_K[i] = 50.0;
			impedance_control_M[i] = 50.0;
			jLastErr[i] = 0.0;
			jKp[i] = 5;//0.7  //2.7   3.7   5(6.7s~7.1S)  8(7.4S会过冲)  7(7.56S会过冲)  6(7.5没明显过冲) 5.5(7.1~7.3S)
			jKd[i] = 0.3;//0.5 比没有时降了0.1S.
			q_maxspeed[i] = 0.4;
			impedance_control_max_pos_inc[i] = 0.15;
			impedance_control_pos_inc[i] = 0.0;
			impedance_control_acc[i] = 0.0;
			joint_admitance_swith[i] = false;
			joint_admitance_status[i] = 0;
		}
		for (int j = 0; j < 3; j++)
		{
			vel_calcu[j] = 0.0;
		}
		counter = 0;
		x_force_error_dot = 0.0;
		for (int m = 0; m < smooth_num; m++)
		{
			z_smooth_force[m] = 0.0;
			x_smooth_force[m] = 0.0;
			y_smooth_force[m] = 0.0;
		}
		
		df_rob_loop_time = 0.008;//
		/*d1 = 0.1273;
		L2 = 0.6127;
		L3 = 0.5716;
		d4 = -0.1639;
		d5 = 0.1157;
		d6 = 0.0819;*/
		//UR3 
		//d1 = 0.1519;
		//L2 = -0.24365;
		//L3 = -0.21325;
		//d4 = 0.11235;
		//d5 = 0.08535;
		//d6 = 0.0819;

		//UR10
		//d1 = 0.1273;
		//L2 = -0.612;
		//L3 = -0.5723;
		//d4 = 0.163941;
		//d5 = 0.1157;
		//d6 = 0.0922;

		//UR10e
		d1 = 0.1807;
		L2 = -0.6127;
		L3 = -0.57155;
		d4 = 0.17415;
		d5 = 0.11985;
		d6 = 0.11655;
		controller_type = CONTROLLER_TYPE::FULL_POSE;

		qcd.resize(6);
		qcd_pre.resize(6);
		q_pos_target.resize(6);
		tool_force_.resize(6);
		tool_coordinate_system_force_.resize(6);
		tcp_speed_.resize(6);
		pos_pre.resize(3);
		actual_current_.resize(6);
		control_current_.resize(6);
		q_position.resize(6);
		actual_current_temp_.resize(6);

		q_pos_target[0] = 0; q_pos_target[1] = 0; q_pos_target[2] = 0;
		q_pos_target[3] = 0; q_pos_target[4] = 0; q_pos_target[5] = 0;
		qcd[0] = 0; qcd[1] = 0; qcd[2] = 0;
		qcd[3] = 0; qcd[4] = 0; qcd[5] = 0;
		qcd_pre[0] = 0; qcd_pre[1] = 0; qcd_pre[2] = 0;
		qcd_pre[3] = 0; qcd_pre[4] = 0; qcd_pre[5] = 0;
		tool_force_preset_(0) = 0; tool_force_preset_(1) = 0; tool_force_preset_(2) = 0;
	}
	CartesianSpaceTrackUR::~CartesianSpaceTrackUR()
	{

	}



	void CartesianSpaceTrackUR::set_kinematic_param(double d1_, double L2_, double L3_, double d4_, double d5_, double d6_)
	{
		d1 = d1_;
		L2 = L2_;
		L3 = L3_;
		d4 = d4_;
		d5 = d5_;
		d6 = d6_;
	}
	void CartesianSpaceTrackUR::set_desired_position(const Eigen::Vector3d& x_, const Eigen::Vector3d& xd_, const Eigen::Vector3d& xdd_)
	{
		x_des = x_;
		xd_des = xd_;
		xdd_des = xdd_;
	}
	void CartesianSpaceTrackUR::set_desired_orientation(const Eigen::Matrix3d& R_, const Eigen::Vector3d& w_, const Eigen::Vector3d& wd_)
	{
		R_des = R_;
		w_des = w_;
		wd_des = wd_;
	}

	void CartesianSpaceTrackUR::set_Qdesired_position(double* qdes)
	{
		for (int i = 0; i < 6; i++)
		{
			Q_des[i]= qdes[i];
		}
	}
	void CartesianSpaceTrackUR::update(const boost::posix_time::ptime& time, const boost::posix_time::time_duration& period)
	{
		//std::cout << "flag" << std::endl;
		//std::ofstream tool_force_data("tool_force_data.txt", std::ios::trunc);//trunc  app
		elapsed_time = boost::get_system_time() - start_time;
		//elapsed_time = time - start_time;
		//inverse kinematic
		double vel_[3];
		double dt_time, xyz_force[3];
		client.force_control_ = force_controle_mode;
		// update the joint-space state
		std::vector<double> q_ = client.getPosition();
		std::vector<double> qd_ = client.getSpeed();
		std::vector<double> qc_ = client.getTargetPosition();
		std::vector<double> tool_force = client.getTCP_force();
		tool_force_ = tool_force;
		tool_coordinate_system_force_ = get_tcp_force_tool(tool_force_);
		tcp_speed_ = client.getTcpSpeed();
		dt_time = (double)elapsed_time.total_milliseconds() / 1000 - (double)elapsed_time_pre.total_milliseconds() / 1000;
		//std::cout << "dt_time:= " << dt_time << std::endl;
		for(int n=0; n < 3; n++)
		{
			if (dt_time < 0.001)
				vel_[n] = 0.0;
			else
				vel_[n] = (x(n) - pos_pre[n])/ dt_time;

			if (tool_force_[n] < 3.0 and tool_force_[n] > -5.0)
				xyz_force[n] = 0;
			else if (tool_force_[n] >= 3.0)
				xyz_force[n] = std::min(50.0, tool_force_[n]);
			else if (tool_force_[2] <= -5.0)
				xyz_force[n] = std::max(-50.0, tool_force_[n]);



			impedance_control_acc[n] = (xyz_force[n] + impedance_control_K[n] * (x_des(n) - x(n)) + impedance_control_D[n] * vel_[n]) / impedance_control_M[n];
			vel_calcu[n] = vel_calcu[n] + impedance_control_acc[n] * dt_time;
			impedance_control_pos_inc[n] = impedance_control_pos_inc[n] + vel_calcu[n] * dt_time;
			if (impedance_control_pos_inc[n] < -impedance_control_max_pos_inc[n])
				impedance_control_pos_inc[n] = -impedance_control_max_pos_inc[n];
			else if (impedance_control_pos_inc[n] > impedance_control_max_pos_inc[n])
				impedance_control_pos_inc[n] = impedance_control_max_pos_inc[n];
			else
			{
				impedance_control_pos_inc[n] = impedance_control_pos_inc[n];
			}
		}
		//std::cout << "vel_[2]:= " << vel_[2] << std::endl;
		//std::cout << "tool_force_[2]:= " << tool_force_[2] << std::endl;
		//std::cout << "xyz_force[2]:= " << xyz_force[2] << std::endl;
		//std::cout << "impedance_control_acc[2]:= " << impedance_control_acc[2] << std::endl;
		//std::cout << "vel_calcu[2]:= " << vel_calcu[2] << std::endl;
		//std::cout << "impedance_control_pos_inc[2]:= " << impedance_control_pos_inc[2] << std::endl;
		//std::cout << "x_des(2) - x(2):= " << x_des(2) - x(2) << std::endl;
		
		// transformation of the joint position to fit the self-defined cordinates
		/*
		q(0) = q_[0]; q(1) = -q_[1]; q(2) = -q_[2];
		q(3) = -q_[3] - M_PI / 2; q(4) = q_[4] - M_PI; q(5) = q_[5];

		q1 = q(0); q2 = q(1); q3 = q(2);
		q4 = q(3); q5 = q(4); q6 = q(5);

		qd(0) = qd_[0]; qd(1) = -qd_[1]; qd(2) = -qd_[2];
		qd(3) = qd_[3]; qd(4) = qd_[4]; qd(5) = qd[5];
		*/
		//读取到的关节位置
		q_position = q_;
		q1 = q_[0]; q2 = q_[1]; q3 = q_[2];
		q4 = q_[3]; q5 = q_[4]; q6 = q_[5];
		//读取到的关节速度
		qd(0) = qd_[0]; qd(1) = qd_[1]; qd(2) = qd_[2];
		qd(3) = qd_[3]; qd(4) = qd_[4]; qd(5) = qd[5];
		cal_pose();//正解,存放到R,x里,
		cal_J();//求雅可比矩阵J,直接存放在J矩阵里
		if (useIBVS)
		{
			Eigen::Vector3d p1, p2, p3,p4, p1_des, p2_des, p3_des, p4_des;
			p1 << im3point_act[0], im3point_act[1], im3point_act[2];
			p2 << im3point_act[3], im3point_act[4], im3point_act[5];
			p3 << im3point_act[6], im3point_act[7], im3point_act[8];
			p4 << im3point_act[9], im3point_act[10], im3point_act[11];

			p1_des << im3point_des[0], im3point_des[1], 0.5;
			p2_des << im3point_des[2], im3point_des[3], 0.5;
			p3_des << im3point_des[4], im3point_des[5], 0.5;
			p4_des << im3point_des[6], im3point_des[7], 0.5;
			//std::cout << "p1 " << p1.transpose() << std::endl;
			//std::cout << "p2 " << p2.transpose() << std::endl;
			//std::cout << "p3 " << p3.transpose() << std::endl;
			//std::cout << "p4 " << p4.transpose() << std::endl;
			image_J = cal_image_J(p1,p2,p3,p4);
			image_desire_J = cal_image_J(p1_des, p2_des, p3_des, p4_des);
			//std::cout <<"robot jacobian:"<< std::endl ;
			//std::cout << J << std::endl << std::endl;
			//std::cout << "j_camara:" << std::endl;
			//std::cout << J_camara << std::endl << std::endl;
			std::cout << "image J:" << std::endl;
			std::cout << image_J << std::endl;
			std::cout << "image_desire_J:" << std::endl;
			//std::cout << image_desire_J << std::endl;
		}
			
		Eigen::Matrix<double, 6, 1> tmp = J * qd;
		Eigen::Matrix<double, 6, 1> car_dx;
		//car_dx << 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001;
		//Eigen::Matrix<double, 6, 1> j_dx = J.inverse() * car_dx;
		//std::cout << "j_dx" << j_dx << std::endl;
		xd = tmp.segment<3>(0);//笛卡儿空间的速度
		w = tmp.segment<3>(3);//笛卡儿空间的姿态速度
		// store the previous qcd
		for (int i = 0; i < 6; i++)
		{
			qcd_pre[i] = qcd[i];
			//std::cout << "tool_force" << tool_force[i] << std::endl;
			//tool_force_data << tool_force[i] << "   ";
		}
		for (int m = 0; m < 3; m++)
		{
			pos_pre[m] = x(m);
		}

		//tool_force_data << (double)elapsed_time.total_milliseconds() / 1000 << "    ";
		//tool_force_data << impedance_control_pos_inc[2];
		//tool_force_data << vel_[2] << "   ";
		//tool_force_data << tool_force_[2] <<"   ";
		//tool_force_data << xyz_force[2] << "   ";
		//tool_force_data << impedance_control_acc[2] << "   ";
		//tool_force_data << vel_calcu[2] << "   ";
		//tool_force_data << impedance_control_pos_inc[2] << "   ";
		//tool_force_data << x_des(2) - x(2) << "   ";
		//tool_force_data << std::endl;

		elapsed_time_pre = elapsed_time;
		// cal new qcd
		if (useIBVS)
			kinematic_controller_IBVS();
		else
			 kinematic_controller_hw();
		//kinematic_controller();
		//tool_force_data.close();
	}

	void CartesianSpaceTrackUR::update_p(const boost::posix_time::ptime& time, const boost::posix_time::time_duration& period)
	{

		elapsed_time = time - start_time;
		//inverse kinematic

		// update the joint-space state
		//double q_current[6]; 
		std::vector<double> q_ ;
		q_.resize(6);
		q_ = client.getPosition();
		//actual_current_ = client.getActualCurrent();
		//control_current_ = client.getControlCurrent();

		q1 = q_[0]; q2 = q_[1]; q3 = q_[2];
		q4 = q_[3]; q5 = q_[4]; q6 = q_[5];
		cal_pose();//正解,存放到R,x里,
		//for (int mm = 0; mm < 6; mm++)
		//{
		//	q_current[mm] = q_[mm];
		//	//std::cout << "curren q:= " << q_current[mm];
		//}
		//std::cout << std ::endl;
		std::vector<double> qd_ = client.getSpeed();
		//读取到的关节速度
		qd(0) = qd_[0]; qd(1) = qd_[1]; qd(2) = qd_[2];
		qd(3) = qd_[3]; qd(4) = qd_[4]; qd(5) = qd_[5];
		// store the previous qcd
		for (int i = 0; i < 6; i++)
			qcd_pre[i] = qcd[i];
		// cal new qcd
		kinematic_jointPcontroller();
	}
	void CartesianSpaceTrackUR::specify_zero_qcd()
	{
		std::vector<double> qcd_cur, q_send;
		qcd_cur.resize(6);
		q_send.resize(6);
		for (int i = 0; i < 6; i++)
		{
			qcd_cur[i] = 0;
			q_send[i] = 0;
		}
		client.setSpeed(q_send, qcd_cur, qdd_target);//q_pos_target
	}

	void CartesianSpaceTrackUR::specify_qcd(double scale_)
	{
		std::vector<double> qcd_cur;
		std::vector<double> q_cur,q_send;
		qcd_cur.resize(6);
		q_cur.resize(6);
		q_send.resize(6);
		q_cur = client.getPosition();
		//std::cout << "current speed:= " << std:: endl;
		//std::cout << "send position:= " << std::endl;
		for (int i = 0; i < 6; i++)
		{
			if ((qcd_pre[i] + scale_ * (qcd[i] - qcd_pre[i])) < -q_maxspeed[i])
				qcd_cur[i] = -q_maxspeed[i];
			else if ((qcd_pre[i] + scale_ * (qcd[i] - qcd_pre[i])) > q_maxspeed[i])
				qcd_cur[i] = q_maxspeed[i];
			else
			{
				qcd_cur[i] = qcd_pre[i] + scale_ * (qcd[i] - qcd_pre[i]);
			}
			q_send[i] = q_position[i] + scale_ * (q_pos_target[i] - q_position[i]);
			//std::cout <<std::setw(12) << qcd[i];
			//std::cout << std::setw(12) << q_send[i];
		}
		//client.setSpeed(qcd_cur, qdd_target);
		//std::cout << std::endl;
		client.setSpeed(q_send, qcd_cur, qdd_target);//q_pos_target
	}
	void CartesianSpaceTrackUR::cal_J()
	{
		/*  Li yi peng early
		J.block<1, 6>(0, 0) << L3 * cos(q1) * sin(q2) * sin(q3) - L2 * cos(q1) * cos(q2) - d6 * cos(q5) * sin(q1) - L3 * cos(q1) * cos(q2) * cos(q3) - d4 * sin(q1) - d5 * cos(q1) * cos(q2) * cos(q3) * cos(q4) + d5 * cos(q1) * cos(q2) * sin(q3) * sin(q4) + d5 * cos(q1) * cos(q3) * sin(q2) * sin(q4) + d5 * cos(q1) * cos(q4) * sin(q2) * sin(q3) - d6 * cos(q1) * cos(q2) * cos(q3) * sin(q4) * sin(q5) - d6 * cos(q1) * cos(q2) * cos(q4) * sin(q3) * sin(q5) - d6 * cos(q1) * cos(q3) * cos(q4) * sin(q2) * sin(q5) + d6 * cos(q1) * sin(q2) * sin(q3) * sin(q4) * sin(q5), sin(q1)* (L3 * sin(q2 + q3) - (d6 * sin(q2 + q3 + q4 + q5)) / 2 + L2 * sin(q2) + (d6 * sin(q2 + q3 + q4 - q5)) / 2 + d5 * sin(q2 + q3 + q4)), sin(q1)* (L3 * sin(q2 + q3) - (d6 * sin(q2 + q3 + q4 + q5)) / 2 + (d6 * sin(q2 + q3 + q4 - q5)) / 2 + d5 * sin(q2 + q3 + q4)), sin(q1)* ((d6 * sin(q2 + q3 + q4 - q5)) / 2 - (d6 * sin(q2 + q3 + q4 + q5)) / 2 + d5 * sin(q2 + q3 + q4)), d6* cos(q5)* sin(q1)* sin(q2)* sin(q3)* sin(q4) - d6 * cos(q2) * cos(q3) * cos(q5) * sin(q1) * sin(q4) - d6 * cos(q2) * cos(q4) * cos(q5) * sin(q1) * sin(q3) - d6 * cos(q3) * cos(q4) * cos(q5) * sin(q1) * sin(q2) - d6 * cos(q1) * sin(q5), 0;
		J.block<1, 6>(1, 0) << d4 * cos(q1) - L2 * cos(q2) * sin(q1) + d6 * cos(q1) * cos(q5) - L3 * cos(q2) * cos(q3) * sin(q1) + L3 * sin(q1) * sin(q2) * sin(q3) - d5 * cos(q2) * cos(q3) * cos(q4) * sin(q1) + d5 * cos(q2) * sin(q1) * sin(q3) * sin(q4) + d5 * cos(q3) * sin(q1) * sin(q2) * sin(q4) + d5 * cos(q4) * sin(q1) * sin(q2) * sin(q3) - d6 * cos(q2) * cos(q3) * sin(q1) * sin(q4) * sin(q5) - d6 * cos(q2) * cos(q4) * sin(q1) * sin(q3) * sin(q5) - d6 * cos(q3) * cos(q4) * sin(q1) * sin(q2) * sin(q5) + d6 * sin(q1) * sin(q2) * sin(q3) * sin(q4) * sin(q5), -cos(q1) * (L3 * sin(q2 + q3) - (d6 * sin(q2 + q3 + q4 + q5)) / 2 + L2 * sin(q2) + (d6 * sin(q2 + q3 + q4 - q5)) / 2 + d5 * sin(q2 + q3 + q4)), -cos(q1) * (L3 * sin(q2 + q3) - (d6 * sin(q2 + q3 + q4 + q5)) / 2 + (d6 * sin(q2 + q3 + q4 - q5)) / 2 + d5 * sin(q2 + q3 + q4)), -cos(q1) * ((d6 * sin(q2 + q3 + q4 - q5)) / 2 - (d6 * sin(q2 + q3 + q4 + q5)) / 2 + d5 * sin(q2 + q3 + q4)), d6* cos(q1)* cos(q2)* cos(q3)* cos(q5)* sin(q4) - d6 * cos(q1) * cos(q5) * sin(q2) * sin(q3) * sin(q4) - d6 * sin(q1) * sin(q5) + d6 * cos(q1) * cos(q2) * cos(q4) * cos(q5) * sin(q3) + d6 * cos(q1) * cos(q3) * cos(q4) * cos(q5) * sin(q2), 0;
		J.block<1, 6>(2, 0) << 0, L3 * cos(q2 + q3) - (d6 * cos(q2 + q3 + q4 + q5)) / 2 + L2 * cos(q2) + (d6 * cos(q2 + q3 + q4 - q5)) / 2 + d5 * cos(q2 + q3 + q4), L3 * cos(q2 + q3) - (d6 * cos(q2 + q3 + q4 + q5)) / 2 + (d6 * cos(q2 + q3 + q4 - q5)) / 2 + d5 * cos(q2 + q3 + q4), (d6 * cos(q2 + q3 + q4 - q5)) / 2 - (d6 * cos(q2 + q3 + q4 + q5)) / 2 + d5 * cos(q2 + q3 + q4), -(d6 * (cos(q2 + q3 + q4 + q5) + cos(q2 + q3 + q4 - q5))) / 2, 0;

		J.block<1, 6>(3, 0) << sin(q2 + q3 + q4 + q6) / 2 + sin(q2 + q3 + q4 - q6) / 2 + cos(q2 + q3 + q4) * cos(q5) * sin(q6), sin(q5) * sin(q6), sin(q5) * sin(q6), sin(q5)* sin(q6), cos(q6), 0;
		J.block<1, 6>(4, 0) << cos(q2 + q3 + q4 + q6) / 2 - cos(q2 + q3 + q4 - q6) / 2 + cos(q2 + q3 + q4) * cos(q5) * cos(q6), cos(q6) * sin(q5), cos(q6) * sin(q5), cos(q6)* sin(q5), -sin(q6), 0;
		J.block<1, 6>(5, 0) << sin(q2 + q3 + q4 - q5) / 2 - sin(q2 + q3 + q4 + q5) / 2, cos(q5), cos(q5), cos(q5), 0, 1;
		*/
		/* simplify*/
		J.block<1, 6>(0, 0) << d6*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)) + d4*cos(q1) - L3*cos(q2 + q3)*sin(q1) - L2*cos(q2)*sin(q1) - d5*sin(q2 + q3 + q4)*sin(q1), -cos(q1)*(L3*sin(q2 + q3) + L2*sin(q2) - d5*cos(q2 + q3 + q4) - d6*sin(q2 + q3 + q4)*sin(q5)), cos(q1)*(d5*cos(q2 + q3 + q4) - L3*sin(q2 + q3) + d6*sin(q2 + q3 + q4)*sin(q5)), cos(q1)*(d5*cos(q2 + q3 + q4) + d6*sin(q2 + q3 + q4)*sin(q5)), -d6*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)), 0;
		J.block<1, 6>(1, 0) << d6*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)) + d4*sin(q1) + L3*cos(q2 + q3)*cos(q1) + L2*cos(q1)*cos(q2) + d5*sin(q2 + q3 + q4)*cos(q1), -sin(q1)*(L3*sin(q2 + q3) + L2*sin(q2) - d5*cos(q2 + q3 + q4) - d6*sin(q2 + q3 + q4)*sin(q5)), sin(q1)*(d5*cos(q2 + q3 + q4) - L3*sin(q2 + q3) + d6*sin(q2 + q3 + q4)*sin(q5)), sin(q1)*(d5*cos(q2 + q3 + q4) + d6*sin(q2 + q3 + q4)*sin(q5)), d6*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)), 0;
		J.block<1, 6>(2, 0) << 0, L3*cos(q2 + q3) + L2*cos(q2) + d5*(cos(q2 + q3)*sin(q4) + sin(q2 + q3)*cos(q4)) - d6*sin(q5)*(cos(q2 + q3)*cos(q4) - sin(q2 + q3)*sin(q4)), L3*cos(q2 + q3) + d5*sin(q2 + q3 + q4) - d6*cos(q2 + q3 + q4)*sin(q5), d5*sin(q2 + q3 + q4) - d6*cos(q2 + q3 + q4)*sin(q5), -d6*sin(q2 + q3 + q4)*cos(q5), 0;
		J.block<1, 6>(3, 0) << 0, sin(q1), sin(q1), sin(q1), sin(q2 + q3 + q4)*cos(q1), cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5);
		J.block<1, 6>(4, 0) << 0, -cos(q1), -cos(q1), -cos(q1), sin(q2 + q3 + q4) * sin(q1), -cos(q1) * cos(q5) - cos(q2 + q3 + q4) * sin(q1) * sin(q5);
		J.block<1, 6>(5, 0) << 1, 0, 0, 0, -cos(q2 + q3 + q4), -sin(q2 + q3 + q4)*sin(q5);
		//std::cout << J1 << std::endl;
		
		//J_camara.block<1, 6>(0, 0) << L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) - (253 * cos(q1) * cos(q5)) / 10000 - d5 * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1))) - (23 * cos(q6) * (cos(q1) * sin(q5) + cos(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))))) / 625 + (757 * sin(q6) * (cos(q1) * sin(q5) + cos(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))))) / 10000 + d4 * cos(q1) - (757 * cos(q6) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 10000 + (253 * sin(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)))) / 10000 - (23 * sin(q6) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 625 + d6 * (cos(q1) * cos(q5) - sin(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)))) - L2 * cos(q2) * sin(q1), d5* (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))) + (757 * cos(q6) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)))) / 10000 - (253 * sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 10000 + (23 * sin(q6) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)))) / 625 - L3 * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + (23 * cos(q5) * cos(q6) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 625 - (757 * cos(q5) * sin(q6) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 10000 - L2 * cos(q1) * sin(q2) + d6 * sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3))), d5* (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))) + (757 * cos(q6) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)))) / 10000 - (253 * sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 10000 + (23 * sin(q6) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)))) / 625 - L3 * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + (23 * cos(q5) * cos(q6) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 625 - (757 * cos(q5) * sin(q6) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 10000 + d6 * sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3))), d5* (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))) + (757 * cos(q6) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)))) / 10000 - (253 * sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 10000 + (23 * sin(q6) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)))) / 625 + (23 * cos(q5) * cos(q6) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 625 - (757 * cos(q5) * sin(q6) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 10000 + d6 * sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3))), (253 * sin(q1) * sin(q5)) / 10000 - (23 * cos(q6) * (cos(q5) * sin(q1) - sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))))) / 625 + (757 * sin(q6) * (cos(q5) * sin(q1) - sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))))) / 10000 + (253 * cos(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)))) / 10000 - d6 * (sin(q1) * sin(q5) + cos(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)))), (757 * cos(q6) * (sin(q1) * sin(q5) + cos(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))))) / 10000 + (23 * sin(q6) * (sin(q1) * sin(q5) + cos(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))))) / 625 + (23 * cos(q6) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 625 - (757 * sin(q6) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 10000;
		//J_camara.block<1, 6>(1, 0) << d5 * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3))) - (253 * cos(q5) * sin(q1)) / 10000 - (23 * cos(q6) * (sin(q1) * sin(q5) + cos(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))))) / 625 + (757 * sin(q6) * (sin(q1) * sin(q5) + cos(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))))) / 10000 + (757 * cos(q6) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 10000 + (253 * sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)))) / 10000 + (23 * sin(q6) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 625 + d4 * sin(q1) + d6 * (cos(q5) * sin(q1) - sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)))) + L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) + L2 * cos(q1) * cos(q2), d6* sin(q5)* (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1))) - d5 * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))) - (757 * cos(q6) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)))) / 10000 - (253 * sin(q5) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 10000 - (23 * sin(q6) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)))) / 625 - L3 * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - L2 * sin(q1) * sin(q2) + (23 * cos(q5) * cos(q6) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 625 - (757 * cos(q5) * sin(q6) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 10000, d6* sin(q5)* (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1))) - d5 * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))) - (757 * cos(q6) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)))) / 10000 - (253 * sin(q5) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 10000 - (23 * sin(q6) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)))) / 625 - L3 * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) + (23 * cos(q5) * cos(q6) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 625 - (757 * cos(q5) * sin(q6) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 10000, d6* sin(q5)* (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1))) - (757 * cos(q6) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)))) / 10000 - (253 * sin(q5) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 10000 - (23 * sin(q6) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)))) / 625 - d5 * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))) + (23 * cos(q5) * cos(q6) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 625 - (757 * cos(q5) * sin(q6) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 10000, (23 * cos(q6) * (cos(q1) * cos(q5) - sin(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))))) / 625 - (253 * cos(q1) * sin(q5)) / 10000 - (757 * sin(q6) * (cos(q1) * cos(q5) - sin(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))))) / 10000 - (253 * cos(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)))) / 10000 + d6 * (cos(q1) * sin(q5) + cos(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)))), (23 * cos(q6) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 625 - (23 * sin(q6) * (cos(q1) * sin(q5) + cos(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))))) / 625 - (757 * cos(q6) * (cos(q1) * sin(q5) + cos(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))))) / 10000 - (757 * sin(q6) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 10000;
		//J_camara.block<1, 6>(2, 0) << 0, (757 * cos(q6) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)))) / 10000 + (253 * sin(q5) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 10000 + (23 * sin(q6) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)))) / 625 + L3 * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) + L2 * cos(q2) + d5 * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3))) + (757 * cos(q5) * sin(q6) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 10000 - d6 * sin(q5) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2))) - (23 * cos(q5) * cos(q6) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 625, (757 * cos(q6) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)))) / 10000 + (253 * sin(q5) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 10000 + (23 * sin(q6) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)))) / 625 + L3 * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) + d5 * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3))) + (757 * cos(q5) * sin(q6) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 10000 - d6 * sin(q5) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2))) - (23 * cos(q5) * cos(q6) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 625, (757 * cos(q6) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)))) / 10000 + (253 * sin(q5) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 10000 + (23 * sin(q6) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)))) / 625 + d5 * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3))) + (757 * cos(q5) * sin(q6) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 10000 - d6 * sin(q5) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2))) - (23 * cos(q5) * cos(q6) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 625, (253 * cos(q5) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)))) / 10000 + (23 * cos(q6) * sin(q5) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)))) / 625 - (757 * sin(q5) * sin(q6) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)))) / 10000 - d6 * cos(q5) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3))), (757 * sin(q6) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 10000 - (23 * cos(q6) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 625 + (23 * cos(q5) * sin(q6) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)))) / 625 + (757 * cos(q5) * cos(q6) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)))) / 10000;
		//J_camara.block<1, 6>(3, 0) << 0, sin(q1), sin(q1), sin(q1), sin(q2 + q3 + q4)* cos(q1), cos(q5)* sin(q1) - cos(q2 + q3 + q4) * cos(q1) * sin(q5);
		//J_camara.block<1, 6>(4, 0) << 0, -cos(q1), -cos(q1), -cos(q1), sin(q2 + q3 + q4)* sin(q1), -cos(q1) * cos(q5) - cos(q2 + q3 + q4) * sin(q1) * sin(q5);
		//J_camara.block<1, 6>(5, 0) << 1, 0, 0, 0, -cos(q2 + q3 + q4), -sin(q2 + q3 + q4) * sin(q5);
		//J_camara.block<1, 6>(3, 0) << cos(q1) * cos(q5) - (2 * cos(q6) * (cos(q1) * sin(q5) + cos(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))))) / 625 + (49 * sin(q6) * (cos(q1) * sin(q5) + cos(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))))) / 10000 - (49 * cos(q6) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 10000 - sin(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))) - (2 * sin(q6) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 625, (49 * cos(q6) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)))) / 10000 + sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3))) + (2 * sin(q6) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)))) / 625 + (2 * cos(q5) * cos(q6) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 625 - (49 * cos(q5) * sin(q6) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 10000, (49 * cos(q6) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)))) / 10000 + sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3))) + (2 * sin(q6) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)))) / 625 + (2 * cos(q5) * cos(q6) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 625 - (49 * cos(q5) * sin(q6) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 10000, (49 * cos(q6) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)))) / 10000 + sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3))) + (2 * sin(q6) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)))) / 625 + (2 * cos(q5) * cos(q6) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 625 - (49 * cos(q5) * sin(q6) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 10000, (49 * sin(q6) * (cos(q5) * sin(q1) - sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))))) / 10000 - (2 * cos(q6) * (cos(q5) * sin(q1) - sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))))) / 625 - sin(q1) * sin(q5) - cos(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))), (49 * cos(q6) * (sin(q1) * sin(q5) + cos(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))))) / 10000 + (2 * sin(q6) * (sin(q1) * sin(q5) + cos(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))))) / 625 + (2 * cos(q6) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 625 - (49 * sin(q6) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 10000;
		//J_camara.block<1, 6>(4, 0) << cos(q5) * sin(q1) - (2 * cos(q6) * (sin(q1) * sin(q5) + cos(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))))) / 625 + (49 * sin(q6) * (sin(q1) * sin(q5) + cos(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))))) / 10000 + (49 * cos(q6) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 10000 - sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))) + (2 * sin(q6) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)))) / 625, sin(q5)* (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1))) - (49 * cos(q6) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)))) / 10000 - (2 * sin(q6) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)))) / 625 + (2 * cos(q5) * cos(q6) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 625 - (49 * cos(q5) * sin(q6) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 10000, sin(q5)* (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1))) - (49 * cos(q6) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)))) / 10000 - (2 * sin(q6) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)))) / 625 + (2 * cos(q5) * cos(q6) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 625 - (49 * cos(q5) * sin(q6) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 10000, sin(q5)* (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1))) - (49 * cos(q6) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)))) / 10000 - (2 * sin(q6) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)))) / 625 + (2 * cos(q5) * cos(q6) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 625 - (49 * cos(q5) * sin(q6) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 10000, cos(q1)* sin(q5) + (2 * cos(q6) * (cos(q1) * cos(q5) - sin(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))))) / 625 - (49 * sin(q6) * (cos(q1) * cos(q5) - sin(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))))) / 10000 + cos(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))), (2 * cos(q6) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 625 - (2 * sin(q6) * (cos(q1) * sin(q5) + cos(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))))) / 625 - (49 * cos(q6) * (cos(q1) * sin(q5) + cos(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))))) / 10000 - (49 * sin(q6) * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)))) / 10000;
		//J_camara.block<1, 6>(5, 0) << 0, (49 * cos(q6) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)))) / 10000 - sin(q5) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2))) + (2 * sin(q6) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)))) / 625 + (49 * cos(q5) * sin(q6) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 10000 - (2 * cos(q5) * cos(q6) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 625, (49 * cos(q6) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)))) / 10000 - sin(q5) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2))) + (2 * sin(q6) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)))) / 625 + (49 * cos(q5) * sin(q6) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 10000 - (2 * cos(q5) * cos(q6) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 625, (49 * cos(q6) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)))) / 10000 - sin(q5) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2))) + (2 * sin(q6) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)))) / 625 + (49 * cos(q5) * sin(q6) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 10000 - (2 * cos(q5) * cos(q6) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 625, (2 * cos(q6) * sin(q5) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)))) / 625 - cos(q5) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3))) - (49 * sin(q5) * sin(q6) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)))) / 10000, (49 * sin(q6) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 10000 - (2 * cos(q6) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)))) / 625 + (2 * cos(q5) * sin(q6) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)))) / 625 + (49 * cos(q5) * cos(q6) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)))) / 10000;

		//hw calculate 
		//J.block<1, 6>(0, 0) << L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) - d5 * (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1))) + d4 * cos(q1) + d6 * (cos(q1) * cos(q5) - sin(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)))) - L2 * cos(q2) * sin(q1), d5* (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))) - L3 * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) - L2 * cos(q1) * sin(q2) + d6 * sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3))), d5* (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))) - L3 * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + d6 * sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3))), d5* (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2))) + d6 * sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3))), -d6 * (sin(q1) * sin(q5) + cos(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)))), 0;
		//J.block<1, 6>(1, 0) << d5 * (cos(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3))) + d4 * sin(q1) + d6 * (cos(q5) * sin(q1) - sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)))) + L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) + L2 * cos(q1) * cos(q2), d6* sin(q5)* (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1))) - d5 * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))) - L3 * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - L2 * sin(q1) * sin(q2), d6* sin(q5)* (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1))) - d5 * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))) - L3 * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)), d6* sin(q5)* (cos(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1))) - d5 * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))), d6* (cos(q1) * sin(q5) + cos(q5) * (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)))), 0;
		//J.block<1, 6>(2, 0) << 0, L3* (cos(q2) * cos(q3) - sin(q2) * sin(q3)) + L2 * cos(q2) + d5 * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3))) - d6 * sin(q5) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2))), L3* (cos(q2) * cos(q3) - sin(q2) * sin(q3)) + d5 * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3))) - d6 * sin(q5) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2))), d5* (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3))) - d6 * sin(q5) * (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2))), -d6 * cos(q5) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3))), 0;
		//J.block<1, 6>(3, 0) << 0, sin(q1), sin(q1), sin(q1), cos(q4)* (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)) + sin(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)), cos(q5)* sin(q1) - sin(q5) * (cos(q4) * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3)) - sin(q4) * (cos(q1) * cos(q2) * sin(q3) + cos(q1) * cos(q3) * sin(q2)));
		//J.block<1, 6>(4, 0) << 0, -cos(q1), -cos(q1), -cos(q1), cos(q4)* (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2)) - sin(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)), sin(q5)* (cos(q4) * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1)) + sin(q4) * (cos(q2) * sin(q1) * sin(q3) + cos(q3) * sin(q1) * sin(q2))) - cos(q1) * cos(q5);
		//J.block<1, 6>(5, 0) << 1, 0, 0, 0, sin(q4)* (cos(q2) * sin(q3) + cos(q3) * sin(q2)) - cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)), -sin(q5) * (cos(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) + sin(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)));
		//std::cout << J << std::endl;

	}
	Eigen::Matrix<double, 8, 6> CartesianSpaceTrackUR::cal_image_J(Eigen::Vector3d point1, Eigen::Vector3d point2, Eigen::Vector3d point3, Eigen::Vector3d point4)
	{
		Eigen::Matrix<double, 8, 6> image_J_temp;

		double f_hat ;
		//double f_skim ;//f/r
		Eigen::Matrix3d inParaM, inParaM_inverse;
		Eigen::Vector3d Image_base_p1, Image_base_p2, Image_base_p3, Image_base_p4;
		inParaM.block<1, 3>(0, 0) << 888.8485, 0.000000, 648.735;
		inParaM.block<1, 3>(1, 0) << 0.000000, 888.8485, 366.436;
		inParaM.block<1, 3>(2, 0) << 0.000000, 0.000000, 1.000000;
		double p_z_lenth[4];
		p_z_lenth[0] = point1[2]; p_z_lenth[1] = point2[2]; p_z_lenth[2] = point3[2]; p_z_lenth[3] = point4[2];
		//point1[2] = 1; point2[2] = 1; point3[2] = 1; point4[2] = 1;
		inParaM_inverse = inParaM.inverse();
		//转换到相机坐标系下
		//Image_base_p1 = p_z_lenth[0]* inParaM_inverse * point1;
		//Image_base_p2 = p_z_lenth[1]* inParaM_inverse * point2;
		//Image_base_p3 = p_z_lenth[2]* inParaM_inverse * point3;
		//Image_base_p4 = p_z_lenth[3]* inParaM_inverse * point4;
		
		Image_base_p1 = point1;
		Image_base_p2 = point2;
		Image_base_p3 = point3;
		Image_base_p4 = point4;

		//图像坐标系
		f_hat = inParaM(0, 0);
		image_J_temp.block<1, 6>(0, 0) << -1 * f_hat / Image_base_p1(2), 0, (point1(0) - inParaM(0, 2)) / Image_base_p1(2), (point1(0) - inParaM(0, 2))* (point1(1) - inParaM(1, 2)) / f_hat, -(1 * f_hat + (point1(0) - inParaM(0, 2)) * (point1(0) - inParaM(0, 2)) / f_hat), (point1(1) - inParaM(1, 2));
		image_J_temp.block<1, 6>(1, 0) << 0, -1 * f_hat / Image_base_p1(2), (point1(1) - inParaM(1, 2)) / Image_base_p1(2), (1 * f_hat + (point1(1) - inParaM(1, 2)) * (point1(1) - inParaM(1, 2)) / f_hat), -(point1(0) - inParaM(0, 2)) * (point1(1) - inParaM(1, 2)) / f_hat, -(point1(0) - inParaM(0, 2));
		image_J_temp.block<1, 6>(2, 0) << -1 * f_hat / Image_base_p2(2), 0, (point2(0) - inParaM(0, 2)) / Image_base_p2(2), (point2(0) - inParaM(0, 2))* (point2(1) - inParaM(1, 2)) / f_hat, -(1 * f_hat + (point2(0) - inParaM(0, 2)) * (point2(0) - inParaM(0, 2)) / f_hat), (point2(1) - inParaM(1, 2));
		image_J_temp.block<1, 6>(3, 0) << 0, -1 * f_hat / Image_base_p2(2), (point2(1) - inParaM(1, 2)) / Image_base_p2(2), (1 * f_hat + (point2(1) - inParaM(1, 2)) * (point2(1) - inParaM(1, 2)) / f_hat), -(point2(0) - inParaM(0, 2)) * (point2(1) - inParaM(1, 2)) / f_hat, -(point2(0) - inParaM(0, 2));
		image_J_temp.block<1, 6>(4, 0) << -1 * f_hat / Image_base_p3(2), 0, (point3(0) - inParaM(0, 2)) / Image_base_p3(2), (point3(0) - inParaM(0, 2))* (point3(1) - inParaM(1, 2)) / f_hat, -(1 * f_hat + (point3(0) - inParaM(0, 2)) * (point3(0) - inParaM(0, 2)) / f_hat), (point3(1) - inParaM(1, 2));
		image_J_temp.block<1, 6>(5, 0) << 0, -1 * f_hat / Image_base_p3(2), (point3(1) - inParaM(1, 2)) / Image_base_p3(2), (1 * f_hat + (point3(1) - inParaM(1, 2)) * (point3(1) - inParaM(1, 2)) / f_hat), -(point3(0) - inParaM(0, 2)) * (point3(1) - inParaM(1, 2)) / f_hat, -(point3(0) - inParaM(0, 2));
		image_J_temp.block<1, 6>(6, 0) << -1 * f_hat / Image_base_p4(2), 0, (point4(0) - inParaM(0, 2)) / Image_base_p4(2), (point4(0) - inParaM(0, 2))* (point4(1) - inParaM(1, 2)) / f_hat, -(1 * f_hat + (point4(0) - inParaM(0, 2)) * (point4(0) - inParaM(0, 2)) / f_hat), (point4(1) - inParaM(1, 2));
		image_J_temp.block<1, 6>(7, 0) << 0, -1 * f_hat / Image_base_p4(2), (point4(1) - inParaM(1, 2)) / Image_base_p4(2), (1 * f_hat + (point4(1) - inParaM(1, 2)) * (point4(1) - inParaM(1, 2)) / f_hat), -(point4(0) - inParaM(0, 2)) * (point4(1) - inParaM(1, 2)) / f_hat, -(point4(0) - inParaM(0, 2));
		//image_J.block<1, 6>(0, 0) << -1* f_hat / Image_base_p1(2), 0, Image_base_p1(0) / Image_base_p1(2), Image_base_p1(0)* Image_base_p1(1)/ f_hat, -(1* f_hat + Image_base_p1(0) * Image_base_p1(0)/ f_hat), Image_base_p1(1);
		//image_J.block<1, 6>(1, 0) << 0, -1* f_hat / Image_base_p1(2), Image_base_p1(1) / Image_base_p1(2), (1* f_hat + Image_base_p1(1) * Image_base_p1(1)/ f_hat), -Image_base_p1(0) * Image_base_p1(1)/ f_hat, -Image_base_p1(0);
		//image_J.block<1, 6>(2, 0) << -1* f_hat / Image_base_p2(2), 0, Image_base_p2(0) / Image_base_p2(2), Image_base_p2(0)* Image_base_p2(1)/ f_hat, -(1* f_hat + Image_base_p2(0) * Image_base_p2(0)/ f_hat), Image_base_p2(1);
		//image_J.block<1, 6>(3, 0) << 0, -1* f_hat / Image_base_p2(2), Image_base_p2(1) / Image_base_p2(2), (1* f_hat + Image_base_p2(1) * Image_base_p2(1)/ f_hat), -Image_base_p2(0) * Image_base_p2(1)/ f_hat, -Image_base_p2(0);
		//image_J.block<1, 6>(4, 0) << -1* f_hat / Image_base_p3(2), 0, Image_base_p3(0) / Image_base_p3(2), Image_base_p3(0)* Image_base_p3(1)/ f_hat, -(1* f_hat + Image_base_p3(0) * Image_base_p3(0)/ f_hat), Image_base_p3(1);
		//image_J.block<1, 6>(5, 0) << 0, -1* f_hat / Image_base_p3(2), Image_base_p3(1) / Image_base_p3(2), (1* f_hat + Image_base_p3(1) * Image_base_p3(1)/ f_hat), -Image_base_p3(0)* Image_base_p3(1)/ f_hat,  -Image_base_p3(0);
		//image_J.block<1, 6>(6, 0) << -1 * f_hat / Image_base_p4(2), 0, Image_base_p4(0) / Image_base_p4(2), Image_base_p4(0)* Image_base_p4(1) / f_hat, -(1 * f_hat + Image_base_p4(0) * Image_base_p4(0) / f_hat), Image_base_p4(1);
		//image_J.block<1, 6>(7, 0) << 0, -1 * f_hat / Image_base_p4(2), Image_base_p4(1) / Image_base_p4(2), (1 * f_hat + Image_base_p4(1) * Image_base_p4(1) / f_hat), -Image_base_p4(0) * Image_base_p4(1) / f_hat, -Image_base_p4(0);
		
		//相机坐标系点
		//f_hat = 1;// inParaM(0, 0);
		//image_J_temp.block<1, 6>(0, 0) << -1/ Image_base_p1(2), 0,  Image_base_p1(0) / Image_base_p1(2), Image_base_p1(0) *Image_base_p1(1), -(1  + Image_base_p1(0) * Image_base_p1(0)), Image_base_p1(1);
		//image_J_temp.block<1, 6>(1, 0) <<0, -1 / Image_base_p1(2),  Image_base_p1(1) / Image_base_p1(2), (1 + Image_base_p1(1) * Image_base_p1(1)), -Image_base_p1(0)* Image_base_p1(1), -Image_base_p1(0);
		//image_J_temp.block<1, 6>(2, 0) << -1 / Image_base_p2(2), 0, Image_base_p2(0) / Image_base_p2(2), Image_base_p2(0)* Image_base_p2(1), -(1 + Image_base_p2(0) * Image_base_p2(0)), Image_base_p2(1);
		//image_J_temp.block<1, 6>(3, 0) << 0, -1 / Image_base_p2(2), Image_base_p2(1) / Image_base_p2(2), (1 + Image_base_p2(1) * Image_base_p2(1)), -Image_base_p2(0) * Image_base_p2(1), -Image_base_p2(0);
		//image_J_temp.block<1, 6>(4, 0) << -1 / Image_base_p3(2), 0, Image_base_p3(0) / Image_base_p3(2), Image_base_p3(0)* Image_base_p3(1), -(1 + Image_base_p3(0) * Image_base_p3(0)), Image_base_p3(1);
		//image_J_temp.block<1, 6>(5, 0) << 0, -1 / Image_base_p3(2), Image_base_p3(1) / Image_base_p3(2), (1 + Image_base_p3(1) * Image_base_p3(1)), -Image_base_p3(0) * Image_base_p3(1), -Image_base_p3(0);
		//image_J_temp.block<1, 6>(6, 0) << -1 / Image_base_p4(2), 0, Image_base_p4(0) / Image_base_p4(2), Image_base_p4(0)* Image_base_p4(1), -(1 + Image_base_p4(0) * Image_base_p4(0)), Image_base_p4(1);
		//image_J_temp.block<1, 6>(7, 0) << 0, -1 / Image_base_p4(2), Image_base_p4(1) / Image_base_p4(2), (1 + Image_base_p4(1) * Image_base_p4(1)), -Image_base_p4(0) * Image_base_p4(1), -Image_base_p4(0);

		//return image_J_temp / f_hat;
		return image_J_temp;
	}
	void CartesianSpaceTrackUR::cal_pose()
	{
		Eigen::Matrix<double, 4, 4> T1, T2, T3, T4, T5, T6, T;
		Eigen::Matrix3d rot_cur;
		Eigen::Vector3d trans_cur;
		/*
		T1 << cos(q1 + M_PI / 2), 0, sin(q1 + M_PI / 2), 0,
			sin(q1 + M_PI / 2), 0, -cos(q1 + M_PI / 2), 0,
			0, 1, 0, d1,
			0, 0, 0, 1;
		T2 << cos(q2), -sin(q2), 0, L2* cos(q2),
			sin(q2), cos(q2), 0, L2* sin(q2),
			0, 0, 1, 0,
			0, 0, 0, 1;
		T3 << cos(q3), -sin(q3), 0, L3* cos(q3),
			sin(q3), cos(q3), 0, L3* sin(q3),
			0, 0, 1, 0,
			0, 0, 0, 1;
		T4 << cos(q4 + M_PI / 2), 0, sin(q4 + M_PI / 2), 0,
			sin(q4 + M_PI / 2), 0, -cos(q4 + M_PI / 2), 0,
			0, 1, 0, d4,
			0, 0, 0, 1;
		T5 << 0, -sin(q5 + M_PI / 2), cos(q5 + M_PI / 2), 0,
			0, cos(q5 + M_PI / 2), sin(q5 + M_PI / 2), 0,
			-1, 0, 0, d5,
			0, 0, 0, 1;
		T6 << -cos(q6), sin(q6), 0, 0,
			-sin(q6), -cos(q6), 0, 0,
			0, 0, 1, d6,
			0, 0, 0, 1;
		*/

		T1 << cos(q1), -sin(q1), 0, 0,
			sin(q1), cos(q1), 0, 0,
			0, 0, 1, d1,
			0, 0, 0, 1;
		T2 << cos(q2), -sin(q2), 0, 0,
			0, 0, -1, 0,
			sin(q2), cos(q2), 0, 0,
			0, 0, 0, 1;
		T3 << cos(q3), -sin(q3), 0, L2,
			sin(q3), cos(q3), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
		T4 << cos(q4), -sin(q4), 0, L3,
			sin(q4), cos(q4), 0, 0,
			0, 0, 1, d4,
			0, 0, 0, 1;
		T5 << cos(q5), -sin(q5), 0, 0,
			0, 0, -1, -d5,
			sin(q5), cos(q5), 0, 0,
			0, 0, 0, 1;
		T6 << cos(q6), -sin(q6), 0, 0,
			0, 0, 1, d6,
			-sin(q6), -cos(q6), 0, 0,
			0, 0, 0, 1;

		T = T1 * T2 * T3 * T4 * T5 * T6;
		Ttool_to_base = T;
		R = T.block<3, 3>(0, 0);
		x = T.block<3, 1>(0, 3);
		
	}
	void CartesianSpaceTrackUR::fkRobot(const double* q, Eigen::Matrix<double, 4, 4>& T, Eigen::Matrix<double, 3, 3>& R, Eigen::Vector3d& x)
	{
		Eigen::Matrix<double, 4, 4> T1, T2, T3, T4, T5, T6, homoT;
		Eigen::Matrix3d rot_cur;
		Eigen::Vector3d trans_cur;
		double q1, q2, q3, q4, q5, q6;
		q1 = *q; q++; q2 = *q; q++; q3 = *q; q++; q4 = *q; q++; q5 = *q; q++; q6 = *q; q++;

		T1 << cos(q1), -sin(q1), 0, 0,
			sin(q1), cos(q1), 0, 0,
			0, 0, 1, d1,
			0, 0, 0, 1;
		T2 << cos(q2), -sin(q2), 0, 0,
			0, 0, -1, 0,
			sin(q2), cos(q2), 0, 0,
			0, 0, 0, 1;
		T3 << cos(q3), -sin(q3), 0, L2,
			sin(q3), cos(q3), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
		T4 << cos(q4), -sin(q4), 0, L3,
			sin(q4), cos(q4), 0, 0,
			0, 0, 1, d4,
			0, 0, 0, 1;
		T5 << cos(q5), -sin(q5), 0, 0,
			0, 0, -1, -d5,
			sin(q5), cos(q5), 0, 0,
			0, 0, 0, 1;
		T6 << cos(q6), -sin(q6), 0, 0,
			0, 0, 1, d6,
			-sin(q6), -cos(q6), 0, 0,
			0, 0, 0, 1;

		T = T1 * T2 * T3 * T4 * T5 * T6;

		R = T.block<3, 3>(0, 0);
		x = T.block<3, 1>(0, 3);
		/*
			*T = homoT(0, 0); T++;
			*T = homoT(0, 1); T++;
			*T = homoT(0, 2); T++;
			*T = homoT(0, 3); T++;
			*T = homoT(1, 0); T++;
			*T = homoT(1, 1); T++;
			*T = homoT(1, 2); T++;
			*T = homoT(1, 3); T++;
			*T = homoT(2, 0); T++;
			*T = homoT(2, 1); T++;
			*T = homoT(2, 2); T++;
			*T = homoT(2, 3); T++;
			*T = homoT(3, 0); T++;
			*T = homoT(3, 1); T++;
			*T = homoT(3, 2); T++;
			*T = homoT(3, 3); T++;
			*/


	}

	int CartesianSpaceTrackUR::inverse(const double* T, double* q_sols, double q6_des) {
		int num_sols = 0;
		double T00 = *T;  T++; double T01 = *T;  T++; double T02 = *T; T++; double T03 = *T; T++;
		double T10 = *T;  T++; double T11 = *T;  T++; double T12 = *T; T++; double T13 = *T; T++;
		double T20 = *T;  T++; double T21 = *T;  T++; double T22 = *T; T++; double T23 = *T;
		/*	std::cout << T00 << "  "; std::cout << T01 << "  "; std::cout << T02 << "  "; std::cout << T03 <<std::endl;
			std::cout << T10 << "  "; std::cout << T11 << "  "; std::cout << T12 << "  "; std::cout << T13 << std::endl;
			std::cout << T20 << "  "; std::cout << T21 << "  "; std::cout << T22 << "  "; std::cout << T23 << std::endl;*/
			//std::cout << T30 << "  "; std::cout << T31 << "  "; std::cout << T32 << "  "; std::cout << T33 << std::endl;
			////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
		double q1[2];
		{
			double A = d6 * T12 - T13;
			double B = d6 * T02 - T03;
			double R = A * A + B * B;
			if (fabs(A) < ZERO_THRESH) {
				double div;
				if (fabs(fabs(d4) - fabs(B)) < ZERO_THRESH)
					div = -SIGN(d4) * SIGN(B);
				else
					div = -d4 / B;
				double arcsin = asin(div);
				if (fabs(arcsin) < ZERO_THRESH)
					arcsin = 0.0;
				if (arcsin < 0.0)
					q1[0] = arcsin + 2.0 * PI;
				else
					q1[0] = arcsin;
				q1[1] = PI - arcsin;
			}
			else if (fabs(B) < ZERO_THRESH) {
				double div;
				if (fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
					div = SIGN(d4) * SIGN(A);
				else
					div = d4 / A;
				double arccos = acos(div);
				q1[0] = arccos;
				q1[1] = 2.0 * PI - arccos;
			}
			else if (d4 * d4 > R) {
				return num_sols;
			}
			else {
				double arccos = acos(d4 / sqrt(R));
				double arctan = atan2(-B, A);
				double pos = arccos + arctan;
				double neg = -arccos + arctan;
				if (fabs(pos) < ZERO_THRESH)
					pos = 0.0;
				if (fabs(neg) < ZERO_THRESH)
					neg = 0.0;
				if (pos >= 0.0)
					q1[0] = pos;
				else
					q1[0] = 2.0 * PI + pos;
				if (neg >= 0.0)
					q1[1] = neg;
				else
					q1[1] = 2.0 * PI + neg;
			}
		}
		////////////////////////////////////////////////////////////////////////////////

		////////////////////////////// wrist 2 joint (q5) //////////////////////////////
		double q5[2][2];
		{
			for (int i = 0; i < 2; i++) {
				double numer = (T03 * sin(q1[i]) - T13 * cos(q1[i]) - d4);
				double div;
				if (fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH)
					div = SIGN(numer) * SIGN(d6);
				else
					div = numer / d6;
				double arccos = acos(div);
				q5[i][0] = arccos;
				q5[i][1] = 2.0 * PI - arccos;
			}
		}
		////////////////////////////////////////////////////////////////////////////////

		{
			for (int i = 0; i < 2; i++) {
				for (int j = 0; j < 2; j++) {
					double c1 = cos(q1[i]), s1 = sin(q1[i]);
					double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
					double q6;
					////////////////////////////// wrist 3 joint (q6) //////////////////////////////
					if (fabs(s5) < ZERO_THRESH)
						q6 = 0.0;
					else {
						q6 = atan2(SIGN(s5) * -(T01 * s1 - T11 * c1),
							SIGN(s5) * (T00 * s1 - T10 * c1));
						if (fabs(q6) < ZERO_THRESH)
							q6 = 0.0;
						if (q6 < 0.0)
							q6 += 2.0 * PI;
					}
					////////////////////////////////////////////////////////////////////////////////

					double q2[2], q3[2], q4[2];
					///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
					double c6 = cos(q6), s6 = sin(q6);
					double x04x = -s5 * (T02 * c1 + T12 * s1) - c5 * (s6 * (T01 * c1 + T11 * s1) - c6 * (T00 * c1 + T10 * s1));
					double x04y = c5 * (T20 * c6 - T21 * s6) - T22 * s5;
					double p13x = d5 * (s6 * (T00 * c1 + T10 * s1) + c6 * (T01 * c1 + T11 * s1)) - d6 * (T02 * c1 + T12 * s1) +
						T03 * c1 + T13 * s1;
					double p13y = T23 - d1 - d6 * T22 + d5 * (T21 * c6 + T20 * s6);

					double c3 = (p13x * p13x + p13y * p13y - L2 * L2 - L3 * L3) / (2.0 * L2 * L3);
					if (fabs(fabs(c3) - 1.0) < ZERO_THRESH)
						c3 = SIGN(c3);
					else if (fabs(c3) > 1.0) {
						// TODO NO SOLUTION
						continue;
					}
					double arccos = acos(c3);
					q3[0] = arccos;
					q3[1] = 2.0 * PI - arccos;
					double denom = L2 * L2 + L3 * L3 + 2 * L2 * L3 * c3;
					double s3 = sin(arccos);
					double A = (L2 + L3 * c3), B = L3 * s3;
					q2[0] = atan2((A * p13y - B * p13x) / denom, (A * p13x + B * p13y) / denom);
					q2[1] = atan2((A * p13y + B * p13x) / denom, (A * p13x - B * p13y) / denom);
					double c23_0 = cos(q2[0] + q3[0]);
					double s23_0 = sin(q2[0] + q3[0]);
					double c23_1 = cos(q2[1] + q3[1]);
					double s23_1 = sin(q2[1] + q3[1]);
					q4[0] = atan2(c23_0 * x04y - s23_0 * x04x, x04x * c23_0 + x04y * s23_0);
					q4[1] = atan2(c23_1 * x04y - s23_1 * x04x, x04x * c23_1 + x04y * s23_1);
					////////////////////////////////////////////////////////////////////////////////
					for (int k = 0; k < 2; k++) {
						if (fabs(q2[k]) < ZERO_THRESH)
							q2[k] = 0.0;
						else if (q2[k] < 0.0) q2[k] += 2.0 * PI;
						if (fabs(q4[k]) < ZERO_THRESH)
							q4[k] = 0.0;
						else if (q4[k] < 0.0) q4[k] += 2.0 * PI;
						q_sols[num_sols * 6 + 0] = q1[i];    q_sols[num_sols * 6 + 1] = q2[k];
						q_sols[num_sols * 6 + 2] = q3[k];    q_sols[num_sols * 6 + 3] = q4[k];
						q_sols[num_sols * 6 + 4] = q5[i][j]; q_sols[num_sols * 6 + 5] = q6;
						num_sols++;
					}

				}
			}
		}
		return num_sols;
	}
	/// <根据反解结果选择最合适的一组解>
	/// 输入：int isolutions:几组解；double* q_sols:存放解的数组[8*6=48]
	///		 double* q_actual_pos：读取到的机器人实际位置；double<vector>& q_choosed：选择的解存放位置
	/// 没有解时，将实际位置赋值给结果
	/// 根据几组反解结果，每一组与实际值做绝对值差值对比，前三个XYZ差值乘以0.6，后三个RX/RY/RZ差值乘以0.4
	/// </summary>
	void ik_solution_choose(int isolutions, double df_q_sols[], double q_actual_pos[], double q_choosed[])
	{
		double df_total_diff[8] = { 0.0 };
		double df_pos_diff[8] = { 0.0 };
		double df_rotate_diff[8] = { 0.0 };
		double df_max_diff = 0.0;
		int i_order = 0;

		if (0 == isolutions)
		{
			for (int mk = 0; mk < 6; mk++)
				 q_choosed[mk] = q_actual_pos[mk];
		}
		for (int i = 0; i < isolutions; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				df_pos_diff[i] += abs(df_q_sols[i * 6 + j] - q_actual_pos[j]);
			}
			for (int m = 3; m < 6; m++)
			{
				df_rotate_diff[i] += abs(df_q_sols[i * 6 + m] - q_actual_pos[m]);
			}
			df_total_diff[i] = 0.6 * df_pos_diff[i] + 0.4 * df_rotate_diff[i];
		}
		for (int k = 0; k < isolutions; k++)
		{
			if (df_total_diff[k] > df_max_diff)
			{
				i_order = k;
				df_max_diff = df_total_diff[k];
			}
		}
		for (int mn = 0; mn < 6; mn++)
			;// q_choosed[mn] = df_q_sols[i_order * 6 + mn];
		
	}


	double* CartesianSpaceTrackUR::joint_space_3traj(double q_start[6], double q_bypass[6], double q_end[6], double* df_speed, double* df_acc, double df_increas, double df_motion_finish_time)
	{
		//最后两个参数传入的是多少个数据点，乘以0.008等于时间

		// 单关节多项式关节空间轨迹
		// 三阶多项式theta(t) = a0 + a1 * t + a2 * t ^ 2 + a3 * t ^ 3
		//指定初始和终止点的位置theta_s theta_f，同时速度均为0
		double a0_3[6], a1_3[6], a2_3[6], a3_3[6], a10[6], a11[6], a12[6], a13[6];//多项式的4个系数
		double* q_return = new double[6];
		std::vector<double> q_ = client.getPosition();
		std::vector<double> qd_ = client.getSpeed();
		for (int i = 0; i < 6; i++)
		{
			a0_3[i] = q_[i];
			a1_3[i] = 0;
			a2_3[i] = (12 * q_bypass[i] - 3 * q_end[i] - 9 * q_[i]) / (4 * pow(df_motion_finish_time * df_rob_loop_time, 2));
			a3_3[i] = (-8 * q_bypass[i] + 3 * q_end[i] + 5 * q_[i]) / (4 * pow(df_motion_finish_time * df_rob_loop_time, 3));

			a10[i] = q_bypass[i];
			a11[i] = (  3 * q_end[i]    - 3 * q_[i])                / (4* df_motion_finish_time * df_rob_loop_time);
			a12[i] = (-12 * q_bypass[i] + 6 * q_end[i] + 6 * q_[i]) / (4 * pow(df_motion_finish_time * df_rob_loop_time, 2));
			a13[i] = (  8 * q_bypass[i] - 5 * q_end[i] - 3 * q_[i]) / (4 * pow(df_motion_finish_time * df_rob_loop_time, 3));

			if (df_increas <= df_motion_finish_time / 2)
			{
				df_speed[i] = a1_3[i] + 2 * a2_3[i] * df_increas * df_rob_loop_time + 3 * a3_3[i] * pow(df_increas * df_rob_loop_time, 2);
				df_acc[i] = 2 * a2_3[i] + 6 * a3_3[i] * df_increas * df_rob_loop_time;
				q_return[i] = a0_3[i] + a1_3[i] * df_increas * df_rob_loop_time + a2_3[i] * pow(df_increas * df_rob_loop_time, 2) + a3_3[i] * pow(df_increas * df_rob_loop_time, 3);
			}
			else
			{
				df_speed[i] = a11[i] + 2 * a12[i] * df_increas * df_rob_loop_time + 3 * a13[i] * pow(df_increas * df_rob_loop_time, 2);
				df_acc[i] = 2 * a12[i] + 6 * a13[i] * df_increas * df_rob_loop_time;
				q_return[i] = a10[i] + a11[i] * df_increas * df_rob_loop_time + a12[i] * pow(df_increas * df_rob_loop_time, 2) + a13[i] * pow(df_increas * df_rob_loop_time, 3);
			}

			//std::cout << "The temp q" << q_return[i] << std::endl;
		}
		return q_return;
	}



	double* CartesianSpaceTrackUR::joint_space_3traj(double q_start[6], double q_end[6], double *df_speed, double *df_acc, double df_increas, double df_motion_finish_time)
	{
		//最后两个参数传入的是多少个数据点，乘以0.008等于时间
	
		// 单关节多项式关节空间轨迹
		// 三阶多项式theta(t) = a0 + a1 * t + a2 * t ^ 2 + a3 * t ^ 3
		//指定初始和终止点的位置theta_s theta_f，同时速度均为0
		double a0_3[6], a1_3[6], a2_3[6], a3_3[6];//多项式的4个系数
		double* q_return = new double[6];
		Eigen::VectorXd vcActQd;
		vcActQd.resize(6);
		vcActQd=get_qd();
		for (int i = 0; i < 6; i++)
		{
			a0_3[i] = q_start[i];
			a1_3[i] = vcActQd(i);
			a2_3[i] = (3 / pow(df_motion_finish_time*df_rob_loop_time,2)) * (q_end[i] - q_start[i]);
			a3_3[i] = (-2 / pow(df_motion_finish_time * df_rob_loop_time, 3)) * (q_end[i] - q_start[i]);
			df_speed[i] = a1_3[i]+2* a2_3[i]* df_increas * df_rob_loop_time+3* a3_3[i] * pow(df_increas * df_rob_loop_time, 2);
			df_acc[i] = 2 * a2_3[i]+6 * a3_3[i] * df_increas * df_rob_loop_time;
			q_return[i] = a0_3[i] + a1_3[i] * df_increas*df_rob_loop_time + a2_3[i] * pow(df_increas * df_rob_loop_time, 2) + a3_3[i] * pow(df_increas * df_rob_loop_time, 3);
			//std::cout << "The temp q" << q_return[i] << std::endl;
		}
		return q_return;
	}

	double* CartesianSpaceTrackUR::joint_space_parabola_fit_line_traj(double q_start[6], double q_end[6], double df_increas, double df_motion_finish_time,double acc)
	{
		return 0;
	}
	double* CartesianSpaceTrackUR::joint_traj(double q_start[6], double q_end[6], double df_increas)
	{
		double* q_temp = new double[6];
		for (int i = 0; i < 6; i++)
		{
			q_temp[i] = q_start[i] + df_increas * (q_end[i] - q_start[i]);
			//std::cout << "The temp q" << q_temp[i] << std::endl;
		}

		return q_temp;
	}
	bool CartesianSpaceTrackUR::joint_gen_point_sequence(Eigen::Matrix<double, 4, 4>& Matrix_start_point, Eigen::Matrix<double, 4, 4>& Matrix_bypass_point, Eigen::Matrix<double, 4, 4>& Matrix_end_point, double* q_solution_sequence, double* q_speed_sequence, double* q_acc_sequence, int i_increase)
	{
		double T_temp[4][4], T_bypass[4][4];
		double* T_Temp = &T_temp[0][0];
		double* pT_bypass = &T_bypass[0][0];
		double q_sols[48], q_deleate_sols[48], q_bypass_sols[48], q_deleate_bypass_sols[48];
		double df_increase = 0.0;
		//关节空间插值	
		double* q_temp; double q_speed_temp[6]; double q_acc_temp[6];
		Eigen::Matrix<double, 4, 4> tb_trans,Mat_bypass;
		tb_trans = (Matrix_end_point.transpose());
		Mat_bypass = (Matrix_bypass_point.transpose());
		T_Temp = tb_trans.data();//T_Temp
		pT_bypass = Mat_bypass.data();
		double q_choosed_pos[6], q_choosed_pos_modify[6], q_former_choosed_pos[6], q_choosed_bypass_pos[6], q_choosed_bypass_pos_modify[6];//单次选出的解
		int num_sols, num_remained_sols, num_bypass_sols, num_bypass_remained_sols;
		std::ofstream record_set_pos("record_jointspace_set_pos.txt", std::ios::app);//trunc
		record_set_pos << "start calculate point" << std::endl;
		std::ofstream record_set_speed("record_jointspace_set_speed.txt", std::ios::app);//trunc
		record_set_speed << "start calculate point speed" << std::endl;
		std::ofstream record_set_acc("record_jointspace_set_acc.txt", std::ios::app);//trunc
		record_set_acc << "start calculate point acc" << std::endl;
		update(boost::get_system_time(), boost::posix_time::milliseconds(8));
		q_former_choosed_pos[0] = q1;
		q_former_choosed_pos[1] = q2;
		q_former_choosed_pos[2] = q3;
		q_former_choosed_pos[3] = q4;
		q_former_choosed_pos[4] = q5;
		q_former_choosed_pos[5] = q6;
		record_set_pos << "current joint pos:" << q1 << "  " << q2 << "  " << q3 << "  " << q4 << "  " << q5 << "  " << q6 << "  " << std::endl;
		record_set_pos << "all solutions:" << std::endl;
		num_sols = inverse(T_Temp, q_sols, 0.0);
		num_bypass_sols = inverse(pT_bypass, q_bypass_sols, 0.0);
		std::cout << "num_sol=" << num_sols << std::endl;
		for (int i = 0; i < 48; i++)
		{
			if ((0 == i % 6))
			{
				std::cout << std::endl;
				record_set_pos << std::endl;
			}

			std::cout << std::setw(12) << q_sols[i];
			record_set_pos << std::setw(16) << q_sols[i];
		}
		std::cout << std::endl;
		record_set_pos << std::endl;
		if ((0 == num_sols) || (0==num_bypass_sols))
			return false;
		num_remained_sols = deleate_cannot_reach(num_sols, q_sols, q_deleate_sols);
		num_bypass_remained_sols = deleate_cannot_reach(num_sols, q_bypass_sols, q_deleate_bypass_sols);
		std::cout << "remain num_sol=" << num_remained_sols << std::endl;
		for (int i = 0; i < 48; i++)
		{
			if ((0 == i % 6))
				std::cout << std::endl;
			std::cout << std::setw(12) << q_deleate_sols[i];
		}
		std::cout << std::endl;
		if ((0 == num_remained_sols) || (0 == num_bypass_remained_sols))
			return false;
		//solution_choose_sigle_axis_diff(num_remained_sols, q_deleate_sols, q_former_choosed_pos, q_choosed_pos);
		solution_choose_total_diff(num_remained_sols, q_deleate_sols, q_former_choosed_pos, q_choosed_pos);
		solution_choose_total_diff(num_bypass_remained_sols, q_deleate_bypass_sols, q_former_choosed_pos, q_choosed_bypass_pos);
		//1~3轴赋值
		q_choosed_pos_modify[0] =
			abs(q_choosed_pos[0] - q_former_choosed_pos[0]) <= abs(q_choosed_pos[0] - 2 * M_PI - q_former_choosed_pos[0]) ? q_choosed_pos[0] : (q_choosed_pos[0] - 2 * M_PI);
		q_choosed_bypass_pos_modify[0] =
			abs(q_choosed_bypass_pos[0] - q_former_choosed_pos[0]) <= abs(q_choosed_bypass_pos[0] - 2 * M_PI - q_former_choosed_pos[0]) ? q_choosed_bypass_pos[0] : (q_choosed_bypass_pos[0] - 2 * M_PI);
		for (int i = 1; i < 3; i++)
		{
			if (q_choosed_pos[i] <= M_PI)
				q_choosed_pos_modify[i] = q_choosed_pos[i];
			else
				q_choosed_pos_modify[i] = q_choosed_pos[i] - 2 * M_PI;
			std::cout << std::setw(12) << q_choosed_pos[i];
		}
		//4~6轴赋值
		for (int i = 3; i < 6; i++)
		{
			//if (q_choosed_pos[i] <= M_PI)
			//	q_choosed_pos_modify[i] = q_choosed_pos[i];
			//else   // 1.0 <= ((double)icounter / itotaltime) ? 1.0 : ((double)icounter / itotaltime);
			q_choosed_pos_modify[i] =
				abs(q_choosed_pos[i] - q_former_choosed_pos[i]) <= abs(q_choosed_pos[i] - 2 * M_PI - q_former_choosed_pos[i]) ? q_choosed_pos[i] : (q_choosed_pos[i] - 2 * M_PI);
			std::cout << std::setw(12) << q_choosed_pos[i];
		}
		std::cout << std::endl;
		for (int inc = 0; inc < i_increase + 1; inc++)
		{
			df_increase = ((double)inc / i_increase);
			//q_temp = joint_traj(&q_former_choosed_pos[0], &q_choosed_pos_modify[0], df_increase);//关节空间直接插值
			q_temp = joint_space_3traj(&q_former_choosed_pos[0], &q_choosed_pos_modify[0], q_speed_temp, q_acc_temp, inc, i_increase);//三次多项式插值
			for (int m = 0; m < 6; m++)
			{
				if (abs(q_temp[m]) < ZERO_THRESH)
					q_solution_sequence[m + inc * 6] = 0;
				else if (std::isnan(q_temp[m]) or std::isinf(q_temp[m]))//or std::isfinite()
					q_solution_sequence[m + inc * 6] = q_temp[m];
				else
					q_solution_sequence[m + inc * 6] = q_temp[m];
				q_speed_sequence[m + inc * 6] = q_speed_temp[m];
				q_acc_sequence[m + inc * 6] = q_acc_temp[m];
				//q_former_choosed_pos[m] = q_solution_sequence[m + inc * 6];
				record_set_pos << std::setw(12) << q_solution_sequence[m + inc * 6];
				record_set_speed << std::setw(12) << q_speed_sequence[m + inc * 6];
				record_set_acc << std::setw(12) << q_acc_sequence[m + inc * 6];
			}



			record_set_pos << std::endl;
			record_set_speed << std::endl;
			record_set_acc << std::endl;
		}
		//record_set_pos << std::endl;
		record_set_pos.close();
		record_set_speed.close();
		record_set_acc.close();
		return true;

	}
	bool CartesianSpaceTrackUR::joint_gen_point_sequence(Eigen::Matrix<double, 4, 4>& Matrix_start_point, Eigen::Matrix<double, 4, 4>& Matrix_end_point, double* q_solution_sequence, double* q_speed_sequence, double* q_acc_sequence ,int i_increase)
	{
		double T_temp[4][4]; double* T_Temp = &T_temp[0][0];
		double q_sols[48],q_deleate_sols[48];
		double df_increase = 0.0;
		//关节空间插值	
		double* q_temp; double q_speed_temp[6]; double q_acc_temp[6];
		Eigen::Matrix<double, 4, 4> tb_trans;
		tb_trans = (Matrix_end_point.transpose());
		T_Temp = tb_trans.data();//T_Temp
		double q_choosed_pos[6], q_choosed_pos_modify[6], q_former_choosed_pos[6];//单次选出的解
		int num_sols, num_remained_sols;
		std::ofstream record_set_pos("record_jointspace_set_pos.txt", std::ios::app);//trunc
		record_set_pos << "start calculate point" << std::endl;
		std::ofstream record_set_speed("record_jointspace_set_speed.txt", std::ios::app);//trunc
		record_set_speed << "start calculate point speed" << std::endl;
		std::ofstream record_set_acc("record_jointspace_set_acc.txt", std::ios::app);//trunc
		record_set_acc << "start calculate point acc" << std::endl;
		update(boost::get_system_time(), boost::posix_time::milliseconds(8));
		q_former_choosed_pos[0] = q1;
		q_former_choosed_pos[1] = q2;
		q_former_choosed_pos[2] = q3;
		q_former_choosed_pos[3] = q4;
		q_former_choosed_pos[4] = q5;
		q_former_choosed_pos[5] = q6;
		record_set_pos << "current joint pos:" << q1 << "  " << q2 << "  " << q3 << "  " << q4 << "  " << q5 << "  " << q6 << "  "<<std::endl;
		record_set_pos << "all solutions:" << std::endl;
		num_sols = inverse(T_Temp, q_sols, 0.0);
		std::cout << "num_sol=" << num_sols << std::endl;
		for (int i = 0; i < 48; i++)
		{
			if ((0 == i % 6))
			{
				std::cout << std::endl;
				record_set_pos <<  std::endl;
			}
				
			std::cout << std::setw(12) << q_sols[i];
			record_set_pos << std::setw(16) << q_sols[i];
		}
		std::cout << std::endl;
		record_set_pos << std::endl;
		if (0 == num_sols)
			return false;
		num_remained_sols = deleate_cannot_reach(num_sols, q_sols, q_deleate_sols);
		std::cout << "remain num_sol=" << num_remained_sols << std::endl;
		for (int i = 0; i < 48; i++)
		{
			if ((0 == i % 6))
				std::cout << std::endl;
			std::cout << std::setw(12) << q_deleate_sols[i];
		}
		std::cout << std::endl;
		if (0 == num_remained_sols)
			return false;
		//solution_choose_sigle_axis_diff(num_remained_sols, q_deleate_sols, q_former_choosed_pos, q_choosed_pos);
		solution_choose_total_diff(num_remained_sols, q_deleate_sols, q_former_choosed_pos, q_choosed_pos);
		//1~3轴赋值
		q_choosed_pos_modify[0] =
			abs(q_choosed_pos[0] - q_former_choosed_pos[0]) <= abs(q_choosed_pos[0] - 2 * M_PI - q_former_choosed_pos[0]) ? q_choosed_pos[0] : (q_choosed_pos[0] - 2 * M_PI);
		for (int i = 1; i < 3; i++)
		{
			if (q_choosed_pos[i]<=M_PI)
				q_choosed_pos_modify[i] =q_choosed_pos[i];
			else
				q_choosed_pos_modify[i] = q_choosed_pos[i]-2 * M_PI;
			std::cout << std::setw(12) << q_choosed_pos[i];
		}
		//4~6轴赋值
		for (int i = 3; i < 6; i++)
		{
			//if (q_choosed_pos[i] <= M_PI)
			//	q_choosed_pos_modify[i] = q_choosed_pos[i];
			//else   // 1.0 <= ((double)icounter / itotaltime) ? 1.0 : ((double)icounter / itotaltime);
				q_choosed_pos_modify[i] = 
				abs(q_choosed_pos[i]-q_former_choosed_pos[i])<= abs(q_choosed_pos[i] -2* M_PI - q_former_choosed_pos[i]) ? q_choosed_pos[i]:(q_choosed_pos[i] - 2 * M_PI);
			std::cout << std::setw(12) << q_choosed_pos[i];
		}
		std::cout << std::endl;
		for (int inc = 0; inc < i_increase + 1; inc++)
		{
			df_increase = ((double)inc / i_increase);
			//q_temp = joint_traj(&q_former_choosed_pos[0], &q_choosed_pos_modify[0], df_increase);//关节空间直接插值
			q_temp = joint_space_3traj(&q_former_choosed_pos[0], &q_choosed_pos_modify[0], q_speed_temp, q_acc_temp, inc, i_increase);//三次多项式插值
			for (int m = 0; m < 6; m++)
			{
				if (abs(q_temp[m]) < ZERO_THRESH)
					q_solution_sequence[m + inc * 6] = 0;
				else if (std::isnan(q_temp[m]) or std::isinf(q_temp[m]))//or std::isfinite()
					q_solution_sequence[m + inc * 6] = q_temp[m];
				else
					q_solution_sequence[m + inc * 6] = q_temp[m];
				q_speed_sequence[m + inc * 6] = q_speed_temp[m];
				q_acc_sequence[m + inc * 6] = q_acc_temp[m];
				//q_former_choosed_pos[m] = q_solution_sequence[m + inc * 6];
				record_set_pos	<< std::setw(12) << q_solution_sequence[m + inc * 6];
				record_set_speed<< std::setw(12) << q_speed_sequence[m + inc * 6];
				record_set_acc	<< std::setw(12) << q_acc_sequence[m + inc * 6];
			}
			


			record_set_pos << std::endl;
			record_set_speed << std::endl;
			record_set_acc << std::endl;
		}
		//record_set_pos << std::endl;
		record_set_pos.close();
		record_set_speed.close();
		record_set_acc.close();
		return true;

	}

	bool CartesianSpaceTrackUR::ik_with_q(Eigen::Matrix<double, 4, 4>& Matrix_end_point,double * desire_q)
	{
		double T_temp[4][4]; double* T_Temp = &T_temp[0][0];
		double q_sols[48], q_deleate_sols[48];
		//关节空间插值	
		double* q_temp; 
		Eigen::Matrix<double, 4, 4> tb_trans;
		tb_trans = (Matrix_end_point.transpose());
		T_Temp = tb_trans.data();//T_Temp
		double* q_choosed_pos_modify = new double[6];
		double q_choosed_pos[6], q_former_choosed_pos[6];//单次选出的解
		int num_sols, num_remained_sols;
		//controller_ur->get_q();
		std::vector<double> q_ = client.getPosition();
		//update(boost::get_system_time(), boost::posix_time::milliseconds(8));
		//q_former_choosed_pos[0] = q1;
		//q_former_choosed_pos[1] = q2;
		//q_former_choosed_pos[2] = q3;
		//q_former_choosed_pos[3] = q4;
		//q_former_choosed_pos[4] = q5;
		//q_former_choosed_pos[5] = q6;
		for (int mm = 0; mm < 6; mm++)
		{
			q_former_choosed_pos[mm] = q_[mm];
			//std::cout << "Currend joint postion:= " << q_former_choosed_pos[mm] << std::endl;
		}
		num_sols = inverse(T_Temp, q_sols, 0.0);
		//std::cout << "num_sol=" << num_sols << std::endl;
		//for (int i = 0; i < 48; i++)
		//{
		//	if ((0 == i % 6))
		//	{
		//		std::cout << std::endl;
		//	}

		//	std::cout << std::setw(12) << q_sols[i];
		//}
		//std::cout << std::endl;

		num_remained_sols = deleate_cannot_reach(num_sols, q_sols, q_deleate_sols);
		if ((0 == num_sols) || (0 == num_remained_sols))
			return false;
		//std::cout <<"num_sols"<< num_sols << std::endl;
		//std::cout << "num_remained_sols" << num_remained_sols << std::endl;
		solution_choose_total_diff(num_remained_sols, q_deleate_sols, q_former_choosed_pos, q_choosed_pos);


				//1~3轴赋值
		desire_q[0] =
			abs(q_choosed_pos[0] - q_former_choosed_pos[0]) <= abs(q_choosed_pos[0] - 2 * M_PI - q_former_choosed_pos[0]) ? q_choosed_pos[0] : (q_choosed_pos[0] - 2 * M_PI);
		for (int i = 1; i < 3; i++)
		{
			if (q_choosed_pos[i] <= M_PI)
				desire_q[i] = q_choosed_pos[i];
			else
				desire_q[i] = q_choosed_pos[i] - 2 * M_PI;
			
		}
		//4~6轴赋值
		for (int i = 3; i < 6; i++)
		{
			desire_q[i] =
				abs(q_choosed_pos[i] - q_former_choosed_pos[i]) <= abs(q_choosed_pos[i] - 2 * M_PI - q_former_choosed_pos[i]) ? q_choosed_pos[i] : (q_choosed_pos[i] - 2 * M_PI);
			
		}
		return true;

	}
	//可能用不上,
	bool CartesianSpaceTrackUR::ik_with_start_pos(Eigen::Matrix<double, 4, 4>& Matrix_start_point, Eigen::Matrix<double, 4, 4>& Matrix_end_point, double* desire_q)
	{
		double T_temp[4][4]; double* T_Temp = &T_temp[0][0];
		double q_sols[48], q_deleate_sols[48];
		//关节空间插值	
		double* q_temp;
		Eigen::Matrix<double, 4, 4> tb_trans;
		tb_trans = (Matrix_end_point.transpose());
		T_Temp = tb_trans.data();//T_Temp
		double* q_choosed_pos_modify = new double[6];
		double q_choosed_pos[6], q_former_choosed_pos[6];//单次选出的解
		int num_sols, num_remained_sols;
		//controller_ur->get_q();
		std::vector<double> q_ = client.getPosition();
		//update(boost::get_system_time(), boost::posix_time::milliseconds(8));
		//q_former_choosed_pos[0] = q1;
		//q_former_choosed_pos[1] = q2;
		//q_former_choosed_pos[2] = q3;
		//q_former_choosed_pos[3] = q4;
		//q_former_choosed_pos[4] = q5;
		//q_former_choosed_pos[5] = q6;
		for (int mm = 0; mm < 6; mm++)
		{
			q_former_choosed_pos[mm] = q_[mm];
			//std::cout << "Currend joint postion:= " << q_former_choosed_pos[mm] << std::endl;
		}
		num_sols = inverse(T_Temp, q_sols, 0.0);


		num_remained_sols = deleate_cannot_reach(num_sols, q_sols, q_deleate_sols);
		if ((0 == num_sols) || (0 == num_remained_sols))
			return false;

		solution_choose_total_diff(num_remained_sols, q_deleate_sols, q_former_choosed_pos, q_choosed_pos);

		//1~3轴赋值
		desire_q[0] =
			abs(q_choosed_pos[0] - q_former_choosed_pos[0]) <= abs(q_choosed_pos[0] - 2 * M_PI - q_former_choosed_pos[0]) ? q_choosed_pos[0] : (q_choosed_pos[0] - 2 * M_PI);
		for (int i = 1; i < 3; i++)
		{
			if (q_choosed_pos[i] <= M_PI)
				desire_q[i] = q_choosed_pos[i];
			else
				desire_q[i] = q_choosed_pos[i] - 2 * M_PI;

		}
		//4~6轴赋值
		for (int i = 3; i < 6; i++)
		{
			desire_q[i] =
				abs(q_choosed_pos[i] - q_former_choosed_pos[i]) <= abs(q_choosed_pos[i] - 2 * M_PI - q_former_choosed_pos[i]) ? q_choosed_pos[i] : (q_choosed_pos[i] - 2 * M_PI);

		}
		return true;

	}


	//姿态插值
	void CartesianSpaceTrackUR::slerp(double starting[4], double ending[4], double result[4], double t)
	{
		float cosa = starting[0] * ending[0] + starting[1] * ending[1] + starting[2] * ending[2] + starting[3] * ending[3];

		// If the dot product is negative, the quaternions have opposite handed-ness and slerp won't take
		// the shorter path. Fix by reversing one quaternion.
		if (cosa < 0.0f)
		{
			ending[0] = -ending[0];
			ending[1] = -ending[1];
			ending[2] = -ending[2];
			ending[3] = -ending[3];
			cosa = -cosa;
		}

		float k0, k1;

		// If the inputs are too close for comfort, linearly interpolate
		if (cosa > 0.9995f)
		{
			k0 = 1.0f - t;
			k1 = t;
		}
		else
		{
			float sina = sqrt(1.0f - cosa * cosa);
			float a = atan2(sina, cosa);
			k0 = sin((1.0f - t) * a) / sina;
			k1 = sin(t * a) / sina;
		}
		result[0] = starting[0] * k0 + ending[0] * k1;
		result[1] = starting[1] * k0 + ending[1] * k1;
		result[2] = starting[2] * k0 + ending[2] * k1;
		result[3] = starting[3] * k0 + ending[3] * k1;
	}

	bool CartesianSpaceTrackUR::Ctraj_gen_step_point_sequence(Eigen::Matrix<double, 4, 4>& Matrix_start_point, Eigen::Matrix<double, 4, 4>& Matrix_end_point, double* q_solution_sequence,int inc, double i_increase, double time_increases)
	{

		Eigen::Matrix<double, 4, 4>  Ta;
		Eigen::Matrix<double, 3, 3>  Ra;
		Eigen::Vector3d  Va;
		Eigen::Matrix<double, 3, 3> R_start, R_end;
		Eigen::Vector3d pos_start, pos_end;
		R_start = Matrix_start_point.block<3, 3>(0, 0);
		R_end = Matrix_end_point.block<3, 3>(0, 0);
		pos_start = Matrix_start_point.block<3, 1>(0, 3);
		pos_end = Matrix_end_point.block<3, 1>(0, 3);
		Eigen::Quaterniond quat1(R_start), quat2(R_end), quat_temp;
		double df_quatA[4], df_quatB[4], df_quat_temp[4];
		Eigen::Matrix3d R5;//生成姿态用
		double p1[3], p2[3], p_temp[3];//位置插值用
		double* T_retrun = new double[16];
		double df_increase = 0.0;
		double q_sols[48], q_deleate_sols[48];
		double q_choosed_pos[6], q_former_choosed_pos[6];//单次选出的解
		int num_sols, num_remained_sols;
		std::ofstream record_set_pos("record_set_pos.txt", std::ios::app);//trunc
		//std::ofstream record_set_pos("record_set_pos.csv", std::ios::trunc);//trunc
		//Eigen::VectorXd vxd_q_act_post = client.getPosition();//get_q();
		//double* df_q_act_post = vxd_q_act_post.transpose().data();
		double T_temp[4][4]; double* T_Temp = &T_temp[0][0];
		df_quatA[0] = quat1.w(); df_quatA[1] = quat1.x(); df_quatA[2] = quat1.y(); df_quatA[3] = quat1.z();
		df_quatB[0] = quat2.w(); df_quatB[1] = quat2.x(); df_quatB[2] = quat2.y(); df_quatB[3] = quat2.z();
		//update(boost::get_system_time(), boost::posix_time::milliseconds(8));
		q_former_choosed_pos[0] = q1;
		q_former_choosed_pos[1] = q2;
		q_former_choosed_pos[2] = q3;
		q_former_choosed_pos[3] = q4;
		q_former_choosed_pos[4] = q5;
		q_former_choosed_pos[5] = q6;
		record_set_pos << "current joint pos:" << q1 << "  " << q2 << "  " << q3 << "  " << q4 << "  " << q5 << "  " << q6 << "  " << std::endl;
		//fkRobot(&q_former_choosed_pos[0], Ta, Ra, Va);
		record_set_pos << "start calculate point" << std::endl;
		//for(int inc=1;inc< i_increase+1; inc++)
		//int inc = 0;
		{
			df_increase = time_increases;
			//df_increase = ((double)inc / i_increase);
			T_retrun = Ctraj(Matrix_start_point, Matrix_end_point, df_increase);
			//T_retrun = Ctraj_cubic_poly(Ta, Matrix_end_point, inc, i_increase);
			num_sols = inverse(T_retrun, q_sols, 0.0);

			if (0 == num_sols)
				return false;
			num_remained_sols = deleate_cannot_reach(num_sols, q_sols, q_deleate_sols);
			if (0 == num_remained_sols)
				return false;
			//solution_choose_sigle_axis_diff(num_remained_sols, q_deleate_sols, q_former_choosed_pos, q_choosed_pos);
			solution_choose_total_diff(num_remained_sols, q_deleate_sols, q_former_choosed_pos, q_choosed_pos);
			for (int m = 0; m < 3; m++)
			{
				if (0 == m)
				{
					if (abs(q_choosed_pos[m]) < ZERO_THRESH)
						q_solution_sequence[m + inc * 6] = 0;
					else if (std::isnan(q_choosed_pos[m]) or std::isinf(q_choosed_pos[m]))//or std::isfinite()
						q_solution_sequence[m + inc * 6] = q_former_choosed_pos[m];
					else if (abs(q_choosed_pos[m] - q_former_choosed_pos[m]) < abs(q_choosed_pos[m] - 2 * M_PI - q_former_choosed_pos[m]))
						q_solution_sequence[m + inc * 6] = q_choosed_pos[m];
					else
						q_solution_sequence[m + inc * 6] = q_choosed_pos[m] - 2 * M_PI;
					//q_solution_sequence[m + inc * 6] = abs(q_former_choosed_pos[m])> abs(q_former_choosed_pos[m] - q_choosed_pos[m] )? q_choosed_pos[m]:q_choosed_pos[m]-2 * M_PI;
					q_former_choosed_pos[m] = q_solution_sequence[m + 0 * 6];
					record_set_pos << std::setw(12) << q_solution_sequence[m + inc * 6];
				}
				else
				{
					if (abs(q_choosed_pos[m]) < ZERO_THRESH)
						q_solution_sequence[m + inc * 6] = 0;
					else if (std::isnan(q_choosed_pos[m]) or std::isinf(q_choosed_pos[m]))//or std::isfinite()
						q_solution_sequence[m + inc * 6] = q_former_choosed_pos[m];
					else if (q_choosed_pos[m] <= M_PI)
						q_solution_sequence[m + inc * 6] = q_choosed_pos[m];
					else
						q_solution_sequence[m + inc * 6] = q_choosed_pos[m] - 2 * M_PI;
					q_former_choosed_pos[m] = q_solution_sequence[m + inc * 6];
					record_set_pos << std::setw(12) << q_solution_sequence[m + inc * 6];
				}
			}
			for (int m = 3; m < 6; m++)
			{
				if (abs(q_choosed_pos[m]) < ZERO_THRESH)
					q_solution_sequence[m + inc * 6] = 0;
				else if (std::isnan(q_choosed_pos[m]) or std::isinf(q_choosed_pos[m]))//or std::isfinite()
					q_solution_sequence[m + inc * 6] = q_former_choosed_pos[m];
				else if (abs(q_choosed_pos[m] - q_former_choosed_pos[m]) < abs(q_choosed_pos[m] - 2 * M_PI - q_former_choosed_pos[m]))
					q_solution_sequence[m + inc * 6] = q_choosed_pos[m];
				else
					q_solution_sequence[m + inc * 6] = q_choosed_pos[m] - 2 * M_PI;
				//q_solution_sequence[m + inc * 6] = abs(q_former_choosed_pos[m])> abs(q_former_choosed_pos[m] - q_choosed_pos[m] )? q_choosed_pos[m]:q_choosed_pos[m]-2 * M_PI;
				q_former_choosed_pos[m] = q_solution_sequence[m + inc * 6];
				record_set_pos << std::setw(12) << q_solution_sequence[m + inc * 6];
			}
			record_set_pos << std::endl;

		}

		record_set_pos.close();
		return true;

	}


	bool CartesianSpaceTrackUR::Ctraj_gen_point_sequence(Eigen::Matrix<double, 4, 4>& Matrix_start_point, Eigen::Matrix<double, 4, 4>& Matrix_end_point, double* q_solution_sequence, int i_increase)
	{

		Eigen::Matrix<double, 4, 4>  Ta;
		Eigen::Matrix<double, 3, 3>  Ra;
		Eigen::Vector3d  Va;
		Eigen::Matrix<double, 3, 3> R_start, R_end;
		Eigen::Vector3d pos_start, pos_end;
		R_start = Matrix_start_point.block<3, 3>(0, 0);
		R_end = Matrix_end_point.block<3, 3>(0, 0);
		pos_start = Matrix_start_point.block<3, 1>(0, 3);
		pos_end = Matrix_end_point.block<3, 1>(0, 3);
		Eigen::Quaterniond quat1(R_start), quat2(R_end), quat_temp;
		double df_quatA[4], df_quatB[4], df_quat_temp[4];
		Eigen::Matrix3d R5;//生成姿态用
		double p1[3], p2[3], p_temp[3];//位置插值用
		double* T_retrun = new double[16];
		double df_increase = 0.0;
		double q_sols[48], q_deleate_sols[48];
		double q_choosed_pos[6],q_former_choosed_pos[6];//单次选出的解
		int num_sols, num_remained_sols;
		std::ofstream record_set_pos("record_set_pos.txt", std::ios::app);//trunc
		//std::ofstream record_set_pos("record_set_pos.csv", std::ios::trunc);//trunc
		//Eigen::VectorXd vxd_q_act_post = client.getPosition();//get_q();
		//double* df_q_act_post = vxd_q_act_post.transpose().data();
		double T_temp[4][4]; double* T_Temp = &T_temp[0][0];
		df_quatA[0] = quat1.w(); df_quatA[1] = quat1.x(); df_quatA[2] = quat1.y(); df_quatA[3] = quat1.z();
		df_quatB[0] = quat2.w(); df_quatB[1] = quat2.x(); df_quatB[2] = quat2.y(); df_quatB[3] = quat2.z();
		update(boost::get_system_time(), boost::posix_time::milliseconds(8));
		q_former_choosed_pos[0] = q1;
		q_former_choosed_pos[1] = q2;
		q_former_choosed_pos[2] = q3;
		q_former_choosed_pos[3] = q4;
		q_former_choosed_pos[4] = q5;
		q_former_choosed_pos[5] = q6;
		record_set_pos << "current joint pos:" << q1 << "  " << q2 << "  " << q3 << "  " << q4 << "  " << q5 << "  " << q6 << "  " << std::endl;
		fkRobot(&q_former_choosed_pos[0], Ta, Ra, Va);
		record_set_pos << "start calculate point"<< std::endl;
		for(int inc=1;inc< i_increase+1; inc++)
		{ 
			//df_increase = ((double)1.0 / i_increase);
			df_increase = ((double)inc / i_increase);
			//T_retrun = Ctraj(Ta, Matrix_end_point, df_increase);
			T_retrun = Ctraj_cubic_poly(Ta, Matrix_end_point, inc, i_increase);
			num_sols = inverse(T_retrun, q_sols, 0.0);

			if (0 == num_sols)
				return false;
			num_remained_sols = deleate_cannot_reach(num_sols, q_sols, q_deleate_sols);
			if (0 == num_remained_sols)
				return false;
			//solution_choose_sigle_axis_diff(num_remained_sols, q_deleate_sols, q_former_choosed_pos, q_choosed_pos);
			solution_choose_total_diff(num_remained_sols, q_deleate_sols, q_former_choosed_pos, q_choosed_pos);
			for (int m = 0; m < 3; m++)
			{
				if (0 == m)
				{
					if (abs(q_choosed_pos[m]) < ZERO_THRESH)
						q_solution_sequence[m + inc * 6] = 0;
					else if (std::isnan(q_choosed_pos[m]) or std::isinf(q_choosed_pos[m]))//or std::isfinite()
						q_solution_sequence[m + inc * 6] = q_former_choosed_pos[m];
					else if (abs(q_choosed_pos[m] - q_former_choosed_pos[m]) < abs(q_choosed_pos[m] - 2 * M_PI - q_former_choosed_pos[m]))
						q_solution_sequence[m + inc * 6] = q_choosed_pos[m];
					else
						q_solution_sequence[m + inc * 6] = q_choosed_pos[m] - 2 * M_PI;
					//q_solution_sequence[m + inc * 6] = abs(q_former_choosed_pos[m])> abs(q_former_choosed_pos[m] - q_choosed_pos[m] )? q_choosed_pos[m]:q_choosed_pos[m]-2 * M_PI;
					q_former_choosed_pos[m] = q_solution_sequence[m + inc * 6];
					record_set_pos << std::setw(12) << q_solution_sequence[m + inc * 6];
				}
				else
				{
				if (abs(q_choosed_pos[m]) < ZERO_THRESH)
					q_solution_sequence[m + inc * 6] = 0;
				else if (std::isnan(q_choosed_pos[m]) or std::isinf(q_choosed_pos[m]))//or std::isfinite()
					q_solution_sequence[m + inc * 6] = q_former_choosed_pos[m];
				else if (q_choosed_pos[m] <= M_PI)
					q_solution_sequence[m + inc * 6] = q_choosed_pos[m];
				else
					q_solution_sequence[m + inc * 6] = q_choosed_pos[m] - 2 * M_PI;
				q_former_choosed_pos[m] = q_solution_sequence[m + inc * 6];
				record_set_pos << std::setw(12) << q_solution_sequence[m + inc * 6];
				}
			}
			for (int m = 3; m < 6; m++)
			{
				if (abs(q_choosed_pos[m]) < ZERO_THRESH)
					q_solution_sequence[m + inc * 6] = 0;
				else if (std::isnan(q_choosed_pos[m]) or std::isinf(q_choosed_pos[m]))//or std::isfinite()
					q_solution_sequence[m + inc * 6] = q_former_choosed_pos[m];
				else if (abs(q_choosed_pos[m] - q_former_choosed_pos[m]) < abs(q_choosed_pos[m] - 2 * M_PI - q_former_choosed_pos[m]))
					q_solution_sequence[m + inc * 6] = q_choosed_pos[m];
				else
					q_solution_sequence[m + inc * 6] = q_choosed_pos[m] - 2 * M_PI;
					//q_solution_sequence[m + inc * 6] = abs(q_former_choosed_pos[m])> abs(q_former_choosed_pos[m] - q_choosed_pos[m] )? q_choosed_pos[m]:q_choosed_pos[m]-2 * M_PI;
				q_former_choosed_pos[m] = q_solution_sequence[m + inc * 6];
				record_set_pos << std::setw(12) << q_solution_sequence[m + inc * 6];
			}
			record_set_pos << std::endl;
				
		}
			
		record_set_pos.close();
		return true;

	}


	double* CartesianSpaceTrackUR::Ctraj(Eigen::Matrix<double, 4, 4>& Matrix_start_point, Eigen::Matrix<double, 4, 4>& Matrix_end_point, double df_increas)
	{
		Eigen::Matrix<double, 3, 3> R_start, R_end;
		Eigen::Vector3d pos_start, pos_end;
		R_start = Matrix_start_point.block<3, 3>(0, 0);
		R_end = Matrix_end_point.block<3, 3>(0, 0);
		pos_start = Matrix_start_point.block<3, 1>(0, 3);
		pos_end = Matrix_end_point.block<3, 1>(0, 3);
		Eigen::Quaterniond quat1(R_start), quat2(R_end), quat_temp;
		double df_quatA[4], df_quatB[4], df_quat_temp[4];
		Eigen::Matrix3d R5;
		double p1[3], p2[3], p_temp[3];
		double* T_retrun = new double[16];
		double T_temp[4][4]; double* T_Temp = &T_temp[0][0];
		df_quatA[0] = quat1.w(); df_quatA[1] = quat1.x(); df_quatA[2] = quat1.y(); df_quatA[3] = quat1.z();
		df_quatB[0] = quat2.w(); df_quatB[1] = quat2.x(); df_quatB[2] = quat2.y(); df_quatB[3] = quat2.z();
		//姿态插值
		slerp(df_quatA, df_quatB, df_quat_temp, df_increas);
		quat_temp.w() = df_quat_temp[0]; quat_temp.x() = df_quat_temp[1]; quat_temp.y() = df_quat_temp[2]; quat_temp.z() = df_quat_temp[3];
		R5 = quat_temp.toRotationMatrix();
		//std::cout << "R5" << R5 << std::endl;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				T_temp[i][j] = R5(i, j);
			}
		}
		//位置部分
		p1[0] = pos_start[0]; p1[1] = pos_start[1]; p1[2] = pos_start[2];
		p2[0] = pos_end[0]; p2[1] = pos_end[1]; p2[2] = pos_end[2];
		//p_temp = p1 + s(p2-p1)   s(0→1）(p1→p2)
		p_temp[0] = p1[0] + df_increas * (p2[0] - p1[0]); p_temp[1] = p1[1] + df_increas * (p2[1] - p1[1]); p_temp[2] = p1[2] + df_increas * (p2[2] - p1[2]);
		//p_temp[0] = p1[0] + df_increas * (p2[0] - p1[0]); p_temp[1] = p1[1] + df_increas * (p2[1] - p1[1]); p_temp[2] = p1[2] + df_increas * (p2[2] - p1[2]);
		T_temp[0][3] = p_temp[0]; T_temp[1][3] = p_temp[1]; T_temp[2][3] = p_temp[2];
		T_temp[3][0] = 0; T_temp[3][1] = 0; T_temp[3][2] = 0; T_temp[3][3] = 1;
		for (int mk = 0; mk < 16; mk++)
		{
			T_retrun[mk] = T_Temp[mk];
		}

		return T_retrun;
	}

	double* CartesianSpaceTrackUR::Ctraj_cubic_poly(Eigen::Matrix<double, 4, 4>& Matrix_start_point, Eigen::Matrix<double, 4, 4>& Matrix_end_point, double df_increas, double df_motion_finish_time)
	{
		Eigen::Matrix<double, 3, 3> R_start, R_end;
		Eigen::Vector3d pos_start, pos_end;
		R_start = Matrix_start_point.block<3, 3>(0, 0);
		R_end = Matrix_end_point.block<3, 3>(0, 0);
		pos_start = Matrix_start_point.block<3, 1>(0, 3);
		pos_end = Matrix_end_point.block<3, 1>(0, 3);
		Eigen::Quaterniond quat1(R_start), quat2(R_end), quat_temp;
		double df_quatA[4], df_quatB[4], df_quat_temp[4];
		Eigen::Matrix3d R5;
		double p1[3], p2[3], p_temp[3];
		double df_time_increase;
		double* T_retrun = new double[16];
		double T_temp[4][4]; double* T_Temp = &T_temp[0][0];
		std::ofstream record_xyz_pos("record_xyz_pos.csv", std::ios::app);//trunc
		df_time_increase = 1.0 <= ((double)df_increas / df_motion_finish_time) ? 1.0 : ((float)df_increas / df_motion_finish_time);
		df_quatA[0] = quat1.w(); df_quatA[1] = quat1.x(); df_quatA[2] = quat1.y(); df_quatA[3] = quat1.z();
		df_quatB[0] = quat2.w(); df_quatB[1] = quat2.x(); df_quatB[2] = quat2.y(); df_quatB[3] = quat2.z();
		//姿态插值
		slerp(df_quatA, df_quatB, df_quat_temp, df_time_increase);
		quat_temp.w() = df_quat_temp[0]; quat_temp.x() = df_quat_temp[1]; quat_temp.y() = df_quat_temp[2]; quat_temp.z() = df_quat_temp[3];
		R5 = quat_temp.toRotationMatrix();
		//std::cout << "R5" << R5 << std::endl;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				T_temp[i][j] = R5(i, j);
			}
		}
		//位置部分
		p1[0] = pos_start[0]; p1[1] = pos_start[1]; p1[2] = pos_start[2];
		p2[0] = pos_end[0]; p2[1] = pos_end[1]; p2[2] = pos_end[2];

		// 三阶多项式theta(t) = a0 + a1 * t + a2 * t ^ 2 + a3 * t ^ 3
		//指定初始和终止点的位置theta_s theta_f，同时速度均为0
		double a0_3[3], a1_3[3], a2_3[3], a3_3[3];//多项式的4个系数
		double* q_return = new double[6];
		for (int i = 0; i < 3; i++)
		{
			a0_3[i] = pos_start[i];
			a1_3[i] = 0;
			a2_3[i] = (3 / pow(df_motion_finish_time * df_rob_loop_time, 2)) * (pos_end[i] - pos_start[i]);
			a3_3[i] = (-2 / pow(df_motion_finish_time * df_rob_loop_time, 3)) * (pos_end[i] - pos_start[i]);
			q_return[i] = a0_3[i] + a1_3[i] * df_increas * df_rob_loop_time + a2_3[i] * pow(df_increas * df_rob_loop_time, 2) + a3_3[i] * pow(df_increas * df_rob_loop_time, 3);
			record_xyz_pos << q_return[i] << ",";
		}
		record_xyz_pos << std::endl;

		//p_temp = p1 + s(p2-p1)   s(0→1）(p1→p2)
		p_temp[0] = q_return[0]; p_temp[1] = q_return[1]; p_temp[2] = q_return[2];
		T_temp[0][3] = p_temp[0]; T_temp[1][3] = p_temp[1]; T_temp[2][3] = p_temp[2];
		T_temp[3][0] = 0; T_temp[3][1] = 0; T_temp[3][2] = 0; T_temp[3][3] = 1;
		for (int mk = 0; mk < 16; mk++)
		{
			T_retrun[mk] = T_Temp[mk];
		}

		record_xyz_pos.close();
		return T_retrun;
	}
	void CartesianSpaceTrackUR::kinematic_controller_IBVS()
	{
		Eigen::Matrix3d dR = R_des.transpose() * R;
		Eigen::Quaterniond d_quat(dR);
		Eigen::Matrix<double, 6, 1>  qrd, xx_desd;//dx,
		Eigen::Matrix<double, 8, 1> dx;
		Eigen::Matrix3d inParaM, inParaM_inverse;
		Eigen::Vector3d Pixel_base_Ap1, Pixel_base_Ap2, Pixel_base_Ap3, Pixel_base_Ap4;
		Eigen::Vector3d Pixel_base_Dp1, Pixel_base_Dp2, Pixel_base_Dp3, Pixel_base_Dp4;
		Eigen::Vector3d Image_base_Ap1, Image_base_Ap2, Image_base_Ap3, Image_base_Ap4;
		Eigen::Vector3d Image_base_Dp1, Image_base_Dp2, Image_base_Dp3, Image_base_Dp4;
		double act_p_z_lenth[4], des_p_z_lenth[4];
		inParaM.block<1, 3>(0, 0) << 888.8485, 0.000000, 648.735;
		inParaM.block<1, 3>(1, 0) << 0.000000, 888.8485, 366.436;
		inParaM.block<1, 3>(2, 0) << 0.000000, 0.000000, 1.000000;
		inParaM_inverse = inParaM.inverse();
		//actual
		act_p_z_lenth[0] = im3point_act[2]; act_p_z_lenth[1] = im3point_act[5]; 
		act_p_z_lenth[2] = im3point_act[8]; act_p_z_lenth[3] = im3point_act[11];
		Pixel_base_Ap1 << im3point_act[0], im3point_act[1], im3point_act[2];
		Pixel_base_Ap2 << im3point_act[3], im3point_act[4], im3point_act[5];
		Pixel_base_Ap3 << im3point_act[6], im3point_act[7], im3point_act[8];
		Pixel_base_Ap4 << im3point_act[9], im3point_act[10], im3point_act[11];
		//desired											
		des_p_z_lenth[0] = 0.5; des_p_z_lenth[1] = 0.5;		
		des_p_z_lenth[2] = 0.5; des_p_z_lenth[3] = 0.5;		
		Pixel_base_Dp1 << im3point_des[0], im3point_des[1], 0.5;
		Pixel_base_Dp2 << im3point_des[2], im3point_des[3], 0.5;
		Pixel_base_Dp3 << im3point_des[4], im3point_des[5], 0.5;
		Pixel_base_Dp4 << im3point_des[6], im3point_des[7], 0.5;

		Image_base_Ap1 =/* act_p_z_lenth[0]* inParaM_inverse *  */Pixel_base_Ap1;
		Image_base_Ap2 =/* act_p_z_lenth[1]* inParaM_inverse *  */Pixel_base_Ap2;
		Image_base_Ap3 =/* act_p_z_lenth[2]* inParaM_inverse *  */Pixel_base_Ap3;
		Image_base_Ap4 =/* act_p_z_lenth[3]* inParaM_inverse *  */Pixel_base_Ap4;

		Image_base_Dp1 =/* des_p_z_lenth[0]* inParaM_inverse *  */Pixel_base_Dp1;
		Image_base_Dp2 =/* des_p_z_lenth[1]* inParaM_inverse *  */Pixel_base_Dp2;
		Image_base_Dp3 =/* des_p_z_lenth[2]* inParaM_inverse *  */Pixel_base_Dp3;
		Image_base_Dp4 =/* des_p_z_lenth[3]* inParaM_inverse *  */Pixel_base_Dp4;

		//dx = desd- actual
		dx << -(Image_base_Dp1(0) - Image_base_Ap1(0)), (Image_base_Dp1(1) - Image_base_Ap1(1)),
			  -(Image_base_Dp2(0) - Image_base_Ap2(0)), (Image_base_Dp2(1) - Image_base_Ap2(1)),
			  -(Image_base_Dp3(0) - Image_base_Ap3(0)), (Image_base_Dp3(1) - Image_base_Ap3(1)),
			  -(Image_base_Dp4(0) - Image_base_Ap4(0)), (Image_base_Dp4(1) - Image_base_Ap4(1));
		double dist = (   sqrt(dx(0) * dx(0) + dx(1) * dx(1))
						+ sqrt(dx(2) * dx(2) + dx(3) * dx(3))
						+ sqrt(dx(4) * dx(4) + dx(5) * dx(5))
						+ sqrt(dx(6) * dx(6) + dx(7) * dx(7))
			) / 4;
			
		double kp = 1;
		double kr = 1;
		Eigen::MatrixXd pesudoJacobian = pinv(image_J);//image_J/2+ image_desire_J / 2
		std::cout << "pesudoJacobian J:" << std::endl;
		std::cout << pesudoJacobian << std::endl;
		std::cout << "dx:=" << dx.transpose() << std::endl;
		bool flag = is_singularity();// || imageJ_is_singularity();

		for (int i = 0; i < 8; i++)
		{
			if ((dx(i)) > 200)
			{
					dx(i) = 200;
			}
			if ((dx(i)) < -200)
			{
				dx(i) = -200;
			}
			if (abs(dx(i)) < 10)
			{
				dx(i) = 0;
			}

		}
		//if (flag)
		//{
		//	qrd = -Kp * J.transpose() * dx;//奇异后为啥写这样
		//}
		//else
		if (flag)
		{
			for (int m = 0; m < 6; m++)
			{
				qrd(m)=0;
			}
		}
		else
		{
			if (abs(dist) > 0.0001)
			{
				//qrd = 0.3 * J.inverse() * (image_J.transpose()* image_J).inverse()* image_J.transpose() * dx;//再乘以图像的差,0.001是系数
				qrd = 0.2 * pesudoJacobian * dx / 888.8485;//再乘以图像的差,0.001是系数  0.2
				//qrd =0.3* J_camara.inverse() * pesudoJacobian * dx;//再乘以图像的差,0.001是系数
				//std::cout <<"camare speed:="<< -(image_J.transpose() * image_J).inverse() * image_J.transpose() * dx;
				std::cout << "camare speed:=" << 0.2 * pesudoJacobian * dx / 888.8485 <<std::endl;
				//qrd = J.inverse() * (xx_desd - Kp * dx);//笛卡儿空间位置的微分
			}
			else
			{
				qrd = 0 * pesudoJacobian * dx;//再乘以图像的差,0.001是系数
				std::cout << "camare speed:=" << 0 * pesudoJacobian * dx << std::endl; 
			}
		}

		//qrd(1) = -qrd(1); qrd(2) = -qrd(2); qrd(3) = -qrd(3); 原始状态
		//Eigen::Matrix<double, 6, 1> temp_q(qrd),temp_qrd;
		//for (int i = 0; i < 6; i++)
		//{
		//	if (i < 3)
		//		temp_q[i] = temp_q(i);
		//	else
		//		temp_q[i] = 0.05 * temp_q(i);
		//	if (abs(temp_q[i]) < 0.001)
		//		temp_q[i] = 0;
		//	if (temp_q[i] > 0.1)
		//		temp_q[i] = 0.1;
		//	if (temp_q[i] < -0.1)
		//		temp_q[i] = -0.1;
		//}
		//temp_q[2] = 0; temp_q[3] = 0; temp_q[4] = 0; temp_q[5] = 0;
	 //   //std::cout << temp_q << std::endl;
		//temp_qrd =  J.inverse()* temp_q;
		//std::cout << "speed" << std::endl;
		//for (int i = 0; i < 6; i++)
		//{
		//	//qcd[i] = temp_qrd(i);
		//	std::cout << qcd[i] << std::endl;
		//}
		for (int i = 0; i < 6; i++)
		{
			if (i < 3)
			{
				qcd[i] = qrd(i);
				if (abs(qcd[i]) < 0.001)
					qcd[i] = 0;
			}
			else
				qcd[i] = 1.5*qrd(i);

			if (qcd[i] > 0.1)
				qcd[i] = 0.1;
			if (qcd[i] < -0.1)
				qcd[i] = -0.1;
		}
		//qcd[2] = 0; 
		qcd[3] =0; qcd[4] = 0; qcd[5] = 0;
		std::cout << "speed" << std::endl;
		for (int i = 0; i < 6; i++)
		{
			//qcd[i] = temp_qrd(i);
			std::cout << qcd[i] << std::endl;
		}
		//qcd[0] = qrd(3); qcd[1] = qrd(4); qcd[2] = qrd(5);
		//qcd[3] = qrd(0); qcd[4] = qrd(1); qcd[5] = qrd(2);

	}

	void CartesianSpaceTrackUR::kinematic_controller()
	{
		Eigen::Matrix3d dR = R_des.transpose() * R;
		Eigen::Quaterniond d_quat(dR);
		Eigen::Matrix<double, 6, 1> dx, qrd, xx_desd;


		switch (controller_type)
		{
		case CONTROLLER_TYPE::POSITION_ONLY:
		{
			dx << x(0) - x_des(0), x(1) - x_des(1), x(2) - x_des(2), 0, 0, 0;
			xx_desd << xd_des(0), xd_des(1), xd_des(2), 0, 0, 0;
			break;
		}
		case CONTROLLER_TYPE::FULL_POSE:
		{
			//x:从机器人读关节位置，正解得到的实际位置，x_des由外面传进来的期望位置
			//dx << x(0) - x_des(0), x(1) - x_des(1), x(2) - x_des(2), d_quat.x(), d_quat.y(), d_quat.z(); 原始
			dx << x(0) - x_des(0), x(1) - x_des(1), x(2) - x_des(2), d_quat.x(), d_quat.y(), d_quat.z();//这个是PID的误差
			xx_desd << xd_des(0), xd_des(1), xd_des(2), w_des(0), w_des(1), w_des(2);//期望位置速度和期望姿态速度
			break;
		}
		default:
			dx.setZero();
			xx_desd.setZero();
			break;
		}

		bool flag = is_singularity();
		if (flag)
		{
			qrd = -Kp * J.transpose() * dx;//奇异后为啥写这样
		}
		else
		{
			qrd = J.inverse() * (xx_desd - Kp * dx);//笛卡儿空间位置的微分
		}

		//qrd(1) = -qrd(1); qrd(2) = -qrd(2); qrd(3) = -qrd(3); 原始状态
		qrd(1) = qrd(1); qrd(2) = qrd(2); qrd(3) = qrd(3);
		for (int i = 0; i < 6; i++)
		{
			qcd[i] = qrd(i);
		}
	}

	void CartesianSpaceTrackUR::kinematic_controller_hw()
	{
		Eigen::Matrix3d dR = R_des.transpose() * R;
		Eigen::Quaterniond d_quat(dR);
		Eigen::Matrix<double, 6, 1> dx, qrd, xx_desd;
		//double z_smooth_force[15], x_smooth_force[15], y_smooth_force[15];
		double z_force_error,x_force_error,y_force_error,z_force_temp=0, x_force_temp=0, y_force_temp=0;
		Eigen::Quaterniond qua_R_des(R_des);
		Eigen::Quaterniond qua_R_cur(R);
		//speedL和工具坐标系力，采用tool_coordinate_system_force_,speedJ和基座系力，采用tool_force_
		x_smooth_force[counter] = tool_force_[0];// tool_force_[0];  tool_coordinate_system_force_
		y_smooth_force[counter] = tool_force_[1];//tool_force_[1];
		z_smooth_force[counter] = tool_force_[2];//tool_force_[2];
		std::vector<double> tool_speed = client.getTcpSpeed();
		
		for (int j = 0; j < smooth_num; j++)
		{
			x_force_temp += x_smooth_force[j];
			y_force_temp += y_smooth_force[j];
			z_force_temp += z_smooth_force[j];
		}

		if (x_force_temp/ smooth_num < 4.0 and x_force_temp / smooth_num > -4.0)
			x_force_error = 0 ;
		else if (x_force_temp / smooth_num >= 4.0)
		{
			Kp = 0;
			x_force_error = std::min(90.0, x_force_temp / smooth_num) - tool_force_preset_(0);
		}
		else if (x_force_temp / smooth_num <= -4.0)
			x_force_error = std::max(-60.0, x_force_temp / smooth_num);

		if (y_force_temp / smooth_num < 4.0 and y_force_temp / smooth_num > -4.0)
			y_force_error = 0;
		else if (y_force_temp / smooth_num >= 4.0)
		{
			Kp = 0;
			y_force_error = std::min(90.0, y_force_temp / smooth_num);// -tool_force_preset_(1);
		}
		else if (y_force_temp / smooth_num <= -4.0)
			y_force_error = std::max(-60.0, y_force_temp / smooth_num);

		if (z_force_temp / smooth_num < 3.0 and z_force_temp / smooth_num > -4.0)
			z_force_error = 0;
		else if (z_force_temp / smooth_num >= 3.0)
		{
			Kp = 0;
			z_force_error = std::min(60.0, z_force_temp / smooth_num);// -tool_force_preset_(2);
		}
		else if (z_force_temp / smooth_num <= -4.0)
		//		z_force_error = std::max(-60.0, z_force_temp / smooth_num);
		{
			Kp = 0;
			z_force_error = std::min(60.0, z_force_temp / smooth_num) - tool_force_preset_(2);
		}
		//tool_force_[5] *= 5;
		//if (abs(tool_force_[5]) < 0.1*5)
		//	tool_force_[5] = 0.0;



		Eigen::Vector3d tcs_force_torque_error(0, 0, z_force_error);//这里误差由上面提供的工具坐标系下的力差
		Eigen::Vector3d base_force_torque_error = R * tcs_force_torque_error;
		//std::cout << "qua_R_des: " << qua_R_des.w() << " " << qua_R_des.x() << " " << qua_R_des.y() << " " << qua_R_des.z() << std::endl;
		//std::cout << "qua_R_cur: " << qua_R_cur.w() << " " << qua_R_cur.x() << " " << qua_R_cur.y() << " " << qua_R_cur.z() << std::endl;
		switch (controller_type)
		{
		case CONTROLLER_TYPE::POSITION_ONLY:
		{
			dx << Kp * (x_des(0) - x(0)), Kp* (x_des(1) - x(1)), Kp* (x_des(2) - x(2)), 0, 0, 0;
			xx_desd << xd_des(0), xd_des(1), xd_des(2), 0, 0, 0;
						break;
		}
		case CONTROLLER_TYPE::FULL_POSE:
		{
			//x:从机器人读关节位置，正解得到的实际位置，x_des由外面传进来的期望位置   +impedance_control_pos_inc[2]   vel_calcu[2]
			if (force_controle_mode)
			{ 
			 //tcs f use speedL  base_force_torque_error(0)
			  //base f use speedJ   x_force_error

				dx << Kp *(x(0) - x_des(0))- Spring_k  * x_force_error +0.5* Spring_k *(x_force_error - x_force_error_dot), x(1) - x_des(1)- Spring_k*0 * y_force_error, x(2) - x_des(2)- Spring_k* 0 * z_force_error, d_quat.x(), d_quat.y(), d_quat.z();// 原始
				xx_desd << xd_des(0), xd_des(1), xd_des(2), w_des(0), w_des(1), w_des(2);//期望位置速度和期望姿态速度
				dx(3) = /*-0.1 * tool_force_[3] * (not (bool)Kp) +*/ Kp * 0.01 * 2 * (-qua_R_cur.x() * (qua_R_cur.w() - qua_R_des.w()) + qua_R_cur.w() * (qua_R_cur.x() - qua_R_des.x()) - qua_R_cur.z() * (qua_R_cur.y() - qua_R_des.y()) + qua_R_cur.y() * (qua_R_cur.z() - qua_R_des.z()));
				dx(4) = -0.1 * tool_force_[4] * (not (bool)Kp) + Kp *0.01 * 2 * (-qua_R_cur.y() * (qua_R_cur.w() - qua_R_des.w()) + qua_R_cur.z() * (qua_R_cur.x() - qua_R_des.x()) + qua_R_cur.w() * (qua_R_cur.y() - qua_R_des.y()) - qua_R_cur.x() * (qua_R_cur.z() - qua_R_des.z()));
				dx(5) = -0.1 * tool_force_[5] * (not (bool)Kp) + Kp * 0.01 * 2 * (-qua_R_cur.z() * (qua_R_cur.w() - qua_R_des.w()) - qua_R_cur.y() * (qua_R_cur.x() - qua_R_des.x()) + qua_R_cur.x() * (qua_R_cur.y() - qua_R_des.y()) + qua_R_cur.w() * (qua_R_cur.z() - qua_R_des.z()));
			//用于调整姿态-0.1 * tool_force_[4] * (not (bool)Kp)
			}
			else
			{
				dx << x(0) - x_des(0) - 0 * x_force_error, x(1) - x_des(1) - 0 * y_force_error, x(2) - x_des(2) - 0 * vel_calcu[2], d_quat.x(), d_quat.y(), d_quat.z();// 原始
				//dx << Kp * (x_des(0) - x(0))+Kd* tatal_deviation[0], Kp* (x_des(1) - x(1)) + Kd * tatal_deviation[1], Kp* (x_des(2) - x(2)) + Kd * tatal_deviation[2], d_quat.x(), d_quat.y(), d_quat.z();//这个是PID的误差
				//dx << Kp*(x_des(0)- x(0)), Kp* (x_des(1)- x(1)), Kp* (x_des(2)- x(2)), d_quat.x(), d_quat.y(), d_quat.z();//这个是PID的误差
				//dx << Kp*(x_des(0)- x(0))+Kd* (x_des(0) - x(0) - last_deviation[0]), Kp* (x_des(1)- x(1)) + Kd * (x_des(1) - x(1) - last_deviation[1]), Kp* (x_des(2)- x(2)) + Kd * (x_des(2) - x(2) - last_deviation[2]), d_quat.x(), d_quat.y(), d_quat.z();//这个是PID的误差
				xx_desd << xd_des(0), xd_des(1), xd_des(2), w_des(0), w_des(1), w_des(2);//期望位置速度和期望姿态速度
				//dx = Kp * dx+Ki+Kd*(dx-last_deviation);  p 0.5
				dx(3) = 0.5 * 2 * (-qua_R_cur.x() * (qua_R_cur.w() - qua_R_des.w()) + qua_R_cur.w() * (qua_R_cur.x() - qua_R_des.x()) - qua_R_cur.z() * (qua_R_cur.y() - qua_R_des.y()) + qua_R_cur.y() * (qua_R_cur.z() - qua_R_des.z()));
				dx(4) = 0.5 * 2 * (-qua_R_cur.y() * (qua_R_cur.w() - qua_R_des.w()) + qua_R_cur.z() * (qua_R_cur.x() - qua_R_des.x()) + qua_R_cur.w() * (qua_R_cur.y() - qua_R_des.y()) - qua_R_cur.x() * (qua_R_cur.z() - qua_R_des.z()));
				dx(5) = 0.5 * 2 * (-qua_R_cur.z() * (qua_R_cur.w() - qua_R_des.w()) - qua_R_cur.y() * (qua_R_cur.x() - qua_R_des.x()) + qua_R_cur.x() * (qua_R_cur.y() - qua_R_des.y()) + qua_R_cur.w() * (qua_R_cur.z() - qua_R_des.z()));
				//std::cout << "dx:= " << dx << std::endl;
			}
			break;
		}
		default:
			dx.setZero();
			xx_desd.setZero();
			break;
		}
		for (int i = 0; i < 6; i++)
		{
			last_deviation[i] = dx(i);
			tatal_deviation[i] += dx(i);
			//std::cout << "last_deviation:= " << last_deviation[i] << std::endl;
			//std::cout << "tatal_deviation:= " << tatal_deviation[i] << std::endl;
		}
		x_force_error_dot = x_force_error;// base_force_torque_error(0);

		bool flag = is_singularity();
		if (flag)
		{
			qrd = -J.transpose() * dx;//奇异后为啥写这样
		}
		else
		{
			qrd = J.inverse() * (xx_desd - dx );//笛卡儿空间第一个D微分项，第二个p*
			//if ( abs((xx_desd(0) - dx(0)))>0.05)
			// 
			//tcs f use speedL  base_force_torque_error(0)
			  //base f use speedJ   x_force_error
			//qrd = 1* (xx_desd - dx);
			//if (qrd(0) > 0.05)
			//	qrd(0) = 0.05;
			//if (qrd(0) < -0.05)
			//	qrd(0) = -0.05;
		}

		//qrd(1) = -qrd(1); qrd(2) = -qrd(2); qrd(3) = -qrd(3); 原始状态
		qrd(1) = qrd(1); qrd(2) = qrd(2); qrd(3) = qrd(3);
		for (int i = 0; i < 6; i++)
		{
			qcd[i] = qrd(i);
		}
		counter++;
		
		if (counter == smooth_num)
		{
			counter = 0;
			//std::cout << "--------------------------------------------------------------------" << std::endl;
		}
			
	}


	void CartesianSpaceTrackUR::kinematic_jointPcontroller()
	{
		Eigen::Matrix<double, 6, 1> q_deviation, qrd, xx_desd;

		Eigen::Quaterniond qua_R_des(R_des);
		Eigen::Quaterniond qua_R_cur(R);
		double q_current[6];
		Eigen::Matrix<double, 6, 1> qv_predict;
		std::vector<double> q_ = client.getPosition();
		for (int mm = 0; mm < 6; mm++)
		{
			q_current[mm] = q_[mm];
			//std::cout << "curren q:= " << q_current[mm];
		}
		switch (controller_type)
		{
		case CONTROLLER_TYPE::POSITION_ONLY:
		{
			q_deviation << Q_des[0] - q_current[0], Q_des[1] - q_current[1], Q_des[2] - q_current[2], 0, 0, 0;
			xx_desd << xd_des(0), xd_des(1), xd_des(2), 0, 0, 0;
			break;
		}
		case CONTROLLER_TYPE::FULL_POSE:
		{
			switch(joint_admitance_status[0])
			{
			case 0:
			if (abs(qd(0)) < 0.01 and joint_admitance_swith[0])
			{
				joint_admitance_swith[0] = 0;
				actual_current_temp_[0] = actual_current_[0];
				joint_admitance_status[0] = 1;
			}
			break;
			case 1:

				if (abs(actual_current_[0] - actual_current_temp_[0]) > 0.3)
				{	//if (abs(actual_current_[0] - control_current_[0])>0.7)
					joint_admitance_swith[0] = 1;
					joint_admitance_status[0] = 2;
				}
			break;
			case 2:
				if (abs(qd(0)) > 0.02)
					joint_admitance_status[0] =0;
			break;

			}
		

			switch (joint_admitance_status[1])
			{
			case 0:
				if (abs(qd(1)) < 0.01 and joint_admitance_swith[1])
				{
					joint_admitance_swith[1] = 0;
					actual_current_temp_[1] = actual_current_[1];
					joint_admitance_status[1] = 1;
				}
				break;
			case 1:

				if (abs(actual_current_[1] - actual_current_temp_[1]) > 0.3)
				{	//if (abs(actual_current_[0] - control_current_[0])>0.7)
					joint_admitance_swith[1] = 1;
					joint_admitance_status[1] = 2;
				}
				break;
			case 2:
				if (abs(qd(1)) > 0.02)
					joint_admitance_status[1] = 0;
				break;

			}
			//qv_predict = J.inverse() * res_predict;
			//q:从机器人读关节位置，正解得到的实际位置，Q_des由外面传进来的期望位置   关节空间阻抗控制时0改为0.05
			//q_deviation << jKp[0]*(Q_des[0] - q_current[0]), jKp[1] *( Q_des[1] - q_current[1]), jKp[2] *( Q_des[2] - q_current[2]), jKp[3] * (Q_des[3] - q_current[3]), jKp[4] * (Q_des[4] - q_current[4]), jKp[5] * (Q_des[5] - q_current[5]);
			q_deviation << jKp[0] * (Q_des[0] - q_current[0]) + jKd[0] * (Q_des[0] - q_current[0] - jLastErr[0]) - 0 * joint_admitance_swith[0] * (actual_current_[0] - actual_current_temp_[0]),
						   jKp[1] * (Q_des[1] - q_current[1]) + jKd[1] * (Q_des[1] - q_current[1] - jLastErr[1]) - 0 * joint_admitance_swith[1] * (actual_current_[1] - actual_current_temp_[1]),
						   jKp[2] * (Q_des[2] - q_current[2]) + jKd[2] * (Q_des[2] - q_current[2] - jLastErr[2]) - 0 * joint_admitance_swith[2] * (actual_current_[2] - 0.5 - actual_current_temp_[2]),
				           jKp[3] * (Q_des[3] - q_current[3]) + jKd[3] * (Q_des[3] - q_current[3] - jLastErr[3]) - 0 * joint_admitance_swith[3] * (actual_current_[3] - 0.15 - actual_current_temp_[3]),
				           jKp[4] * (Q_des[4] - q_current[4]) + jKd[4] * (Q_des[4] - q_current[4] - jLastErr[4]) - 0 * joint_admitance_swith[4] * (actual_current_[4] - 0.15 - actual_current_temp_[4]),
				           jKp[5] * (Q_des[5] - q_current[5]) + jKd[5] * (Q_des[5] - q_current[5] - jLastErr[5]) - 0 * joint_admitance_swith[5] * (actual_current_[5] - 0.15 - actual_current_temp_[5]);

			break;
		}
		default:
			q_deviation.setZero();
			xx_desd.setZero();
			break;
		}
		//for (int i = 0; i < 6; i++)
		//{
		//	//last_deviation[i] = q_deviation(i);
		//	//tatal_deviation[i] += q_deviation(i);
		//	std::cout << "q_deviation:= " << std::setw(12)<< q_deviation[i] ;
		//	//std::cout << "q_act:= " << q(i) << std::endl;
		//	//std::cout << "Q_des:= " << Q_des[i] << std::endl;
		//	//std::cout << "tatal_deviation:= " << tatal_deviation[i] << std::endl;
		//}
		//std::cout<< std::endl;
		//for (int i = 0; i < 6; i++)
		//{
		//	//last_deviation[i] = q_deviation(i);
		//	//tatal_deviation[i] += q_deviation(i);
		//	//std::cout << "q_deviation:= " << q_deviation[i] << std::endl;
		//	std::cout << "q_act:= " << std::setw(12) << q(i) << std::endl;
		//	//std::cout << "Q_des:= " << Q_des[i] << std::endl;
		//	//std::cout << "tatal_deviation:= " << tatal_deviation[i] << std::endl;
		//}
		//std::cout << std::endl;
		//for (int i = 0; i < 6; i++)
		//{
		//	//last_deviation[i] = q_deviation(i);
		//	//tatal_deviation[i] += q_deviation(i);
		//	//std::cout << "q_deviation:= " << q_deviation[i] << std::endl;
		//	//std::cout << "q_act:= " << q(i) << std::endl;
		//	std::cout << "Q_des:= " << std::setw(12) << Q_des[i] << std::endl;
		//	//std::cout << "tatal_deviation:= " << tatal_deviation[i] << std::endl;
		//}
		//std::cout << std::endl;
		//bool flag = is_singularity();
		//if (flag)
		//{
		//	qrd = J.transpose() * dx;//奇异后为啥写这样
		//}
		//else
		//{
		//	qrd = J.inverse() * (xx_desd + dx);//笛卡儿空间第一个D微分项，第二个p*
		//}

		
		for (int i = 0; i < 6; i++)
		{
			qrd(i) = q_deviation[i];
			qcd[i] = qrd(i);
			jLastErr[i] = Q_des[i] - q_current[i];
		}
	}

	bool CartesianSpaceTrackUR::is_singularity()
	{
		// cal the minimal eigenvalue of the jacobian matrix
		Eigen::EigenSolver<Eigen::Matrix<double, 6, 6>> es(J.transpose() * J);
		Eigen::Matrix<double, 6, 6> D = es.pseudoEigenvalueMatrix();//特征向量矩阵

		double min_eig = D(0, 0);
		for (int i = 1; i < 6; i++)
		{
			if (min_eig > D(i, i))
				min_eig = D(i, i);
		}
		if (min_eig <= 1e-4) // close to singularity
			return true;
		else
			return false;
	}
	bool CartesianSpaceTrackUR::imageJ_is_singularity()
	{
		// cal the minimal eigenvalue of the jacobian matrix
		Eigen::EigenSolver<Eigen::Matrix<double, 6, 6>> es(image_J.transpose() * image_J);
		Eigen::Matrix<double, 6, 6> D = es.pseudoEigenvalueMatrix();//特征向量矩阵

		double min_eig = D(0, 0);
		for (int i = 1; i < 6; i++)
		{
			if (min_eig > D(i, i))
				min_eig = D(i, i);
		}
		if (min_eig <= 1e-4) // close to singularity
			return true;
		else
			return false;
	}
	void CartesianSpaceTrackUR::solution_choose_sigle_axis_diff(int isolutions, double df_q_sols[], double q_actual_pos[], double q_choosed[])
	{
		//std::cout << "This is test function!			" << std::endl;

		double df_total_diff[8] = { 0.0 };
		double df_pos_diff[8] = { 0.0 };
		double df_rotate_diff[8] = { 0.0 };
		double df_min_diff = 0.5;//2度对应的弧度0.0349，1，2关节最大速度120度每秒，3，4，5，6最大速度180度每秒
		double df_temp,dt_temp_act_pos;
		int i_order = 0;
		bool b_choosed = false;

		std::cout << "The total sulutions is " << isolutions << std::endl;
		if (0 == isolutions)
		{
			for (int mk = 0; mk < 6; mk++)
				q_choosed[mk] = q_actual_pos[mk];
		}
		else
		{
			for (int i = 0; i < isolutions; i++)
			{
				for (int j = 0; j < 6; j++)
				{
					dt_temp_act_pos = q_actual_pos[j];
					if (j < 3)
					{
						if (0 == j)
						{
							df_temp = df_q_sols[i * 6 + j];
							if (std::min(abs(df_temp - dt_temp_act_pos), abs(df_temp - dt_temp_act_pos - 2 * M_PI)) > df_min_diff)
								//if (abs(df_temp - dt_temp_act_pos) > df_min_diff and abs(df_temp - dt_temp_act_pos - 2 * M_PI) > df_min_diff)
							{
								std::cout << "The choose process is too big" << abs(df_temp - dt_temp_act_pos)
									<< "顺序是" << i << "关节是" << j << "df_temp" << df_temp << "q_sols_" << df_q_sols[i * 6 + j] << "q_actual_pos" << dt_temp_act_pos << std::endl;
								break;
							}
						}
						else
						{
							if (df_q_sols[i * 6 + j] <= M_PI)
								df_temp = df_q_sols[i * 6 + j];
							else
								df_temp = df_q_sols[i * 6 + j] - 2 * M_PI;
							if (abs(df_temp - dt_temp_act_pos) > df_min_diff)
							{
								std::cout << "The choose process is too big" << abs(df_temp - dt_temp_act_pos)
									<< "顺序是" << i << "关节是" << j << "df_temp" << df_temp << "q_sols_" << df_q_sols[i * 6 + j] << "q_actual_pos" << dt_temp_act_pos << std::endl;
								break;
							}
						}

					}
					else
					{
						df_temp = df_q_sols[i * 6 + j];
						if (std::min(abs(df_temp - dt_temp_act_pos), abs(df_temp - dt_temp_act_pos - 2 * M_PI))> df_min_diff)
						//if (abs(df_temp - dt_temp_act_pos) > df_min_diff and abs(df_temp - dt_temp_act_pos - 2 * M_PI) > df_min_diff)
						{
							std::cout << "The choose process is too big" << abs(df_temp - dt_temp_act_pos)
								<< "顺序是" << i << "关节是" << j << "df_temp" << df_temp << "q_sols_" << df_q_sols[i * 6 + j] << "q_actual_pos" << dt_temp_act_pos << std::endl;
							break;
						}
					}
						
					if (5 == j)
					{
						i_order = i;
						b_choosed = true;
						//std::cout << "###############The choosed i_order=" << i_order << std::endl;
						break;
					}
				}
				if (b_choosed)
					break;
			}
			//std::cout <<"The choosed i_order="<< i_order << std::endl;
			if (b_choosed)
			{
				for (int mn = 0; mn < 6; mn++)
				{
					q_choosed[mn] = df_q_sols[i_order * 6 + mn];//i_order
					//std::cout << "q_choosed[mn] " << q_choosed[mn];
				}
				//std::cout << std::endl;
			}
			else
			{
				for (int mn = 0; mn < 6; mn++)
				{
					q_choosed[mn] = q_actual_pos[0 * 6 + mn];//i_order
					//std::cout << "q_choosed[mn] " << q_choosed[mn];
				}
			}

		}
	}
	int  CartesianSpaceTrackUR::deleate_cannot_reach(int isolutions, double df_q_source_sols[], double df_q_result_sols[])
	{
		int m = 0;
		int i_num_of_solutions = isolutions;
		if (0 == isolutions)
		{
			for (int i = 0; i < 48; i++)
				df_q_result_sols[i] = df_q_source_sols[i];
			return 0;
		}

		//用于转存原始数据
		//用于去除碰地的情况（2关节0~pi,2关节+3关节>2PI）
		//2关节0~pi
		for (int i = 0; i < isolutions; i++)
		{
			if (df_q_source_sols[i * 6 + 1] > 0.1 & df_q_source_sols[i * 6 + 1] < M_PI)
			{
				i_num_of_solutions--;
				continue;
			}
			
			for (int j = 0; j < 6; j++)
				df_q_result_sols[j+m*6] = df_q_source_sols[j + i * 6];
			m++;
		}
		return m;
	}

	void CartesianSpaceTrackUR::solution_choose_total_diff(int isolutions, double df_q_sols[], double q_actual_pos[],  double q_choosed[])
	{
		//std::cout << "This is test function!			" << std::endl;

		double df_total_diff[8] = { 0.0 };
		double df_pos_diff[8] = { 0.0 };
		double df_rotate_diff[8] = { 0.0 };
		double df_min_diff = 100.0;
		double df_temp=0.0;
		int i_order = 0;

		if (0 == isolutions)
		{
			for (int mk = 0; mk < 6; mk++)
				q_choosed[mk] = q_actual_pos[mk];
		}
		else
		{
			//取舍判断
			for (int i = 0; i < isolutions; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					if (0==j)
						df_pos_diff[i] += abs(df_q_sols[i * 6 + j] - q_actual_pos[j]) <= abs(df_q_sols[i * 6 + j] - 2 * M_PI - q_actual_pos[j]) ? abs(df_q_sols[i * 6 + j] - q_actual_pos[j]) : abs(df_q_sols[i * 6 + j] - 2 * M_PI - q_actual_pos[j]);
					else
					{
						if (df_q_sols[i * 6 + j] <= M_PI)
							df_temp = df_q_sols[i * 6 + j];
						else
							df_temp = df_q_sols[i * 6 + j] - 2 * M_PI;
						df_pos_diff[i] += abs(df_temp - q_actual_pos[j]);
					}
				}
				for (int m = 3; m < 6; m++)
				{
					df_temp = df_q_sols[i * 6 + m];
					
					df_rotate_diff[i] += abs(df_q_sols[i * 6 + m] - q_actual_pos[m])<= abs(df_q_sols[i * 6 + m] - 2 * M_PI- q_actual_pos[m])? abs(df_q_sols[i * 6 + m] - q_actual_pos[m]) : abs(df_q_sols[i * 6 + m] - 2 * M_PI - q_actual_pos[m]);
				}

				df_total_diff[i] =  df_pos_diff[i] + df_rotate_diff[i];

			}
			for (int k = 0; k < isolutions; k++)
			{
				//std::cout <<"The total error of every solution =" << df_total_diff[k] << std::endl;
				if (df_total_diff[k] < df_min_diff)
				{
					i_order = k;
					df_min_diff = df_total_diff[k];
				}
			}
			//std::cout <<"The choosed i_order=" <<i_order << std::endl;

			for (int mn = 0; mn < 6; mn++)
			{
				q_choosed[mn] = df_q_sols[i_order * 6 + mn];
				//std::cout <<"The choosed qorder=" << q_choosed[mn] << std::endl;
			}
				

		}
	}



	Eigen::VectorXd CartesianSpaceTrackUR::get_tool_pose()
	{
		Eigen::VectorXd pose;
		pose.resize(6);
		Vec theta = rot2vec(R);
		pose << theta(0), theta(1), theta(2), x(0), x(1), x(2);
		return pose;
	}

	std::vector<double> CartesianSpaceTrackUR::get_tcp_force_tool(std::vector<double> tool_force)
	{
		Eigen::VectorXd force_torque;
		Eigen::Vector3d force_B, torque_B;
		force_torque.resize(6); force_B.resize(3); torque_B.resize(3);
		force_torque << tool_force[0], tool_force[1], tool_force[2], tool_force[3], tool_force[4], tool_force[5];
		force_B[0] = force_torque[0];force_B[1] = force_torque[1];force_B[2] = force_torque[2];
		torque_B[0] = force_torque[3]; torque_B[1] = force_torque[4]; torque_B[2] = force_torque[5];
		std::vector<double> tcp_force_torque;
		tcp_force_torque.resize(6);
		Eigen::Vector3d tcp_force, tcp_torque;
		tcp_force = R.inverse() * force_B;
		tcp_torque = R.inverse() * torque_B;
		for (int i = 0; i < 3; i++)
		{
			tcp_force_torque[i] = tcp_force(i);
			tcp_force_torque[i+3] = tcp_torque(i);
		}
		return tcp_force_torque;
	}

	Eigen::MatrixXd CartesianSpaceTrackUR::pinv(Eigen::MatrixXd  A)
	{
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
		double  pinvtoler = 1.e-8; //tolerance
		int row = A.rows();
		int col = A.cols();
		int k = std::min(row, col);
		Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col, row);
		Eigen::MatrixXd singularValues_inv = svd.singularValues();//奇异值
		Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
		for (long i = 0; i < k; ++i) {
			if (singularValues_inv(i) > pinvtoler)
				singularValues_inv(i) = 1.0 / singularValues_inv(i);
			else singularValues_inv(i) = 0;
		}
		for (long i = 0; i < k; ++i)
		{
			singularValues_inv_mat(i, i) = singularValues_inv(i);
		}
		X = (svd.matrixV()) * (singularValues_inv_mat) * (svd.matrixU().transpose());

		return X;

	}

}