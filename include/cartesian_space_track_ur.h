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
#pragma once

#include<abstract_controller_ur.h>
#include<Eigen/Dense>
#define _USE_MATH_DEFINES
#include<math.h>
#include<dll_export.h>
#include<vector>
#include<deque>
const int smooth_num = 1;
namespace Robot {
	class ROBOTIC_GRASP_DLL CartesianSpaceTrackUR : public AbstractControllerUR {
	public:
		const double ZERO_THRESH = 0.00000001;
		int SIGN(double x) {
			return (x > 0) - (x < 0);
		}
		const double PI = M_PI;
		enum CONTROLLER_TYPE
		{
			POSITION_ONLY,
			FULL_POSE
		};
	public:
		CartesianSpaceTrackUR(ur_sdk::client& client_);
		~CartesianSpaceTrackUR();
		virtual void update(const boost::posix_time::ptime& time, const boost::posix_time::time_duration& period);
		void update_p(const boost::posix_time::ptime& time, const boost::posix_time::time_duration& period);
		// cal the position and orientation of the end frame
		void cal_pose();
		void fkRobot(const double* q, Eigen::Matrix<double, 4, 4>& T, Eigen::Matrix<double, 3, 3>& R, Eigen::Vector3d& x);
		//cal the 6 joint angle (radian)
		int inverse(const double* T, double* q_sols, double q6_des);
		double* joint_traj(double q_start[6], double q_end[6], double df_increas);
		double* joint_space_3traj(double q_start[6], double q_bypass[6], double q_end[6], double* df_speed, double* df_acc, double df_increas, double df_motion_finish_time);
		double* joint_space_3traj(double q_start[6], double q_end[6], double* df_speed, double* df_acc, double df_increas, double df_motion_finish_time);
		double* joint_space_parabola_fit_line_traj(double q_start[6], double q_end[6], double df_increas, double df_motion_finish_time,double acc);
		//joint space generate point sequence
		bool joint_gen_point_sequence(Eigen::Matrix<double, 4, 4>& Matrix_start_point, Eigen::Matrix<double, 4, 4>& Matrix_bypass_point, Eigen::Matrix<double, 4, 4>& Matrix_end_point, double* q_solution_sequence, double* q_speed_sequence, double* q_acc_sequence, int i_increase);
		bool joint_gen_point_sequence(Eigen::Matrix<double, 4, 4>& Matrix_start_point, Eigen::Matrix<double, 4, 4>& Matrix_end_point, double* q_solution_sequence, double* q_speed_sequence, double* q_acc_sequence, int i_increase);
		//Cartesian coordinates trajectory
		bool Ctraj_gen_point_sequence(Eigen::Matrix<double, 4, 4>& Matrix_start_point, Eigen::Matrix<double, 4, 4>& Matrix_end_point, double* q_solution_sequence, int i_increas);
		bool Ctraj_gen_step_point_sequence(Eigen::Matrix<double, 4, 4>& Matrix_start_point, Eigen::Matrix<double, 4, 4>& Matrix_end_point, double* q_solution_sequence,int inc, double i_increase,double time_increases);
		double* Ctraj(Eigen::Matrix<double, 4, 4>& Matrix_start_point, Eigen::Matrix<double, 4, 4>& Matrix_end_point, double df_increas);
		double* Ctraj_cubic_poly(Eigen::Matrix<double, 4, 4>& Matrix_start_point, Eigen::Matrix<double, 4, 4>& Matrix_end_point, double df_increas, double df_motion_finish_time);
		void specify_qcd(double scale_); // this function should be called after update
		void specify_zero_qcd();//set all joint to 0 speed;
		// set the kinematic parameters of the ur robot
		void set_kinematic_param(double d1_, double L2_, double L3_, double d4_, double d5_, double d6_);
		// set the desired orientation and position of the end frame
		void set_desired_position(const Eigen::Vector3d& x_, const Eigen::Vector3d& xd_, const Eigen::Vector3d& xdd_);
		void set_Qdesired_position( double * qdes);
		void set_desired_orientation(const Eigen::Matrix3d& R_, const Eigen::Vector3d& w_, const Eigen::Vector3d& wd_);
		void ik_solution_choose(int isolutions, double df_q_sols[], double q_actual_pos[], double q_choosed[]);
		void solution_choose_sigle_axis_diff(int isolutions, double df_q_sols[], double q_actual_pos[], double q_choosed[]);
		void solution_choose_total_diff(int isolutions, double df_q_sols[], double q_actual_pos[],  double q_choosed[]);
		int deleate_cannot_reach(int isolutions, double df_q_source_sols[], double df_q_result_sols[]);
		bool ik_with_q(Eigen::Matrix<double, 4, 4>& Matrix_end_point, double* desire_q);
		bool ik_with_start_pos(Eigen::Matrix<double, 4, 4>& Matrix_start_point, Eigen::Matrix<double, 4, 4>& Matrix_end_point, double* desire_q);
		void slerp(double starting[4], double ending[4], double result[4], double t);
		Eigen::VectorXd get_tool_pose();
		Eigen::MatrixXd pinv(Eigen::MatrixXd  A);
		std::vector<double> get_tcp_force_tool(std::vector<double> tool_force);
		Eigen::Vector3d x;
		Eigen::Matrix<double, 4, 4> Ttool_to_base;
		std::vector<double> pos_pre;//上一次的位置
		double q1, q2, q3, q4, q5, q6;
		std::vector <double> q_position;
		double Kp,Ki,Kd,Spring_k, impedance_control_M[6], impedance_control_D[6], impedance_control_K[6], impedance_control_acc[6], impedance_control_pos_inc[6];
		double jKp[6],jKd[6],jLastErr[6];
		double vel_calcu[3];
		Eigen::Matrix<double, 6, 1> res_predict;
		bool bfirstrun = false;
		CONTROLLER_TYPE controller_type;
		double df_rob_loop_time ;
		std::vector<double> tool_force_;
		std::vector<double> tool_coordinate_system_force_;
		std::vector<double> tcp_speed_;
		std::vector<double> actual_current_;
		std::vector<double> control_current_;
		std::vector<double> actual_current_temp_;
		Eigen::Vector3d tool_force_preset_;
		bool force_controle_mode;
		bool useIBVS;
		bool joint_admitance_swith[6];
		int joint_admitance_status[6];
		double im3point_des[8];//3点图像期望像素位置
		double im3point_act[12];//3点图像实际像素位置
	private:
		//pid last deviation
		double last_deviation[6],last_last_deviation[6];
		double tatal_deviation[6];

		double x_force_error_dot;
		// position and velocity in cartesian space
		//Eigen::Vector3d x;
		Eigen::Vector3d xd;
		// orientation of end frame
		Eigen::Matrix3d R;
		Eigen::Vector3d w;
		// desired position and orientation of the end frame 
		Eigen::Matrix3d R_des;
		Eigen::Vector3d w_des;
		Eigen::Vector3d wd_des;
		Eigen::Vector3d x_des;
		Eigen::Vector3d xd_des;
		Eigen::Vector3d xdd_des;
		// desired q position of joint
		double Q_des[6];
		//double im3point_des[6];//3点图像期望像素位置
		//double im3point_act[6];//3点图像实际像素位置
		// Jacobian matrix
		Eigen::Matrix<double, 6, 6> J,J_camara;
		Eigen::Matrix<double, 8, 6> image_J,image_desire_J;
		double q_maxspeed[6],impedance_control_max_pos_inc[6];
		//double q1, q2, q3, q4, q5, q6;
		// kinematic parameters
		double d1, L2, L3, d4, d5, d6;
		// cal the Jacobian matrix
		void cal_J();
		Eigen::Matrix<double, 8, 6> cal_image_J(Eigen::Vector3d point1, Eigen::Vector3d point2, Eigen::Vector3d point3, Eigen::Vector3d point4);
		int counter;
		double z_smooth_force[smooth_num], x_smooth_force[smooth_num], y_smooth_force[smooth_num];
		/*
		// cal the position and orientation of the end frame
		void cal_pose();
		//cal the 6 joint angle (radian)
		int inverse(const double* T, double* q_sols, double q6_des);
		//Cartesian coordinates trajectory
		void Ctraj(double T1[4][4], double T2[4][4], int t);
		*/

		// 
		void kinematic_controller();
		void kinematic_controller_hw();
		void kinematic_controller_IBVS();
		void kinematic_jointPcontroller();
		// singularity monitering
		bool is_singularity();
		bool imageJ_is_singularity();
	};

}