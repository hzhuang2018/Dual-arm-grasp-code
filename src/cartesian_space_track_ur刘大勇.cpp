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
#include<Eigen/Eigenvalues>

namespace Robot {

	CartesianSpaceTrackUR::CartesianSpaceTrackUR(ur_sdk::client& client_) : AbstractControllerUR(client_)
	{
		Kp = 1.0;
		/*
		d1 = 0.1273;
		L2 = 0.6127;
		L3 = 0.5716;
		d4 = -0.1639;
		d5 = 0.1157;
		d6 = 0.0819;
		*/
		d1 = 0.1273;
		L2 = -0.6127;
		L3 = -0.5716;
		d4 = 0.1639;
		d5 = 0.1157;
		d6 = 0.0819;
		controller_type = CONTROLLER_TYPE::POSITION_ONLY;

		qcd.resize(6);
		qcd_pre.resize(6);
		qcd[0] = 0; qcd[1] = 0; qcd[2] = 0;
		qcd[3] = 0; qcd[4] = 0; qcd[5] = 0;
		qcd_pre[0] = 0; qcd_pre[1] = 0; qcd_pre[2] = 0;
		qcd_pre[3] = 0; qcd_pre[4] = 0; qcd_pre[5] = 0;
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
	void CartesianSpaceTrackUR::update(const boost::posix_time::ptime& time, const boost::posix_time::time_duration& period)
	{
		elapsed_time = time - start_time;
		// update the joint-space state
		std::vector<double> q_ = client.getPosition();
		std::vector<double> qd_ = client.getSpeed();
		std::vector<double> qc_ = client.getTargetPosition();
		// transformation of the joint position to fit the self-defined cordinates
		/*
		q(0) = q_[0]; q(1) = -q_[1]; q(2) = -q_[2];
		q(3) = -q_[3] - M_PI / 2; q(4) = q_[4] - M_PI; q(5) = q_[5];
		q1 = q(0); q2 = q(1); q3 = q(2);
		q4 = q(3); q5 = q(4); q6 = q(5);

		qd(0) = qd_[0]; qd(1) = -qd_[1]; qd(2) = -qd_[2];
		qd(3) = qd_[3]; qd(4) = qd_[4]; qd(5) = qd[5];
		*/
		// 20210923
		for (int i = 0; i < 6; i++)
		{
			q(i) = q_[i];
			qd(i) = qd_[i];
		}
		q1 = q(0); q2 = q(1); q3 = q(2);
		q4 = q(3); q5 = q(4); q6 = q(5);
		//////////////

		cal_pose();
		cal_J();
		Eigen::Matrix<double, 6, 1> tmp = J * qd;
		xd = tmp.segment<3>(0);
		w = tmp.segment<3>(3);
		// store the previous qcd
		for (int i = 0;i < 6;i++)
			qcd_pre[i] = qcd[i];
		// cal new qcd
		kinematic_controller();
	}
	void CartesianSpaceTrackUR::specify_qcd(double scale_)
	{
		std::vector<double> qcd_cur;
		qcd_cur.resize(6);
		for (int i = 0;i < 6;i++)
		{
			qcd_cur[i] = qcd_pre[i] + scale_ * (qcd[i] - qcd_pre[i]);
		}
		client.setSpeed(qcd_cur, qdd_target);
	}
	void CartesianSpaceTrackUR::cal_J()
	{
		//J.block<1, 6>(0, 0) << L3 * cos(q1) * sin(q2) * sin(q3) - L2 * cos(q1) * cos(q2) - d6 * cos(q5) * sin(q1) - L3 * cos(q1) * cos(q2) * cos(q3) - d4 * sin(q1) - d5 * cos(q1) * cos(q2) * cos(q3) * cos(q4) + d5 * cos(q1) * cos(q2) * sin(q3) * sin(q4) + d5 * cos(q1) * cos(q3) * sin(q2) * sin(q4) + d5 * cos(q1) * cos(q4) * sin(q2) * sin(q3) - d6 * cos(q1) * cos(q2) * cos(q3) * sin(q4) * sin(q5) - d6 * cos(q1) * cos(q2) * cos(q4) * sin(q3) * sin(q5) - d6 * cos(q1) * cos(q3) * cos(q4) * sin(q2) * sin(q5) + d6 * cos(q1) * sin(q2) * sin(q3) * sin(q4) * sin(q5), sin(q1)* (L3 * sin(q2 + q3) - (d6 * sin(q2 + q3 + q4 + q5)) / 2 + L2 * sin(q2) + (d6 * sin(q2 + q3 + q4 - q5)) / 2 + d5 * sin(q2 + q3 + q4)), sin(q1)* (L3 * sin(q2 + q3) - (d6 * sin(q2 + q3 + q4 + q5)) / 2 + (d6 * sin(q2 + q3 + q4 - q5)) / 2 + d5 * sin(q2 + q3 + q4)), sin(q1)* ((d6 * sin(q2 + q3 + q4 - q5)) / 2 - (d6 * sin(q2 + q3 + q4 + q5)) / 2 + d5 * sin(q2 + q3 + q4)), d6* cos(q5)* sin(q1)* sin(q2)* sin(q3)* sin(q4) - d6 * cos(q2) * cos(q3) * cos(q5) * sin(q1) * sin(q4) - d6 * cos(q2) * cos(q4) * cos(q5) * sin(q1) * sin(q3) - d6 * cos(q3) * cos(q4) * cos(q5) * sin(q1) * sin(q2) - d6 * cos(q1) * sin(q5), 0;
		//J.block<1, 6>(1, 0) << d4 * cos(q1) - L2 * cos(q2) * sin(q1) + d6 * cos(q1) * cos(q5) - L3 * cos(q2) * cos(q3) * sin(q1) + L3 * sin(q1) * sin(q2) * sin(q3) - d5 * cos(q2) * cos(q3) * cos(q4) * sin(q1) + d5 * cos(q2) * sin(q1) * sin(q3) * sin(q4) + d5 * cos(q3) * sin(q1) * sin(q2) * sin(q4) + d5 * cos(q4) * sin(q1) * sin(q2) * sin(q3) - d6 * cos(q2) * cos(q3) * sin(q1) * sin(q4) * sin(q5) - d6 * cos(q2) * cos(q4) * sin(q1) * sin(q3) * sin(q5) - d6 * cos(q3) * cos(q4) * sin(q1) * sin(q2) * sin(q5) + d6 * sin(q1) * sin(q2) * sin(q3) * sin(q4) * sin(q5), -cos(q1) * (L3 * sin(q2 + q3) - (d6 * sin(q2 + q3 + q4 + q5)) / 2 + L2 * sin(q2) + (d6 * sin(q2 + q3 + q4 - q5)) / 2 + d5 * sin(q2 + q3 + q4)), -cos(q1) * (L3 * sin(q2 + q3) - (d6 * sin(q2 + q3 + q4 + q5)) / 2 + (d6 * sin(q2 + q3 + q4 - q5)) / 2 + d5 * sin(q2 + q3 + q4)), -cos(q1) * ((d6 * sin(q2 + q3 + q4 - q5)) / 2 - (d6 * sin(q2 + q3 + q4 + q5)) / 2 + d5 * sin(q2 + q3 + q4)), d6* cos(q1)* cos(q2)* cos(q3)* cos(q5)* sin(q4) - d6 * cos(q1) * cos(q5) * sin(q2) * sin(q3) * sin(q4) - d6 * sin(q1) * sin(q5) + d6 * cos(q1) * cos(q2) * cos(q4) * cos(q5) * sin(q3) + d6 * cos(q1) * cos(q3) * cos(q4) * cos(q5) * sin(q2), 0;
		//J.block<1, 6>(2, 0) << 0, L3 * cos(q2 + q3) - (d6 * cos(q2 + q3 + q4 + q5)) / 2 + L2 * cos(q2) + (d6 * cos(q2 + q3 + q4 - q5)) / 2 + d5 * cos(q2 + q3 + q4), L3 * cos(q2 + q3) - (d6 * cos(q2 + q3 + q4 + q5)) / 2 + (d6 * cos(q2 + q3 + q4 - q5)) / 2 + d5 * cos(q2 + q3 + q4), (d6 * cos(q2 + q3 + q4 - q5)) / 2 - (d6 * cos(q2 + q3 + q4 + q5)) / 2 + d5 * cos(q2 + q3 + q4), -(d6 * (cos(q2 + q3 + q4 + q5) + cos(q2 + q3 + q4 - q5))) / 2, 0;

		//J.block<1, 6>(3, 0) << sin(q2 + q3 + q4 + q6) / 2 + sin(q2 + q3 + q4 - q6) / 2 + cos(q2 + q3 + q4) * cos(q5) * sin(q6), sin(q5) * sin(q6), sin(q5) * sin(q6), sin(q5)* sin(q6), cos(q6), 0;
		//J.block<1, 6>(4, 0) << cos(q2 + q3 + q4 + q6) / 2 - cos(q2 + q3 + q4 - q6) / 2 + cos(q2 + q3 + q4) * cos(q5) * cos(q6), cos(q6) * sin(q5), cos(q6) * sin(q5), cos(q6)* sin(q5), -sin(q6), 0;
		//J.block<1, 6>(5, 0) << sin(q2 + q3 + q4 - q5) / 2 - sin(q2 + q3 + q4 + q5) / 2, cos(q5), cos(q5), cos(q5), 0, 1;
		
		// 20210923更新，与UR机器人坐标系保持一致
		J.block<1, 6>(0, 0) << d6 * (cos(q1) * cos(q5) + cos(q2 + q3 + q4) * sin(q1) * sin(q5)) + d4 * cos(q1) - L2 * cos(q2) * sin(q1) - d5 * sin(q2 + q3 + q4) * sin(q1) - L3 * cos(q2) * cos(q3) * sin(q1) + L3 * sin(q1) * sin(q2) * sin(q3), -cos(q1) * (L2 * sin(q2) - d5 * cos(q2 + q3 + q4) + L3 * cos(q2) * sin(q3) + L3 * cos(q3) * sin(q2) - d6 * sin(q2 + q3 + q4) * sin(q5)), cos(q1)* (d5 * cos(q2 + q3 + q4) - L3 * cos(q2) * sin(q3) - L3 * cos(q3) * sin(q2) + d6 * sin(q2 + q3 + q4) * sin(q5)), cos(q1)* (d5 * cos(q2 + q3 + q4) + d6 * sin(q2 + q3 + q4) * sin(q5)), -d6 * (sin(q1) * sin(q5) + cos(q2 + q3 + q4) * cos(q1) * cos(q5)), 0;
		J.block<1, 6>(1, 0) << d6 * (cos(q5) * sin(q1) - cos(q2 + q3 + q4) * cos(q1) * sin(q5)) + d4 * sin(q1) + L2 * cos(q1) * cos(q2) + d5 * sin(q2 + q3 + q4) * cos(q1) + L3 * cos(q1) * cos(q2) * cos(q3) - L3 * cos(q1) * sin(q2) * sin(q3), -sin(q1) * (L2 * sin(q2) - d5 * cos(q2 + q3 + q4) + L3 * cos(q2) * sin(q3) + L3 * cos(q3) * sin(q2) - d6 * sin(q2 + q3 + q4) * sin(q5)), sin(q1)* (d5 * cos(q2 + q3 + q4) - L3 * cos(q2) * sin(q3) - L3 * cos(q3) * sin(q2) + d6 * sin(q2 + q3 + q4) * sin(q5)), sin(q1)* (d5 * cos(q2 + q3 + q4) + d6 * sin(q2 + q3 + q4) * sin(q5)), d6* (cos(q1) * sin(q5) - cos(q2 + q3 + q4) * cos(q5) * sin(q1)), 0;
		J.block<1, 6>(2, 0) << 0, L3* cos(q2 + q3) + L2 * cos(q2) + d5 * (cos(q2 + q3) * sin(q4) + sin(q2 + q3) * cos(q4)) - d6 * sin(q5) * (cos(q2 + q3) * cos(q4) - sin(q2 + q3) * sin(q4)), L3* cos(q2 + q3) + d5 * sin(q2 + q3 + q4) - d6 * cos(q2 + q3 + q4) * sin(q5), d5* sin(q2 + q3 + q4) - d6 * cos(q2 + q3 + q4) * sin(q5), -d6 * sin(q2 + q3 + q4) * cos(q5), 0;

		J.block<1, 6>(3, 0) << cos(q2 + q3 + q4) * sin(q6) + sin(q2 + q3 + q4) * cos(q5) * cos(q6), cos(q6)* sin(q5), cos(q6)* sin(q5), cos(q6)* sin(q5), -sin(q6), 0;
		J.block<1, 6>(4, 0) << cos(q2 + q3 + q4) * cos(q6) - sin(q2 + q3 + q4) * cos(q5) * sin(q6), -sin(q5) * sin(q6), -sin(q5) * sin(q6), -sin(q5) * sin(q6), -cos(q6), 0;
		J.block<1, 6>(5, 0) << -sin(q2 + q3 + q4) * sin(q5), cos(q5), cos(q5), cos(q5), 0, 1;

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
		// 202109123
		T1 << cos(q1), 0, sin(q1), 0,
			sin(q1), 0, -cos(q1), 0,
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
		T4 << cos(q4), 0, sin(q4), 0,
			sin(q4), 0, -cos(q4), 0,
			0, 1, 0, d4,
			0, 0, 0, 1;
		T5 << cos(q5), 0, -sin(q5), 0,
			sin(q5), 0, cos(q5), 0,
			0, -1, 0, d5,
			0, 0, 0, 1;
		T6 << cos(q6), -sin(q6), 0, 0,
			sin(q6), cos(q6), 0, 0,
			0, 0, 1, d6,
			0, 0, 0, 1;

		T = T1 * T2 * T3 * T4 * T5 * T6;
		R = T.block<3, 3>(0, 0);
		x = T.block<3, 1>(0, 3);
	}

	void CartesianSpaceTrackUR::kinematic_controller()
	{
		Eigen::Matrix3d dR = R_des.transpose() * R;
		Eigen::Quaterniond d_quat(dR);
		Eigen::Matrix<double, 6, 1> dx, qrd, xx_desd;
		

		switch (controller_type)
		{
		case CONTROLLER_TYPE::POSITION_ONLY :
		{
			dx << x(0) - x_des(0), x(1) - x_des(1), x(2) - x_des(2), 0, 0, 0;
			xx_desd << xd_des(0), xd_des(1), xd_des(2), 0, 0, 0;
			break;
		}
		case CONTROLLER_TYPE::FULL_POSE :
		{
			dx << x(0) - x_des(0), x(1) - x_des(1), x(2) - x_des(2), d_quat.x(), d_quat.y(), d_quat.z();
			xx_desd << xd_des(0), xd_des(1), xd_des(2), w_des(0), w_des(1), w_des(2);
			break;
		}
		case CONTROLLER_TYPE::NO_CONTROL :
		{
			dx.setZero();
			xx_desd.setZero();
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
			qrd = - Kp * J.transpose() * dx;
		}
		else
		{
			qrd = J.inverse() * (xx_desd - Kp * dx);
		}
		
		//qrd(1) = -qrd(1); qrd(2) = -qrd(2); qrd(3) = -qrd(3);

		for (int i = 0;i < 6;i++)
		{
			qcd[i] = qrd(i);
		}
	}
	bool CartesianSpaceTrackUR::is_singularity()
	{
		// cal the minimal eigenvalue of the jacobian matrix
		Eigen::EigenSolver<Eigen::Matrix<double, 6, 6>> es(J.transpose()*J);
		Eigen::Matrix<double, 6, 6> D = es.pseudoEigenvalueMatrix();

		double min_eig = D(0,0);
		for (int i = 1;i < 6;i++)
		{
			if (min_eig > D(i, i))
				min_eig = D(i, i);
		}
		if (min_eig <= 1e-4) // close to singularity
			return true;
		else
			return false;
	}

	Eigen::VectorXd CartesianSpaceTrackUR::get_tool_pose()
	{
		Eigen::VectorXd pose;
		pose.resize(6);
		Vec theta = rot2vec(R);
		pose << theta(0), theta(1), theta(2), x(0), x(1), x(2);
		return pose;
	}

	Eigen::VectorXd CartesianSpaceTrackUR::get_tool_pose_1()
	{
		Eigen::VectorXd pose;
		pose.resize(6);
		std::vector<double> pose_;
		pose_ = client.getToolPose();
		pose << pose_[3], pose_[4], pose_[5], pose_[0], pose_[1], pose_[2];
		return pose;
	}
}