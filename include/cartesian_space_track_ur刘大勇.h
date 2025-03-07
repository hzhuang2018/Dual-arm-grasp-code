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

namespace Robot {

	class ROBOTIC_GRASP_DLL CartesianSpaceTrackUR : public AbstractControllerUR {
	public:
		enum CONTROLLER_TYPE
		{
			POSITION_ONLY,
			FULL_POSE,
			NO_CONTROL
		};
	public:
		CartesianSpaceTrackUR(ur_sdk::client& client_);
		~CartesianSpaceTrackUR();

		virtual void update(const boost::posix_time::ptime& time, const boost::posix_time::time_duration& period);
		// 
		void specify_qcd(double scale_); // this function should be called after update
		// set the kinematic parameters of the ur robot
		void set_kinematic_param(double d1_, double L2_, double L3_, double d4_, double d5_, double d6_);
		// set the desired orientation and position of the end frame
		void set_desired_position(const Eigen::Vector3d& x_, const Eigen::Vector3d& xd_, const Eigen::Vector3d& xdd_);
		void set_desired_orientation(const Eigen::Matrix3d& R_, const Eigen::Vector3d& w_, const Eigen::Vector3d& wd_);

		Eigen::VectorXd get_tool_pose();
		Eigen::VectorXd get_tool_pose_1();

		double Kp;

		CONTROLLER_TYPE controller_type;
	private:
		// position and velocity in cartesian space
		Eigen::Vector3d x;
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
		// Jacobian matrix
		Eigen::Matrix<double, 6, 6> J;

		double q1, q2, q3, q4, q5, q6;
		// kinematic parameters
		double d1, L2, L3, d4, d5, d6;
		// cal the Jacobian matrix
		void cal_J();
		// cal the position and orientation of the end frame
		void cal_pose();
		// 
		void kinematic_controller();
		// singularity monitering
		bool is_singularity();
	};

}