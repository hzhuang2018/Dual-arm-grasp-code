// abstract_controller_ur.h
//
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
#ifndef BOOST_USE_WINDOWS_H
#define BOOST_USE_WINDOWS_H
#endif
#include "ur_sdk.h"
#include "dll_export.h"
#include <Eigen/Dense>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace Robot {

	class ROBOTIC_GRASP_DLL AbstractControllerUR
	{
	public:
		AbstractControllerUR(ur_sdk::client& client_);
		~AbstractControllerUR();
		/* initilize the values of the state */
		void start(const boost::posix_time::ptime& time);
		/* virtual method that users can reload to implement their own contol algorithms */
		virtual void update(const boost::posix_time::ptime& time, const boost::posix_time::time_duration& period);
		void stop(const boost::posix_time::ptime& time);
		/* functions that are used to generate the state log */
		bool log_init(const char* _Filename);
		void log_start(const boost::posix_time::ptime& time);
		void log_update(const boost::posix_time::ptime& time, const boost::posix_time::time_duration& period);
		void log_stop(const boost::posix_time::ptime& time);
		/* */
		void set_hh(double hh_);
		/* functions that return state of ur */
		const Eigen::VectorXd& get_q();
		const Eigen::VectorXd& get_qd();
		std::vector<double> get_qcd();
		std::vector<double> get_qcd_pre();
		std::vector<double> q_pos_target;//关节位置，用来发给机器人的关节位置指令
		std::vector<double> qcd;//关节速度，用来发给机器人的速度指令
		std::vector<double> qcd_pre;//上一次的关节速度，用于发给机器人的速度指令

	protected:
		ur_sdk::client& client;

		boost::posix_time::ptime start_time;
		boost::posix_time::time_duration elapsed_time;
		boost::posix_time::time_duration elapsed_time_pre;
		double hh;
		// state of local robot
		Eigen::VectorXd q;//用来读取关节位置
		Eigen::VectorXd qd;//用来读取关节速度
		Eigen::VectorXd qc;//用来读取关节加速度

		//std::vector<double> q_pos_target;//关节位置，用来发给机器人的关节位置指令
		//std::vector<double> qcd;//关节速度，用来发给机器人的速度指令
		//std::vector<double> qcd_pre;//上一次的关节速度，用于发给机器人的速度指令

		Eigen::VectorXd integral_q_qc;

		double qdd_target;

		// state of remote robot
		Eigen::VectorXd q_remote;
		Eigen::VectorXd qd_remote;

		std::ofstream of;
	};
}