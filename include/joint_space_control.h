//
// joint_space_control.h
//
//
// Copyright (c) 2017 Zhou Yang @ BICE (bicezhou@163.com)
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

#ifndef JOINT_SPACE_CONTROL_H
#define	JOINT_SPACE_CONTROL_H

#ifndef BOOST_USE_WINDOWS_H
#define BOOST_USE_WINDOWS_H
#endif
#include "ur_sdk.h"
#include "dll_export.h"
#include <Eigen/Core>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace controllers
{
	class ROBOTIC_GRASP_DLL Joint_Space_Control
	{
	public:
		Joint_Space_Control(ur_sdk::client& client);
		~Joint_Space_Control();

		bool init(const boost::posix_time::ptime& time);
		void starting(const boost::posix_time::ptime& time);
		void update(const boost::posix_time::ptime& time, const boost::posix_time::time_duration& period);
		void command();
		void stopping(const boost::posix_time::ptime& time);

		bool log_init(const char *_Filename);
		void log_start(const boost::posix_time::ptime& time);
		void log_update(const boost::posix_time::ptime& time, const boost::posix_time::time_duration& period);
		void log_stop(const boost::posix_time::ptime& time);
	private:
		ur_sdk::client& client_;

		boost::posix_time::ptime start_time_;
		boost::posix_time::time_duration elapsed_time_;

		std::vector<double> q_actual_;
		std::vector<double> qd_actual_;
		std::vector<double> q_target_;
		std::vector<double> qd_target_;
		double qdd_target_;

		std::vector<double> start_position_;

		std::ofstream of_;
	};

}

#endif	/* JOINT_SPACE_CONTROL_H */