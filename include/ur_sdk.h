//
// ur_sdk.h
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

#ifndef UR_SDK_H
#define	UR_SDK_H

#ifndef BOOST_USE_WINDOWS_H
#define BOOST_USE_WINDOWS_H
#endif

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/endian/conversion.hpp>
#include <boost/date_time.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "connection.hpp" // Must come before boost/serialization headers.
#include <boost/serialization/vector.hpp>
#include "messages.hpp"
#ifdef __linux__ 
#include <sys/socket.h>
#endif

#ifdef _WIN32
#include <WinSock2.h>  
#endif

#include "dll_export.h"

namespace ur_sdk {
	/// Downloads stock quote information from a server.
	class ROBOTIC_GRASP_DLL client
	{
	public:
		/// Constructor starts the asynchronous connect operation.
		client(boost::asio::io_service& io_service,
			boost::condition_variable& condition,
			const std::string& host, const std::string& service);

		/// Handle completion of a connect operation.
		void handle_connect(const boost::system::error_code& e);
		/// Handle completion of a read operation.
		void handle_read(const boost::system::error_code& e);

		void setSpeed(std::vector<double>& qd, double& qdd);
		void setSpeed(std::vector<double>& q, std::vector<double>& qd,double& qdd);
		std::vector<double> getSpeed();
		std::vector<double> getPosition();
		std::vector<double> getTCP_force();
		std::vector<double> getTcpSpeed();
		std::vector<double> getTargetPosition();
		std::vector<double> getTargetSpeed();
		std::vector<double> getTargetAcc();
		std::vector<double> getActualCurrent();
		std::vector<double> getControlCurrent();

		/* get pose of tool */
		std::vector<double> getToolPose();
		bool force_control_;
		bool log_init(const char *_Filename);
		void log_start();
		void log_update();
		void log_stop();

	private:
		/// The connection to the server.
		connection connection_;

		/// The data received from the server.
		rt_client_obj rt_client_obj_;
		bool run_flag;
		std::vector<double> q_;//用于机器人位置控制指令
		std::vector<double> qd_;//用于机器人速度控制指令
		double qdd_;//用于机器人加速度控制指令
		
		unsigned int ctr_cout_;

		std::ofstream of_ur_;

		boost::posix_time::ptime start_time_;
		boost::condition_variable* p_condition_;
	};

} // namespace ur_sdk

#endif	/* UR_SDK_H */