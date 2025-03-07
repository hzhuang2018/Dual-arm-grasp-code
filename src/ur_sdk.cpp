// ur_sdk.cpp
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

#include "ur_sdk.h"


#define UR_DATA_TO_COUT 0

namespace ur_sdk {
	/// Constructor starts the asynchronous connect operation.
	client::client(boost::asio::io_service& io_service,
		boost::condition_variable& condition,
		const std::string& host, const std::string& service)
		: connection_(io_service)
	{
		p_condition_ = &condition;
		// Resolve the host name into an IP address.
		boost::asio::ip::tcp::resolver resolver(io_service);
		boost::asio::ip::tcp::resolver::query query(host, service);
		boost::asio::ip::tcp::resolver::iterator endpoint_iterator =
			resolver.resolve(query);

		// Start an asynchronous connect operation.
		boost::asio::async_connect(connection_.socket(), endpoint_iterator,
			boost::bind(&client::handle_connect, this,
			boost::asio::placeholders::error));

		//run_flag = false;
	}

	/// Handle completion of a connect operation.
	void client::handle_connect(const boost::system::error_code& e)
	{
		if (!e)
		{
			// Successfully established connection. Start operation to read the list
			// of stocks. The connection::async_read() function will automatically
			// decode the data that is read from the underlying socket.
			unsigned long int s = (connection_.socket()).native_handle();
			char flag[]={1};
			setsockopt(s, IPPROTO_TCP, TCP_NODELAY, flag, sizeof(int));//TCP_QUICKACK 
			//setsockopt(s, IPPROTO_TCP, TCP_QUICKACK, flag, sizeof(int));
			setsockopt(s, SOL_SOCKET, SO_REUSEADDR, flag, sizeof(int));
			
			connection_.async_read(rt_client_obj_,
				boost::bind(&client::handle_read, this,
				boost::asio::placeholders::error));
			start_time_ = boost::posix_time::microsec_clock::local_time();
		}
		else
		{
			// An error occurred. Log it and return. Since we are not starting a new
			// operation the io_service will run out of work to do and the client will
			// exit.
			std::cerr << e.message() << std::endl;
		}
	}

	/// Handle completion of a read operation.
	void client::handle_read(const boost::system::error_code& e)
	{
		if (!e)
		{
			// Print out the data that was received.
			std::ostringstream command;
			//string command;
			//command << "movel(p[" 
			//	<< rt_client_obj_.Tool_vector_target[0]+0.05 <<","
			//	<< rt_client_obj_.Tool_vector_target[1] <<","
			//	<< rt_client_obj_.Tool_vector_target[2] <<","
			//	<< rt_client_obj_.Tool_vector_target[3] <<","
			//	<< rt_client_obj_.Tool_vector_target[4] <<","
			//	<< rt_client_obj_.Tool_vector_target[5] 
			//	<< "])\n";

			
			//if (qd_.size() == 6)
			//{
			//	command << "speedj(["
			//		<< qd_[0] << ","
			//		<< qd_[1] << ","
			//		<< qd_[2] << ","
			//		<< qd_[3] << ","
			//		<< qd_[4] << ","
			//		<< qd_[5] << "],"
			//		<< qdd_ << ","
			//		<< "0.072)\n";
			//		//<<"zero_ftsensor()\n";
			//	//std::string command_ = command.str();
			//	boost::asio::write(connection_.socket(),boost::asio::buffer(command.str()));
			//}
			if (!run_flag)
			{
				command <<
					"def zero_sensor():\n\t"
					<< "zero_ftsensor()\n"
					<< "sleep(0.1)\n"
					<< "end\n"

					//<< "thread Force_properties_calculation_thread_1():\n\t"
					//<<"while (True):\n"
					//<<"force_mode(tool_pose(), [1, 0, 0, 0, 0, 0], [5.0, 0.0, 0.0, 0.0, 0.0, 0.0], 2, [0.05, 0.05, 0.05, 0.07, 0.07, 0.07])\n"
					//<<"sync()\n"
					//<< "end\n"
					//<< "end\n"
					//<<" global thread_handler_1 = run Force_properties_calculation_thread_1()\n"

					;
				//std::string command_ = command.str();
				boost::asio::write(connection_.socket(), boost::asio::buffer(command.str()));
				run_flag = true;
			}


			if (qd_.size() == 6)
			{
				command <<
					"def free():\n\t" <<
					"speedj(["
					<< qd_[0] << ","
					<< qd_[1] << ","
					<< qd_[2] << ","
					<< qd_[3] << ","
					<< qd_[4] << ","
					<< qd_[5] << "],"
					<< 0.5 << ","//qdd_
					<< "0.072)\n"//0.072
				  //  <<"zero_ftsensor()\n"
					<<"end\n";
			//std::string command_ = command.str();
				boost::asio::write(connection_.socket(), boost::asio::buffer(command.str()));
			}

			//if (qd_.size() == 6)
			//{
			//	command <<
			//		"def free():\n\t" <<
			//		"thread force_thread():\n\t"<<
			//		"while(True):\n"<<
			//		"force_mode(tool_pose(), [0, 0, 1, 0, 0, 0], [0.0, 0.0, 5.0, 0.0, 0.0, 0.0], 2, [0.1, 0.1, 0.05, 0.17, 0.17, 0.17])\n"<<
			//		"sync()\n"<<
			//		"end\n"<<
			//		"end\n"<<
			//		"global thread_handler_1 = run force_thread()\n"<<
			//		"speedj(["
			//		<< qd_[0] << ","
			//		<< qd_[1] << ","
			//		<< qd_[2] << ","
			//		<< qd_[3] << ","
			//		<< qd_[4] << ","
			//		<< qd_[5] << "],"
			//		<< qdd_ << ","
			//		<< "0.072)\n"
			//		<< "zero_ftsensor()\n"
			//		<< "end\n";
			//	//std::string command_ = command.str();
			//	boost::asio::write(connection_.socket(), boost::asio::buffer(command.str()));
			//}

			
			
			//if (q_.size() == 6)
			//{
			//	command << "servoj([" 
			//		<< q_[0] <<","
			//		<< q_[1] <<","
			//		<< q_[2] <<","
			//		<< q_[3] <<","
			//		<< q_[4] <<","
			//		<< q_[5] << "],"
			//		<< 0 <<","
			//		<< 0 << ","
			//		<< 0.0072 << ","//0.008  0.0072
			//		<< 0.2 << ","//0.1
			//		<< "100)\n";//300
			//	//std::string command_ = command.str();
			//	boost::asio::write(connection_.socket(),boost::asio::buffer(command.str()));
			//}
			
			p_condition_->notify_all();
			boost::posix_time::ptime now(boost::posix_time::microsec_clock::local_time());
#if UR_DATA_TO_COUT
			// Print out the data that was received.
			if (ctr_cout_++ >125)
			{
				std::cout <<"["<< now.time_of_day() <<"]"
					<< "Time: " << rt_client_obj_.Time <<"\n"
					<< rt_client_obj_.q_actual[0] <<",    "
					<< rt_client_obj_.q_actual[1] <<",    "
					<< rt_client_obj_.q_actual[2] <<",    "
					<< rt_client_obj_.q_actual[3] <<",    "
					<< rt_client_obj_.q_actual[4] <<",    "
					<< rt_client_obj_.q_actual[5] <<"\n";
				ctr_cout_ = 0;
			}
#endif
		}
		else
		{
			// An error occurred.
			std::cerr << e.message() << std::endl;
		}

		// starting a new operation.
		connection_.async_read(rt_client_obj_,boost::bind(&client::handle_read, this,	boost::asio::placeholders::error));
	}

	void client::setSpeed(std::vector<double>& qd, double& qdd)
	{
		if(qd.size()==6)
		{
			qd_ = qd;
			qdd_ = qdd;
		}
	}

	void client::setSpeed(std::vector<double>& q, std::vector<double>& qd, double& qdd)
	{
		if (qd.size() == 6)
		{
			q_ = q;
			qd_ = qd;
			qdd_ = qdd;
			//run_flag = false;
		}
	}

	std::vector<double> client::getSpeed()
	{
		std::vector<double> qd;
		for (int i = 0;i < 6;i++)
		{
			qd.push_back(rt_client_obj_.qd_actual[i]);
		}
		return qd;
	}

	std::vector<double> client::getTcpSpeed()
	{
		std::vector<double> tcp_actual_speed;
		for (int i = 0; i < 6; i++)
		{
			tcp_actual_speed.push_back(rt_client_obj_.TCP_speed_actual[i]);
		}
		return tcp_actual_speed;
	}

	std::vector<double> client::getPosition()
	{
		std::vector<double> q;
		for (int i = 0;i < 6;i++)
		{
			q.push_back(rt_client_obj_.q_actual[i]);
		}
		return q;
	}
	std::vector<double> client::getTCP_force()
	{
		std::vector<double> force;
		for (int i = 0; i < 6; i++)
		{
			force.push_back(rt_client_obj_.TCP_force[i]);
		}
		return force;
	}

	std::vector<double> client::getTargetPosition()
	{
		std::vector<double> q;
		for (int i = 0;i < 6;i++)
		{
			q.push_back(rt_client_obj_.q_target[i]);
		}
		return q;
	}

	std::vector<double> client::getTargetSpeed()
	{
		std::vector<double> qd;
		for (int i = 0;i < 6;i++)
		{
			qd.push_back(rt_client_obj_.qd_target[i]);
		}
		return qd;
	}

	std::vector<double> client::getTargetAcc()
	{
		std::vector<double> qdd;
		for (int i = 0;i < 6;i++)
		{
			qdd.push_back(rt_client_obj_.qdd_target[i]);
		}
		return qdd;
	}

	std::vector<double> client::getToolPose()
	{
		std::vector<double> pose;
		for (int i=0; i<6; i++)
		{
			pose.push_back(rt_client_obj_.Tool_vector_actual[i]);
		}
		return pose;
	}

	std::vector<double> client::getActualCurrent()
	{
		std::vector<double> ActualCurrent;
		for (int i = 0; i < 6; i++)
		{
			ActualCurrent.push_back(rt_client_obj_.I_actual[i]);
		}
		return ActualCurrent;
	}

	std::vector<double> client::getControlCurrent()
	{
		std::vector<double> ControlCurrent;
		for (int i = 0; i < 6; i++)
		{
			ControlCurrent.push_back(rt_client_obj_.I_control[i]);
		}
		return ControlCurrent;
	}

	bool client::log_init(const char *_Filename)
	{
		of_ur_.open(_Filename);
		return of_ur_.is_open();
	}

	void client::log_start()
	{
		if(of_ur_.is_open())
		{
			of_ur_ << "time" << "\t";
			for (int i = 0;i < 6;i++)
			{
				of_ur_ << "q_target_" << i << "\t";
			}
			for (int i = 0;i < 6;i++)
			{
				of_ur_ << "qd_target_" << i << "\t";
			}
			for (int i = 0;i < 6;i++)
			{
				of_ur_ << "q_actual_" << i << "\t";
			}
			for (int i = 0;i < 6;i++)
			{
				of_ur_ << "qd_actual_" << i << "\t";
			}
			of_ur_ << "\n";
		}
	}

	void client::log_update()
	{
		if(of_ur_.is_open())
		{
			boost::posix_time::ptime now(boost::posix_time::microsec_clock::local_time());
			boost::posix_time::time_duration elapsed_time;
			elapsed_time = now - start_time_;
			of_ur_ << double(elapsed_time.total_milliseconds()) << "\t";
			for (int i = 0;i < 6;i++)
			{
				of_ur_ << rt_client_obj_.q_target[i] << "\t";
			}
			for (int i = 0;i < 6;i++)
			{
				of_ur_ << rt_client_obj_.qd_target[i] << "\t";
			}
			for (int i = 0;i < 6;i++)
			{
				of_ur_ << rt_client_obj_.q_actual[i] << "\t";
			}
			for (int i = 0;i < 6;i++)
			{
				of_ur_ << rt_client_obj_.qd_actual[i] << "\t";
			}
			of_ur_ << "\n";
		}
	}

	void client::log_stop()
	{
		of_ur_.close();
	}

} // namespace ur_sdk