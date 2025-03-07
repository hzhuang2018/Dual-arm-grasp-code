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
			command<<"FFFEFDFC010802010000000000FB";
			boost::asio::write(connection_.socket(), boost::asio::buffer(command.str()));
			connection_.async_read(rt_client_obj_,boost::bind(&client::handle_read, this,	boost::asio::placeholders::error));
		}

	}		

	void client::handle_write(const boost::system::error_code& e)
	{
		if (!e)
		{
			// Print out the data that was received.
			std::ostringstream command;
			command<<"FFFEFDFC010802010000000000FB";
			boost::asio::write(connection_.socket(), boost::asio::buffer(command.str()));
			
		}

	}		


} // namespace ur_sdk