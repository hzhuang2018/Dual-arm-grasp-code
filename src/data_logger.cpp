//
// data_logger.cpp
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

#include "data_logger.h"

namespace controllers
{
	Data_Logger::Data_Logger(ur_sdk::client& client):client_(client) {}
	Data_Logger::~Data_Logger() {}

	bool Data_Logger::init(char *_Filename)
	{
		return client_.log_init(_Filename);
	}

	void Data_Logger::starting(const boost::posix_time::ptime& time)
	{
		client_.log_start();
	}

	void Data_Logger::update(const boost::posix_time::ptime& time, const boost::posix_time::time_duration& period)
	{
		client_.log_update();
	}

	void Data_Logger::command()
	{

	}

	void Data_Logger::stopping(const boost::posix_time::ptime& time)
	{
		client_.log_stop();
	}
}
