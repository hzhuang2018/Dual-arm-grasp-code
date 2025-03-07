//
// data_logger.h
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

#ifndef DATA_LOGGER_H
#define	DATA_LOGGER_H

#include "ur_sdk.h"
#include "dll_export.h"
#include <boost/date_time/posix_time/posix_time.hpp>

namespace controllers
{
	class ROBOTIC_GRASP_DLL Data_Logger
	{
	public:
		Data_Logger(ur_sdk::client& client);
		~Data_Logger();

		bool init(char *_Filename);
		void starting(const boost::posix_time::ptime& time);
		void update(const boost::posix_time::ptime& time, const boost::posix_time::time_duration& period);
		void command();
		void stopping(const boost::posix_time::ptime& time);
	private:
		ur_sdk::client& client_;
	};

}

#endif	/* Data_Logger_H */