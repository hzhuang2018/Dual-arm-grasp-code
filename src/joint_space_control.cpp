//
// joint_space_control.cpp
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

#include "joint_space_control.h"

#include <math.h>

namespace controllers
{
	Joint_Space_Control::Joint_Space_Control(ur_sdk::client& client):client_(client) {}
	Joint_Space_Control::~Joint_Space_Control() {}

	bool Joint_Space_Control::init(const boost::posix_time::ptime& time)
	{
		q_actual_.resize(6);
		qd_actual_.resize(6);
		q_target_.resize(6);
		qd_target_.resize(6);
		qdd_target_ = 0;

		return 0;
	}

	void Joint_Space_Control::starting(const boost::posix_time::ptime& time)
	{
		start_time_ = time;
		start_position_ = client_.getPosition();
		q_actual_ = client_.getPosition();
		qd_actual_ = client_.getSpeed();
		qdd_target_ = 1.5;
	}

	void Joint_Space_Control::update(const boost::posix_time::ptime& time, const boost::posix_time::time_duration& period)
	{
		elapsed_time_ = time - start_time_;
		q_actual_ = client_.getPosition();
		qd_actual_ = client_.getSpeed();
		for (int i = 0;i < 6;i++)
		{
			q_target_[i] = 0.1*(cos(elapsed_time_.total_milliseconds()/1000.0) - 1) + start_position_[i];
			qd_target_[i] = -0.1*sin(elapsed_time_.total_milliseconds()/1000.0) + 1.0*(q_target_[i]-q_actual_[i]);
		}
		client_.setSpeed(qd_target_,qdd_target_);
	}

	void Joint_Space_Control::command()
	{
		
	}

	void Joint_Space_Control::stopping(const boost::posix_time::ptime& time)
	{
		
	}

	bool Joint_Space_Control::log_init(const char *_Filename)
	{
		of_.open(_Filename);
		return of_.is_open();
	}

	void Joint_Space_Control::log_start(const boost::posix_time::ptime& time)
	{
		if(of_.is_open())
		{
			of_ << "time" << "\t";
			for (int i = 0;i < 6;i++)
			{
				of_ << "q_target_" << i << "\t";
			}
			for (int i = 0;i < 6;i++)
			{
				of_ << "qd_target_" << i << "\t";
			}
			for (int i = 0;i < 6;i++)
			{
				of_ << "q_actual_" << i << "\t";
			}
			for (int i = 0;i < 6;i++)
			{
				of_ << "qd_actual_" << i << "\t";
			}
			of_ << "\n";
		}
	}

	void Joint_Space_Control::log_update(const boost::posix_time::ptime& time, const boost::posix_time::time_duration& period)
	{
		if(of_.is_open())
		{
			of_ << double(elapsed_time_.total_milliseconds()) << "\t";
			for (int i = 0;i < 6;i++)
			{
				of_ << q_target_[i] << "\t";
			}
			for (int i = 0;i < 6;i++)
			{
				of_ << qd_target_[i] << "\t";
			}
			for (int i = 0;i < 6;i++)
			{
				of_ << q_actual_[i] << "\t";
			}
			for (int i = 0;i < 6;i++)
			{
				of_ << qd_actual_[i] << "\t";
			}
			of_ << "\n";
		}
	}

	void Joint_Space_Control::log_stop(const boost::posix_time::ptime& time)
	{
		of_.close();
	}
}
