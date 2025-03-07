// abstract_controller_ur.cpp
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
#include "abstract_controller_ur.h"

namespace Robot {

	AbstractControllerUR::AbstractControllerUR(ur_sdk::client& client_) :
		client(client_)
	{
		q.resize(6);
		qd.resize(6);
		qc.resize(6);
		qcd.resize(6);
		qdd_target = 1.5;
		integral_q_qc.resize(6);
		integral_q_qc.setZero();
	}
	AbstractControllerUR::~AbstractControllerUR()
	{

	}
	void AbstractControllerUR::set_hh(double hh_)
	{
		hh = hh_;
	}

	void AbstractControllerUR::start(const boost::posix_time::ptime& time)
	{
		start_time = time;
		std::vector<double> q_ = client.getPosition();
		std::vector<double> qd_ = client.getSpeed();
		std::vector<double> qc_ = client.getTargetPosition();

		for (int i = 0; i < 6; i++)
		{
			q(i) = q_[i];
			qd(i) = qd_[i];
			qc(i) = qc_[i];
		}

	}
	void AbstractControllerUR::update(const boost::posix_time::ptime& time,
		const boost::posix_time::time_duration& period)
	{
		elapsed_time = time - start_time;
		// update state
		std::vector<double> q_ = client.getPosition();
		std::vector<double> qd_ = client.getSpeed();
		std::vector<double> qc_ = client.getTargetPosition();

		for (int i = 0; i < 6; i++)
		{
			q(i) = q_[i];
			qd(i) = qd_[i];
			qc(i) = qc_[i];
		}
	

		client.setSpeed(qcd, qdd_target);
		// update variables
		double hh = period.total_milliseconds() / 1000.0;
		integral_q_qc = (q - qc) * hh;
	}

	const Eigen::VectorXd& AbstractControllerUR::get_q()
	{
		return q;
	}
	const Eigen::VectorXd& AbstractControllerUR::get_qd()
	{
		return qd;
	}

	void AbstractControllerUR::stop(const boost::posix_time::ptime& time)
	{

	}
	std::vector<double> AbstractControllerUR::get_qcd()
	{
		return qcd;
	}
	std::vector<double> AbstractControllerUR::get_qcd_pre()
	{
		return qcd_pre;
	}
	bool AbstractControllerUR::log_init(const char* _Filename)
	{
		of.open(_Filename);
		return of.is_open();
	}
	void AbstractControllerUR::log_start(const boost::posix_time::ptime& time)
	{
		if (of.is_open())
		{
			of << "time" << "\t";
			for (int i = 0; i < 6; i++)
			{
				of << "q_target_" << i << "\t";
			}
			for (int i = 0; i < 6; i++)
			{
				of << "qd_target_" << i << "\t";
			}
			for (int i = 0; i < 6; i++)
			{
				of << "q_actual_" << i << "\t";
			}
			for (int i = 0; i < 6; i++)
			{
				of << "qd_actual_" << i << "\t";
			}
			of << "\n";
		}
	}
	void AbstractControllerUR::log_update(const boost::posix_time::ptime& time,
		const boost::posix_time::time_duration& period)
	{
		if (of.is_open())
		{
			of << double(elapsed_time.total_milliseconds()) << "\t";
			for (int i = 0; i < 6; i++)
			{
				of << q(i) << "\t";
			}
			for (int i = 0; i < 6; i++)
			{
				of << qd(i) << "\t";
			}
			for (int i = 0; i < 6; i++)
			{
				of << qc(i) << "\t";
			}
			for (int i = 0; i < 6; i++)
			{
				of << qcd[i] << "\t";
			}
			of << "\n";
		}
	}
	void AbstractControllerUR::log_stop(const boost::posix_time::ptime& time)
	{
		of.close();
	}
}