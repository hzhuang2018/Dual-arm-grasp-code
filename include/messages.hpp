//
// messages.hpp
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

#ifndef UR_MESSAGES_HPP
#define UR_MESSAGES_HPP

#include <string>
#include "dll_export.h"
#define CB3_3_5
#define CBE_5_11

namespace ur_sdk {

	/// Structure to hold information about a single stock.
	struct ROBOTIC_GRASP_DLL rt_client_obj
	{
		double	Time;
		double	q_target[6];
		double	qd_target[6];
		double	qdd_target[6];
		double	I_target[6];
		double	M_target[6];
		double	q_actual[6];
		double	qd_actual[6];
		double	I_actual[6];
		double	I_control[6];
		double	Tool_vector_actual[6];
		double	TCP_speed_actual[6];
		double	TCP_force[6];
		double	Tool_vector_target[6];
		double	TCP_speed_target[6];
		double	Digital_input_bits;
		double	Motor_temperatures[6];
		double	Controller_Timer;
		double	Test_value;
		double	Robot_Mode;
		double	Joint_Modes[6];
		double	Safety_Mode;
		double	preserve_0[6];
		double	Tool_Accelerometer_values[3];
		double	preserve_1[6];
		double	Speed_scaling;
		double	Linear_momentum_norm;
		double	preserve_2;
		double	preserve_3;
		double	V_main;	
		double	V_robot;	
		double	I_robot;	
		double	V_actual[6];
		double	Digital_outputs;
		double	Program_state;
#ifdef	CB3_3_5
		double  Elbow_position[3];
		double  Elbow_velocity[3];
#endif
		
#ifdef	CBE_5_11
		double  Safety_status;
		double	preserve_4;
		double	preserve_5;
		double	preserve_6;
		double	Payload_mass;
		double	Payload_cog[3];
		double	Payload_inertia[6];
#endif
		// template <typename Archive>
		// void serialize(Archive& ar, const unsigned int version)
		// {
		//ar &	Time;
		//ar &	q_target[6];
		//ar &	qd_target[6];
		//ar &	qdd_target[6];
		//ar &	I_target[6];
		//ar &	M_target[6];
		//ar &	q_actual[6];
		//ar &	qd_actual[6];
		//ar &	I_actual[6];
		//ar &	I_control[6];
		//ar &	Tool_vector_actual[6];
		//ar &	TCP_speed_actual[6];
		//ar &	TCP_force[6];
		//ar &	Tool_vector_target[6];
		//ar &	TCP_speed_target[6];
		//ar &	Digital_input_bits;
		//ar &	Motor_temperatures[6];
		//ar &	Controller_Timer;
		//ar &	Test_value;
		//ar &	Robot_Mode;
		//ar &	Joint_Modes[6];
		//ar &	Safety_Mode;
		//ar &	preserve_0[6];
		//ar &	Tool_Accelerometer_values[3];
		//ar &	preserve_1[6];
		//ar &	Speed_scaling;
		//ar &	Linear_momentum_norm;
		//ar &	preserve_2;
		//ar &	preserve_3;
		//ar &	V_main;	
		//ar &	V_robot;	
		//ar &	I_robot;	
		//ar &	V_actual[6];
		//ar &	Digital_outputs;
		//ar &	Program_state;
		// }
	};

} // namespace ur_sdk

#endif // MESSAGES_HPP