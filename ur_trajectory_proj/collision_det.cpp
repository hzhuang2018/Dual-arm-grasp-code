#include "collision_det.h"
bool bCrash = false;
Eigen::Matrix3d setRPY(const Eigen::Vector3d rot)
{
	Eigen::Matrix3d ret;
	ret = Eigen::AngleAxisd(rot.x(), Eigen::Vector3d::UnitX())
		* Eigen::AngleAxisd(rot.y(), Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(rot.z(), Eigen::Vector3d::UnitZ());
	return ret;
}

void collision_detection(const CartesianSpaceTrackUR* controller_ur, const  CartesianSpaceTrackUR* controller_ur2)
{
	//Eigen::Matrix<double, 4, 4> T1, T2, T3, T4, T5, T6, T;
	boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
	boost::posix_time::ptime cur_time, circle_end_time;
	boost::posix_time::time_duration d_time, circle_time;
	double d_ms, cirtime_ms;

	//UR10e
	double d1 = 0.1807;
	double L2 = -0.6127;
	double L3 = -0.57155;
	double d4 = 0.17415;
	double d5 = 0.11985;
	double d6 = 0.11655;

	//offset 
	double shoulder_offset = 0.176;//0.18
	double elbow_offset = 0.137;//0.13

	//robot 1 collision model
	// base1
	std::shared_ptr<fcl::CollisionGeometry<double>> cylinder_geometry_base1(new fcl::Cylinder<double>(0.075, 0.27));//(0.19, 0.3)
	// base2
	std::shared_ptr<fcl::CollisionGeometry<double>> cylinder_geometry_base2(new fcl::Cylinder<double>(0.055, 0.74));
	// base3
	std::shared_ptr<fcl::CollisionGeometry<double>> cylinder_geometry_base3(new fcl::Cylinder<double>(0.045, 0.675));
	// base4
	std::shared_ptr<fcl::CollisionGeometry<double>> cylinder_geometry_base4(new fcl::Cylinder<double>(0.045, 0.12));
	// base5
	std::shared_ptr<fcl::CollisionGeometry<double>> cylinder_geometry_base5(new fcl::Cylinder<double>(0.045, 0.17));
	// Sphere
	//std::shared_ptr<fcl::CollisionGeometry<double>> sphere_geometry(new fcl::Sphere<double>(0.1));
    // Box
	std::shared_ptr<fcl::CollisionGeometry<double>> box_geometry(new fcl::Box<double>(0.4, 0.4, 0.3)); 
	// Plane
	auto half_space = std::make_shared<fcl::Halfspace<double>>(fcl::Vector3<double>::UnitZ(), 0.0);
	// Pose of half space frame H in the world frame W.
	fcl::Transform3<double> X_WH = fcl::Transform3<double>::Identity();
	fcl::CollisionObject<double> half_space_co(half_space, X_WH);





    //fcl::collision_detection::CollisionRobotFCL new_crobot(*(dynamic_cast<collision_detection::CollisionRobotFCL*>(crobot_.get())));
	fcl::CollisionObject<double> cylinder_group1[5]
	{
		cylinder_geometry_base1 ,
		cylinder_geometry_base2 ,
		cylinder_geometry_base3 ,
		cylinder_geometry_base4 ,
		cylinder_geometry_base5
	};

	fcl::CollisionObject<double> box1(box_geometry);
	//fcl::CollisionObject<double> ball(sphere_geometry);

	//robot 2 collision model
	// base1
	std::shared_ptr<fcl::CollisionGeometry<double>> cylinder_geometry2_base1(new fcl::Cylinder<double>(0.075, 0.27));//(0.19, 0.3)
	// base2
	std::shared_ptr<fcl::CollisionGeometry<double>> cylinder_geometry2_base2(new fcl::Cylinder<double>(0.055, 0.74));
	// base3
	std::shared_ptr<fcl::CollisionGeometry<double>> cylinder_geometry2_base3(new fcl::Cylinder<double>(0.045, 0.675));
	// base4
	std::shared_ptr<fcl::CollisionGeometry<double>> cylinder_geometry2_base4(new fcl::Cylinder<double>(0.045, 0.12));
	// base5
	std::shared_ptr<fcl::CollisionGeometry<double>> cylinder_geometry2_base5(new fcl::Cylinder<double>(0.045, 0.17));
	// Sphere
	//std::shared_ptr<fcl::CollisionGeometry<double>> sphere_geometry(new fcl::Sphere<double>(0.1));

	//fcl::collision_detection::CollisionRobotFCL new_crobot(*(dynamic_cast<collision_detection::CollisionRobotFCL*>(crobot_.get())));
	fcl::CollisionObject<double> cylinder_group2[5]
	{ 
		cylinder_geometry2_base1 ,
		cylinder_geometry2_base2 ,
		cylinder_geometry2_base3 ,
		cylinder_geometry2_base4 ,
		cylinder_geometry2_base5
	};

	// Distance Request and Result 
	fcl::DistanceRequest<double> requestMaster[6], requestSlave[6], requestS_Mcy5[6];
	fcl::DistanceResult<double>  resultMaster[6],  resultSlave[6],  resultS_Mcy5[6];

	for (int i = 0; i < 6; i++)
	{
		requestMaster[i].enable_nearest_points = true;
		requestMaster[i].enable_signed_distance = true;
		requestMaster[i].gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;

		requestSlave[i].enable_nearest_points = true;
		requestSlave[i].enable_signed_distance = true;
		requestSlave[i].gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;

		requestS_Mcy5[i].enable_nearest_points = true;
		requestS_Mcy5[i].enable_signed_distance = true;
		requestS_Mcy5[i].gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
		// request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD; GST_INDEP
		resultS_Mcy5[i].clear();
		resultMaster[i].clear();
		resultSlave[i].clear();
	}

	Eigen::Matrix<double, 4, 4>  rob_left_base_to_rob_right_base,slave_base_to_master_base;
	rob_left_base_to_rob_right_base << 0.988042, 0.15257, 0.0222721, -1.91979,
		-0.152422, 0.988281, -0.00819861, 0.209646,
		-0.0232619, 0.00470581, 0.999718, 0.0139865,
		0, 0, 0, 1;
	slave_base_to_master_base = rob_left_base_to_rob_right_base.inverse();
	while (1)
	{
		if ((0 == controller_ur2->q1) && (0 == controller_ur2->q2) && (0 == controller_ur2->q3) &&
			(0 == controller_ur2->q4) && (0 == controller_ur2->q5) && (0 == controller_ur2->q6))
			continue;

		cur_time = boost::posix_time::microsec_clock::local_time();
		d_time = cur_time - start_time;
		d_ms = (double)d_time.total_milliseconds() / 1000;
		printf("time: %f\n", d_ms);

		//fcl::Vector3d balltrans2(0.9, -0.2057 + 0.085, 0.7934);
		//fcl::Vector3d ballrot2(0., 0., 0.);
		//ball.setTranslation(balltrans2);
		//ball.setRotation(setRPY(ballrot2));
		Eigen::Matrix<double, 4, 4>T1_temp, T1_trans, T2_temp, T2_trans, T3_temp, T4_temp, T5_temp;
		Eigen::Matrix<double, 4, 4> T0, T1, T2, T3,  T4, T5, T6, T2cy2, T3cy3, T1_4;

		fcl::Vector3d trans2_temp;

		// base1 set trans and rot
		T1_trans << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.135,//0.27/2
			0, 0, 0, 1;
		T1_temp = T1_trans;

		// base2 set trans and rot

		T0 << 1, 0, 0, 0,
			   0, 1, 0, -shoulder_offset,
			   0, 0, 1, 0,
			   0, 0, 0, 1;
		T1 << cos(controller_ur->q1), -sin(controller_ur->q1), 0, 0,
			sin(controller_ur->q1), cos(controller_ur->q1), 0, 0,
			0, 0, 1, d1,
			0, 0, 0, 1;
		T2 << cos(controller_ur->q2), -sin(controller_ur->q2), 0, 0,
			0, 0, -1, 0,
			sin(controller_ur->q2), cos(controller_ur->q2), 0, 0,
			0, 0, 0, 1;
		//cout << T21*T20*T22 << endl << endl;
		T3 << cos(controller_ur->q3), -sin(controller_ur->q3), 0, L2,//cylinder2 length　-0.7　　　　　double L2 = -0.6127;　　 double L3 = -0.57155;
			sin(controller_ur->q3), cos(controller_ur->q3), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

		T2cy2 << cos(controller_ur->q3), -sin(controller_ur->q3), 0, L2*0.5,//cylinder2 length　-0.7　　　　　double L2 = -0.6127;　　 double L3 = -0.57155;
			sin(controller_ur->q3), cos(controller_ur->q3), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

		T3cy3 << 1, 0, 0, L3* 0.5,//-0.65  
			0, 1, 0, 0,
			0, 0, 1, -elbow_offset,
			0, 0, 0, 1;
		T4 << cos(controller_ur->q4), -sin(controller_ur->q4), 0, L3,//cylinder3 length
			sin(controller_ur->q4), cos(controller_ur->q4), 0, 0,
			0, 0, 1, d4,//d4 =0.17415  -两次平移  0.12415
			0, 0, 0, 1;

		T5 << cos(controller_ur->q5), -sin(controller_ur->q5), 0, 0,
			0, 0, -1, -d5,
			sin(controller_ur->q5), cos(controller_ur->q5), 0, 0,
			0, 0, 0, 1;
		T6 << cos(controller_ur->q6), -sin(controller_ur->q6), 0, 0,
			0, 0, 1, d6 - 0.17 / 2,//d6 =0.11655
			-sin(controller_ur->q6), -cos(controller_ur->q6), 0, 0,
			0, 0, 0, 1;

		cylinder_group1[0].setTranslation(T1_temp.block<3, 1>(0, 3).transpose());
		cylinder_group1[0].setRotation(T1_temp.block<3, 3>(0, 0));

		//cylinder2.setRotation((T21*T20*T22 * T23).block<3, 3>(0, 0));
		fcl::Vector3d cy1rot(0, 0, controller_ur->q1);
		
		fcl::Vector3d cy2rot(0, -controller_ur->q2 - M_PI / 2, 0.);//cy2rot(0, controller_ur->q2 + M_PI / 2, 0.);
		cylinder_group1[1].setRotation(setRPY(cy1rot) * setRPY(cy2rot));

		cylinder_group1[1].setTranslation((T1 * T0 * T2 * T2cy2).block<3, 1>(0, 3).transpose());//平移用这个
		//cout << (T21 * T20 * T22 * T23).block<3, 1>(0, 3).transpose() << endl;
		fcl::Vector3d cy3rot(0, -controller_ur->q2 - M_PI / 2 - controller_ur->q3, 0.);//(0, controller_ur->q2 + M_PI / 2 + controller_ur->q3, 0.)
		cylinder_group1[2].setRotation(setRPY(cy1rot) * setRPY(cy3rot));
		cylinder_group1[2].setTranslation((T1 * T0 * T2 * T3 * T3cy3).block<3, 1>(0, 3).transpose());

		fcl::Vector3d cy4rot(0, controller_ur->q2 - controller_ur->q3 - controller_ur->q4, 0.);
		T1_4 = T1 * T2 * T3 * T4;
		cylinder_group1[3].setRotation(setRPY(cy1rot) * setRPY(cy4rot));	
		cylinder_group1[3].setTranslation((T1_4).block<3, 1>(0, 3).transpose());

		fcl::Vector3d cy55rot(0, 0, controller_ur->q1 + controller_ur->q5);
		fcl::Vector3d cy5rot(M_PI / 2, controller_ur->q2 + M_PI / 2 + controller_ur->q3 + controller_ur->q4 + M_PI / 2, 0);
		//cylinder5.setRotation(setRPY(cy55rot) * setRPY(cy5rot));
		cylinder_group1[4].setRotation((T1* T2* T3* T4* T5* T6).block<3, 3>(0, 0));
		cylinder_group1[4].setTranslation((T1_4* T5 * T6).block<3, 1>(0, 3).transpose());

		fcl::Vector3d box_trans2(0.7,0.,0.);
		box1.setTranslation(box_trans2);

		//for (int i = 0; i < 5; i++)
		//{
		//	fcl::Transform3d cy1aabb = cylinder_group1[i].getTransform();
		//	cout << "group1 cy"<<i<<"translation" << endl;
		//	cout << cy1aabb.matrix() <<  endl;
		//}

		//int m = 0;
		//for(int i=0;i<3;i++)
		//	for (int j = i + 2; j < 5; j++)
		//	{
		//		double dist = fcl::distance(&cylinder_group1[i], &cylinder_group1[j], requestMaster[m], resultMaster[m]);
		//		cout << "cylinder_group1: " << i << "\t cylinder_group1: " << j << "  ";
		//		cout<<dist<<"\t"<< resultMaster[m].min_distance << endl;
		//		//std::cout << result52.nearest_points[0].transpose() << std::endl;
		//		//std::cout << result52.nearest_points[1].transpose() << std::endl;
		//		m++;
		//	}
		int m = 0;
		for(int i=1;i<5;i++)
			{
				double dist1 = fcl::distance(&box1, &cylinder_group1[i], requestMaster[i], resultMaster[i]);
				//double dist2 = fcl::distance(&cylinder_group1[3], &box1, requestMaster[1], resultMaster[1]);
				//double dist3 = fcl::distance(&cylinder_group1[3], &box1, requestMaster[2], resultMaster[2]);
				//double dist4 = fcl::distance(&cylinder_group1[4], &box1, requestMaster[3], resultMaster[3]);
				cout << "cylinder_group1: " << i << "\t box1: " <<  "  "<< dist1<<  "\t" << resultMaster[i].min_distance << endl;
				//cout << "cylinder_group1: " << 2 << "\t box1: " << "  " << dist2 << "\t" << resultMaster[1].min_distance << endl;
				//cout << "cylinder_group1: " << 3 << "\t box1: " << "  " << dist3 << "\t" << resultMaster[2].min_distance << endl;
				//cout << "cylinder_group1: " << 4 << "\t box1: " << "  " << dist4 << "\t" << resultMaster[3].min_distance << endl;
				if (dist1 < 0.1 )//||dist2 < 0.1||dist3 < 0.1||dist4 < 0.1
					bCrash = true;
				//std::cout << result52.nearest_points[0].transpose() << std::endl;
				//std::cout << result52.nearest_points[1].transpose() << std::endl;
				m++;
			}

		//set cylinder_group2 rotation and translation 
		Eigen::Matrix<double, 4, 4> T20, T21, T22, T23, T24, T25, T26, T22cy2, T23cy3, T21_24;
		T20 << 1, 0, 0, 0,
			0, 1, 0, -shoulder_offset,
			0, 0, 1, 0,
			0, 0, 0, 1;
		T21 << cos(controller_ur2->q1), -sin(controller_ur2->q1), 0, 0,
			sin(controller_ur2->q1), cos(controller_ur2->q1), 0, 0,
			0, 0, 1, d1,
			0, 0, 0, 1;
		T22 << cos(controller_ur2->q2), -sin(controller_ur2->q2), 0, 0,
			0, 0, -1, 0,
			sin(controller_ur2->q2), cos(controller_ur2->q2), 0, 0,
			0, 0, 0, 1;
		//cout << T21*T20*T22 << endl << endl;
		T23 << cos(controller_ur2->q3), -sin(controller_ur2->q3), 0, L2,//cylinder2 length　-0.7　　　　　double L2 = -0.6127;　　 double L3 = -0.57155;
			sin(controller_ur2->q3), cos(controller_ur2->q3), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

		T22cy2 << cos(controller_ur2->q3), -sin(controller_ur2->q3), 0, L2 * 0.5,//cylinder2 length　-0.7　　　　　double L2 = -0.6127;　　 double L3 = -0.57155;
			sin(controller_ur2->q3), cos(controller_ur2->q3), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
		T23cy3 << 1, 0, 0, L3 * 0.5,//-0.65  
			0, 1, 0, 0,
			0, 0, 1, -elbow_offset,
			0, 0, 0, 1;
		T24 << cos(controller_ur2->q4), -sin(controller_ur2->q4), 0, L3,//cylinder3 length
			sin(controller_ur2->q4), cos(controller_ur2->q4), 0, 0,
			0, 0, 1, d4,//d4 =0.17415  -两次平移  0.12415
			0, 0, 0, 1;

		T25 << cos(controller_ur2->q5), -sin(controller_ur2->q5), 0, 0,
			0, 0, -1, -d5,
			sin(controller_ur2->q5), cos(controller_ur2->q5), 0, 0,
			0, 0, 0, 1;
		T26 << cos(controller_ur2->q6), -sin(controller_ur2->q6), 0, 0,
			0, 0, 1, d6 - 0.17 / 2,//d6 =0.11655
			-sin(controller_ur2->q6), -cos(controller_ur2->q6), 0, 0,
			0, 0, 0, 1;

		//cy1  slave_base_to_master_base*
		cylinder_group2[0].setTranslation((slave_base_to_master_base* T1_temp).block<3, 1>(0, 3).transpose());
		cylinder_group2[0].setRotation((slave_base_to_master_base* T1_temp).block<3, 3>(0, 0));
		//cy2
		fcl::Vector3d cy21rot(0, 0, controller_ur2->q1);
		fcl::Vector3d cy22rot(0, -controller_ur2->q2 - M_PI / 2, 0.);
		//cy2rot<<(0, -controller_ur2->q2 - M_PI / 2, 0.);

		cylinder_group2[1].setRotation(slave_base_to_master_base.block<3, 3>(0, 0)* setRPY(cy21rot)* setRPY(cy22rot));
		cylinder_group2[1].setTranslation((slave_base_to_master_base* T21* T20* T22* T22cy2).block<3, 1>(0, 3).transpose());//平移用这个
		//cy3
		fcl::Vector3d cy23rot(0, -controller_ur2->q2 - M_PI / 2 - controller_ur2->q3, 0.);//(0, controller_ur->q2 + M_PI / 2 + controller_ur->q3, 0.)

		cylinder_group2[2].setRotation(slave_base_to_master_base.block<3, 3>(0, 0)* setRPY(cy21rot)* setRPY(cy23rot));
		cylinder_group2[2].setTranslation((slave_base_to_master_base* T21* T20* T22* T23* T23cy3).block<3, 1>(0, 3).transpose());
		//cy4
		fcl::Vector3d cy24rot(0, controller_ur2->q2 - controller_ur2->q3 - controller_ur2->q4, 0.);
		cylinder_group2[3].setRotation(slave_base_to_master_base.block<3,3>(0,0)* setRPY(cy21rot)* setRPY(cy24rot));

		T21_24 = T21 * T22 * T23 * T24;
		cylinder_group2[3].setTranslation((slave_base_to_master_base* T21_24).block<3, 1>(0, 3).transpose());
		//cy5

		cylinder_group2[4].setRotation((slave_base_to_master_base* T21_24* T25* T26).block<3, 3>(0, 0));
		cylinder_group2[4].setTranslation((slave_base_to_master_base* T21_24* T25* T26).block<3, 1>(0, 3).transpose());


		//for (int i = 0; i < 5; i++)
		//{
		//	fcl::Transform3d cy1aabb = cylinder_group2[i].getTransform();
		//	cout << "gourp2 cy" << i << "translation" << endl;
		//	cout << cy1aabb.matrix() << endl;
		//}
		//double dists1_m5 = fcl::distance(&cylinder_group2[0], &cylinder_group1[4], requestS_Mcy5[0], resultS_Mcy5[0]);
		//double dists2_m5 = fcl::distance(&cylinder_group2[1], &cylinder_group1[4], requestS_Mcy5[1], resultS_Mcy5[1]);
		//double dists3_m5 = fcl::distance(&cylinder_group2[2], &cylinder_group1[4], requestS_Mcy5[2], resultS_Mcy5[2]);
		//double dists4_m5 = fcl::distance(&cylinder_group2[3], &cylinder_group1[4], requestS_Mcy5[3], resultS_Mcy5[3]);
		//double dists5_m5 = fcl::distance(&cylinder_group2[4], &cylinder_group1[4], requestS_Mcy5[4], resultS_Mcy5[4]);
		// Show results
		//cout << "dists1_m5:= " << dists1_m5 << "\t min_distance:= " << resultS_Mcy5[0].min_distance << endl;
		//cout << "dists2_m5:= " << dists2_m5 << "\t min_distance:= " << resultS_Mcy5[1].min_distance << endl;
		//cout << "dists3_m5:= " << dists3_m5 << "\t min_distance:= " << resultS_Mcy5[2].min_distance << endl;
		//cout << "dists4_m5:= " << dists4_m5 << "\t min_distance:= " << resultS_Mcy5[3].min_distance << endl;
		//cout << "dists5_m5:= " << dists5_m5 << "\t min_distance:= " << resultS_Mcy5[4].min_distance << endl;

		
		//double dists1_m5 = fcl::distance(&cylinder_group2[2], &cylinder_group2[4], requestS_Mcy5[0], resultS_Mcy5[0]);
		//double dists2_m5 = fcl::distance(&cylinder_group2[1], &cylinder_group2[4], requestS_Mcy5[1], resultS_Mcy5[1]);
		//double dists3_m5 = fcl::distance(&cylinder_group2[0], &cylinder_group2[4], requestS_Mcy5[2], resultS_Mcy5[2]);
		//double dists4_m5 = fcl::distance(&cylinder_group2[1], &cylinder_group2[3], requestS_Mcy5[3], resultS_Mcy5[3]);
		//double dists5_m5 = fcl::distance(&cylinder_group2[0], &cylinder_group2[3], requestS_Mcy5[4], resultS_Mcy5[4]);
		//// Show results
		//cout << "dists1_m5:= " << dists1_m5 << "\t min_distance:= " << resultS_Mcy5[0].min_distance << endl;
		//cout << "dists2_m5:= " << dists2_m5 << "\t min_distance:= " << resultS_Mcy5[1].min_distance << endl;
		//cout << "dists3_m5:= " << dists3_m5 << "\t min_distance:= " << resultS_Mcy5[2].min_distance << endl;
		//cout << "dists4_m5:= " << dists4_m5 << "\t min_distance:= " << resultS_Mcy5[3].min_distance << endl;
		//cout << "dists5_m5:= " << dists5_m5 << "\t min_distance:= " << resultS_Mcy5[4].min_distance << endl;

		//std::cout << resultS_Mcy5[2].nearest_points[0].transpose() << std::endl;
		//std::cout << resultS_Mcy5[2].nearest_points[1].transpose() << std::endl;


		//do
		//{
		//	circle_end_time = boost::posix_time::microsec_clock::local_time(); //boost::get_system_time();
		//	circle_time = circle_end_time - cur_time;
		//	cirtime_ms = (double)circle_time.total_milliseconds() / 1000;
		//	//cout << cirtime_ms << endl;
		//} while (cirtime_ms < 0.001);//0.007  从循环开始到这里0.056S，
	}
}