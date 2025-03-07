#include "collision_dualarm.h"
#include "planner_joint_dualarm.h"
//extern planner_joint  myplanner;//planner_joint   planner_cartisian
collision_dualarm::collision_dualarm()
{
	//UR10e
	d1 = 0.1807;
	L2 = -0.6127;
	L3 = -0.57155;
	d4 = 0.17415;
	d5 = 0.11985;
	d6 = 0.11655;

	//offset 
	shoulder_offset = 0.176;//0.18
	elbow_offset = 0.137;//0.13
	//后三个圆柱体半径添加0.02,是为了直接用碰撞检测，用距离检测时太费时间
		// base1 Cylinder
	cylinder_geometry_base[0] =  std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Cylinder<double>(0.075, 0.27)); 
	cylinder_geometry2_base[0] = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Cylinder<double>(0.075, 0.27));
	// base2 
	cylinder_geometry_base[1] =  std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Cylinder<double>(0.055, 0.74));
	cylinder_geometry2_base[1] = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Cylinder<double>(0.055, 0.74));
	// base3
	cylinder_geometry_base[2] =  std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Cylinder<double>(0.045 + 0.02, 0.675));
	cylinder_geometry2_base[2] = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Cylinder<double>(0.045+0.02, 0.675));
	// base4
	cylinder_geometry_base[3] = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Cylinder<double>(0.045 + 0.02, 0.12));
	cylinder_geometry2_base[3] = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Cylinder<double>(0.045+0.02, 0.12));
	// base5
	cylinder_geometry_base[4] = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Cylinder<double>(0.045+0.05, 0.17+0.05));
	cylinder_geometry2_base[4] = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Cylinder<double>(0.045 + 0.05, 0.17+0.05));
	//box
	box_geometry_base = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Box<double>(0.32, 0.22, 0.385));//(0.4,0.4,0.3)
	//box_geometry_base2 = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Box<double>(0.68,0.135,0.4257));
	for (int i = 0; i < 5; i++)
	{
		cylinder_group[i]  = new  fcl::CollisionObject<double>(cylinder_geometry_base[i]);
		cylinder_group2[i] = new  fcl::CollisionObject<double>(cylinder_geometry2_base[i]);
	}
		

}
collision_dualarm::~collision_dualarm() {
}
void collision_dualarm::setEnviment(std::shared_ptr<fcl::CollisionGeometry<double>> & fcl_tree)
{
	box_geometry_base = fcl_tree;
	box1 = new fcl::CollisionObject<double>(std::shared_ptr<fcl::CollisionGeometry<double>>(fcl_tree));
}


//==============================================================================
template <typename S>
void collision_dualarm::generateBoxesFromOctomap(std::vector<fcl::CollisionObject<S>*>& boxes, fcl::OcTree<S>& tree)
{
	std::vector<std::array<S, 6> > boxes_ = tree.toBoxes();

	for (std::size_t i = 0; i < boxes_.size(); ++i)
	{
		S x = boxes_[i][0];
		S y = boxes_[i][1];
		S z = boxes_[i][2];
		S size = boxes_[i][3];
		S cost = boxes_[i][4];
		S threshold = boxes_[i][5];

		fcl::Box<S>* box = new fcl::Box<S>(size, size, size);
		box->cost_density = cost;
		box->threshold_occupied = threshold;
		fcl::CollisionObject<S>* obj = new fcl::CollisionObject<S>(std::shared_ptr<fcl::CollisionGeometry<S>>(box), fcl::Transform3<S>(fcl::Translation3<S>(fcl::Vector3<S>(x, y, z))));
		boxes.push_back(obj);
	}

	std::cout << "boxes size: " << boxes.size() << std::endl;

}


void collision_dualarm::setPara(double * q)
{
	for (int i = 0; i < 12; i++)
		_q[i] = q[i];


}
Eigen::Matrix3d collision_dualarm::setRPY(const Eigen::Vector3d rot)
{
	Eigen::Matrix3d ret;
	ret = Eigen::AngleAxisd(rot.x(), Eigen::Vector3d::UnitX())
		* Eigen::AngleAxisd(rot.y(), Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(rot.z(), Eigen::Vector3d::UnitZ());
	return ret;
};
bool collision_dualarm::collision_detection()
{
	//return true;
	//Eigen::Matrix<double, 4, 4> T1, T2, T3, T4, T5, T6, T;
	boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
	boost::posix_time::ptime cur_time, circle_end_time;
	boost::posix_time::time_duration d_time, circle_time;
	double d_ms, cirtime_ms;

	fcl::CollisionRequest<double> coll_requestMaster(1, false, 1, false);
	fcl::CollisionResult<double>  coll_resultMaster;



	Eigen::Matrix<double, 4, 4>  rob_left_base_to_rob_right_base, slave_base_to_master_base;
	rob_left_base_to_rob_right_base << 0.988042, 0.15257, 0.0222721, -1.91979,
		-0.152422, 0.988281, -0.00819861, 0.209646,
		-0.0232619, 0.00470581, 0.999718, 0.0139865,
		0, 0, 0, 1;
	slave_base_to_master_base = rob_left_base_to_rob_right_base.inverse();


		cur_time = boost::posix_time::microsec_clock::local_time();
		d_time = cur_time - start_time;
		d_ms = (double)d_time.total_milliseconds() / 1000;
		//printf("time: %f\n", d_ms);

		Eigen::Matrix<double, 4, 4>T1_temp, T1_trans, T2_temp, T2_trans, T3_temp, T4_temp, T5_temp;
		Eigen::Matrix<double, 4, 4> T0, T1, T2, T3, T4, T5, T6, T2cy2, T3cy3, T1_4;

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
		T1 << cos(_q[0]), -sin(_q[0]), 0, 0,
			sin(_q[0]), cos(_q[0]), 0, 0,
			0, 0, 1, d1,
			0, 0, 0, 1;
		T2 << cos(_q[1]), -sin(_q[1]), 0, 0,
			0, 0, -1, 0,
			sin(_q[1]), cos(_q[1]), 0, 0,
			0, 0, 0, 1;
		//cout << T21*T20*T22 << endl << endl;
		T3 << cos(_q[2]), -sin(_q[2]), 0, L2,//cylinder2 length　-0.7　　　　　double L2 = -0.6127;　　 double L3 = -0.57155;
			sin(_q[2]), cos(_q[2]), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

		T2cy2 << cos(_q[2]), -sin(_q[2]), 0, L2 * 0.5,//cylinder2 length　-0.7　　　　　double L2 = -0.6127;　　 double L3 = -0.57155;
			sin(_q[2]), cos(_q[2]), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

		T3cy3 << 1, 0, 0, L3 * 0.5,//-0.65  
			0, 1, 0, 0,
			0, 0, 1, -elbow_offset,
			0, 0, 0, 1;
		T4 << cos(_q[3]), -sin(_q[3]), 0, L3,//cylinder3 length
			sin(_q[3]), cos(_q[3]), 0, 0,
			0, 0, 1, d4,//d4 =0.17415  -两次平移  0.12415
			0, 0, 0, 1;

		T5 << cos(_q[4]), -sin(_q[4]), 0, 0,
			0, 0, -1, -d5,
			sin(_q[4]), cos(_q[4]), 0, 0,
			0, 0, 0, 1;
		T6 << cos(_q[5]), -sin(_q[5]), 0, 0,
			0, 0, 1, d6 - 0.17 / 2,//d6 =0.11655
			-sin(_q[5]), -cos(_q[5]), 0, 0,
			0, 0, 0, 1;

		cylinder_group[0]->setTranslation(T1_temp.block<3, 1>(0, 3).transpose());
		cylinder_group[0]->setRotation(T1_temp.block<3, 3>(0, 0));

		//cylinder2.setRotation((T21*T20*T22 * T23).block<3, 3>(0, 0));
		fcl::Vector3d cy1rot(0, 0, _q[0]);

		fcl::Vector3d cy2rot(0, -_q[1]- M_PI / 2, 0.);//cy2rot(0, _q[]2 + M_PI / 2, 0.);
		cylinder_group[1]->setRotation(setRPY(cy1rot) * setRPY(cy2rot));
		cylinder_group[1]->setTranslation((T1 * T0 * T2 * T2cy2).block<3, 1>(0, 3).transpose());//平移用这个
		//cout << (T21 * T20 * T22 * T23).block<3, 1>(0, 3).transpose() << endl;
		fcl::Vector3d cy3rot(0, -_q[1] - M_PI / 2 - _q[2], 0.);//(0, _q[]2 + M_PI / 2 + _q[]3, 0.)
		cylinder_group[2]->setRotation(setRPY(cy1rot) * setRPY(cy3rot));
		cylinder_group[2]->setTranslation((T1 * T0 * T2 * T3 * T3cy3).block<3, 1>(0, 3).transpose());

		fcl::Vector3d cy4rot(0, _q[1] - _q[2] - _q[3], 0.);
		T1_4 = T1 * T2 * T3 * T4;
		cylinder_group[3]->setRotation(setRPY(cy1rot) * setRPY(cy4rot));
		cylinder_group[3]->setTranslation((T1_4).block<3, 1>(0, 3).transpose());

		fcl::Vector3d cy55rot(0, 0, _q[0] + _q[4]);
		fcl::Vector3d cy5rot(M_PI / 2, _q[1] + M_PI / 2 + _q[2] + _q[3] + M_PI / 2, 0);
		//cylinder5.setRotation(setRPY(cy55rot) * setRPY(cy5rot));
		cylinder_group[4]->setRotation((T1 * T2 * T3 * T4 * T5 * T6).block<3, 3>(0, 0));
		cylinder_group[4]->setTranslation((T1_4 * T5 * T6).block<3, 1>(0, 3).transpose());

		//fcl::Vector3d box_trans2(0.7, 0., 0.);
		//box1.setTranslation(box_trans2);

		//set cylinder_group2 rotation and translation 
		Eigen::Matrix<double, 4, 4> T20, T21, T22, T23, T24, T25, T26, T22cy2, T23cy3, T21_24;
		T20 << 1, 0, 0, 0,
			0, 1, 0, -shoulder_offset,
			0, 0, 1, 0,
			0, 0, 0, 1;
		T21 << cos(_q[6]), -sin(_q[6]), 0, 0,
			sin(_q[6]), cos(_q[6]), 0, 0,
			0, 0, 1, d1,
			0, 0, 0, 1;
		T22 << cos(_q[7]), -sin(_q[7]), 0, 0,
			0, 0, -1, 0,
			sin(_q[7]), cos(_q[7]), 0, 0,
			0, 0, 0, 1;
		//cout << T21*T20*T22 << endl << endl;
		T23 << cos(_q[8]), -sin(_q[8]), 0, L2,//cylinder2 length　-0.7　　　　　double L2 = -0.6127;　　 double L3 = -0.57155;
			sin(_q[8]), cos(_q[8]), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

		T22cy2 << cos(_q[8]), -sin(_q[8]), 0, L2 * 0.5,//cylinder2 length　-0.7　　　　　double L2 = -0.6127;　　 double L3 = -0.57155;
			sin(_q[8]), cos(_q[8]), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
		T23cy3 << 1, 0, 0, L3 * 0.5,//-0.65  
			0, 1, 0, 0,
			0, 0, 1, -elbow_offset,
			0, 0, 0, 1;
		T24 << cos(_q[9]), -sin(_q[9]), 0, L3,//cylinder3 length
			sin(_q[9]), cos(_q[9]), 0, 0,
			0, 0, 1, d4,//d4 =0.17415  -两次平移  0.12415
			0, 0, 0, 1;

		T25 << cos(_q[10]), -sin(_q[10]), 0, 0,
			0, 0, -1, -d5,
			sin(_q[10]), cos(_q[10]), 0, 0,
			0, 0, 0, 1;
		T26 << cos(_q[11]), -sin(_q[11]), 0, 0,
			0, 0, 1, d6 - 0.17 / 2,//d6 =0.11655
			-sin(_q[11]), -cos(_q[11]), 0, 0,
			0, 0, 0, 1;

		//cy1  slave_base_to_master_base*
		cylinder_group2[0]->setTranslation((slave_base_to_master_base * T1_temp).block<3, 1>(0, 3).transpose());
		cylinder_group2[0]->setRotation((slave_base_to_master_base * T1_temp).block<3, 3>(0, 0));
		//cy2
		fcl::Vector3d cy21rot(0, 0, _q[6]);
		fcl::Vector3d cy22rot(0, -_q[7] - M_PI / 2, 0.);
		//cy2rot<<(0, -_q[]2 - M_PI / 2, 0.);

		cylinder_group2[1]->setRotation(slave_base_to_master_base.block<3, 3>(0, 0) * setRPY(cy21rot) * setRPY(cy22rot));
		cylinder_group2[1]->setTranslation((slave_base_to_master_base * T21 * T20 * T22 * T22cy2).block<3, 1>(0, 3).transpose());//平移用这个
		//cy3
		fcl::Vector3d cy23rot(0, -_q[7] - M_PI / 2 - _q[8], 0.);//(0, _q[]2 + M_PI / 2 + _q[]3, 0.)

		cylinder_group2[2]->setRotation(slave_base_to_master_base.block<3, 3>(0, 0) * setRPY(cy21rot) * setRPY(cy23rot));
		cylinder_group2[2]->setTranslation((slave_base_to_master_base * T21 * T20 * T22 * T23 * T23cy3).block<3, 1>(0, 3).transpose());
		//cy4
		fcl::Vector3d cy24rot(0, _q[7] - _q[8] - _q[9], 0.);
		cylinder_group2[3]->setRotation(slave_base_to_master_base.block<3, 3>(0, 0) * setRPY(cy21rot) * setRPY(cy24rot));

		T21_24 = T21 * T22 * T23 * T24;
		cylinder_group2[3]->setTranslation((slave_base_to_master_base * T21_24).block<3, 1>(0, 3).transpose());
		//cy5

		cylinder_group2[4]->setRotation((slave_base_to_master_base * T21_24 * T25 * T26).block<3, 3>(0, 0));
		cylinder_group2[4]->setTranslation((slave_base_to_master_base * T21_24 * T25 * T26).block<3, 1>(0, 3).transpose());
		
		//障碍物与两个机器人分别进行检测
		for (int i = 2; i < 5; i++)
		{
			bool dist1 = fcl::collide(box1, cylinder_group[i], coll_requestMaster, coll_resultMaster);
			if (coll_resultMaster.isCollision())//||dist2 < 0.1||dist3 < 0.1||dist4 < 0.1
				return true;//发生碰撞  cout << "与障碍物碰撞" << endl;//
			dist1 = fcl::collide(box1, cylinder_group2[i], coll_requestMaster, coll_resultMaster);
			if (coll_resultMaster.isCollision())//||dist2 < 0.1||dist3 < 0.1||dist4 < 0.1
				return true;//发生碰撞  cout << "与障碍物碰撞" << endl;//
		}
		//两个机器人最后三个圆柱体分别进行检测
		for (int i = 2; i < 5; i++)
			for (int j = 2; j < 5; j=j+2){
			bool dist1 = fcl::collide(cylinder_group[i], cylinder_group2[j], coll_requestMaster, coll_resultMaster);
			if (coll_resultMaster.isCollision())//||dist2 < 0.1||dist3 < 0.1||dist4 < 0.1
				return true;//发生碰撞  cout << "与障碍物碰撞" << endl;//

			}



	
		{
			circle_end_time = boost::posix_time::microsec_clock::local_time(); //boost::get_system_time();
			circle_time = circle_end_time - start_time;
			cirtime_ms = (double)circle_time.total_milliseconds() / 1000;
			cout << cirtime_ms << endl;
		} 
		return false;//没有碰撞
};