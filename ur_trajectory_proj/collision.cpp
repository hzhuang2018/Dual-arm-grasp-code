#include "collision.h"
#include "planner_joint.h"
extern planner_joint  myplanner;//planner_joint   planner_cartisian
Class_collision::Class_collision()
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
	//������Բ����뾶���0.02,��Ϊ��ֱ������ײ��⣬�þ�����ʱ̫��ʱ��
		// base1 Cylinder
	cylinder_geometry_base[0] = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Cylinder<double>(0.075, 0.27));
	// base2 
	cylinder_geometry_base[1] = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Cylinder<double>(0.055, 0.74));
	// base3
	cylinder_geometry_base[2] = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Cylinder<double>(0.045+0.02, 0.675));
	// base4
	cylinder_geometry_base[3] = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Cylinder<double>(0.045+0.02, 0.12));
	// base5
	cylinder_geometry_base[4] = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Cylinder<double>(0.045+0.02, 0.17));
	//box
	box_geometry_base = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Box<double>(0.32, 0.22, 0.385));//(0.4,0.4,0.3)
	box_geometry_base2 = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Box<double>(0.68,0.135,0.4257));
	for (int i = 0; i < 5; i++)
		cylinder_group[i] = new  fcl::CollisionObject<double>(cylinder_geometry_base[i]);

	// box1= fcl::CollisionObject<double>(box_geometry_base);
	//box1 = new fcl::CollisionObject<double>(std::shared_ptr<fcl::CollisionGeometry<double>>(box_geometry_base));
	//myplanner.tree_obj.toBoxes();






	//box1 = new fcl::CollisionObject<double>(std::shared_ptr<fcl::CollisionGeometry<double>>(myplanner.tree_obj));
	//box1->toBoxes();
	//cout<< box1->getCollisionGeometry();
	//cout << box1->getAABB();
	//box2 = new fcl::CollisionObject<double>(std::shared_ptr<fcl::CollisionGeometry<double>>(box_geometry_base2));
}

void Class_collision::setEnviment(std::shared_ptr<fcl::CollisionGeometry<double>> & fcl_tree)
{
	box_geometry_base = fcl_tree;
	box1 = new fcl::CollisionObject<double>(std::shared_ptr<fcl::CollisionGeometry<double>>(fcl_tree));
	//fcl::OcTree<double> fcl_test(octomap::OcTree ;
	//fcl_test.toBoxes();
	//generateBoxesFromOctomap();
}


//==============================================================================
template <typename S>
void Class_collision::generateBoxesFromOctomap(std::vector<fcl::CollisionObject<S>*>& boxes, fcl::OcTree<S>& tree)
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


void Class_collision::setPara(double * q)
{
	for (int i = 0; i < 6; i++)
		_q[i] = q[i];


}
Eigen::Matrix3d Class_collision::setRPY(const Eigen::Vector3d rot)
{
	Eigen::Matrix3d ret;
	ret = Eigen::AngleAxisd(rot.x(), Eigen::Vector3d::UnitX())
		* Eigen::AngleAxisd(rot.y(), Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(rot.z(), Eigen::Vector3d::UnitZ());
	return ret;
};
bool Class_collision::collision_detection()
{
	boost::posix_time::ptime start_time = boost::get_system_time();

	//Sleep(1);
	//setPara();
	// Distance Request and Result 
	fcl::DistanceRequest<double> requestMaster[6];
	fcl::DistanceResult<double>  resultMaster[6];
	//fcl::CollisionRequest<double>  requestMaster[6];
	fcl::CollisionRequest<double> coll_requestMaster(1, false, 1, false);
	fcl::CollisionResult<double>  coll_resultMaster;

	for (int i = 0; i < 6; i++)
	{
		requestMaster[i].enable_nearest_points = true;
		requestMaster[i].enable_signed_distance = true;
		requestMaster[i].gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
		// request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD; GST_INDEP
		resultMaster[i].clear();
	}

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
	T3 << cos(_q[2]), -sin(_q[2]), 0, L2,//cylinder2 length��-0.7����������double L2 = -0.6127;���� double L3 = -0.57155;
		sin(_q[2]), cos(_q[2]), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	T2cy2 << cos(_q[2]), -sin(_q[2]), 0, L2 * 0.5,//cylinder2 length��-0.7����������double L2 = -0.6127;���� double L3 = -0.57155;
		sin(_q[2]), cos(_q[2]), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	T3cy3 << 1, 0, 0, L3 * 0.5,//-0.65  
		0, 1, 0, 0,
		0, 0, 1, -elbow_offset,
		0, 0, 0, 1;
	T4 << cos(_q[3]), -sin(_q[3]), 0, L3,//cylinder3 length
		sin(_q[3]), cos(_q[3]), 0, 0,
		0, 0, 1, d4,//d4 =0.17415  -����ƽ��  0.12415
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

	fcl::Vector3d cy2rot(0, -_q[1] - M_PI / 2, 0.);//cy2rot(0, controller_ur->q2 + M_PI / 2, 0.);
	cylinder_group[1]->setRotation(setRPY(cy1rot) * setRPY(cy2rot));

	cylinder_group[1]->setTranslation((T1 * T0 * T2 * T2cy2).block<3, 1>(0, 3).transpose());//ƽ�������
	//cout << (T21 * T20 * T22 * T23).block<3, 1>(0, 3).transpose() << endl;
	fcl::Vector3d cy3rot(0, -_q[1] - M_PI / 2 - _q[2], 0.);//(0, controller_ur->q2 + M_PI / 2 + controller_ur->q3, 0.)
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

	//fcl::Vector3d box_trans2(0.84, 0., 0.215);
	//box1->setTranslation(box_trans2);

	//fcl::Vector3d box_trans3(0.9897, 0.6352+0.13, 0.213);
	//box2->setTranslation(box_trans3);
	//�Ƿ����ϰ�����ײ

	//box1->setTranslation(fcl::Vector3d(0,0,0));
	//box1->computeAABB();
	//cout<<box1->getAABB().min_;
	//cout << box1->getAABB().max_;
	//fcl::Transform3<double> box1_trans = box1->getTransform();
	//cout << box1_trans.translation();
	//box1->computeAABB();
	//fcl::AABB<double> box1_aabb =  box1->getAABB();
	//cout << "width:=" << box1_aabb.width() << endl;
	//cout << "depth:=" << box1_aabb.depth() << endl;
	//cout << "heigh:=" << box1_aabb.height() <<endl;
	//cout << "center:=" << box1_aabb.center() << endl;
	//cout << "center:=" << box1_aabb.min_.x() << box1_aabb.min_.y() << box1_aabb.min_.z() << endl;
	//cout << "center:=" << box1_aabb.max_.x() << box1_aabb.max_.y() << box1_aabb.max_.z() << endl;
	for (int i = 1; i < 5; i++)
	{
		bool dist1 = fcl::collide(box1, cylinder_group[i], coll_requestMaster, coll_resultMaster);
		//double dist1 = fcl::distance(box1, cylinder_group[i], requestMaster[i], resultMaster[i]);//box1
		//double dist2 = fcl::distance(box2, cylinder_group[i], requestMaster[i], resultMaster[i]);//box2
		//cout << "cylinder_group1: " << i << "\t box1: " << "  " << dist1 << "\t" << resultMaster[i].min_distance << endl;
		if (coll_resultMaster.isCollision())//||dist2 < 0.1||dist3 < 0.1||dist4 < 0.1
		//if (dist1 < 0.03 )//|| dist2 < 0.03)
			 return true;//������ײ  cout << "���ϰ�����ײ" << endl;//
	}
	//�Ƿ�������ײ
	//for (int i =0; i < 3; i++)
	//{
	//	for (int j = i+2; j < 5; j++)
	//	{
	//		bool dist1 = fcl::collide(cylinder_group[i], cylinder_group[j], coll_requestMaster, coll_resultMaster);
	//		//double dist1 = fcl::distance(cylinder_group[i], cylinder_group[j], requestMaster[i], resultMaster[i]);
	//		//cout << "cylinder_group1: " << i << "\t box1: " << "  " << dist1 << "\t" << resultMaster[i].min_distance << endl;
	//		if (coll_resultMaster.isCollision())//||dist2 < 0.1||dist3 < 0.1||dist4 < 0.1
	//		//if (dist1 < 0.1)
	//		return true;  //cout << "������ײ" << endl;
	//	}
	//}
	boost::posix_time::ptime cur_time = boost::get_system_time();
	boost::posix_time::time_duration d_time;
	d_time = cur_time - start_time;
	double d_ms = (double)d_time.total_milliseconds() / 1000;
	//printf("collistion time: %f\n", d_ms);
	return false;//û����ײ
};