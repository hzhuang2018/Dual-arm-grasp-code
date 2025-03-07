#pragma once

#include <iostream>
#include <corecrt_math_defines.h>
#include <iostream>
#include <Eigen/Core>
#include<cmath>
#include <math.h>
#include <iostream>
#include <Eigen/Core>

// Collision, Distance 
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>
// Distance Request & Result
#include <fcl/narrowphase/distance_request.h>
#include <fcl/narrowphase/distance_result.h>
//#include <data_types.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include<Eigen/Eigenvalues>

#include "cartesian_space_track_ur.h"
#include <camera.h>
#include <ur_sdk.h>
#include <conio.h>
using namespace Robot;
using namespace octomap;
using namespace std;
class collision_dualarm
{
public:
	double _q[12];//6¹Ø½Ú½Ç
	std::shared_ptr<fcl::CollisionGeometry<double>> cylinder_geometry_base[5], cylinder_geometry2_base[5];//robot 5 
	std::shared_ptr<fcl::CollisionGeometry<double>> box_geometry_base/*, box_geometry_base2*/;//box obstacle

	fcl::CollisionObject<double> * cylinder_group[5];
	fcl::CollisionObject<double> * cylinder_group2[5];
	fcl::CollisionObject<double> *box1,*box2;
	//UR10e
	double d1;
	double L2;
	double L3;
	double d4;
	double d5;
	double d6;

	//offset 
	double shoulder_offset;
	double elbow_offset;

public:
	collision_dualarm();
	~collision_dualarm();
	template <typename S>
	void generateBoxesFromOctomap(std::vector<fcl::CollisionObject<S>*>& env, fcl::OcTree<S>& tree);
	//void setEnvimentFro(std::shared_ptr<fcl::CollisionGeometry<double>>&);
	void setEnviment(std::shared_ptr<fcl::CollisionGeometry<double>> &);
	void setPara(double *);
	//void generateBoxesFromOctomap(std::vector<fcl::CollisionObject<double> *>& boxes, fcl::OcTree<double>& tree);
	Eigen::Matrix3d setRPY(const Eigen::Vector3d rot);
	bool collision_detection();
};

