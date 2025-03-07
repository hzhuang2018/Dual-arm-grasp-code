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
class Class_collision
{
public:
	double _q[6];//6¹Ø½Ú½Ç
	std::shared_ptr<fcl::CollisionGeometry<double>> cylinder_geometry_base[5];//robot 5 
	std::shared_ptr<fcl::CollisionGeometry<double>> box_geometry_base;//box obstacle
	//std::shared_ptr<fcl::CollisionObject<double>> cylinder_group[5];
	//std::shared_ptr<fcl::CollisionObject<double>> box1;
	//CollisionObject* obj = new CollisionObject(std::shared_ptr<CollisionGeometry>(box), Transform3f(Vec3f(x, y, z)));

	fcl::CollisionObject<double> * cylinder_group[5];
	fcl::CollisionObject<double> *box1;
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
	Class_collision();
	void setPara(double *);
	Eigen::Matrix3d setRPY(const Eigen::Vector3d rot);
	bool collision_detection();
};

