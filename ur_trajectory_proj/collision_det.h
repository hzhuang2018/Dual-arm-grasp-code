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


Eigen::Matrix3d setRPY(const Eigen::Vector3d rot);
void collision_detection(const CartesianSpaceTrackUR* controller_ur, const  CartesianSpaceTrackUR* controller_ur2);