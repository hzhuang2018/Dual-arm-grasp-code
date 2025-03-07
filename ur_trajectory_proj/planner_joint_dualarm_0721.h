#pragma once
//#include <octomap_msgs/Octomap.h>
//#include <octomap_msgs/conversions.h>
//#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
//#include <message_filters/subscriber.h>
//#include "visualization_msgs/Marker.h"
//#include <trajectory_msgs/MultiDOFJointTrajectory.h>
//#include <nav_msgs/Odometry.h>
//#include <geometry_msgs/Pose.h>
//#include <nav_msgs/Path.h>
//#include <geometry_msgs/PoseStamped.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
// #include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>  
#include <ompl/geometric/planners/rrt/RRTConnect.h> 
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>  
#include <ompl/geometric/planners/prm/LazyPRMstar.h> 
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include "fcl/config.h"
//#include "fcl/octree.h"
//#include "fcl/traversal/traversal_node_octree.h"
//#include "fcl/collision.h"
//#include "fcl/broadphase/broadphase.h"
//#include "fcl/math/transform.h"


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


namespace ob = ompl::base;
namespace og = ompl::geometric;
//namespace oc = ompl::control;
//using namespace fcl;
using namespace octomap;

class planner_joint_dualarm {
public:
	friend class collision_dualarm;
	void setStart(double x, double y, double z, double Rx, double Ry, double Rz);
	void setStart(Eigen::VectorXd rob_q, Eigen::VectorXd rob2_q);
	
	void setGoal(double x, double y, double z, double Rx, double Ry, double Rz,
		double x2, double y2, double z2, double Rx2, double Ry2, double Rz2, double basedRob1);

	void updateMap(std::shared_ptr<fcl::CollisionGeometry<double>> map);

	// Constructor
	//planner_joint(CartesianSpaceTrackUR *ur);
	 planner_joint_dualarm();
	// Destructor
	~planner_joint_dualarm();

	void replan(deque<double>& _path_point);

	void plan(deque<double>& _path_point);
	CartesianSpaceTrackUR* _ur;
	CartesianSpaceTrackUR* _ur2;
	bool is_cartisian_planner;
	Eigen::Matrix<double, 4, 4>  rob_left_base_to_rob_right_base;
private:
	// construct the state space we are planning in
	ob::StateSpacePtr space;


	// construct an instance of  space information from this state space
	ob::SpaceInformationPtr si;
	//og::SimpleSetup ss;
	// create a problem instance
	ob::ProblemDefinitionPtr pdef;

	og::PathGeometric* path_smooth=NULL;

	bool replan_flag = false;

	std::shared_ptr<fcl::CollisionGeometry<double>> Quadcopter;//obstacle model

	std::shared_ptr<fcl::CollisionGeometry<double>> tree_obj;//robot model 

	bool isStateValid(const ob::State* state);
	// Returns a structure representing the optimization objective to use
	// for optimal motion planning. This method returns an objective which
	// attempts to minimize the length in configuration space of computed
	// paths.
	ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si);
	ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si);
};
