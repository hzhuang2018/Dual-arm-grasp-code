#include  "planner.h"
#include "collision.h"
	void planner::setStart(double x, double y, double z)
	{
		ob::ScopedState<ob::SE3StateSpace> start(space);
		//start.random();
		start->setXYZ(x, y, z);
		start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
		//start->as<ob::SO3StateSpace::StateType>(1)->setAxisAngle()
		pdef->clearStartStates();
		pdef->addStartState(start);
	}
	void planner::setGoal(double x, double y, double z)
	{
		ob::ScopedState<ob::SE3StateSpace> goal(space);
		goal->setXYZ(x, y, z);
		goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
		pdef->clearGoal();
		pdef->setGoalState(goal);
		std::cout << "goal set to: " << x << " " << y << " " << z << std::endl;
	}
	void planner::updateMap(std::shared_ptr<fcl::CollisionGeometry<double>> map)
	{
		tree_obj = map;
	}
	// Constructor
	planner::planner(CartesianSpaceTrackUR* ur)
	{
		_ur = ur;
		//四旋翼的障碍物几何形状
		Quadcopter = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Sphere<double>(1));
		//分辨率参数设置
		//fcl::OcTree<double> * tree = new fcl::OcTree<double>(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.15)));
		//tree_obj = std::shared_ptr<fcl::CollisionGeometry<double>>(tree);
		tree_obj = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Sphere<double>(0.005));

		//解的状态空间
		space = ob::StateSpacePtr(new ob::SE3StateSpace());

		// create a start state
		ob::ScopedState<ob::SE3StateSpace> start(space);

		// create a goal state
		ob::ScopedState<ob::SE3StateSpace> goal(space);

		// set the bounds for the R^3 part of SE(3)
		// 搜索的三维范围设置
		ob::RealVectorBounds bounds(3);


		//bounds.setLow(-1);
		//bounds.setHigh(1);
		bounds.setLow(0, 0);
		bounds.setHigh(0, 1.2);
		bounds.setLow(1, -1.2);
		bounds.setHigh(1, 1.2);
		bounds.setLow(2, 0);
		bounds.setHigh(2, 1.2);

		//auto space(std::make_shared<ob::SE3StateSpace>());
		//ob::RealVectorBounds bounds(3);
		//bounds.setLow(-1);
		//bounds.setHigh(1);
		//space->setBounds(bounds);

		space->as<ob::SE3StateSpace>()->setBounds(bounds);

		// construct an instance of  space information from this state space
		si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

		start->setXYZ(0, 0, 0);
		start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
		
		//start->rotation(0.3, 0.2, 0.2, 0.55);

		//start.random();

		goal->setXYZ(0.8, 0.8, 0.8);
		goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
		// goal.random();


		// set state validity checking for this space
		si->setStateValidityChecker(std::bind(&planner::isStateValid, this, std::placeholders::_1));
		si->setStateValidityCheckingResolution(0.02);//0.001
		si->setup();

		// create a problem instance
		pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

		// set the start and goal states
		pdef->setStartAndGoalStates(start, goal);

		// set Optimizattion objective
		pdef->setOptimizationObjective(planner::getThresholdPathLengthObj(si));//getThresholdPathLengthObj getPathLengthObjWithCostToGo
	
		std::cout << "Initialized: " << std::endl;
	}
	// Destructor
	planner::~planner()
	{
	}
	void planner::replan(void)
	{

		std::cout << "Total Points:" << path_smooth->getStateCount() << std::endl;
		if (path_smooth->getStateCount() <= 2)
			;// plan();
		else
		{
			for (std::size_t idx = 0; idx < path_smooth->getStateCount(); idx++)
			{
				if (!replan_flag)
					replan_flag = !isStateValid(path_smooth->getState(idx));
				else
					break;

			}
			if (replan_flag)
				;// plan();
			else
				std::cout << "Replanning not required" << std::endl;
		}

	}
	void planner::plan(deque<double> & _path_point)
	{

		// create a planner for the defined space
		og::InformedRRTstar* rrt = new og::InformedRRTstar(si);//InformedRRTstar

		//设置rrt的参数range
		rrt->setRange(0.05);//0.05

		ob::PlannerPtr plan(rrt);

		// set the problem we are trying to solve for the planner
		plan->setProblemDefinition(pdef);

		// perform setup steps for the planner
		plan->setup();

		// print the settings for this space
		si->printSettings(std::cout);

		std::cout << "problem setting\n";
		// print the problem settings
		pdef->print(std::cout);

		// attempt to solve the problem within one second of planning time
		ob::PlannerStatus solved = plan->solve(30);

		if (solved)
		{
			// get the goal representation from the problem definition (not the same as the goal state)
			// and inquire about the found path
			std::cout << "Found solution:" << std::endl;
			ob::PathPtr path = pdef->getSolutionPath();
			og::PathGeometric* pth = pdef->getSolutionPath()->as<og::PathGeometric>();
			//pth->printAsMatrix(std::cout);


			std::ofstream ofs0("path0.txt", std::ios::trunc);
			std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->printAsMatrix(ofs0);
			// print the path to screen
			// path->print(std::cout);


			//nav_msgs::Path msg;
			//msg.header.stamp = ros::Time::now();
			//msg.header.frame_id = "map";
			/*
			for (std::size_t path_idx = 0; path_idx < pth->getStateCount(); path_idx++)
			{
				const ob::SE3StateSpace::StateType* se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

				// extract the first component of the state and cast it to what we expect
				const ob::RealVectorStateSpace::StateType* pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

				// extract the second component of the state and cast it to what we expect
				const ob::SO3StateSpace::StateType* rot = se3state->as<ob::SO3StateSpace::StateType>(1);

				//geometry_msgs::PoseStamped pose;

				//				pose.header.frame_id = "/world"

				//pose.pose.position.x = pos->values[0];
				//pose.pose.position.y = pos->values[1];
				//pose.pose.position.z = pos->values[2];

				//pose.pose.orientation.x = rot->x;
				//pose.pose.orientation.y = rot->y;
				//pose.pose.orientation.z = rot->z;
				//pose.pose.orientation.w = rot->w;

				//msg.poses.push_back(pose);

			}
			//traj_pub.publish(msg);
			*/

			//Path smoothing using bspline
			//B样条曲线优化
			og::PathSimplifier* pathBSpline = new og::PathSimplifier(si);
			path_smooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
			pathBSpline->smoothBSpline(*path_smooth, 3);
			// std::cout << "Smoothed Path" << std::endl;
			// path_smooth.print(std::cout);
			std::ofstream ofs1("pathB.txt", std::ios::trunc);
			//std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->printAsMatrix(ofs0);
			//path_smooth->print(ofs1);
			//Publish path as markers

			//nav_msgs::Path smooth_msg;
			//smooth_msg.header.stamp = ros::Time::now();
			//smooth_msg.header.frame_id = "map";

			for (std::size_t idx = 0; idx < path_smooth->getStateCount(); idx++)
			{
				// cast the abstract state type to the type we expect
				const ob::SE3StateSpace::StateType* se3state = path_smooth->getState(idx)->as<ob::SE3StateSpace::StateType>();

				// extract the first component of the state and cast it to what we expect
				const ob::RealVectorStateSpace::StateType* pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

				// extract the second component of the state and cast it to what we expect
				const ob::SO3StateSpace::StateType* rot = se3state->as<ob::SO3StateSpace::StateType>(1);

				//geometry_msgs::PoseStamped point;

				//pose.header.frame_id = "/world"

				//point.pose.position.x = pos->values[0];
				//point.pose.position.y = pos->values[1];
				//point.pose.position.z = pos->values[2];
				ofs1 << pos->values[0] << " " << pos->values[1] << " " << pos->values[2] << " " << std::endl;
				_path_point.push_back(pos->values[0]);
				_path_point.push_back(pos->values[1]);
				_path_point.push_back(pos->values[2]);
				//point.pose.orientation.x = rot->x;
				//point.pose.orientation.y = rot->y;
				//point.pose.orientation.z = rot->z;
				//point.pose.orientation.w = rot->w;

				//smooth_msg.poses.push_back(point);

				//std::cout << "Published marker: " << idx << std::endl;
			}

			//vis_pub.publish(smooth_msg);
			// ros::Duration(0.1).sleep();


			// Clear memory
			pdef->clearSolutionPaths();
			replan_flag = false;

		}
		else
			std::cout << "No solution found" << std::endl;
	}

	bool planner::isStateValid_backup(const ob::State* state)
	{
		// cast the abstract state type to the type we expect
		const ob::SE3StateSpace::StateType* se3state = state->as<ob::SE3StateSpace::StateType>();

		// extract the first component of the state and cast it to what we expect
		const ob::RealVectorStateSpace::StateType* pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

		// extract the second component of the state and cast it to what we expect
		const ob::SO3StateSpace::StateType* rot = se3state->as<ob::SO3StateSpace::StateType>(1);

		fcl::CollisionObject<double> treeObj((tree_obj));
		fcl::CollisionObject<double> aircraftObject(Quadcopter);
		

		// check validity of state defined by pos & rot
		fcl::Vector3d translation(pos->values[0], pos->values[1], pos->values[2]);
		fcl::Quaterniond rotation(rot->w, rot->x, rot->y, rot->z);
		treeObj.setTransform(rotation, translation);

		fcl::Vector3d translation1(2, 0, 2);
		fcl::Quaterniond rotation1(1, 0, 0, 0);
		aircraftObject.setTransform(rotation1, translation1);

		fcl::CollisionRequest<double> requestType(1, false, 1, false);
		fcl::CollisionResult<double> collisionResult;
		fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);

		return(!collisionResult.isCollision());
	}

	bool planner::isStateValid(const ob::State* state)
	{ 
		// cast the abstract state type to the type we expect
		const ob::SE3StateSpace::StateType* se3state = state->as<ob::SE3StateSpace::StateType>();

		// extract the first component of the state and cast it to what we expect
		const ob::RealVectorStateSpace::StateType* pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

		// extract the second component of the state and cast it to what we expect
		const ob::SO3StateSpace::StateType* rot = se3state->as<ob::SO3StateSpace::StateType>(1);

		// check validity of state defined by pos & rot
		fcl::Vector3d translation(pos->values[0], pos->values[1], pos->values[2]);
		fcl::Quaterniond rotation(rot->w, rot->x, rot->y, rot->z);


		Eigen::Matrix<double, 4, 4> des_Matrix = Eigen::Matrix4d::Identity();

		des_Matrix.block<3, 3>(0, 0) = rotation.toRotationMatrix();
		des_Matrix.block<3, 1>(0, 3) = translation;
		//cout << des_Matrix << endl;
		double desire_q[6];
		if(! _ur->ik_with_q(des_Matrix, desire_q))
		{ 
			//cout << "无解" << endl;
			return false;
		}
			

		Class_collision collision_detecter;
		collision_detecter.setPara(desire_q);
		if (collision_detecter.collision_detection())
		{
			//cout << "有解,检测碰撞时发生碰撞" << endl;
			return false;
		}
			
		//cout << "规划成功一次" << endl;
		return true;
		//fcl::CollisionObject<double> treeObj((tree_obj));
		//fcl::CollisionObject<double> aircraftObject(Quadcopter);

		//

		//treeObj.setTransform(rotation, translation);

		//fcl::Vector3d translation1(2, 0, 2);
		//fcl::Quaterniond rotation1(1, 0, 0, 0);
		//aircraftObject.setTransform(rotation1, translation1);

		//fcl::CollisionRequest<double> requestType(1, false, 1, false);
		//fcl::CollisionResult<double> collisionResult;
		//fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);

		//return(!collisionResult.isCollision());
	}

	// Returns a structure representing the optimization objective to use
	// for optimal motion planning. This method returns an objective which
	// attempts to minimize the length in configuration space of computed
	// paths.
	ob::OptimizationObjectivePtr planner::getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
	{
		ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
		obj->setCostThreshold(ob::Cost(0.000));//1.51
		return obj;
	}

	ob::OptimizationObjectivePtr planner::getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
	{
		ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
		obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
		obj->setCostThreshold(ob::Cost(0.000));
		return obj;
	}

