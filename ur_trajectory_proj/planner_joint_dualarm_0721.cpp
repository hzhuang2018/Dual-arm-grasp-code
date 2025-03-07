#include  "planner_joint_dualarm.h"
#include "collision_dualarm.h"
#include <opencv2/core/eigen.hpp>
//#include "C:\3rdparties\opencv\include\opencv2\core\eigen.hpp"
	void planner_joint_dualarm::setStart(double x, double y, double z, double Rx, double Ry, double Rz)
	{
		double df_pos_orient[6];
		//set target oritention
		df_pos_orient[3] = 3.14159;//R ORIENT  3.14159
		df_pos_orient[4] = 0;//P ORIENT   0.0 
		df_pos_orient[5] = -1.57;//Y ORIENT  -1.57
		//Eigen::Vector3d eulerAngle_start_point = Rc.eulerAngles(2, 1, 0);
		Eigen::Vector3d eulerAngle(df_pos_orient[3], df_pos_orient[4], df_pos_orient[5]);
		Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
		Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
		Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
		Eigen::AngleAxisd rotation_vector;
		rotation_vector = yawAngle * pitchAngle * rollAngle;//ZYX
		Eigen::Matrix3d orient_rotation_matrix = Eigen::Matrix3d::Identity();
		orient_rotation_matrix = rotation_vector.matrix();
		Eigen::Matrix<double, 4, 4> start_Matrix = Eigen::Matrix4d::Identity();
		start_Matrix.block<3, 3>(0, 0) = orient_rotation_matrix;
		start_Matrix(0, 3) = x;
		start_Matrix(1, 3) = y;
		start_Matrix(2, 3) = z;
		cout << "start_m" << start_Matrix << endl;
		//ob::ScopedState<ob::SE3StateSpace> start(space);
		////start.random();
		//start->setXYZ(x, y, z);
		//start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
		//start->as<ob::SO3StateSpace::StateType>(1)->setAxisAngle()
		pdef->clearStartStates();
		double desire_q[6];
		_ur->ik_with_q(start_Matrix, desire_q);
		ob::ScopedState<ob::RealVectorStateSpace> start_joint_space(space);//ob::ScopedState<ob::RealVectorStateSpace> start(space);
		//start_joint_space[0] = -0.472459; start_joint_space[1] = -1.63; start_joint_space[2] = -2.347;
		//start_joint_space[3] = -0.733; start_joint_space[4] = 1.5708; start_joint_space[5] = 2.66993;
		for (int i = 0; i < 6; i++)
		{
			start_joint_space[i] = desire_q[i];
		}
		pdef->addStartState(start_joint_space);
	}

	void planner_joint_dualarm::setStart(Eigen::VectorXd rob_q, Eigen::VectorXd rob2_q)
	{

		pdef->clearStartStates();
		Eigen::VectorXd desire_q;
		desire_q.resize(6);
		desire_q = rob_q;// _ur->get_q();
		ob::ScopedState<ob::RealVectorStateSpace> start_joint_space(space);
		for (int i = 0; i < 6; i++)
		{
			start_joint_space[i] =  desire_q[i];
			start_joint_space[i+6] = rob2_q[i];
			//cout << desire_q[i];
		}
		pdef->addStartState(start_joint_space);
	}

	void planner_joint_dualarm::setGoal(double x, double y, double z, double Rx, double Ry, double Rz,
					double x2, double y2, double z2, double Rx2, double Ry2, double Rz2,
					double basedRob1	)
	{

//cv::Mat myvec(3, 1, CV_64FC1) ;
//cv::Mat myresult(3, 1, CV_64FC1);
//myvec.at<double>(0, 0) = 1.921;
//myvec.at<double>(1, 0) = -1.929;
//myvec.at<double>(2, 0) = -0.42;
//cv::Mat Rccc(3, 3, CV_64FC1);
//cv::Mat Rccc1(3, 3, CV_64FC1);
//cv::Rodrigues(myvec, Rccc);
//cv::Rodrigues(Rccc, myresult);
//cv::Rodrigues(myresult, Rccc1);
//cout << Rccc << endl;
//cout << myresult << endl;
//cout << Rccc1 << endl;




		double df_pos_orient[6];
	
		cv::Mat theta(3, 1, CV_64FC1), theta2(3, 1, CV_64FC1);
		cv::Mat Rc(3, 3, CV_64FC1), Rc2(3, 3, CV_64FC1);
		theta.at<double>(0, 0) = Rx;
		theta.at<double>(1, 0) = Ry;
		theta.at<double>(2, 0) = Rz;

		theta2.at<double>(0, 0) = Rx2;
		theta2.at<double>(1, 0) = Ry2;
		theta2.at<double>(2, 0) = Rz2;

		cv::Rodrigues(theta, Rc);
		cv::Rodrigues(theta2, Rc2);
		Eigen::Matrix3d orient_rotation_matrix1 = Eigen::Matrix3d::Identity();
		Eigen::Matrix3d orient_rotation_matrix2 = Eigen::Matrix3d::Identity();
		cv::cv2eigen( Rc, orient_rotation_matrix1);
		cv::cv2eigen(Rc2, orient_rotation_matrix2);
		//orient_rotation_matrix1(0, 0) = Rc.at<double>(0, 0); orient_rotation_matrix1(0, 1) = Rc.at<double>(0, 1); orient_rotation_matrix1(0, 2) = Rc.at<double>(0, 2);
		//orient_rotation_matrix1(1, 0) = Rc.at<double>(1, 0); orient_rotation_matrix1(1, 1) = Rc.at<double>(1, 1); orient_rotation_matrix1(1, 2) = Rc.at<double>(1, 2);
		//orient_rotation_matrix1(2, 0) = Rc.at<double>(2, 0); orient_rotation_matrix1(2, 1) = Rc.at<double>(2, 1); orient_rotation_matrix1(2, 2) = Rc.at<double>(2, 2);
		//cout <<"eigen" <<orient_rotation_matrix1 << endl;
		Eigen::Matrix<double, 4, 4> goal_Matrix = Eigen::Matrix4d::Identity();
		Eigen::Matrix<double, 4, 4> goal_Matrix2 = Eigen::Matrix4d::Identity();
		goal_Matrix.block<3, 3>(0, 0) = orient_rotation_matrix1;
		goal_Matrix(0, 3) = x;		goal_Matrix(1, 3) = y;		goal_Matrix(2, 3) = z;
		//2号机器人的位姿，这里是用1号的基系描述
		goal_Matrix2.block<3, 3>(0, 0) = orient_rotation_matrix2;
		goal_Matrix2(0, 3) = x2;		goal_Matrix2(1, 3) = y2;		goal_Matrix2(2, 3) = z2;
		Eigen::Matrix4d rob2_tran;
		if (basedRob1)
			rob2_tran = rob_left_base_to_rob_right_base * goal_Matrix2;
		else
			rob2_tran =  goal_Matrix2;
		cout << goal_Matrix2 << endl;
		cout << rob2_tran << endl;
		double desire_q[6], desire2_q[6];
		_ur->ik_with_q(goal_Matrix, desire_q);
		_ur2->ik_with_q(rob2_tran, desire2_q);

		
		ob::ScopedState<ob::RealVectorStateSpace> goal_joint_space(space);//ob::ScopedState<ob::RealVectorStateSpace> start(space);

		for (int i = 0; i < 6; i++)
		{
			goal_joint_space[i] = desire_q[i];
			goal_joint_space[i+6] = desire2_q[i];
		}

		pdef->clearGoal();

		pdef->setGoalState(goal_joint_space);
		std::cout << "goal set to: " <<goal_joint_space << std::endl;
	}

	void planner_joint_dualarm::updateMap(std::shared_ptr<fcl::CollisionGeometry<double>> map)
	{
		std::shared_ptr<fcl::CollisionGeometry<double>> cylinder_geometry_base1(new fcl::Cylinder<double>(0.075, 0.27));

		tree_obj = map;

		
	}
	// Constructor
	//planner_joint::planner_joint(CartesianSpaceTrackUR* ur)
	planner_joint_dualarm::planner_joint_dualarm()
	{
		
		rob_left_base_to_rob_right_base << 0.988042, 0.15257, 0.0222721, -1.91979,
										  -0.152422, 0.988281, -0.00819861, 0.209646,
										 -0.0232619, 0.00470581, 0.999718, 0.0139865,
											0, 0, 0, 1;

		is_cartisian_planner = false;
		//四旋翼的障碍物几何形状
		Quadcopter = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Sphere<double>(1));
		tree_obj = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Sphere<double>(0.005));

		//解的状态空间
		space = ob::StateSpacePtr(new ob::RealVectorStateSpace(12));//RealVectorStateSpace
		// create a start state
		//ob::ScopedState<ob::SE3StateSpace> start(space);
		ob::ScopedState<ob::RealVectorStateSpace> start(space);
		// create a goal state
		//ob::ScopedState<ob::SE3StateSpace> goal(space);
		ob::ScopedState<ob::RealVectorStateSpace> goal(space);

		// 搜索的三维范围设置
		ob::RealVectorBounds bounds(12);

		//bounds for robot 1
		bounds.setLow(0, -2*M_PI);
		bounds.setHigh(0, 2*M_PI);
		bounds.setLow(1, -M_PI-M_PI/6);
		bounds.setHigh(1, 0+M_PI / 6);
		bounds.setLow(2, - M_PI);
		bounds.setHigh(2, M_PI);
		bounds.setLow(3, -2 * M_PI);
		bounds.setHigh(3, 2 * M_PI);
		bounds.setLow(4, -2 * M_PI);
		bounds.setHigh(4, 2 * M_PI);
		bounds.setLow(5, -2 * M_PI);
		bounds.setHigh(5, 2 * M_PI);
		//bounds for robot 2
		bounds.setLow(0 + 6, -2 * M_PI);
		bounds.setHigh(0 + 6, 2 * M_PI);
		bounds.setLow(1 + 6, -M_PI - M_PI / 6);
		bounds.setHigh(1 + 6, 0 + M_PI / 6);
		bounds.setLow(2 + 6, -M_PI);
		bounds.setHigh(2 + 6, M_PI);
		bounds.setLow(3 + 6, -2 * M_PI);
		bounds.setHigh(3 + 6, 2 * M_PI);
		bounds.setLow(4 + 6, -2 * M_PI);
		bounds.setHigh(4 + 6, 2 * M_PI);
		bounds.setLow(5 + 6, -2 * M_PI);
		bounds.setHigh(5 + 6, 2 * M_PI);

		space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
		// construct an instance of  space information from this state space
		si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

		start.random();

		goal.random();


		// set state validity checking for this space
		si->setStateValidityChecker(std::bind(&planner_joint_dualarm::isStateValid, this, std::placeholders::_1));
		si->setStateValidityCheckingResolution(0.02);//0.001(最近用的）  0.02
		si->setup();

		// create a problem instance
		pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

		// set the start and goal states
		pdef->setStartAndGoalStates(start, goal);

		// set Optimizattion objective
		pdef->setOptimizationObjective(planner_joint_dualarm::getThresholdPathLengthObj(si));//getThresholdPathLengthObj getPathLengthObjWithCostToGo
	
		std::cout << "Initialized: " << std::endl;
	}
	// Destructor
	planner_joint_dualarm::~planner_joint_dualarm()
	{
	}
	void planner_joint_dualarm::replan(deque<double>& _path_point)
	{

		std::cout << "Total Points:" << path_smooth->getStateCount() << std::endl;
		if (path_smooth->getStateCount() <= 2)
			 plan(_path_point);
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
				 plan(_path_point);
			else
				std::cout << "Replanning not required" << std::endl;
		}

	}
	void planner_joint_dualarm::plan(deque<double> & _path_point)
	{

		// create a planner for the defined space
		og::ABITstar* rrt = new og::ABITstar(si);//InformedRRTstar  LBKPIECE1  PRMstar  KPIECE1 SBL LazyPRMstar  RRTConnect BITstar  ABITstar
		
		//设置rrt的参数range
		//rrt->setRange(0.02);//0.05  0.02(最近值） 
		
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
		ob::PlannerStatus solved = plan->solve(1);

		if (solved)
		{
			// get the goal representation from the problem definition (not the same as the goal state)
			// and inquire about the found path
			std::cout << "Found solution:" << std::endl;
			ob::PathPtr path = pdef->getSolutionPath();
			og::PathGeometric* pth = pdef->getSolutionPath()->as<og::PathGeometric>();
			//pth->printAsMatrix(std::cout);


			std::ofstream ofs0("path0.txt", std::ios::trunc);
			//std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->printAsMatrix(ofs0);
			for (std::size_t path_idx = 0; path_idx < pth->getStateCount(); path_idx++)
			{
				const ob::RealVectorStateSpace::StateType* se3state = pth->getState(path_idx)->as<ob::RealVectorStateSpace::StateType>();
			}

			// print the path to screen
			// path->print(std::cout);

			//Path smoothing using bspline
			//B样条曲线优化
			og::PathSimplifier* pathBSpline = new og::PathSimplifier(si);
			path_smooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
			pathBSpline->smoothBSpline(*path_smooth, 3);
			// std::cout << "Smoothed Path" << std::endl;
			// path_smooth.print(std::cout);
			std::ofstream ofs1("pathB.txt", std::ios::trunc);
			std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->printAsMatrix(ofs0);
			
			//path_smooth->print(ofs1);

			for (std::size_t idx = 0; idx < path_smooth->getStateCount(); idx++)
			{
				const ob::RealVectorStateSpace::StateType* se3state = path_smooth->getState(idx)->as<ob::RealVectorStateSpace::StateType>();

				// extract the first component of the state and cast it to what we expect
				//const ob::RealVectorStateSpace::StateType* pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

				_path_point.push_back(se3state->values[0]);
				_path_point.push_back(se3state->values[1]);
				_path_point.push_back(se3state->values[2]);
				_path_point.push_back(se3state->values[3]);
				_path_point.push_back(se3state->values[4]);
				_path_point.push_back(se3state->values[5]);

				_path_point.push_back(se3state->values[0 + 6]);
				_path_point.push_back(se3state->values[1 + 6]);
				_path_point.push_back(se3state->values[2 + 6]);
				_path_point.push_back(se3state->values[3 + 6]);
				_path_point.push_back(se3state->values[4 + 6]);
				_path_point.push_back(se3state->values[5 + 6]);

				ofs1 << se3state->values[0] << " " << se3state->values[1] << " " << se3state->values[2] << " " <<
						se3state->values[3] << " " << se3state->values[4] << " " << se3state->values[5] << " " <<
						se3state->values[0 + 6] << " " << se3state->values[1 + 6] << " " << se3state->values[2 + 6] << " " <<
						se3state->values[3 + 6] << " " << se3state->values[4 + 6] << " " << se3state->values[5 + 6] << " " <<
						std::endl;

			}

			// Clear memory
			pdef->clearSolutionPaths();
			replan_flag = false;
		
		}
		else
			std::cout << "No solution found" << std::endl;
	}

	bool planner_joint_dualarm::isStateValid(const ob::State* state)
	{ 
		//// cast the abstract state type to the type we expect
		const ob::RealVectorStateSpace::StateType* se3state = state->as<ob::RealVectorStateSpace::StateType>();

		double desire_q[12];

		//const ob::RealVectorStateSpace::StateType* se3state = pth->getState(path_idx)->as<ob::RealVectorStateSpace::StateType>();

		// extract the first component of the state and cast it to what we expect
		//const ob::RealVectorStateSpace::StateType* pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

		desire_q[0] = se3state->values[0];
		desire_q[1] = se3state->values[1];
		desire_q[2] = se3state->values[2];
		desire_q[3] = se3state->values[3];
		desire_q[4] = se3state->values[4];
		desire_q[5] = se3state->values[5];

		desire_q[6] = se3state->values[0 + 6];
		desire_q[7] = se3state->values[1 + 6];
		desire_q[8] = se3state->values[2 + 6];
		desire_q[9] = se3state->values[3 + 6];
		desire_q[10] = se3state->values[4 + 6];
		desire_q[11] = se3state->values[5 + 6];

		collision_dualarm collision_detecter;
		collision_detecter.setEnviment(tree_obj);
		collision_detecter.setPara(desire_q);
		if (collision_detecter.collision_detection())
		{
			cout << "有解,检测碰撞时发生碰撞" << endl;
			return false;
		}
			
		cout << "规划成功一次" << endl;
		return true;

	}

	// Returns a structure representing the optimization objective to use
	// for optimal motion planning. This method returns an objective which
	// attempts to minimize the length in configuration space of computed
	// paths.
	ob::OptimizationObjectivePtr planner_joint_dualarm::getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
	{
		ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
		obj->setCostThreshold(ob::Cost(0.000));//1.51
		return obj;
	}

	ob::OptimizationObjectivePtr planner_joint_dualarm::getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
	{
		ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
		obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
		//obj->setCostThreshold(ob::Cost(0.000));
		return obj;
	}

