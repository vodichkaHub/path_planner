#include "ros/ros.h"

#include "ompl/base/ScopedState.h"
#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/StateValidityChecker.h"
#include <ompl/base/ProblemDefinition.h>

#include "ompl/base/PlannerTerminationCondition.h"

#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/planners/prm/PRM.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/SimpleSetup.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"


enum plannerType
{
	RRT = 1,
	RRTConnect = 2,
	PRM = 3
};

namespace ps
{
	class ProblemSolver
	{

	public:
		ProblemSolver(const plannerType type)
		{
			this->planner = type;
		}

		void plan(const double run_time)
		{
			ROS_INFO("plan running");
			ROS_INFO("%d %d %d", this->map_setteled, this->start_pose_setteled, this->goal_pose_setteled);
			 if (this->map_setteled && this->start_pose_setteled && this->goal_pose_setteled)
			 {
				 ROS_INFO("setteled");
				 auto space(std::make_shared<ompl::base::RealVectorStateSpace>(2));
				 space->setBounds(0.0, (double)this->og_map.info.width);

				 auto si(std::make_shared<ompl::base::SpaceInformation>(space));

				 si->setup();

				 ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(space);
				 start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = this->start_pose.position.x;
				 start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = this->start_pose.position.y;

				 ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(space);
				 goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = this->goal_pose.position.x;
				 goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = this->goal_pose.position.y;

				 auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
				 pdef->setStartAndGoalStates(start, goal);

				 auto planner(std::make_shared<ompl::geometric::RRT>(si));

				 planner->setProblemDefinition(pdef);

				 auto ss(std::make_shared<ompl::geometric::SimpleSetup>(si));

//				 si->

				 auto t_cond(ompl::base::timedPlannerTerminationCondition(run_time));
				 ompl::base::PlannerStatus solved = planner->solve(t_cond);

				 ROS_INFO("plan started %i", solved);
				 if (solved)
				 {
					 ROS_INFO("SOLVED %f", pdef->getSolutionPath()->length());
				 }
			 }
		 }

		void poseSetupCallback(const geometry_msgs::Pose&);
		void mapSetupCallback(const nav_msgs::OccupancyGrid&);

	private:

		nav_msgs::OccupancyGrid og_map;
		bool map_setteled = false;

		geometry_msgs::Pose start_pose;
		bool start_pose_setteled = false;

		geometry_msgs::Pose goal_pose;
		bool goal_pose_setteled = false;

		plannerType planner = RRT;
	};

	//class ValidityChecker : public ompl::base::StateValidityChecker
	//{
	//
	//public:
	//     ValidityChecker(const ompl::base::SpaceInformationPtr& si) :
	//         ompl::base::StateValidityChecker(si) {}
	//
	//     bool isValid(const ompl::base::State* state) const override
	//     {
	//    	 return this->clearance(state) > 0.0;
	//     }

	//     double clearance(const ompl::base::State* state) const override
	//     {
	//
	//     }
	//};

	void ProblemSolver::poseSetupCallback(const geometry_msgs::Pose& pose)
	{
		if (!this->start_pose_setteled)
		{
			this->start_pose = pose;
			this->start_pose_setteled = true;
		}
		else
		{
			if (!this->goal_pose_setteled)
			{
				this->goal_pose = pose;
				this->goal_pose_setteled = true;
			}
		}
		ROS_INFO("%d %d %d", this->map_setteled, this->start_pose_setteled, this->goal_pose_setteled);
	}

	void ProblemSolver::mapSetupCallback(const nav_msgs::OccupancyGrid& map)
	{
		if(!this->map_setteled)
		{
			this->og_map = map;
			this->map_setteled = true;
		}
		ROS_INFO("%d %d %d", this->map_setteled, this->start_pose_setteled, this->goal_pose_setteled);
	}
} // namespace ps

void pickUpPlannerType(int input_arg, plannerType *planner_type)
{
	switch (input_arg) {
			case 1:
				*planner_type = RRT;
				break;
			case 2:
				*planner_type = RRTConnect;
				break;
			case 3:
				*planner_type = PRM;
				break;
			default:
				*planner_type = RRT;
				break;
		}
}

int main(int argc, char **argv) {

	ros::init(argc,  argv, "path_planner_solver_node");
	ros::NodeHandle nh;

	if (!argc)
	{
		ROS_ERROR("Invaild path planner type argument!");
		ros::shutdown();
	}

	plannerType planner_type;

	std::string arg = argv[1];
	int input_arg = stoi(arg);
	pickUpPlannerType(input_arg, &planner_type);

	ps::ProblemSolver solver(planner_type);

	ros::Subscriber initial_pose_sub = nh.subscribe("/initial_pose", 2, &ps::ProblemSolver::poseSetupCallback, &solver);
	ros::Subscriber initial_map_sub = nh.subscribe("/initial_map", 1, &ps::ProblemSolver::mapSetupCallback, &solver);

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		solver.plan(100.0);

		ros::spinOnce();
		loop_rate.sleep();
	}

//	ros::spin();

	return 0;
}
