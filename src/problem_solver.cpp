#include "ros/ros.h"

#include "ompl/base/StateSpaceTypes.h"
#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/StateValidityChecker.h"

#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/PlannerIncludes.h"

#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"


enum plannerType
{
	RRT = 1,
	RRTConnect = 2,
	PRM = 3
};

class ProblemSolver
{

public:
	ProblemSolver(const plannerType type)
	{
		this->planner = type;
	}

	 void plan(double runTime, plannerType plannerType)
	 {

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

class ValidityChecker : public ompl::base::StateValidityChecker
{

public:
     ValidityChecker(const ompl::base::SpaceInformationPtr& si) :
         ompl::base::StateValidityChecker(si) {}

     // Returns whether the given state's position overlaps the
     // circular obstacle
     bool isValid(const ompl::base::State *state) const override
     {
    	 int i = 2;
         return true;
     }
};

void ProblemSolver::poseSetupCallback(const geometry_msgs::Pose& pose)
{
	if (!this->start_pose_setteled)
	{
		this->start_pose = pose;
		this->start_pose_setteled = true;
	}
	else
	{
		if (!this->start_pose_setteled)
		{
			this->goal_pose = pose;
			this->goal_pose_setteled = true;
		}
	}
}

void ProblemSolver::mapSetupCallback(const nav_msgs::OccupancyGrid& map)
{
	if(!this->map_setteled)
	{
		this->og_map = map;
		this->map_setteled = true;
	}
}

void pickUpPlannerType(int input_arg, plannerType *planner_type)
{
	ROS_INFO("argv: %i", input_arg);
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

	ros::init(argc,  argv, "problem_solver");
	ros::NodeHandle nh;

	plannerType planner_type;

	std::string arg = argv[1];
	int input_arg = stoi(arg);
	pickUpPlannerType(input_arg, &planner_type);

	ProblemSolver PS_object(planner_type);

	ros::Subscriber initial_pose_sub = nh.subscribe("/initial_pose", 2, &ProblemSolver::poseSetupCallback, &PS_object);
	ros::Subscriber initial_map_sub = nh.subscribe("/initial_map", 1, &ProblemSolver::mapSetupCallback, &PS_object);


	ros::spin();

	return 0;
}
