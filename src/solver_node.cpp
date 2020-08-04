#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <ompl-1.5/ompl/base/PlannerStatus.h>
#include <ompl-1.5/ompl/base/PlannerTerminationCondition.h>
#include <ompl-1.5/ompl/base/ProblemDefinition.h>
#include <ompl-1.5/ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl-1.5/ompl/base/ScopedState.h>
#include <ompl-1.5/ompl/base/SpaceInformation.h>
#include <ompl-1.5/ompl/base/State.h>
#include <ompl-1.5/ompl/base/StateValidityChecker.h>
#include <ompl-1.5/ompl/geometric/planners/rrt/RRT.h>
#include <ompl-1.5/ompl/geometric/PathGeometric.h>

#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/rate.h>
#include <ros/subscriber.h>
#include <ros/time.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

enum plannerType
{
	RRT = 1,
	RRTConnect = 2,
	PRM = 3
};

namespace ps
{
	class ValidityChecker : public ompl::base::StateValidityChecker
	{

	public:
		 ValidityChecker(const ompl::base::SpaceInformationPtr& si, std::shared_ptr<costmap_2d::Costmap2DROS> cm) :
			 ompl::base::StateValidityChecker(si)
		 {
			 costmap = cm;
			 map = costmap->getCostmap();
		 }

		 bool isValid(const ompl::base::State* state) const override
		 {
			 double x = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
			 double y = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

			 int cell_cost = map->getCost(floor(x), floor(y));

			 return cell_cost == costmap_2d::FREE_SPACE;
		 }

	private:
		 std::shared_ptr<costmap_2d::Costmap2DROS> costmap;
		 costmap_2d::Costmap2D *map;
	};

	class ProblemSolver
	{

	public:
		ProblemSolver(const plannerType type, const ros::Publisher *pub)
		{
			publisher = pub;
			planner = type;
		}

		void plan(const double run_time)
		{
			 auto space(std::make_shared<ompl::base::RealVectorStateSpace>(2));
			 space->setBounds(0.0, (double)this->og_map->info.width);

			 auto si(std::make_shared<ompl::base::SpaceInformation>(space));

			 tf::TransformListener tf(ros::Duration(1));
			 auto costmap(std::make_shared<costmap_2d::Costmap2DROS>("costmap", *tf.getTF2BufferPtr()));
			 si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(new ps::ValidityChecker(si, costmap)));

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

			 auto t_cond(ompl::base::timedPlannerTerminationCondition(run_time));
			 ompl::base::PlannerStatus solved = planner->solve(t_cond);


			 if (pdef->hasSolution())
			 {
				 this->problem_solved = true;
				 setSolutionPath(pdef);
				 resetPoseSetteld();
			 }
		 }

		void publish(const std::vector<ompl::base::RealVectorStateSpace::StateType *> path)
		{
			if (this->problem_solved)
			{
				auto path_msg(std::make_shared<nav_msgs::Path>());
				auto pose(std::make_shared<geometry_msgs::PoseStamped>());

				for (int i = 0; i < path.size(); ++i) {
					pose->header.stamp = ros::Time();
					pose->header.frame_id = "map";
					pose->pose.position.x = path.at(i)->values[0];
					pose->pose.position.y = path.at(i)->values[1];

					path_msg->poses.push_back(*pose);
				}
				path_msg->header.stamp = ros::Time();
				path_msg->header.frame_id = "map";
				publisher->publish(*path_msg);
			}
		}

		void setSolutionPath(std::shared_ptr<ompl::base::ProblemDefinition> pdef)
		{
			 ompl::geometric::PathGeometric path_geometric(dynamic_cast<const ompl::geometric::PathGeometric &>(*pdef->getSolutionPath()));
			 const std::vector<ompl::base::State*> &states = path_geometric.getStates();
			 ompl::base::State *state;

			 std::vector<ompl::base::RealVectorStateSpace::StateType *> result;

			for (int i = 0; i < states.size(); ++i) {
				state = states[i]->as<ompl::base::State>();
				result.push_back(state->as<ompl::base::RealVectorStateSpace::StateType>());
			}
//
			this->publish(result);
		}

		void startPoseSetupCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr&);
		void goalPoseSetupCallback(const geometry_msgs::PoseStampedConstPtr&);
		void mapSetupCallback(const nav_msgs::OccupancyGridConstPtr&);

		bool problem_solved = false;
		bool og_map_setteled = false;
		bool start_pose_setteled = false;
		bool goal_pose_setteled = false;
	private:

		void resetPoseSetteld ()
		{
			this->start_pose_setteled = false;
			this->goal_pose_setteled = false;
		}
		nav_msgs::OccupancyGridConstPtr og_map;

		geometry_msgs::Pose start_pose;

		geometry_msgs::Pose goal_pose;

		plannerType planner = RRT;

		const ros::Publisher *publisher;
	};

	void ProblemSolver::startPoseSetupCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& start)
	{
		if (!this->start_pose_setteled)
		{
			this->start_pose = start->pose.pose;
			this->start_pose_setteled = true;
			ROS_INFO("start set");
		}
	}

	void ProblemSolver::goalPoseSetupCallback(const geometry_msgs::PoseStampedConstPtr& goal)
	{
		if (!this->goal_pose_setteled)
		{
			this->goal_pose = goal->pose;
			this->goal_pose_setteled = true;
			this->problem_solved = false;
			ROS_INFO("goal set");
		}
	}

	void ProblemSolver::mapSetupCallback(const nav_msgs::OccupancyGridConstPtr& map)
	{
		if (!this->og_map_setteled)
		{
			this->og_map = map;
			this->og_map_setteled = true;
			ROS_INFO("map set");
		}
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

	if (!argc)
	{
		ROS_ERROR("Invaild path planner type argument!");
		ros::shutdown();
	}
	ros::init(argc,  argv, "path_planner_solver_node");
	ros::NodeHandle nh;


	plannerType planner_type;

	std::string arg = argv[1];
	int input_arg = stoi(arg);

	pickUpPlannerType(input_arg, &planner_type);

	ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/solver_path", 1000);

	ps::ProblemSolver solver(planner_type, &path_pub);

	ros::Subscriber initial_pose_sub = nh.subscribe("/map", 2, &ps::ProblemSolver::mapSetupCallback, &solver);
	ros::Subscriber start_sub = nh.subscribe("/initialpose", 1, &ps::ProblemSolver::startPoseSetupCallback, &solver);
	ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, &ps::ProblemSolver::goalPoseSetupCallback, &solver);

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		if (solver.start_pose_setteled && solver.goal_pose_setteled && solver.og_map_setteled && !solver.problem_solved)
		{
			std::clock_t t_start = std::clock();
			solver.plan(200.0);
			std::clock_t t_end = std::clock();

			double runtime = ((double)t_end - (double)t_start) / (double)CLOCKS_PER_SEC;
			ROS_INFO("Runtime: %f seconds", runtime);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}


//	ros::spin();

	return 0;
}
