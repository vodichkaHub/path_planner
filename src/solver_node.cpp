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
#include <ompl-1.5/ompl/geometric/planners/prm/PRM.h>
#include <ompl-1.5/ompl/geometric/planners/rrt/RRT.h>
#include <ompl-1.5/ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl-1.5/ompl/geometric/PathGeometric.h>

#include "path_planner/PlannerTypeConfig.h"
#include "path_planner/PlannerType.h"

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

#include "dynamic_reconfigure/Config.h"

namespace ps
{
	enum plannerType
	{
		RRT = 1,
		RRTConnect = 2,
		PRM = 3
	};

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
		ProblemSolver(const ros::Publisher *pub)
		{
			publisher = pub;
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

			 switch (this->planner_type) {
				case ps::plannerType::RRT:
				{
					auto planner_rrt(std::make_unique<ompl::geometric::RRT>(si));
					solve(planner_rrt, pdef, run_time);
					break;
				}
				case ps::plannerType::RRTConnect:
				{
					auto planner_rrtc(std::make_unique<ompl::geometric::RRTConnect>(si));
					solve(planner_rrtc, pdef, run_time);
					break;
				}
				case ps::plannerType::PRM:
				{
					auto planner_prm(std::make_unique<ompl::geometric::PRM>(si));
					solve(planner_prm, pdef, run_time);
					break;
				}
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
		void plannerTypeSetupCallback(const dynamic_reconfigure::ConfigConstPtr&);

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

		template<class T>
		void solve(T &planner, std::shared_ptr<ompl::base::ProblemDefinition> pdef, const double run_time)
		{
			planner->setProblemDefinition(pdef);

			 auto t_cond(ompl::base::timedPlannerTerminationCondition(run_time));
			 ompl::base::PlannerStatus solved = planner->solve(t_cond);


			 if (pdef->hasSolution())
			 {
				 planner->printSettings(std::cout);
				 this->problem_solved = true;
				 setSolutionPath(pdef);
				 resetPoseSetteld();
			 }
		}

		nav_msgs::OccupancyGridConstPtr og_map;

		geometry_msgs::Pose start_pose;

		geometry_msgs::Pose goal_pose;

		plannerType planner_type;

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


	void ProblemSolver::plannerTypeSetupCallback(const dynamic_reconfigure::ConfigConstPtr& type)
	{
		switch (type->ints.at(0).value) {
			case 1:
				this->planner_type = ps::plannerType::RRT;
				break;
			case 2:
				this->planner_type = ps::plannerType::RRTConnect;
				break;
			case 3:
				this->planner_type = ps::plannerType::PRM;
				break;
		}
		ROS_INFO("planner type set");
	}
} // namespace ps

int main(int argc, char **argv) {

	ros::init(argc,  argv, "path_planner_solver_node");
	ros::NodeHandle nh;

	ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/solver_path", 1000);

	ps::ProblemSolver solver(&path_pub);

	ros::Subscriber initial_pose_sub = nh.subscribe("/map", 2, &ps::ProblemSolver::mapSetupCallback, &solver);
	ros::Subscriber start_sub = nh.subscribe("/initialpose", 1, &ps::ProblemSolver::startPoseSetupCallback, &solver);
	ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, &ps::ProblemSolver::goalPoseSetupCallback, &solver);
	ros::Subscriber planner_type_sub = nh.subscribe("/path_planner_cfg_node/parameter_updates", 1, &ps::ProblemSolver::plannerTypeSetupCallback, &solver);

	ros::Rate loop_rate(10);

	double runtime;
	std::clock_t t_start;
	std::clock_t t_end;

	while (ros::ok())
	{
		if (solver.start_pose_setteled && solver.goal_pose_setteled && solver.og_map_setteled && !solver.problem_solved)
		{
			t_start = std::clock();
			solver.plan(200.0);
			t_end = std::clock();

			runtime = ((double)t_end - (double)t_start) / (double)CLOCKS_PER_SEC;
			ROS_INFO("Runtime: %f seconds", runtime);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
