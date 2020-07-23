#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/String.h"

#include <memory>

using namespace geometry_msgs;

PoseWithCovarianceStampedConstPtr start_pos;
PoseStampedConstPtr goal_pos;

void callbackS(const PoseWithCovarianceStampedConstPtr& start)
{
	start_pos = start;
}

void callbackG(const PoseStampedConstPtr& goal)
{
	goal_pos = goal;
}

int main(int argc, char **argv) {
	ros::init(argc,  argv, "setup_node");
	ros::NodeHandle nh;

	ros::Subscriber start_sub = nh.subscribe("/initialpose", 1, &callbackS);
	ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, &callbackG);

	ros::Publisher initial_conds_pub = nh.advertise<geometry_msgs::Pose>("planner_initial_conds", 2);

	ros::Rate loop_rate(10);


	while (ros::ok())
	{
		if (start_pos && goal_pos)
		{
			initial_conds_pub.publish(start_pos->pose.pose);
			initial_conds_pub.publish(goal_pos->pose);
			break;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
//	ros::spin();

	return 0;
}


