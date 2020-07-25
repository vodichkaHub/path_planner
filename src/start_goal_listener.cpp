#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"

#include <memory>

using namespace geometry_msgs;

PoseWithCovarianceStampedConstPtr start_pos = nullptr;
PoseStampedConstPtr goal_pos = nullptr;
nav_msgs::OccupancyGridConstPtr OG = nullptr;

void callbackS(const PoseWithCovarianceStampedConstPtr& start)
{
	start_pos = start;
}

void callbackG(const PoseStampedConstPtr& goal)
{
	goal_pos = goal;
}

void callbackM(const nav_msgs::OccupancyGridConstPtr& map)
{
	OG = map;
}

int main(int argc, char **argv) {
	ros::init(argc,  argv, "setup_initial");
	ros::NodeHandle nh;

	ros::Subscriber start_sub = nh.subscribe("/initialpose", 1, &callbackS);
	ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, &callbackG);
	ros::Subscriber map_sub = nh.subscribe("/map", 1, &callbackM);

	ros::Publisher initial_pose_pub = nh.advertise<geometry_msgs::Pose>("initial_pose", 2);
	ros::Publisher initial_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("initial_map", 1);

	ros::Rate loop_rate(10);


	while (ros::ok())
	{
		if (start_pos && goal_pos && OG)
		{
			initial_pose_pub.publish(start_pos->pose.pose);
			initial_pose_pub.publish(goal_pos->pose);
			initial_map_pub.publish(OG);
			break;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
//	ros::spin();

	return 0;
}


