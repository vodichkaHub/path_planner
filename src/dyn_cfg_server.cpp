#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <ros/publisher.h>

#include <path_planner/PlannerTypeConfig.h>
#include <path_planner/PlannerType.h>

void callback(path_planner::PlannerTypeConfig &config, uint32_t level) {

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_planner_cfg_node");
  ros::NodeHandle nh;

  dynamic_reconfigure::Server<path_planner::PlannerTypeConfig> server;
  dynamic_reconfigure::Server<path_planner::PlannerTypeConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ros::spin();
  return 0;
}
