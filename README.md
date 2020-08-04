# ROS path planner

## Tools:
 - OMPL
 - costmap_2D
 
## Usage
 1. Run:
 ```roslaunch /{your catkin workspace dir}/src/path_planner/run.launch planner:=1```
 
 Where planner is: *{RRT=1, RRTConnect=2, PRM=3}*.
 
 2. Add screen by topic **/map** (Rviz)
 3. Add screen by topic **/solver_path** (Rviz)
 4. Point start with *2D Pose Estimate* interface (Rviz)
 5. Point goal with *2D Nav Goal* interface (Rviz)
 
 ---
 
![](./rviz_screenshot_2020_08_04-18_54_45.png)
