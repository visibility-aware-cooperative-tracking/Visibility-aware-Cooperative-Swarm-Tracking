#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/st_replan_fsm.h>

using namespace st_planner;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "st_planner_node");
  ros::NodeHandle nh("~");

  STReplanFSM st_planner;
  st_planner.init(nh);

  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::Duration(0.1).sleep();
  ros::waitForShutdown();

  return 0;
}


