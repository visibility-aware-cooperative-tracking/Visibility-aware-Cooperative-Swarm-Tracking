
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/target_replan_fsm.h>

using namespace st_planner;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "target_planner_node");
  ros::NodeHandle nh("~");

  TargetReplanFSM target_planner;

  target_planner.init(nh);

  // ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}
