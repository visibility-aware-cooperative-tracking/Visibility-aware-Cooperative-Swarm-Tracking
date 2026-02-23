#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "rog_map/rog_map.h"
#include <omp.h>
#include <plan_env/vis_sdf.h>
#include <traj_utils/visualization.hpp>
#include <traj_utils/scope_timer.hpp>


VisSDF::Ptr vsdf_;
rog_map::ROGMap::Ptr rogmap_;
display::Visualization::Ptr visualization_;
ros::Timer VSDF_timer_;
Eigen::Vector3d center_pos_;

void timerCallback(const ros::TimerEvent &e)
{
  cout<< " -- Updating SSDF."<<endl;
  
  int idx = 0;
  vsdf_->generateSDF(center_pos_); 

  vsdf_->publishSDFwithColorTF();
  vsdf_->publishSpherewithColorTF();

  visualization_->visualize_a_ball(center_pos_, 0.12, "target_center", display::red);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ssdf_example_node");
  ros::NodeHandle nh("~");

  cout<< " -- Init ROGMap Begin."<<endl;
  rog_map::ROGMapConfig cfg;
  rog_map::ROSParamLoader ld(nh, cfg);
  rogmap_.reset(new rog_map::ROGMap(nh, cfg));
  visualization_.reset(new display::Visualization(nh));

  vsdf_.reset(new VisSDF);
  vsdf_->initSDF(nh, rogmap_);
  center_pos_ << 0.0 ,  0.0 ,  0.0;

  VSDF_timer_ = nh.createTimer(ros::Duration(3.0), &timerCallback);
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::Duration(1.0).sleep();
  ros::waitForShutdown();
  return 0;
}


