#ifndef _TARGET_REPLAN_FSM_H_
#define _TARGET_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <rog_map/rog_map.h>
#include <path_searching/prediction.hpp>
#include <path_searching/kino_search.hpp>
#include <path_searching/astar_search.hpp>
#include <plan_manage/SFCgen_ciri.hpp>
#include <geometry_msgs/PoseStamped.h>

#include <traj_utils/planning_visualization.h>
#include <traj_utils/visualization.hpp>
#include <traj_utils/traj_set.hpp>

#include <optimizer_corridor/traj_optimizer.h>
#include <traj_utils/poly_traj_utils.hpp>

#include <traj_utils/PolyTraj.h>
#include <cmath>

#include <stdlib.h>
#include <time.h>

using std::vector;
typedef std::vector<std::vector<Eigen::Vector3d>> TmtList;

namespace st_planner
{

class TargetReplanFSM
{
public:
  TargetReplanFSM() {}
  ~TargetReplanFSM() {}

  void init(ros::NodeHandle &nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE
  {
    INIT,
    WAIT_TARGET,
    GEN_NEW_TRAJ,
    REPLAN_TRAJ,
    EXEC_TRAJ,
    EMERGENCY_STOP,
    SEQUENTIAL_START
  };
  /* planning utils */
  PlanningVisualization::Ptr visualization_;
  display::Visualization::Ptr visualization2_;

  rog_map::ROGMapConfig rog_cfg_;
  std::shared_ptr<rog_map::ROGMap> rogmap_;
  astar_search::astarSearch::Ptr astar_search_;
  SFCgen_ciri::SFCgen_ciri::Ptr sfc_gen_;
  traj_opt::TrajOpt::Ptr traj_opt_;
  
  TrajSet traj_set_; //store trajs
  TmtList tmt_list_;

  /* parameters */
  int target_type_; 
  double no_replan_thresh_, replan_thresh_;
  double waypoints_[50][3];
  int waypoint_num_, wpt_id_;
  double planning_horizen_;
  double emergency_time_;
  bool flag_realworld_experiment_;
  bool enable_fail_safe_;
  bool enable_ground_height_measurement_; 
  bool flag_escape_emergency_;

  double max_follow_dist_;
  double corridor_bound_;
  
  //Helper params for server
  int drone_id_;
  Eigen::Vector3d relative_tracking_p_{Eigen::Vector3d(-10000, -10000, -10000)};
  Eigen::Vector3d object_p_{Eigen::Vector3d(-10000, -10000, -10000)};
  Eigen::Vector3d object_v_{Eigen::Vector3d(-10000, -10000, -10000)};
  Eigen::Quaterniond object_q_{Eigen::Quaterniond(1, 0, 0, 0)};

  bool have_trigger_, have_goal_, have_odom_, have_recv_pre_agent_, have_traj_;

  FSM_EXEC_STATE exec_state_;
  int continously_called_times_{0};

  Eigen::Vector3d start_pt_, start_vel_, start_acc_;   // start state
  Eigen::Vector3d end_pt_;                             // goal state
  Eigen::Vector3d local_target_pt_, local_target_vel_; // local target state
  Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;     // odometry state
  Eigen::Vector3d target_pos_, target_vel_;
  double target_t_; // target state

  //predicted path
  std::vector<Eigen::Vector3d> predict_path_, predict_vels_;

  Eigen::Vector3d last_predict_vel_;
  double predict_T_;
  double predict_dt_;
  double last_replan_t_;//Last replan time
  double global_time_start_;//For reference traj


  //As target 
  Eigen::Vector3d nav_goal_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_;
  ros::Subscriber waypoint_sub_, odom_sub_, trigger_sub_, broadcast_ploytraj_sub_, mandatory_stop_sub_, object_sub_, goal_sub_;
  ros::Publisher poly_traj_pub_, data_disp_pub_, broadcast_ploytraj_pub_, heartbeat_pub_, goal_pub_, ground_height_pub_;


  
  ros::Timer ciri_timer_;

  /* state machine functions */
  void execFSMCallback(const ros::TimerEvent &e);
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  void printFSMExecState();
  std::pair<int, TargetReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();


  /* input-output */
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr& msgPtr);


  /* prediction */
  bool predictPath(std::vector<Eigen::Vector3d> &predict_path, std::vector<Eigen::Vector3d> &predict_vels_);
  bool predictPathCurve(std::vector<Eigen::Vector3d> &predict_path, std::vector<Eigen::Vector3d> &predict_vels_);

  /* helper functions */
  void polyTraj2Msg(traj_utils::PolyTraj &poly_msg);
};

} // namespace st_planner

#endif