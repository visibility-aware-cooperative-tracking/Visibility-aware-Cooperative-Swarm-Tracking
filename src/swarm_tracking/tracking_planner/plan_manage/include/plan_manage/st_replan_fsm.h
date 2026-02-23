#ifndef _ST_REPLAN_FSM_H_
#define _ST_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include "ros/subscriber.h"
#include "ros/node_handle.h"
#include "ros/topic_manager.h"
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include "cpu_memory_query.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <ifaddrs.h>

#include <plan_manage/SFCgen_ciri.hpp>
#include <plan_env/vis_sdf.h>
#include <path_searching/prediction.hpp>
#include <path_searching/kino_search.hpp>
#include <path_searching/astar_search.hpp>
#include <traj_utils/visualization.hpp>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/scope_timer.hpp>
#include <traj_utils/traj_set.hpp>
#include <optimizer_corridor/traj_optimizer.h>
#include <traj_utils/poly_traj_utils.hpp>

#include <traj_utils/PolyTraj.h>
#include <swarm_msgs/MincoTraj.h>
#include <swarm_msgs/TeammateIDList.h>
#include <cmath>
#include <rog_map/rog_map.h>

#include <omp.h>



#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "log/"+name))

using std::vector;
typedef std::vector<std::vector<Eigen::Vector3d>> TmtList;
typedef std::vector<VisSDF::Ptr> VSDFList;

namespace st_planner
{

class STReplanFSM
{
public:
  STReplanFSM() {}
  ~STReplanFSM() { 
      if(print_log_)
      {
        replan_log_.close(); 
        fsm_log_.close();
        // cpumem_log_.close();
      }
    }

  void init(ros::NodeHandle &nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE
  {
    INIT,
    WAIT_INIT_TARGET,
    REPLAN_TRAJ,
    EXEC_TRAJ,
    EMERGENCY_STOP,
  };
  /* planning utils */
  display::Visualization::Ptr visualization_;

  ros::NodeHandle nh_;

  rog_map::ROGMapConfig rog_cfg_;
  std::shared_ptr<rog_map::ROGMap> rogmap_;

  kino_search::KinoSearch::Ptr kino_search_;
  astar_search::astarSearch::Ptr astar_search_;
  SFCgen_ciri::SFCgen_ciri::Ptr sfc_gen_;

  // SFCgen::SFCgen::Ptr sfc_gen_;
  prediction::Predict::Ptr predictor_;

  std::mutex exec_state_mutex_, swarm_data_mutex_, replan_mutex_;
  
  traj_opt::TrajOpt::Ptr traj_opt_;
  
  TrajSet traj_set_; //store trajs
  TmtList swarm_tmt_list_; //drone state for in-swarm costs (bearing, formation)


  std::ofstream replan_log_;
  std::ofstream fsm_log_;
  // std::ofstream cpumem_log_;

  /* parameters */
  //workflow control
  double replan_thresh_;
  double emergency_time_;

  bool print_log_;
  bool flag_omni_fov_;
  bool flag_allow_hover_;
  int state_dim_;
  double target_vel_stop_bar_, self_vel_stop_bar_;

  bool have_trigger_, have_target_, have_odom_, have_traj_;
  // bool have_recv_pre_agent_;
  bool have_all_tmt_info_;

  bool use_kino_search_;
  bool force_hover_, wait_hover_;
  int stop_call_num_{0};
  double predict_T_, predict_dt_;
  double desired_dist_, tolerance_d_; 

  double min_flight_height_;
  double fov_ctr_theta_, fov_theta_;
  double LI_extrinsic_;
  double ciri_robot_r_;
  int ciri_iter_num_;
  bool is_kino_path_;

  //Helper params
  int drone_id_;
  int candi_tracker_num_{0};
  int tmt_id_num_{0};
  std::vector<int> candi_tracker_ids_;
  std::vector<int> swarm_id_list_;
  std::vector<int> tmtodom_recv_count_;
  
  Eigen::Vector3d relative_tracking_p_{Eigen::Vector3d(-10000, -10000, -10000)};

  FSM_EXEC_STATE exec_state_;
  int continously_called_times_{0};

  Eigen::Vector3d start_pt_, start_vel_, start_acc_;   // start state
  Eigen::Vector3d end_pt_;                             // goal state
  Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;     // odometry state
  Eigen::Vector3d target_pos_, target_vel_;
  double odom_yaw_;
  double predict_stamp_, target_t_; 
  double replan_time_budget_; 

  //predicted and bias path
  std::vector<Eigen::Vector3d> view_path_, view_vels_, target_path_, target_vels_;
  std::vector<ros::Subscriber> teammate_odom_subscribers_;

  VSDFList vsdfs_; 
  double vsdf_time_cost_{0.0};
  int vsdf_update_num_{0};

  double last_replan_t_{-999.9};//Last replan time
  double last_hover_t_{-999.9};//Last hover time

  //logs
  int replan_attemp_num_{0};
  double search_time_{0.0};
  double corridor_time_{0.0};
  double minco_time_{0.0};

  double average_minco_time_{0.0};
  int trial_num_{0};

  bool trigger_vel_{false};
  bool trigger_dis_{false};

  //cpumem logs
  int current_pid_;
  unsigned long totalcputime_; 
  unsigned long procputime_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, cpumem_timer_;
  ros::Subscriber waypoint_sub_, odom_sub_, trigger_sub_, broadcast_polytraj_sub_;
  ros::Subscriber target_sub_, detect_target_sub_;
  ros::Subscriber teammate_id_list_sub_;
  ros::Publisher poly_traj_pub_, broadcast_polytraj_pub_, ground_height_pub_, heartbeat_pub_;
  ros::Publisher land_sig_pub_;


  /* state machine functions */
  void execFSMCallback(const ros::TimerEvent &e);
  void safetyCheckCallback(const ros::TimerEvent &e);
  void cpumemCallback(const ros::TimerEvent &e);
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  void printFSMExecState();
  std::pair<int, STReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();

  /* input-output */
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void targetCallback(const nav_msgs::OdometryConstPtr &msg);
 
  void triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  void RecvBroadcastMINCOTrajCallback(const swarm_msgs::MincoTrajConstPtr &msg);

  void teammateOdomCallback(const nav_msgs::OdometryConstPtr &msg, int teammate_id);

  /* helper functions */
  void polyTraj2Msg(traj_utils::PolyTraj &poly_msg, swarm_msgs::MincoTraj &MINCO_msg);
  void target2view(const std::vector<Eigen::Vector3d> &target_path,
                  const std::vector<Eigen::Vector3d> &target_vels,
                  std::vector<Eigen::Vector3d> &view_path,
                  std::vector<Eigen::Vector3d> &view_vels);     

  void updateSwarmTmtList(const Eigen::Vector3d &start_p); 
  bool isTargetVisible();
  bool callStop(const Eigen::Vector3d& stop_pos, const double& stop_yaw);
  bool callReplan();

};

} // namespace st_planner

#endif