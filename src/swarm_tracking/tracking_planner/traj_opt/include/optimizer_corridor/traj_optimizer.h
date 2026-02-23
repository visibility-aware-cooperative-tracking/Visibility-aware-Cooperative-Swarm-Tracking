#pragma once
#include <ros/ros.h>

#include "minco.hpp"
#include "flatmap.hpp"
#include <random>
#include <ros/package.h>
#include <optimizer_corridor/geoutils.hpp>
#include <optimizer_corridor/lbfgs_raw.hpp>
#include <traj_utils/traj_set.hpp>
#include <traj_utils/scope_timer.hpp>
#include "rog_map/rog_map.h"
#include <plan_env/vis_sdf.h>

typedef std::vector<std::vector<Eigen::Vector3d>> TmtList;
typedef std::vector<VisSDF::Ptr> VSDFList;
namespace traj_opt {

struct facet {
  Eigen::Vector3d a_;
  Eigen::Vector3d b_;
  Eigen::Vector3d c_;
};

class TrajOpt {
 public:
  ros::NodeHandle nh_;
  std::shared_ptr<rog_map::ROGMap> rogmap_;
  flatness::FlatnessMap flatmap_;

  int N_, K_, dim_t_, dim_p_, dim_yaw_;
  double vmax_, amax_, omega_max_;

  //Opt Weights
  double rhoP_, rhoV_, rhoA_, rhoT_, rhoSwarm_, rhoOmega_;
  double rhoDistance_, rhoOcclusion_, rhoSeparation_, rhoVSDF_;
  double rhoFOV_vertical_, rhoFOV_horizontal_;
  double rhoFOV_vertical_init_, rhoFOV_horizontal_init_;

  double tracking_dist_, tolerance_d_;

  double corridor_clearance_, swarm_clearance_, occlusion_clearance_;
  double vsdf_clearance_;

  double min_flight_height_;
  double tracking_dt_, tracking_T_;
  double LI_extrinsic_;
  double sep_theta_{0.0};
  
  double fov_theta_, fov_ctr_theta_;
  double des_delta_h_;

  bool have_swarm_penal_{true}; 
  bool is_fov_omni_;
  bool use_init_opt_;
  int max_outloop_num_;
  int idx_start_fov_cost_;

  int out_loop_num_{0};
  double init_overhead_{0.0};
  double regu_overhead_{0.0};

  // corridor
  std::vector<Eigen::MatrixXd> cfgVs_;
  std::vector<Eigen::MatrixXd> cfgHs_;
  // Minimum Snap Optimizer
  minco::MINCO_S4 snapOpt_;  //R3 traj
  minco::MINCO_S2 accOpt_;   //yaw traj
  // weight for each vertex
  Eigen::VectorXd p_;
  // duration of each piece of the trajectory
  Eigen::VectorXd t_;
  // yaw angle
  Eigen::VectorXd yaw_;
   // tail acc
  Eigen::Vector3d tailA_;
  // tail vel
  Eigen::Vector3d tailV_;
  // bound states
  Eigen::MatrixXd iniS_;
  Eigen::MatrixXd finS_;
  // temporal grad 
  Eigen::VectorXd gradT_;

  double* x_;
  double sum_T_;
  
  double t_traj_start_; 
  double t_replan_start_; 
  double replan_time_budget_;
  int drone_id_, swarm_size_;

  //Tracking points and dt
  std::vector<Eigen::Vector3d> tracking_pts_;
  std::vector<double> init_yaw_path_; 
  std::vector<Eigen::Vector3d> front_path_;

  bool is_init_value_{true};
  bool is_fov_first_{false};
  double accumu_vsdf_distance_;
  std::vector<double> init_opt_res_dis_;
  bool is_init_overtime_{false};

  int init_opt_iternum_;

  double n2_n3_;
  double cos_fov_theta_;
  double costheta_n2n3_{-999};

  bool is_kino_path_{false};
  enum OPT_STATE
  {
    VALID,
    OBS_OCC,
    ZAX_OCC,
    SWM_OCC,
    TGT_OCC,
    OPT_ISS,
    COD_ISS,
    BAD_TMT,
    TIM_OUT
  };
  string opt_state_str_[9] = {"VALID", "OBS occ", "ZAX Occ", "SWM Occ", "TGT Occ", "OPT iss", "COD iss", "Tmt bad", "Tim Out"};
 
  //ptr to swarm data
  SwarmTrajData *swarm_trajs_{NULL};
  TmtList *teammate_list_{NULL}; 
  VSDFList *vsdf_list_{NULL};

  typedef std::shared_ptr<TrajOpt> Ptr;
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  TrajOpt(ros::NodeHandle& nh);
  virtual ~TrajOpt() {}

  //for Tracker Traj
  virtual int generate_traj(const double t_replan_start,
                            const double t_traj_start,
                            const Eigen::MatrixXd& iniState,
                            const Eigen::MatrixXd& finState,
                            const double &traj_duration,
                            const std::vector<Eigen::Vector3d>& front_wpts,
                            const std::vector<Eigen::Vector3d>& tracking_pts,
                            const std::vector<Eigen::MatrixXd>& hPolys,
                            Trajectory& traj,
                            Trajectory1D& traj_yaw,
                            bool is_kino_traj) = 0;

  virtual bool generate_stop_traj(const Eigen::Vector3d& start_pos, const double& start_yaw,
                                  const Eigen::Vector3d& stop_pos, const double& stop_yaw,
                                  Trajectory& traj, Trajectory1D& traj_yaw) = 0;
  virtual bool generate_stop_traj(const Eigen::MatrixXd& initState, const double& duration, 
                                  const Eigen::Vector3d& stop_pos, const double& stop_yaw,
                                  Trajectory& traj, Trajectory1D& traj_yaw) = 0;
  
  virtual bool generate_init_traj(const double t_replan_start,
                                  const Eigen::MatrixXd& iniState,
                                  const Eigen::MatrixXd& finState,
                                  const std::vector<Eigen::Vector3d>& front_wpts,
                                  const std::vector<Eigen::MatrixXd>& hPolys,
                                  Trajectory& traj) = 0;

  virtual bool setInitValues(const Eigen::MatrixXd& iniState, 
                             const Eigen::MatrixXd& finState)   = 0;
  virtual int optimize(const double& delta = 1e-4)              = 0;
  virtual int optimize_init(const double& delta = 1e-4)         = 0;
  virtual void addTimeIntPenalty(double& cost)                  = 0;
  virtual void addTimeCost(double& cost)                        = 0;
  virtual void addTailCost(double& cost)                        = 0;
  virtual void addTimeIntPenalty_init(double& cost)             = 0;
  virtual void addTimeCost_init(double& cost)                   = 0;
  virtual void addTailCost_init(double& cost)                   = 0;

  bool extractVs(const std::vector<Eigen::MatrixXd>& hPs,
                 std::vector<Eigen::MatrixXd>& vPs) const;

  bool grad_cost_corridor(const Eigen::Vector3d& p,
                        const Eigen::MatrixXd& hPoly,
                        Eigen::Vector3d& gradp,
                        double& cost);

  bool grad_cost_v(const Eigen::Vector3d& v,
                   Eigen::Vector3d& gradv,
                   double& cost);
  bool grad_cost_a(const Eigen::Vector3d& a,
                   Eigen::Vector3d& grada,
                   double& cost);

  bool grad_cost_floor(const Eigen::Vector3d& p,
                       Eigen::Vector3d& gradp,
                       double& cost); 

  bool grad_cost_omega(const Eigen::Vector3d& omega,
                       Eigen::Vector3d& gradomega,
                       double& cost);

  bool grad_cost_distance(const Eigen::Vector3d& p,
                          const Eigen::Vector3d& target_p,
                          Eigen::Vector3d& gradp,
                          double& cost);

  bool grad_cost_vsdf(const Eigen::Vector3d& pos,
                      const VisSDF::Ptr& vsdf,
                      Eigen::Vector3d& gradp,
                      double& cost);

  bool grad_cost_FOV(const Eigen::Vector3d& pos, 
                     const Eigen::Vector4d& quat,
                     const Eigen::Vector3d& target,
                     Eigen::Vector3d& gradp,
                     Eigen::Vector4d& gradquat,
                     double& cost);

  bool grad_cost_swarm_collision(double t_g,
                                const Eigen::Vector3d& pos,
                                const Eigen::Vector3d& vel,
                                Eigen::Vector3d& gradp,
                                double &gradt,
                                double &grad_prev_t,
                                double &cost);
  
  bool grad_cost_swarm_bearing(double t_g, 
                               const Eigen::Vector3d& p, 
                               const Eigen::Vector3d& center,
                               Eigen::Vector3d& gradp,
                               double& costp);
  
  bool grad_cost_direct_angular_sep(double t_g, 
                                    const Eigen::Vector3d& p, 
                                    const Eigen::Vector3d& center,
                                    Eigen::Vector3d& gradp,
                                    double& costp);

  int checkCollision(const Trajectory& traj);

  bool isTrajInCorridors(const Trajectory& traj, const double out_margin = 0.01);
  bool isPosInCorridor(const Eigen::Vector3d& pos, const Eigen::MatrixXd& hPoly, const double out_margin);

  //for Target Traj              
  virtual bool generate_traj(const Eigen::MatrixXd& iniState,
                             const Eigen::MatrixXd& finState,
                             const std::vector<Eigen::MatrixXd>& hPolys,
                             Trajectory& traj) = 0;
  virtual void addTargetPenalty(double& cost)  = 0;

  //Helper funcs
  inline double getSwarmClc() { return swarm_clearance_; }
  inline void setEnvironment(const std::shared_ptr<rog_map::ROGMap>& map) { rogmap_ = map; }
  inline void setSwarmData(SwarmTrajData *swarm_trajs, TmtList *teammate_list) 
  { swarm_trajs_ = swarm_trajs; 
    teammate_list_ = teammate_list;}
  
  inline void setMaxRate(double vmax, double amax) { vmax_ = vmax; amax_ = amax; }

  inline void setTimeBudget(const double& replan_time_budget)
  { replan_time_budget_ = replan_time_budget; }

  inline void setVSDFs(VSDFList *vsdf_list)
  { vsdf_list_ = vsdf_list; }
};

class TrajOptConeFOV : public TrajOpt
{
public:

  TrajOptConeFOV(ros::NodeHandle& nh) : TrajOpt(nh) {}
  int generate_traj(const double t_replan_start,
                    const double t_traj_start,
                    const Eigen::MatrixXd& iniState,
                    const Eigen::MatrixXd& finState,
                    const double &traj_duration,
                    const std::vector<Eigen::Vector3d>& front_wpts,
                    const std::vector<Eigen::Vector3d>& tracking_pts,
                    const std::vector<Eigen::MatrixXd>& hPolys,
                    Trajectory& traj,
                    Trajectory1D& traj_yaw,
                    bool is_kino_traj) override;
  
  bool generate_stop_traj(const Eigen::Vector3d& start_pos, const double& start_yaw,
                          const Eigen::Vector3d& stop_pos, const double& stop_yaw,
                          Trajectory& traj, Trajectory1D& traj_yaw) override;
  bool generate_stop_traj(const Eigen::MatrixXd& initState, const double& duration, 
                          const Eigen::Vector3d& stop_pos, const double& stop_yaw,
                          Trajectory& traj, Trajectory1D& traj_yaw) override;

  bool setInitValues(const Eigen::MatrixXd& iniState, 
                     const Eigen::MatrixXd& finState) override;
  int optimize(const double& delta = 1e-4)            override;         
  int optimize_init(const double& delta = 1e-4)       override;        
  void addTimeIntPenalty(double& cost)                override;          
  void addTimeCost(double& cost)                      override;                
  void addTailCost(double& cost)                      override;                
  void addTimeIntPenalty_init(double& cost)           override;            
  void addTimeCost_init(double& cost)                 override;            
  void addTailCost_init(double& cost)                 override; 

  bool generate_traj(const Eigen::MatrixXd& iniState,
                     const Eigen::MatrixXd& finState,
                     const std::vector<Eigen::MatrixXd>& hPolys,
                     Trajectory& traj) override;
  void addTargetPenalty(double& cost)  override;

  bool generate_init_traj(const double t_replan_start,
                          const Eigen::MatrixXd& iniState,
                          const Eigen::MatrixXd& finState,
                          const std::vector<Eigen::Vector3d>& front_wpts,
                          const std::vector<Eigen::MatrixXd>& hPolys,
                          Trajectory& traj) override;

  bool grad_cost_pt_tracking(const Eigen::Vector3d& pos,
                             const double& yaw,
                             const Eigen::Vector3d& pos_des,
                             const double& yaw_des, 
                             Eigen::Vector3d& gradp,
                             double& grad_yaw,
                             double& cost);
  bool checkTrajFOV(const Trajectory& traj, const Trajectory1D& traj_yaw, bool print_flg);

};

class TrajOptOmniFOV : public TrajOpt
{
public:

  TrajOptOmniFOV(ros::NodeHandle& nh) : TrajOpt(nh) {}
  int generate_traj(const double t_replan_start,
                    const double t_traj_start,
                    const Eigen::MatrixXd& iniState,
                    const Eigen::MatrixXd& finState,
                    const double &traj_duration,
                    const std::vector<Eigen::Vector3d>& front_wpts,
                    const std::vector<Eigen::Vector3d>& tracking_pts,
                    const std::vector<Eigen::MatrixXd>& hPolys,
                    Trajectory& traj,
                    Trajectory1D& traj_yaw,
                    bool is_kino_traj) override;

  bool generate_stop_traj(const Eigen::Vector3d& start_pos, const double& start_yaw,
                          const Eigen::Vector3d& stop_pos, const double& stop_yaw,
                          Trajectory& traj, Trajectory1D& traj_yaw) override;

  bool generate_stop_traj(const Eigen::MatrixXd& initState, const double& duration, 
                          const Eigen::Vector3d& stop_pos, const double& stop_yaw,
                          Trajectory& traj, Trajectory1D& traj_yaw) override;

  bool setInitValues(const Eigen::MatrixXd& iniState, 
                     const Eigen::MatrixXd& finState) override;
  int optimize(const double& delta = 1e-4)            override;         
  int optimize_init(const double& delta = 1e-4)       override;        
  void addTimeIntPenalty(double& cost)                override;          
  void addTimeCost(double& cost)                      override;                
  void addTailCost(double& cost)                      override;                
  void addTimeIntPenalty_init(double& cost)           override;            
  void addTimeCost_init(double& cost)                 override;            
  void addTailCost_init(double& cost)                 override;   

  bool generate_traj(const Eigen::MatrixXd& iniState,
                     const Eigen::MatrixXd& finState,
                     const std::vector<Eigen::MatrixXd>& hPolys,
                     Trajectory& traj) override;
  void addTargetPenalty(double& cost)  override;   

  bool grad_cost_pt_tracking(const Eigen::Vector3d& pos,
                             const Eigen::Vector3d& pos_des,    
                             Eigen::Vector3d& gradp,
                             double& cost);
  
  bool generate_init_traj(const double t_replan_start,
                          const Eigen::MatrixXd& iniState,
                          const Eigen::MatrixXd& finState,
                          const std::vector<Eigen::Vector3d>& front_wpts,
                          const std::vector<Eigen::MatrixXd>& hPolys,
                          Trajectory& traj) override;

  bool checkTrajFOV(const Trajectory& traj, bool print_flg);
};

}