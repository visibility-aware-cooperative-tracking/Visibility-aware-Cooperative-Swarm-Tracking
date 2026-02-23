#include <plan_manage/target_replan_fsm.h>

namespace st_planner
{

  void TargetReplanFSM::init(ros::NodeHandle &nh)
  {
    ROS_WARN("Start target init");

    global_time_start_ = ros::Time::now().toSec();
    last_replan_t_ = -1;

    exec_state_ = FSM_EXEC_STATE::INIT;
    have_goal_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;
    have_traj_ = false;

    /*  fsm param  */
    nh.param("manager/drone_id", drone_id_, -1);
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);//1.0m
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/relative_tracking_x", relative_tracking_p_(0), -10000.0);
    nh.param("fsm/relative_tracking_y", relative_tracking_p_(1), -10000.0);
    nh.param("fsm/relative_tracking_z", relative_tracking_p_(2), -10000.0);
    nh.param("fsm/ground_height_measurement", enable_ground_height_measurement_, true);

    nh.param("fsm/predict_T", predict_T_, 1.0);
    nh.param("fsm/predict_dt", predict_dt_, 0.2);
    nh.param("fsm/follow_dist", max_follow_dist_, 0.7);
  
    nh.param("corridor_bbd", corridor_bound_, 1.6);

    have_trigger_ = !flag_realworld_experiment_;


    ROS_WARN("Target Init Visual");
    visualization_.reset(new PlanningVisualization(nh));
    visualization2_.reset(new display::Visualization(nh));

    
    rog_map::ROSParamLoader ld(nh, rog_cfg_);
    rogmap_.reset(new rog_map::ROGMap(nh, rog_cfg_));

    //Set astar searcher
    astar_search_.reset(new astar_search::astarSearch(nh, rogmap_));

    //Set corridor generator
    sfc_gen_.reset(new SFCgen_ciri::SFCgen_ciri(nh, rogmap_));
    sfc_gen_->setParam(0.28, 3);
    
    ROS_WARN("Target Init traj_opt");
    traj_opt_.reset(new traj_opt::TrajOptOmniFOV(nh));
    traj_opt_->setSwarmData(&traj_set_.swarm_traj, &tmt_list_);

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.1), &TargetReplanFSM::execFSMCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &TargetReplanFSM::odometryCallback, this, ros::TransportHints().tcpNoDelay());
    goal_sub_ = nh.subscribe("goal", 1, &TargetReplanFSM::goalCallback, this, ros::TransportHints().tcpNoDelay());//Track the object
    
    poly_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);

  
  }

  bool TargetReplanFSM::predictPath(std::vector<Eigen::Vector3d> &predict_path,
                                std::vector<Eigen::Vector3d> &predict_vels)
  {
    predict_path.clear(); predict_vels.clear();
    double obj_vx = 1.0, eval_t = 0;
    while (eval_t <= predict_T_)
    {
      predict_path.emplace_back(target_pos_(0) + eval_t * obj_vx + relative_tracking_p_(0), 
                                target_pos_(1) + relative_tracking_p_(1), 
                                target_pos_(2) + relative_tracking_p_(2));
      predict_vels.emplace_back(obj_vx, 0, 0);
      eval_t += predict_dt_;
    }
    return true;
  }

  bool TargetReplanFSM::predictPathCurve(std::vector<Eigen::Vector3d> &predict_path,
                                         std::vector<Eigen::Vector3d> &predict_vels)
  {
    predict_path.clear(); predict_vels.clear();
    double obj_vx = 1.0, eval_t = 0;
    Eigen::Vector3d view_point;
    while (eval_t <= predict_T_)
    {
      predict_path.emplace_back(target_pos_(0) + eval_t * obj_vx + relative_tracking_p_(0), 
                                target_pos_(1) + relative_tracking_p_(1) * cos((target_t_ + eval_t - global_time_start_) / 1.8), 
                                target_pos_(2) + relative_tracking_p_(2));
      
      predict_vels.emplace_back(obj_vx, 
                                -relative_tracking_p_(1) * sin((target_t_ + eval_t - global_time_start_) / 1.8) / 1.8, 
                                0);

      eval_t += predict_dt_;
    }
    
    last_predict_vel_ << obj_vx, -relative_tracking_p_(1) * sin((target_t_ + predict_T_ - global_time_start_) / 1.8) / 1.8, 0;
    return true;
  }

  void TargetReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 500)
    {
      fsm_num = 0;
      printFSMExecState();
    }

    switch (exec_state_)
    {
      case INIT:
      {
        if (!have_odom_)
        {
          ROS_ERROR("DONT HAVE odom!");
          goto force_return; // return;
        }
        changeFSMExecState(WAIT_TARGET, "FSM");
        break;
      }

      case WAIT_TARGET:
      {
        if (!have_goal_ || !have_trigger_ )
        {
          ROS_ERROR("Target don't HAVE goal! Use 3D Nav Goal in the RViz to select goal."); //debug
          goto force_return; // return;
        }
        else
        {
          changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        break;
      }

      case EXEC_TRAJ:
      {
        ROS_WARN_STREAM("Received goal: " << nav_goal_);

        changeFSMExecState(REPLAN_TRAJ, "FSM");
        ROS_WARN_STREAM("Go replan state.");
        break;
      }

      case REPLAN_TRAJ:
      {

        if((nav_goal_ - odom_pos_).norm() < 1.0)
        { 
          goto force_return;
        }

        Eigen::MatrixXd iniState; iniState.setZero(3, 4);
        double cur_replan_t = (ros::Time::now() + ros::Duration(0.014)).toSec();
        double exec_dur = cur_replan_t - last_replan_t_;
        if(!have_traj_ || exec_dur > traj_set_.local_traj.duration)
        {
          iniState.col(0) = odom_pos_;
          iniState.col(1) = odom_vel_;
        }else{
          
          iniState.col(0) = traj_set_.local_traj.traj.getPos(exec_dur);
          iniState.col(1) = traj_set_.local_traj.traj.getVel(exec_dur);
          iniState.col(2) = traj_set_.local_traj.traj.getAcc(exec_dur);
          iniState.col(3) = traj_set_.local_traj.traj.getJer(exec_dur);
        }
         
        Eigen::Vector3d p_start = iniState.col(0);//Set searching start as current odom

        std::vector<Eigen::Vector3d> astar_path, astar_path2;
        
        astar_path.clear();

        bool new_success;

        new_success = astar_search_->short_astar(p_start, nav_goal_, astar_path);
        
        visualization2_->visualize_balls_path(astar_path, "astar_nodes", display::Color::steelblue, 0.06, 1.0);
    
        if(!new_success)
        {
          goto force_return;
        }
       
        //Storing corridors
        std::vector<Eigen::MatrixXd> hPolys;
        std::vector<geometry_utils::Polytope> pPolys;
        
          
        sfc_gen_->generateCorridorAlongPath(astar_path, corridor_bound_, pPolys, hPolys);
        sfc_gen_->visualizeCorridors(pPolys);
      
        Eigen::MatrixXd finState;
        finState.setZero(3, 4);
        finState.col(0) = astar_path.back();

        Trajectory traj;

        new_success = traj_opt_->generate_traj(iniState, finState, hPolys, traj);
        double traj_gen_t = ros::Time::now().toSec();
      

        if(new_success){
          //reset traj
          have_traj_ = true;
          last_replan_t_ = traj_gen_t;
          traj_set_.setTraj(traj, traj_gen_t, drone_id_);
;
          visualization2_->visualize_traj(traj, "minco_traj");


          traj_utils::PolyTraj poly_msg;
          polyTraj2Msg(poly_msg);
          poly_traj_pub_.publish(poly_msg);
            
        }else{
          goto force_return;
        }
        break;
      }

    }
    force_return:;
    exec_timer_.start();
  }
  
  void TargetReplanFSM::polyTraj2Msg(traj_utils::PolyTraj &poly_msg)
  {
    //Set poly_traj msg from traj_set for server
    auto data = &traj_set_.local_traj;
    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceNum();

    poly_msg.drone_id = drone_id_;
    poly_msg.traj_id = data->traj_id;
    poly_msg.start_time = ros::Time(data->start_time);
    poly_msg.order = 7; 
    poly_msg.duration.resize(piece_num);
    poly_msg.coef_x.resize(8 * piece_num);
    poly_msg.coef_y.resize(8 * piece_num);
    poly_msg.coef_z.resize(8 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      poly_msg.duration[i] = durs(i);

      CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
      int i8 = i * 8;
      for (int j = 0; j < 8; j++)
      {
        poly_msg.coef_x[i8 + j] = cMat(0, j);
        poly_msg.coef_y[i8 + j] = cMat(1, j);
        poly_msg.coef_z[i8 + j] = cMat(2, j);
      }
    }
  }


  void TargetReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {
    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    // cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void TargetReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    //
  }

  std::pair<int, TargetReplanFSM::FSM_EXEC_STATE> TargetReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }


  void TargetReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;
    have_odom_ = true;
  }
  
  void TargetReplanFSM::goalCallback(const geometry_msgs::PoseStampedConstPtr& msgPtr)
  {
    static int cur_goal_id = 0;

    double flight_height = 1.7;
    nav_goal_ << msgPtr->pose.position.x, msgPtr->pose.position.y, flight_height;
    have_goal_ = true;
  }

}

// namespace st_planner
