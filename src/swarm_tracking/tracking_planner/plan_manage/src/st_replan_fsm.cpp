#include <plan_manage/st_replan_fsm.h>

namespace st_planner
{

  void STReplanFSM::init(ros::NodeHandle &nh)
  {
    nh_ = nh;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_trigger_ = false;
    have_odom_ = false;

    have_all_tmt_info_ = false;

    have_traj_ = false;

    use_kino_search_ = true;

    wait_hover_ = true;//init true. indicating normal tracking stop
    force_hover_ = false;//init false. indicating emergency stop
    stop_call_num_ = 0;

    /*  fsm param  */
    nh.param("planning/is_fov_omni", flag_omni_fov_, false);
    nh.param("planning/predict_T", predict_T_, 1.0);
    nh.param("planning/predict_dt", predict_dt_, 0.2);
    nh.param("planning/track_dist", desired_dist_, 1.9);
    nh.param("planning/tolerance_d", tolerance_d_, 0.4);
    nh.param("planning/fov_theta", fov_theta_, 0.26);//15 degree
    nh.param("planning/fov_ctr_theta", fov_ctr_theta_, 0.0);
    nh.param("planning/min_flight_height", min_flight_height_, 0.2);
    nh.param("planning/print_log", print_log_, false);
    nh.param("planning/ciri_robot_r", ciri_robot_r_, 0.29);
    nh.param("planning/ciri_iter_num", ciri_iter_num_, 5);
    nh.param("planning/LI_extrinsic", LI_extrinsic_, 0.075);
    nh.param("planning/use_kino_search", use_kino_search_, true);
    nh.param("planning/drone_id", drone_id_, -1);

    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/relative_tracking_x", relative_tracking_p_(0), -10000.0);
    nh.param("fsm/relative_tracking_y", relative_tracking_p_(1), -10000.0);
    nh.param("fsm/relative_tracking_z", relative_tracking_p_(2), -10000.0); 
    nh.param("fsm/tracker_ids", candi_tracker_ids_, candi_tracker_ids_);
    nh.param("fsm/replan_time_budget", replan_time_budget_, 0.080);
    nh.param("fsm/target_vel_stop_bar", target_vel_stop_bar_, 0.30);
    nh.param("fsm/self_vel_stop_bar", self_vel_stop_bar_, 0.45);
    nh.param("fsm/flag_allow_hover", flag_allow_hover_, true);
    
    if(candi_tracker_ids_.empty()) 
    { ROS_INFO("\033[41;1m[FSM] Candi Tracker ID Set Could Not Be Empty!\033[0m");
      if(print_log_) std::cout << "Candi Tracker ID Set Could Not Be Empty!" << std::endl;
      return; }

    if(std::count(candi_tracker_ids_.begin(), candi_tracker_ids_.end(), drone_id_)==0)
    { ROS_INFO("\033[41;1m[FSM] Candi Tracker ID Set Should Contain DroneID !\033[0m");
      if(print_log_) std::cout << "Candi Tracker ID Set Should Contain DroneID !" << std::endl;
      return; }
    
    std::sort(candi_tracker_ids_.begin(), candi_tracker_ids_.end());
    candi_tracker_num_ = candi_tracker_ids_.size();
    cout << "\033[42;1m[FSM] Tracker id: ";
    for(int i = 0; i < candi_tracker_num_; i++)
      cout << candi_tracker_ids_[i] << " ";
    cout << "\033[0m" << endl;

    if(candi_tracker_ids_.size() == 1) have_all_tmt_info_ = true;

    teammate_odom_subscribers_.resize(candi_tracker_num_); //init teammate_odom_subs
    tmtodom_recv_count_.assign(candi_tracker_ids_.back() + 1, 0);
    swarm_id_list_ = candi_tracker_ids_;

    ROS_WARN_STREAM("[FSM] param: drone_id: " << drone_id_);
    ROS_WARN_STREAM("[FSM] param: thresh_replan_time: " << replan_thresh_);
    ROS_WARN_STREAM("[FSM] param: emergency_time: " << emergency_time_);
    ROS_WARN_STREAM("[FSM] param: relative_tracking_x: " << relative_tracking_p_(0));
    ROS_WARN_STREAM("[FSM] param: relative_tracking_y: " << relative_tracking_p_(1));
    ROS_WARN_STREAM("[FSM] param: relative_tracking_z: " << relative_tracking_p_(2));
    ROS_WARN_STREAM("[FSM] param: predict_T: " << predict_T_);
    ROS_WARN_STREAM("[FSM] param: predict_dt: " << predict_dt_);
    ROS_WARN_STREAM("[FSM] param: track_dist: " << desired_dist_);
    ROS_WARN_STREAM("[FSM] param: tolerance_d: " << tolerance_d_);
    ROS_WARN_STREAM("[FSM] param: min_flight_height: " << min_flight_height_);

    /* initialize main modules */
    visualization_.reset(new display::Visualization(nh));

    rog_map::ROSParamLoader ld(nh, rog_cfg_);
    rogmap_.reset(new rog_map::ROGMap(nh, rog_cfg_));

    //Set astar searcher
    astar_search_.reset(new astar_search::astarSearch(nh, rogmap_));

    //Set corridor generator
    sfc_gen_.reset(new SFCgen_ciri::SFCgen_ciri(nh, rogmap_));
    sfc_gen_->setParam(ciri_robot_r_, ciri_iter_num_);

    vsdfs_.resize(floor(predict_T_ / predict_dt_) + 1);
    for(auto &vsdf : vsdfs_)
    {
      vsdf.reset(new VisSDF);
      vsdf->initSDF(nh, rogmap_);
    }

    //Set kinodynamic searcher
    kino_search_.reset(new kino_search::KinoSearch);
    kino_search_->setParam(nh);
    kino_search_->setEnvironment(rogmap_);
    kino_search_->init(drone_id_, &swarm_tmt_list_);
    kino_search_->setVSDFs(&vsdfs_);

    //Set predictor
    predictor_.reset(new prediction::Predict(nh, rogmap_));

    //Set trajectory optimizer
    if(flag_omni_fov_) traj_opt_.reset(new traj_opt::TrajOptOmniFOV(nh));
    else               traj_opt_.reset(new traj_opt::TrajOptConeFOV(nh));
    
    traj_opt_->setEnvironment(rogmap_);
    traj_opt_->setSwarmData(&traj_set_.swarm_traj, &swarm_tmt_list_);
    traj_opt_->setVSDFs(&vsdfs_);
    traj_opt_->setTimeBudget(replan_time_budget_);

    state_dim_ = flag_omni_fov_ ? 3 : 4;

    if(print_log_)
    {
      int unit_time_int = (int)(ros::Time::now().toSec()) % 100000;
      replan_log_ = ofstream(DEBUG_FILE_DIR("drone" + std::to_string(drone_id_) + "_replan_log_"  + std::to_string(unit_time_int) + ".txt"), ios::trunc);
      fsm_log_ = ofstream(DEBUG_FILE_DIR("drone" + std::to_string(drone_id_) + "_fsm_log_"  + std::to_string(unit_time_int) + ".txt"), ios::trunc);
      // cpumem_log_ = ofstream(DEBUG_FILE_DIR("cpumem_log.txt"), ios::trunc);
    }

    //Set cpumem
    current_pid_ = GetCurrentPid();
    totalcputime_ = get_cpu_total_occupy();
    procputime_ = get_cpu_proc_occupy(current_pid_);
    
    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.005), &STReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.1), &STReplanFSM::safetyCheckCallback, this);
    // cpumem_timer_ = nh.createTimer(ros::Duration(3.0), &STReplanFSM::cpumemCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &STReplanFSM::odometryCallback, this, ros::TransportHints().tcpNoDelay());

    for(int i = 0; i < candi_tracker_ids_.size(); i++)
    {
      if(candi_tracker_ids_[i] == drone_id_) continue;
      //topic_name should distinguish is_real_exp or not
      teammate_odom_subscribers_[i] = nh.subscribe<nav_msgs::Odometry>("/teammate_odom/UAV" + to_string(candi_tracker_ids_[i]), 1000, 
                                                   boost::bind(&STReplanFSM::teammateOdomCallback, this, _1, candi_tracker_ids_[i]));
    }

    target_sub_ = nh.subscribe("object", 1, &STReplanFSM::targetCallback, this, ros::TransportHints().tcpNoDelay());//Track the object
    
    trigger_sub_ = nh.subscribe("trigger", 1, &STReplanFSM::triggerCallback, this, ros::TransportHints().tcpNoDelay());
    
    poly_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);
    heartbeat_pub_ = nh.advertise<std_msgs::Empty>("/heartbeat", 10);
    land_sig_pub_ = nh.advertise<std_msgs::Empty>("/planner_cmd/land", 10);

    broadcast_polytraj_pub_ = nh.advertise<swarm_msgs::MincoTraj>("planning/broadcast_traj_send", 10);
    broadcast_polytraj_sub_ = nh.subscribe<swarm_msgs::MincoTraj>("planning/broadcast_traj_recv", 100,
                                                                  &STReplanFSM::RecvBroadcastMINCOTrajCallback,
                                                                  this,
                                                                  ros::TransportHints().tcpNoDelay());

    int count = 0;
    while (ros::ok() && count++ < 500)
    {
      ros::spinOnce();
      ros::Duration(0.001).sleep();
    }

  }

  void STReplanFSM::target2view(const std::vector<Eigen::Vector3d> &target_path,
                                const std::vector<Eigen::Vector3d> &target_vels,
                                std::vector<Eigen::Vector3d> &view_path,
                                std::vector<Eigen::Vector3d> &view_vels)
  {
    view_path.clear();view_vels.clear();
    view_vels = target_vels;
    for(auto pt : target_path)
      view_path.emplace_back(pt + relative_tracking_p_);
  }

  void STReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); 

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 50)
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
          goto force_return; // return;
        }
        changeFSMExecState(WAIT_INIT_TARGET, "FSM");
        break;
      }

      case WAIT_INIT_TARGET:
      {
        if (!have_target_ || !have_trigger_ || !have_all_tmt_info_ )
          goto force_return; // return;
        else
        {
          changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        break;
      }

      case EXEC_TRAJ:
      {
        /*-- Old EXEC_TRAJ --*/
        double distance_stop_coef_bar = 4.0;
        double t_cur = ros::Time::now().toSec();
        if(wait_hover_ && odom_vel_.norm() > 0.2) //Wait for fully hover
          goto force_return;

         /*-- conditions to stop tracking --*/

        double stop_dist = (target_pos_ + relative_tracking_p_ - odom_pos_).norm();
        bool is_stop_dist_valid = (stop_dist > desired_dist_ && stop_dist - desired_dist_ < distance_stop_coef_bar * tolerance_d_) ||
                                  (stop_dist < desired_dist_ && stop_dist > 0.8 * (desired_dist_ - tolerance_d_));

        if(flag_allow_hover_ 
           && (is_stop_dist_valid 
           && odom_vel_.norm() < self_vel_stop_bar_ 
           && target_vel_.norm() < target_vel_stop_bar_
           && isTargetVisible()))
        {
          if(!wait_hover_){
            wait_hover_ = true;
            ROS_INFO_STREAM("[FSM] Start Hovering.");
            if(print_log_){
              std::cout << "[FSM] Start Hovering." << std::endl;
              fsm_log_ << "[FSM] Start Hovering." << std::endl;
            }
      
            callStop(odom_pos_ + odom_vel_ * 0.1, odom_yaw_); 
          }
          goto force_return;
        }else if(!have_traj_ || //to set first plan
                (traj_set_.local_traj.start_time > 0 && 
                (t_cur - traj_set_.local_traj.start_time) > (replan_thresh_ - replan_time_budget_)))//exceed thresh time
        {
          //go replan
          if(wait_hover_ || force_hover_){
            trigger_vel_ = !(target_vel_.norm() < target_vel_stop_bar_);

            trigger_dis_ = !is_stop_dist_valid;
          }else{
            trigger_dis_ = false; trigger_vel_ = false;
          }
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }

   
        break;
        /*-- Old EXEC_TRAJ --*/
        
      }

      case REPLAN_TRAJ:
      {
        if(callReplan())
        {
          visualization_->visualize_trajcps(traj_set_.local_traj.traj, "minco_traj_cps");
          //Succeed. Pub the generated traj to server & broadcast
          bool last_wait_hover{wait_hover_}, last_force_hover{force_hover_};

          // ROS_INFO_STREAM("\033[33;1m[FSM] Replan Succeed.\033[0m");
          if(print_log_){
            std::cout << "[FSM] [" <<  setiosflags(ios::fixed) << setprecision(4) << ros::Time::now().toSec() << "] Replan Succeed. " << std::endl;
            fsm_log_ << "[FSM] [" <<  setiosflags(ios::fixed) << setprecision(4) << ros::Time::now().toSec() << "] Replan Succeed. " << std::endl;;
          }
         
          wait_hover_ = false;
          force_hover_ = false;
          stop_call_num_ = 0;
          if(wait_hover_ == !last_wait_hover) 
          {
            ROS_INFO_STREAM("\033[33;1m[FSM]  &&. From Wait Hover to EXEC.\033[0m");
            if(print_log_){
              std::cout << "[FSM] &&. From Wait Hover to EXEC." << std::endl;
              fsm_log_ << "[FSM] &&. From Wait Hover to EXEC." << std::endl;
            }
          }
            
          if(force_hover_ == !last_force_hover) 
          {
            ROS_INFO_STREAM("\033[41;1m[FSM] &&. From FORCE to EXEC.\033[0m");
            if(print_log_){
              std::cout << "[FSM] &&. From FORCE to EXEC." << std::endl;
              fsm_log_ << "[FSM] &&. From FORCE to EXEC." << std::endl;
            }
          }

          changeFSMExecState(EXEC_TRAJ, "FSM");
        }else{

          if(force_hover_)
          {
            ROS_INFO_STREAM("\033[41;1m[FSM] Replan Failed. Still Emer Stop.\033[0m");
            if(print_log_){
              std::cout << "[FSM] Replan Failed. Still Emer Stop." << std::endl;
              fsm_log_ << "[FSM] Replan Failed. Still Emer Stop." << std::endl;
            }
            changeFSMExecState(EMERGENCY_STOP, "FSM");
          }else{
            ROS_INFO_STREAM("\033[33;1m[FSM] Replan Failed. Exec Last Traj/Hover.\033[0m");
            if(print_log_){
              std::cout << "[FSM] Replan Failed. Exec Last Traj/Hover." << std::endl;
              fsm_log_ << "[FSM] Replan Failed. Exec Last Traj/Hover." << std::endl;
            }
            changeFSMExecState(EXEC_TRAJ, "FSM");
          }
        }
        
  
        break;
      }

      case EMERGENCY_STOP:
      { 
        if(print_log_){
          fsm_log_ << "[ " << setiosflags(ios::fixed) << setprecision(4) << ros::Time::now().toSec() << " ]! ExecTimer into Emergency, call_stop: " 
          << stop_call_num_ << std::endl;
        }

        force_hover_ = true;
        //Emergency case handle in this stage
        if(stop_call_num_ < 2){
          callStop(odom_pos_, odom_yaw_);
          stop_call_num_++;
        }
        else
        {
          //Fully hover and have target information
          if(odom_vel_.norm() < 0.1 && have_target_)
          {
            //rebound replan after hover
            changeFSMExecState(REPLAN_TRAJ, "FSM");
          }
        }
      }

    }
    force_return:;
    exec_timer_.start();
  }

  bool STReplanFSM::callReplan()
  {

    std::lock_guard<std::mutex> lock(replan_mutex_);

    if(print_log_){
      double replan_init_time = ros::Time::now().toSec();
      replan_attemp_num_++;
      std::cout << setiosflags(ios::fixed) << setprecision(4) << replan_init_time 
                << ", Replan start: " << replan_attemp_num_ << " trial. " << std::endl;
      replan_log_ << drone_id_ << "," << replan_attemp_num_ << "," 
      << "WaitHover: " << wait_hover_ << "," 
      << "ForceHover: " << force_hover_ << "," 
      << "triDis: " << trigger_dis_ << ","
      << "triVel: " << trigger_vel_ << "," 
      << setiosflags(ios::fixed) << setprecision(4) << replan_init_time << ",";
    }

    //Predict the target motion
    predict_stamp_ = target_t_;


    //compensate
    Eigen::Vector3d predict_start_p = target_vel_.norm() > 0.35 ? 
                                      target_pos_ + target_vel_ * replan_time_budget_ * 1.5 :
                                      target_pos_;
    
    bool new_success = predictor_->predict(predict_start_p, target_vel_, target_path_, target_vels_);

    double prop_v_max, prop_a_max;
    double target_vel_norm = target_vel_.norm();
    if(target_vel_norm < 1.3)
    {
      prop_v_max = 3.5;
      prop_a_max = 2.5;
    }else if(target_vel_norm < 2.6){
      prop_v_max = 4.2;
      prop_a_max = 3.4;
    }else{
      prop_v_max = 5.2;
      prop_a_max = 4.2;
    }

    if(!new_success){
      ROS_INFO_STREAM("\033[33;1m[Replan] Prediction Failed.\033[0m");
      if(print_log_){
        std::cout << "[Replan] Prediction Failed." << std::endl;
        replan_log_ << " Predict failed. " << std::endl;
      }
      return false;
    }
    target2view(target_path_, target_vels_, view_path_, view_vels_);
    visualization_->visualize_balls_path(view_path_, "target_pts", display::Color::red, 0.12, 0.8);
    //view_path is the actual target motion with relative pose bias

    /*----Generate VSDF----*/
    TimeConsuming vsdf_timer("vsdf_t", false);

    omp_set_num_threads(14);
    #pragma omp parallel for schedule(static)  
      for(int i = 0; i < vsdfs_.size(); i++)
        vsdfs_[i]->generateSDF(view_path_[i]);

      
    double vsdf_time = vsdf_timer.stop();
    vsdf_update_num_++;
    vsdf_time_cost_ += vsdf_time;
    if(print_log_) replan_log_ << " VSDF time(ms): " << vsdf_time  << ", ";

    if(drone_id_ == 0) vsdfs_[0]->publishSDF();

    TimeConsuming replan_timer("replan_t", false);

    /*----Get IniStates----*/
    double start_replan_t = ros::Time::now().toSec();
    double eval_replan_t = start_replan_t + ros::Duration(replan_time_budget_).toSec();
    auto local_data = &traj_set_.local_traj;
    bool is_replan_from_traj{true};
    if(!have_traj_ || wait_hover_ || force_hover_){
      is_replan_from_traj = false;
    }else if(eval_replan_t - local_data->start_time > 
             local_data->duration){
      is_replan_from_traj = false; }

    Eigen::MatrixXd iniState; iniState.setZero(state_dim_, 4);
    if(is_replan_from_traj){
      double exec_dur_on_last = eval_replan_t - local_data->start_time;
      if(flag_omni_fov_){
        iniState.col(0) = local_data->traj.getPos(exec_dur_on_last);
        iniState.col(1) = local_data->traj.getVel(exec_dur_on_last);
        iniState.col(2) = local_data->traj.getAcc(exec_dur_on_last);
        iniState.col(3) = local_data->traj.getJer(exec_dur_on_last);
      }else{
        iniState.col(0) << local_data->traj.getPos(exec_dur_on_last), 
                           local_data->traj_yaw.getPos(exec_dur_on_last);
        iniState.col(1) << local_data->traj.getVel(exec_dur_on_last), 
                           local_data->traj_yaw.getVel(exec_dur_on_last);
        iniState.col(2) << local_data->traj.getAcc(exec_dur_on_last), 
                           local_data->traj_yaw.getAcc(exec_dur_on_last);
        iniState.col(3) << local_data->traj.getJer(exec_dur_on_last), 
                           local_data->traj_yaw.getJer(exec_dur_on_last);
      }
    }else{
      if(flag_omni_fov_){
        iniState.col(0) = odom_pos_;
        iniState.col(1) = odom_vel_;
      }else{
        iniState.col(0) << odom_pos_, odom_yaw_;
        iniState.col(1) << odom_vel_, 0;
      }
    }

    /*----planning----*/
    Eigen::Vector3d p_start = iniState.col(0).head<3>();//Set searching start as current state
    Eigen::Vector3d v_start = iniState.col(1).head<3>();
    Eigen::Vector3d a_start = iniState.col(2).head<3>();

    std::vector<Eigen::Vector3d> front_way_pts, front_vel_pts, front_acc_pts;
    std::vector<Eigen::Vector3d> astar_path;
    
    Eigen::Vector3d valid_p_start = p_start;

    if(rogmap_->isOccupiedInflate(p_start))
    {
      if(rogmap_->isOccupiedInflateWithNbr(p_start, valid_p_start))
      {
        ROS_INFO_STREAM("\033[33;1m[Replan] Start Point & Nbrs: All not valid.\033[0m");
        if(print_log_){
          std::cout << "[Replan] Start Point & Nbrs: All not valid." << std::endl;
          replan_log_ << " Start Point Invalid. " << std::endl;
        }
        return false;
      }
    }
    front_way_pts.clear();
    front_vel_pts.clear();   
    front_acc_pts.clear();
    astar_path.clear();
    bool path_occ_flg = false;

    swarm_data_mutex_.lock();

    updateSwarmTmtList(p_start);

    // //Update the current pos of teamates for kino searching

    TimeConsuming search_timer("search_t", false);

    if(use_kino_search_){
      int search_issue{0};
      search_issue = kino_search_->search(valid_p_start, v_start, view_path_, prop_v_max, prop_a_max);
    
      if(search_issue){
        //Conduct astar search if kino search failed
        // ROS_INFO_STREAM("\033[33;1m[Replan] KinoFailed. Using Astar.\033[0m");
        if(print_log_){
          std::cout << "[Replan] KinoFailed. Using Astar." << std::endl;
          replan_log_ << " KinoFailCase: " << kino_search_->search_state_str_[search_issue]  << ", ";
        }
        new_success = astar_search_->findTrackingPath(valid_p_start, view_path_, front_way_pts, astar_path);

        if(new_success)
        {
          front_way_pts = {valid_p_start};
          bool out_of_range{false};
          for(auto pt : astar_path)
          {
            if((pt - valid_p_start).norm() > 5.0 * predict_T_)
            {
              front_way_pts.emplace_back(pt);
              out_of_range = true;
              break;
            }
          }
          if(!out_of_range) front_way_pts.emplace_back(astar_path.back());
        }
        
        is_kino_path_ = false; 
      }else{
        kino_search_->retrievePath(front_way_pts, front_vel_pts, front_acc_pts);

        new_success = true;
        is_kino_path_ = true;
        visualization_->visualize_balls_path(front_way_pts, "kino_search_wpts", display::Color::black, 0.09, 0.8);
      }
    }else{
      new_success = astar_search_->findTrackingPath(valid_p_start, view_path_, front_way_pts, astar_path);

        if(new_success)
        {
          front_way_pts = {valid_p_start};
          bool out_of_range{false};
          for(auto pt : astar_path)
          {
            if((pt - valid_p_start).norm() > 5.0 * predict_T_)
            {
              front_way_pts.emplace_back(pt);
              out_of_range = true;
              break;
            }
          }
          if(!out_of_range) front_way_pts.emplace_back(astar_path.back());
        }
        
      is_kino_path_ = false; 
    }
    

    search_time_ = search_timer.stop();

    if(!new_success)
    {
      ROS_INFO_STREAM("\033[33;1m[Replan] Front-End: Cannot find a feasible path.\033[0m");
      if(print_log_){
        std::cout << "[Replan] Front-End: Cannot find a feasible path." << std::endl;
        replan_log_ << "Front-end Failed. " << std::endl;
      }
      swarm_data_mutex_.unlock();

      return false;
    }else{
      if(print_log_) replan_log_ << " Front-End time(ms): " << search_time_  << ", ";
    }

    //Generate visible sector for each waypt
    
    std::vector<Eigen::Vector3d> corridor_way_pts = front_way_pts;
    corridor_way_pts.insert(corridor_way_pts.begin(), valid_p_start);

    TimeConsuming pts2path_timer("pts2path_t", false);
    new_success = astar_search_->pts2path(corridor_way_pts, astar_path); 
    double pts2path_time = pts2path_timer.stop();
    if(!new_success)
    {
      ROS_INFO_STREAM("\033[33;1m[Replan] Pts2Path: Cannot find a valid path for corridors.\033[0m");
      if(print_log_){
        std::cout << "[Replan] Pts2Path: Cannot find a valid path for corridors." << std::endl;
        replan_log_ << "Pts2Path Failed. " << std::endl;
      }
      swarm_data_mutex_.unlock();

      return false;
    }else{
      if(print_log_) replan_log_ << "Pts2path time(ms): " << pts2path_time << ", ";
    }

    astar_path.insert(astar_path.begin(), p_start); //include the original start point
    visualization_->visualize_balls_path(astar_path, "astar_nodes", display::Color::purple, 0.03, 1.0);
    
    //Generate corridors
    std::vector<Eigen::MatrixXd> hPolys;
    std::vector<geometry_utils::Polytope> pPolys;

    TimeConsuming corridor_timer("corridor_t", false);

    new_success = sfc_gen_->generateCorridorAlongPath(astar_path, 2.0, pPolys, hPolys);

    corridor_time_ = corridor_timer.stop();

    if(!new_success)
    {
      ROS_INFO_STREAM("\033[33;1m[Replan] Corridor: Cannot generate a feasible corridor.\033[0m");
      if(print_log_){
        std::cout << "[Replan] Corridor: Cannot generate a feasible corridor." << std::endl;
        replan_log_ << "Corridor Failed. " << std::endl;
      }
      swarm_data_mutex_.unlock();

      return false;
    }else{
      if(print_log_) replan_log_ << "sfc_gen time(ms): " << corridor_time_  << ", ";
    }

    sfc_gen_->visualizeCorridors(pPolys);
    // sfc_gen_->visCorridor(hPolys);

    //Generate back-end Traj
    double traj_duration = predict_T_;
    Eigen::Vector3d p_end = astar_path.back();

    Eigen::MatrixXd finState;
    finState.setZero(state_dim_, 4);
    finState.col(0).head(3) = p_end;
    finState.col(1).head(3) = view_vels_.back();

    Trajectory opt_traj, opt_traj_init;
    Trajectory1D opt_traj_yaw;

    TimeConsuming minco_timer("minco_t", false);

    int opt_issue{0};

    updateSwarmTmtList(p_start);//Update the teammate pos for grad evaluation

    traj_opt_->setMaxRate(prop_v_max, prop_a_max);

    opt_issue = traj_opt_->generate_traj(start_replan_t, eval_replan_t, iniState, finState, traj_duration, front_way_pts, target_path_, 
                                         hPolys, opt_traj, opt_traj_yaw, is_kino_path_);

    swarm_data_mutex_.unlock();
    minco_time_ = minco_timer.stop();
    trial_num_++;
    average_minco_time_ += minco_time_;

    double total_replan_time = replan_timer.stop();

    if(opt_issue){
      ////
    }else{

      if(total_replan_time > 1e3 * replan_time_budget_)
      {
        ROS_WARN_STREAM("\033[31;1m replan total: " << total_replan_time 
                      << " ms, VSDF: " << vsdf_time  
                      << " ms, search: " << search_time_  << " ms, cord: " << corridor_time_
                      << " ms, opt: " << minco_time_ << "\033[0m");
        
        ROS_WARN_STREAM("\033[1;32m init opt: " << traj_opt_->init_overhead_ << " ms, regu_opt: " << traj_opt_->regu_overhead_
                      << " ms, out NUM: " << traj_opt_->out_loop_num_ 
                      << "\033[0m");
      }else{
        
      }
    }

    if(opt_issue){
      //Back-end optimization failed
      ROS_INFO_STREAM("\033[33;1m[Replan] Back-End: MINCO Failed: " << traj_opt_->opt_state_str_[opt_issue] << "\033[0m"); 
      if(print_log_){
        std::cout << "[Replan] Back-End: MINCO Failed: " << traj_opt_->opt_state_str_[opt_issue] << std::endl;
        replan_log_ << "MINCO Failed: " << traj_opt_->opt_state_str_[opt_issue] << ". " << std::endl;
      }
      return false;
    }else{
      if(print_log_) replan_log_ << "minco time(ms): " << minco_time_ << ", overall time(ms): " << total_replan_time << ". " << std::endl;
      double finish_replan_t = ros::Time::now().toSec();
      if(is_replan_from_traj){
        if(finish_replan_t < eval_replan_t){
          if(print_log_){
            ROS_WARN_STREAM("drone " << drone_id_ << " start sleep for replan. ");
            std::cout << "drone " << drone_id_ << " start sleep for replan. " << std::endl;
          }
          ros::Duration(eval_replan_t - finish_replan_t).sleep();
        }
        traj_set_.setTraj(opt_traj, opt_traj_yaw, eval_replan_t, drone_id_); //correct one
      }else{
        traj_set_.setTraj(opt_traj, opt_traj_yaw, finish_replan_t, drone_id_);
        have_traj_ = true;
      }

      //publish the traj
      traj_utils::PolyTraj poly_msg;
      swarm_msgs::MincoTraj MINCO_msg;
      polyTraj2Msg(poly_msg, MINCO_msg);
      poly_traj_pub_.publish(poly_msg);
      broadcast_polytraj_pub_.publish(MINCO_msg);
      return true;
    }
  }

  void STReplanFSM::polyTraj2Msg(traj_utils::PolyTraj &poly_msg, 
                                 swarm_msgs::MincoTraj &MINCO_msg)
  {
    //Set poly_traj msg from traj_set for server
    auto data = &traj_set_.local_traj;
    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceNum();

    poly_msg.drone_id = drone_id_;
    poly_msg.traj_id = data->traj_id;
    poly_msg.is_final_traj = data->is_final_traj;
    poly_msg.start_time = ros::Time(data->start_time);
    poly_msg.order = 7; 
    poly_msg.duration.resize(piece_num);
    poly_msg.coef_x.resize(8 * piece_num);
    poly_msg.coef_y.resize(8 * piece_num);
    poly_msg.coef_z.resize(8 * piece_num);
    poly_msg.coef_yaw.resize(4 * piece_num);
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
      if(!flag_omni_fov_)
      {
        int i4 = i * 4;
        CoefficientMat1D cMatYaw = data->traj_yaw.getPiece(i).getCoeffMat();
        for (int j = 0; j < 4; j++){
          poly_msg.coef_yaw[i4 + j] = cMatYaw(0, j);
        }
      }
    }

    //Set minco_traj msg from traj_set for broadcast
    MINCO_msg.drone_id = drone_id_;
    MINCO_msg.traj_id = data->traj_id;
    MINCO_msg.is_final_traj = data->is_final_traj;
    MINCO_msg.start_time = ros::Time(data->start_time);
    MINCO_msg.order = 7;
    MINCO_msg.duration.resize(piece_num);
    Eigen::Vector3d vec;
    vec = data->traj.getPos(0);
    MINCO_msg.start_p[0] = vec(0), MINCO_msg.start_p[1] = vec(1), MINCO_msg.start_p[2] = vec(2);
    vec = data->traj.getVel(0);
    MINCO_msg.start_v[0] = vec(0), MINCO_msg.start_v[1] = vec(1), MINCO_msg.start_v[2] = vec(2);
    vec = data->traj.getAcc(0);
    MINCO_msg.start_a[0] = vec(0), MINCO_msg.start_a[1] = vec(1), MINCO_msg.start_a[2] = vec(2);
    vec = data->traj.getJer(0);
    MINCO_msg.start_j[0] = vec(0), MINCO_msg.start_j[1] = vec(1), MINCO_msg.start_j[2] = vec(2);
    vec = data->traj.getPos(data->duration);
    MINCO_msg.end_p[0] = vec(0), MINCO_msg.end_p[1] = vec(1), MINCO_msg.end_p[2] = vec(2);
    vec = data->traj.getVel(data->duration);
    MINCO_msg.end_v[0] = vec(0), MINCO_msg.end_v[1] = vec(1), MINCO_msg.end_v[2] = vec(2);
    vec = data->traj.getAcc(data->duration);
    MINCO_msg.end_a[0] = vec(0), MINCO_msg.end_a[1] = vec(1), MINCO_msg.end_a[2] = vec(2);
    vec = data->traj.getJer(data->duration);
    MINCO_msg.end_j[0] = vec(0), MINCO_msg.end_j[1] = vec(1), MINCO_msg.end_j[2] = vec(2);
    MINCO_msg.inner_x.resize(piece_num - 1);
    MINCO_msg.inner_y.resize(piece_num - 1);
    MINCO_msg.inner_z.resize(piece_num - 1);
    Eigen::MatrixXd pos = data->traj.getPositions();
    for (int i = 0; i < piece_num - 1; i++)
    {
      MINCO_msg.inner_x[i] = pos(0, i + 1);
      MINCO_msg.inner_y[i] = pos(1, i + 1);
      MINCO_msg.inner_z[i] = pos(2, i + 1);
    }
    for (int i = 0; i < piece_num; i++)
      MINCO_msg.duration[i] = durs[i];

  }

  void STReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {
    std::lock_guard<std::mutex> lock(exec_state_mutex_);
    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[5] = {"INIT", "WAIT_TARGET", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;

    if(print_log_) 
    {
      std::cout << "[ " << setiosflags(ios::fixed) << setprecision(4) << ros::Time::now().toSec() << " ]![" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] 
      << std::endl;
      fsm_log_ << "[ " << setiosflags(ios::fixed) << setprecision(4) << ros::Time::now().toSec() << " ]![" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] 
      << std::endl;
    }

    if(pre_s == 4 || (int)exec_state_ == 4)
      cout << "\033[41;1m[ Drone " << drone_id_ << " " <<  pos_call << "]: from " << state_str[pre_s] << " to " << state_str[int(new_state)] << "\033[0m" << endl;
     
  }

  void STReplanFSM::printFSMExecState()
  {
    static string state_str[5] = {"INIT", "WAIT_TARGET", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    if((int)exec_state_ == 4) cout << "\033[41;1m";
    cout << "[ " << setiosflags(ios::fixed) << setprecision(4) << ros::Time::now().toSec() << " ]![drone " <<  drone_id_ << " FSM]: state: " + state_str[int(exec_state_)];
    if(print_log_) 
    fsm_log_ << "[ " << setiosflags(ios::fixed) << setprecision(4) << ros::Time::now().toSec() << " ]![drone " <<  drone_id_ << " FSM]: state: " + state_str[int(exec_state_)];


    ////-------------------
    // some warnings
    if (!have_odom_)
    {
      cout << ", waiting for odom";
      if(print_log_) fsm_log_ << ", waiting for odom";
    }
    if (!have_target_)
    {
      cout << ", waiting for valid target";
      if(print_log_) fsm_log_ << ", waiting for valid target";
    }
    if (!have_trigger_)
    {
      cout << ", waiting for trigger";
      if(print_log_) fsm_log_ << ", waiting for trigger";
    }
    if (!have_all_tmt_info_)
    {
      cout << ", haven't collect info from all teammate";
      if(print_log_) fsm_log_ << ", haven't collect info from all teammate";
    }

    if (wait_hover_)
    {
      cout << ", wait hovering";
      if(print_log_) fsm_log_ << ", wait hovering";
    }
    if (force_hover_)
    {
      cout << ", force hovering";
      if(print_log_) fsm_log_ << ", force hovering";
    }
    ////--------------------
    cout << "\033[0m" << endl;
    if(print_log_) fsm_log_ << endl;
    
    std::fflush(stdout);

  }

  //Temporarily deprecated
  std::pair<int, STReplanFSM::FSM_EXEC_STATE> STReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  bool STReplanFSM::isTargetVisible() 
  {
    //check FOV in Hover state
    Eigen::Vector3d odom_p = odom_pos_;
    Eigen::Vector3d target_p = target_pos_;

    if(!rogmap_->isLineFreeInflate(odom_p, target_p)) return false;

    double odom_yaw = odom_yaw_;
    Eigen::AngleAxisd yawAngle(odom_yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> q(yawAngle);
    Eigen::Matrix3d R = q.matrix();
    Eigen::Vector3d pe(0, 0, LI_extrinsic_);
    Eigen::Vector3d pb = R.transpose() * (target_p - odom_p) - pe;
    double px = pb(0), py = pb(1), pz = pb(2);
    double n2_pb = sqrt(px * px + py * py);
    double n_pb = sqrt(px * px + py * py + pz * pz); 
    double tanA = tan(fov_ctr_theta_);
    Eigen::Vector3d pc(px, py, tanA * n2_pb);
    double pen_vert = cos(fov_theta_ + 0.05) - pb.dot(pc) / n_pb / pc.norm();

    if(flag_omni_fov_){
      return pen_vert <= 0;
    }else{
      if(pen_vert > 0) return false;
      if((1 - pb(0) / n2_pb) > 0.03) return false;
      return true;
    }
  }

  void STReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    tf::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    odom_yaw_ = yaw;  

    have_odom_ = true;

    // visualize FOV
    
    Eigen::Quaterniond eigen_q(msg->pose.pose.orientation.w,
                              msg->pose.pose.orientation.x,
                              msg->pose.pose.orientation.y,
                              msg->pose.pose.orientation.z);
    Eigen::Matrix3d R = eigen_q.toRotationMatrix();

    double visual_len = 3.7;
    if(flag_omni_fov_){
      Eigen::Vector3d pe(0, 0, 0);
      Eigen::Vector3d pb = R.transpose() * (target_pos_ - odom_pos_) - pe;
      double x = pb(0);
      double y = pb(1);

      double fov_theta_ub = fov_ctr_theta_ > 0 ? fov_ctr_theta_ + fov_theta_ + 0.05 : fov_ctr_theta_ + fov_theta_ + 0.12; 
      double fov_theta_lb = fov_ctr_theta_ > 0 ? fov_ctr_theta_ - fov_theta_ - 0.05 : fov_ctr_theta_ - fov_theta_ - 0.12;
      //upsidedown mid360 has larger fov clearance to compensate

      double fov_z_ub = tan(fov_theta_ub) * sqrt(x * x + y * y);
      double fov_z_lb = tan(fov_theta_lb) * sqrt(x * x + y * y);
      Eigen::Vector3d fov_vecB1(x, y, fov_z_ub);
      Eigen::Vector3d fov_vecB2(x, y, fov_z_lb);
      fov_vecB1 = visual_len * fov_vecB1.normalized();
      fov_vecB2 = visual_len * fov_vecB2.normalized();
      Eigen::Vector3d fov_ptW1 = R * fov_vecB1 + odom_pos_;
      Eigen::Vector3d fov_ptW2 = R * fov_vecB2 + odom_pos_;
      std::vector<traj_opt::facet> facet_list;
      traj_opt::facet fov_facet;
      fov_facet.a_ = odom_pos_;
      fov_facet.b_ = fov_ptW1;
      fov_facet.c_ = fov_ptW2;
      facet_list.emplace_back(fov_facet);
   
      visualization_->visualize_mesh_and_edge(facet_list, "drone_fov", display::Color::red, 0.08);
      
    }else{
      double avia_theta = fov_theta_ + 0.2;
      double xc = visual_len * cos(avia_theta);
      double yc = visual_len * sin(avia_theta);
      double zc = visual_len * tan(avia_theta);

      Eigen::Vector3d vtc_1, vtc_2, vtc_3, vtc_4;
      Eigen::Vector3d vtw_1, vtw_2, vtw_3, vtw_4;
      vtc_1 << xc, yc, zc;
      vtc_2 << xc, yc, -zc;
      vtc_3 << xc, -yc, -zc;
      vtc_4 << xc, -yc, zc;
      vtw_1 = R * vtc_1 + odom_pos_;
      vtw_2 = R * vtc_2 + odom_pos_;
      vtw_3 = R * vtc_3 + odom_pos_;
      vtw_4 = R * vtc_4 + odom_pos_;
      std::vector<traj_opt::facet> facet_list;
      traj_opt::facet fov_facet;
      fov_facet.a_ = odom_pos_;
      fov_facet.b_ = vtw_1;
      fov_facet.c_ = vtw_2;
      facet_list.emplace_back(fov_facet);
      fov_facet.a_ = odom_pos_;
      fov_facet.b_ = vtw_2;
      fov_facet.c_ = vtw_3;
      facet_list.emplace_back(fov_facet);
      fov_facet.a_ = odom_pos_;
      fov_facet.b_ = vtw_3;
      fov_facet.c_ = vtw_4;
      facet_list.emplace_back(fov_facet);
      fov_facet.a_ = odom_pos_;
      fov_facet.b_ = vtw_1;
      fov_facet.c_ = vtw_4;

      facet_list.emplace_back(fov_facet);
      visualization_->visualize_mesh_and_edge(facet_list, "drone_fov", display::Color::blue, 0.08);
    }
    
  }

  void STReplanFSM::targetCallback(const nav_msgs::OdometryConstPtr &msg)
  {
      target_pos_(0) = msg->pose.pose.position.x;
      target_pos_(1) = msg->pose.pose.position.y;
      target_pos_(2) = msg->pose.pose.position.z;

      target_vel_(0) = msg->twist.twist.linear.x;
      target_vel_(1) = msg->twist.twist.linear.y;
      target_vel_(2) = msg->twist.twist.linear.z;
      if(target_vel_.norm() < 0.20) target_vel_ << 0,0,0;

      target_t_ = msg->header.stamp.toSec(); 
  }


  void STReplanFSM::triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    have_trigger_ = true; 
    have_target_ = true;
  }

  void STReplanFSM::RecvBroadcastMINCOTrajCallback(const swarm_msgs::MincoTrajConstPtr &msg)
  {

    const size_t recv_id = (size_t)msg->drone_id;
    if ((int)recv_id == drone_id_) // myself
      return;

    if (msg->drone_id < 0)
    {
      ROS_ERROR("[Recv] drone_id < 0 is not allowed in a swarm system!");
      if(print_log_){
        std::cout << "[Recv] drone_id < 0 is not allowed in a swarm system!" << std::endl;
        fsm_log_ << "[Recv] drone_id < 0 is not allowed in a swarm system!" << std::endl;
      }
      return;
    }
    if (msg->order != 7)
    {
      ROS_ERROR("[Recv] Only support trajectory order equals 7 now!");
      if(print_log_){
        std::cout << "[Recv] Only support trajectory order equals 7 now!" << std::endl;
        fsm_log_ << "[Recv] Only support trajectory order equals 7 now!" << std::endl;
      }
      return;
    }
    if (msg->duration.size() != (msg->inner_x.size() + 1))
    {
      ROS_ERROR("[Recv] WRONG trajectory parameters.");
      if(print_log_){
        std::cout << "[Recv] WRONG trajectory parameters." << std::endl;
        fsm_log_ << "[Recv] WRONG trajectory parameters." << std::endl;
      }
      return;
    }

    if (traj_set_.swarm_traj.size() > recv_id &&
        traj_set_.swarm_traj[recv_id].drone_id == (int)recv_id &&
        msg->start_time.toSec() - traj_set_.swarm_traj[recv_id].start_time <= 0)
    {
      ROS_WARN("[Recv] Received drone %d's trajectory out of order or duplicated, abandon it.", (int)recv_id);
      if(print_log_){
        std::cout << "[Recv] Received drone " <<  (int)recv_id << "'s trajectory out of order or duplicated, abandon it." << std::endl;
        fsm_log_ << "[Recv] Received drone " <<  (int)recv_id << "'s trajectory out of order or duplicated, abandon it." << std::endl;
      }
      return;
    }

    ros::Time t_now = ros::Time::now();
    if (abs((t_now - msg->start_time).toSec()) > 0.25)
    {
      if (abs((t_now - msg->start_time).toSec()) < 10.0) // 10 seconds offset, more likely to be caused by unsynced system time.
      {
        ROS_ERROR("[Recv] Time stamp diff: Local - Remote Agent %d = %fs",
                 msg->drone_id, (t_now - msg->start_time).toSec());
        if(print_log_){
          std::cout << "[Recv] Time stamp diff: Local - Remote Agent " <<  msg->drone_id << "=" << (t_now - msg->start_time).toSec() << "s" << std::endl;
          fsm_log_ << "[Recv] Time stamp diff: Local - Remote Agent " <<  msg->drone_id << "=" << (t_now - msg->start_time).toSec() << "s" << std::endl;
        }
      }
      else
      {
        ROS_ERROR("[Recv] Time stamp diff: Local - Remote Agent %d = %fs, swarm time seems not synchronized, abandon!",
                  msg->drone_id, (t_now - msg->start_time).toSec());
        if(print_log_){
          std::cout << "[Recv] Time stamp diff: Local - Remote Agent " <<  msg->drone_id << "=" << (t_now - msg->start_time).toSec() << "s,warm time seems not synchronized, abandon!" << std::endl;
          fsm_log_ << "[Recv] Time stamp diff: Local - Remote Agent " <<  msg->drone_id << "=" << (t_now - msg->start_time).toSec() << "s,warm time seems not synchronized, abandon!" << std::endl;
        }
        return;
      }
    }

    /* Fill up the buffer */
    
    swarm_data_mutex_.lock();
   
    if (traj_set_.swarm_traj.size() <= recv_id) //Should not be inside this sec
    {
      ROS_ERROR_STREAM("[Recv] swarm buffer of " << recv_id << " should be filled up before SwarmTrajRecv.");
      if(print_log_){
        std::cout << "[Recv] swarm buffer of " << recv_id << " should be filled up before SwarmTrajRecv." << std::endl;
        fsm_log_ << "[Recv] swarm buffer of " << recv_id << " should be filled up before SwarmTrajRecv." << std::endl;
      }
      for (size_t i = traj_set_.swarm_traj.size(); i <= recv_id; i++)
      {
        TrajData blank;
        blank.drone_id = -1;
        blank.have_yaw = false;
        traj_set_.swarm_traj.push_back(blank);
      }
    }else if( traj_set_.swarm_traj[recv_id].drone_id != (int)recv_id ){
        ROS_ERROR_STREAM("[Recv] ID in buffer of " << recv_id << " should be set by TmtOdomCB.");
        if(print_log_){
          std::cout << "[Recv] ID in buffer of " << recv_id << " should be set by TmtOdomCB." << std::endl;
          fsm_log_ << "[Recv] ID in buffer of " << recv_id << " should be set by TmtOdomCB." << std::endl;
        }
    }

    if(traj_set_.swarm_traj[recv_id].is_preset_traj)
    {
      tmtodom_recv_count_[recv_id] = INT_MIN;
    }

    /* Store data */
    traj_set_.swarm_traj[recv_id].drone_id = recv_id;
    traj_set_.swarm_traj[recv_id].traj_id = msg->traj_id;
    traj_set_.swarm_traj[recv_id].start_time = msg->start_time.toSec();
    traj_set_.swarm_traj[recv_id].is_preset_traj = false;
    
    int piece_nums = msg->duration.size();
    Eigen::Matrix<double, 3, 4> headState, tailState;
    headState << msg->start_p[0], msg->start_v[0], msg->start_a[0], msg->start_j[0],
                 msg->start_p[1], msg->start_v[1], msg->start_a[1], msg->start_j[1],
                 msg->start_p[2], msg->start_v[2], msg->start_a[2], msg->start_j[2];

    tailState << msg->end_p[0], msg->end_v[0], msg->end_a[0], msg->end_j[0],
                 msg->end_p[1], msg->end_v[1], msg->end_a[1], msg->end_j[1],
                 msg->end_p[2], msg->end_v[2], msg->end_a[2], msg->end_j[2];
    Eigen::MatrixXd innerPts(3, piece_nums - 1);
    Eigen::VectorXd durations(piece_nums);
    for (int i = 0; i < piece_nums - 1; i++)
      innerPts.col(i) << msg->inner_x[i], msg->inner_y[i], msg->inner_z[i];
    for (int i = 0; i < piece_nums; i++)
      durations(i) = msg->duration[i];
    
    minco::MINCO_S4 MSO;
    MSO.reset(piece_nums);
    MSO.generate(headState, tailState, innerPts, durations);

    Trajectory trajectory = MSO.getTraj();
    traj_set_.swarm_traj[recv_id].traj = trajectory;
    traj_set_.swarm_traj[recv_id].duration = trajectory.getTotalDuration();
    traj_set_.swarm_traj[recv_id].start_pos = trajectory.getPos(0.0);
    
    visualization_->visualize_traj(trajectory, "teammate_traj_" + to_string(recv_id));

    /* Check if all teammate information collected */
    if(!have_all_tmt_info_)
    {
      bool have_all_tmt{true};
      for(int i = 0; i < candi_tracker_num_; i++)
      {
        if(candi_tracker_ids_[i] == drone_id_) continue;
        if(traj_set_.swarm_traj.size() <= candi_tracker_ids_[i] || 
           traj_set_.swarm_traj.at(candi_tracker_ids_[i]).drone_id != candi_tracker_ids_[i])
           {
            have_all_tmt = false;
            break;
           }
      }
      have_all_tmt_info_ = have_all_tmt;
    }

    swarm_data_mutex_.unlock();

    //set the teammateOdom to avoid if this teammate is ongoing final traj
    if(msg->is_final_traj && 
      !traj_set_.swarm_traj[recv_id].is_final_traj)
    {
      traj_set_.swarm_traj[recv_id].is_final_traj = true;
      int teammate_idx_in_list = (int)(find(candi_tracker_ids_.begin(), candi_tracker_ids_.end(), recv_id) - candi_tracker_ids_.begin());
      tmtodom_recv_count_[recv_id] = 0;
      if(print_log_){
        std::cout << "[Leave] Revoking odomTraj from drone " << recv_id << std::endl;
        fsm_log_ << "[Leave] Revoking odomTraj from drone " << recv_id << std::endl;
      }
      teammate_odom_subscribers_[teammate_idx_in_list] = nh_.subscribe<nav_msgs::Odometry>("/teammate_odom/UAV" + to_string(candi_tracker_ids_[teammate_idx_in_list]), 1000, 
                                 boost::bind(&STReplanFSM::teammateOdomCallback, this, _1, candi_tracker_ids_[teammate_idx_in_list]));
    }
  }

  void STReplanFSM::teammateOdomCallback(const nav_msgs::OdometryConstPtr &msg, int teammate_id)
  {
    int& ctr = tmtodom_recv_count_[teammate_id]; 
    if(ctr < 0){
      auto it = find(candi_tracker_ids_.begin(), candi_tracker_ids_.end(), teammate_id);
      teammate_odom_subscribers_[(int)(it - candi_tracker_ids_.begin())].shutdown(); 
    }
    ctr++;
    if(ctr % 10 != 1) return;

    Eigen::Vector3d tmt_odom_pos, tmt_odom_vel;
    tmt_odom_pos(0) = msg->pose.pose.position.x;
    tmt_odom_pos(1) = msg->pose.pose.position.y;
    tmt_odom_pos(2) = msg->pose.pose.position.z;
    tmt_odom_vel(0) = msg->twist.twist.linear.x;
    tmt_odom_vel(1) = msg->twist.twist.linear.y;
    tmt_odom_vel(2) = msg->twist.twist.linear.z;
    Trajectory tmt_odom_traj;
    Trajectory1D tmt_odom_traj_yaw;
    traj_opt_->generate_stop_traj(tmt_odom_pos, 0.0, tmt_odom_pos + 0.2 * tmt_odom_vel, 0.0, tmt_odom_traj, tmt_odom_traj_yaw);

    swarm_data_mutex_.lock();

    if (traj_set_.swarm_traj.size() <= teammate_id)
    {
      for (size_t i = traj_set_.swarm_traj.size(); i <= teammate_id; i++)
      {
        TrajData blank;
        blank.drone_id = -1;
        blank.have_yaw = false;
        traj_set_.swarm_traj.push_back(blank);
      }
    }

    /* Store data */ 
    traj_set_.swarm_traj[teammate_id].drone_id = teammate_id;
    traj_set_.swarm_traj[teammate_id].traj_id = 0;
    traj_set_.swarm_traj[teammate_id].start_time = ros::Time::now().toSec();
    traj_set_.swarm_traj[teammate_id].is_preset_traj = !traj_set_.swarm_traj[teammate_id].is_final_traj;
    traj_set_.swarm_traj[teammate_id].traj = tmt_odom_traj;
    traj_set_.swarm_traj[teammate_id].duration = tmt_odom_traj.getTotalDuration();
    traj_set_.swarm_traj[teammate_id].start_pos = tmt_odom_pos;

    /* Check if all teammate information collected */
    if(!have_all_tmt_info_)
    {
      bool have_all_tmt{true};
      for(int i = 0; i < candi_tracker_num_; i++)
      {
        if(candi_tracker_ids_[i] == drone_id_) continue;
        if(traj_set_.swarm_traj.size() <= candi_tracker_ids_[i] || 
           traj_set_.swarm_traj.at(candi_tracker_ids_[i]).drone_id != candi_tracker_ids_[i])
           {
            have_all_tmt = false;
            break;
           }
      }
      have_all_tmt_info_ = have_all_tmt;
    }
    swarm_data_mutex_.unlock();
  }


  void STReplanFSM::safetyCheckCallback(const ros::TimerEvent &e)
  { 

    if(!have_traj_) return;
    if(force_hover_) return; //Already in emergency
    
    string safety_case;

    if(have_traj_ && !have_target_)//Target lost not likely in known odom
    {
      ROS_INFO_STREAM("\033[31;1m [Safe] Target Lost. Emergency stop!\033[0m");
      if(print_log_){
        std::cout << "[Safe] Target Lost. Emergency stop!" << std::endl;
        fsm_log_ << "[Safe] Target Lost. Emergency stop!" << std::endl;
      }
      safety_case = "TGTLOS";
      force_hover_ = true;
      changeFSMExecState(EMERGENCY_STOP, safety_case);
      return;
    }

    if(have_target_ && fabs(ros::Time::now().toSec() - target_t_) > 0.7)
    {
      ROS_INFO_STREAM("\033[31;1m [Safe] Target SUSpend. Emergency stop!\033[0m");
      if(print_log_){
        std::cout << "[Safe] Target SUSpend. Emergency stop!" << std::endl;
        fsm_log_ << "[Safe] Target SUSpend. Emergency stop!" << std::endl;
      }
      safety_case = "TGTSUS";
      force_hover_ = true;
      changeFSMExecState(EMERGENCY_STOP, safety_case);
      return;
    }
    
    TrajData *data = &traj_set_.local_traj;
    if(exec_state_ == WAIT_INIT_TARGET || data->traj_id <= 0)
      return;

    constexpr double time_step = 0.02;
    double t_cur_global = ros::Time::now().toSec();
    double t_cur = t_cur_global - data->start_time;
    double clearance = 0.8 * traj_opt_->getSwarmClc();
    double target_clearance = min(0.7 * (desired_dist_ - tolerance_d_), 0.8);

    if(!data->is_stop_traj)
    {
      if(target_vel_.norm() > target_vel_stop_bar_ && (t_cur > data->duration - 0.01))
      {
        ROS_INFO_STREAM("\033[31;1m [Safe] Replan Suspend. No following Plan. Emergency stop!\033[0m");
        if(print_log_){
          std::cout << "[Safe] Replan Suspend. No following Plan. Emergency stop!" << std::endl;
          fsm_log_ << "[Safe] Replan Suspend. No following Plan. Emergency stop!" << std::endl;
        }
        safety_case = "REPSUS";
        force_hover_ = true;
        changeFSMExecState(EMERGENCY_STOP, safety_case);
        return;
      }
    }

    double t_temp;//store the predicted collision time
    double t_glb, t_lcl;
    bool occ = false;

    for (double t = t_cur; t < data->duration; t += time_step)
    {
      t_glb = t - t_cur + t_cur_global;

      //Check environment collision 
      if (rogmap_->isOccupiedInflateWithNbr(data->traj.getPos(t)))
      {
        safety_case = "OBSOcc";
        t_temp = t;
        occ = true;
        break;
      }

      // Check height(z-thres hardcoded)
      if (data->traj.getPos(t).z() < min_flight_height_ - 0.1)
      {
        safety_case = "ZAXOcc";
        t_temp = t;
        occ = true;
        break;
      }

      //Check swarm collision 
      for (size_t id = 0; id < traj_set_.swarm_traj.size(); id++)
      {
        if ((traj_set_.swarm_traj.at(id).drone_id < 0) ||
            (traj_set_.swarm_traj.at(id).drone_id != (int)id) ||
            (traj_set_.swarm_traj.at(id).drone_id == drone_id_))
        {
          continue;
        }

        double t_swm = t_glb - traj_set_.swarm_traj.at(id).start_time;
        if (t_swm < 0)//unlikely minor case
          continue;

        if (t_swm > traj_set_.swarm_traj.at(id).duration)
          continue;

        if ((data->traj.getPos(t) - traj_set_.swarm_traj.at(id).traj.getPos(t_swm)).norm() 
            < clearance)
        {
          safety_case = "SWMOcc";
          t_temp = t;
          occ = true;
          break;
        }
      }
      if(occ) break;
    }

    //Check target collision 
    for(int i = 0; i < view_path_.size(); i++)
    {
      t_lcl = predict_stamp_ + i * predict_dt_ - data->start_time;
      if(t_lcl < 0) continue;
      t_lcl = t_lcl > data->duration ? data->duration : t_lcl;
  
      if((view_path_[i] - data->traj.getPos(t_lcl)).norm() < target_clearance)
      {
        safety_case = "TGTOcc";
        t_temp = t_lcl;
        occ = true;
        break;
      }
    }
    
    if(occ)
    {
      //Handle the collision immediately, make a chance
      ROS_INFO_STREAM("\033[31;1m [Safe] Collision detected. Make a chance now: " << safety_case << "\033[0m");
      if(print_log_){
        std::cout << "\033[31;1m [Safe] Collision detected. Make a chance now: " << safety_case << "\033[0m" << std::endl;
        fsm_log_ << "\033[31;1m [Safe] Collision detected. Make a chance now: " << safety_case << "\033[0m" << std::endl;
      }
      if(callReplan())
      {
        ROS_WARN("\033[35;1m [Safe] Make Chance Replan Succeed\033[0m");
        if(print_log_){
          std::cout << "\033[35;1m [Safe] Make Chance Replan Succeed\033[0m" << std::endl;
          fsm_log_ << "\033[35;1m [Safe] Make Chance Replan Succeed\033[0m" << std::endl;
        }
        changeFSMExecState(EXEC_TRAJ, safety_case);
      }else{
        if(t_temp - t_cur < emergency_time_ - 1.5 * replan_time_budget_) 
        {
          //To Emergency Stage
          ROS_INFO_STREAM("\033[31;1m [Safe] Emergency stop! Collision time = "<< t_temp - t_cur << "\033[0m");
          if(print_log_){
            std::cout <<"\033[31;1m [Safe] Emergency stop! Collision time = "<< t_temp - t_cur << "\033[0m"<< std::endl;
            fsm_log_ <<"\033[31;1m [Safe] Emergency stop! Collision time = "<< t_temp - t_cur << "\033[0m"<< std::endl;
          }
          force_hover_ = true;
          changeFSMExecState(EMERGENCY_STOP, safety_case);
        }
        else
        { 
          //Retry Replan stage
          ROS_WARN("\033[35;1m [Safe] Emergency Back to replan stage\033[0m");
          if(print_log_){
            std::cout <<"\033[35;1m [Safe] Emergency Back to replan stage\033[0m"<< std::endl;
            fsm_log_ <<"\033[35;1m [Safe] Emergency Back to replan stage\033[0m"<< std::endl;
          }
          changeFSMExecState(REPLAN_TRAJ, safety_case);
        }
      } 
    }
  }


  void STReplanFSM::cpumemCallback(const ros::TimerEvent &e)
  {
    if(!print_log_) return;
    // float cpu_usage_ratio = GetCpuUsageRatio(current_pid_, totalcputime_, procputime_);
    // float memory_usage = GetMemoryUsage(current_pid_);
    // cout<<"-------------------------"<< endl;
    // cout << "CPU Usage Ratio: " << cpu_usage_ratio * 100 << "%" <<endl;
    // cout << "Memory Usage: " << memory_usage/1024 <<"GB" <<endl;
    // cout<<"-------------------------"<< endl;

    // cpumem_log_ << setiosflags(ios::fixed) << setprecision(5) << ros::Time::now().toSec() << ", " 
    // << cpu_usage_ratio * 100 << "%, " << memory_usage/1024 <<"GB, " << endl;
  }


  //Update the UAVs within the distance threshold
  void STReplanFSM::updateSwarmTmtList(const Eigen::Vector3d &start_p)
  {

    for(auto it : swarm_tmt_list_) it.clear();
    swarm_tmt_list_.clear();

    if(swarm_id_list_.size() <= 1)
      return;

    double t_now = ros::Time::now().toSec();
    std::vector<Eigen::Vector3d> nbr_seq;
    Eigen::Vector3d swarm_p, swarm_v;
 
    std::vector<int> valid_swarm_id_list_;
    for(auto tmt_id : swarm_id_list_)
    {
      if(tmt_id == drone_id_ || traj_set_.swarm_traj[tmt_id].drone_id == drone_id_)
          continue;
      if((traj_set_.swarm_traj[tmt_id].drone_id != tmt_id))
          continue;
      if((traj_set_.swarm_traj[tmt_id].drone_id < 0))
          continue;
      
      double traj_i_start_time = traj_set_.swarm_traj[tmt_id].start_time;
      if (t_now < traj_i_start_time + traj_set_.swarm_traj[tmt_id].duration)
      {
        swarm_p = traj_set_.swarm_traj[tmt_id].traj.getPos(t_now - traj_i_start_time);
      }
      else
      {
        double exceed_time = t_now - (traj_i_start_time + traj_set_.swarm_traj[tmt_id].duration);
        swarm_v = traj_set_.swarm_traj[tmt_id].traj.getVel(traj_set_.swarm_traj[tmt_id].duration);
        swarm_p = traj_set_.swarm_traj[tmt_id].traj.getPos(traj_set_.swarm_traj[tmt_id].duration) +
                  exceed_time * swarm_v;
      }
 
      valid_swarm_id_list_.emplace_back(tmt_id);
      
    }
    double pt_time;

    for(double eval_t = 0; eval_t <= predict_T_; eval_t += predict_dt_){
      nbr_seq.clear();
      for(auto tmt_id : valid_swarm_id_list_)
      {
        double traj_i_start_time = traj_set_.swarm_traj[tmt_id].start_time;
        pt_time = t_now + eval_t;
        if (pt_time < traj_i_start_time + traj_set_.swarm_traj[tmt_id].duration)
        {
          swarm_p = traj_set_.swarm_traj[tmt_id].traj.getPos(pt_time - traj_i_start_time);
        }
        else
        {
          double exceed_time = pt_time - (traj_i_start_time + traj_set_.swarm_traj[tmt_id].duration);
          swarm_v = traj_set_.swarm_traj[tmt_id].traj.getVel(traj_set_.swarm_traj[tmt_id].duration);
          swarm_p = traj_set_.swarm_traj[tmt_id].traj.getPos(traj_set_.swarm_traj[tmt_id].duration) +
                    exceed_time * swarm_v;
        }
        nbr_seq.emplace_back(swarm_p);
      }
      if(!nbr_seq.empty()) swarm_tmt_list_.emplace_back(nbr_seq);
    }
  }

  bool STReplanFSM::callStop(const Eigen::Vector3d& stop_pos, const double& stop_yaw)
  {
    std::lock_guard<std::mutex> lock(replan_mutex_);

    ROS_INFO_STREAM("\033[31;1m [CallStop] Called Stop!\033[0m");
    if(print_log_){
      std::cout << "[Safe] [CallStop] Called Stop!" << std::endl;
      fsm_log_ << "[Safe] [CallStop] Called Stop!" << std::endl;
    }

    //Generate Stop Traj
    Trajectory stop_traj;
    Trajectory1D stop_traj_yaw;

    //stop in-situ
    traj_opt_->generate_stop_traj(stop_pos, stop_yaw, stop_pos, stop_yaw, stop_traj, stop_traj_yaw);
    
    visualization_->visualize_traj(stop_traj, "minco_traj");
    traj_set_.setTraj(stop_traj, stop_traj_yaw, ros::Time::now().toSec(), drone_id_, true, false);

    //Broadcast the traj
    traj_utils::PolyTraj poly_msg;
    swarm_msgs::MincoTraj MINCO_msg;
    polyTraj2Msg(poly_msg, MINCO_msg);
    poly_traj_pub_.publish(poly_msg);
    broadcast_polytraj_pub_.publish(MINCO_msg);
    return true;
  }

}

// namespace st_planner





