#include <path_searching/kino_search.hpp>

//hard-tuned baseline params
#define W_ACC      0.06
#define W_ZACC     20.0
#define W_VIS      60.0
#define W_DIS      2.00
#define W_FRM      0.05
#define W_BEA      10.0  
#define W_DIST_UB  100.0
#define W_DIST_LB  5.00

using namespace kino_search;

KinoSearch::~KinoSearch()
{
    for (int i = 0; i < allocate_num_; i++)
    {
        delete node_pool_[i];
    }
}

void KinoSearch::setParam(ros::NodeHandle &nh)
{
    nh.param("planning/predict_dt", tau_, 0.2);//prediction time step
    nh.param("planning/predict_T", tau_dur_, 2.0);//prediction time horizon
    nh.param("planning/track_dist", track_dist_, 1.9);

    nh.param("planning/tolerance_d", tolerance_d_, 0.3);
    nh.param("planning/fov_theta", fov_theta_, 0.26);
    nh.param("planning/fov_ctr_theta", fov_ctr_theta_, 0.0);
    nh.param("planning/min_flight_height", min_flight_height_, 0.3);

    nh.param("search/max_vel", v_max_, 2.0);
    nh.param("search/max_acc", a_max_init_, 2.0);
    nh.param("search/max_z", z_max_init_, 0.4);
    nh.param("search/allocate_num", allocate_num_, -1);
    nh.param("search/swarm_clearance", swarm_clearance_, 1.0);
    nh.param("search/occlusion_clearance", occlusion_clearance_, -1.0);
    nh.param("search/check_num", check_num_, 2);
    nh.param("search/topo_check_time", topo_check_time_, -1.0);

    nh.param("search/wei_vis", wei_vis_, 1.0);
    nh.param("search/wei_bea", wei_bea_, 1.0);
    nh.param("search/wei_sep", wei_sep_, 1.0);
    nh.param("search/wei_dist", wei_dist_, 1.0);

    ROS_WARN_STREAM("[Search] param: max_vel: " << v_max_);
    ROS_WARN_STREAM("[Search] param: max_acc: " << a_max_init_);
    ROS_WARN_STREAM("[Search] param: max_z: " << z_max_init_);
    ROS_WARN_STREAM("[Search] param: tau: " << tau_);
    ROS_WARN_STREAM("[Search] param: tau_duration: " << tau_dur_);
    ROS_WARN_STREAM("[Search] param: track_dist: " << track_dist_);
    ROS_WARN_STREAM("[Search] param: tolerance_d: " << tolerance_d_);
    ROS_WARN_STREAM("[Search] param: CheckNum: " << check_num_);
    ROS_WARN_STREAM("[Search] param: swarm_clearance: " << swarm_clearance_);
    ROS_WARN_STREAM("[Search] param: occlusion_clearance: " << occlusion_clearance_);
    ROS_WARN_STREAM("[Search] param: topo_check_time: " << topo_check_time_);
    ROS_WARN_STREAM("[Search] param: fov_theta: " << fov_theta_);
    ROS_WARN_STREAM("[Search] param: fov_ctr_theta: " << fov_ctr_theta_);
    ROS_WARN_STREAM("[Search] param: allocate_num: " << allocate_num_);
}

void KinoSearch::setEnvironment(std::shared_ptr<rog_map::ROGMap>& env)
{
  this->rogmap_ = env;
  this->resolution_ = rogmap_->getResolution();
  this->ceil_height_ = rogmap_->getCeilHeight() - 2 * resolution_;
  this->ground_height_ = rogmap_->getGroundHeight() + 2 * resolution_;
}

void KinoSearch::setMaxRate(double vmax, double amax)
{
  if(vmax > this->v_max_) 
  {
    this->v_max_ = vmax;
  }
  this->a_max_init_ = amax;
}

void KinoSearch::setVSDFs(VSDFList *vsdf_list)
{
  this->vsdf_list_ = vsdf_list;
}

void KinoSearch::init(int drone_id, TmtList* swarm_tmt_ptr)
{
  drone_id_ = drone_id;
  swarm_tmt_ptr_ = swarm_tmt_ptr;

  phi_tau_ = Eigen::MatrixXd::Identity(6, 6);
  this->inv_resolution_ = 1.0 / resolution_;
  node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++)
  {
    node_pool_[i] = new Node;
  }

  use_node_num_ = 0;
  last_path_.clear();

  last_vel_path_.clear();
  last_acc_path_.clear();
}

void KinoSearch::resetSearch()
{

  //Reset Buffer
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++)
  {
    NodePtr node = node_pool_[i];
    node->parent = NULL;
    node->node_state = UNVISITED;
  }
  use_node_num_ = 0;
}

int KinoSearch::search(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel,
                       const std::vector<Eigen::Vector3d>& tgt_p, 
                       double des_v_max, 
                       double des_a_max)
{


  if(start_vel.norm() < des_v_max) this->v_max_ = des_v_max;
  this->a_max_init_ = des_a_max;

  int search_res = SEARCH_STATE::VALID;

  //Set target sequence
  tgt_pos_ = tgt_p;
  start_pos_ = start_pos;
  start_vel_ = start_vel;
  a_max_ = a_max_init_;
  z_max_ = z_max_init_;

  if(rogmap_->isOccupiedInflate(start_pos_))
  {
    ROS_WARN_STREAM("\033[34;1m[Kino] Start point is not valid(OBS). Return.\033[0m");
    std::cout << "[Kino] Start point is not valid(OBS). Return." << std::endl;
    search_res = SEARCH_STATE::INIT_OBS;
    return search_res;
  }

  if (start_vel_.norm() > (v_max_ + 0.4))
  {
    ROS_WARN_STREAM("\033[34;1m[Kino] Invalid init velocity. Return.\033[0m");
    std::cout << "[Kino] Invalid init velocity. Return." << std::endl;
    search_res = SEARCH_STATE::INIT_VEL;
    return search_res;
  }

  int kino_trial_num{0};
  do{
    search_res = searchOnce(start_pos_, start_vel_);
    kino_trial_num++;
    z_max_ *= 1.5;
    a_max_ *= 1.2;
  }while(search_res == SEARCH_STATE::SET_EMPTY && kino_trial_num < 5);

  if(search_res == SEARCH_STATE::VALID && kino_trial_num > 1)
  {
    ROS_WARN_STREAM("\033[34;1m[Kino] Kino Extra Trial Succeed!.\033[0m");
    std::cout << "[Kino] Kino Extra Trial Succeed!." << std::endl;
  }
  return search_res;
}


void KinoSearch::stateTransit(Eigen::Matrix<double, 6, 1>& state0,
                              Eigen::Matrix<double, 6, 1>& state1, 
                              Eigen::Vector3d um,
                              double tau)
{
  for (int i = 0; i < 3; ++i)
    phi_tau_(i, i + 3) = tau;

  Eigen::Matrix<double, 6, 1> integral;
  integral.head(3) = 0.5 * pow(tau, 2) * um; 
  integral.tail(3) = tau * um; 
  state1 = phi_tau_ * state0 + integral;
}

int KinoSearch::timeToIndex(double time)
{
  return (int)(time / tau_);
}

Eigen::Vector3i KinoSearch::posToIndex(Eigen::Vector3d pt)
{
  return rogmap_->posToGlobalIndex(pt);
}

int KinoSearch::searchOnce(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel)
{ 
  resetSearch();//Reset searcher
 
  t_now_ = ros::Time::now().toSec();
  double max_t{0.0};

  //Set first node
  NodePtr cur_node = node_pool_[0];
  cur_node->parent = NULL;
  cur_node->state.head(3) = start_pos; 
  cur_node->state.tail(3) = start_vel;
  cur_node->index = posToIndex(start_pos);
  cur_node->cost = 0.0;
  cur_node->heu = getHeu(0.0);
  cur_node->node_state = OPEN;
  cur_node->time = 0.0;
  cur_node->time_idx = timeToIndex(cur_node->time);

  open_set_.push(cur_node);
  use_node_num_ += 1;
  expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);

  //Args for each iter
  NodePtr expand_node = NULL;
  NodePtr terminate_node = NULL;
  Eigen::Matrix<double, 6, 1> cur_state;
  Eigen::Matrix<double, 6, 1> pro_state;
  Eigen::Matrix<double, 6, 1> xt_check;
  Eigen::Vector3d um(0,0,0);
  std::vector<Eigen::Vector3d> inputs_candi;
  std::vector<Eigen::Vector3d> inputs_prior;
  std::vector<double> fov_costs;

  Eigen::Vector3i pro_id;
  Eigen::Vector3d pos_check;
  std::vector<NodePtr> tmp_expand_nodes;

  Eigen::Vector3d pos_visual;
  Eigen::Matrix<double, 6, 1> xt_visual;
  

  // int prune_nums{0};
  while(!open_set_.empty())
  {
    cur_node = open_set_.top();
    if(cur_node->time >= tau_dur_)
    {
      terminate_node = cur_node;
      tracePath(terminate_node);
      return SEARCH_STATE::VALID;
    }

    open_set_.pop();
    cur_node->node_state = CLOSE;
    
    cur_state = cur_node->state;
    inputs_candi.clear();
    fov_costs.clear();
    inputs_prior.clear();
    tmp_expand_nodes.clear();
    int check_id = cur_node->time_idx + 1;

    for (double ax = -a_max_; ax <= a_max_ + 1e-3; ax += a_max_) //more robust
       for (double ay = -a_max_; ay <= a_max_ + 1e-3; ay += a_max_)
          for (double az = -z_max_; az <= z_max_ + 1e-3; az += z_max_)
          {
            inputs_prior.emplace_back(ax, ay, az);
          }
    
    for (int i = 0; i < inputs_prior.size(); i++)
    {
      um = inputs_prior[i];
      stateTransit(cur_state, pro_state, um, tau_);

      //Check if in close set

      pro_id = posToIndex(pro_state.head(3));
      NodePtr pro_node = expanded_nodes_.find(pro_id, check_id);
      if (pro_node != NULL && pro_node->node_state == CLOSE)
        continue;

      //Check if vel valid
      if (pro_state.segment(3, 3).norm() > (v_max_ + 0.4))
        continue;

      if (pro_state(2) <= min_flight_height_ - 0.1)
        continue;

      if (pro_state(2) <= ground_height_)
        continue;

      if (pro_state(2) >= ceil_height_)
        continue;

      //Check if safe
      bool is_occ = false;
      for (int k = 1; k <= check_num_; ++k)
      {
        double dt = tau_ * double(k) / double(check_num_);
        stateTransit(cur_state, xt_check, um, dt);
        pos_check = xt_check.head(3);

        if(rogmap_->isOccupiedInflate(pos_check))
        {
          is_occ = true;
          break;
        }
      }
      if (is_occ) 
        continue;

      //Check if topo not changed
      if((!last_path_.empty()) && (cur_node->time + tau_ <= topo_check_time_))
      {
        if(!rogmap_->isLineFreeInflate(pro_state.head(3), last_path_[cur_node->time_idx + 1], DBL_MAX))
          continue;
      }

      // Check if swarm safe
      if(!isSwarmSafe(pro_state.head(3), check_id))
        continue;
      
      //Check if target safe
      if(check_id > 2){
        double tgt_dist = (pro_state.head(3) - tgt_pos_[check_id]).norm() ;
        if(tgt_dist < 0.8 * (track_dist_ - tolerance_d_) || 
           tgt_dist > track_dist_ + 8.0 * tolerance_d_){ 
              continue;
        }
      }

      double tmp_cost, tmp_heu;
      
      tmp_cost = W_ACC * um.head(3).norm() + cur_node->cost + getCost(pro_state, cur_node->time + tau_);
      tmp_heu = getHeu(cur_node->time + tau_);

      bool prune = false;
      //Prune
      for (int j = 0; j < tmp_expand_nodes.size(); ++j)
      {
        expand_node = tmp_expand_nodes[j];
        if((pro_id - expand_node->index).norm() == 0  &&  expand_node->time_idx == check_id)
        {
          //Prune and reset the node with lower cost
          prune = true;
          // prune_nums++;
          if(tmp_cost < expand_node->cost)
          {
            expand_node->heu = tmp_heu;
            expand_node->cost = tmp_cost;
            expand_node->state = pro_state;
            expand_node->input = um;
          }
          break;
        }
      }
      
      //Not pruning
      if(!prune)
      {
        if(pro_node == NULL)//unvisit
        {
          pro_node = node_pool_[use_node_num_];
          pro_node->index = pro_id;
          pro_node->state = pro_state;
          pro_node->heu = tmp_heu;
          pro_node->cost = tmp_cost;
          pro_node->input = um;
          pro_node->parent = cur_node;
          pro_node->node_state = OPEN;
          pro_node->time = cur_node->time + tau_;
          pro_node->time_idx = check_id;

          //Add the node
          if(pro_node->time > max_t) max_t = pro_node->time;

          open_set_.push(pro_node);
          expanded_nodes_.insert(pro_id, pro_node->time_idx, pro_node);
          tmp_expand_nodes.push_back(pro_node);
          use_node_num_ += 1;

          if(use_node_num_ == allocate_num_ - 2)
          {
            ROS_WARN_STREAM("\033[34;1m[Kino] Run out of memory.\033[0m");
            std::cout << "[Kino] Run out of memory." << std::endl;
            return SEARCH_STATE::MEMS_UP;
          }
        }else if (pro_node->node_state == OPEN){
          if(tmp_cost < pro_node->cost)
          {
            pro_node->state = pro_state;
            pro_node->heu = tmp_heu;
            pro_node->cost = tmp_cost;
            pro_node->input = um;
            pro_node->parent = cur_node;

            if(pro_node->time > max_t) max_t = pro_node->time;
          }
        }else{
          ROS_WARN_STREAM("\033[34;1m[Kino] Error type in searching: " << pro_node->node_state << "\033[0m");
          std::cout << "[Kino] Error type in searching: " << pro_node->node_state << std::endl;
        }
      }//End of not pruning

    }//End of looping inputs
    
  }//End of main loop while(open_set)

  if(open_set_.empty()){
    ROS_WARN_STREAM("\033[34;1m[Kino] OpenSet EMPTY. Infeasible solution space. Return.\033[0m");
    std::cout << "[Kino] OpenSet EMPTY. Infeasible solution space. Return." << std::endl;
    return SEARCH_STATE::SET_EMPTY;
  }
  
}//End of search func


double KinoSearch::getHeu(double time)
{

  return 15.0 * (tau_dur_ - time);
}

void KinoSearch::tracePath(NodePtr end_node)
{
  NodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  while(cur_node->parent != NULL)
  {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }
  reverse(path_nodes_.begin(), path_nodes_.end());

  last_path_.clear();
  last_vel_path_.clear();
  last_acc_path_.clear();

  for(size_t i = 0; i < path_nodes_.size(); i++)
  {
    last_path_.emplace_back(path_nodes_[i]->state.head(3));
    last_vel_path_.emplace_back(path_nodes_[i]->state.tail(3));
    last_acc_path_.emplace_back(path_nodes_[i]->input);
  }
}

void KinoSearch::retrievePath(std::vector<Eigen::Vector3d>& path,
                              std::vector<Eigen::Vector3d>& vel_path,
                              std::vector<Eigen::Vector3d>& acc_path)
{
  path = last_path_;
  vel_path = last_vel_path_;
  acc_path = last_acc_path_;
}

bool KinoSearch::isBadVisibility(const Eigen::Vector3d& pos, const int& idx)
{
  if(idx < 2) return false;
  return (vsdf_list_->at(idx)->getCrtOcc(pos) == 1);
}

bool KinoSearch::isSwarmSafe(const Eigen::Vector3d& pos, const int& idx)
{
  if(swarm_tmt_ptr_->empty()) return true;

  const double clc_2 = (swarm_clearance_ * 1.3) * (swarm_clearance_ * 1.3);
  constexpr double a = 1.5, b = 1.0, inv_a2 = 1 / a / a, inv_b2 = 1 / b / b;
  for(auto nbr : swarm_tmt_ptr_->at(idx))
  {
    Eigen::Vector3d dist_vec = pos - nbr;
    double ellip_dist2 = dist_vec(2) * dist_vec(2) * inv_a2 + (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2;
    double dist2_err = clc_2 - ellip_dist2;
    double dist2_err2 = dist2_err * dist2_err;
    double dist2_err3 = dist2_err2 * dist2_err;
    if(dist2_err3 > 0)
    {return false; }
  }
  return true;
}


double KinoSearch::getDirectSepCost(const Eigen::Vector3d& pos, const int& idx)
{ 
  if(swarm_tmt_ptr_->empty()) return 0.0;

  double cost_c = -1;
  Eigen::Vector3d nbr_bearing;
  Eigen::Vector3d ego_bearing;
  Eigen::Vector3d tgt = tgt_pos_[idx];
  Eigen::Vector3d relative_pos = pos - tgt;
  ego_bearing = relative_pos.normalized();
  for(auto nbr : swarm_tmt_ptr_->at(idx))
  {
    nbr_bearing = (nbr - tgt).normalized();
    double dnorm = (nbr_bearing - ego_bearing).norm();
    if(dnorm < 1e-3)
    {
      return DBL_MAX;
    }else{
      cost_c *= dnorm;
    }
  }
  return wei_sep_ * W_FRM * cost_c;
}

double KinoSearch::getBeaCost(const Eigen::Vector3d& pos, const int& idx)
{
  if(swarm_tmt_ptr_->empty()) return 0.0;

  double bearing_cost{0};
  Eigen::Vector3d tgt = tgt_pos_[idx];
  double theta;
  Eigen::Vector3d vecx, vecn;
  vecx = pos - tgt;

  for(auto tmt : swarm_tmt_ptr_->at(idx))
  {
    vecn = tmt - tgt; 
    theta = acos(vecx.dot(vecn) / vecx.norm() / vecn.norm());
    
    if(1.2 * occlusion_clearance_ - theta > 0)
      bearing_cost += wei_bea_ * W_BEA * (1.2 * occlusion_clearance_ - theta);
  }
  return bearing_cost;
}

double KinoSearch::getDisCost(const Eigen::Vector3d& pos, const int& idx)
{
  double ub{track_dist_+tolerance_d_}, lb{track_dist_-tolerance_d_};
  double dist = (pos - tgt_pos_[idx]).norm();
  if(dist < lb){
    return wei_dist_ * W_DIST_LB * pow((lb - dist), 3.0);
  }else if(dist > ub){
    return W_DIST_UB * pow((dist - ub), 2.0);
  }else{
    return 0.0;
  }
}

double KinoSearch::getCost(Eigen::Matrix<double, 6, 1>& state0, 
                           double time)
{
  int time_id = timeToIndex(time);
  Eigen::Vector3d pos = state0.head(3);

  return getVisCost(pos, time_id) + getBeaCost(pos, time_id) + getDirectSepCost(pos, time_id);

}


double KinoSearch::getVisCost(const Eigen::Vector3d& pos, const int& idx)
{
  if(vsdf_list_->at(idx)->getCrtOcc(pos) == 1)
  {
    double vis_dist;
    vsdf_list_->at(idx)->evaluateSDTTrilinear(pos, vis_dist);
    return wei_vis_ * W_VIS * vis_dist;
  }else{
    return 0.0;
  }
}


void KinoSearch::pairsort(std::vector<int>& prior_id, const std::vector<double>& fov_cost, 
                          int total_num, int prior_num)
{
  std::pair<double, int> pairlist[total_num];
  for(int i = 0; i < total_num; i++)
  {
    pairlist[i].first = fov_cost[i];
    pairlist[i].second = i;
  }

  sort(pairlist, pairlist + total_num);
  prior_id.clear();
  for(int i = 0; i < prior_num; i++)
    prior_id.emplace_back(pairlist[i].second);
}




