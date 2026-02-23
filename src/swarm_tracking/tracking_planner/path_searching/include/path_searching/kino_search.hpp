#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>
#include "rog_map/rog_map.h"
#include "plan_env/vis_sdf.h"
#include <boost/functional/hash.hpp>
#include <queue>
#include <unordered_map>
#include <utility>
#include <traj_utils/traj_set.hpp>

template <typename T>
struct matrix_hash3 : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }
    return seed;
  }
};

namespace kino_search{

enum NodeState { OPEN,
                 CLOSE,
                 UNVISITED };

class Node {
public:
    Eigen::Matrix<double, 6, 1> state;//pos & vel & acc
    Eigen::Vector3i index;
    Eigen::Vector3d input; //jerk
    
    double time;
    int time_idx;
    double cost;
    double heu;
    Node* parent;
    NodeState node_state;

    Node()
    {
      parent = NULL;
      node_state = UNVISITED;
    }
    ~Node(){};
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef Node* NodePtr;
typedef std::vector<std::vector<Eigen::Vector3d>> TmtList;
typedef std::vector<VisSDF::Ptr> VSDFList;

class NodeComparator {
 public:
  bool operator()(NodePtr node1, NodePtr node2) {
    return node1->cost + node1->heu > node2->cost + node2->heu;
  }
};

class NodeHashTable{
private:
    std::unordered_map<Eigen::Vector4i, NodePtr, matrix_hash3<Eigen::Vector4i>>
    data_;

public: 
    NodeHashTable(/* args */) {}
    ~NodeHashTable() {}
    
    void insert(Eigen::Vector3i idx, int time_idx, NodePtr node) {
    data_.insert(std::make_pair(
        Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
    }

    NodePtr find(Eigen::Vector3i idx, int time_idx) {
        auto iter = data_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
        return iter == data_.end() ? NULL : iter->second;
    }

    void clear(){
        data_.clear();
    }
};

class KinoSearch{
private:

    int drone_id_;
    std::shared_ptr<rog_map::ROGMap> rogmap_;
    
    //Buffer
    vector<NodePtr> node_pool_;//storing the nodes
    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set_;//priority_que
    NodeHashTable expanded_nodes_;
    std::vector<NodePtr> path_nodes_; //To retrieve path
    int use_node_num_;
    double t_now_;
    
    //Data
    Eigen::Vector3d start_pos_, start_vel_, start_acc;
    TmtList* swarm_tmt_ptr_{NULL};
    TmtList* candi_tmt_ptr_{NULL};
    SwarmTrajData *swarm_trajs_{NULL};
    std::vector<Eigen::Vector3d> bearing_seq_;

    std::vector<Eigen::Vector3d> tgt_pos_;
    std::vector<Eigen::Vector3d> last_path_;

    std::vector<Eigen::Vector3d> last_vel_path_;
    std::vector<Eigen::Vector3d> last_acc_path_;

    Eigen::Matrix<double, 6, 6> phi_tau_;
    
    //Params
    int nbr_size_;
    double tau_;
    double tau_dur_;
    double v_max_, a_max_;
    double a_max_init_;
    double z_max_init_;

    double z_max_; 
    int allocate_num_;
    double resolution_, inv_resolution_;
    double swarm_clearance_, occlusion_clearance_, separation_clearance_;
    double track_dist_;

    double tolerance_d_;
    double fov_theta_;
    double fov_ctr_theta_;
    double ceil_height_;
    double ground_height_;

    VSDFList *vsdf_list_{NULL};
    
    int check_num_; 
    double topo_check_time_; 

    double wei_vis_, wei_bea_, wei_sep_, wei_dist_;
    double min_flight_height_;
    
    //helper funcs
    int timeToIndex(double time);
    void tracePath(NodePtr end_node);
    Eigen::Vector3i posToIndex(Eigen::Vector3d pt);

    //transit, cost, heu
    double getHeu(double time);

    void stateTransit(Eigen::Matrix<double, 6, 1>& state0,
                      Eigen::Matrix<double, 6, 1>& state1, 
                      Eigen::Vector3d um, //jerk
                      double tau);
    
    bool isSwarmSafe(const Eigen::Vector3d& pos, const int& idx);

    double getCost(Eigen::Matrix<double, 6, 1>& state0, 
                   double time);

    double getDisCost(const Eigen::Vector3d& pos, const int& idx);
    double getVisCost(const Eigen::Vector3d& pos, const int& idx);
    double getBeaCost(const Eigen::Vector3d& pos, const int& idx);
    double getDirectSepCost(const Eigen::Vector3d& pos, const int& idx);

    bool isBadVisibility(const Eigen::Vector3d& pos, const int& idx);

    void pairsort(std::vector<int>& prior_id, const std::vector<double>& fov_cost, 
                  int total_num, int prior_num);
public:

    //for visualization
    std::vector<Eigen::Vector3d> primitives_list_;

    //flags
    enum SEARCH_STATE
    {
      VALID = 0,
      INIT_OBS,
      INIT_SWM,
      INIT_VEL,
      TIME_UP,
      MEMS_UP,
      SET_EMPTY,
    };
    string search_state_str_[7] = {"VALID", "Init obs", "Init swarm", "Init velocity", "Times up", "Mems up", "OpenSet Empty"};

    KinoSearch(){};
    ~KinoSearch();

    //setParam, setEnv, init
    void init(int drone_id, TmtList* swarm_tmt_ptr);
    
    void resetSearch();
               
    void setParam(ros::NodeHandle& nh);

    void setEnvironment(std::shared_ptr<rog_map::ROGMap>& env);

    void setVSDFs(VSDFList *vsdf_list);

    void setMaxRate(double vmax, double amax);

    int search(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel, 
               const std::vector<Eigen::Vector3d>& tgt_p,
               double des_v_max, 
               double des_a_max);
    
    int searchOnce(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel);

    void retrievePath(std::vector<Eigen::Vector3d>& path,
                              std::vector<Eigen::Vector3d>& vel_path,
                              std::vector<Eigen::Vector3d>& acc_path);

    typedef shared_ptr<KinoSearch> Ptr;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}