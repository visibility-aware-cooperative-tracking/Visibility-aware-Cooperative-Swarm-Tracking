#pragma once

#include <ros/ros.h>
#include <Eigen/Core>
#include <unordered_map>
#include <memory>
#include <queue>
#include <rog_map/rog_map.h>

namespace std {
template <typename Scalar, int Rows, int Cols>
struct hash<Eigen::Matrix<Scalar, Rows, Cols>> {
  size_t operator()(const Eigen::Matrix<Scalar, Rows, Cols>& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < (size_t)matrix.size(); ++i) {
      Scalar elem = *(matrix.data() + i);
      seed ^=
          std::hash<Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};
}  // namespace std

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

namespace astar_search{ 

enum State { OPEN,
             CLOSE,
             UNVISITED };

struct Node {
  Eigen::Vector3i idx;
  bool valid = false;
  State state = UNVISITED;
  double g, h;
  Node* parent = nullptr;
};

typedef Node* NodePtr;
class NodeComparator {
 public:
  bool operator()(NodePtr& lhs, NodePtr& rhs) {
    return lhs->g + lhs->h > rhs->g + rhs->h;
  }
};

class astarSearch{
    static constexpr int MAX_MEMORY = 510000;
    static constexpr int MAX_LOCAL_SEARCH_SIZE = 500000;
    static constexpr double MAX_DURATION = 0.010;
    
private:
    ros::Time t_start_;

    std::unordered_map<Eigen::Vector3i, NodePtr> visited_nodes_;
    std::shared_ptr<rog_map::ROGMap> mapPtr_;
    NodePtr data_[MAX_MEMORY];
    double desired_dist_, theta_clearance_, tolerance_d_;

    double resolution_, min_flight_height_;
    int ceil_idx_, ground_idx_, min_flight_height_idx_;

    inline NodePtr visit(const Eigen::Vector3i& idx) {
        auto iter = visited_nodes_.find(idx);
        if (iter == visited_nodes_.end()) {
        auto ptr = data_[visited_nodes_.size()];
        ptr->idx = idx;
        ptr->valid = ((idx(2) < ceil_idx_) && (idx(2) > ground_idx_) 
                     && (idx(2) > min_flight_height_idx_) && (!mapPtr_->isOccupiedInflate(idx)));
        ptr->state = UNVISITED;
        visited_nodes_[idx] = ptr;
        return ptr;
        } else {
        return iter->second;
        }
    }

public:
    astarSearch(ros::NodeHandle& nh,
                std::shared_ptr<rog_map::ROGMap>& mapPtr) : mapPtr_(mapPtr)
    {
        nh.param("planning/track_dist", desired_dist_, 1.9);
        nh.param("planning/tolerance_d", tolerance_d_, 0.1);
        nh.param("optimization/theta_clearance", theta_clearance_, 0.4);
        nh.param("planning/min_flight_height", min_flight_height_, 0.3);

        resolution_ = mapPtr_->getResolution();
        double ceil_height = mapPtr_->getCeilHeight();
        double ground_height = mapPtr_->getGroundHeight();
        ceil_idx_ = (int)(ceil_height / resolution_ + sign(ceil_height) * 0.5) - 1;
        ground_idx_ = (int)(ground_height / resolution_ + sign(ground_height) * 0.5) + 1;
        min_flight_height_idx_ = (int)(min_flight_height_ / resolution_ + sign(min_flight_height_) * 0.5) - 2;
        for (int i = 0; i < MAX_MEMORY; ++i) {
            data_[i] = new Node;
        }
    }

    ~astarSearch(){
        for (int i = 0; i < MAX_MEMORY; ++i) 
            delete data_[i];
    }


    //Astar with Z-axis distance search
    inline bool astar_dist(const Eigen::Vector3d& start_p,
                           const Eigen::Vector3d& end_p,
                           std::vector<Eigen::Vector3d>& path) {
        Eigen::Vector3i start_idx;
        mapPtr_->posToGlobalIndex(start_p, start_idx);
        Eigen::Vector3i end_idx;
        mapPtr_->posToGlobalIndex(end_p, end_idx);
        path.clear();
        if (start_idx == end_idx) {
            path.push_back(start_p);
            path.push_back(end_p);
            return true;
        }
        auto stopCondition = [&](const NodePtr& ptr) -> bool {
            return ptr->h < tolerance_d_ / resolution_; 
        };
        auto calulateHeuristic = [&](const NodePtr& ptr) {    //debug
            Eigen::Vector3i dp = end_idx - ptr->idx;
            double dr = dp.head(3).norm();
            double lambda = 1 - desired_dist_ / resolution_ / dr; 
            double dx = lambda * dp.x();        
            double dy = lambda * dp.y();
            double dz = lambda * dp.z();
            ptr->h = fabs(dx) + fabs(dy) + abs(dz);
            double dx0 = (start_idx - end_idx).x();
            double dy0 = (start_idx - end_idx).y();
            double cross = fabs(dx * dy0 - dy * dx0) + abs(dz);
            ptr->h += 0.001 * cross;
        };
        // initialization of datastructures
        std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set;
        std::vector<std::pair<Eigen::Vector3i, double>> neighbors;
        // NOTE 6-connected graph
        for (int i = 0; i < 3; ++i) {
            Eigen::Vector3i neighbor(0, 0, 0);
            neighbor[i] = 1;
            neighbors.emplace_back(neighbor, 1);
            neighbor[i] = -1;
            neighbors.emplace_back(neighbor, 1);
        }
        bool ret = false;
        NodePtr curPtr = visit(start_idx);
        // NOTE we should not permit the start pos invalid! (for corridor generation)
        if (!curPtr->valid) {
            visited_nodes_.clear();
            std::cout << "[short astar]start position invalid!" << std::endl;
            return false;
        }
        curPtr->parent = nullptr;
        curPtr->g = 0;
        calulateHeuristic(curPtr);
        curPtr->state = CLOSE;

        while (visited_nodes_.size() < MAX_MEMORY) {
            for (const auto& neighbor : neighbors) {
                auto neighbor_idx = curPtr->idx + neighbor.first;
                auto neighbor_dist = neighbor.second;
                NodePtr neighborPtr = visit(neighbor_idx);
                if (neighborPtr->state == CLOSE) {
                    continue;
                }
                if (neighborPtr->state == OPEN) {
                    // check neighbor's g score
                    // determine whether to change its parent to current
                    if (neighborPtr->g > curPtr->g + neighbor_dist) {
                        neighborPtr->parent = curPtr;
                        neighborPtr->g = curPtr->g + neighbor_dist;
                    }
                    continue;
                }
                if (neighborPtr->state == UNVISITED) {
                    if (neighborPtr->valid) {
                        neighborPtr->parent = curPtr;
                        neighborPtr->state = OPEN;
                        neighborPtr->g = curPtr->g + neighbor_dist;
                        calulateHeuristic(neighborPtr);
                        open_set.push(neighborPtr);
                    }
                }
            }  // for each neighbor
            if (open_set.empty()) {
                std::cout << "[short astar] no way!" << std::endl;
                break;
            }
            curPtr = open_set.top();
            open_set.pop();
            curPtr->state = CLOSE;
            if (stopCondition(curPtr)) {

                ret = true;
                break;
            }
            if (visited_nodes_.size() == MAX_LOCAL_SEARCH_SIZE) {
                std::cout << "[short astar] out of memory!" << std::endl;
            }
        }
        if (ret) {
            path.push_back(mapPtr_->globalIndexToPos(curPtr->idx));
            for (NodePtr ptr = curPtr->parent; ptr->parent != nullptr; ptr = ptr->parent) {
                path.push_back(mapPtr_->globalIndexToPos(ptr->idx));
            }
            std::reverse(path.begin(), path.end());
        }
        visited_nodes_.clear();
        return ret;
    }



    //Generate path and waypoints(not including the start_pt & heu: dist keeping)
    bool findTrackingPath(const Eigen::Vector3d &start_pt,
                          const std::vector<Eigen::Vector3d> &target_pt,
                          std::vector<Eigen::Vector3d> &waypoints,
                          std::vector<Eigen::Vector3d> &path)
    {
        if(mapPtr_->isOccupiedInflate(start_pt)){
                ROS_ERROR("[Astar: FindTracking] start point not valid.");
                std::cout << "[Astar: FindTracking] start point not valid." << std::endl;
                return false;
        }
        path.clear(); path.push_back(start_pt);
        Eigen::Vector3d way_pt = start_pt;
        waypoints.clear(); 
        std::vector<Eigen::Vector3d> tmp_path;
        for(size_t i = 0; i < target_pt.size(); i++)
        {
            if(!astar_dist(way_pt, target_pt[i], tmp_path))
                return false;
            way_pt = tmp_path.back();
            waypoints.push_back(way_pt);//push after first search
            path.insert(path.end(),tmp_path.begin(),tmp_path.end());
        }

        return true;
    }

    //Astar with exact end condition
    inline bool short_astar(const Eigen::Vector3d& start_p,
                          const Eigen::Vector3d& end_p,
                          std::vector<Eigen::Vector3d>& path) {
                              
        Eigen::Vector3i start_idx = mapPtr_->posToGlobalIndex(start_p);
        Eigen::Vector3i end_idx = mapPtr_->posToGlobalIndex(end_p);
        path.clear();
        if (start_idx == end_idx) {
            path.push_back(start_p);
            path.push_back(end_p);
            return true;
        }
        auto stopCondition = [&](const NodePtr& ptr) -> bool {
            return ptr->idx == end_idx;
        };
        auto calulateHeuristic = [&](const NodePtr& ptr) {
            Eigen::Vector3i dp = end_idx - ptr->idx;
            int dx = dp.x();
            int dy = dp.y();
            int dz = dp.z();
            ptr->h = abs(dx) + abs(dy) + abs(dz);
            double dx0 = (start_idx - end_idx).x();
            double dy0 = (start_idx - end_idx).y();
            double cross = fabs(dx * dy0 - dy * dx0) + abs(dz);
            ptr->h += 0.001 * cross;
        };
        // initialization of datastructures
        std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set;
        std::vector<std::pair<Eigen::Vector3i, double>> neighbors;
        // NOTE 6-connected graph
        for (int i = 0; i < 3; ++i) {
        Eigen::Vector3i neighbor(0, 0, 0);
        neighbor[i] = 1;
        neighbors.emplace_back(neighbor, 1);
        neighbor[i] = -1;
        neighbors.emplace_back(neighbor, 1);
        }
        bool ret = false;
        NodePtr curPtr = visit(start_idx);
       
        if (!curPtr->valid) {
        visited_nodes_.clear();
        std::cout << "[short astar]start postition invalid!" << std::endl;
        return false;
        }
        curPtr->parent = nullptr;
        curPtr->g = 0;
        calulateHeuristic(curPtr);
        curPtr->state = CLOSE;

        t_start_ = ros::Time::now();
        int visited_node_size = 0;  
        double t_cost = 0.0;
        while (visited_nodes_.size() < MAX_MEMORY && t_cost <= MAX_DURATION) {
        for (const auto& neighbor : neighbors) {
            auto neighbor_idx = curPtr->idx + neighbor.first;
            auto neighbor_dist = neighbor.second;
            NodePtr neighborPtr = visit(neighbor_idx);
            if (neighborPtr->state == CLOSE) {
                continue;
            }
            if (neighborPtr->state == OPEN) {
            // check neighbor's g score
            // determine whether to change its parent to current
            if (neighborPtr->g > curPtr->g + neighbor_dist) {
                neighborPtr->parent = curPtr;
                neighborPtr->g = curPtr->g + neighbor_dist;
            }
            continue;
            }
            if (neighborPtr->state == UNVISITED) {
                if (neighborPtr->valid) {
                    neighborPtr->parent = curPtr;
                    neighborPtr->state = OPEN;
                    neighborPtr->g = curPtr->g + neighbor_dist;
                    calulateHeuristic(neighborPtr);
                    open_set.push(neighborPtr);
                }
            }
        }  // for each neighbor
        if (open_set.empty()) {
            // std::cout << "start postition invalid!" << std::endl;
            std::cout << "[short astar] no way!" << std::endl;
            break;
        }
        curPtr = open_set.top();
        open_set.pop();
        curPtr->state = CLOSE;
        if (stopCondition(curPtr)) {
            ret = true;
            break;
        }
        visited_node_size = visited_nodes_.size();
        if (visited_node_size == MAX_LOCAL_SEARCH_SIZE) {
            std::cout << "[short astar] out of memory!" << std::endl;
        }
        if(visited_node_size % 10 == 0)
        {
            t_cost = (ros::Time::now() - t_start_).toSec();
          
        }
        
        }
        if (ret) {
            path.push_back(mapPtr_->globalIndexToPos(curPtr->idx));
            for (NodePtr ptr = curPtr->parent; ptr->parent != nullptr; ptr = ptr->parent) {
                path.push_back(mapPtr_->globalIndexToPos(ptr->idx));
            }
            std::reverse(path.begin(), path.end());
        }


        visited_nodes_.clear();
        return ret;
    }
    
    inline bool pts2path(const std::vector<Eigen::Vector3d>& wayPts, std::vector<Eigen::Vector3d>& path) {
     
        path.clear();
        path.push_back(wayPts.front());
        int M = wayPts.size();
        std::vector<Eigen::Vector3d> short_path;
        for (int i = 0; i < M - 1; ++i) {
            const Eigen::Vector3d& p0 = path.back();
            const Eigen::Vector3d& p1 = wayPts[i + 1];
            if (mapPtr_->posToGlobalIndex(p0) == mapPtr_->posToGlobalIndex(p1)) {
                continue;
            }

            short_path.clear();
            if(!short_astar(p0, p1, short_path))//All way points generate path
            {
                return false;
            }
            for (const auto& p : short_path) {
                path.push_back(p);
            }
            path.push_back(p1);
        }
        if (path.size() < 2) {
            Eigen::Vector3d p = path.front();
            p.z() += 0.1;
            path.push_back(p);
        }
        return true;
    } 
    

    inline void visible_pair(const Eigen::Vector3d& center,
                           Eigen::Vector3d& seed,
                           Eigen::Vector3d& visible_p,
                           double& theta,
                           bool res_occ_flg) {
        Eigen::Vector3d dp = seed - center;
        double theta0 = atan2(dp.y(), dp.x());
        double d_theta = resolution_ / desired_dist_ / 2;
        double t_l, t_r;
        for (t_l = theta0 - d_theta; t_l > theta0 - M_PI; t_l -= d_theta) {
            Eigen::Vector3d p = center;
            p.x() += desired_dist_ * cos(t_l);
            p.y() += desired_dist_ * sin(t_l);
            if (!mapPtr_->isLineFreeInflate(p, center, DBL_MAX)) {
                t_l += d_theta;
                break;
            }
        }
        for (t_r = theta0 + d_theta; t_r < theta0 + M_PI; t_r += d_theta) {
            Eigen::Vector3d p = center;
            p.x() += desired_dist_ * cos(t_r);
            p.y() += desired_dist_ * sin(t_r);
            if (!mapPtr_->isLineFreeInflate(p, center, DBL_MAX)) {
                t_r -= d_theta;
                break;
            }
        }
        double theta_v = (t_l + t_r) / 2;
        visible_p = center;
        visible_p.x() += desired_dist_ * cos(theta_v);
        visible_p.y() += desired_dist_ * sin(theta_v);
        theta = (t_r - t_l) / 2;
        
        if(!res_occ_flg){
            Eigen::Vector3d seed_cand;
            double theta_c = theta < theta_clearance_ ? theta : theta_clearance_;
            
            if (theta0 - t_l < theta_c) {
                seed_cand << center(0) + desired_dist_ * cos(t_l + theta_c), 
                            center(1) + desired_dist_ * sin(t_l + theta_c),
                            center(2);  
                if(!mapPtr_->isOccupiedInflate(seed_cand))
                {
                    seed = seed_cand;
                }
            } else if (t_r - theta0 < theta_c) {
                seed_cand << center(0) + desired_dist_ * cos(t_r - theta_c), 
                            center(1) + desired_dist_ * sin(t_r - theta_c),
                            center(2);  
                if(!mapPtr_->isOccupiedInflate(seed_cand))
                {
                    seed = seed_cand;
                }
            }
        }
        return;
    }
    
    //Generate fan sector sequences for all the seeds
    inline void generate_visible_regions(const std::vector<Eigen::Vector3d>& targets,
                                         std::vector<Eigen::Vector3d>& seeds,
                                         std::vector<Eigen::Vector3d>& visible_ps,
                                         std::vector<double>& thetas,
                                         bool res_occ_flg) {
        assert(targets.size() == seeds.size());
        visible_ps.clear();
        thetas.clear();
        Eigen::Vector3d visible_p;
        double theta = 0;
        int M = targets.size();
        for (int i = 0; i < M; ++i) {
            visible_pair(targets[i], seeds[i], visible_p, theta, res_occ_flg);
            visible_ps.push_back(visible_p);
            thetas.push_back(theta);
        }
        return;
    }

    typedef std::shared_ptr<astarSearch> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

}; // End of class astarSearch

}