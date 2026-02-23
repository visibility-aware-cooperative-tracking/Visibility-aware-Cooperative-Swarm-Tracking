#pragma once

#include <ros/ros.h>
#include "rog_map/rog_map.h"
#include <Eigen/Core>
#include <queue>

namespace prediction {

struct Node {
  Eigen::Vector3d p, v, a;
  double t;
  double score;
  double h;
  Node* parent = nullptr;
};
typedef Node* NodePtr;
class NodeComparator {
 public:
  bool operator()(NodePtr& lhs, NodePtr& rhs) {
    return lhs->score + lhs->h > rhs->score + rhs->h;
  }
};
class Predict {
 private:
  static constexpr int MAX_MEMORY = 20000; //1 << 22 org

  std::shared_ptr<rog_map::ROGMap> rogmap_;

  // searching

  double dt;
  double pre_dur;
  double rho_a;
  double car_z, vmax;
  
  NodePtr data[MAX_MEMORY];
  int stack_top;

  inline bool isValid(const Eigen::Vector3d& p, const Eigen::Vector3d& v) const {
    return (v.norm() < vmax) && (!rogmap_->isOccupied(p));
  }

 public:
  Predict(ros::NodeHandle& nh, std::shared_ptr<rog_map::ROGMap>& rogmap) : rogmap_(rogmap) 
  {
    nh.getParam("planning/predict_T", pre_dur);
    nh.getParam("planning/predict_dt", dt);
    nh.getParam("prediction/rho_a", rho_a);
    nh.getParam("prediction/vmax", vmax);
    for (int i = 0; i < MAX_MEMORY; ++i) {
      data[i] = new Node;
    }
  }

  //Predict 
  inline bool predict(const Eigen::Vector3d& target_p,
                      const Eigen::Vector3d& target_v,
                      std::vector<Eigen::Vector3d>& target_predict_pos,
                      std::vector<Eigen::Vector3d>& target_predict_vel,
                      const double& max_time = 0.05) {
    auto score = [&](const NodePtr& ptr) -> double {
      return rho_a * ptr->a.norm();
    };
    Eigen::Vector3d end_p = target_p + target_v * pre_dur;
    auto calH = [&](const NodePtr& ptr) -> double {
      return 0.001 * (ptr->p - end_p).norm();
    };
    ros::Time t_start = ros::Time::now();
    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set;

    Eigen::Vector3d input(0, 0, 0);

    stack_top = 0;
    NodePtr curPtr = data[stack_top++];
    curPtr->p = target_p;
    curPtr->v = target_v;
    curPtr->a.setZero();
    curPtr->parent = nullptr;
    curPtr->score = 0;
    curPtr->h = 0;
    curPtr->t = 0;
    double dt2_2 = dt * dt / 2;

    while (curPtr->t < pre_dur - 1e-3) {//exceed prediction time
      for (input.x() = -3; input.x() <= 3 + 1e-3; input.x() += 1.5)
        for (input.y() = -3; input.y() <= 3 + 1e-3; input.y() += 1.5) 
          for (input.z() = -2; input.z() <= 2 + 1e-3; input.z() += 2) 
          {//Search with acc 2.0.
            Eigen::Vector3d p = curPtr->p + curPtr->v * dt + input * dt2_2;
            Eigen::Vector3d v = curPtr->v + input * dt;
            if (!isValid(p, v)) {
              
              continue;
            }
            if (stack_top == MAX_MEMORY) {
              std::cout << "\033[34;1m[prediction] out of memory!\033[0m" << std::endl;
              return false;
            }
            double t_cost = (ros::Time::now() - t_start).toSec();
            if (t_cost > max_time) {
              std::cout << "\033[34;1m[prediction] too slow!\033[0m" << std::endl;
              return false;
            }
            NodePtr ptr = data[stack_top++];
            ptr->p = p;
            ptr->v = v;
            ptr->a = input;
            ptr->parent = curPtr;
            ptr->t = curPtr->t + dt;
            ptr->score = curPtr->score + score(ptr);
            ptr->h = calH(ptr);
            open_set.push(ptr);
          }

      if (open_set.empty()) {
        std::cout << "\033[34;1m[prediction] no way!\033[0m" << std::endl;
        return false;
      }
      curPtr = open_set.top();
      open_set.pop();
    }
    target_predict_pos.clear();
    target_predict_vel.clear();
    while (curPtr != nullptr) {
      target_predict_pos.push_back(curPtr->p);
      target_predict_vel.push_back(curPtr->v);
      curPtr = curPtr->parent;
    }
    std::reverse(target_predict_pos.begin(), target_predict_pos.end());
    std::reverse(target_predict_vel.begin(), target_predict_vel.end());

    return true;
  }

  typedef std::shared_ptr<Predict> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace prediction
