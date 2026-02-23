#ifndef _VIS_SDF_H
#define _VIS_SDF_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <random>
#include <queue>
#include <ros/ros.h>
#include <tuple>
#include <memory.h>
#include <visualization_msgs/Marker.h>
#include <rog_map/rog_map.h>
#include <cmath>
#include <algorithm>

using namespace std;

class VisSDF
{
public:
  VisSDF() {}
  ~VisSDF() {}

  // occlusion map management
  inline void boundIndex(Eigen::Vector3i &idx);
  inline int toOccAddress(const Eigen::Vector3i &idx);
  inline int toOccAddress(int x, int y, int z);
  inline int toDistAddress(const Eigen::Vector3i &idx);
  inline int toDistAddress(int x, int y, int z);
  inline int toLayerAddress(const Eigen::Vector2i &id);
  inline int toLayerAddress(int x, int y);
  inline void distAddress2idx(const int &addr, Eigen::Vector3i &idx);
  inline double aCos(const double val);
  
  inline void c2s(const Eigen::Vector3d &xyz, Eigen::Vector3d &tpr);
  inline void s2c(const Eigen::Vector3d &tpr, Eigen::Vector3d &xyz);

  //sph: incl, azth, radi / theta, phi, rd in local frame;
  //crt: cartesian xyz in world frame
  inline void crt2idx(const Eigen::Vector3d &crt, Eigen::Vector3i &idx);
  inline void sph2idx(const Eigen::Vector3d &sph, Eigen::Vector3i &idx);
  inline void idx2crt(const Eigen::Vector3i &idx, Eigen::Vector3d &crt);
  inline void idx2sph(const Eigen::Vector3i &idx, Eigen::Vector3d &sph);

  inline bool isInSDF(const Eigen::Vector3d &sph);
  inline bool isInSDF(const Eigen::Vector3i &idx); 
  inline bool isInSDF(const Eigen::Vector3d &sph, int margin);
  inline bool isInSDF(const Eigen::Vector3i &idx, int margin); 
  inline bool isInLayer(Eigen::Vector2i &id, int &layer_id);
  inline bool isInLayer(Eigen::Vector3i &idx);

  inline void setOccluded(Eigen::Vector3d sph);
  inline void setVisible(Eigen::Vector3d sph);
  inline void setOccluded(Eigen::Vector3i idx);
  inline void setVisible(Eigen::Vector3i idx);

  void initSDF(ros::NodeHandle &nh, std::shared_ptr<rog_map::ROGMap> &mapPtr);
  void resetSDF(const Eigen::Vector3d &center_pos);
  void generateSDF(const Eigen::Vector3d &center_pos);

  //sdf
  template <typename F_get_val, typename F_set_val>
  void L1Scan(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int vnum);
  
  template <typename F_get_itrsec, typename F_set_val>
  void L2Scan(F_get_itrsec f_get_itrsec, F_set_val f_set_val, int start, int end, int vnum, bool can_print);

  double calcItrsecTheta(const double &theta1, const double &phi1, 
                         const double &theta2, const double &phi2, const double &phi0);
  double calcItrsecTheta(const int &theta1_idx, const int &phi1_idx, 
                         const int &theta2_idx, const int &phi2_idx, const int &phi0_idx);
  double calcSphDist(const double &theta1, const double &phi1,
                     const double &theta2, const double &phi2);
  double calcSphDist(const int &theta1_idx, const int &phi1_idx,
                     const int &theta2_idx, const int &phi2_idx);

  void updateVSDFbase();

  void increPropVSDF();

  void updateOccQueue(int &layer_id, bool with_expansion);

  void publishSDF();
  void publishSDFwithColorTF();
  void publishSpherewithColorTF();


  inline int getCrtOcc(const Eigen::Vector3d &crt);
  inline int getOcc(const Eigen::Vector3d &sph);
  inline int getOcc(const Eigen::Vector3i &idx);
  inline double getCrtDist(const Eigen::Vector3d &crt);
  inline double getDist(const Eigen::Vector3d &sph);
  inline double getDist(const Eigen::Vector3i &idx);

  void evaluateSDTwithGrad(const Eigen::Vector3d& crt, double& dist, Eigen::Vector3d& grad);
  void evaluateSDTwithGradTrilinear(const Eigen::Vector3d& crt, double& dist, Eigen::Vector3d& grad);
  void evaluateSDTwithGradTrilinearOld(const Eigen::Vector3d& crt, double& dist, Eigen::Vector3d& grad);
  void interpolateSDTwithGrad(double values[2][2], const Eigen::Vector3d& diff, 
                              double& value, Eigen::Vector3d& grad);
  void interpolateSDTwithGradTrilinear(double values[2][2][2], const Eigen::Vector3d& diff, 
                                       double& value, Eigen::Vector3d& grad);
  void evaluateSDTTrilinear(const Eigen::Vector3d& crt, double& dist);
  void interpolateSDTTrilinear(double values[2][2][2], const Eigen::Vector3d& diff, 
                                       double& value);

  void boxHit();
  
  typedef std::shared_ptr<VisSDF> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  /* map ptr */
  // OccMap::Ptr map_ptr_;
  std::shared_ptr<rog_map::ROGMap> map_ptr_;

  /* sdf params */
  Eigen::Vector3d ctr_pos_;
  Eigen::Vector3d sdf_origin_, sdf_size_;
  Eigen::Vector3i sdf_voxel_num_;          
  double theta_lb_, theta_ub_;
  double rd_lb_, rd_ub_;     
  double height_lb_, height_ub_;     

  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  bool floor_as_occ_, ceiling_as_occ_;
  int inflation_step_;

  /* vsdf */
  std::vector<uint8_t> occupancy_buffer_;
  std::vector<int> depth_buffer_;
  std::vector<int> tmp_phi_buffer_;
  std::vector<double> distance_buffer_;
  std::vector<double> tmp_distance_buffer_;

  std::vector<Eigen::Vector2i> closest_obstacle_buffer_;
  std::vector<std::vector<Eigen::Vector2i>> incre_list_;
  std::vector<std::vector<Eigen::Vector2i>> expand_list_;
  std::vector<int> theta_idx_lb_;
  std::vector<int> theta_idx_ub_;

  std::queue<Eigen::Vector2i> update_queue_;

  /* connectivity */
  const int undefined_{-std::numeric_limits<int>::max()};

  std::vector<Eigen::Vector2i> inflation_dirs_;
  const std::vector<Eigen::Vector2i> dirs0_ = {Eigen::Vector2i(-1, 1), Eigen::Vector2i(0, 1),
                                               Eigen::Vector2i(1, 1), Eigen::Vector2i(1, 0),
                                               Eigen::Vector2i(1, -1), Eigen::Vector2i(0, -1),
                                               Eigen::Vector2i(-1, -1), Eigen::Vector2i(-1, 0)};
  const std::vector<Eigen::Vector2i> dirs1_ = {Eigen::Vector2i(-1, 1), Eigen::Vector2i(0, 1),
                                               Eigen::Vector2i(1, 1), Eigen::Vector2i(1, 0),
                                               Eigen::Vector2i(-1, 0)};
  const std::vector<Eigen::Vector2i> dirs2_ = {Eigen::Vector2i(1, 0),
                                               Eigen::Vector2i(1, -1), Eigen::Vector2i(0, -1),
                                               Eigen::Vector2i(-1, -1), Eigen::Vector2i(-1, 0)};
  const std::vector<Eigen::Vector2i> dirs3_ = {Eigen::Vector2i(-1, 1), Eigen::Vector2i(0, 1),
                                               Eigen::Vector2i(1, 1), Eigen::Vector2i(1, 0),
                                               Eigen::Vector2i(1, -1)};
  const std::vector<Eigen::Vector2i> dirs4_ = {Eigen::Vector2i(-1, 1), Eigen::Vector2i(0, 1),
                                               Eigen::Vector2i(1, 1)};
  const std::vector<Eigen::Vector2i> dirs5_ = {Eigen::Vector2i(1, 0),
                                               Eigen::Vector2i(1, -1), Eigen::Vector2i(0, -1)};
  const std::vector<Eigen::Vector2i> dirs6_ = {Eigen::Vector2i(-1, 1), Eigen::Vector2i(0, 1),
                                               Eigen::Vector2i(0, -1),
                                               Eigen::Vector2i(-1, -1), Eigen::Vector2i(-1, 0)};
  const std::vector<Eigen::Vector2i> dirs7_ = {Eigen::Vector2i(-1, 1), Eigen::Vector2i(0, 1),
                                               Eigen::Vector2i(-1, 0)};
  const std::vector<Eigen::Vector2i> dirs8_ = {Eigen::Vector2i(0, -1), Eigen::Vector2i(-1, -1), 
                                               Eigen::Vector2i(-1, 0)};
 
  bool esdf_need_update_;
  double esdf_time_, max_esdf_time_;

  ros::Publisher vsdf_pub_, vsdf_colored_pub_;
  ros::Publisher sph_colored_pub_;

  std::vector<Eigen::Vector3d> mark_crt_set_;
  std::vector<Eigen::Vector3i> mark_sph_idx_set_;

};

inline double VisSDF::aCos(const double val)
{
  if(val <= -1.0){
    return M_PI;
  }else if(val >= 1.0){
    return 0;
  }else{
    return acos(val);
  }
}

inline int VisSDF::toOccAddress(const Eigen::Vector3i &idx)
{
  return idx(0) * sdf_voxel_num_(1) * sdf_voxel_num_(2) + idx(1) * sdf_voxel_num_(2) + idx(2);
}

inline int VisSDF::toOccAddress(int x, int y, int z)
{
  return x * sdf_voxel_num_(1) * sdf_voxel_num_(2) + y * sdf_voxel_num_(2) + z;
}

inline int VisSDF::toDistAddress(const Eigen::Vector3i &idx)
{
  return idx(2) * sdf_voxel_num_(1) * sdf_voxel_num_(0) + idx(1) * sdf_voxel_num_(0) + idx(0);
}

inline int VisSDF::toDistAddress(int x, int y, int z)
{
  return z * sdf_voxel_num_(1) * sdf_voxel_num_(0) + y * sdf_voxel_num_(0) + x;
}

inline int VisSDF::toLayerAddress(const Eigen::Vector2i &id)
{
  return id(1) * sdf_voxel_num_(0) + id(0);
}

inline int VisSDF::toLayerAddress(int x, int y)
{
  return y * sdf_voxel_num_(0) + x;
}

inline void VisSDF::distAddress2idx(const int &addr, Eigen::Vector3i &idx)
{
  idx <<  addr % sdf_voxel_num_(0),
          addr % (sdf_voxel_num_(1) * sdf_voxel_num_(0)) / sdf_voxel_num_(0), 
          addr / (sdf_voxel_num_(1) * sdf_voxel_num_(0));
}

inline void VisSDF::sph2idx(const Eigen::Vector3d &sph, Eigen::Vector3i &idx)
{
  idx = ((sph - sdf_origin_) * resolution_inv_).array().floor().cast<int>();
}

inline void VisSDF::idx2sph(const Eigen::Vector3i &idx, Eigen::Vector3d &sph)
{
  sph = (idx.cast<double>() + Eigen::Vector3d(0.5, 0.5, 0.5)) * resolution_ + sdf_origin_;
}

inline void VisSDF::crt2idx(const Eigen::Vector3d &crt, Eigen::Vector3i &idx)
{
  Eigen::Vector3d sph;
  c2s(crt - ctr_pos_, sph);
  sph2idx(sph, idx);
}

inline void VisSDF::idx2crt(const Eigen::Vector3i &idx, Eigen::Vector3d &crt)
{
  Eigen::Vector3d sph;
  idx2sph(idx, sph);
  s2c(sph, crt);
  crt += ctr_pos_;
}

// note that the radius should be larger than 0
inline void VisSDF::c2s(const Eigen::Vector3d &xyz, Eigen::Vector3d &tpr)
{
  tpr(2) = xyz.norm();
  tpr(0) = aCos(xyz(2) / tpr(2));
  double at = atan2(xyz(1), xyz(0));
  tpr(1) = at < 0 ? at + 2 * M_PI : at;
}


inline void VisSDF::s2c(const Eigen::Vector3d &tpr, Eigen::Vector3d &xyz)
{
  xyz << tpr(2) * sin(tpr(0)) * cos(tpr(1)), 
         tpr(2) * sin(tpr(0)) * sin(tpr(1)), 
         tpr(2) * cos(tpr(0));
}


inline void VisSDF::boundIndex(Eigen::Vector3i &id) 
{
  Eigen::Vector3i id_bound;
  id_bound(2) = max(min(id(2), sdf_voxel_num_(2) - 1), 0);
  id_bound(1) = max(min(id(1), sdf_voxel_num_(1) - 1), 0);
  id_bound(0) = max(min(id(0), theta_idx_ub_[id_bound(2)]), theta_idx_lb_[id_bound(2)]);
  id = id_bound;
}


inline bool VisSDF::isInSDF(const Eigen::Vector3d &sph)
{
  Eigen::Vector3i idx;
  sph2idx(sph, idx);
  return isInSDF(idx);
}

inline bool VisSDF::isInSDF(const Eigen::Vector3i &idx)
{
  return (idx(2) < sdf_voxel_num_(2)&& idx(2) >= 0&&
          idx(0) <= theta_idx_ub_[idx(2)]&&
          idx(0) >= theta_idx_lb_[idx(2)]);
}

inline bool VisSDF::isInSDF(const Eigen::Vector3d &sph, int margin)
{
  Eigen::Vector3i idx;
  sph2idx(sph, idx);
  return isInSDF(idx, margin);
}

inline bool VisSDF::isInSDF(const Eigen::Vector3i &idx, int margin)
{
  return (idx(2) < (sdf_voxel_num_(2) - margin) && idx(2) >= margin &&
          idx(0) <= (theta_idx_ub_[idx(2)] - margin) &&
          idx(0) >= (theta_idx_lb_[idx(2)] + margin) );
}

inline bool VisSDF::isInLayer(Eigen::Vector2i &id, int &layer_id)
{
  if(id(0) > theta_idx_ub_[layer_id] || id(0) < theta_idx_lb_[layer_id]) 
    return false;
  if(id(1) >= sdf_voxel_num_(1)) id(1) = 0;
  if(id(1) < 0) id(1) = sdf_voxel_num_(1) - 1;
  return true;
}

inline bool VisSDF::isInLayer(Eigen::Vector3i &idx)
{
  if(idx(0) > theta_idx_ub_[idx(2)] || idx(0) < theta_idx_lb_[idx(2)]) 
    return false;
  if(idx(1) >= sdf_voxel_num_(1)) idx(1) = 0;
  if(idx(1) < 0) idx(1) = sdf_voxel_num_(1) - 1;
  return true;
}


inline void VisSDF::setOccluded(Eigen::Vector3d sph)
{
  if (!isInSDF(sph))
    return;
  Eigen::Vector3i idx;
  sph2idx(sph, idx);
  occupancy_buffer_[toOccAddress(idx)] = 1;
}


inline void VisSDF::setOccluded(Eigen::Vector3i idx)
{
  occupancy_buffer_[toOccAddress(idx)] = 1;
}


inline void VisSDF::setVisible(Eigen::Vector3d sph)
{
  if (!isInSDF(sph))
    return;
  Eigen::Vector3i idx;
  sph2idx(sph, idx);
  occupancy_buffer_[toOccAddress(idx)] = 0;
}


inline void VisSDF::setVisible(Eigen::Vector3i idx)
{
  occupancy_buffer_[toOccAddress(idx)] = 0;
}

inline int VisSDF::getCrtOcc(const Eigen::Vector3d &crt)
{
  Eigen::Vector3i idx;
  crt2idx(crt, idx);

  if(!isInSDF(idx)) return 1;
  return int(occupancy_buffer_[toOccAddress(idx)]);
}


inline int VisSDF::getOcc(const Eigen::Vector3d &sph)
{
  Eigen::Vector3i idx;
  sph2idx(sph, idx);
 
  if(!isInSDF(idx)) return 1;
  return int(occupancy_buffer_[toOccAddress(idx)]);
}


inline int VisSDF::getOcc(const Eigen::Vector3i &idx)
{
  Eigen::Vector3i tmp = idx;
 
  if(!isInLayer(tmp)) return 1;
  return int(occupancy_buffer_[toOccAddress(tmp)]);
}

inline double VisSDF::getCrtDist(const Eigen::Vector3d& crt) {
  Eigen::Vector3i idx;
  crt2idx(crt, idx);

  if(!isInSDF(idx)) return -M_PI;
  return distance_buffer_[toDistAddress(idx)];
}

inline double VisSDF::getDist(const Eigen::Vector3d& sph) {
  Eigen::Vector3i idx;
  sph2idx(sph, idx);

  if(!isInSDF(idx)) return -M_PI;
  return distance_buffer_[toDistAddress(idx)];
}

inline double VisSDF::getDist(const Eigen::Vector3i& idx) {
  Eigen::Vector3i tmp = idx;

  if(!isInLayer(tmp)) return -M_PI;
  return distance_buffer_[toDistAddress(tmp)];
}

inline double VisSDF::calcItrsecTheta(const double &theta1, const double &phi1, 
                                      const double &theta2, const double &phi2, const double &phi0){
  double ct1 = cos(theta1); double ct2 = cos(theta2);
  double a = sin(theta1) * cos(phi1 - phi0);
  double b = sin(theta2) * cos(phi2 - phi0);

  if(a == b){
    return M_PI_2;
  }else{
    double c = (ct2 - ct1) / (a - b);
    if(c >= 0){
      return atan(c);
    }else{
      return atan(c) + M_PI;
    }
  }
}

inline double VisSDF::calcItrsecTheta(const int &theta1_idx, const int &phi1_idx, 
                                      const int &theta2_idx, const int &phi2_idx, const int &phi0_idx){
  double theta1 = theta1_idx * resolution_ + theta_lb_;
  double theta2 = theta2_idx * resolution_ + theta_lb_;
  double ct1 = cos(theta1); double ct2 = cos(theta2);
  double a = sin(theta1) * cos(resolution_ * (phi1_idx - phi0_idx));
  double b = sin(theta2) * cos(resolution_ * (phi2_idx - phi0_idx));
  if(a == b){
    return M_PI_2;
  }else{
    double c = (ct2 - ct1) / (a - b);
    if(c >= 0){
      return atan(c);
    }else{
      return atan(c) + M_PI;
    }
  }
}


inline double VisSDF::calcSphDist(const double &theta1, const double &phi1,
                                  const double &theta2, const double &phi2)
{
  double cosC = cos(theta1) * cos(theta2) + sin(theta1) * sin(theta2) * cos(phi1 - phi2);
  return aCos(cosC);
}

inline double VisSDF::calcSphDist(const int &theta1_idx, const int &phi1_idx,
                                  const int &theta2_idx, const int &phi2_idx)
{
  double theta1 = theta1_idx * resolution_ + theta_lb_;
  double theta2 = theta2_idx * resolution_ + theta_lb_;
  double cosC = cos(theta1) * cos(theta2) + sin(theta1) * sin(theta2) * cos((phi1_idx - phi2_idx) * resolution_);
  return aCos(cosC);
}



#endif
