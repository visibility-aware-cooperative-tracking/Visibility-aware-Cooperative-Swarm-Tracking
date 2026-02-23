#ifndef _SFC_GEN_CIRI_H_
#define _SFC_GEN_CIRI_H_

#include <ros/ros.h>
#include <Eigen/Core>
#include <unordered_map>
#include <memory>
#include <queue>
#include "ciri_solver/ciri.h"
#include "rog_map/rog_map.h"
#include "geoutils.hpp"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace SFCgen_ciri{
using namespace geometry_utils;
using namespace type_utils;
class SFCgen_ciri {

 private:
  ros::Publisher hPolyPub_;
  std::shared_ptr<rog_map::ROGMap> mapPtr_;
  double robot_r_;
  int iter_num_;
  double resolution_;
  double ceil_height_;
  double ground_height_;
  ciri::CIRI::Ptr ciri_;

 public:
  SFCgen_ciri(ros::NodeHandle& nh,
              std::shared_ptr<rog_map::ROGMap>& mapPtr);

  ~SFCgen_ciri() {}

  void setParam(double robot_r, int iter_num);

  void compressPoly(Eigen::MatrixXd& poly, double dx);

  void getSeedBBox(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const double& bbox_width,
                   Eigen::Vector3d &box_min, Eigen::Vector3d &box_max);
  
  bool filterCorridor(std::vector<Eigen::MatrixXd>& hPolys);

  bool generateOneCorridorWithLine(const Eigen::Vector3d& line_head,
                                    const Eigen::Vector3d& line_tail,
                                    const double& bbox_width,
                                    Polytope& pPoly,
                                    Eigen::MatrixXd& hPoly);

  bool generateCorridorAlongPath(const std::vector<Eigen::Vector3d>& path,
                                 const double& bbox_width,
                                 std::vector<Polytope>& pPolys,
                                 std::vector<Eigen::MatrixXd>& hPolys);
  
  static void convertPpolyToHpoly(Polytope& pPoly, MatrixXd& hPoly);

  static void convertHpolyToPpoly(MatrixXd& hPoly, Polytope& pPoly);

  void visualizeCorridors(const std::vector<Polytope> &sfcs);

  bool isInBox(const Eigen::Vector3d& pt,   
               const Eigen::Vector3d& box_min,
               const Eigen::Vector3d& box_max);

  typedef std::shared_ptr<SFCgen_ciri> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

}; //end of class SFCgen

}

#endif