#include "plan_manage/SFCgen_ciri.hpp"


namespace SFCgen_ciri{
SFCgen_ciri::SFCgen_ciri(ros::NodeHandle& nh,
  std::shared_ptr<rog_map::ROGMap>& mapPtr) : mapPtr_(mapPtr) {
  hPolyPub_ = nh.advertise<visualization_msgs::MarkerArray>("visualization/corridors", 100);
  ciri_.reset(new ciri::CIRI);
}

void SFCgen_ciri::setParam(double robot_r, int iter_num)
{
  ciri_->setupParams(robot_r, iter_num);
  resolution_ = mapPtr_->getResolution();
  ceil_height_ = mapPtr_->getCeilHeight();
  ground_height_ = mapPtr_->getGroundHeight();
}

void SFCgen_ciri::getSeedBBox(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const double& bbox_width,
                              Eigen::Vector3d &box_min, Eigen::Vector3d &box_max) {
  box_min = p1.cwiseMin(p2);
  box_max = p1.cwiseMax(p2);
  box_min -= Eigen::Vector3d(bbox_width, bbox_width, bbox_width);
  box_max += Eigen::Vector3d(bbox_width, bbox_width, bbox_width);
  box_min.z() = std::max(box_min.z(), ground_height_);
  box_max.z() = std::min(box_max.z(), ceil_height_);
}

void SFCgen_ciri::convertPpolyToHpoly(Polytope& pPoly, MatrixXd& hPoly) {
  //convert ax+by+cz+d<0 to (outernormal^T point^T)^T
  int surfNum = pPoly.SurfNum();
  auto hyperPlanes = pPoly.GetPlanes();
  hPoly.resize(6, surfNum);
  for (int i = 0; i < surfNum; ++i) {
    Eigen::Vector3d _n, _p;
    double a, b, c, d;
    a = hyperPlanes(i, 0);
    b = hyperPlanes(i, 1);
    c = hyperPlanes(i, 2);
    d = hyperPlanes(i, 3);

    _n << a, b, c;
    if (fabs(c) > 1e-3) {
        _p << 0.0, 0.0, -d / c;
    } else if (fabs(b) > 1e-3) {
        _p << 0.0, -d / b, 0.0;
    } else if (fabs(a) > 1e-3) {
        _p << -d / a, 0.0, 0.0;
    }
    hPoly.col(i) << _n, _p;
  }        
}

void SFCgen_ciri::convertHpolyToPpoly(MatrixXd& hPoly, Polytope& pPoly) {
  //convert (outernormal^T point^T)^T to ax+by+cz+d<0
  MatD4f planes;
  int surfNum = static_cast<int>(hPoly.cols());
  planes.resize(surfNum, 4);
  for (int i = 0; i < surfNum; ++i) {
    planes.row(i) << hPoly.col(i).head(3).transpose(), -hPoly.col(i).head(3).dot(hPoly.col(i).tail(3));
  }
  pPoly.SetPlanes(planes);
}

bool SFCgen_ciri::isInBox(const Eigen::Vector3d& pt,   
                          const Eigen::Vector3d& box_min,
                          const Eigen::Vector3d& box_max)
{
  if(pt(0) < box_min(0) || pt(0) > box_max(0)) return false;
  if(pt(1) < box_min(1) || pt(1) > box_max(1)) return false;
  if(pt(2) < box_min(2) || pt(2) > box_max(2)) return false;
  return true;
}

bool SFCgen_ciri::generateOneCorridorWithLine(const Eigen::Vector3d& line_head,
                                                const Eigen::Vector3d& line_tail,
                                                const double& bbox_width,
                                                Polytope& pPoly,
                                                Eigen::MatrixXd& hPoly)
{
  Eigen::Vector3d box_max, box_min;
  vec_E<Vec3f> pc, pts{line_head, line_tail};
  getSeedBBox(line_head, line_tail, bbox_width, box_min, box_max);
  
  mapPtr_->boundBoxByLocalMap(box_min, box_max);
  mapPtr_->boxSearch(box_min, box_max, OCCUPIED, pc);
  // box_min.z()+=robot_r_;
  // box_max.z()-=robot_r_;

  MatD4f planes;

  Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
  bd(0, 0) = 1.0;
  bd(1, 0) = -1.0;
  bd(2, 1) = 1.0;
  bd(3, 1) = -1.0;
  bd(4, 2) = 1.0;
  bd(5, 2) = -1.0;
  bd(0, 3) = -box_max.x();
  bd(1, 3) = box_min.x();
  bd(2, 3) = -box_max.y();
  bd(3, 3) = box_min.y();
  bd(4, 3) = -box_max.z();
  bd(5, 3) = box_min.z();
  
  if (pc.empty()) {
    planes.resize(6, 4);
    planes.row(0) << 1, 0, 0, -box_max.x();
    planes.row(1) << 0, 1, 0, -box_max.y();
    planes.row(2) << 0, 0, 1, -box_max.z();
    planes.row(3) << -1, 0, 0, box_min.x();
    planes.row(4) << 0, -1, 0, box_min.y();
    planes.row(5) << 0, 0, -1, box_min.z();
    pPoly.SetPlanes(planes);

    convertPpolyToHpoly(pPoly, hPoly);

    return true;
  }
  Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pp(pc[0].data(), 3, pc.size());
  RET_CODE success = ciri_->comvexDecomposition(bd, pp, line_head, line_tail);
  if (success == type_utils::SUCCESS) {
    ciri_->getPolytope(pPoly);
    convertPpolyToHpoly(pPoly, hPoly);

    return true;
  } else {

    return false;
  }
}

void SFCgen_ciri::visualizeCorridors(const std::vector<Polytope> &sfcs) {
  if (hPolyPub_.getNumSubscribers() <= 0) {
    return;
  }

  visualization_msgs::Marker del;
  visualization_msgs::MarkerArray arr;
  del.action = visualization_msgs::Marker::DELETEALL;
  arr.markers.push_back(del);
  hPolyPub_.publish(arr);

  int color_id = 0;
  for (auto p: sfcs) {
    p.Visualize(hPolyPub_, "sfc", false, Color::SteelBlue(), Color::Black(), Color::Orange(), 0.15,
                0.1 / 2);
  }
}

void SFCgen_ciri::compressPoly(Eigen::MatrixXd& poly, double dx) {
  for (int i = 0; i < poly.cols(); ++i) {
    poly.col(i).tail(3) = poly.col(i).tail(3) - poly.col(i).head(3) * dx;
  }
}

bool SFCgen_ciri::filterCorridor(std::vector<Eigen::MatrixXd>& hPolys) {
  bool ret = false;
  if (hPolys.size() <= 2) {
    return ret;
  }
  std::vector<Eigen::MatrixXd> ret_polys;
  Eigen::MatrixXd hPoly0 = hPolys[0];
  Eigen::MatrixXd curIH;
  Eigen::Vector3d interior;
  for (int i = 2; i < (int)hPolys.size(); i++) {
    curIH.resize(6, hPoly0.cols() + hPolys[i].cols());
    curIH << hPoly0, hPolys[i];
    if (manage_geoutils::findInteriorDist(curIH, interior) < 2.3) {
      ret_polys.push_back(hPoly0);
      hPoly0 = hPolys[i - 1];
    } else {
      ret = true;
    }
  }
  ret_polys.push_back(hPoly0);
  ret_polys.push_back(hPolys.back());
  hPolys = ret_polys;
  return ret;
}


bool SFCgen_ciri::generateCorridorAlongPath(const std::vector<Eigen::Vector3d>& path,
                                            const double& bbox_width,
                                            std::vector<Polytope>& pPolys,
                                            std::vector<Eigen::MatrixXd>& hPolys){
  if(path.size() <= 1)
  {
    ROS_WARN_STREAM("[SFC] Path Not Valid!");
    return false;
  }
  int path_len = path.size();
  int idx = 0;
  Eigen::MatrixXd hPoly_candi;
  Polytope pPoly_candi;
  pPolys.clear();
  hPolys.clear();
  while (idx < path_len - 1) {

    int next_idx = idx;
    
    while (next_idx + 1 < path_len && mapPtr_->isLineFree(path[idx], path[next_idx + 1], 1 * bbox_width)) {
      next_idx++;
    }

    if(!generateOneCorridorWithLine(path[idx], path[next_idx], bbox_width, pPoly_candi, hPoly_candi))
    {
      ROS_WARN_STREAM("[SFC] Corridor Failed!");
      return false;
    }
    hPolys.emplace_back(hPoly_candi);
    idx = next_idx;
    while (idx + 1 < path_len && pPoly_candi.PointIsInside(path[idx + 1])) {
      idx++;
    }
  }

  for(auto& hPoly : hPolys)
  {
    convertHpolyToPpoly(hPoly, pPoly_candi);
    pPolys.emplace_back(pPoly_candi);
  }
  return true;
}


}