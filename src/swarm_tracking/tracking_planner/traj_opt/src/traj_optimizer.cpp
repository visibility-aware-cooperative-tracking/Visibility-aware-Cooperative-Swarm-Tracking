#include <optimizer_corridor/traj_optimizer.h>

namespace traj_opt {

//SECTION Helper Funcs

static double smoothedL1(const double& x,
                         double& grad) {
  static double mu = 0.001;
  if (x < 0.0) {
    return 0.0;
  } else if (x > mu) {
    grad = 1.0;
    return x - 0.5 * mu;
  } else {
    const double xdmu = x / mu;
    const double sqrxdmu = xdmu * xdmu;
    const double mumxd2 = mu - 0.5 * x;
    grad = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
    return mumxd2 * sqrxdmu * xdmu;
  }
}

static double penF3(const double& x, double& grad) {
  double x2 = x * x;
  grad = 3 * x2;
  return x * x2;
}

static double distF(const double& x, double& grad) {
  static double eps = 0.05;
  static double eps2 = eps * eps;
  static double eps3 = eps * eps2;
  if (x < 2 * eps) {
    double x2 = x * x;
    double x3 = x * x2;
    double x4 = x2 * x2;
    grad = 12 / eps2 * x2 - 4 / eps3 * x3;
    return 4 / eps2 * x3 - x4 / eps3;
  } else {
    grad = 16;
    return 16 * (x - eps);
  }
}

static Eigen::Vector3d f_N(const Eigen::Vector3d& x) {
  return x.normalized();
}

static Eigen::MatrixXd f_DN(const Eigen::Vector3d& x) {
  double x_norm_2 = x.squaredNorm();
  return (Eigen::MatrixXd::Identity(3, 3) - x * x.transpose() / x_norm_2) / sqrt(x_norm_2);
}

static Eigen::MatrixXd f_D2N(const Eigen::Vector3d& x, const Eigen::Vector3d& y) {
  double x_norm_2 = x.squaredNorm();
  double x_norm_3 = x_norm_2 * x.norm();
  Eigen::MatrixXd A = (3 * x * x.transpose() / x_norm_2 - Eigen::MatrixXd::Identity(3, 3));
  return (A * y * x.transpose() - x * y.transpose() - x.dot(y) * Eigen::MatrixXd::Identity(3, 3)) / x_norm_3;
}

static Eigen::Matrix3d q2R(const Eigen::Vector4d& q)
{
  Eigen::Quaterniond Quat(q(0), q(1), q(2), q(3));
  return Quat.toRotationMatrix();
}

// !SECTION  Helper Funcs

bool TrajOpt::extractVs(const std::vector<Eigen::MatrixXd>& hPs,
                        std::vector<Eigen::MatrixXd>& vPs) const {
  const int M = hPs.size() - 1;

  vPs.clear();
  vPs.reserve(2 * M + 1);

  int nv;
  Eigen::MatrixXd curIH, curIV, curIOB;
  for (int i = 0; i < M; i++) {
    if (!geoutils::enumerateVs(hPs[i], curIV)) {
      return false;
    }
    nv = curIV.cols();
    curIOB.resize(3, nv);
    curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
    vPs.push_back(curIOB);

    curIH.resize(6, hPs[i].cols() + hPs[i + 1].cols());
    curIH << hPs[i], hPs[i + 1];
    if (!geoutils::enumerateVs(curIH, curIV)) {
      return false;
    }
    nv = curIV.cols();
    curIOB.resize(3, nv);
    curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
    vPs.push_back(curIOB);
  }

  if (!geoutils::enumerateVs(hPs.back(), curIV)) {
    return false;
  }
  nv = curIV.cols();
  curIOB.resize(3, nv);
  curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
  vPs.push_back(curIOB);

  return true;
}

TrajOpt::TrajOpt(ros::NodeHandle& nh) : nh_(nh) {
  nh.param("planning/is_fov_omni", is_fov_omni_, false);
  nh.param("planning/fov_theta", fov_theta_, 0.26);//15 degree
  nh.param("planning/fov_ctr_theta", fov_ctr_theta_, 0.0);
  nh.param("planning/min_flight_height", min_flight_height_, 0.4);
  nh.param("planning/predict_dt", tracking_dt_, 1.0);
  nh.param("planning/predict_T", tracking_T_, 3.0);
  nh.param("planning/track_dist", tracking_dist_, 1.9);


  nh.param("planning/tolerance_d", tolerance_d_, 0.3);
  nh.param("planning/drone_id", drone_id_, -1);
  nh.param("planning/LI_extrinsic", LI_extrinsic_, 0.075);

  nh.param("optimization/max_outloop_num", max_outloop_num_, 4);
  nh.param("optimization/use_init_opt", use_init_opt_, true);
  nh.param("optimization/idx_start_fov_cost", idx_start_fov_cost_, 2);
  nh.param("optimization/K", K_, 8); 
  nh.param("optimization/vmax", vmax_, 3.0);
  nh.param("optimization/amax", amax_, 6.0);
  nh.param("optimization/omega_max", omega_max_, 2.4);

  nh.param("optimization/rhoP", rhoP_, 10000.0);
  nh.param("optimization/rhoV", rhoV_, 1000.0);
  nh.param("optimization/rhoA", rhoA_, 1000.0);
  nh.param("optimization/rhoT", rhoT_, 1000.0);
  nh.param("optimization/rhoOmega", rhoOmega_, 6000.0);
  nh.param("optimization/rhoSwarm", rhoSwarm_, 1500.0);
  nh.param("optimization/rhoDistance", rhoDistance_, 1000.0);
  nh.param("optimization/rhoOcclusion", rhoOcclusion_, 1000.0);
  nh.param("optimization/rhoSeparation", rhoSeparation_, 1000.0);
  nh.param("optimization/rhoFOV_vertical", rhoFOV_vertical_init_, 10000.0);
  nh.param("optimization/rhoFOV_horizontal", rhoFOV_horizontal_init_, 40000.0);
  nh.param("optimization/rhoVSDF", rhoVSDF_, 30000000.0);

  nh.param("optimization/corridor_clearance", corridor_clearance_, 0.0);
  nh.param("optimization/swarm_clearance", swarm_clearance_, 0.5);
  nh.param("optimization/occlusion_clearance", occlusion_clearance_, 0.1);
  nh.param("optimization/des_delta_h", des_delta_h_, 0.3);

  ROS_WARN_STREAM("[OPT] param: vmax: " << vmax_);
  ROS_WARN_STREAM("[OPT] param: amax: " << amax_);
  ROS_WARN_STREAM("[OPT] param: omegamax: " << omega_max_);
  ROS_WARN_STREAM("[OPT] param: rhoP: " << rhoP_);
  ROS_WARN_STREAM("[OPT] param: rhoV: " << rhoV_);
  ROS_WARN_STREAM("[OPT] param: rhoA: " << rhoA_);
  ROS_WARN_STREAM("[OPT] param: rhoT: " << rhoT_);
  ROS_WARN_STREAM("[OPT] param: rhoOmega: " << rhoOmega_);
  ROS_WARN_STREAM("[OPT] param: rhoSwarm: " << rhoSwarm_);
  ROS_WARN_STREAM("[OPT] param: rhoVSDF: " << rhoVSDF_);
  
  ROS_WARN_STREAM("[OPT] param: rhoDistance: "   << rhoDistance_);
  ROS_WARN_STREAM("[OPT] param: rhoFOV_vertical: "   << rhoFOV_vertical_init_);
  ROS_WARN_STREAM("[OPT] param: rhoFOV_horizontal: "   << rhoFOV_horizontal_init_);
  ROS_WARN_STREAM("[OPT] param: rhoOcclusion: "  << rhoOcclusion_);

  ROS_WARN_STREAM("[OPT] param: fov_theta: "  << fov_theta_);
  ROS_WARN_STREAM("[OPT] param: fov_ctr_theta: "  << fov_ctr_theta_);
  ROS_WARN_STREAM("[OPT] param: occlusion_clearance: "  << occlusion_clearance_);
  ROS_WARN_STREAM("[OPT] param: corridor_clearance: "   << corridor_clearance_);
  ROS_WARN_STREAM("[OPT] param: swarm_clearance: "  << swarm_clearance_);
  ROS_WARN_STREAM("[OPT] param: des_delta_h: "  << des_delta_h_);
}

bool TrajOpt::grad_cost_corridor(const Eigen::Vector3d& p,
                                 const Eigen::MatrixXd& hPoly,
                                 Eigen::Vector3d& gradp,
                                 double& cost) {
  bool ret = false;
  gradp.setZero();
  cost = 0;
  for (int i = 0; i < hPoly.cols(); ++i) {
    Eigen::Vector3d norm_vec = hPoly.col(i).head<3>();
    double pen = norm_vec.dot(p - hPoly.col(i).tail<3>() + corridor_clearance_ * norm_vec);
    if (pen > 0) {
      double pen2 = pen * pen;
      gradp += rhoP_ * 3 * pen2 * norm_vec;
      cost += rhoP_ * pen2 * pen;
      ret = true;
    }
  }
  return ret;
}


bool TrajOpt::grad_cost_v(const Eigen::Vector3d& v,
                          Eigen::Vector3d& gradv,
                          double& cost) {
  gradv.setZero();
  cost = 0;
  double vpen = v.squaredNorm() - vmax_ * vmax_;
  if (vpen > 0) {
    gradv = rhoV_ * 6 * vpen * vpen * v;
    cost = rhoV_ * vpen * vpen * vpen;
    return true;
  }
  return false;
}


bool TrajOpt::grad_cost_a(const Eigen::Vector3d& a,
                          Eigen::Vector3d& grada,
                          double& cost) {
  grada.setZero();
  cost = 0;
  double apen = a.squaredNorm() - amax_ * amax_;
  if (apen > 0) {
    grada = rhoA_ * 6 * apen * apen * a;
    cost = rhoA_ * apen * apen * apen;
    return true;
  }
  return false;
}

bool TrajOpt::grad_cost_floor(const Eigen::Vector3d& p,
                              Eigen::Vector3d& gradp,
                              double& cost)
{
  double pen = min_flight_height_ - p.z();
  if (pen > 0) {
    double grad = 0;
    cost = smoothedL1(pen, grad);
    cost *= 100000 * rhoP_;
    gradp.setZero();
    gradp.z() = -100000 * rhoP_ * grad;
    return true;
  } else {
    return false;
  }
}


bool TrajOpt::grad_cost_omega(const Eigen::Vector3d& omega,
                              Eigen::Vector3d& gradomega,
                              double& cost)
{
  gradomega.setZero();
  cost = 0;
  bool ret = false;

  Eigen::Vector3d max_vec, rate_vec;
  Eigen::Matrix3d grad_tmp;

  max_vec << omega_max_ / 3.0, omega_max_ / 3.0, omega_max_;
  rate_vec = omega;
  grad_tmp <<
  2 * omega(0), 0.0, 0.0,
  0.0, 2 * omega(1), 0.0,
  0.0, 0.0, 2 * omega(2);
  for(int i = 0; i < 3; i++)
  {
    double pen = rate_vec(i) * rate_vec(i) - max_vec(i) * max_vec(i);
    if(pen > 0)
    {
      ret = true;
      double pen2 = pen * pen;
      cost += rhoOmega_ * pen * pen2;
      gradomega += rhoOmega_ * 3 * pen2 * grad_tmp.row(i);
    }
  }
  return ret;
}


bool TrajOpt::grad_cost_FOV(const Eigen::Vector3d& pos, 
                            const Eigen::Vector4d& quat,
                            const Eigen::Vector3d& target,
                            Eigen::Vector3d& gradp,
                            Eigen::Vector4d& gradquat,
                            double& cost)
{
  if(rhoFOV_horizontal_init_ == 0 || rhoFOV_vertical_init_ == 0) return false;

  bool ret = false;
  gradp.setZero();
  gradquat.setZero();
  cost = 0;
  
  Eigen::Matrix3d R = q2R(quat);
  Eigen::Vector3d pe(0, 0, LI_extrinsic_);
  Eigen::Vector3d pb = R.transpose() * (target - pos) - pe;
  Eigen::MatrixXd DpbDquat(4, 3);
  Eigen::Vector3d DcostDpb; 
  double px = pb(0), py = pb(1), pz = pb(2);
  double n2_pb = sqrt(px * px + py * py);
  double n_pb = sqrt(px * px + py * py + pz * pz); 
  double tanA = tan(fov_ctr_theta_);
  Eigen::Vector3d pc(px, py, tanA * n2_pb);

  double pen_vert = cos(fov_theta_) - pb.dot(pc) / n_pb / pc.norm();
  double pen_horz = (1 - pb(0) / n2_pb);

  if(pen_vert > 0 || (!is_fov_omni_ && pen_horz > 0)){
    double l1 = (target - pos).x();
    double l2 = (target - pos).y();
    double l3 = (target - pos).z();
    double w{quat(0)}, x{quat(1)}, y{quat(2)}, z{quat(3)};
    DpbDquat(0, 0) = 2*l2*z - 2*l3*y;
    DpbDquat(0, 1) = 2*l3*x - 2*l1*z;
    DpbDquat(0, 2) = 2*l1*y - 2*l2*x;
    DpbDquat(1, 0) = 2*l2*y + 2*l3*z;
    DpbDquat(1, 1) = 2*l3*w - 4*l2*x + 2*l1*y;
    DpbDquat(1, 2) = 2*l1*z - 4*l3*x - 2*l2*w;
    DpbDquat(2, 0) = 2*l2*x - 2*l3*w - 4*l1*y;
    DpbDquat(2, 1) = 2*l1*x + 2*l3*z;
    DpbDquat(2, 2) = 2*l1*w - 4*l3*y + 2*l2*z;
    DpbDquat(3, 0) = 2*l2*w + 2*l3*x - 4*l1*z;
    DpbDquat(3, 1) = 2*l3*y - 2*l1*w - 4*l2*z;
    DpbDquat(3, 2) = 2*l1*x + 2*l2*y;
  }else{
    return ret;
  }

  // if(pen_vert > 0)
  if(true)
  {
    double num1 = tanA * n2_pb * n2_pb - pz * n2_pb;
    double den1 = n2_pb * n2_pb * sqrt(tanA * tanA + 1) * n_pb * n_pb * n_pb;
    DcostDpb(0) = px * pz * num1 / den1;
    DcostDpb(1) = py * pz * num1 / den1;
    DcostDpb(2) = (pz * n2_pb - tanA * n2_pb * n2_pb) / (sqrt(tanA * tanA + 1) * n_pb * n_pb * n_pb);

    ret = true;
    double grad{0};
    cost += rhoFOV_vertical_ * smoothedL1(pen_vert, grad);
    gradp -= rhoFOV_vertical_ * grad * R * DcostDpb;
    gradquat += rhoFOV_vertical_ * grad * DpbDquat * DcostDpb;    
  }

  if(!is_fov_omni_ && pen_horz > 0)
  {
    DcostDpb(0) = -py * py / (n2_pb * n2_pb * n2_pb);
    DcostDpb(1) = px * py / (n2_pb * n2_pb * n2_pb);
    DcostDpb(2) = 0;

    ret = true;
    double grad{1.0};
    cost += rhoFOV_horizontal_ * smoothedL1(pen_horz, grad);
    gradp -= rhoFOV_horizontal_ * grad * R * DcostDpb;
    gradquat += rhoFOV_horizontal_ * grad * DpbDquat * DcostDpb;
  }

  return ret;
}


bool TrajOpt::grad_cost_distance(const Eigen::Vector3d& p,
                                   const Eigen::Vector3d& target_p,
                                   Eigen::Vector3d& gradp,
                                   double& costp) {
  double upper = tracking_dist_ + tolerance_d_;
  double lower = tracking_dist_ - tolerance_d_;
  upper = upper * upper;
  lower = lower * lower;

  Eigen::Vector3d dp = (p - target_p);
  double dr2 = dp.squaredNorm();

  bool ret = false;
  gradp.setZero();
  costp = 0;
  double pen;

  pen = dr2 - upper;
  if (pen > 0) {
    double grad;
    costp += distF(pen, grad);
    gradp += 2 * grad * dp;
    ret = true;
  } else {
    pen = lower - dr2;
    if (pen > 0) {
      double pen2 = pen * pen;
      gradp -= 400 * 6 * pen2 * dp;
      costp += 400 * pen2 * pen;
      ret = true;
    }
  }
  gradp *= rhoDistance_;
  costp *= rhoDistance_;

  double pen_z = target_p.z() - des_delta_h_ - p.z();
  if(pen_z > 0)
  {
    ret = true;
    double grad = 0;
    costp += 10000 * rhoP_ * smoothedL1(pen_z, grad);
    gradp.z() += -10000 * rhoP_ * grad;
  }

  return ret;
}

bool TrajOpt::grad_cost_swarm_collision(double t_g,
                                        const Eigen::Vector3d& pos,
                                        const Eigen::Vector3d& vel,
                                        Eigen::Vector3d& gradp,
                                        double &gradt,
                                        double &grad_prev_t,
                                        double &cost)
{
  bool ret = false;
  gradp.setZero(); gradt = 0; grad_prev_t = 0; cost = 0;
  
  double clr2 = (swarm_clearance_ * 1.48) * (swarm_clearance_ * 1.48);
  constexpr double a = 1.5, b = 1.0, inv_a2 = 1 / a / a, inv_b2 = 1 / b / b;
  double pt_time = t_traj_start_ + t_g;

  for (size_t id = 0; id < swarm_trajs_->size(); id++)
  {
    if ((swarm_trajs_->at(id).drone_id < 0) || swarm_trajs_->at(id).drone_id == drone_id_)
    {
      continue;
    }

    if(swarm_trajs_->at(id).drone_id != (int)id)
    {
      continue;
    }

    double traj_i_start_time = swarm_trajs_->at(id).start_time;
    Eigen::Vector3d swarm_p, swarm_v;
    if (pt_time < traj_i_start_time + swarm_trajs_->at(id).duration)
    {
      swarm_p = swarm_trajs_->at(id).traj.getPos(pt_time - traj_i_start_time);
      swarm_v = swarm_trajs_->at(id).traj.getVel(pt_time - traj_i_start_time);
    }
    else
    {
      double exceed_time = pt_time - (traj_i_start_time + swarm_trajs_->at(id).duration);
      swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
      swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
                exceed_time * swarm_v;
    }
    Eigen::Vector3d dist_vec = pos - swarm_p;
    double ellip_dist2 = dist_vec(2) * dist_vec(2) * inv_a2 + 
                         (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2;
    double pen = clr2 - ellip_dist2;
    
    if (pen > 0)
    {
      ret = true;
      double grad;
      cost += rhoSwarm_ * penF3(pen, grad);
      Eigen::Vector3d DJDp = rhoSwarm_ * grad * (-2) * 
                             Eigen::Vector3d(inv_b2 * dist_vec(0), inv_b2 * dist_vec(1), inv_a2 * dist_vec(2));
      gradp += DJDp;
      gradt += DJDp.dot(vel - swarm_v);
      grad_prev_t += DJDp.dot(-swarm_v);
    }
  }
  return ret;
}

bool TrajOpt::grad_cost_vsdf(const Eigen::Vector3d& pos,
                             const VisSDF::Ptr& vsdf,
                             Eigen::Vector3d& gradp,
                             double& cost)
{
  bool ret = false;
  gradp.setZero();  
  cost = 0;

  double dist;
  Eigen::Vector3d dist_grad;
  vsdf->evaluateSDTwithGradTrilinear(pos, dist, dist_grad);
  double pen = dist;
  if(pen > 0)
  {
    ret = true;
    double grad = 0;
    cost = rhoVSDF_ * pow(pen, 3);
    gradp = rhoVSDF_ * 3 * pow(pen, 2) * dist_grad;
  }
  return ret;
}


bool TrajOpt::grad_cost_swarm_bearing(double t_g, 
                                      const Eigen::Vector3d& p, 
                                      const Eigen::Vector3d& center,
                                      Eigen::Vector3d& gradp,
                                      double& costp)
{
  bool ret = false;
  if(teammate_list_->empty()) return ret;

  gradp.setZero(); costp = 0;
  Eigen::Vector3d vecx = p - center; 

  double norm_x = vecx.norm();

  double pt_time = t_traj_start_ + t_g;
  double cosOcc = cos(occlusion_clearance_);

  double cosB, pen_sep, pen_occ, inner_product, norm_n;
  Eigen::Vector3d vecn, dcosBdp;
  
  int t_idx = (int)(t_g / tracking_dt_);

  for (auto tmt_pos : teammate_list_->at(t_idx))
  {
    vecn = tmt_pos - center; 

    norm_n = vecn.norm();
    inner_product = vecx.dot(vecn);
    cosB = inner_product / norm_x / norm_n;
    dcosBdp = (norm_x * vecn - inner_product / norm_x * vecx) / norm_x / norm_x / norm_n;
    

    //Occlusion cost
    pen_occ = cosB - cosOcc;
    if (pen_occ > 0) {
      ret = true;
      double grad = 0;
      costp += rhoOcclusion_ * penF3(pen_occ, grad); //cubic
      gradp += rhoOcclusion_ * grad * dcosBdp;
    } 
  }
  return ret;
}


bool TrajOpt::grad_cost_direct_angular_sep(double t_g, 
                                           const Eigen::Vector3d& p, 
                                           const Eigen::Vector3d& center,
                                           Eigen::Vector3d& gradp,
                                           double& costp)
{

  if(teammate_list_->empty()) return false;
  int t_idx = (int)(t_g / tracking_dt_);

  gradp.setZero(); costp = 0;

  double cost_c = -1;
  Eigen::Vector3d nbr_bearing;
  Eigen::Vector3d ego_bearing;
  Eigen::Vector3d relative_pos = p - center;
  ego_bearing = relative_pos.normalized();
  for (auto tmt_pos : teammate_list_->at(t_idx))
  {
    nbr_bearing = (tmt_pos - center).normalized();
    double dnorm = (nbr_bearing - ego_bearing).norm();
    if(dnorm == 0)
    {
      return true;
    }else{
      cost_c *= dnorm;
    }
  }

  Eigen::Vector3d DcostDx(0,0,0), DcostDn(0,0,0);
  
  for(auto tmt_pos : teammate_list_->at(t_idx))
  {
    nbr_bearing = (tmt_pos - center).normalized();
    DcostDn += (ego_bearing - nbr_bearing) * (cost_c / (ego_bearing - nbr_bearing).squaredNorm());
  }

  double d0 = DcostDn(0);
  double d1 = DcostDn(1);
  double d2 = DcostDn(2);
  double x0 = relative_pos(0);
  double x1 = relative_pos(1);
  double x2 = relative_pos(2);

  double xnorm = relative_pos.norm();
  double sig = xnorm * xnorm * xnorm;

  DcostDx << (d0 * x1 * x1 - d1 * x0 * x1 + d0 * x2 * x2 - d2 * x0 * x2) / sig,
             (d1 * x0 * x0 - d0 * x1 * x0 + d1 * x2 * x2 - d2 * x1 * x2) / sig,
             (d2 * x0 * x0 - d0 * x2 * x0 + d2 * x1 * x1 - d1 * x2 * x1) / sig;
  costp = rhoSeparation_ * cost_c;
  gradp = rhoSeparation_ * DcostDx;
  return true;
}


//For tracking task, we check the whole trajectory
int TrajOpt::checkCollision(const Trajectory& traj)
{
  double T_end = traj.getTotalDuration();

  int occ = OPT_STATE::VALID;
  double dt = 0.02;
  double pt_time, traj_time;

  for (double t = 0.0; t < T_end; t += dt)
  {
    //Check environment collision
    Eigen::Vector3d pos = traj.getPos(t);
    pt_time = t + t_traj_start_;

    // if(rogmap_->isOccupiedInflate(pos)){ 
    if(rogmap_->isOccupiedInflateWithNbr(pos)){ 
      occ = OPT_STATE::OBS_OCC;
      break;
    }

    if(pos.z() < min_flight_height_ - 0.1){
      occ = OPT_STATE::ZAX_OCC;
      break;
    }

    for (size_t id = 0; id < swarm_trajs_->size(); id++)
    {
      if ((swarm_trajs_->at(id).drone_id < 0) || swarm_trajs_->at(id).drone_id == drone_id_)
        continue;
      
      if(swarm_trajs_->at(id).drone_id != (int)id)
        continue;
      
      traj_time = pt_time - swarm_trajs_->at(id).start_time;

      if(traj_time < 0)
        continue;

      if(traj_time > swarm_trajs_->at(id).duration)
        continue;
    
      if((pos - swarm_trajs_->at(id).traj.getPos(traj_time)).norm() < 0.9 * swarm_clearance_)
      {
        occ = OPT_STATE::SWM_OCC;
        break;
      }
    }
    if(occ) break;
  }   

  //Check target collision
  int iter{0};
  for(double t = 0.0; t < T_end; t += tracking_dt_)
  {
    if((tracking_pts_[iter++] - traj.getPos(t)).norm() < 0.75 * (tracking_dist_ - tolerance_d_))
    {
      occ = OPT_STATE::TGT_OCC;
      break;
    }
  }
  
  return occ;
}

//----------------
bool TrajOpt::isTrajInCorridors(const Trajectory& traj, const double out_margin)
{

  int innerLoop = K_ + 1;
  double prev_t = 0.0;
  for(int i = 0; i < traj.getPieceNum(); ++i)
  {
    double step = traj[i].getDuration() / K_;
    for(int j = 0; j < innerLoop; ++j)
    {
      double t = prev_t + j * step;
      Eigen::Vector3d pos = traj.getPos(t);

      bool is_pos_in_corridors{false};
      for(auto& hPoly : cfgHs_)
      {
        if(isPosInCorridor(pos, hPoly, out_margin)) is_pos_in_corridors = true;
      }

      if(!is_pos_in_corridors) return false;
    }
    prev_t += traj[i].getDuration();
  }
  return true;
}


bool TrajOpt::isPosInCorridor(const Eigen::Vector3d& pos, const Eigen::MatrixXd& hPoly, const double out_margin)
{
  Eigen::Vector3d norm_vec;
  for(int k = 0; k < hPoly.cols(); k++)
  {
    norm_vec = hPoly.col(k).head<3>();
    double pen = norm_vec.dot(pos - hPoly.col(k).tail<3>() - out_margin * norm_vec);
    if(pen > 0) return false;
  }
  return true;
}


}  // namespace traj_opt
