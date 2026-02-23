#include <optimizer_corridor/traj_optimizer.h>

namespace traj_opt {

//SECTION Helper Funcs

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

static Eigen::Matrix3d q2R(const Eigen::Vector4d& q)
{
  Eigen::Quaterniond Quat(q(0), q(1), q(2), q(3));
  return Quat.toRotationMatrix();
}

static double angDiff(const double& des_ang, const double& src_ang)
{
  double diff = des_ang - src_ang;
  return diff - floor((diff + M_PI)  / (2 * M_PI)) * (2 * M_PI);
}
// !SECTION  Helper Funcs

// SECTION  Variables Transformation and Grad Prop
static double expC2(double t) {
  return t > 0.0 ? ((0.5 * t + 1.0) * t + 1.0)
                 : 1.0 / ((0.5 * t - 1.0) * t + 1.0);
}
static double logC2(double T) {
  return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
}

static void forwardT(const Eigen::Ref<const Eigen::VectorXd>& t, const double& sT, Eigen::Ref<Eigen::VectorXd> vecT) {
  int M = t.size();
  for (int i = 0; i < M; ++i) {
    vecT(i) = expC2(t(i));
  }
  vecT(M) = 0.0;
  vecT /= 1.0 + vecT.sum();
  vecT(M) = 1.0 - vecT.sum();
  vecT *= sT;
  return;
}
static void backwardT(const Eigen::Ref<const Eigen::VectorXd>& vecT, Eigen::Ref<Eigen::VectorXd> t) {
  int M = t.size();
  t = vecT.head(M) / vecT(M);
  for (int i = 0; i < M; ++i) {
    t(i) = logC2(vecT(i));
  }
  return;
}

static void addLayerTGrad(const Eigen::Ref<const Eigen::VectorXd>& t,
                          const double& sT,
                          const Eigen::Ref<const Eigen::VectorXd>& gradT,
                          Eigen::Ref<Eigen::VectorXd> gradt) {
  int Ms1 = t.size();
  Eigen::VectorXd gFree = sT * gradT.head(Ms1);
  double gTail = sT * gradT(Ms1);
  Eigen::VectorXd dExpTau(Ms1);
  double expTauSum = 0.0, gFreeDotExpTau = 0.0;
  double denSqrt, expTau;
  for (int i = 0; i < Ms1; i++) {
    if (t(i) > 0) {
      expTau = (0.5 * t(i) + 1.0) * t(i) + 1.0;
      dExpTau(i) = t(i) + 1.0;
      expTauSum += expTau;
      gFreeDotExpTau += expTau * gFree(i);
    } else {
      denSqrt = (0.5 * t(i) - 1.0) * t(i) + 1.0;
      expTau = 1.0 / denSqrt;
      dExpTau(i) = (1.0 - t(i)) / (denSqrt * denSqrt);
      expTauSum += expTau;
      gFreeDotExpTau += expTau * gFree(i);
    }
  }
  denSqrt = expTauSum + 1.0;
  gradt = (gFree.array() - gTail) * dExpTau.array() / denSqrt -
          (gFreeDotExpTau - gTail * expTauSum) * dExpTau.array() / (denSqrt * denSqrt);
}

static void forwardP(const Eigen::Ref<const Eigen::VectorXd>& p,
                     const std::vector<Eigen::MatrixXd>& cfgPolyVs,
                     Eigen::MatrixXd& inP) {
  int M = cfgPolyVs.size();
  Eigen::VectorXd q;
  int j = 0, k;
  for (int i = 0; i < M; ++i) {
    k = cfgPolyVs[i].cols() - 1;
    q = 2.0 / (1.0 + p.segment(j, k).squaredNorm()) * p.segment(j, k);
    inP.col(i) = cfgPolyVs[i].rightCols(k) * q.cwiseProduct(q) +
                 cfgPolyVs[i].col(0);
    j += k;
  }
  return;
}
static double objectiveNLS(void* ptrPOBs,
                           const double* x,
                           double* grad,
                           const int n) {
  const Eigen::MatrixXd& pobs = *(Eigen::MatrixXd*)ptrPOBs;
  Eigen::Map<const Eigen::VectorXd> p(x, n);
  Eigen::Map<Eigen::VectorXd> gradp(grad, n);

  double qnsqr = p.squaredNorm();
  double qnsqrp1 = qnsqr + 1.0;
  double qnsqrp1sqr = qnsqrp1 * qnsqrp1;
  Eigen::VectorXd r = 2.0 / qnsqrp1 * p;

  Eigen::Vector3d delta = pobs.rightCols(n) * r.cwiseProduct(r) +
                          pobs.col(1) - pobs.col(0);
  double cost = delta.squaredNorm();
  Eigen::Vector3d gradR3 = 2 * delta;

  Eigen::VectorXd gdr = pobs.rightCols(n).transpose() * gradR3;
  gdr = gdr.array() * r.array() * 2.0;
  gradp = gdr * 2.0 / qnsqrp1 -
          p * 4.0 * gdr.dot(p) / qnsqrp1sqr;

  return cost;
}

static void backwardP(const Eigen::Ref<const Eigen::MatrixXd>& inP,
                      const std::vector<Eigen::MatrixXd>& cfgPolyVs,
                      Eigen::VectorXd& p) {
  int M = inP.cols();
  int j = 0, k;

  // Parameters for tiny nonlinear least squares
  double minSqrD;
  lbfgs::lbfgs_parameter_t nls_params;
  lbfgs::lbfgs_load_default_parameters(&nls_params);
  nls_params.g_epsilon = FLT_EPSILON;
  nls_params.max_iterations = 128;

  Eigen::MatrixXd pobs;
  for (int i = 0; i < M; i++) {
    k = cfgPolyVs[i].cols() - 1;
    p.segment(j, k).setConstant(1.0 / (sqrt(k + 1.0) + 1.0));
    pobs.resize(3, k + 2);
    pobs << inP.col(i), cfgPolyVs[i];
    lbfgs::lbfgs_optimize(k,
                          p.data() + j,
                          &minSqrD,
                          &objectiveNLS,
                          nullptr,
                          nullptr,
                          &pobs,
                          &nls_params);
    j += k;
  }
  return;
}
static void addLayerPGrad(const Eigen::Ref<const Eigen::VectorXd>& p,
                          const std::vector<Eigen::MatrixXd>& cfgPolyVs,
                          const Eigen::Ref<const Eigen::MatrixXd>& gradInPs,
                          Eigen::Ref<Eigen::VectorXd> grad) {
  int M = gradInPs.cols();

  int j = 0, k;
  double qnsqr, qnsqrp1, qnsqrp1sqr;
  Eigen::VectorXd q, r, gdr;
  for (int i = 0; i < M; i++) {
    k = cfgPolyVs[i].cols() - 1;
    q = p.segment(j, k);
    qnsqr = q.squaredNorm();
    qnsqrp1 = qnsqr + 1.0;
    qnsqrp1sqr = qnsqrp1 * qnsqrp1;
    r = 2.0 / qnsqrp1 * q;
    gdr = cfgPolyVs[i].rightCols(k).transpose() * gradInPs.col(i).head<3>();
    gdr = gdr.array() * r.array() * 2.0;

    grad.segment(j, k) = gdr * 2.0 / qnsqrp1 -
                         q * 4.0 * gdr.dot(q) / qnsqrp1sqr;
    j += k;
  }
  return;
}

static void addLayerYawGrad(const Eigen::Ref<const Eigen::MatrixXd>& gradInPs,
                            Eigen::Ref<Eigen::VectorXd> grad) {
  for (int i = 0; i < gradInPs.cols(); i++)
    grad(i) = gradInPs(0, i);
  return;
}
// !SECTION  Variables Transformation and Grad Prop

// SECTION object function
static inline double objectiveFunc(void* ptrObj,
                                   const double* x,
                                   double* grad,
                                   const int n) {
  TrajOpt& obj = *(TrajOpt*)ptrObj;

  Eigen::Map<const Eigen::VectorXd> yaw(x, obj.dim_yaw_);
  Eigen::Map<const Eigen::VectorXd> t(x + obj.dim_yaw_, obj.dim_t_);
  Eigen::Map<const Eigen::VectorXd> p(x + obj.dim_yaw_ + obj.dim_t_, obj.dim_p_);
  Eigen::Map<const Eigen::Vector3d> tailA(x + obj.dim_yaw_ + obj.dim_t_ + obj.dim_p_);
  Eigen::Map<Eigen::VectorXd> gradyaw(grad, obj.dim_yaw_);
  Eigen::Map<Eigen::VectorXd> gradt(grad + obj.dim_yaw_, obj.dim_t_);
  Eigen::Map<Eigen::VectorXd> gradp(grad + obj.dim_yaw_ + obj.dim_t_, obj.dim_p_);
  Eigen::Map<Eigen::Vector3d> gradtailA(grad + obj.dim_yaw_ + obj.dim_t_ + obj.dim_p_);

  Eigen::VectorXd T(obj.N_);
  Eigen::VectorXd Yaw(obj.N_);
  Eigen::MatrixXd P(3, obj.N_);

  forwardT(t, obj.sum_T_, T);
  forwardP(p, obj.cfgVs_, P);
  Yaw = yaw;

  obj.finS_.setZero();
  obj.finS_.col(0) << P.rightCols(1), Yaw(obj.N_ - 1);
  obj.finS_.col(1).head<3>() = obj.tailV_;
  obj.finS_.col(2).head<3>() = tailA;

  obj.snapOpt_.generate(obj.iniS_.topRows<3>(), obj.finS_.topRows<3>(), P, T);
  obj.accOpt_.generate(obj.iniS_.bottomLeftCorner<1, 2>(), 
                       obj.finS_.bottomLeftCorner<1, 2>(), 
                       Yaw.transpose(), T);
  obj.gradT_.setZero();

  double cost = obj.snapOpt_.getTrajSnapCost();
  cost += obj.accOpt_.getTrajAccCost();
  obj.snapOpt_.calGrads_CT();
  obj.accOpt_.calGrads_CT();
  obj.addTimeIntPenalty(cost);
  obj.addTimeCost(cost);
  obj.snapOpt_.calGrads_PT();
  obj.accOpt_.calGrads_PT();
  obj.gradT_ += obj.snapOpt_.gdT;
  obj.gradT_ += obj.accOpt_.gdT;
  obj.addTailCost(cost); 
  addLayerTGrad(t, obj.sum_T_, obj.gradT_, gradt);
  Eigen::MatrixXd gradPInTail(3, obj.N_);
  gradPInTail << obj.snapOpt_.gdP, obj.snapOpt_.gdTail.col(0);
  addLayerPGrad(p, obj.cfgVs_, gradPInTail, gradp);
  Eigen::MatrixXd gradYawInTail(1, obj.N_);
  gradYawInTail << obj.accOpt_.gdP, obj.accOpt_.gdTail.col(0);
  addLayerYawGrad(gradYawInTail, gradyaw);
  gradtailA = obj.snapOpt_.gdTail.col(2);

  return cost;
}


// SECTION object function
static inline double initObjectiveFunc(void* ptrObj,
                                       const double* x,
                                       double* grad,
                                       const int n) {
  TrajOpt& obj = *(TrajOpt*)ptrObj;

  Eigen::Map<const Eigen::VectorXd> yaw(x, obj.dim_yaw_);
  Eigen::Map<const Eigen::VectorXd> t(x + obj.dim_yaw_, obj.dim_t_);
  Eigen::Map<const Eigen::VectorXd> p(x + obj.dim_yaw_ + obj.dim_t_, obj.dim_p_);
  Eigen::Map<const Eigen::Vector3d> tailA(x + obj.dim_yaw_ + obj.dim_t_ + obj.dim_p_);
  Eigen::Map<Eigen::VectorXd> gradyaw(grad, obj.dim_yaw_);
  Eigen::Map<Eigen::VectorXd> gradt(grad + obj.dim_yaw_, obj.dim_t_);
  Eigen::Map<Eigen::VectorXd> gradp(grad + obj.dim_yaw_ + obj.dim_t_, obj.dim_p_);
  Eigen::Map<Eigen::Vector3d> gradtailA(grad + obj.dim_yaw_ + obj.dim_t_ + obj.dim_p_);

  Eigen::VectorXd T(obj.N_);
  Eigen::VectorXd Yaw(obj.N_);
  Eigen::MatrixXd P(3, obj.N_);

  forwardT(t, obj.sum_T_, T);
  forwardP(p, obj.cfgVs_, P);
  Yaw = yaw;

  obj.finS_.setZero();
  obj.finS_.col(0) << P.rightCols(1), Yaw(obj.N_ - 1);
  obj.finS_.col(1).head<3>() = obj.tailV_;
  obj.finS_.col(2).head<3>() = tailA;

  obj.snapOpt_.generate(obj.iniS_.topRows<3>(), obj.finS_.topRows<3>(), P, T);
  obj.accOpt_.generate(obj.iniS_.bottomLeftCorner<1, 2>(), 
                       obj.finS_.bottomLeftCorner<1, 2>(), 
                       Yaw.transpose(), T);
  obj.gradT_.setZero();

  double cost = obj.snapOpt_.getTrajSnapCost();
  cost += obj.accOpt_.getTrajAccCost();
  obj.snapOpt_.calGrads_CT();
  obj.accOpt_.calGrads_CT();
  obj.addTimeIntPenalty_init(cost);
  obj.addTimeCost_init(cost);
  obj.snapOpt_.calGrads_PT();
  obj.accOpt_.calGrads_PT();
  obj.gradT_ += obj.snapOpt_.gdT;
  obj.gradT_ += obj.accOpt_.gdT;
  obj.addTailCost_init(cost); 
  addLayerTGrad(t, obj.sum_T_, obj.gradT_, gradt);
  Eigen::MatrixXd gradPInTail(3, obj.N_);
  gradPInTail << obj.snapOpt_.gdP, obj.snapOpt_.gdTail.col(0);
  addLayerPGrad(p, obj.cfgVs_, gradPInTail, gradp);
  Eigen::MatrixXd gradYawInTail(1, obj.N_);
  gradYawInTail << obj.accOpt_.gdP, obj.accOpt_.gdTail.col(0);
  addLayerYawGrad(gradYawInTail, gradyaw);
  gradtailA = obj.snapOpt_.gdTail.col(2);

  return cost;
}


// !SECTION object function

static inline int opt_feedback(void* ptrObj,
                               const double* x,
                               const double* grad,
                               const double fx,
                               const double xnorm,
                               const double gnorm,
                               const double step,
                               int n,
                               int k,
                               int ls) {
  TrajOpt& obj = *(TrajOpt*)ptrObj;
  double t_now = ros::Time::now().toSec();
  if((t_now - obj.t_replan_start_) > (obj.replan_time_budget_ + 0.015))
  {
    return 1;
  }else{
    return 0;
  }
}

static inline int init_opt_feedback(void* ptrObj,
                                    const double* x,
                                    const double* grad,
                                    const double fx,
                                    const double xnorm,
                                    const double gnorm,
                                    const double step,
                                    int n,
                                    int k,
                                    int ls) {


  TrajOpt& obj = *(TrajOpt*)ptrObj;
  obj.init_opt_iternum_++;
  double t_now = ros::Time::now().toSec();

  Eigen::Map<const Eigen::VectorXd> yaw(x, obj.dim_yaw_);
  Eigen::Map<const Eigen::VectorXd> t(x + obj.dim_yaw_, obj.dim_t_);
  Eigen::Map<const Eigen::VectorXd> p(x + obj.dim_yaw_ + obj.dim_t_, obj.dim_p_);
  Eigen::Map<const Eigen::Vector3d> tailA(x + obj.dim_yaw_ + obj.dim_t_ + obj.dim_p_);

  Eigen::VectorXd T(obj.N_);
  Eigen::VectorXd Yaw(obj.N_);
  Eigen::MatrixXd P(3, obj.N_);

  forwardT(t, obj.sum_T_, T);
  forwardP(p, obj.cfgVs_, P);
  Yaw = yaw;

  obj.finS_.setZero();
  obj.finS_.col(0) << P.rightCols(1), Yaw(obj.N_ - 1);
  obj.finS_.col(1).head<3>() = obj.tailV_;
  obj.finS_.col(2).head<3>() = tailA;

  obj.snapOpt_.generate(obj.iniS_.topRows<3>(), obj.finS_.topRows<3>(), P, T);
  Trajectory mid_traj = obj.snapOpt_.getTraj();
  obj.init_opt_res_dis_.clear();
  for(int i = 0; i < obj.front_path_.size(); i++)
  {
    Eigen::Vector3d pos = mid_traj.getPos(i * obj.tracking_dt_);
    obj.init_opt_res_dis_.emplace_back((pos - obj.front_path_[i]).norm());
  }

  if((t_now - obj.t_replan_start_) > (obj.replan_time_budget_ - 0.010))
  {
    obj.is_init_overtime_ = true;
    return 1;
  }else{
    return 0;
  }
}


bool TrajOptConeFOV::setInitValues(const Eigen::MatrixXd& iniState, const Eigen::MatrixXd& finState) {
  
  iniS_ = iniState;
  finS_ = finState;
  iniS_(3, 0) = iniS_(3, 0) - floor((iniS_(3, 0) + M_PI)  / (2 * M_PI)) * (2 * M_PI);
  finS_(3, 0) = finS_(3, 0) - floor((finS_(3, 0) + M_PI)  / (2 * M_PI)) * (2 * M_PI);

  double tempNorm = iniS_.col(1).head(3).norm();
  iniS_.col(1).head(3) *= tempNorm > vmax_ ? (vmax_ / tempNorm) : 1.0;
  tempNorm = iniS_.col(2).head(3).norm();
  iniS_.col(2).head(3) *= tempNorm > amax_ ? (amax_ / tempNorm) : 1.0;
  tempNorm = finS_.col(1).head(3).norm();
  finS_.col(1).head(3) *= tempNorm > vmax_ ? (vmax_ / tempNorm) : 1.0;
  tempNorm = finS_.col(2).head(3).norm();
  finS_.col(2).head(3) *= tempNorm > amax_ ? (amax_ / tempNorm) : 1.0;

  //normal init
  Eigen::VectorXd T(N_);
  T.setConstant(sum_T_ / N_); 
  backwardT(T, t_);
  gradT_.setZero();

  Eigen::MatrixXd P(3, N_); 
  for (int i = 0; i < N_ - 1; ++i) {
    int k = cfgVs_[i].cols() - 1;
    P.col(i) = cfgVs_[i].rightCols(k).rowwise().sum() / (1.0 + k) + cfgVs_[i].col(0);
  }
  P.col(N_ - 1) = finS_.col(0).head(3);//tail Pos
  backwardP(P, cfgVs_, p_);

  double t_dur, rt;
  double yaw_prev, yaw_des, yaw_diff;
  yaw_prev = iniS_(3, 0);
  Eigen::Vector3d target_inp;
  for (int i = 0; i < N_; ++i){
    t_dur = T.head(i + 1).sum();
    rt = std::fmod(t_dur, tracking_dt_) / tracking_dt_;
    target_inp = (i < N_ - 1) ? 
                (1 - rt) * tracking_pts_[(int)(t_dur / tracking_dt_)]
                + rt * tracking_pts_[(int)(t_dur / tracking_dt_) + 1] :
                tracking_pts_.back();
    yaw_des = atan2(target_inp.y() - P.col(i).y(), target_inp.x() - P.col(i).x());
    yaw_diff = angDiff(yaw_des, yaw_prev);
    yaw_prev = yaw_[i] = yaw_prev + sgn(yaw_diff) * 
                          std::min(abs(yaw_diff), tracking_dt_ * omega_max_/2);
  }

  if(is_kino_path_)
  {
    init_yaw_path_.clear(); init_yaw_path_.resize(front_path_.size());
    init_yaw_path_[0] = iniS_(3, 0);
    for(int i = 1; i < front_path_.size(); i++)
    {
      yaw_des = atan2(tracking_pts_[i].y() - front_path_[i].y(), 
                      tracking_pts_[i].x() - front_path_[i].x());
      yaw_diff = angDiff(yaw_des, init_yaw_path_[i - 1]);
      init_yaw_path_[i] = init_yaw_path_[i - 1] + sgn(yaw_diff) * 
                          std::min(abs(yaw_diff), tracking_dt_ * omega_max_ / 2);
    }
  }

  finS_(3, 0) = is_kino_path_ ? init_yaw_path_.back() : yaw_(N_ - 1);

  tailV_ = finS_.col(1).head<3>();
  tailA_ = finS_.col(2).head<3>();
  snapOpt_.reset(N_);
  accOpt_.reset(N_);

  if(is_kino_path_ && use_init_opt_){

    TimeConsuming init_opt_timer("init_opt", false);
    int init_opt_ret = optimize_init();
    double init_opt_time = init_opt_timer.stop();
    init_overhead_ = init_opt_time;

    if (init_opt_ret < 0) {
      // ROS_WARN_STREAM("\033[32m[OPT] init: " << lbfgs::lbfgs_strerror(init_opt_ret) << "\033[0m");
      // std::cout << "\033[32m[OPT] init: " << lbfgs::lbfgs_strerror(init_opt_ret) << "\033[0m" << std::endl;
      return false;
    }

  }

  return true;
}


int TrajOptConeFOV::optimize_init(const double& delta) {
  // Setup for L-BFGS solver
  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
  lbfgs_params.mem_size = 16;
  lbfgs_params.past = 3;
  lbfgs_params.g_epsilon = 1e-10;
  lbfgs_params.min_step = 1e-32;
  lbfgs_params.delta = delta;
  lbfgs_params.line_search_type = 0;
  Eigen::Map<Eigen::VectorXd> yaw(x_, dim_yaw_);
  Eigen::Map<Eigen::VectorXd> t(x_ + dim_yaw_, dim_t_);
  Eigen::Map<Eigen::VectorXd> p(x_ + dim_yaw_ + dim_t_, dim_p_);
  Eigen::Map<Eigen::Vector3d> tailA(x_ + dim_yaw_ + dim_t_ + dim_p_);
  t = t_;
  p = p_;
  yaw = yaw_;
  tailA = tailA_;
  double minObjective;

  is_fov_first_ = true;
  is_init_value_ = true;
 
  auto ret = lbfgs::lbfgs_optimize(dim_yaw_ + dim_t_ + dim_p_ + 3, x_, &minObjective,
                                   &initObjectiveFunc, nullptr,
                                   &init_opt_feedback, this, &lbfgs_params);
  t_ = t;
  p_ = p;
  yaw_ = yaw;
  tailA_ = tailA;
  return ret;
}

int TrajOptConeFOV::optimize(const double& delta) {
  // Setup for L-BFGS solver
  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
  lbfgs_params.mem_size = 32;
  lbfgs_params.past = 3;
  lbfgs_params.g_epsilon = 1e-10;
  lbfgs_params.min_step = 1e-32;
  lbfgs_params.delta = delta;
  lbfgs_params.line_search_type = 0;
  Eigen::Map<Eigen::VectorXd> yaw(x_, dim_yaw_);
  Eigen::Map<Eigen::VectorXd> t(x_ + dim_yaw_, dim_t_);
  Eigen::Map<Eigen::VectorXd> p(x_ + dim_yaw_ + dim_t_, dim_p_);
  Eigen::Map<Eigen::Vector3d> tailA(x_ + dim_yaw_ + dim_t_ + dim_p_);
  t = t_;
  p = p_;
  yaw = yaw_;
  tailA = tailA_;
  double minObjective;

  is_fov_first_ = true;
  is_init_value_ = true;

  auto ret = lbfgs::lbfgs_optimize(dim_yaw_ + dim_t_ + dim_p_ + 3, x_, &minObjective,
                                   &objectiveFunc, nullptr,
                                   &opt_feedback, this, &lbfgs_params);
  t_ = t;
  p_ = p;
  yaw_ = yaw;
  tailA_ = tailA;
  return ret;
}

bool TrajOptConeFOV::generate_init_traj(const double t_replan_start,
                          const Eigen::MatrixXd& iniState,
                          const Eigen::MatrixXd& finState,
                          const std::vector<Eigen::Vector3d>& front_wpts,
                          const std::vector<Eigen::MatrixXd>& hPolys,
                          Trajectory& traj) {}

int TrajOptConeFOV::generate_traj(const double t_replan_start,
                                  const double t_traj_start,
                                  const Eigen::MatrixXd& iniState, //4x4
                                  const Eigen::MatrixXd& finState,//4x4
                                  const double &traj_duration,
                                  const std::vector<Eigen::Vector3d>& front_wpts,
                                  const std::vector<Eigen::Vector3d>& tracking_pts,
                                  const std::vector<Eigen::MatrixXd>& hPolys,
                                  Trajectory& traj,
                                  Trajectory1D& traj_yaw,
                                  bool is_kino_path)
  {

    out_loop_num_ = 0;
    init_overhead_ = 0.0;
    regu_overhead_ = 0.0;
    init_opt_iternum_ = 0;
    is_init_overtime_ = false;

    is_kino_path_ = is_kino_path; 
    int opt_state = OPT_STATE::VALID;
    t_replan_start_  = t_replan_start;
    t_traj_start_ = t_traj_start;
    cfgHs_ = hPolys;

    if (cfgHs_.size() == 1) {
      cfgHs_.push_back(cfgHs_[0]);
    }
    if (!extractVs(cfgHs_, cfgVs_)) { 
      // ROS_WARN("\033[32m[OPT] ExtractVs fail!\033[0m");
      // std::cout << "[OPT] extractVs fail!" << std::endl;
      opt_state = OPT_STATE::COD_ISS;
      return opt_state;
    }
   
    sum_T_ = traj_duration;
    tracking_pts_ = tracking_pts;
    front_path_ = front_wpts;
  

    N_ = 2 * cfgHs_.size(); 
    dim_yaw_ = N_;
    dim_t_ = N_ - 1;
    dim_p_ = 0;
    cfgVs_.emplace_back(cfgVs_.back());
    for (const auto& cfgV : cfgVs_) {
      dim_p_ += cfgV.cols() - 1;
    }

    p_.resize(dim_p_);
    yaw_.resize(dim_yaw_);
    t_.resize(dim_t_);
    gradT_.resize(N_);  
    x_ = new double[dim_yaw_ + dim_p_ + dim_t_ + 3];

    if(!setInitValues(iniState, finState))
    {
      opt_state = OPT_STATE::OPT_ISS;
      return opt_state;
    }

    rhoFOV_horizontal_ = rhoFOV_horizontal_init_;
    rhoFOV_vertical_ = rhoFOV_vertical_init_;

    TimeConsuming regu_opt_timer("regu_opt_t", false);

    int se3_trial{0};
    do{

      int opt_ret = optimize();
      if (opt_ret < 0) {
        // ROS_WARN_STREAM("\033[32m[OPT] main: " << lbfgs::lbfgs_strerror(opt_ret) << " iternum: " << se3_trial << "\033[0m");
        // std::cout << "\033[32m[OPT] main: " << lbfgs::lbfgs_strerror(opt_ret) << "\033[0m" << std::endl;
        if(se3_trial > 0){
          break;
        }else{
          opt_state = OPT_STATE::OPT_ISS;
          return opt_state;
        }
      }
      Eigen::VectorXd T(N_);
      Eigen::MatrixXd P(3, N_);
      Eigen::VectorXd Yaw(N_);
      forwardT(t_, sum_T_, T);
      forwardP(p_, cfgVs_, P);
      Yaw = yaw_;
      
      finS_.setZero();
      finS_.col(0) << P.rightCols(1), Yaw(N_ - 1);
      finS_.col(1).head<3>() = tailV_;
      finS_.col(2).head<3>() = tailA_;
      snapOpt_.generate(iniS_.topRows<3>(), finS_.topRows<3>(), P, T);
      accOpt_.generate(iniS_.bottomLeftCorner<1, 2>(), 
                       finS_.bottomLeftCorner<1, 2>(), 
                       Yaw.transpose(), T);
    
      traj = snapOpt_.getTraj();
      traj_yaw = accOpt_.getTraj();
      se3_trial++;
    }while(rhoFOV_horizontal_init_ > 0 && rhoFOV_vertical_init_ > 0 
           && !checkTrajFOV(traj, traj_yaw, false) && se3_trial < max_outloop_num_);
    regu_overhead_ = regu_opt_timer.stop();
    out_loop_num_ = se3_trial;
 
    opt_state = checkCollision(traj);

    delete[] x_;
    if(opt_state){
      // ROS_ERROR_STREAM("\033[32m[OPT] Traj is in collision: " << opt_state_str_[opt_state] << "\033[0m");
      // std::cout << "\033[32m[OPT] Traj is in collision: " << opt_state_str_[opt_state] << "\033[0m" << std::endl;
    }

    return opt_state;
  }

void TrajOptConeFOV::addTimeIntPenalty_init(double& cost)
{
  Eigen::Vector3d pos, vel, acc, jer, snp;
  Eigen::Vector3d grad_tmp, grad_p, grad_v, grad_a;
  double cost_tmp, cost_inner;
  Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3;
  double s1, s2, s3, s4, s5, s6, s7;
  double step, alpha;
  Eigen::Matrix<double, 8, 3> gradViola_c;
  double gradViola_t;
  double omg;

  double t_pre = 0;
  int innerLoop;
  for (int i = 0; i < N_; ++i) {
    const auto& c = snapOpt_.b.block<8, 3>(i * 8, 0);
    step = snapOpt_.T1(i) / K_;
    s1 = 0.0;
    innerLoop = K_ + 1;

    const auto& hPoly = cfgHs_[i / 2];
    for (int j = 0; j < innerLoop; ++j) {
      s2 = s1 * s1;
      s3 = s2 * s1;
      s4 = s2 * s2;
      s5 = s4 * s1;
      s6 = s4 * s2;
      s7 = s4 * s3;
      beta0 << 1.0, s1, s2, s3, s4, s5, s6, s7;
      beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
      beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3, 30.0 * s4, 42.0 * s5;
      beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2, 120.0 * s3, 210.0 * s4;
      alpha = 1.0 / K_ * j;
      pos = c.transpose() * beta0;
      vel = c.transpose() * beta1;
      acc = c.transpose() * beta2;
      jer = c.transpose() * beta3;

      grad_p.setZero();
      grad_v.setZero();
      grad_a.setZero();
      cost_inner = 0.0;

      omg = (j == 0 || j == innerLoop - 1) ? 0.5 : 1.0;

      if (grad_cost_corridor(pos, hPoly, grad_tmp, cost_tmp)) {
        grad_p += grad_tmp;
        cost_inner += cost_tmp;
      }

      if (grad_cost_floor(pos, grad_tmp, cost_tmp)) {
        grad_p += grad_tmp;
        cost_inner += cost_tmp;
      }

      if (grad_cost_v(vel, grad_tmp, cost_tmp)) {
        grad_v += grad_tmp;
        cost_inner += cost_tmp;
      }

      if (grad_cost_a(acc, grad_tmp, cost_tmp)) {
        grad_a += grad_tmp;
        cost_inner += cost_tmp;
      }

      gradViola_c = beta0 * grad_p.transpose();
      gradViola_t = grad_p.transpose() * vel;
      gradViola_c += beta1 * grad_v.transpose();
      gradViola_t += grad_v.transpose() * acc;
      gradViola_c += beta2 * grad_a.transpose();
      gradViola_t += grad_a.transpose() * jer;
      
      snapOpt_.gdC.block<8, 3>(i * 8, 0) += omg * step * gradViola_c;
      snapOpt_.gdT(i) += omg * (cost_inner / K_ + alpha * step * gradViola_t);
      cost += omg * step * cost_inner;

      s1 += step;
    }
    t_pre += snapOpt_.T1(i);
  }
}


void TrajOptConeFOV::addTimeIntPenalty(double& cost) {
  //xyz
  Eigen::Vector3d pos, vel, acc, jer, snp;
  Eigen::Vector3d grad_tmp, grad_tmp2, grad_p, grad_v, grad_a, grad_j;
  Eigen::Vector3d grad_total_p, grad_total_v, grad_total_a, grad_total_j;

  //yaw
  double yaw, dyaw, ddyaw, grad_yaw, grad_dyaw;
  double grad_tmp_yaw, grad_tmp_dyaw;
  double grad_total_yaw, grad_total_dyaw;

  //bodyrate
  Eigen::Vector4d quat, gradquat, grad_tmp_quat;
  Eigen::Vector3d omega, gradomega, grad_tmp_omega;

  //time
  double grad_t, grad_prev_t;

  double cost_tmp, cost_inner;
  Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4;
  Eigen::Matrix<double, 4, 1> ceta0, ceta1, ceta2;
  double s1, s2, s3, s4, s5, s6, s7;
  double step, alpha;
  Eigen::Matrix<double, 8, 3> gradViola_c1;
  Eigen::Matrix<double, 4, 1> gradViola_c2;
  double gradViola_t; 
  double omg;

  double t_pre = 0;

  int innerLoop;
  for (int i = 0; i < N_; ++i) {

    const auto& c1 = snapOpt_.b.block<8, 3>(i * 8, 0);
    const auto& c2 = accOpt_.b.block<4, 1>(i * 4, 0);
    step = snapOpt_.T1(i) / K_;
    s1 = 0.0;
    innerLoop = K_ + 1;

    const auto& hPoly = cfgHs_[i / 2];
    for (int j = 0; j < innerLoop; ++j) {

      alpha = 1.0 / K_ * j;
      s2 = s1 * s1;
      s3 = s2 * s1;
      s4 = s2 * s2;
      s5 = s4 * s1;
      s6 = s4 * s2;
      s7 = s4 * s3;

      beta0 << 1.0, s1, s2, s3, s4, s5, s6, s7;
      beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
      beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3, 30.0 * s4, 42.0 * s5;
      beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2, 120.0 * s3, 210.0 * s4;
      beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1, 360.0 * s2, 840.0 * s3;
      pos = c1.transpose() * beta0;
      vel = c1.transpose() * beta1;
      acc = c1.transpose() * beta2;
      jer = c1.transpose() * beta3;
      snp = c1.transpose() * beta4;

      ceta0 << 1.0, s1, s2, s3;
      ceta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2;
      ceta2 << 0.0, 0.0, 2.0, 6.0 * s1;
      yaw = c2.transpose() * ceta0;
      dyaw = c2.transpose() * ceta1;
      ddyaw = c2.transpose() * ceta2;
      
      flatmap_.forward(vel, acc, jer, yaw, dyaw, quat, omega);

      grad_p.setZero(); 
      grad_v.setZero(); 
      grad_a.setZero();
      grad_j.setZero(); 
      gradquat.setZero();  
      gradomega.setZero();
      grad_yaw = 0; grad_dyaw = 0;
      grad_t = 0; grad_prev_t = 0;
      cost_inner = 0;

      omg = (j == 0 || j == innerLoop - 1) ? 0.5 : 1.0;

      if (grad_cost_corridor(pos, hPoly, grad_tmp, cost_tmp)) {
        grad_p += grad_tmp;
        cost_inner += cost_tmp;
      }

      if (grad_cost_floor(pos, grad_tmp, cost_tmp)) {
        grad_p += grad_tmp;
        cost_inner += cost_tmp;
      }

      if (grad_cost_v(vel, grad_tmp, cost_tmp)) {
        grad_v += grad_tmp;
        cost_inner += cost_tmp;
      }

      if (grad_cost_a(acc, grad_tmp, cost_tmp)) {
        grad_a += grad_tmp;
        cost_inner += cost_tmp;
      }

      if (grad_cost_omega(omega, grad_tmp_omega, cost_tmp))
      {
        gradomega += grad_tmp_omega;
        cost_inner += cost_tmp;
      }

      flatmap_.backward(grad_p, grad_v, grad_a, gradquat, gradomega,
                        grad_total_p, grad_total_v, grad_total_a,
                        grad_total_j, grad_total_yaw, grad_total_dyaw);

      gradViola_c1 = beta0 * grad_total_p.transpose();
      gradViola_t = grad_total_p.transpose() * vel;
      gradViola_c1 += beta1 * grad_total_v.transpose();
      gradViola_t += grad_total_v.transpose() * acc;
      gradViola_c1 += beta2 * grad_total_a.transpose();
      gradViola_t += grad_total_a.transpose() * jer;
      gradViola_c1 += beta3 * grad_total_j.transpose();
      gradViola_t += grad_total_j.transpose() * snp;
      gradViola_c2 = ceta0 * grad_total_yaw;
      gradViola_t += grad_total_yaw * dyaw;
      gradViola_c2 += ceta1 * grad_total_dyaw;
      gradViola_t += grad_total_dyaw * ddyaw;

      if (grad_cost_swarm_collision(t_pre + step * j, pos, vel, grad_tmp, grad_t, 
                                    grad_prev_t, cost_tmp))
      {
        gradViola_c1 += beta0 * grad_tmp.transpose();
        gradViola_t += grad_t;
        cost_inner += cost_tmp;
      }
      
      snapOpt_.gdC.block<8, 3>(i * 8, 0) += omg * step * gradViola_c1;
      accOpt_.gdC.block<4, 1>(i * 4, 0) += omg * step * gradViola_c2;
    
      gradT_(i) += omg * (cost_inner / K_ + alpha * step * gradViola_t);
      if(i > 0) gradT_.head(i).array() += omg * step * grad_prev_t;
      cost += omg * step * cost_inner;
      s1 += step;
    }
    t_pre += snapOpt_.T1(i);
  }
}

void TrajOptConeFOV::addTimeCost_init(double& cost) {
  const auto& T = snapOpt_.T1;
  int piece = 0;
  int M = tracking_pts_.size() - 1; 
  double t = 0;
  double t_pre = 0;

  double step = tracking_dt_;
  Eigen::Matrix<double, 8, 1> beta0, beta1;
  Eigen::Matrix<double, 4, 1> ceta0, ceta1;
  double s1, s2, s3, s4, s5, s6, s7;

  //xyz
  Eigen::Vector3d pos, vel;
  Eigen::Vector3d grad_tmp, grad_p;

  //yaw
  double yaw, dyaw, grad_yaw;
  double grad_tmp_yaw;

  double cost_tmp, cost_inner;
  Eigen::Matrix<double, 8, 3> gradViola_c1;
  Eigen::Matrix<double, 4, 1> gradViola_c2;
  double gradViola_t;

  for (int i = 0; i < M; ++i) {
    double rho = exp2(-3.0 * i / M);
    while (t - t_pre > T(piece)) {
      t_pre += T(piece);
      piece++;
    }
    s1 = t - t_pre;
    s2 = s1 * s1;
    s3 = s2 * s1;
    s4 = s2 * s2;
    s5 = s4 * s1;
    s6 = s4 * s2;
    s7 = s4 * s3;
    beta0 << 1.0, s1, s2, s3, s4, s5, s6, s7;
    beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
    const auto& c1 = snapOpt_.b.block<8, 3>(piece * 8, 0);
    const auto& c2 = accOpt_.b.block<4, 1>(piece * 4, 0);
    pos = c1.transpose() * beta0;
    vel = c1.transpose() * beta1;

    ceta0 << 1.0, s1, s2, s3;
    ceta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2;
    yaw = c2.transpose() * ceta0;
    dyaw = c2.transpose() * ceta1;

    Eigen::Vector3d target_p = tracking_pts_[i];
    grad_p.setZero(); 
  
    grad_yaw = 0; 
    cost_inner = 0;

    if(i > 0)
    { 
      if (grad_cost_pt_tracking(pos, yaw, front_path_[i], init_yaw_path_[i], 
                                grad_tmp, grad_tmp_yaw, cost_tmp))
      {
        grad_p += grad_tmp;
        grad_yaw += grad_tmp_yaw;
        cost_inner += cost_tmp;
      }
    }
    gradViola_c1 = beta0 * grad_p.transpose();
    gradViola_t = grad_p.transpose() * vel;
    gradViola_c2 = ceta0 * grad_yaw;
    gradViola_t += grad_yaw * dyaw;

    snapOpt_.gdC.block<8, 3>(piece * 8, 0) += rho * step * gradViola_c1;
    accOpt_.gdC.block<4, 1>(piece * 4, 0) += rho * step * gradViola_c2;
    if(piece > 0){
      gradT_.head(piece).array() += -rho * step * gradViola_t;
    }
    cost += rho * step * cost_inner;
    
    t += step;
  }
}

void TrajOptConeFOV::addTimeCost(double& cost) {
  const auto& T = snapOpt_.T1;
  int piece = 0;
  int M = tracking_pts_.size() - 1; //originally set: .size() - 1
  double t = 0;
  double t_pre = 0;

  double step = tracking_dt_;
  Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4;
  Eigen::Matrix<double, 4, 1> ceta0, ceta1, ceta2;
  double s1, s2, s3, s4, s5, s6, s7;

  //xyz
  Eigen::Vector3d pos, vel, acc, jer, snp;
  Eigen::Vector3d grad_tmp, grad_tmp2, grad_p, grad_v, grad_a, grad_j;
  Eigen::Vector3d grad_total_p, grad_total_v, grad_total_a, grad_total_j;

  //yaw
  double yaw, dyaw, ddyaw, grad_yaw, grad_dyaw;
  double grad_tmp_yaw, grad_tmp_dyaw;
  double grad_total_yaw, grad_total_dyaw;

  //bodyrate
  Eigen::Vector4d quat, gradquat, grad_tmp_quat;
  Eigen::Vector3d omega, gradomega, grad_tmp_omega;

  double cost_tmp, cost_inner;
  Eigen::Matrix<double, 8, 3> gradViola_c1;
  Eigen::Matrix<double, 4, 1> gradViola_c2;
  double gradViola_t;

  for (int i = 0; i < M; ++i) {
    double rho = exp2(-3.0 * i / M);
    while (t - t_pre > T(piece)) {
      t_pre += T(piece);
      piece++;
    }
    s1 = t - t_pre;
    s2 = s1 * s1;
    s3 = s2 * s1;
    s4 = s2 * s2;
    s5 = s4 * s1;
    s6 = s4 * s2;
    s7 = s4 * s3;
    beta0 << 1.0, s1, s2, s3, s4, s5, s6, s7;
    beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
    beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3, 30.0 * s4, 42.0 * s5;
    beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2, 120.0 * s3, 210.0 * s4;
    beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1, 360.0 * s2, 840.0 * s3;
    const auto& c1 = snapOpt_.b.block<8, 3>(piece * 8, 0);
    const auto& c2 = accOpt_.b.block<4, 1>(piece * 4, 0);
    pos = c1.transpose() * beta0;
    vel = c1.transpose() * beta1;
    acc = c1.transpose() * beta2;
    jer = c1.transpose() * beta3;
    snp = c1.transpose() * beta4;

    ceta0 << 1.0, s1, s2, s3;
    ceta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2;
    ceta2 << 0.0, 0.0, 2.0, 6.0 * s1;
    yaw = c2.transpose() * ceta0;
    dyaw = c2.transpose() * ceta1;
    ddyaw = c2.transpose() * ceta2;

    Eigen::Vector3d target_p = tracking_pts_[i];

    flatmap_.forward(vel, acc, jer, yaw, dyaw, quat, omega);
    
    grad_p.setZero(); 
    grad_v.setZero(); 
    grad_a.setZero(); 
    grad_j.setZero(); 
    gradquat.setZero();  
    gradomega.setZero();
    grad_yaw = 0; grad_dyaw = 0;
    cost_inner = 0;

    if(i > 0)
    { 
      if (grad_cost_distance(pos, target_p, grad_tmp, cost_tmp)) {
        grad_p += grad_tmp;
        cost_inner += cost_tmp;
      }

      if(!teammate_list_->empty()) 
      {

        if(grad_cost_vsdf(pos, vsdf_list_->at(i), grad_tmp, cost_tmp))
        {
          grad_p += grad_tmp;
          cost_inner += cost_tmp;
        }

        if (grad_cost_swarm_bearing(t, pos, target_p, grad_tmp, cost_tmp)){
          grad_p += grad_tmp;
          cost_inner += cost_tmp;
        }

        if (grad_cost_direct_angular_sep(t, pos, target_p, grad_tmp, cost_tmp)){
          grad_p += grad_tmp;
          cost_inner += cost_tmp;
        }
      }
    }

    if(i >= idx_start_fov_cost_)
    {
      if (grad_cost_FOV(pos, quat, target_p, grad_tmp, grad_tmp_quat, cost_tmp))
      {
        grad_p += grad_tmp;
        gradquat += grad_tmp_quat;
        cost_inner += cost_tmp;
      }
    }

    flatmap_.backward(grad_p, grad_v, grad_a, gradquat, gradomega,
                      grad_total_p, grad_total_v, grad_total_a,
                      grad_total_j, grad_total_yaw, grad_total_dyaw);

    gradViola_c1 = beta0 * grad_total_p.transpose();
    gradViola_t = grad_total_p.transpose() * vel;
    gradViola_c1 += beta1 * grad_total_v.transpose();
    gradViola_t += grad_total_v.transpose() * acc;
    gradViola_c1 += beta2 * grad_total_a.transpose();
    gradViola_t += grad_total_a.transpose() * jer;
    gradViola_c1 += beta3 * grad_total_j.transpose();
    gradViola_t += grad_total_j.transpose() * snp;
    gradViola_c2 = ceta0 * grad_total_yaw;
    gradViola_t += grad_total_yaw * dyaw;
    gradViola_c2 += ceta1 * grad_total_dyaw;
    gradViola_t += grad_total_dyaw * ddyaw;

    snapOpt_.gdC.block<8, 3>(piece * 8, 0) += rho * step * gradViola_c1;
    accOpt_.gdC.block<4, 1>(piece * 4, 0) += rho * step * gradViola_c2;
    if(piece > 0){
      gradT_.head(piece).array() += -rho * step * gradViola_t;
    }
    cost += rho * step * cost_inner;
    
    t += step;
  }
}

void TrajOptConeFOV::addTailCost(double& cost){
  Eigen::Vector3d target_p = tracking_pts_.back();
  //xyz
  Eigen::Vector3d pos, vel, acc, jer;
  Eigen::Vector3d grad_tmp, grad_p, grad_v, grad_a, grad_j;
  Eigen::Vector3d grad_total_p, grad_total_v, grad_total_a, grad_total_j;

  //yaw
  double yaw, dyaw, grad_yaw, grad_dyaw;
  double grad_total_yaw, grad_total_dyaw;

  //bodyrate
  Eigen::Vector4d quat, gradquat, grad_tmp_quat;
  Eigen::Vector3d omega, gradomega, grad_tmp_omega;

  double cost_tmp;

  grad_p.setZero();
  grad_v.setZero(); 
  grad_a.setZero(); 
  grad_j.setZero();
  gradquat.setZero();  
  gradomega.setZero();
  grad_yaw = 0; grad_dyaw = 0;
  
  pos = finS_.col(0).head(3);
  vel = finS_.col(1).head(3);
  acc = finS_.col(2).head(3);
  jer = finS_.col(3).head(3);
  yaw = finS_(3, 0);
  dyaw = finS_(3, 1);
  
  flatmap_.forward(vel, acc, jer, yaw, dyaw, quat, omega);

  if (grad_cost_floor(pos, grad_tmp, cost_tmp)) {
    grad_p += grad_tmp;
    cost += cost_tmp;
  }

  if (grad_cost_distance(pos, target_p, grad_tmp, cost_tmp)) {
    grad_p += grad_tmp;
    cost += cost_tmp;
  }

  if (grad_cost_FOV(pos, quat, target_p, grad_tmp, grad_tmp_quat, cost_tmp))
  {
    grad_p += grad_tmp;
    gradquat += grad_tmp_quat;
    cost += cost_tmp;
  }

  if(!teammate_list_->empty()) 
  {
    if (grad_cost_swarm_bearing(sum_T_, pos, target_p, grad_tmp, cost_tmp)){
      grad_p += grad_tmp;
      cost += cost_tmp;
    }

    if (grad_cost_vsdf(pos, vsdf_list_->back(), grad_tmp, cost_tmp)){
      grad_p += grad_tmp;
      cost += cost_tmp;
    }

    if (grad_cost_direct_angular_sep(sum_T_, pos, target_p, grad_tmp, cost_tmp)){
      grad_p += grad_tmp;
      cost += cost_tmp;
    }
  }

  flatmap_.backward(grad_p, grad_v, grad_a, gradquat, gradomega,
                    grad_total_p, grad_total_v, grad_total_a,
                    grad_total_j, grad_total_yaw, grad_total_dyaw);

  snapOpt_.gdTail.col(0) += grad_total_p;
  snapOpt_.gdTail.col(1) += grad_total_v;
  snapOpt_.gdTail.col(2) += grad_total_a;
  snapOpt_.gdTail.col(3) += grad_total_j;
  accOpt_.gdTail(0) += grad_total_yaw;
  accOpt_.gdTail(1) += grad_total_dyaw;

}


void TrajOptConeFOV::addTailCost_init(double& cost){
  Eigen::Vector3d target_p = tracking_pts_.back();
  //xyz
  Eigen::Vector3d pos, vel, acc, jer;
  Eigen::Vector3d grad_tmp, grad_p, grad_v, grad_a, grad_j;

  //yaw
  double yaw, dyaw, grad_yaw, grad_dyaw, grad_tmp_yaw;
  double cost_tmp;

  grad_p.setZero();
  grad_v.setZero(); 
  grad_a.setZero(); 
  grad_j.setZero();
  grad_yaw = 0; grad_dyaw = 0;
  
  pos = finS_.col(0).head(3);
  vel = finS_.col(1).head(3);
  acc = finS_.col(2).head(3);
  jer = finS_.col(3).head(3);
  yaw = finS_(3, 0);
  dyaw = finS_(3, 1);
  
  if (grad_cost_pt_tracking(pos, yaw, front_path_.back(), init_yaw_path_.back(), 
                            grad_tmp, grad_tmp_yaw, cost_tmp))
  {
    grad_p += grad_tmp;
    grad_yaw += grad_tmp_yaw;
    cost += cost_tmp;
  }

  snapOpt_.gdTail.col(0) += grad_p;
  accOpt_.gdTail(0) += grad_yaw;
}

bool TrajOptConeFOV::grad_cost_pt_tracking(const Eigen::Vector3d& pos,
                                    const double& yaw,
                                    const Eigen::Vector3d& pos_des,
                                    const double& yaw_des, 
                                    Eigen::Vector3d& gradp,
                                    double& grad_yaw,
                                    double& cost)
{
  gradp = 2 * (pos - pos_des);
  grad_yaw = 2 * (yaw - yaw_des);
  cost = (gradp / 2).squaredNorm() + (grad_yaw / 2) * (grad_yaw / 2);
  
  gradp *= 1000;
  grad_yaw *= 1000;
  cost *= 1000;
  return true;
}


bool TrajOptConeFOV::checkTrajFOV(const Trajectory& traj, const Trajectory1D& traj_yaw, bool print_flg)
{
  double T_end = traj.getTotalDuration();
  int iter{0};
  bool is_vert_valid{true};
  bool is_hori_valid{true};

  for(double t = 0.0; t < T_end + tracking_dt_ / 2.0; t += tracking_dt_)
  {
    Eigen::Vector3d target_p = tracking_pts_[iter++];
    if(iter <= idx_start_fov_cost_ || iter >= tracking_pts_.size() + 1)
      continue;

    Eigen::Vector3d pos = t < T_end ? traj.getPos(t) : traj.getPos(T_end);
    Eigen::Vector3d acc = t < T_end ? traj.getAcc(t) : traj.getAcc(T_end);
    Eigen::Vector3d vel = t < T_end ? traj.getVel(t) : traj.getVel(T_end);
    Eigen::Vector3d jer = t < T_end ? traj.getJer(t) : traj.getJer(T_end);

    double psi = t < T_end ? traj_yaw.getPos(t) : traj_yaw.getPos(T_end);
    double dpsi = t < T_end ? traj_yaw.getVel(t) : traj_yaw.getVel(T_end);

    Eigen::Vector4d quat;
    Eigen::Vector3d omega;

    flatmap_.forward(vel, acc, jer, psi, dpsi, quat, omega);

    Eigen::Matrix3d R = q2R(quat);
    Eigen::Vector3d pe(0, 0, LI_extrinsic_);
    Eigen::Vector3d pb = R.transpose() * (target_p - pos) - pe;
    double px = pb(0), py = pb(1), pz = pb(2);
    double n2_pb = sqrt(px * px + py * py);
    double n_pb = sqrt(px * px + py * py + pz * pz); 
    double tanA = tan(fov_ctr_theta_);
    Eigen::Vector3d pc(px, py, tanA * n2_pb);

    double pen_vert = cos(fov_theta_) - pb.dot(pc) / n_pb / pc.norm();
    double pen_horz = (1 - pb(0) / n2_pb);
    double yaw_des = atan2(target_p.y() - pos.y(), target_p.x() - pos.x());

    double yaw_clip = std::fmod(yaw_des + M_PI, 2 * M_PI) - M_PI;
    yaw_des = yaw_clip < -M_PI ? yaw_clip + 2 * M_PI : yaw_clip;
    yaw_clip = std::fmod(psi + M_PI, 2 * M_PI) - M_PI;
    psi = yaw_clip < -M_PI ? yaw_clip + 2 * M_PI : yaw_clip;

    if(pen_vert > 0.004){
      if(print_flg){
        ROS_INFO("------------------------------");
        ROS_WARN_STREAM("TargetPos NO.(start from No.0): " << iter - 1);
        ROS_WARN_STREAM("cos_fov_theta: " << cos(fov_theta_));
        ROS_ERROR_STREAM("pen_vertical_FOV: " << pen_vert);
        ROS_INFO("------------------------------");
      }
      is_vert_valid = false;
    }

    if(pen_horz > 0.007){
      if(print_flg){
        ROS_INFO("------------------------------");
        ROS_WARN_STREAM("TargetPos NO.(start from No.0): " << iter - 1);
        ROS_WARN_STREAM("Yaw Angle(deg, clipped): " << psi / M_PI *180);
        ROS_WARN_STREAM("Desired Yaw Angle(deg, clipped): " << yaw_des / M_PI *180 );
        ROS_ERROR_STREAM("pen_horizontal_FOV: " << pen_horz);
        ROS_INFO("------------------------------");
      }
      is_hori_valid = false;
    }
  }

  if(is_hori_valid && is_vert_valid){
    return true;
  }else{
    rhoFOV_horizontal_ *= (is_hori_valid ? 1 : 3);
    rhoFOV_vertical_ *= (is_vert_valid ? 1 : 3);
    return false;
  }
}


bool TrajOptConeFOV::generate_stop_traj(const Eigen::Vector3d& start_pos, const double& start_yaw,
                                        const Eigen::Vector3d& stop_pos, const double& stop_yaw,
                                        Trajectory& traj, Trajectory1D& traj_yaw)
{
  minco::MINCO_S4 stopMSO;
  minco::MINCO_S2 yawMSO;
  auto ZERO = Eigen::Vector4d::Zero();
  Eigen::Matrix<double, 4, 4> headState, tailState;
  headState.col(0) << start_pos, start_yaw;
  headState.col(1) = ZERO;
  headState.col(2) = ZERO;
  headState.col(3) = ZERO;

  tailState = headState;
  tailState.col(0) << stop_pos, stop_yaw;

  stopMSO.reset(2);
  yawMSO.reset(2);
  stopMSO.generate(headState.topRows<3>(), tailState.topRows<3>(), 
                   tailState.col(0).head(3), Eigen::Vector2d(0.5, 0.5));
  yawMSO.generate(headState.bottomLeftCorner<1, 2>(), 
                  tailState.bottomLeftCorner<1, 2>(), 
                  tailState.col(0).tail(1), Eigen::Vector2d(0.5, 0.5));
  traj = stopMSO.getTraj();
  traj_yaw = yawMSO.getTraj();
  return true;
}


bool TrajOptConeFOV::generate_stop_traj(const Eigen::MatrixXd& initState, //4x4 PVAJ
                                        const double& duration, 
                                        const Eigen::Vector3d& stop_pos, const double& stop_yaw,
                                        Trajectory& traj, Trajectory1D& traj_yaw)
{
  minco::MINCO_S4 stopMSO;
  minco::MINCO_S2 yawMSO;
  auto ZERO = Eigen::Vector4d::Zero();
  Eigen::Matrix<double, 4, 4> tailState;
  tailState.col(0) << stop_pos, stop_yaw;
  tailState.col(1) = ZERO;
  tailState.col(2) = ZERO;
  tailState.col(3) = ZERO;
  
  stopMSO.reset(1);
  yawMSO.reset(1);
  Eigen::MatrixXd inPs; Eigen::VectorXd durations(1); durations << duration;
  stopMSO.generate(initState.topRows<3>(), tailState.topRows<3>(), 
                   inPs, durations);
  traj = stopMSO.getTraj();
  yawMSO.generate(initState.bottomLeftCorner<1, 2>(), 
                  tailState.bottomLeftCorner<1, 2>(), 
                  inPs, durations);
  traj_yaw = yawMSO.getTraj();
  return true;
}

}  // namespace traj_opt
