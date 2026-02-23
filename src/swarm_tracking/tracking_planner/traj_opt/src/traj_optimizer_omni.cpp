#include <optimizer_corridor/traj_optimizer.h>

namespace traj_opt {

//SECTION Helper Funcs

static Eigen::Matrix3d q2R(const Eigen::Vector4d& q)
{
  Eigen::Quaterniond Quat(q(0), q(1), q(2), q(3));
  return Quat.toRotationMatrix();
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
// !SECTION  Variables Transformation and Grad Prop

// SECTION object function
static inline double objectiveFunc(void* ptrObj,
                                   const double* x,
                                   double* grad,
                                   const int n) {
  TrajOptOmniFOV& obj = *(TrajOptOmniFOV*)ptrObj;

  Eigen::Map<const Eigen::VectorXd> t(x, obj.dim_t_);
  Eigen::Map<const Eigen::VectorXd> p(x + obj.dim_t_, obj.dim_p_);
  Eigen::Map<const Eigen::Vector3d> tailA(x + obj.dim_t_ + obj.dim_p_);
  Eigen::Map<Eigen::VectorXd> gradt(grad, obj.dim_t_);
  Eigen::Map<Eigen::VectorXd> gradp(grad + obj.dim_t_, obj.dim_p_);
  Eigen::Map<Eigen::Vector3d> gradtailA(grad + obj.dim_t_ + obj.dim_p_);

  Eigen::VectorXd T(obj.N_);
  Eigen::MatrixXd P(3, obj.N_);

  forwardT(t, obj.sum_T_, T);
  forwardP(p, obj.cfgVs_, P);

  obj.finS_.setZero();
  obj.finS_.col(0) = P.rightCols(1);
  obj.finS_.col(1) = obj.tailV_;
  obj.finS_.col(2) = tailA;

  obj.snapOpt_.generate(obj.iniS_, obj.finS_, P, T);
  obj.gradT_.setZero();

  double cost = obj.snapOpt_.getTrajSnapCost();
  obj.snapOpt_.calGrads_CT();
  obj.addTimeIntPenalty(cost);
  obj.addTimeCost(cost);
  obj.snapOpt_.calGrads_PT();
  obj.gradT_ += obj.snapOpt_.gdT;
  obj.addTailCost(cost); 
  addLayerTGrad(t, obj.sum_T_, obj.gradT_, gradt);
  Eigen::MatrixXd gradPInTail(3, obj.N_);
  gradPInTail << obj.snapOpt_.gdP, obj.snapOpt_.gdTail.col(0);
  addLayerPGrad(p, obj.cfgVs_, gradPInTail, gradp);
  Eigen::MatrixXd gradYawInTail(1, obj.N_);
  gradtailA = obj.snapOpt_.gdTail.col(2);

  return cost;
}

// SECTION object function
static inline double initObjectiveFunc(void* ptrObj,
                                       const double* x,
                                       double* grad,
                                       const int n) {
  TrajOpt& obj = *(TrajOpt*)ptrObj;

  Eigen::Map<const Eigen::VectorXd> t(x, obj.dim_t_);
  Eigen::Map<const Eigen::VectorXd> p(x + obj.dim_t_, obj.dim_p_);
  Eigen::Map<const Eigen::Vector3d> tailA(x + obj.dim_t_ + obj.dim_p_);
  Eigen::Map<Eigen::VectorXd> gradt(grad, obj.dim_t_);
  Eigen::Map<Eigen::VectorXd> gradp(grad + obj.dim_t_, obj.dim_p_);
  Eigen::Map<Eigen::Vector3d> gradtailA(grad + obj.dim_t_ + obj.dim_p_);

  Eigen::VectorXd T(obj.N_);
  Eigen::MatrixXd P(3, obj.N_);

  forwardT(t, obj.sum_T_, T);
  forwardP(p, obj.cfgVs_, P);

  obj.finS_.setZero();
  obj.finS_.col(0) = P.rightCols(1);
  obj.finS_.col(1) = obj.tailV_;
  obj.finS_.col(2) = tailA;

  obj.snapOpt_.generate(obj.iniS_, obj.finS_, P, T);
  obj.gradT_.setZero();

  double cost = obj.snapOpt_.getTrajSnapCost();
  obj.snapOpt_.calGrads_CT();
  obj.addTimeIntPenalty_init(cost);
  obj.addTimeCost_init(cost);
  obj.snapOpt_.calGrads_PT();
  obj.gradT_ += obj.snapOpt_.gdT;
  obj.addTailCost_init(cost); 
  addLayerTGrad(t, obj.sum_T_, obj.gradT_, gradt);
  Eigen::MatrixXd gradPInTail(3, obj.N_);
  gradPInTail << obj.snapOpt_.gdP, obj.snapOpt_.gdTail.col(0);
  addLayerPGrad(p, obj.cfgVs_, gradPInTail, gradp);
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

  Eigen::Map<const Eigen::VectorXd> t(x, obj.dim_t_);
  Eigen::Map<const Eigen::VectorXd> p(x + obj.dim_t_, obj.dim_p_);
  Eigen::Map<const Eigen::Vector3d> tailA(x + obj.dim_t_ + obj.dim_p_);
  Eigen::VectorXd T(obj.N_);
  Eigen::MatrixXd P(3, obj.N_);

  forwardT(t, obj.sum_T_, T);
  forwardP(p, obj.cfgVs_, P);

  obj.finS_.setZero();
  obj.finS_.col(0) = P.rightCols(1);
  obj.finS_.col(1) = obj.tailV_;
  obj.finS_.col(2) = tailA;

  obj.snapOpt_.generate(obj.iniS_, obj.finS_, P, T);
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

bool TrajOptOmniFOV::setInitValues(const Eigen::MatrixXd& iniState, const Eigen::MatrixXd& finState) {
  
  iniS_ = iniState;
  finS_ = finState;
  double tempNorm = iniS_.col(1).head(3).norm();
  iniS_.col(1).head(3) *= tempNorm > vmax_ ? (vmax_ / tempNorm) : 1.0;
  tempNorm = iniS_.col(2).head(3).norm();
  iniS_.col(2).head(3) *= tempNorm > amax_ ? (amax_ / tempNorm) : 1.0;
  tempNorm = finS_.col(1).head(3).norm();
  finS_.col(1).head(3) *= tempNorm > vmax_ ? (vmax_ / tempNorm) : 1.0;
  tempNorm = finS_.col(2).head(3).norm();
  finS_.col(2).head(3) *= tempNorm > amax_ ? (amax_ / tempNorm) : 1.0;

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

  tailV_ = finS_.col(1).head<3>();
  tailA_ = finS_.col(2).head<3>();
  snapOpt_.reset(N_);

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


int TrajOptOmniFOV::optimize_init(const double& delta) {
  // Setup for L-BFGS solver
  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
  lbfgs_params.mem_size = 16;
  lbfgs_params.past = 3;
  lbfgs_params.g_epsilon = 1e-10;
  lbfgs_params.min_step = 1e-32;
  lbfgs_params.delta = delta;
  lbfgs_params.line_search_type = 1;
  Eigen::Map<Eigen::VectorXd> t(x_, dim_t_);
  Eigen::Map<Eigen::VectorXd> p(x_ + dim_t_, dim_p_);
  Eigen::Map<Eigen::Vector3d> tailA(x_ + dim_t_ + dim_p_);
  t = t_;
  p = p_;
  tailA = tailA_;
  double minObjective;
 
  auto ret = lbfgs::lbfgs_optimize(dim_t_ + dim_p_ + 3, x_, &minObjective,
                                   &initObjectiveFunc, nullptr,
                                   &init_opt_feedback, this, &lbfgs_params);
  t_ = t;
  p_ = p;
  tailA_ = tailA;
  return ret;
}

int TrajOptOmniFOV::optimize(const double& delta) {
  // Setup for L-BFGS solver
  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
  lbfgs_params.mem_size = 32;
  lbfgs_params.past = 3;
  lbfgs_params.g_epsilon = 1e-10;
  lbfgs_params.min_step = 1e-32;
  lbfgs_params.delta = delta;
  lbfgs_params.line_search_type = 1;
  Eigen::Map<Eigen::VectorXd> t(x_ , dim_t_);
  Eigen::Map<Eigen::VectorXd> p(x_ + dim_t_, dim_p_);
  Eigen::Map<Eigen::Vector3d> tailA(x_ + dim_t_ + dim_p_);
  t = t_;
  p = p_;
  tailA = tailA_;
  double minObjective;

  auto ret = lbfgs::lbfgs_optimize(dim_t_ + dim_p_ + 3, x_, &minObjective,
                                   &objectiveFunc, nullptr,
                                   &opt_feedback, this, &lbfgs_params);
  t_ = t;
  p_ = p;
  tailA_ = tailA;
  return ret;
}


bool TrajOptOmniFOV::generate_init_traj(const double t_replan_start,
                                        const Eigen::MatrixXd& iniState,
                                        const Eigen::MatrixXd& finState,
                                        const std::vector<Eigen::Vector3d>& front_wpts,
                                        const std::vector<Eigen::MatrixXd>& hPolys,
                                        Trajectory& traj)
{  

  init_overhead_ = 0.0;
  init_opt_iternum_ = 0;
  init_opt_res_dis_.clear();
  is_kino_path_ = true; 
  int opt_state = OPT_STATE::VALID;
  t_replan_start_ = t_replan_start;
  cfgHs_ = hPolys;

  if (cfgHs_.size() == 1) {
    cfgHs_.push_back(cfgHs_[0]);
  }
  if (!extractVs(cfgHs_, cfgVs_)) { 
    // ROS_WARN("\033[32m[OPT] ExtractVs fail!\033[0m");
    opt_state = OPT_STATE::COD_ISS;
    return opt_state;
  }
  
  sum_T_ = 1.8;
  front_path_ = front_wpts;

  N_ = 2 * cfgHs_.size(); 
  dim_t_ = N_ - 1;
  dim_p_ = 0;
  cfgVs_.emplace_back(cfgVs_.back());
  for (const auto& cfgV : cfgVs_) {
    dim_p_ += cfgV.cols() - 1;
  }

  p_.resize(dim_p_);
  t_.resize(dim_t_);
  gradT_.resize(N_);
  x_ = new double[dim_t_ + dim_p_ + 3];
  
  bool init_res = setInitValues(iniState, finState);
  
  traj = snapOpt_.getTraj();
  if(!init_res)
  {
    opt_state = OPT_STATE::OPT_ISS;
    return false;
  }
  return true;
}

int TrajOptOmniFOV::generate_traj(const double t_replan_start,
                                  const double t_traj_start,
                                  const Eigen::MatrixXd& iniState, //3x4
                                  const Eigen::MatrixXd& finState,//3x4
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
    t_replan_start_ = t_replan_start;
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
    dim_t_ = N_ - 1;
    dim_p_ = 0;
    cfgVs_.emplace_back(cfgVs_.back());
    for (const auto& cfgV : cfgVs_) {
      dim_p_ += cfgV.cols() - 1;
    }

    p_.resize(dim_p_);
    t_.resize(dim_t_);
    gradT_.resize(N_);
    x_ = new double[dim_t_ + dim_p_ + 3];

    if(!setInitValues(iniState, finState))
    {
      opt_state = OPT_STATE::OPT_ISS;
      return opt_state;
    }

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
      Eigen::MatrixXd P(3, N_);//contains the end
      forwardT(t_, sum_T_, T);
      forwardP(p_, cfgVs_, P);
      
      finS_.setZero();
      finS_.col(0) = P.rightCols(1);
      finS_.col(1) = tailV_;
      finS_.col(2) = tailA_;
      snapOpt_.generate(iniS_, finS_, P, T);
      traj = snapOpt_.getTraj();
      se3_trial++;
    }while(rhoFOV_vertical_init_ > 0 && !checkTrajFOV(traj, false) && se3_trial < max_outloop_num_);
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

void TrajOptOmniFOV::addTimeIntPenalty_init(double& cost)
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


void TrajOptOmniFOV::addTimeIntPenalty(double& cost) {
  //xyz
  Eigen::Vector3d pos, vel, acc, jer, snp;
  Eigen::Vector3d grad_tmp, grad_tmp2, grad_p, grad_v, grad_a, grad_j;
  Eigen::Vector3d grad_total_p, grad_total_v, grad_total_a, grad_total_j;

  //bodyrate
  Eigen::Vector4d quat, gradquat, grad_tmp_quat;
  Eigen::Vector3d omega, gradomega, grad_tmp_omega;

  //time
  double grad_t, grad_prev_t;

  double cost_tmp, cost_inner;
  Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4;
  double s1, s2, s3, s4, s5, s6, s7;
  double step, alpha;
  double grad_total_yaw, grad_total_dyaw;
  Eigen::Matrix<double, 8, 3> gradViola_c1;
  double gradViola_t; 
  double omg;

  double t_pre = 0;

  int innerLoop;
  for (int i = 0; i < N_; ++i) {

    const auto& c1 = snapOpt_.b.block<8, 3>(i * 8, 0);
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
      
      flatmap_.forward(vel, acc, jer, 0.0, 0.0, quat, omega);

      grad_p.setZero(); 
      grad_v.setZero(); 
      grad_a.setZero();
      grad_j.setZero(); 
      gradquat.setZero();  
      gradomega.setZero();
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


      if (grad_cost_swarm_collision(t_pre + step * j, pos, vel, grad_tmp, grad_t, 
                                    grad_prev_t, cost_tmp))
      {
        gradViola_c1 += beta0 * grad_tmp.transpose();
        gradViola_t += grad_t;
        cost_inner += cost_tmp;
      }
      
      snapOpt_.gdC.block<8, 3>(i * 8, 0) += omg * step * gradViola_c1;
      gradT_(i) += omg * (cost_inner / K_ + alpha * step * gradViola_t);
      if(i > 0) gradT_.head(i).array() += omg * step * grad_prev_t;
      cost += omg * step * cost_inner;
      s1 += step;
    }
    t_pre += snapOpt_.T1(i);
  }
}

void TrajOptOmniFOV::addTimeCost_init(double& cost) {
  const auto& T = snapOpt_.T1;
  int piece = 0;
  int M = tracking_pts_.size() - 1;
  double t = 0;
  double t_pre = 0;

  double step = tracking_dt_;
  Eigen::Matrix<double, 8, 1> beta0, beta1;
  double s1, s2, s3, s4, s5, s6, s7;

  //xyz
  Eigen::Vector3d pos, vel;
  Eigen::Vector3d grad_tmp, grad_p;

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
    pos = c1.transpose() * beta0;
    vel = c1.transpose() * beta1;

    Eigen::Vector3d target_p = tracking_pts_[i];
    grad_p.setZero(); 
    cost_inner = 0;

    if(i > 0)
    { 
      if (grad_cost_pt_tracking(pos, front_path_[i], 
                                grad_tmp, cost_tmp))
      {
        grad_p += grad_tmp;
        cost_inner += cost_tmp;
      }
    }

    gradViola_c1 = beta0 * grad_p.transpose();
    gradViola_t = grad_p.transpose() * vel;

    snapOpt_.gdC.block<8, 3>(piece * 8, 0) += rho * step * gradViola_c1;
    if(piece > 0){
      gradT_.head(piece).array() += -rho * step * gradViola_t;
    }
    cost += rho * step * cost_inner;
    t += step;
  }
}

void TrajOptOmniFOV::addTimeCost(double& cost) {
  const auto& T = snapOpt_.T1;
  int piece = 0;
  int M = tracking_pts_.size() - 1; 
  double t = 0;
  double t_pre = 0;

  double step = tracking_dt_;
  Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4;
  double s1, s2, s3, s4, s5, s6, s7;

  //xyz
  Eigen::Vector3d pos, vel, acc, jer, snp;
  Eigen::Vector3d grad_tmp, grad_tmp2, grad_p, grad_v, grad_a, grad_j;
  Eigen::Vector3d grad_total_p, grad_total_v, grad_total_a, grad_total_j;
  double grad_total_yaw, grad_total_dyaw;

  //bodyrate
  Eigen::Vector4d quat, gradquat, grad_tmp_quat;
  Eigen::Vector3d omega, gradomega, grad_tmp_omega;

  double cost_tmp, cost_inner;
  Eigen::Matrix<double, 8, 3> gradViola_c1;
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
    pos = c1.transpose() * beta0;
    vel = c1.transpose() * beta1;
    acc = c1.transpose() * beta2;
    jer = c1.transpose() * beta3;
    snp = c1.transpose() * beta4;

    Eigen::Vector3d target_p = tracking_pts_[i];

    flatmap_.forward(vel, acc, jer, 0.0, 0.0, quat, omega);
    
    grad_p.setZero(); 
    grad_v.setZero(); 
    grad_a.setZero(); 
    grad_j.setZero(); 
    gradquat.setZero();  
    gradomega.setZero();
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

    snapOpt_.gdC.block<8, 3>(piece * 8, 0) += rho * step * gradViola_c1;
    if(piece > 0){
      gradT_.head(piece).array() += -rho * step * gradViola_t;
    }
    cost += rho * step * cost_inner;
    
    t += step;
  }
}

void TrajOptOmniFOV::addTailCost(double& cost){
  Eigen::Vector3d target_p = tracking_pts_.back();
  //xyz
  Eigen::Vector3d pos, vel, acc, jer;
  Eigen::Vector3d grad_tmp, grad_p, grad_v, grad_a, grad_j;
  Eigen::Vector3d grad_total_p, grad_total_v, grad_total_a, grad_total_j;
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

  pos = finS_.col(0);
  vel = finS_.col(1);
  acc = finS_.col(2);
  jer = finS_.col(3);

  flatmap_.forward(vel, acc, jer, 0.0, 0.0, quat, omega);

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

    if (grad_cost_direct_angular_sep(sum_T_, pos, target_p, grad_tmp, cost_tmp)){
      grad_p += grad_tmp;
      cost += cost_tmp;
    }

    if (grad_cost_vsdf(pos, vsdf_list_->back(), grad_tmp, cost_tmp)){
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
}


void TrajOptOmniFOV::addTailCost_init(double& cost){
  Eigen::Vector3d target_p = tracking_pts_.back();
  //xyz
  Eigen::Vector3d pos;
  Eigen::Vector3d grad_tmp, grad_p;
  double cost_tmp;

  grad_p.setZero();
  pos = finS_.col(0);
  if (grad_cost_pt_tracking(pos, front_path_.back(), 
                            grad_tmp, cost_tmp))
  {
    grad_p += grad_tmp;
    cost += cost_tmp;
  }
  snapOpt_.gdTail.col(0) += grad_p;
}


bool TrajOptOmniFOV::grad_cost_pt_tracking(const Eigen::Vector3d& pos,                       
                                           const Eigen::Vector3d& pos_des,                     
                                           Eigen::Vector3d& gradp,
                                           double& cost)
{
  gradp = 2 * (pos - pos_des);
  cost = (gradp / 2).squaredNorm();
  gradp *= 1000;
  cost *= 1000;
  return true;
}

bool TrajOptOmniFOV::checkTrajFOV(const Trajectory& traj, bool print_flg)
{
  double T_end = traj.getTotalDuration();
  int iter{0};

  for(double t = 0.0; t < T_end + tracking_dt_ / 2.0; t += tracking_dt_)
  {
    Eigen::Vector3d target_p = tracking_pts_[iter++];
    if(iter <= idx_start_fov_cost_ || iter >= tracking_pts_.size() + 1)
      continue;

    Eigen::Vector3d pos = t < T_end ? traj.getPos(t) : traj.getPos(T_end);
    Eigen::Vector3d acc = t < T_end ? traj.getAcc(t) : traj.getAcc(T_end);
    Eigen::Vector3d vel = t < T_end ? traj.getVel(t) : traj.getVel(T_end);
    Eigen::Vector3d jer = t < T_end ? traj.getJer(t) : traj.getJer(T_end);

    Eigen::Vector4d quat;
    Eigen::Vector3d omega;

    flatmap_.forward(vel, acc, jer, 0.0, 0.0, quat, omega);

    Eigen::Matrix3d R = q2R(quat);
    Eigen::Vector3d pe(0, 0, LI_extrinsic_);
    Eigen::Vector3d pb = R.transpose() * (target_p - pos) - pe;
    double px = pb(0), py = pb(1), pz = pb(2);
    double n2_pb = sqrt(px * px + py * py);
    double n_pb = sqrt(px * px + py * py + pz * pz); 
    double tanA = tan(fov_ctr_theta_);
    Eigen::Vector3d pc(px, py, tanA * n2_pb);

    double pen_vert = cos(fov_theta_) - pb.dot(pc) / n_pb / pc.norm();

    if(pen_vert > 0.004){
      if(print_flg){
        ROS_INFO("------------------------------");
        ROS_WARN_STREAM("TargetPos NO.(start from No.0): " << iter - 1);
        ROS_WARN_STREAM("cos_fov_theta: " << cos(fov_theta_));
        ROS_ERROR_STREAM("pen_vertical_FOV: " << pen_vert);
        ROS_INFO("------------------------------");
      }
      rhoFOV_vertical_ *= 3;
      return false;
    }
    
  }
  return true;
}


bool TrajOptOmniFOV::generate_stop_traj(const Eigen::Vector3d& start_pos, const double& start_yaw,
                                        const Eigen::Vector3d& stop_pos, const double& stop_yaw,
                                        Trajectory& traj, Trajectory1D& traj_yaw)
{
  minco::MINCO_S4 stopMSO;
  auto ZERO = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 3, 4> headState, tailState;
  headState << start_pos, ZERO, ZERO, ZERO;
  tailState << stop_pos, ZERO, ZERO, ZERO;

  stopMSO.reset(2);
  stopMSO.generate(headState, tailState, 
                   stop_pos, Eigen::Vector2d(0.5, 0.5));
  traj = stopMSO.getTraj();
  return true;
}


bool TrajOptOmniFOV::generate_stop_traj(const Eigen::MatrixXd& initState, 
                                        const double& duration, 
                                        const Eigen::Vector3d& stop_pos, const double& stop_yaw,
                                        Trajectory& traj, Trajectory1D& traj_yaw)
{
  minco::MINCO_S4 stopMSO;
  auto ZERO = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 3, 4> headState = initState.block(0, 0, 3, 4);
  Eigen::Matrix<double, 3, 4> tailState;
  tailState << stop_pos, ZERO, ZERO, ZERO;
  
  stopMSO.reset(1);
  Eigen::MatrixXd inPs; Eigen::VectorXd durations(1); durations << duration;
  stopMSO.generate(headState, tailState, 
                   inPs, durations);
  traj = stopMSO.getTraj();
  return true;
}

}  // namespace traj_opt
