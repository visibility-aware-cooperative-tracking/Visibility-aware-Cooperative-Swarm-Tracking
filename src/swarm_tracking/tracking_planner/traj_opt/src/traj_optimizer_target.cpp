#include <optimizer_corridor/traj_optimizer.h>
//Traj optimizer for target drone

namespace traj_opt {
  // SECTION  variables transformation and gradient transmission
static double expC2(double t) {
  return t > 0.0 ? ((0.5 * t + 1.0) * t + 1.0)
                 : 1.0 / ((0.5 * t - 1.0) * t + 1.0);
}
static double logC2(double T) {
  return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
}
static void forwardT(const Eigen::Ref<const Eigen::VectorXd>& t, Eigen::Ref<Eigen::VectorXd> vecT) {
  int M = t.size();
  for (int i = 0; i < M; ++i) {
    vecT(i) = expC2(t(i));
  }
  return;
}
static void backwardT(const Eigen::Ref<const Eigen::VectorXd>& vecT, Eigen::Ref<Eigen::VectorXd> t) {
  int M = vecT.size();
  for (int i = 0; i < M; ++i) {
    t(i) = logC2(vecT(i));
  }
  return;
}
static inline double gdT2t(double t) {
  if (t > 0) {
    return t + 1.0;
  } else {
    double denSqrt = (0.5 * t - 1.0) * t + 1.0;
    return (1.0 - t) / (denSqrt * denSqrt);
  }
}
static void addLayerTGrad(const Eigen::Ref<const Eigen::VectorXd>& t,
                          const Eigen::Ref<const Eigen::VectorXd>& gradT,
                          Eigen::Ref<Eigen::VectorXd> gradt) {
  int M = t.size();
  for (int i = 0; i < M; ++i) {
    gradt(i) = gradT(i) * gdT2t(t(i));
  }
  return;
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
    gdr = cfgPolyVs[i].rightCols(k).transpose() * gradInPs.col(i);
    gdr = gdr.array() * r.array() * 2.0;

    grad.segment(j, k) = gdr * 2.0 / qnsqrp1 -
                         q * 4.0 * gdr.dot(q) / qnsqrp1sqr;
    j += k;
  }
  return;
}
// !SECTION variables transformation and gradient transmission

// SECTION object function
static inline double objectiveFunc(void* ptrObj,
                                   const double* x,
                                   double* grad,
                                   const int n) {
  TrajOptOmniFOV& obj = *(TrajOptOmniFOV*)ptrObj;

  Eigen::Map<const Eigen::VectorXd> t(x, obj.dim_t_);
  Eigen::Map<const Eigen::VectorXd> p(x + obj.dim_t_, obj.dim_p_);
  Eigen::Map<Eigen::VectorXd> gradt(grad, obj.dim_t_);
  Eigen::Map<Eigen::VectorXd> gradp(grad + obj.dim_t_, obj.dim_p_);

  Eigen::VectorXd T(obj.N_);
  Eigen::MatrixXd P(3, obj.N_ - 1);
  forwardT(t, T);
  forwardP(p, obj.cfgVs_, P);

  obj.snapOpt_.generate(obj.iniS_, obj.finS_, P, T);
  double cost = obj.snapOpt_.getTrajSnapCost();
  obj.snapOpt_.calGrads_CT();
  obj.addTargetPenalty(cost);
  obj.snapOpt_.calGrads_PT();
  obj.snapOpt_.gdT.array() += obj.rhoT_;
  cost += obj.rhoT_ * T.sum();
  addLayerTGrad(t, obj.snapOpt_.gdT, gradt);
  addLayerPGrad(p, obj.cfgVs_, obj.snapOpt_.gdP, gradp);

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
  return k > 1e3;
}

void TrajOptOmniFOV::addTargetPenalty(double& cost)
{
  Eigen::Vector3d pos, vel, acc, jer, snp;
  Eigen::Vector3d grad_tmp, grad_tmp2, grad_p, grad_v, grad_a, grad_j;
  double cost_tmp, cost_inner;
  Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4;
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
      beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1, 360.0 * s2, 840.0 * s3;
      alpha = 1.0 / K_ * j;
      pos = c.transpose() * beta0;
      vel = c.transpose() * beta1;
      acc = c.transpose() * beta2;
      jer = c.transpose() * beta3;
      snp = c.transpose() * beta4;

      grad_p.setZero();
      grad_v.setZero();
      grad_a.setZero();
      grad_j.setZero();
      cost_inner = 0.0;

      omg = (j == 0 || j == innerLoop - 1) ? 0.5 : 1.0;

      if (grad_cost_corridor(pos, hPoly, grad_tmp, cost_tmp)) {
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
      gradViola_c += beta3 * grad_j.transpose();
      gradViola_t += grad_j.transpose() * snp;
      
      snapOpt_.gdC.block<8, 3>(i * 8, 0) += omg * step * gradViola_c;
      snapOpt_.gdT(i) += omg * (cost_inner / K_ + alpha * step * gradViola_t);
      cost += omg * step * cost_inner;

      s1 += step;
    }
    t_pre += snapOpt_.T1(i);
  }
}

bool TrajOptOmniFOV::generate_traj(const Eigen::MatrixXd& iniState, //3x4
                                   const Eigen::MatrixXd& finState, //3x4
                                   const std::vector<Eigen::MatrixXd>& hPolys,
                                   Trajectory& traj) {

  cfgHs_ = hPolys;
  if (cfgHs_.size() == 1) {
    cfgHs_.push_back(cfgHs_[0]);
  }
  if (!extractVs(cfgHs_, cfgVs_)) {
    // ROS_ERROR("extractVs fail!");
    return false;
  }
  N_ = 2 * cfgHs_.size();

  // NOTE: one corridor two pieces
  dim_t_ = N_;
  dim_p_ = 0;
  for (const auto& cfgV : cfgVs_) {
    dim_p_ += cfgV.cols() - 1;
  }
  // std::cout << "dim_p_: " << dim_p_ << std::endl;
  p_.resize(dim_p_);
  t_.resize(dim_t_);
  x_ = new double[dim_p_ + dim_t_];
  Eigen::VectorXd T(N_);
  Eigen::MatrixXd P(3, N_ - 1);

  // NOTE set boundary conditions
  iniS_ = iniState;
  finS_ = finState;
  double tempNorm = iniS_.col(1).norm();
  iniS_.col(1) *= tempNorm > vmax_ ? (vmax_ / tempNorm) : 1.0;
  tempNorm = finS_.col(1).norm();
  finS_.col(1) *= tempNorm > vmax_ ? (vmax_ / tempNorm) : 1.0;
  tempNorm = iniS_.col(2).norm();
  iniS_.col(2) *= tempNorm > amax_ ? (amax_ / tempNorm) : 1.0;
  tempNorm = finS_.col(2).norm();
  finS_.col(2) *= tempNorm > amax_ ? (amax_ / tempNorm) : 1.0;

  T.setConstant((finState.col(0) - iniState.col(0)).norm() / vmax_ / N_);
  backwardT(T, t_);
  for (int i = 0; i < N_ - 1; ++i) {
    int k = cfgVs_[i].cols() - 1;
    P.col(i) = cfgVs_[i].rightCols(k).rowwise().sum() / (1.0 + k) + cfgVs_[i].col(0);
  }
  backwardP(P, cfgVs_, p_);
  snapOpt_.reset(N_);

  // NOTE optimization
  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
  lbfgs_params.mem_size = 16;
  lbfgs_params.past = 3;
  lbfgs_params.g_epsilon = 1e-10;
  lbfgs_params.min_step = 1e-32;
  lbfgs_params.delta = 1e-4;
  Eigen::Map<Eigen::VectorXd> t(x_, dim_t_);
  Eigen::Map<Eigen::VectorXd> p(x_ + dim_t_, dim_p_);
  t = t_;
  p = p_;
  double minObjective;
  auto opt_ret = lbfgs::lbfgs_optimize(dim_t_ + dim_p_, x_, &minObjective,
                                       &objectiveFunc, nullptr,
                                       &opt_feedback, this, &lbfgs_params);

  t_ = t;
  p_ = p;
  if (opt_ret < 0) {
    return false;
  }
  forwardT(t_, T);
  forwardP(p_, cfgVs_, P);
  snapOpt_.generate(iniS_, finS_, P, T);
  traj = snapOpt_.getTraj();
  delete[] x_;
  return true;
}

// neglect cone-fov target
void TrajOptConeFOV::addTargetPenalty(double& cost) {}
bool TrajOptConeFOV::generate_traj(const Eigen::MatrixXd& iniState, //3x4
                                   const Eigen::MatrixXd& finState, //3x4
                                   const std::vector<Eigen::MatrixXd>& hPolys,
                                   Trajectory& traj) {
                                    return false;
                                   }

}  // namespace traj_opt
