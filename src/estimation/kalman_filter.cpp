#include "estimation/kalman_filter.h"

KalmanFilter::KalmanFilter(
    const Eigen::MatrixXd& _A, // System dynamics.
    const Eigen::MatrixXd& _B, // Control.
    const Eigen::MatrixXd& _C, // Output.
    const Eigen::MatrixXd& _P, // Estimate error covariance.
    const Eigen::MatrixXd& _R, // Measurement noise covariance.
    const Eigen::MatrixXd& _Q  // Process noise covariance.
)
: A(_A)
, B(_B)
, C(_C)
, P(_P)
, R(_R)
, Q(_Q)
, P0(_P)
, K(_A.rows(), _C.rows())
, x_hat(_A.rows())
, x_tmp_(_A.rows())
, AP_(_A.rows(), _A.rows())
, PCt_(_A.rows(), _C.rows())
, S_(_C.rows(), _C.rows())
, Kt_(_C.rows(), _A.rows())
, CP_(_C.rows(), _A.rows())
, innovation_(_C.rows())
, llt_(_C.rows())
{
  x_hat.setZero();
}

KalmanFilter::~KalmanFilter() {

}

void KalmanFilter::init() {

  P = P0;
  x_hat.setZero();
}

void KalmanFilter::init(const Eigen::VectorXd& x) {

  init();
  x_hat = x;
}

void KalmanFilter::predict(const Eigen::Ref<const Eigen::VectorXd>& u) {

  // x = A x + B u (through x_tmp_: x_hat appears on both sides)
  x_tmp_.noalias() = A * x_hat;
  x_tmp_.noalias() += B * u;
  x_hat = x_tmp_;

  // P = A P A^T + Q
  AP_.noalias() = A * P;
  P.noalias() = AP_ * A.transpose();
  P += Q;
}

void KalmanFilter::update(const Eigen::Ref<const Eigen::VectorXd>& y) {

  // innovation covariance S = C P C^T + R
  PCt_.noalias() = P * C.transpose();
  S_.noalias() = C * PCt_;
  S_ += R;

  // gain K = P C^T S^-1, solved as S K^T = (P C^T)^T (Cholesky, S is symmetric positive definite)
  llt_.compute(S_);
  Kt_ = PCt_.transpose();
  llt_.solveInPlace(Kt_);
  K = Kt_.transpose();

  // x += K (y - C x)
  innovation_ = y;
  innovation_.noalias() -= C * x_hat;
  x_hat.noalias() += K * innovation_;

  // P = (I - K C) P = P - K (C P), with C P = (P C^T)^T (P symmetric)
  CP_ = PCt_.transpose();
  P.noalias() -= K * CP_;

  // keep P symmetric against round-off: P = (P + P^T) / 2
  AP_ = P.transpose();
  P += AP_;
  P *= 0.5;
}

const Eigen::VectorXd& KalmanFilter::get_state() const {
  return x_hat;
}

void KalmanFilter::set_Q(const Eigen::MatrixXd& _Q) {
  Q = _Q;
}

void KalmanFilter::set_R(const Eigen::MatrixXd& _R) {
  R = _R;
}

void KalmanFilter::set_state(const Eigen::VectorXd& x) {
  x_hat = x;
}

const Eigen::MatrixXd& KalmanFilter::get_P() const {
  return P;
}
