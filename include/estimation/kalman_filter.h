#pragma once

#include <Eigen/Dense>

// Linear Kalman filter. Sizes are fixed at construction: predict() / update() work in buffers
// allocated there, so they never allocate heap memory (real-time loop).
class KalmanFilter {

public:
  KalmanFilter(
    const Eigen::MatrixXd& A, // System dynamics.
    const Eigen::MatrixXd& B, // Control.
    const Eigen::MatrixXd& C, // Output.
    const Eigen::MatrixXd& P, // Estimate error covariance.
    const Eigen::MatrixXd& R, // Measurement noise covariance.
    const Eigen::MatrixXd& Q  // Process noise covariance.
    );
  ~KalmanFilter();

  void init();
  void init(const Eigen::VectorXd& x);
  // u, y: any Eigen vector of the right size (e.g. a Vector3d), taken without a copy
  void predict(const Eigen::Ref<const Eigen::VectorXd>& u);
  void update(const Eigen::Ref<const Eigen::VectorXd>& y);
  const Eigen::VectorXd& get_state() const;

  // Time-varying noise (e.g. scaled by contact state every step); same size as at construction
  void set_Q(const Eigen::MatrixXd& Q);
  void set_R(const Eigen::MatrixXd& R);
  // Overwrite the estimate without touching P (init() resets P)
  void set_state(const Eigen::VectorXd& x);
  const Eigen::MatrixXd& get_P() const;

private:
  Eigen::MatrixXd A, B, C, P, R, Q;
  Eigen::MatrixXd P0;   // initial P.
  Eigen::MatrixXd K;    // kalman gain.

  Eigen::VectorXd x_hat; // estimated state.

  // work buffers of predict() / update(), sized in the constructor (n states, m measurements)
  Eigen::VectorXd x_tmp_;        // n
  Eigen::MatrixXd AP_;           // n x n: A P, then P^T for the symmetrization
  Eigen::MatrixXd PCt_;          // n x m: P C^T
  Eigen::MatrixXd S_;            // m x m: innovation covariance C P C^T + R
  Eigen::MatrixXd Kt_;           // m x n: K^T = S^-1 (P C^T)^T
  Eigen::MatrixXd CP_;           // m x n: C P
  Eigen::VectorXd innovation_;   // m: y - C x
  Eigen::LLT<Eigen::MatrixXd> llt_; // Cholesky of S (symmetric positive definite)
};
