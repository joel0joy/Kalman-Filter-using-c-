#include <iostream>
#include <stdexcept>
#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter(
    double dt,
    const Eigen::MatrixXd& F,
    const Eigen::MatrixXd& H,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P)
  : F(F), H(H), Q(Q), R(R), P(P),
    m(H.rows()), n(F.rows()), dt(dt), initialized(false),
    I(n, n), X_hat(n), X_hat_new(n)
{
  I.setIdentity();
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init() {
  X_hat.setZero();
  P0 = P;
  t0 = 0;
  t = t0;
  initialized = true;
}

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) {
  X_hat = x0;
  P0 = P;
  this->t0 = t0;
  t = t0;
  initialized = true;
}

void KalmanFilter::update(const Eigen::VectorXd& z) {
  if(!initialized) {
    throw std::runtime_error("Filter is not initialized! Call init() first.");
  }

  // --- 1. PREDICT STEP ---
  // State extrapolation: x_hat^- = F * x_hat
  X_hat_new = F * X_hat;

  // Covariance extrapolation: P^- = F * P * F^T + Q
  P = F * P * F.transpose() + Q;

  // --- 2. UPDATE STEP ---
  // Kalman Gain: K = P^- * H^T * (H * P^- * H^T + R)^-1
  K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

  // State update: x_hat = x_hat^- + K * (z - H * x_hat^-)
  X_hat = X_hat_new + K * (z - H * X_hat_new);

  // Covariance update: P = (I - K * H) * P^-
  P = (I - K * H) * P;

  t += dt;
}

void KalmanFilter::update(const Eigen::VectorXd& z, double dt, const Eigen::MatrixXd& F) {
  this->F = F;
  this->dt = dt;
  update(z);
}