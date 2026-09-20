#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "KalmanFilter.hpp"

int main(int argc, char* argv[]) {

  int n = 3; // States: [position, velocity, acceleration]
  int m = 1; // Measurements: [position]

  double dt = 1.0 / 30; // Time interval (30 Hz)

  // Declare standard Kalman matrices
  Eigen::MatrixXd F(n, n); // State transition matrix
  Eigen::MatrixXd H(m, n); // Observation matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  // Constant acceleration projectile motion model
  F << 1, dt,  0,
       0,  1, dt,
       0,  0,  1;

  // Measure position only: z = H * x = [1, 0, 0] * [pos, vel, acc]^T
  H << 1, 0, 0;

  // System covariances
  Q << .05, .05, .0,
       .05, .05, .0,
        .0,  .0, .0;

  R << 5; // Sensor variance

  P << .1,    .1,   .1,
       .1, 10000,   10,
       .1,    10,  100;

  // Instantiate filter with standard variables
  KalmanFilter kf(dt, F, H, Q, R, P);

  // Noisy sensor measurements of a falling projectile (z)
  std::vector<double> measurements = {
      1.04202710058, 1.10726790452, 1.2913511148,  1.48485250951, 1.72825901034,
      1.74216489744, 2.11672039768, 2.14529225112, 2.16029641405, 2.21269371128,
      2.57709350237, 2.6682215744,  2.51641839428, 2.76034056782, 2.88131780617,
      2.88373786518, 2.9448468727,  2.82866600131, 3.0006601946,  3.12920591669,
      2.858361783,   2.83808170354, 2.68975330958, 2.66533185589, 2.81613499531,
      2.81003612051, 2.88321849354, 2.69789264832, 2.4342229249,  2.23464791825,
      2.30278776224, 2.02069770395, 1.94393985809, 1.82498398739, 1.52526230354,
      1.86967808173, 1.18073207847, 1.10729605087, 0.916168349913,0.678547664519,
      0.562381751596,0.355468474885,-0.155607486619,-0.287198661013,-0.602973173813
  };

  // Initial state vector x0: [pos, vel, acc]
  Eigen::VectorXd x0(n);
  double t0 = 0.0;
  x0 << measurements[0], 0.0, -9.81;
  kf.init(t0, x0);

  // Sensor reading vector z
  Eigen::VectorXd z(m);

  std::cout << "t = " << t0 << ", x_hat[0]: " << kf.state().transpose() << std::endl;

  for(size_t i = 0; i < measurements.size(); i++) {
    z << measurements[i];
    kf.update(z);

    std::cout << "t = " << kf.time()
              << ", z[" << i << "] = " << z.transpose()
              << ", x_hat[" << i << "] = " << kf.state().transpose()
              << std::endl;
  }

  return 0;
}