#include "extended_kalman_filter.h"

namespace kf {

ExtendedKalmanFilter::ExtendedKalmanFilter() : KalmanFilter() {
  timestamp_m = 0.0;
  is_first_update_m = true;
}

ExtendedKalmanFilter::~ExtendedKalmanFilter() {}

void ExtendedKalmanFilter::initialize(double timestamp, const Eigen::VectorXd &x,
                     const Eigen::MatrixXd &P, const Eigen::VectorXd &u,
                     const Eigen::MatrixXd &R) {
  timestamp_m = timestamp;
  KalmanFilter::initialize(x, P, u, Eigen::MatrixXd::Zero(x.rows(), x.cols()),
                           Eigen::MatrixXd::Zero(x.rows(), x.cols()),
                           Eigen::MatrixXd::Zero(x.rows(), R.cols()), R);
}

void ExtendedKalmanFilter::predict(double timestamp) {
  x_m = g(timestamp, x_m, u_m);

  updateF(timestamp);
  updateQ(timestamp);
  P_m = F_m * P_m * F_m.transpose() + Q_m;
}

void ExtendedKalmanFilter::update(double timestamp, const Eigen::VectorXd &z) {
  updateH();
  timestamp_m = timestamp;

  Eigen::VectorXd y = z - h(x_m, z);
  Eigen::MatrixXd S = H_m * P_m * H_m.transpose() + R_m;
  Eigen::MatrixXd K = P_m * H_m.transpose() * S.inverse();
  x_m = x_m + K * y;
  P_m = (I_m - K * H_m) * P_m;
}

void ExtendedKalmanFilter::reset(double timestamp, const Eigen::VectorXd &x,
                const Eigen::MatrixXd &P) {
  timestamp_m = timestamp;
  x_m = x;
  P_m = P;
}

}  // namespace kf
