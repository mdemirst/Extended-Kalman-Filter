#include "ekf_lidar.h"

namespace kf {

EKFLidar::EKFLidar() {}

Eigen::VectorXd EKFLidar::g(double timestamp, const Eigen::VectorXd &x,
                            const Eigen::VectorXd &u) {
  double dt = timestamp - timestamp_m;

  return Eigen::Vector4d(x[0] + dt * x[2], x[1] + dt * x[3], x[2], x[3]) + u;
}

void EKFLidar::updateF(double timestamp) {
  double dt = timestamp - timestamp_m;

  F_m = (Eigen::Matrix4d() << 1.0, 0.0, dt, 0.0, 0.0, 1.0, 0.0, dt, 0.0, 0.0,
         1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
            .finished();
}

void EKFLidar::updateQ(double timestamp) {
  double noise_ax = 9.0;
  double noise_ay = 9.0;
  double dt = timestamp - timestamp_m;
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  Eigen::MatrixXd Q(4, 4);

  Q(0, 0) = (dt_4 / 4) * noise_ax;
  Q(0, 1) = 0.0;
  Q(0, 2) = (dt_3 / 2) * noise_ax;
  Q(0, 3) = 0.0;
  Q(1, 0) = 0.0;
  Q(1, 1) = (dt_4 / 4) * noise_ay;
  Q(1, 2) = 0.0;
  Q(1, 3) = (dt_3 / 2) * noise_ay;
  Q(2, 0) = (dt_3 / 2) * noise_ax;
  Q(2, 1) = 0.0;
  Q(2, 2) = dt_2 * noise_ax;
  Q(2, 3) = 0.0;
  Q(3, 0) = 0.0;
  Q(3, 1) = (dt_3 / 2) * noise_ay;
  Q(3, 2) = 0.0;
  Q(3, 3) = dt_2 * noise_ay;

  Q_m = Q;
}

Eigen::VectorXd EKFLidar::h(const Eigen::VectorXd &x,
                            const Eigen::VectorXd &z) {
  return Eigen::Vector2d(x[0], x[1]);
}

void EKFLidar::updateH() {
  H_m = (Eigen::MatrixXd(2, 4) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0)
            .finished();
}

}  // namespace kf
