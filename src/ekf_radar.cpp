#include "ekf_radar.h"
#include <iostream>
#include "tools.h"

namespace kf {

EKFRadar::EKFRadar() {}

Eigen::VectorXd EKFRadar::g(double timestamp, const Eigen::VectorXd &x,
                            const Eigen::VectorXd &u) {
  double dt = timestamp - timestamp_m;

  return Eigen::Vector4d(x[0] + dt * x[2], x[1] + dt * x[3], x[2], x[3]) + u;
}

void EKFRadar::updateF(double timestamp) {
  double dt = timestamp - timestamp_m;

  F_m = (Eigen::Matrix4d() << 1.0, 0.0, dt, 0.0, 0.0, 1.0, 0.0, dt, 0.0, 0.0,
         1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
            .finished();
}

void EKFRadar::updateQ(double timestamp) {
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

Eigen::VectorXd EKFRadar::h(const Eigen::VectorXd &x,
                            const Eigen::VectorXd &z) {
  double range = sqrt(x[0] * x[0] + x[1] * x[1]);

  if (range < 0.00001) {
    throw std::runtime_error("Error in radar measurement!");
  }

  return Eigen::Vector3d(range, Tools::wrapAngleAround(atan2(x[1], x[0]), z[1]),
                         (x[0] * x[2] + x[1] * x[3]) / range);
}

void EKFRadar::updateH() {
  Eigen::MatrixXd H(3, 4);

  double px = x_m[0];
  double py = x_m[1];
  double vx = x_m[2];
  double vy = x_m[3];

  double px2_py2 = px * px + py * py;
  double sqrt_px2_py2 = sqrt(px2_py2);
  double c3 = px2_py2 * sqrt_px2_py2;

  if (sqrt_px2_py2 < 0.00001) {
    throw std::runtime_error("Error in radar measurement!");
  }

  H(0, 0) = px / sqrt_px2_py2;
  H(0, 1) = py / sqrt_px2_py2;
  H(0, 2) = 0;
  H(0, 3) = 0;
  H(1, 0) = -py / px2_py2;
  H(1, 1) = px / px2_py2;
  H(1, 2) = 0;
  H(1, 3) = 0;
  H(2, 0) = py * (vx * py - vy * px) / c3;
  H(2, 1) = px * (vy * px - vx * py) / c3;
  H(2, 2) = px / sqrt_px2_py2;
  H(2, 3) = py / sqrt_px2_py2;

  H_m = H;
}

}  // namespace kf
