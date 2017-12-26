#include "ekf_fusion.h"
#include <iostream>
#include "tools.h"

namespace kf {

EKFFusion::EKFFusion() {
  is_initialized_m = false;

  P_initial_m = (Eigen::MatrixXd(4,4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 1000).finished();

  u_m = (Eigen::VectorXd(4) << 0.0, 0.0, 0.0, 0.0).finished();

  R_lidar_m = (Eigen::MatrixXd(2,2) << 0.0225, 0, 0, 0.0225).finished();

  R_radar_m = (Eigen::MatrixXd(3,3) << 0.09, 0, 0, 0, 0.0009, 0, 0, 0, 0.09).finished();

}

void EKFFusion::processMeasurement(const MeasurementPackage &measurement_pack) {
  if (is_initialized_m == false) {
    initializeFilters(measurement_pack);
  } else {
    double timestamp = measurement_pack.timestamp_m / 1000000.0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      ekf_radar_m.predict(timestamp);
      ekf_radar_m.update(timestamp, measurement_pack.raw_measurements_m);

      ekf_lidar_m.reset(timestamp, ekf_radar_m.getX(), ekf_radar_m.getP());
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_lidar_m.predict(timestamp);
      ekf_lidar_m.update(timestamp, measurement_pack.raw_measurements_m);

      ekf_radar_m.reset(timestamp, ekf_lidar_m.getX(), ekf_lidar_m.getP());
    }
  }
}

Eigen::VectorXd EKFFusion::getX() { return ekf_radar_m.getX(); }

Eigen::MatrixXd EKFFusion::getP() { return ekf_radar_m.getP(); }

void EKFFusion::initializeFilters(const MeasurementPackage &measurement_pack) {
  double timestamp = measurement_pack.timestamp_m / 1000000.0;

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    Eigen::VectorXd x =
        Tools::polarToCartesian(measurement_pack.raw_measurements_m);

    ekf_lidar_m.initialize(timestamp, x, P_initial_m, u_m, R_lidar_m);
    ekf_radar_m.initialize(timestamp, x, P_initial_m, u_m, R_radar_m);

    is_initialized_m = true;
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    Eigen::Vector4d x(measurement_pack.raw_measurements_m[0],
                      measurement_pack.raw_measurements_m[1], 0.0, 0.0);

    ekf_lidar_m.initialize(timestamp, x, P_initial_m, u_m, R_lidar_m);
    ekf_radar_m.initialize(timestamp, x, P_initial_m, u_m, R_radar_m);

    is_initialized_m = true;
  }
}
}
