#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include "kalman_filter.h"

namespace kf {
class ExtendedKalmanFilter : public KalmanFilter {
 protected:
  bool is_first_update_m;
  double timestamp_m;

 public:
  ExtendedKalmanFilter();
  virtual ~ExtendedKalmanFilter();
  void initialize(double timestamp, const Eigen::VectorXd& x,
                  const Eigen::MatrixXd& P, const Eigen::VectorXd& u,
                  const Eigen::MatrixXd& R);
  void predict(double timestamp);
  void update(double timestamp, const Eigen::VectorXd& z);
  void reset(double timestamp, const Eigen::VectorXd& x,
             const Eigen::MatrixXd& P);

 protected:
  virtual Eigen::VectorXd g(double timestamp, const Eigen::VectorXd& x,
                            const Eigen::VectorXd& u) = 0;
  virtual void updateF(double timestamp) = 0;
  virtual void updateQ(double timestamp) = 0;
  virtual Eigen::VectorXd h(const Eigen::VectorXd& x,
                            const Eigen::VectorXd& z) = 0;
  virtual void updateH() = 0;
};
}

#endif
