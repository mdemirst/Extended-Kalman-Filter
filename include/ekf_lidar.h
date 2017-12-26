#include "EKF/extended_kalman_filter.h"

namespace kf {
class EKFLidar : public ExtendedKalmanFilter {
 protected:
 public:
  EKFLidar();

 protected:
  Eigen::VectorXd g(double timestamp, const Eigen::VectorXd& x,
                    const Eigen::VectorXd& u);
  void updateF(double timestamp);
  void updateQ(double timestamp);
  Eigen::VectorXd h(const Eigen::VectorXd& x, const Eigen::VectorXd& z);
  void updateH();
};
}
