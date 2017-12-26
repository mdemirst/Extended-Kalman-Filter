#include "ekf_lidar.h"
#include "ekf_radar.h"
#include "measurement_package.h"

namespace kf
{

class EKFFusion
{
protected:
    EKFLidar ekf_lidar_m;
    EKFRadar ekf_radar_m;
    bool is_initialized_m;
    Eigen::MatrixXd P_initial_m;
    Eigen::VectorXd u_m;
    Eigen::MatrixXd R_lidar_m;
    Eigen::MatrixXd R_radar_m;

public:
    EKFFusion();
    void processMeasurement(const MeasurementPackage &measurement_pack);
    Eigen::VectorXd getX();
    Eigen::MatrixXd getP();

protected:
    void initializeFilters(const MeasurementPackage &measurement_pack);

};

} // namespace kf
