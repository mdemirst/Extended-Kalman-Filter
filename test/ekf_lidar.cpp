#include "gtest/gtest.h"

#include "ekf_lidar.h"
#include "tools.h"
#include "measurement_package.h"

using namespace kf;
using namespace Eigen;

class EKFLidarTest : public ::testing::Test {
 public:

    EKFLidarTest()
    {
        is_initialized_m = false;

        P_initial_m << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

        u_m << 0.0, 0.0, 0.0, 0.0;

        Eigen::MatrixXd R_lidar(2, 2);
        R_lidar << 0.0225, 0,
              0, 0.0225;
        R_lidar_m = R_lidar;
    }

    void SetUp() {}

    void TearDown() {}

    ~EKFLidarTest() {}

    void initialize()
    {
        MeasurementPackage measurement_pack;
        measurement_pack.timestamp_m = 0.0;
        measurement_pack.sensor_type_ = MeasurementPackage::LASER;
        measurement_pack.raw_measurements_m = Eigen::Vector2d(0,0);

        Eigen::Vector4d x(measurement_pack.raw_measurements_m[0],
                          measurement_pack.raw_measurements_m[1],
                          0.0, 0.0);

        ekf_lidar_m.initialize(measurement_pack.timestamp_m, x, P_initial_m, u_m, R_lidar_m);

        printState();

        is_initialized_m = true;
    }

    void printState()
    {
        std::cout << "State: " << std::endl;
        std::cout << "x = " << std::endl << ekf_lidar_m.getX() << std::endl;
        std::cout << "P = " << std::endl << ekf_lidar_m.getP() << std::endl << std::endl;
    }

    EKFLidar ekf_lidar_m;
    bool is_initialized_m;
    Eigen::Matrix4d P_initial_m;
    Eigen::Vector4d u_m;
    Eigen::MatrixXd R_lidar_m;
};


TEST_F(EKFLidarTest, initialize)
{
    initialize();
}

TEST_F(EKFLidarTest, filterRun)
{
    if(is_initialized_m == false)
        initialize();

    MeasurementPackage measurement_pack;

    // vx = 1.0, vy = 2.0, step_size = 1.0

    // Step 1 (x = 1.0, y = 2.0)
    measurement_pack.timestamp_m = 1.0;
    measurement_pack.sensor_type_ = MeasurementPackage::LASER;
    measurement_pack.raw_measurements_m = Eigen::Vector2d(1,2);

    ekf_lidar_m.predict(measurement_pack.timestamp_m);
    ekf_lidar_m.update(measurement_pack.timestamp_m, measurement_pack.raw_measurements_m);

    printState();


    // Step 2 (x = 2.0, y = 4.0)
    measurement_pack.timestamp_m = 2.0;
    measurement_pack.sensor_type_ = MeasurementPackage::LASER;
    measurement_pack.raw_measurements_m = Eigen::Vector2d(2,4);

    ekf_lidar_m.predict(measurement_pack.timestamp_m);
    ekf_lidar_m.update(measurement_pack.timestamp_m, measurement_pack.raw_measurements_m);

    printState();


    // Step 3 (x = 3.0, y = 6.0)
    measurement_pack.timestamp_m = 3.0;
    measurement_pack.sensor_type_ = MeasurementPackage::LASER;
    measurement_pack.raw_measurements_m = Eigen::Vector2d(3,6);

    ekf_lidar_m.predict(measurement_pack.timestamp_m);
    ekf_lidar_m.update(measurement_pack.timestamp_m, measurement_pack.raw_measurements_m);

    printState();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
