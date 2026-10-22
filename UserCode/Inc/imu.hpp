//
// Created by Siwei Wang on 2026/10/21.
//

#ifndef IMU_PROJECT_IMU_HPP
#define IMU_PROJECT_IMU_HPP
#include <cstdint>
#include <cmath>

const float R_imu[3][3] = {{-1.0f, 0.0f, 0.0f},
                         {0.0f, 1.0f, 0.0f},
                         {0.0f, 0.0f, -1.0f}};

class IMU
{
public:
    void init();
    void accCalculate();
    void gyroCalculate();
    // void zeroCalibration();
    void sensorDataTransform();

private:
    float acc_range = 0.0f;
    float gyro_range = 0.0f;

    struct imu_data
    {
        float accel[3] = {};
        float gyro[3] = {};
    };

    imu_data raw_data_, sensor_data_;
    imu_data calibration_bias_;

    const float R_imu_[3][3] = {{-1.0f, 0.0f, 0.0f},
                         {0.0f, 1.0f, 0.0f},
                         {0.0f, 0.0f, -1.0f}};
    const float k_acc_ = 1.0f;
};

#endif //IMU_PROJECT_IMU_HPP
