//
// Created by Siwei Wang on 2026/10/21.
//

#ifndef IMU_PROJECT_IMU_HPP
#define IMU_PROJECT_IMU_HPP

#include "quaternion.hpp"
#include "imu.hpp"
#include "bmi088.hpp"
#include "stm32f4xx_hal.h"


const float R_imu[3][3] = {{-1.0f, 0.0f, 0.0f},
                         {0.0f, 1.0f, 0.0f},
                         {0.0f, 0.0f, -1.0f}};

class IMU
{
public:
    void init();
    void accCalculate();
    void gyroCalculate();
    void sensorDataTransform();
    void positionUpdate();
    void eulerAngleTransform();
    void lowPassFilter();
    void mahonyFilter();

private:
    float acc_range_ = 0.0f;
    float gyro_range_ = 0.0f;

    struct imu_data
    {
        float accel[3] = {};
        float gyro[3] = {};
    };

    imu_data raw_data_, sensor_data_;
    imu_data calibration_bias_;
    imu_data filtered_data_;
    float k_low_pass_filter_ = 0.0f;
    float mahony_kp = 0.0f, mahony_ki = 0.0f;

    const float R_imu_[3][3] = {{-1.0f, 0.0f, 0.0f},
                         {0.0f, 1.0f, 0.0f},
                         {0.0f, 0.0f, -1.0f}};
    const float k_acc_ = 1.0f;

    Quaternion q_pos_{1.0f, 0.0f, 0.0f, 0.0f};

    float pos_pitch_ = 0.0f, pos_yaw_ = 0.0f, pos_roll_ = 0.0f;
};

#endif //IMU_PROJECT_IMU_HPP
