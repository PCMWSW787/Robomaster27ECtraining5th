//
// Created by Siwei Wang on 2026/10/21.
//

#include "imu.hpp"
#include "bmi088.hpp"
#include "stm32f4xx_hal.h"

void IMU::accCalculate()
{
    uint8_t raw_acc_data[6] = {};
    bmi088_accel_read_reg(0x12, raw_acc_data, 6);
    int16_t raw_acc_data_x = 0, raw_acc_data_y = 0, raw_acc_data_z = 0;
    raw_acc_data_x = static_cast<int16_t>(static_cast<uint16_t>(raw_acc_data[1]) << 8 | static_cast<uint16_t>(raw_acc_data[0]));
    raw_acc_data_y = static_cast<int16_t>(static_cast<uint16_t>(raw_acc_data[3]) << 8 | static_cast<uint16_t>(raw_acc_data[2]));
    raw_acc_data_z = static_cast<int16_t>(static_cast<uint16_t>(raw_acc_data[5]) << 8 | static_cast<uint16_t>(raw_acc_data[4]));

    raw_data_.accel[0] = static_cast<float>(raw_acc_data_x) / 32768.0f * 1000.0f * acc_range;
    raw_data_.accel[1] = static_cast<float>(raw_acc_data_y) / 32768.0f * 1000.0f * acc_range;
    raw_data_.accel[2] = static_cast<float>(raw_acc_data_z) / 32768.0f * 1000.0f * acc_range;
}

void IMU::gyroCalculate()
{
    uint8_t raw_gyro_data[6] = {};
    bmi088_gyro_read_reg(0x02, raw_gyro_data, 6);
    int16_t raw_gyro_data_x = 0, raw_gyro_data_y = 0, raw_gyro_data_z = 0;
    raw_gyro_data_x = static_cast<int16_t>(static_cast<uint16_t>(raw_gyro_data[1]) << 8 | static_cast<uint16_t>(raw_gyro_data[0]));
    raw_gyro_data_y = static_cast<int16_t>(static_cast<uint16_t>(raw_gyro_data[3]) << 8 | static_cast<uint16_t>(raw_gyro_data[2]));
    raw_gyro_data_z = static_cast<int16_t>(static_cast<uint16_t>(raw_gyro_data[5]) << 8 | static_cast<uint16_t>(raw_gyro_data[4]));

    raw_data_.gyro[0] = static_cast<float>(raw_gyro_data_x) / 32768.0f * gyro_range;
    raw_data_.gyro[1] = static_cast<float>(raw_gyro_data_y) / 32768.0f * gyro_range;
    raw_data_.gyro[2] = static_cast<float>(raw_gyro_data_z) / 32768.0f * gyro_range;
}

void IMU::init()
{
    uint8_t raw_acc_range = 0;
    bmi088_accel_read_reg(0x41, &raw_acc_range, 1);
    if (raw_acc_range == 0x00) acc_range = 3;
    else if (raw_acc_range == 0x01) acc_range = 6;
    else if (raw_acc_range == 0x02) acc_range = 12;
    else if (raw_acc_range == 0x03) acc_range = 24;

    uint8_t raw_gyro_range = 0;
    bmi088_gyro_read_reg(0x0F, &raw_gyro_range, 1);
    if (raw_gyro_range == 0x00) gyro_range = 2000.0f;
    else if (raw_gyro_range == 0x01) gyro_range = 1000.0f;
    else if (raw_gyro_range == 0x02) gyro_range = 500.0f;
    else if (raw_gyro_range == 0x03) gyro_range = 250.0f;
    else if (raw_gyro_range == 0x04) gyro_range = 125.0f;

    for (uint8_t i = 0; i < 100; i++)
    {
        gyroCalculate();
        calibration_bias_.gyro[0] = (raw_data_.gyro[0] + calibration_bias_.gyro[0] * i) / (i + 1);
        calibration_bias_.gyro[1] = (raw_data_.gyro[1] + calibration_bias_.gyro[1] * i) / (i + 1);
        calibration_bias_.gyro[2] = (raw_data_.gyro[2] + calibration_bias_.gyro[2] * i) / (i + 1);
        HAL_Delay(1);
    }
}

void IMU::sensorDataTransform() {
    for (uint8_t i = 0; i < 3; i++) {
        sensor_data_.gyro[i] = (raw_data_.gyro[0] - calibration_bias_.gyro[0])  * R_imu_[i][0] +
                          (raw_data_.gyro[1] - calibration_bias_.gyro[1]) * R_imu_[i][1] +
                          (raw_data_.gyro[2] - calibration_bias_.gyro[2]) * R_imu_[i][2];
        sensor_data_.accel[i] = k_acc_ * ((raw_data_.accel[0] - calibration_bias_.accel[0]) * R_imu_[i][0] +
                                         (raw_data_.accel[1] - calibration_bias_.accel[1]) * R_imu_[i][1] +
                                         (raw_data_.accel[2] - calibration_bias_.accel[2]) * R_imu_[i][2]);
    }
}