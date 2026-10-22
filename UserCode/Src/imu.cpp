//
// Created by Siwei Wang on 2026/10/21.
//

#include "imu.hpp"
#include "math.h"

const float PI = 3.141592;

void IMU::accCalculate()
{
    uint8_t raw_acc_data[6] = {};
    bmi088_accel_read_reg(0x12, raw_acc_data, 6);
    int16_t raw_acc_data_x = 0, raw_acc_data_y = 0, raw_acc_data_z = 0;
    raw_acc_data_x = static_cast<int16_t>(static_cast<uint16_t>(raw_acc_data[1]) << 8 | static_cast<uint16_t>(raw_acc_data[0]));
    raw_acc_data_y = static_cast<int16_t>(static_cast<uint16_t>(raw_acc_data[3]) << 8 | static_cast<uint16_t>(raw_acc_data[2]));
    raw_acc_data_z = static_cast<int16_t>(static_cast<uint16_t>(raw_acc_data[5]) << 8 | static_cast<uint16_t>(raw_acc_data[4]));

    raw_data_.accel[0] = static_cast<float>(raw_acc_data_x) / 32768.0f * 1000.0f * acc_range_;
    raw_data_.accel[1] = static_cast<float>(raw_acc_data_y) / 32768.0f * 1000.0f * acc_range_;
    raw_data_.accel[2] = static_cast<float>(raw_acc_data_z) / 32768.0f * 1000.0f * acc_range_;
}

void IMU::gyroCalculate()
{
    uint8_t raw_gyro_data[6] = {};
    bmi088_gyro_read_reg(0x02, raw_gyro_data, 6);
    int16_t raw_gyro_data_x = 0, raw_gyro_data_y = 0, raw_gyro_data_z = 0;
    raw_gyro_data_x = static_cast<int16_t>(static_cast<uint16_t>(raw_gyro_data[1]) << 8 | static_cast<uint16_t>(raw_gyro_data[0]));
    raw_gyro_data_y = static_cast<int16_t>(static_cast<uint16_t>(raw_gyro_data[3]) << 8 | static_cast<uint16_t>(raw_gyro_data[2]));
    raw_gyro_data_z = static_cast<int16_t>(static_cast<uint16_t>(raw_gyro_data[5]) << 8 | static_cast<uint16_t>(raw_gyro_data[4]));

    raw_data_.gyro[0] = static_cast<float>(raw_gyro_data_x) / 32768.0f * gyro_range_;
    raw_data_.gyro[1] = static_cast<float>(raw_gyro_data_y) / 32768.0f * gyro_range_;
    raw_data_.gyro[2] = static_cast<float>(raw_gyro_data_z) / 32768.0f * gyro_range_;
}

void IMU::init()
{
    uint8_t raw_acc_range = 0;
    bmi088_accel_read_reg(0x41, &raw_acc_range, 1);
    if (raw_acc_range == 0x00) acc_range_ = 3;
    else if (raw_acc_range == 0x01) acc_range_ = 6;
    else if (raw_acc_range == 0x02) acc_range_ = 12;
    else if (raw_acc_range == 0x03) acc_range_ = 24;

    uint8_t raw_gyro_range = 0;
    bmi088_gyro_read_reg(0x0F, &raw_gyro_range, 1);
    if (raw_gyro_range == 0x00) gyro_range_ = 2000.0f;
    else if (raw_gyro_range == 0x01) gyro_range_ = 1000.0f;
    else if (raw_gyro_range == 0x02) gyro_range_ = 500.0f;
    else if (raw_gyro_range == 0x03) gyro_range_ = 250.0f;
    else if (raw_gyro_range == 0x04) gyro_range_ = 125.0f;

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

void IMU::positionUpdate()
{
    q_pos_ = q_pos_ +
        (q_pos_ * 0.5f) *
        Quaternion{0.0f, sensor_data_.gyro[0], sensor_data_.gyro[1], sensor_data_.gyro[2]} *
        0.001f;
    q_pos_.normalize();
}

void IMU::eulerAngleTransform(){

    const float qw = q_pos_.w_, qx = q_pos_.x_, qy = q_pos_.y_, qz = q_pos_.z_;
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabs(sinp) >= 0.9999999f) {
        pos_pitch_ = copysign(PI / 2.0f, sinp);
        pos_yaw_ = 2.0f * atan2f(-copysign(1.0f, sinp) * qx, qw);
        pos_roll_ = 0.0f;
    }
    else {
        pos_pitch_ = asinf(sinp);
        pos_yaw_ = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
        pos_roll_ = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));
    }
}

void IMU::lowPassFilter()
{
    for (uint8_t i = 0; i < 3; i++){
        filtered_data_.accel[i] = k_low_pass_filter_ * sensor_data_.accel[i] + ( 1.0f - k_low_pass_filter_) * filtered_data_.accel[i];
        filtered_data_.gyro[i] = sensor_data_.gyro[i];
    }
}

void IMU::mahonyFilter()
{
    float U_measure[3] = {}, U_predict[3] = {} ;
    float l = 0;
    l = sqrt(filtered_data_.accel[0] * filtered_data_.accel[0] +filtered_data_.accel[1] * filtered_data_.accel[1] +filtered_data_.accel[2] * filtered_data_.accel[2]);
    for (uint8_t i = 0; i < 3; i++){
        U_measure[i] = filtered_data_.accel[i] / l;
    }
    float x = q_pos_.x_, y = q_pos_.y_, z = q_pos_.z_, w = q_pos_.w_;
    U_predict[0] = 2 * (x * z - w * y);
    U_predict[1] = 2 * (y * z + w * x);
    U_predict[2] = 1 - 2 * (x * x + y * y);

    float Error[3] = {
        U_measure[1] * U_predict[2] - U_measure[2] * U_predict[1],
        U_measure[2] * U_predict[0] - U_measure[0] * U_predict[2],
        U_measure[0] * U_predict[1] - U_measure[1] * U_predict[0]
    };

    float omega_corrcted[3] = {
        filtered_data_.gyro[0] + mahony_ki * Error[0] + mahony_kp * Error[0],
        filtered_data_.gyro[1] + mahony_ki * Error[1] + mahony_kp * Error[1],
        filtered_data_.gyro[2] + mahony_ki * Error[2] + mahony_kp * Error[2]
    };

    q_pos_ = q_pos_ +
        (q_pos_ * 0.5f) *
        Quaternion{0.0f, omega_corrcted[0], omega_corrcted[1], omega_corrcted[2]} *
        0.001f;
    q_pos_.normalize();

}