//
// Created by Siwei Wang on 2026/10/21.
//

#include "callback.hpp"
#include "tim.h"
#include "imu.hpp"

IMU imu;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6)
    {
        imu.accCalculate();
        imu.gyroCalculate();
        imu.sensorDataTransform();
        imu.positionUpdate();
    }
}
#ifdef __cplusplus
extern "C" {
#endif

void IMU_init()
{
    imu.init();
}

#ifdef __cplusplus
}
#endif