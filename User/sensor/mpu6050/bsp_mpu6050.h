#ifndef __BSP_MPU6050_H
#define __BSP_MPU6050_H

#include "stm32f4xx.h"

uint8_t MPU6050_DMP_Init(void);
uint8_t MPU6050_DMP_GetData(float *pitch, float *roll, float *yaw);

#endif /* __BSP_MPU6050_H */
