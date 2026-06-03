#include <stddef.h>
#include "bsp_mpu6050.h"
#include "bsp_mpu6050_i2c.h"
#include "inv_mpu.h"

uint8_t MPU6050_DMP_Init(void)
{
    /* Init I2C1 first */
    MPU6050_I2C_Init();

    /* Init MPU6050 + DMP (no INT pin used, polling mode) */
    return mpu_dmp_init(NULL, 0, 0);
}

uint8_t MPU6050_DMP_GetData(float *pitch, float *roll, float *yaw)
{
    return mpu_dmp_get_data(pitch, roll, yaw);
}
