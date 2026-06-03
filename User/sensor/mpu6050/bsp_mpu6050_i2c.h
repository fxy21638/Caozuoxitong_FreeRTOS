#ifndef __BSP_MPU6050_I2C_H
#define __BSP_MPU6050_I2C_H

#include "stm32f4xx.h"

/* I2C1: PB6=SCL, PB7=SDA, 400kHz */
#define MPU6050_I2C              I2C1
#define MPU6050_I2C_CLK          RCC_APB1Periph_I2C1

#define MPU6050_I2C_SCL_PIN      GPIO_Pin_6
#define MPU6050_I2C_SCL_GPIO     GPIOB
#define MPU6050_I2C_SCL_CLK      RCC_AHB1Periph_GPIOB
#define MPU6050_I2C_SCL_SOURCE   GPIO_PinSource6

#define MPU6050_I2C_SDA_PIN      GPIO_Pin_7
#define MPU6050_I2C_SDA_GPIO     GPIOB
#define MPU6050_I2C_SDA_CLK      RCC_AHB1Periph_GPIOB
#define MPU6050_I2C_SDA_SOURCE   GPIO_PinSource7

#define MPU6050_I2C_AF           GPIO_AF_I2C1

/* MPU6050 I2C address (AD0 pin = GND) */
#define MPU6050_ADDR             0x68

void MPU6050_I2C_Init(void);
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
void Delay_ms(unsigned long ms);
void mget_ms(unsigned long *time);

#endif /* __BSP_MPU6050_I2C_H */
