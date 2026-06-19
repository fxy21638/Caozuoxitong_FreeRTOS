/**
  *********************************************************************
  * @file    bsp_dht11.h
  * @author  fire
  * @version V1.0
  * @date    2026-06-10
  * @brief   DHT11 温湿度传感器驱动层
  *********************************************************************
  */

#ifndef __DHT11_H
#define __DHT11_H

#include "stm32f4xx.h"
#include <stdint.h>
#include <stddef.h>

#define DHT11_OK               0
#define DHT11_ERROR_TIMEOUT    1
#define DHT11_ERROR_CHECKSUM   2

void DHT11_GPIO_Config(void);
uint8_t DHT11_Read(uint8_t *pHumidity, uint8_t *pTemperature);
uint8_t DHT11_ReadAverage(uint8_t *pHumidity, uint8_t *pTemperature, uint8_t sampleCount);

#endif /* __DHT11_H */
