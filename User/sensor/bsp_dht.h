#ifndef __BSP_DHT_H
#define __BSP_DHT_H

#include "stm32f4xx.h"

/* DHT11/DHT22 data pin: PG9 */
#define DHT_GPIO_PORT   GPIOG
#define DHT_GPIO_PIN    GPIO_Pin_9
#define DHT_GPIO_CLK    RCC_AHB1Periph_GPIOG

#define DHT11  1
#define DHT22  2

uint8_t DHT_Init(void);
uint8_t DHT_Read_Data(uint8_t *temp, uint8_t *humi);
uint8_t DHT_Read_Data_Float(float *temp, float *humi);

#endif /* __BSP_DHT_H */
