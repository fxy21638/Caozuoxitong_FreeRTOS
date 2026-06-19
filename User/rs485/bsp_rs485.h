#ifndef __BSP_RS485_H
#define __BSP_RS485_H

#include "stm32f4xx.h"
#include <stdio.h>

// RS485 引脚定义
#define RS485_USART                 USART2
#define RS485_USART_CLK             RCC_APB1Periph_USART2
#define RS485_USART_BAUDRATE        115200

#define RS485_TX_PIN                GPIO_Pin_5
#define RS485_TX_GPIO_PORT          GPIOD
#define RS485_TX_GPIO_CLK           RCC_AHB1Periph_GPIOD
#define RS485_TX_SOURCE             GPIO_PinSource5
#define RS485_TX_AF                 GPIO_AF_USART2

#define RS485_RX_PIN                GPIO_Pin_6
#define RS485_RX_GPIO_PORT          GPIOD
#define RS485_RX_GPIO_CLK           RCC_AHB1Periph_GPIOD
#define RS485_RX_SOURCE             GPIO_PinSource6
#define RS485_RX_AF                 GPIO_AF_USART2

// RS485 收发控制引脚 (RE/DE)
#define RS485_RE_PIN                GPIO_Pin_8
#define RS485_RE_GPIO_PORT          GPIOB
#define RS485_RE_GPIO_CLK           RCC_AHB1Periph_GPIOB

// 中断定义
#define RS485_USART_IRQ             USART2_IRQn

#define RS485_TX_EN()               GPIO_SetBits(RS485_RE_GPIO_PORT, RS485_RE_PIN)
#define RS485_RX_EN()               GPIO_ResetBits(RS485_RE_GPIO_PORT, RS485_RE_PIN)

// 接收环形缓冲
#define RS485_RX_BUF_SIZE           256

void RS485_Config(void);
void RS485_Send_Data(uint8_t *buf, uint8_t len);

uint8_t RS485_RxISR(void);
uint8_t RS485_RingPut(uint8_t byte);
uint8_t RS485_RingRead(uint8_t *byte);

#endif /* __BSP_RS485_H */
