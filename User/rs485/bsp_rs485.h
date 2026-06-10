#ifndef __BSP_RS485_H
#define __BSP_RS485_H

#include "stm32f4xx.h"
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/*  USART 选择（USART2：PD5=TX, PD6=RX）                                       */
/* -------------------------------------------------------------------------- */
#define RS485_USART             USART2
#define RS485_USART_CLK         RCC_APB1Periph_USART2
#define RS485_USART_IRQn        USART2_IRQn
#define RS485_BAUDRATE          115200

#define RS485_TX_PORT           GPIOD
#define RS485_TX_PIN            GPIO_Pin_5
#define RS485_RX_PORT           GPIOD
#define RS485_RX_PIN            GPIO_Pin_6
#define RS485_GPIO_PORT_CLK     RCC_AHB1Periph_GPIOD
#define RS485_GPIO_AF           GPIO_AF_USART2

/* -------------------------------------------------------------------------- */
/*  DE/RE 方向控制（PG14：高电平=发送, 低电平=接收）                             */
/* -------------------------------------------------------------------------- */
#define RS485_DE_PORT           GPIOG
#define RS485_DE_PIN            GPIO_Pin_14
#define RS485_DE_PORT_CLK       RCC_AHB1Periph_GPIOG

#define RS485_TX_MODE()         GPIO_SetBits(RS485_DE_PORT, RS485_DE_PIN)
#define RS485_RX_MODE()         GPIO_ResetBits(RS485_DE_PORT, RS485_DE_PIN)

/* -------------------------------------------------------------------------- */
/*  接收环形缓冲大小                                                            */
/* -------------------------------------------------------------------------- */
#define RS485_RX_BUF_SIZE       256

/* -------------------------------------------------------------------------- */
/*  公开接口                                                                   */
/* -------------------------------------------------------------------------- */
void     RS485_Init(void);
void     RS485_SendByte(uint8_t byte);
void     RS485_SendBytes(const uint8_t *data, uint16_t len);
void     RS485_StartTx(void);            /* 拉高 DE, 进入发送模式               */
void     RS485_FinishTx(void);           /* 等待发送完成, 拉低 DE 回到接收模式   */

/* 中断服务函数 */
uint8_t  RS485_RxISR(void);              /* ISR 中调用：读 DR, 清 ORE, 存入环形缓冲 */

/* 环形缓冲（ISR 写, 任务读） */
uint8_t  RS485_RingPut(uint8_t byte);
uint8_t  RS485_RingRead(uint8_t *byte);

#endif /* __BSP_RS485_H */
