#ifndef __BSP_RS485_H
#define __BSP_RS485_H

#include "stm32f4xx.h"
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/*  USART Selection (USART2: PD5=TX, PD6=RX)                                 */
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
/*  DE/RE Control (PG14: high=TX, low=RX)                                    */
/* -------------------------------------------------------------------------- */
#define RS485_DE_PORT           GPIOG
#define RS485_DE_PIN            GPIO_Pin_14
#define RS485_DE_PORT_CLK       RCC_AHB1Periph_GPIOG

#define RS485_TX_MODE()         GPIO_SetBits(RS485_DE_PORT, RS485_DE_PIN)
#define RS485_RX_MODE()         GPIO_ResetBits(RS485_DE_PORT, RS485_DE_PIN)

/* -------------------------------------------------------------------------- */
/*  RX Ring Buffer Size                                                      */
/* -------------------------------------------------------------------------- */
#define RS485_RX_BUF_SIZE       256

/* -------------------------------------------------------------------------- */
/*  Public API                                                               */
/* -------------------------------------------------------------------------- */
void     RS485_Init(void);
void     RS485_SendByte(uint8_t byte);
void     RS485_SendBytes(const uint8_t *data, uint16_t len);
void     RS485_StartTx(void);            /* Set DE high, enable TC interrupt  */
void     RS485_FinishTx(void);           /* Set DE low, back to RX mode       */

/* ISR helpers */
uint8_t  RS485_RxISR(void);              /* ISR: read DR, clear ORE, push ring */

/* Ring buffer (ISR writes, task reads) */
uint8_t  RS485_RingPut(uint8_t byte);
uint8_t  RS485_RingRead(uint8_t *byte);

#endif /* __BSP_RS485_H */
