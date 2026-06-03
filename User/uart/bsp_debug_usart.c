/**
  ******************************************************************************
  * @file    bsp_debug_usart.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   USART1 driver - RXNE interrupt receive mode, printf redirect
  ******************************************************************************
  */

#include "bsp_debug_usart.h"
#include "FreeRTOS.h"
#include "task.h"

/**
  * @brief  NVIC config for USART1 interrupt
  * @param  None
  * @retval None
  */
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


/**
  * @brief  USART1 GPIO and peripheral init, 115200 8-N-1, RXNE interrupt
  * @param  None
  * @retval None
  */
void Debug_USART_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  RCC_AHB1PeriphClockCmd(DEBUG_USART_RX_GPIO_CLK|DEBUG_USART_TX_GPIO_CLK,ENABLE);
  RCC_APB2PeriphClockCmd(DEBUG_USART_CLK, ENABLE);

  /* GPIO init */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_PIN;
  GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_PIN;
  GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(DEBUG_USART_RX_GPIO_PORT, DEBUG_USART_RX_SOURCE, DEBUG_USART_RX_AF);
  GPIO_PinAFConfig(DEBUG_USART_TX_GPIO_PORT, DEBUG_USART_TX_SOURCE, DEBUG_USART_TX_AF);

  /* USART config */
  USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(DEBUG_USART, &USART_InitStructure);

  NVIC_Configuration();

  /* Enable RXNE interrupt (byte-by-byte receive) */
  USART_ITConfig(DEBUG_USART, USART_IT_RXNE, ENABLE);

  USART_Cmd(DEBUG_USART, ENABLE);
}


/*****************  Send one byte **********************/
void Usart_SendByte(USART_TypeDef * pUSARTx, uint8_t ch)
{
	USART_SendData(pUSARTx, ch);
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
}

/*****************  Send string **********************/
void Usart_SendString(USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k = 0;
  do
  {
      Usart_SendByte(pUSARTx, *(str + k));
      k++;
  } while(*(str + k) != '\0');

  while(USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET)
  {}
}

/* Redirect printf to USART1 */
int fputc(int ch, FILE *f)
{
	USART_SendData(DEBUG_USART, (uint8_t) ch);
	while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);
	return (ch);
}

/* Redirect scanf to USART1 */
int fgetc(FILE *f)
{
	while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_RXNE) == RESET);
	return (int)USART_ReceiveData(DEBUG_USART);
}

/*********************************************END OF FILE**********************/
