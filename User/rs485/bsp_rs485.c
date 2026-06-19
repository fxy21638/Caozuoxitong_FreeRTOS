#include "bsp_rs485.h"

/* ISR �?任务 字节传递环形缓�?*/
static volatile uint8_t  rs485_rx_ring[RS485_RX_BUF_SIZE];
static volatile uint16_t rs485_rx_wr;
static volatile uint16_t rs485_rx_rd;

/**
  * @brief  RS485 硬件配置 (USART2, PD5/PD6, PB8)
  * @param  �?  * @retval �?  */
void RS485_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RS485_TX_GPIO_CLK | RS485_RX_GPIO_CLK | RS485_RE_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(RS485_USART_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = RS485_RE_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(RS485_RE_GPIO_PORT, &GPIO_InitStructure);
    RS485_RX_EN();

    GPIO_InitStructure.GPIO_Pin = RS485_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(RS485_TX_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = RS485_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(RS485_RX_GPIO_PORT, &GPIO_InitStructure);

    GPIO_PinAFConfig(RS485_TX_GPIO_PORT, RS485_TX_SOURCE, RS485_TX_AF);
    GPIO_PinAFConfig(RS485_RX_GPIO_PORT, RS485_RX_SOURCE, RS485_RX_AF);

    USART_InitStructure.USART_BaudRate = RS485_USART_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(RS485_USART, &USART_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = RS485_USART_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(RS485_USART, USART_IT_RXNE, ENABLE);
    USART_Cmd(RS485_USART, ENABLE);

    rs485_rx_wr = 0;
    rs485_rx_rd = 0;
}

/**
  * @brief  RS485 发送数据块
  * @param  buf: 数据缓冲区指�?  * @param  len: 发送长�?  * @retval �?  */
void RS485_Send_Data(uint8_t *buf, uint8_t len)
{
    uint8_t t;
    uint32_t tc_timeout;
    volatile uint32_t tmp;
    volatile uint32_t i;

    printf("[485-TX] %d bytes:", len);
    for (t = 0; t < len; t++) {
        printf(" %02X", buf[t]);
    }
    printf("\n");

    USART_ITConfig(RS485_USART, USART_IT_RXNE, DISABLE);

    tmp = RS485_USART->SR;
    tmp = RS485_USART->DR;
    (void)tmp;
    USART_ClearFlag(RS485_USART, USART_FLAG_TC);

    RS485_TX_EN();
    for (i = 0; i < 50000; i++) { ; }

    for (t = 0; t < len; t++) {
        while (USART_GetFlagStatus(RS485_USART, USART_FLAG_TXE) == RESET) { ; }
        USART_SendData(RS485_USART, buf[t]);
    }

    tc_timeout = 500000;
    while ((USART_GetFlagStatus(RS485_USART, USART_FLAG_TC) == RESET) && --tc_timeout) { ; }
    if (tc_timeout == 0) {
        printf("[485-TX] TC TIMEOUT!\n");
    }

    /* 清零 TX 期间的残余 (发送前已经关 RXNE, 不会有新数据进来) */
    tmp = RS485_USART->SR;
    tmp = RS485_USART->DR;
    (void)tmp;

    RS485_RX_EN();

    /* RXNE 必须在 DE 切回后立即打开, 不能等 delay 完才开 */
    USART_ITConfig(RS485_USART, USART_IT_RXNE, ENABLE);

    for (i = 0; i < 100000; i++) { ; }
}

/* ------------------------------------------------------------------------- */
/*  接收 ISR + 环形缓冲                                                     */
/* ------------------------------------------------------------------------- */
uint8_t RS485_RxISR(void)
{
    uint8_t byte = 0;
    if (USART_GetFlagStatus(RS485_USART, USART_FLAG_RXNE) != RESET) {
        byte = (uint8_t)USART_ReceiveData(RS485_USART);
        RS485_RingPut(byte);
    }
    if (USART_GetFlagStatus(RS485_USART, USART_FLAG_ORE) != RESET) {
        (void)USART_ReceiveData(RS485_USART);
    }
    return byte;
}

volatile uint32_t g_rs485_rx_overflow = 0;

uint8_t RS485_RingPut(uint8_t byte)
{
    uint16_t next = (rs485_rx_wr + 1) % RS485_RX_BUF_SIZE;
    if (next == rs485_rx_rd) { g_rs485_rx_overflow++; return 0; }
    rs485_rx_ring[rs485_rx_wr] = byte;
    rs485_rx_wr = next;
    return 1;
}

uint8_t RS485_RingRead(uint8_t *byte)
{
    if (rs485_rx_wr == rs485_rx_rd) return 0;
    *byte = rs485_rx_ring[rs485_rx_rd];
    rs485_rx_rd = (rs485_rx_rd + 1) % RS485_RX_BUF_SIZE;
    return 1;
}
