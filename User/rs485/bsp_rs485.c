#include "bsp_rs485.h"

/* ISR → 任务 字节传递环形缓冲 */
static volatile uint8_t  rs485_rx_ring[RS485_RX_BUF_SIZE];
static volatile uint16_t rs485_rx_wr;
static volatile uint16_t rs485_rx_rd;

/**
  * @brief  RS485 硬件配置 (USART2, PD5/PD6, PB8)
  * @param  无
  * @retval 无
  */
void RS485_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 1. 使能时钟
    RCC_AHB1PeriphClockCmd(RS485_TX_GPIO_CLK | RS485_RX_GPIO_CLK | RS485_RE_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(RS485_USART_CLK, ENABLE);

    // 2. 配置 RE/DE 控制引脚 (PB8)
    GPIO_InitStructure.GPIO_Pin = RS485_RE_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(RS485_RE_GPIO_PORT, &GPIO_InitStructure);

    // 默认为接收模式
    RS485_RX_EN();

    // 3. 配置 USART TX 引脚 (PD5)
    GPIO_InitStructure.GPIO_Pin = RS485_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(RS485_TX_GPIO_PORT, &GPIO_InitStructure);

    // 4. 配置 USART RX 引脚 (PD6)
    GPIO_InitStructure.GPIO_Pin = RS485_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(RS485_RX_GPIO_PORT, &GPIO_InitStructure);

    // 5. 连接复用功能
    GPIO_PinAFConfig(RS485_TX_GPIO_PORT, RS485_TX_SOURCE, RS485_TX_AF);
    GPIO_PinAFConfig(RS485_RX_GPIO_PORT, RS485_RX_SOURCE, RS485_RX_AF);

    // 6. 配置 USART 参数
    USART_InitStructure.USART_BaudRate = RS485_USART_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(RS485_USART, &USART_InitStructure);

    // 7. 串口中断优先级配置
    NVIC_InitStructure.NVIC_IRQChannel = RS485_USART_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 8. 使能串口及接收中断
    USART_ITConfig(RS485_USART, USART_IT_RXNE, ENABLE);
    USART_Cmd(RS485_USART, ENABLE);

    // 9. 复位环形缓冲
    rs485_rx_wr = 0;
    rs485_rx_rd = 0;
}

/**
  * @brief  RS485 发送数据块
  * @param  buf: 数据缓冲区指针
  * @param  len: 发送长度
  * @retval 无
  */
void RS485_Send_Data(uint8_t *buf, uint8_t len)
{
    uint8_t t;

    // 1. 发送前先禁用接收中断，防止自发自收或切换毛刺引发中断风暴
    USART_ITConfig(RS485_USART, USART_IT_RXNE, DISABLE);

    RS485_TX_EN(); // 切换为发送模式

    // 等待方向切换稳定
    for(volatile int i = 0; i < 1000; i++);

    for(t = 0; t < len; t++)
    {
        // 循环期间等待 TXE (发送数据寄存器空)
        while(USART_GetFlagStatus(RS485_USART, USART_FLAG_TXE) == RESET);
        USART_SendData(RS485_USART, buf[t]);
    }
    // 所有字节压入寄存器后，最后等待 TC (发送完成)
    while(USART_GetFlagStatus(RS485_USART, USART_FLAG_TC) == RESET);

    // 2. 切换回接收模式
    RS485_RX_EN();

    // 等待方向切换稳定
    for(volatile int i = 0; i < 1000; i++);

    // 3. 在重新开启接收中断前，清除发送期间和切换时可能产生的任何错误或垃圾数据
    // 先读 SR 再读 DR 清除 ORE/FE/NE 等错误标志并清空接收缓冲区
    {
        volatile uint32_t tmpsr = RS485_USART->SR;
        volatile uint32_t tmpdr = RS485_USART->DR;
        (void)tmpsr;
        (void)tmpdr;
    }

    // 4. 重新使能接收中断
    USART_ITConfig(RS485_USART, USART_IT_RXNE, ENABLE);
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

uint8_t RS485_RingPut(uint8_t byte)
{
    uint16_t next = (rs485_rx_wr + 1) % RS485_RX_BUF_SIZE;
    if (next == rs485_rx_rd) return 0;
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
