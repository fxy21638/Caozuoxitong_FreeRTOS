#include "bsp_rs485.h"

/* Ring buffer for ISR-to-task byte transfer */
static volatile uint8_t rs485_rx_ring[RS485_RX_BUF_SIZE];
static volatile uint16_t rs485_rx_wr;
static volatile uint16_t rs485_rx_rd;

/* -------------------------------------------------------------------------- */
/*  RS485 Hardware Initialization                                            */
/* -------------------------------------------------------------------------- */
void RS485_Init(void)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef  nvic;

    /* ---- GPIO clocks ---- */
    RCC_AHB1PeriphClockCmd(RS485_GPIO_PORT_CLK | RS485_DE_PORT_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(RS485_USART_CLK, ENABLE);

    /* ---- USART2 TX (PD5) : AF push-pull ---- */
    gpio.GPIO_Pin   = RS485_TX_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(RS485_TX_PORT, &gpio);
    GPIO_PinAFConfig(RS485_TX_PORT, GPIO_PinSource5, RS485_GPIO_AF);

    /* ---- USART2 RX (PD6) : AF pull-up ---- */
    gpio.GPIO_Pin   = RS485_RX_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(RS485_RX_PORT, &gpio);
    GPIO_PinAFConfig(RS485_RX_PORT, GPIO_PinSource6, RS485_GPIO_AF);

    /* ---- DE/RE (PG14) : push-pull output, default RX ---- */
    gpio.GPIO_Pin   = RS485_DE_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(RS485_DE_PORT, &gpio);
    RS485_RX_MODE();

    /* ---- USART2 : 115200 8N1 ---- */
    USART_StructInit(&usart);
    usart.USART_BaudRate            = RS485_BAUDRATE;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(RS485_USART, &usart);

    /* ---- NVIC : RXNE interrupt (preempt 6, sub 0) ---- */
    nvic.NVIC_IRQChannel                   = RS485_USART_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 6;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    /* Enable RXNE interrupt; TC interrupt enabled on-demand in StartTx */
    USART_ITConfig(RS485_USART, USART_IT_RXNE, ENABLE);

    /* Enable USART */
    USART_Cmd(RS485_USART, ENABLE);

    /* Reset ring buffer */
    rs485_rx_wr = 0;
    rs485_rx_rd = 0;
}

/* -------------------------------------------------------------------------- */
/*  Send API (called from task context)                                      */
/* -------------------------------------------------------------------------- */
void RS485_SendByte(uint8_t byte)
{
    USART_SendData(RS485_USART, byte);
    while (USART_GetFlagStatus(RS485_USART, USART_FLAG_TXE) == RESET);
}

void RS485_SendBytes(const uint8_t *data, uint16_t len)
{
    while (len--) {
        RS485_SendByte(*data++);
    }
}

/* -------------------------------------------------------------------------- */
/*  DE/RE Control                                                            */
/* -------------------------------------------------------------------------- */
void RS485_StartTx(void)
{
    RS485_TX_MODE();
    USART_ITConfig(RS485_USART, USART_IT_TC, ENABLE);
}

void RS485_FinishTx(void)
{
    /* Wait for TC then switch to RX */
    while (USART_GetFlagStatus(RS485_USART, USART_FLAG_TC) == RESET);
    RS485_RX_MODE();
    USART_ITConfig(RS485_USART, USART_IT_TC, DISABLE);
}

/* -------------------------------------------------------------------------- */
/*  RX Ring Buffer (called from ISR)                                         */
/* -------------------------------------------------------------------------- */
uint8_t RS485_RxISR(void)
{
    uint8_t byte = 0;

    if (USART_GetFlagStatus(RS485_USART, USART_FLAG_RXNE) != RESET) {
        byte = (uint8_t)USART_ReceiveData(RS485_USART);
        RS485_RingPut(byte);
    }

    /* Clear ORE if set (prevents RXNE lockup) */
    if (USART_GetFlagStatus(RS485_USART, USART_FLAG_ORE) != RESET) {
        (void)USART_ReceiveData(RS485_USART);
    }

    return byte;
}

/* Ring write (ISR side) */
uint8_t RS485_RingPut(uint8_t byte)
{
    uint16_t next = (rs485_rx_wr + 1) % RS485_RX_BUF_SIZE;
    if (next == rs485_rx_rd) return 0;   /* full */
    rs485_rx_ring[rs485_rx_wr] = byte;
    rs485_rx_wr = next;
    return 1;
}

/* Ring read (task side) */
uint8_t RS485_RingRead(uint8_t *byte)
{
    if (rs485_rx_wr == rs485_rx_rd) return 0;  /* empty */
    *byte = rs485_rx_ring[rs485_rx_rd];
    rs485_rx_rd = (rs485_rx_rd + 1) % RS485_RX_BUF_SIZE;
    return 1;
}
