/**
  *********************************************************************
  * @file    rs485_app.c
  * @brief   RS485 协议解析 —— 温湿度采集前端 (地址 0x02)
  *
  * 支持的指令:
  *   Type 0x01 — 温湿度查询: 应答当前 DHT11 采集值
  *   Type 0x03 — LED 控制:   控制板载 LED 亮灭
  *
  * 帧格式:
  *   | 0xAA | Addr | Len | Type | Data... | CRC8 | 0x55 |
  *
  * 温湿度请求:  AA 02 01 01 CRC 55
  * 温湿度应答:  AA 01 05 01 TempH TempL HumH HumL CRC 55
  * LED控制请求: AA 02 02 03 0x01/0x00 CRC 55
  * LED控制应答: AA 01 02 03 0x01/0x00 CRC 55
  *********************************************************************
  */

#include "./rs485/rs485_app.h"
#include "./rs485/bsp_rs485.h"
#include "bsp_led.h"
#include <string.h>
#include <stdio.h>

/* 全局变量定义 */
char g_LastRecvMsg[50] = "NULL";

/* 状态机状态 */
typedef enum {
    STATE_WAIT_HEAD,
    STATE_ADDR,
    STATE_LEN,
    STATE_TYPE,
    STATE_DATA,
    STATE_CRC,
    STATE_TAIL
} RS485_State;

/**
  * @brief  CRC-8 计算 (多项式 0x07, 初始值 0x00)
  * @param  data: 数据指针
  * @param  len:  数据长度
  * @retval CRC-8 校验值
  */
static uint8_t CalcCRC8(uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
  * @brief  处理合法的数据帧并应答
  * @param  frame: 完整帧缓冲区
  * @param  len:   帧总长度（含帧头帧尾）
  * @retval 无
  */
static void RS485_HandleMessage(uint8_t *frame, uint8_t len)
{
    uint8_t type = frame[3];
    (void)len;  /* 未使用，保留供扩展 */

    if (type == MSG_TYPE_TEMP_HUMI) {
        /* 收到温湿度查询指令，打包回传当前采集数据 */
        uint8_t tx_buf[10];

        tx_buf[0] = RS485_FRAME_HEAD;       /* 帧头 */
        tx_buf[1] = 0x01;                   /* 目标地址：管理端 */
        tx_buf[2] = 0x05;                   /* 长度: Type(1B) + TempH(1B) + TempL(1B) + HumH(1B) + HumL(1B) = 5 */
        tx_buf[3] = MSG_TYPE_TEMP_HUMI;     /* 类型：温湿度 */

        /* 临界区保护，防止 DHT11_Task 更新数据时读到一半 */
        taskENTER_CRITICAL();
        tx_buf[4] = g_CurrentTemperature;   /* 温度整数部分 */
        tx_buf[5] = 0;                      /* 温度小数部分 (DHT11 无小数) */
        tx_buf[6] = g_CurrentHumidity;      /* 湿度整数部分 */
        tx_buf[7] = 0;                      /* 湿度小数部分 (DHT11 无小数) */
        taskEXIT_CRITICAL();

        tx_buf[8] = CalcCRC8(tx_buf, 8);    /* CRC8 校验 (帧头到数据末共 8 字节) */
        tx_buf[9] = RS485_FRAME_TAIL;       /* 帧尾 */

        RS485_Send_Data(tx_buf, 10);

        printf("RS485: send temp=%dC humi=%d%%\r\n",
               g_CurrentTemperature, g_CurrentHumidity);
    }
    else if (type == MSG_TYPE_LED_CTRL) {
        /* LED 控制指令 */
        uint8_t ctrl = frame[4];

        if (ctrl == 0x01) {
            LED2_ON;    /* 绿灯亮 */
        } else {
            LED2_OFF;   /* 绿灯灭 */
        }

        /* 构造应答帧回传管理端 */
        uint8_t tx_buf[7];
        tx_buf[0] = RS485_FRAME_HEAD;
        tx_buf[1] = 0x01;
        tx_buf[2] = 0x02;                   /* 长度: Type(1) + Data(1) = 2 */
        tx_buf[3] = MSG_TYPE_LED_CTRL;
        tx_buf[4] = ctrl;
        tx_buf[5] = CalcCRC8(tx_buf, 5);    /* CRC8 */
        tx_buf[6] = RS485_FRAME_TAIL;

        RS485_Send_Data(tx_buf, 7);

        printf("RS485: LED %s\r\n", (ctrl == 0x01) ? "ON" : "OFF");
    }
}

/**
  * @brief  RS485 协议解析任务
  * @param  parameter: 任务参数（未使用）
  * @retval 无
  *
  * 状态机逐字节解析 RS485 帧：
  * - 100ms 字节超时自动复位状态机（满足大作业要求）
  * - 地址过滤，只处理目标地址匹配的帧
  * - CRC 校验通过后调用 HandleMessage 应答
  */
void RS485_Task(void *parameter)
{
    uint8_t rx_byte;
    RS485_State state = STATE_WAIT_HEAD;
    /* 静态缓冲区，避免大数组占用任务栈 */
    static uint8_t frame_buf[256];
    uint16_t frame_idx = 0;
    uint8_t expect_len = 0;
    uint8_t data_cnt = 0;

    (void)parameter;

    while (1)
    {
        /* 阻塞等待串口数据，超时时间 100ms
         * 核心机制：字节间隔超过 100ms 则丢弃不完整帧 */
        if (xQueueReceive(RS485_RxQueue, &rx_byte, pdMS_TO_TICKS(100)) == pdFALSE)
        {
            /* 超时，复位状态机 */
            state = STATE_WAIT_HEAD;
            continue;
        }

        /* 防溢出保护 */
        if (frame_idx >= 250) {
            state = STATE_WAIT_HEAD;
            frame_idx = 0;
        }

        switch (state)
        {
            case STATE_WAIT_HEAD:
                if (rx_byte == RS485_FRAME_HEAD) {
                    frame_idx = 0;
                    frame_buf[frame_idx++] = rx_byte;
                    state = STATE_ADDR;
                }
                break;

            case STATE_ADDR:
                if (rx_byte == MY_RS485_ADDRESS) {
                    frame_buf[frame_idx++] = rx_byte;
                    state = STATE_LEN;
                } else {
                    /* 地址不匹配，丢弃该帧，重新寻找帧头 */
                    state = STATE_WAIT_HEAD;
                }
                break;

            case STATE_LEN:
                expect_len = rx_byte;
                frame_buf[frame_idx++] = rx_byte;
                state = STATE_TYPE;
                break;

            case STATE_TYPE:
                frame_buf[frame_idx++] = rx_byte;
                if (expect_len > 1) {
                    data_cnt = 0;
                    state = STATE_DATA;
                } else {
                    /* 无数据段，直接进入 CRC 校验 */
                    state = STATE_CRC;
                }
                break;

            case STATE_DATA:
                frame_buf[frame_idx++] = rx_byte;
                data_cnt++;
                if (data_cnt >= (expect_len - 1)) {
                    state = STATE_CRC;
                }
                break;

            case STATE_CRC:
                frame_buf[frame_idx++] = rx_byte;
                state = STATE_TAIL;
                break;

            case STATE_TAIL:
                if (rx_byte == RS485_FRAME_TAIL) {
                    frame_buf[frame_idx++] = rx_byte;

                    /* CRC 校验：校验范围从帧头到 CRC 前一字节 */
                    uint8_t calc_crc = CalcCRC8(frame_buf, frame_idx - 2);
                    uint8_t recv_crc = frame_buf[frame_idx - 2];

                    if (calc_crc == recv_crc) {
                        /* 记录原始 HEX 数据到全局变量（供 LCD 显示调试） */
                        int pos = 0;
                        for (int i = 0; i < (int)frame_idx; i++) {
                            pos += sprintf(&g_LastRecvMsg[pos], "%02X ", frame_buf[i]);
                            if (pos >= 48) break;
                        }

                        /* CRC 校验通过，处理指令并应答 */
                        RS485_HandleMessage(frame_buf, frame_idx);
                    }
                }
                /* 无论成功与否，复位状态机等待下一帧 */
                state = STATE_WAIT_HEAD;
                break;
        }
    }
}
