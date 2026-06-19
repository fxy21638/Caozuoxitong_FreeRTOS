#ifndef __RS485_APP_H
#define __RS485_APP_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stm32f4xx.h"

/* 协议定义 */
#define RS485_FRAME_HEAD            0xAA
#define RS485_FRAME_TAIL            0x55
#define MY_RS485_ADDRESS            0x02    /* 温湿度采集前端地址 */

/* 消息类型 */
#define MSG_TYPE_TEMP_HUMI          0x01    /* 温湿度 */
#define MSG_TYPE_MPU6050            0x02    /* MPU6050 姿态角 */
#define MSG_TYPE_LED_CTRL           0x03    /* LED 控制 */

/* RS485 接收队列 */
extern QueueHandle_t RS485_RxQueue;

/* 当前温湿度数据（由 DHT11_Task 更新，RS485_Task 读取） */
extern uint8_t g_CurrentTemperature;
extern uint8_t g_CurrentHumidity;

/* 最新收到的有效帧数据（供 LCD 显示调试用） */
extern char g_LastRecvMsg[50];

/* RS485 协议解析任务 */
void RS485_Task(void *parameter);

#endif /* __RS485_APP_H */
