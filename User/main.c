/* ========================================================================== */
/*  main.c — 管理端 (地址 0x01)                                                */
/*                                                                            */
/*  任务: RS485RxTask (pri 6), RS485TxTask (pri 5), PollTask (pri 4)          */
/*  RS485 总线: USART2 (PD5/PD6), DE/RE = PG14                                */
/*  协议: AA + Addr + Len + Type + Data[N] + CRC-8 + 55                       */
/* ========================================================================== */

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"

#include "config.h"
#include "bsp_led.h"
#include "bsp_debug_usart.h"
#include "./lcd/bsp_lcd.h"
#include "./rs485/bsp_rs485.h"
#include "protocol.h"

#include <string.h>
#include <stdio.h>

/* ========================================================================== */
/*  任务句柄与全局变量                                                         */
/* ========================================================================== */
TaskHandle_t  RS485RxTask_Handle   = NULL;    /* ISR 需要引用，不可 static    */
TimerHandle_t FrameTimeout_Handle  = NULL;    /* ISR 需要引用，不可 static    */
static QueueHandle_t g_tx_queue    = NULL;    /* TX 请求队列                   */

/* 任务通知位 */
#define NOTIFY_RX_BYTE      (1UL << 0)        /* ISR: 收到新字节               */
#define NOTIFY_FRAME_TO      (1UL << 1)        /* 定时器: 帧间超时              */

/* 协议解析器实例 */
static ProtocolParser_t g_parser;

/* PollTask 与 RS485RxTask 之间的应答同步标志 */
static volatile uint8_t g_response_ok;

/* LCD 显示缓冲 */
static char lcd_buf[64];

/* ========================================================================== */
/*  TX 请求结构体                                                              */
/* ========================================================================== */
typedef struct {
    uint8_t dest_addr;                         /* 目标地址                     */
    uint8_t msg_type;                          /* 消息类型                     */
    uint8_t data[RS485_MAX_PAYLOAD];           /* 数据载荷                     */
    uint8_t data_len;                          /* 数据长度                     */
} RS485TxRequest_t;

/* ========================================================================== */
/*  100ms 帧超时回调：通知 RS485RxTask 复位不完整帧                             */
/* ========================================================================== */
static void FrameTimeoutCallback(TimerHandle_t xTimer)
{
    (void)xTimer;
    if (g_parser.byte_count > 0) {
        xTaskNotify(RS485RxTask_Handle, NOTIFY_FRAME_TO, eSetBits);
    }
}

/* ========================================================================== */
/*  RS485 接收任务 (优先级 6)                                                   */
/*                                                                            */
/*  流程:                                                                      */
/*    1. 等待 ISR 通知 (NOTIFY_RX_BYTE)                                        */
/*    2. 从 RS485 环形缓冲逐字节读出                                            */
/*    3. 喂入协议状态机 Protocol_FeedByte()                                     */
/*    4. 完整帧就绪 → 按消息类型分发处理 → 更新 LCD                             */
/*    5. 收到 NOTIFY_FRAME_TO → 复位解析器丢弃不完整帧                          */
/* ========================================================================== */
static void RS485RxTask(void *pvParameters)
{
    (void)pvParameters;
    uint32_t notified;
    uint8_t  byte;

    Protocol_Init(&g_parser, DEVICE_ADDR, 1);   /* 管理端 = 接收所有地址 */

    while (1) {
        /* 等待 ISR 字节通知或帧超时通知 */
        xTaskNotifyWait(0, 0xFFFFFFFF, &notified, portMAX_DELAY);

        /* 消费环形缓冲中的所有字节 */
        while (RS485_RingRead(&byte)) {
            if (Protocol_FeedByte(&g_parser, byte)) {
                const ProtocolFrame_t *f = &g_parser.frame;

                switch (f->type) {

                case MSG_TYPE_TEMP_HUMI:        /* 温湿度应答 */
                    if (f->data_len >= 4) {
                        /* 温度 = ((TempH<<8)|TempL) / 10.0 */
                        float temp = (int16_t)((f->data[0] << 8) | f->data[1]) / 10.0f;
                        float humi = (int16_t)((f->data[2] << 8) | f->data[3]) / 10.0f;
                        sprintf(lcd_buf, "DHT: %5.1fC  %5.1f%%   ", temp, humi);
                        LCD_SetTextColor(LCD_COLOR_GREEN);
                        LCD_SetBackColor(LCD_COLOR_BLACK);
                        LCD_DisplayStringLine(LCD_LINE_2, (uint8_t *)lcd_buf);
                        printf("[RX] DHT  temp=%.1f C  humi=%.1f %%\n", temp, humi);
                    }
                    g_response_ok = 1;
                    LED2_TOGGLE;                /* 可视化反馈                   */
                    break;

                case MSG_TYPE_MPU6050:          /* 姿态角应答 */
                    if (f->data_len >= 12) {
                        /* 3 个 float (DMP 输出)，小端字节序 */
                        float pitch, roll, yaw;
                        memcpy(&pitch, &f->data[0], 4);
                        memcpy(&roll,  &f->data[4], 4);
                        memcpy(&yaw,   &f->data[8], 4);
                        sprintf(lcd_buf, "MPU: P%6.1f R%6.1f Y%6.1f   ", pitch, roll, yaw);
                        LCD_SetTextColor(LCD_COLOR_GREEN);
                        LCD_SetBackColor(LCD_COLOR_BLACK);
                        LCD_DisplayStringLine(LCD_LINE_4, (uint8_t *)lcd_buf);
                        printf("[RX] MPU  pitch=%.1f roll=%.1f yaw=%.1f\n", pitch, roll, yaw);
                    }
                    g_response_ok = 1;
                    LED2_TOGGLE;
                    break;

                case MSG_TYPE_LED_CTRL:         /* LED 控制应答 */
                    if (f->data_len >= 1) {
                        if (f->data[0]) {
                            LED1_ON;
                            printf("[RX] LED on\n");
                        } else {
                            LED1_OFF;
                            printf("[RX] LED off\n");
                        }
                    }
                    break;

                default:
                    printf("[RX] 未知消息类型 0x%02X\n", f->type);
                    break;
                }

                Protocol_ClearFrame(&g_parser);
            }
        }

        /* 帧超时 → 丢弃不完整帧 */
        if (notified & NOTIFY_FRAME_TO) {
            Protocol_Reset(&g_parser);
        }
    }
}

/* ========================================================================== */
/*  RS485 发送任务 (优先级 5)                                                   */
/*                                                                            */
/*  等待 TX 队列 → 调用 Protocol_BuildXxx 组帧 (含 CRC) →                      */
/*  拉高 DE → 发送字节 → 等待 TC → 拉低 DE 回到接收                            */
/* ========================================================================== */
static void RS485TxTask(void *pvParameters)
{
    (void)pvParameters;
    RS485TxRequest_t req;
    uint8_t tx_buf[PROTOCOL_BUF_SIZE];
    uint8_t frame_len;

    while (1) {
        if (xQueueReceive(g_tx_queue, &req, portMAX_DELAY) == pdPASS) {
            /* 根据是否有数据载荷选择组帧方式 */
            if (req.data_len > 0) {
                frame_len = Protocol_BuildResponse(tx_buf, req.dest_addr,
                                                   req.msg_type, req.data, req.data_len);
            } else {
                frame_len = Protocol_BuildRequest(tx_buf, req.dest_addr, req.msg_type);
            }

            /* 发送帧 */
            RS485_StartTx();
            RS485_SendBytes(tx_buf, frame_len);
            RS485_FinishTx();

            LED3_TOGGLE;                        /* 发送反馈                     */
        }
    }
}

/* ========================================================================== */
/*  轮询任务 (优先级 4)                                                         */
/*                                                                            */
/*  每 10 秒:                                                                  */
/*    1. 向采集前端1 (0x02) 请求温湿度 → 等待 500ms (含一次重试)                 */
/*    2. 向采集前端2 (0x03) 请求姿态角 → 等待 500ms (含一次重试)                 */
/*    3. 等待下一轮询周期                                                       */
/* ========================================================================== */
static void PollTask(void *pvParameters)
{
    (void)pvParameters;
    RS485TxRequest_t req;
    TickType_t last_wake;
    uint8_t retry;

    /* 等待系统就绪 */
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* LCD 标题行 */
    LCD_SetTextColor(LCD_COLOR_CYAN);
    LCD_SetBackColor(LCD_COLOR_BLACK);
    LCD_DisplayStringLine(LCD_LINE_0, (uint8_t *)"RS485 Manager (0x01)");
    LCD_DisplayStringLine(LCD_LINE_1, (uint8_t *)"Poll: --");
    LCD_DisplayStringLine(LCD_LINE_3, (uint8_t *)"Poll: --");

    last_wake = xTaskGetTickCount();

    while (1) {
        /* ---- 轮询采集前端1 (0x02) : DHT 温湿度 ---- */
        LCD_DisplayStringLine(LCD_LINE_1, (uint8_t *)"Poll: C1 (DHT)...     ");
        printf("[POLL] 向采集前端1 (0x02) 请求温湿度\n");

        req.dest_addr = ADDR_COLLECTOR_1;
        req.msg_type  = MSG_TYPE_TEMP_HUMI;
        req.data_len  = 0;

        for (retry = 0; retry < 2; retry++) {
            g_response_ok = 0;
            xQueueSend(g_tx_queue, &req, portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(RETRY_TIMEOUT_MS));
            if (g_response_ok) break;
            if (retry == 0) printf("[POLL] 重试 C1...\n");
        }

        /* ---- 轮询采集前端2 (0x03) : MPU6050 姿态 ---- */
        LCD_DisplayStringLine(LCD_LINE_3, (uint8_t *)"Poll: C2 (MPU)...     ");
        printf("[POLL] 向采集前端2 (0x03) 请求姿态\n");

        req.dest_addr = ADDR_COLLECTOR_2;
        req.msg_type  = MSG_TYPE_MPU6050;

        for (retry = 0; retry < 2; retry++) {
            g_response_ok = 0;
            xQueueSend(g_tx_queue, &req, portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(RETRY_TIMEOUT_MS));
            if (g_response_ok) break;
            if (retry == 0) printf("[POLL] 重试 C2...\n");
        }

        /* 清除 Poll 状态行 */
        LCD_DisplayStringLine(LCD_LINE_1, (uint8_t *)"                    ");
        LCD_DisplayStringLine(LCD_LINE_3, (uint8_t *)"                    ");

        /* 等待下一轮询周期 */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}

/* ========================================================================== */
/*  BSP 初始化                                                                 */
/* ========================================================================== */
static void BSP_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);   /* 抢占优先级分组 4 位    */
    LED_GPIO_Config();                                 /* RGB LED                */
    Debug_USART_Config();                              /* 调试串口 (printf)      */
    LCD_Init();                                        /* LTDC LCD 控制器        */
    LCD_LayerInit();                                   /* LCD 图层初始化         */
    LTDC_Cmd(ENABLE);
    LCD_SetLayer(LCD_BACKGROUND_LAYER);
    LCD_Clear(LCD_COLOR_BLACK);
    LCD_SetLayer(LCD_FOREGROUND_LAYER);
    LCD_SetTransparency(0xFF);
    LCD_Clear(LCD_COLOR_BLACK);
    RS485_Init();                                      /* RS485 USART2 总线       */
}

/* ========================================================================== */
/*  主入口                                                                     */
/* ========================================================================== */
int main(void)
{
    BSP_Init();

    printf("\n=== RS485 管理端 (地址 0x%02X) ===\n", DEVICE_ADDR);
    printf("轮询周期: %d ms  |  超时: %d ms  |  帧超时: %d ms\n",
           POLL_INTERVAL_MS, RETRY_TIMEOUT_MS, FRAME_TIMEOUT_MS);

    /* 帧超时定时器: 100ms 单次，收到字节时 ISR 重置，超时时复位解析器 */
    FrameTimeout_Handle = xTimerCreate("FrameTO",
        pdMS_TO_TICKS(FRAME_TIMEOUT_MS), pdFALSE, (void *)0, FrameTimeoutCallback);

    /* TX 请求队列 (最多 8 个待发送请求) */
    g_tx_queue = xQueueCreate(8, sizeof(RS485TxRequest_t));

    /* 创建任务 */
    xTaskCreate(RS485RxTask, "RS485Rx", 512, NULL, 6, &RS485RxTask_Handle);
    xTaskCreate(RS485TxTask, "RS485Tx", 256, NULL, 5, NULL);
    xTaskCreate(PollTask,    "Poll",    512, NULL, 4, NULL);

    printf("任务已创建。启动调度器...\n\n");
    vTaskStartScheduler();

    while (1) {}
}
