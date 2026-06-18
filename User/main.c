/* ========================================================================== */
/*  main.c — 物联网总线通信系统                                                  */
/*                                                                             */
/*  ROLE_MANAGER    (0x01): LCD+触摸 GUI, 10s 轮询两个采集前端, LED 控制        */
/*  ROLE_COLLECTOR_1(0x02): DHT 温湿度采集, RS485 查询响应, LCD 显示            */
/*  ROLE_COLLECTOR_2(0x03): MPU6050 姿态采集, RS485 查询响应, LCD 显示          */
/* ========================================================================== */

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"

#include "config.h"
#include "bsp_led.h"
#include "bsp_debug_usart.h"
#include "./key/bsp_key.h"
#include "./lcd/bsp_lcd.h"
#include "./rs485/bsp_rs485.h"
#include "protocol.h"

#include <string.h>
#include <stdio.h>

/* ========================================================================== */
/*  公共句柄 (所有角色共享)                                                      */
/* ========================================================================== */
TaskHandle_t  RS485RxTask_Handle   = NULL;    /* ISR 需要引用                  */
TimerHandle_t FrameTimeout_Handle  = NULL;    /* ISR 需要引用                  */
static ProtocolParser_t g_parser;             /* 协议解析器实例                */

/* ========================================================================== */
/*  栈溢出钩子 — configCHECK_FOR_STACK_OVERFLOW=2 需要                          */
/*  任务栈溢出时立即打印任务名, 防止静默死锁                                    */
/* ========================================================================== */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    printf("\n[FAULT] STACK OVERFLOW in task: %s\n", pcTaskName);
    taskDISABLE_INTERRUPTS();
    for (;;) {}    /* 停机便于调试 */
}

/* ========================================================================== */
/*  内存分配失败钩子                                                            */
/* ========================================================================== */
void vApplicationMallocFailedHook(void)
{
    printf("\n[FAULT] MALLOC FAILED\n");
    taskDISABLE_INTERRUPTS();
    for (;;) {}
}

/* 任务通知位 */
#define NOTIFY_RX_BYTE      (1UL << 0)        /* ISR: 收到新字节               */
#define NOTIFY_FRAME_TO      (1UL << 1)        /* 定时器: 帧间超时              */

/* ========================================================================== */
/*  TX 请求结构体 (所有角色共用)                                                  */
/* ========================================================================== */
typedef struct {
    uint8_t dest_addr;                         /* 目标地址                     */
    uint8_t msg_type;                          /* 消息类型                     */
    uint8_t data[RS485_MAX_PAYLOAD];           /* 数据载荷                     */
    uint8_t data_len;                          /* 数据长度                     */
} RS485TxRequest_t;

/* ---- TX 队列 ---- */
#if DEVICE_ROLE == ROLE_MANAGER
/* 管理端: g_tx_queue 非 static — gui.c 通过 extern 引用发送触摸按钮请求      */
QueueHandle_t g_tx_queue = NULL;
#else
/* 采集前端: 仅 main.c 内部使用                                              */
static QueueHandle_t g_tx_queue = NULL;
#endif

/* ========================================================================== */
/*  100ms 帧超时回调: 通知 RS485RxTask 复位不完整帧                             */
/* ========================================================================== */
static void FrameTimeoutCallback(TimerHandle_t xTimer)
{
    (void)xTimer;
    if (g_parser.byte_count > 0) {
        xTaskNotify(RS485RxTask_Handle, NOTIFY_FRAME_TO, eSetBits);
    }
}

/* ========================================================================== */
/*  RS485 发送任务 (优先级 5) — 所有角色共用                                     */
/*                                                                            */
/*  等待 TX 队列 → 组帧 (含 CRC) → 拉高 DE → 发送 → 等待 TC → 拉低 DE          */
/* ========================================================================== */
static void RS485TxTask(void *pvParameters)
{
    (void)pvParameters;
    RS485TxRequest_t req;
    uint8_t tx_buf[PROTOCOL_BUF_SIZE];
    uint8_t frame_len;

    while (1) {
        if (xQueueReceive(g_tx_queue, &req, portMAX_DELAY) == pdPASS) {
            if (req.data_len > 0) {
                frame_len = Protocol_BuildResponse(tx_buf, req.dest_addr,
                                                   req.msg_type, req.data, req.data_len);
            } else {
                frame_len = Protocol_BuildRequest(tx_buf, req.dest_addr, req.msg_type);
            }

            printf("[TX] -> 0x%02X type=0x%02X len=%d\n",
                   req.dest_addr, req.msg_type, frame_len);
            RS485_Send_Data(tx_buf, frame_len);

            LED3_TOGGLE;
        }
    }
}

/* ========================================================================== */
/*  LCD 简单显示 (采集前端用)                                                    */
/* ========================================================================== */
#if DEVICE_ROLE != ROLE_MANAGER
static char lcd_buf[64];

static void LCD_PrintLine(uint16_t line, uint16_t fg, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(lcd_buf, sizeof(lcd_buf), fmt, args);
    va_end(args);
    LCD_SetTextColor(fg);
    LCD_SetBackColor(LCD_COLOR_BLACK);
    LCD_DisplayStringLine(line, (uint8_t *)lcd_buf);
}
#endif

/* ========================================================================== */
/*  ROLE_MANAGER — 管理端 (地址 0x01)                                          */
/* ========================================================================== */
#if DEVICE_ROLE == ROLE_MANAGER

#include "./gui/gui.h"

static QueueHandle_t g_gui_queue = NULL;       /* → GUI_Task                  */
static volatile uint8_t g_response_ok;         /* PollTask 应答同步标志        */

/* -------------------------------------------------------------------------- */
/*  RS485 接收任务 (优先级 6)                                                   */
/* -------------------------------------------------------------------------- */
static void RS485RxTask(void *pvParameters)
{
    (void)pvParameters;
    uint32_t notified;
    uint8_t  byte;

    Protocol_Init(&g_parser, DEVICE_ADDR, 1);   /* is_manager = 1             */

    while (1) {
        xTaskNotifyWait(0, 0xFFFFFFFF, &notified, portMAX_DELAY);

        while (RS485_RingRead(&byte)) {
            if (Protocol_FeedByte(&g_parser, byte)) {
                const ProtocolFrame_t *f = &g_parser.frame;
                GUIMsg_t gui_msg;

                switch (f->type) {

                case MSG_TYPE_TEMP_HUMI:
                    if (f->data_len >= 4) {
                        float temp = (int16_t)((f->data[0] << 8) | f->data[1]) / 10.0f;
                        float humi = (int16_t)((f->data[2] << 8) | f->data[3]) / 10.0f;

                        gui_msg.type = GUI_SENSOR_DHT;
                        gui_msg.data.dht.temp = temp;
                        gui_msg.data.dht.humi = humi;
                        xQueueSend(g_gui_queue, &gui_msg, 0);

                        printf("[RX] DHT  temp=%.1f C  humi=%.1f %%\n", temp, humi);
                    }
                    g_response_ok = 1;
                    LED2_TOGGLE;
                    break;

                case MSG_TYPE_MPU6050:
                    if (f->data_len >= 12) {
                        float pitch, roll, yaw;
                        memcpy(&pitch, &f->data[0], 4);
                        memcpy(&roll,  &f->data[4], 4);
                        memcpy(&yaw,   &f->data[8], 4);

                        gui_msg.type = GUI_SENSOR_MPU6050;
                        gui_msg.data.mpu.pitch = pitch;
                        gui_msg.data.mpu.roll  = roll;
                        gui_msg.data.mpu.yaw   = yaw;
                        xQueueSend(g_gui_queue, &gui_msg, 0);

                        printf("[RX] MPU  pitch=%.1f roll=%.1f yaw=%.1f\n", pitch, roll, yaw);
                    }
                    g_response_ok = 1;
                    LED2_TOGGLE;
                    break;

                case MSG_TYPE_LED_CTRL:
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
                    printf("[RX] unknown type 0x%02X\n", f->type);
                    break;
                }

                Protocol_ClearFrame(&g_parser);
            }
        }

        if (notified & NOTIFY_FRAME_TO) {
            Protocol_Reset(&g_parser);
        }
    }
}

#if 0  /* ---- PollTask 已禁用：改为 KEY1/KEY2 手动触发 (GUI_Task 处理) ---- */
/* -------------------------------------------------------------------------- */
/*  轮询任务 (优先级 4)                                                         */
/* -------------------------------------------------------------------------- */
static void PollTask(void *pvParameters)
{
    (void)pvParameters;
    RS485TxRequest_t req;
    TickType_t last_wake;
    uint8_t retry;

    vTaskDelay(pdMS_TO_TICKS(2000));
    last_wake = xTaskGetTickCount();

    while (1) {
        printf("[POLL] request DHT from 0x02\n");
        req.dest_addr = ADDR_COLLECTOR_1;
        req.msg_type  = MSG_TYPE_TEMP_HUMI;
        req.data_len  = 0;

        for (retry = 0; retry < 2; retry++) {
            g_response_ok = 0;
            xQueueSend(g_tx_queue, &req, portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(RETRY_TIMEOUT_MS));
            if (g_response_ok) break;
            if (retry == 0) printf("[POLL] retry C1...\n");
        }

        printf("[POLL] request MPU from 0x03\n");
        req.dest_addr = ADDR_COLLECTOR_2;
        req.msg_type  = MSG_TYPE_MPU6050;

        for (retry = 0; retry < 2; retry++) {
            g_response_ok = 0;
            xQueueSend(g_tx_queue, &req, portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(RETRY_TIMEOUT_MS));
            if (g_response_ok) break;
            if (retry == 0) printf("[POLL] retry C2...\n");
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}
#endif  /* PollTask disabled */

/* -------------------------------------------------------------------------- */
/*  BSP 初始化 (管理端)                                                         */
/* -------------------------------------------------------------------------- */
static void BSP_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    LED_GPIO_Config();
    Debug_USART_Config();

    /* KEY1(PA0)/KEY2(PC13) GPIO 初始化 (输入模式 + 外部上拉) */
    Key_GPIO_Config();

    LCD_Init();
    LCD_LayerInit();
    LTDC_Cmd(ENABLE);
    LCD_SetLayer(LCD_BACKGROUND_LAYER);
    LCD_Clear(LCD_COLOR_BLACK);
    LCD_SetLayer(LCD_FOREGROUND_LAYER);
    LCD_SetTransparency(0xFF);
    LCD_Clear(LCD_COLOR_BLACK);
    RS485_Config();
}

/* -------------------------------------------------------------------------- */
/*  主入口 (管理端)                                                             */
/* -------------------------------------------------------------------------- */
int main(void)
{
    BSP_Init();

    printf("\n=== RS485 Manager (0x%02X) ===\n", DEVICE_ADDR);
    printf("Manual mode: KEY1→DHT  KEY2→MPU  FrameTO: %d ms\n",
           FRAME_TIMEOUT_MS);

    FrameTimeout_Handle = xTimerCreate("FrameTO",
        pdMS_TO_TICKS(FRAME_TIMEOUT_MS), pdFALSE, (void *)0, FrameTimeoutCallback);

    g_tx_queue   = xQueueCreate(8,  sizeof(RS485TxRequest_t));
    g_gui_queue  = xQueueCreate(8,  sizeof(GUIMsg_t));

    xTaskCreate(RS485RxTask, "RS485Rx", 512, NULL, 6, &RS485RxTask_Handle);
    xTaskCreate(RS485TxTask, "RS485Tx", 256, NULL, 5, NULL);
    /* PollTask 已禁用 — 改为 KEY1/KEY2 手动触发 (GUI_Task 内处理按键) */
    xTaskCreate(GUI_Task,    "GUI",    4096, (void *)g_gui_queue, 3, NULL);

    printf("Scheduler start...\n\n");
    vTaskStartScheduler();

    while (1) {}
}

/* ========================================================================== */
/*  ROLE_COLLECTOR_1 — 采集前端1 (地址 0x02, DHT 温湿度)                        */
/* ========================================================================== */
#elif DEVICE_ROLE == ROLE_COLLECTOR_1

#include "./sensor/dht/bsp_dht.h"
#include "./data/data_store.h"

static QueueHandle_t g_gui_queue = NULL;
static DataStore_t   g_store;
static float         g_temp = 0.0f, g_humi = 0.0f;
static uint8_t       g_sensor_valid = 0;
static uint8_t       g_led_state = 0;

/* -------------------------------------------------------------------------- */
/*  RS485 接收任务                                                              */
/*  接收管理端查询 (MSG_TYPE_TEMP_HUMI) 和 LED 控制 (MSG_TYPE_LED_CTRL)         */
/* -------------------------------------------------------------------------- */
static void RS485RxTask(void *pvParameters)
{
    (void)pvParameters;
    uint32_t notified;
    uint8_t  byte;

    Protocol_Init(&g_parser, DEVICE_ADDR, 0);   /* is_manager = 0             */

    while (1) {
        xTaskNotifyWait(0, 0xFFFFFFFF, &notified, portMAX_DELAY);

        while (RS485_RingRead(&byte)) {
            if (Protocol_FeedByte(&g_parser, byte)) {
                const ProtocolFrame_t *f = &g_parser.frame;
                RS485TxRequest_t tx_req;

                switch (f->type) {

                case MSG_TYPE_TEMP_HUMI:
                    /* 管理端请求温湿度 → 应答最新数据 */
                    if (g_sensor_valid) {
                        int16_t t_raw = (int16_t)(g_temp * 10.0f);
                        int16_t h_raw = (int16_t)(g_humi * 10.0f);
                        uint8_t resp[4];
                        resp[0] = (uint8_t)(t_raw >> 8);
                        resp[1] = (uint8_t)(t_raw & 0xFF);
                        resp[2] = (uint8_t)(h_raw >> 8);
                        resp[3] = (uint8_t)(h_raw & 0xFF);

                        tx_req.dest_addr = ADDR_MANAGER;
                        tx_req.msg_type  = MSG_TYPE_TEMP_HUMI;
                        memcpy(tx_req.data, resp, 4);
                        tx_req.data_len = 4;
                        xQueueSend(g_tx_queue, &tx_req, 0);
                        LED2_TOGGLE;
                        printf("[C1] DHT response: %.1fC %.1f%%\n", g_temp, g_humi);
                    }
                    break;

                case MSG_TYPE_LED_CTRL:
                    if (f->data_len >= 1) {
                        g_led_state = f->data[0] ? 1 : 0;
                        if (g_led_state) LED1_ON; else LED1_OFF;

                        /* 应答 LED 状态 */
                        tx_req.dest_addr = ADDR_MANAGER;
                        tx_req.msg_type  = MSG_TYPE_LED_CTRL;
                        tx_req.data[0]   = g_led_state;
                        tx_req.data_len  = 1;
                        xQueueSend(g_tx_queue, &tx_req, 0);
                        printf("[C1] LED %s\n", g_led_state ? "ON" : "OFF");
                    }
                    break;

                default:
                    break;
                }

                Protocol_ClearFrame(&g_parser);
            }
        }

        if (notified & NOTIFY_FRAME_TO) {
            Protocol_Reset(&g_parser);
        }
    }
}

/* -------------------------------------------------------------------------- */
/*  传感器采集任务 (优先级 4)                                                    */
/* -------------------------------------------------------------------------- */
static void SensorTask(void *pvParameters)
{
    (void)pvParameters;
    float temp, humi;
    GUIMsg_t msg;
    SensorRecord_t rec;

    DHT_Init();
    DataStore_Init(&g_store);
    vTaskDelay(pdMS_TO_TICKS(2000));           /* DHT 上电稳定                  */

    while (1) {
        if (DHT_Read_Data_Float(&temp, &humi)) {
            g_temp = temp;
            g_humi = humi;
            g_sensor_valid = 1;

            /* 存入环形缓冲 */
            rec.type = SENSOR_TYPE_DHT;
            rec.tick = xTaskGetTickCount();
            rec.data.dht.temp = temp;
            rec.data.dht.humi = humi;
            DataStore_Push(&g_store, &rec);

            /* 发送到 GUI */
            msg.type = GUI_SENSOR_DHT;
            msg.data.dht.temp = temp;
            msg.data.dht.humi = humi;
            xQueueSend(g_gui_queue, &msg, 0);

            printf("[C1] DHT read OK: %.1fC %.1f%%\n", temp, humi);
        } else {
            g_sensor_valid = 0;
            printf("[C1] DHT read FAIL\n");
        }

        vTaskDelay(pdMS_TO_TICKS(DHT_INTERVAL_MS));
    }
}

/* -------------------------------------------------------------------------- */
/*  GUI 任务 (简单 LCD 行显示)                                                  */
/* -------------------------------------------------------------------------- */
static void GUITask(void *pvParameters)
{
    (void)pvParameters;
    GUIMsg_t msg;
    uint32_t count = 0;

    LCD_SetTextColor(LCD_COLOR_CYAN);
    LCD_SetBackColor(LCD_COLOR_BLACK);
    LCD_DisplayStringLine(LCD_LINE_0, (uint8_t *)"Collector 1 (0x02) DHT");
    LCD_DisplayStringLine(LCD_LINE_1, (uint8_t *)"Temp:  ---.- C");
    LCD_DisplayStringLine(LCD_LINE_2, (uint8_t *)"Humi:  ---.- %");
    LCD_DisplayStringLine(LCD_LINE_3, (uint8_t *)"Store: 0      ");

    while (1) {
        if (xQueueReceive(g_gui_queue, &msg, pdMS_TO_TICKS(500)) == pdPASS) {
            count++;
            LCD_PrintLine(LCD_LINE_1, LCD_COLOR_GREEN,
                          "Temp:  %5.1f C", msg.data.dht.temp);
            LCD_PrintLine(LCD_LINE_2, LCD_COLOR_GREEN,
                          "Humi:  %5.1f %%", msg.data.dht.humi);
            LCD_PrintLine(LCD_LINE_3, LCD_COLOR_YELLOW,
                          "Store: %-2d      ", DataStore_Count(&g_store));
            LCD_PrintLine(LCD_LINE_4, LCD_COLOR_WHITE,
                          "Updates: %lu", count);
        }
        LCD_PrintLine(LCD_LINE_5, LCD_COLOR_WHITE,
                      "LED: %s  ", g_led_state ? "ON " : "OFF");
    }
}

/* -------------------------------------------------------------------------- */
/*  BSP 初始化                                                                 */
/* -------------------------------------------------------------------------- */
static void BSP_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    LED_GPIO_Config();
    Debug_USART_Config();

    /* KEY1(PA0)/KEY2(PC13) GPIO 初始化 (输入模式 + 外部上拉) */
    Key_GPIO_Config();

    LCD_Init();
    LCD_LayerInit();
    LTDC_Cmd(ENABLE);
    LCD_SetLayer(LCD_BACKGROUND_LAYER);
    LCD_Clear(LCD_COLOR_BLACK);
    LCD_SetLayer(LCD_FOREGROUND_LAYER);
    LCD_SetTransparency(0xFF);
    LCD_Clear(LCD_COLOR_BLACK);
    RS485_Config();
}

/* -------------------------------------------------------------------------- */
/*  主入口 (采集前端1)                                                           */
/* -------------------------------------------------------------------------- */
int main(void)
{
    BSP_Init();

    printf("\n=== RS485 Collector 1 (0x%02X) DHT ===\n", DEVICE_ADDR);
    printf("DHT interval: %d ms\n", DHT_INTERVAL_MS);

    FrameTimeout_Handle = xTimerCreate("FrameTO",
        pdMS_TO_TICKS(FRAME_TIMEOUT_MS), pdFALSE, (void *)0, FrameTimeoutCallback);

    g_tx_queue  = xQueueCreate(8, sizeof(RS485TxRequest_t));
    g_gui_queue = xQueueCreate(8, sizeof(GUIMsg_t));

    xTaskCreate(RS485RxTask, "RS485Rx", 512, NULL, 6, &RS485RxTask_Handle);
    xTaskCreate(RS485TxTask, "RS485Tx", 256, NULL, 5, NULL);
    xTaskCreate(SensorTask,  "Sensor",  512, NULL, 4, NULL);
    xTaskCreate(GUITask,     "GUI",    2048, NULL, 3, NULL);

    printf("Scheduler start...\n\n");
    vTaskStartScheduler();

    while (1) {}
}

/* ========================================================================== */
/*  ROLE_COLLECTOR_2 — 采集前端2 (地址 0x03, MPU6050 姿态)                      */
/* ========================================================================== */
#elif DEVICE_ROLE == ROLE_COLLECTOR_2

#include "./sensor/mpu6050/bsp_mpu6050.h"
#include "./data/data_store.h"

static QueueHandle_t g_gui_queue = NULL;
static DataStore_t   g_store;
static float         g_pitch = 0.0f, g_roll = 0.0f, g_yaw = 0.0f;
static uint8_t       g_sensor_valid = 0;

/* -------------------------------------------------------------------------- */
/*  RS485 接收任务                                                              */
/*  接收管理端查询 (MSG_TYPE_MPU6050)，应答最新姿态角                            */
/* -------------------------------------------------------------------------- */
static void RS485RxTask(void *pvParameters)
{
    (void)pvParameters;
    uint32_t notified;
    uint8_t  byte;

    Protocol_Init(&g_parser, DEVICE_ADDR, 0);   /* is_manager = 0             */

    while (1) {
        xTaskNotifyWait(0, 0xFFFFFFFF, &notified, portMAX_DELAY);

        while (RS485_RingRead(&byte)) {
            if (Protocol_FeedByte(&g_parser, byte)) {
                const ProtocolFrame_t *f = &g_parser.frame;

                if (f->type == MSG_TYPE_MPU6050 && g_sensor_valid) {
                    RS485TxRequest_t tx_req;
                    uint8_t flat[12];

                    memcpy(&flat[0],  &g_pitch, 4);
                    memcpy(&flat[4],  &g_roll,  4);
                    memcpy(&flat[8],  &g_yaw,   4);

                    tx_req.dest_addr = ADDR_MANAGER;
                    tx_req.msg_type  = MSG_TYPE_MPU6050;
                    memcpy(tx_req.data, flat, 12);
                    tx_req.data_len = 12;
                    xQueueSend(g_tx_queue, &tx_req, 0);
                    LED2_TOGGLE;
                    printf("[C2] MPU response: P%.1f R%.1f Y%.1f\n",
                           g_pitch, g_roll, g_yaw);
                }

                Protocol_ClearFrame(&g_parser);
            }
        }

        if (notified & NOTIFY_FRAME_TO) {
            Protocol_Reset(&g_parser);
        }
    }
}

/* -------------------------------------------------------------------------- */
/*  传感器采集任务 (优先级 4)                                                    */
/* -------------------------------------------------------------------------- */
static void SensorTask(void *pvParameters)
{
    (void)pvParameters;
    float pitch, roll, yaw;
    GUIMsg_t msg;
    SensorRecord_t rec;

    vTaskDelay(pdMS_TO_TICKS(500));
    if (!MPU6050_DMP_Init()) {
        printf("[C2] MPU6050 DMP init FAIL!\n");
        vTaskSuspend(NULL);                    /* DMP 初始化失败, 挂起自己     */
    }
    printf("[C2] MPU6050 DMP init OK\n");

    DataStore_Init(&g_store);

    while (1) {
        if (MPU6050_DMP_GetData(&pitch, &roll, &yaw)) {
            g_pitch = pitch;
            g_roll  = roll;
            g_yaw   = yaw;
            g_sensor_valid = 1;

            /* 存入环形缓冲 */
            rec.type = SENSOR_TYPE_MPU6050;
            rec.tick = xTaskGetTickCount();
            rec.data.mpu.pitch = pitch;
            rec.data.mpu.roll  = roll;
            rec.data.mpu.yaw   = yaw;
            DataStore_Push(&g_store, &rec);

            /* 发送到 GUI */
            msg.type = GUI_SENSOR_MPU6050;
            msg.data.mpu.pitch = pitch;
            msg.data.mpu.roll  = roll;
            msg.data.mpu.yaw   = yaw;
            xQueueSend(g_gui_queue, &msg, 0);

            printf("[C2] MPU: P%.1f R%.1f Y%.1f\n", pitch, roll, yaw);
        } else {
            printf("[C2] MPU read FAIL\n");
        }

        vTaskDelay(pdMS_TO_TICKS(MPU6050_INTERVAL_MS));
    }
}

/* -------------------------------------------------------------------------- */
/*  GUI 任务 (简单 LCD 行显示)                                                  */
/* -------------------------------------------------------------------------- */
static void GUITask(void *pvParameters)
{
    (void)pvParameters;
    GUIMsg_t msg;
    uint32_t count = 0;

    LCD_SetTextColor(LCD_COLOR_CYAN);
    LCD_SetBackColor(LCD_COLOR_BLACK);
    LCD_DisplayStringLine(LCD_LINE_0, (uint8_t *)"Collector 2 (0x03) MPU");
    LCD_DisplayStringLine(LCD_LINE_1, (uint8_t *)"Pitch: -----.-");
    LCD_DisplayStringLine(LCD_LINE_2, (uint8_t *)"Roll : -----.-");
    LCD_DisplayStringLine(LCD_LINE_3, (uint8_t *)"Yaw  : -----.-");

    while (1) {
        if (xQueueReceive(g_gui_queue, &msg, pdMS_TO_TICKS(500)) == pdPASS) {
            count++;
            LCD_PrintLine(LCD_LINE_1, LCD_COLOR_GREEN,
                          "Pitch: %7.1f", msg.data.mpu.pitch);
            LCD_PrintLine(LCD_LINE_2, LCD_COLOR_GREEN,
                          "Roll : %7.1f", msg.data.mpu.roll);
            LCD_PrintLine(LCD_LINE_3, LCD_COLOR_GREEN,
                          "Yaw  : %7.1f", msg.data.mpu.yaw);
            LCD_PrintLine(LCD_LINE_4, LCD_COLOR_YELLOW,
                          "Store: %-2d      ", DataStore_Count(&g_store));
            LCD_PrintLine(LCD_LINE_5, LCD_COLOR_WHITE,
                          "Updates: %lu", count);
        }
    }
}

/* -------------------------------------------------------------------------- */
/*  BSP 初始化                                                                 */
/* -------------------------------------------------------------------------- */
static void BSP_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    LED_GPIO_Config();
    Debug_USART_Config();

    /* KEY1(PA0)/KEY2(PC13) GPIO 初始化 (输入模式 + 外部上拉) */
    Key_GPIO_Config();

    LCD_Init();
    LCD_LayerInit();
    LTDC_Cmd(ENABLE);
    LCD_SetLayer(LCD_BACKGROUND_LAYER);
    LCD_Clear(LCD_COLOR_BLACK);
    LCD_SetLayer(LCD_FOREGROUND_LAYER);
    LCD_SetTransparency(0xFF);
    LCD_Clear(LCD_COLOR_BLACK);
    RS485_Config();
}

/* -------------------------------------------------------------------------- */
/*  主入口 (采集前端2)                                                           */
/* -------------------------------------------------------------------------- */
int main(void)
{
    BSP_Init();

    printf("\n=== RS485 Collector 2 (0x%02X) MPU6050 ===\n", DEVICE_ADDR);
    printf("MPU interval: %d ms\n", MPU6050_INTERVAL_MS);

    FrameTimeout_Handle = xTimerCreate("FrameTO",
        pdMS_TO_TICKS(FRAME_TIMEOUT_MS), pdFALSE, (void *)0, FrameTimeoutCallback);

    g_tx_queue  = xQueueCreate(8, sizeof(RS485TxRequest_t));
    g_gui_queue = xQueueCreate(8, sizeof(GUIMsg_t));

    xTaskCreate(RS485RxTask, "RS485Rx", 512, NULL, 6, &RS485RxTask_Handle);
    xTaskCreate(RS485TxTask, "RS485Tx", 256, NULL, 5, NULL);
    xTaskCreate(SensorTask,  "Sensor",  512, NULL, 4, NULL);
    xTaskCreate(GUITask,     "GUI",    2048, NULL, 3, NULL);

    printf("Scheduler start...\n\n");
    vTaskStartScheduler();

    while (1) {}
}

/* ========================================================================== */
/*  未知角色 — 编译错误                                                          */
/* ========================================================================== */
#else
#error "DEVICE_ROLE must be ROLE_MANAGER, ROLE_COLLECTOR_1, or ROLE_COLLECTOR_2"
#endif
