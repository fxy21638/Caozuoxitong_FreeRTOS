/**
  *********************************************************************
  * @file    main.c
  * @version V2.1
  * @date    2026-06-19
  * @brief   FreeRTOS DHT11 + RS485 + Touch GUI 温湿度采集前端 (地址 0x02)
  *
  * 架构:
  *   触摸 ISR  →  BinarySemaphore  →  DHT11_GUI_Task
  *            (极轻量，只发信号)    (任务上下文做 I2C 读取)
  *
  *   EXTI15_10 (下降沿) → give semaphore
  *   DHT11_GUI_Task: take semaphore → GTP_TouchProcess() → 读 xTouchQueue
  *                   → 按键判断 → 切换屏幕 / DHT11 采集
  *********************************************************************
  */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "bsp_led.h"
#include "bsp_debug_usart.h"
#include "./lcd/bsp_lcd.h"
#include "./dht11/bsp_dht11.h"
#include "./rs485/bsp_rs485.h"
#include "./rs485/rs485_app.h"
#include "./touch/gt9xx.h"
#include <stdio.h>
#include <string.h>

/* ── 常量 ──────────────────────────────────── */
#define DHT11_HISTORY_COUNT     10

#define SCREEN_MAIN             0
#define SCREEN_HISTORY          1

/* 触摸按键区域 —— 必须与 GUI_DrawButton 绘制的矩形完全一致 */
#define BTN_X1                  220
#define BTN_X2                  580
#define BTN_Y1                  340
#define BTN_Y2                  400
#define BTN_HEIGHT              (BTN_Y2 - BTN_Y1)   /* 60 */
#define BTN_WIDTH               (BTN_X2 - BTN_X1)   /* 360 */

/* 触摸事件结构体 (与 gt9xx.c 中定义一致) */
typedef struct {
    int32_t x;
    int32_t y;
} TouchEvent_t;

/* 历史记录结构体 */
typedef struct {
    uint8_t    humidity;
    uint8_t    temperature;
    uint8_t    valid;
    TickType_t timestamp;
} DHT11_RecordTypeDef;

/* ── 任务句柄 ──────────────────────────────── */
static TaskHandle_t AppTaskCreate_Handle  = NULL;
static TaskHandle_t DHT11_Task_Handle     = NULL;
static TaskHandle_t RS485_Task_Handle     = NULL;

/* ── 队列 ──────────────────────────────────── */
QueueHandle_t RS485_RxQueue   = NULL;
QueueHandle_t xTouchQueue     = NULL;

/* ── 信号量 ────────────────────────────────── */
QueueHandle_t xTouchSemaphore = NULL;   /* binary semaphore for touch ISR → task */

/* ── 全局温湿度 ────────────────────────────── */
uint8_t g_CurrentTemperature = 0;
uint8_t g_CurrentHumidity    = 0;

/* ── 历史记录 ──────────────────────────────── */
static DHT11_RecordTypeDef DHT11_History[DHT11_HISTORY_COUNT];
static uint8_t  DHT11_HistoryIndex = 0;
static uint8_t  DHT11_HistoryCount = 0;
static uint32_t DHT11_SampleCount  = 0;

/* ── 屏幕状态 ──────────────────────────────── */
static uint8_t screen_mode          = SCREEN_MAIN;
static uint8_t screen_need_redraw   = 1;   /* 标记屏幕需要刷新 */
static TickType_t last_redraw_tick  = 0;   /* 上次重绘时间，用于定期刷新防花屏 */
#define SCREEN_REFRESH_INTERVAL_MS   2000   /* 强制刷新间隔（毫秒），防止SDRAM帧缓冲数据老化 */

/* ── 函数声明 ──────────────────────────────── */
static void AppTaskCreate(void);
static void DHT11_Task(void *pvParameters);
static void BSP_Init(void);
static void DHT11_Record(uint8_t humidity, uint8_t temperature, uint8_t valid);

static void GUI_DrawMainScreen(void);
static void GUI_DrawHistoryScreen(void);
static void GUI_DrawButton(void);
static uint8_t GUI_IsTouchInButton(TouchEvent_t *e);

int main(void)
{
    BSP_Init();

    printf("\r\n=== Fire F429  DHT11 + RS485 + Touch  (Addr 0x02) ===\r\n\r\n");

    if (pdPASS == xTaskCreate((TaskFunction_t)AppTaskCreate,
                              "AppTaskCreate", 512, NULL, 1,
                              &AppTaskCreate_Handle))
        vTaskStartScheduler();

    return -1;
}

/* ── AppTaskCreate ──────────────────────────────── */
static void AppTaskCreate(void)
{
    taskENTER_CRITICAL();

    if (pdPASS == xTaskCreate((TaskFunction_t)DHT11_Task,
                              "DHT11_GUI", 1280, NULL, 2, &DHT11_Task_Handle))
        printf("[OK] DHT11_GUI_Task\r\n");

    if (pdPASS == xTaskCreate((TaskFunction_t)RS485_Task,
                              "RS485", 1024, NULL, 3, &RS485_Task_Handle))
        printf("[OK] RS485_Task\r\n");

    taskEXIT_CRITICAL();
    vTaskDelete(AppTaskCreate_Handle);
}

/* ── DHT11 + GUI 主任务 ─────────────────────────── */
static void DHT11_Task(void *parameter)
{
    uint8_t     humidity, temperature, result;
    TickType_t  last_read_tick   = 0;
    TickType_t  last_touch_tick  = 0;   /* 按键消抖：上次处理时间 */
    TickType_t  now;
    TouchEvent_t touch_event;
    uint8_t     redraw;
    uint8_t     touch_handled;

    (void)parameter;

    screen_need_redraw = 1;

    while (1)
    {
        redraw = 0;

        /* ── 等待触摸中断信号量（100ms 超时）── */
        if (xSemaphoreTake(xTouchSemaphore, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            /*
             * 在任务上下文中执行 I2C 触摸读取。
             * GTP_TouchProcess() → Goodix_TS_Work_Func() → GTP_Touch_Down()
             *   → xQueueSend(xTouchQueue, ...)
             */
            GTP_TouchProcess();

            touch_handled = 0;

            /* 读取触摸事件队列 */
            while (xQueueReceive(xTouchQueue, &touch_event, 0) == pdTRUE)
            {
                if (!touch_handled && GUI_IsTouchInButton(&touch_event))
                {
                    now = xTaskGetTickCount();

                    /* 消抖：距上次按键至少 800ms 才响应 */
                    if ((now - last_touch_tick) >= pdMS_TO_TICKS(800))
                    {
                        last_touch_tick = now;

                        if (screen_mode == SCREEN_MAIN) {
                            screen_mode = SCREEN_HISTORY;
                            printf("[GUI] -> History\r\n");
                        } else {
                            screen_mode = SCREEN_MAIN;
                            printf("[GUI] -> Main\r\n");
                        }
                        redraw = 1;
                        touch_handled = 1;  /* 本周期只处理一次按键 */
                    }
                }
            }
        }

        /* ── 每 5 秒采集 DHT11 ──────────────── */
        now = xTaskGetTickCount();
        if ((now - last_read_tick) >= pdMS_TO_TICKS(5000))
        {
            last_read_tick = now;
            redraw = 1;

            result = DHT11_ReadAverage(&humidity, &temperature, 5);

            if (result == DHT11_OK)
            {
                printf("[DHT11] %d%%  %dC\r\n", humidity, temperature);
                LED_GREEN;

                taskENTER_CRITICAL();
                g_CurrentHumidity    = humidity;
                g_CurrentTemperature = temperature;
                taskEXIT_CRITICAL();

                DHT11_Record(humidity, temperature, 1);
            }
            else if (result == DHT11_ERROR_CHECKSUM)
            {
                printf("[DHT11] Checksum error\r\n");
                LED_RED;
                DHT11_Record(0, 0, 0);
            }
            else
            {
                printf("[DHT11] Timeout\r\n");
                LED_BLUE;
                DHT11_Record(0, 0, 0);
            }
        }

        /* ── 定期强制刷新：防止SDRAM帧缓冲老化导致花屏 ── */
        if (!redraw && !screen_need_redraw)
        {
            if ((now - last_redraw_tick) >= pdMS_TO_TICKS(SCREEN_REFRESH_INTERVAL_MS))
            {
                redraw = 1;
            }
        }

        /* ── 需要时重绘屏幕 ────────────────── */
        if (redraw || screen_need_redraw)
        {
            screen_need_redraw = 0;
            last_redraw_tick = now;

            if (screen_mode == SCREEN_MAIN)
                GUI_DrawMainScreen();
            else
                GUI_DrawHistoryScreen();

            /* 重绘后重载LTDC寄存器，防止长时间运行后同步漂移 */
            LTDC_ReloadConfig(LTDC_IMReload);
        }
    }
}

/* ── BSP_Init ──────────────────────────────────── */
static void BSP_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    LED_GPIO_Config();
    Debug_USART_Config();
    DHT11_GPIO_Config();
    RS485_Config();

    /* RS485 接收队列 */
    RS485_RxQueue = xQueueCreate(256, sizeof(uint8_t));

    /* 触摸事件队列 */
    xTouchQueue = xQueueCreate(10, sizeof(TouchEvent_t));

    /* 触摸信号量（ISR → 任务） */
    xTouchSemaphore = xSemaphoreCreateBinary();
    if (xTouchSemaphore == NULL)
        printf("[ERROR] Touch semaphore!\r\n");

    /* LCD 初始化 */
    LCD_Init();
    LCD_LayerInit();
    LTDC_Cmd(ENABLE);
    LCD_SetLayer(LCD_BACKGROUND_LAYER);
    LCD_Clear(LCD_COLOR_BLACK);
    LCD_SetLayer(LCD_FOREGROUND_LAYER);
    LCD_SetTransparency(0xFF);
    LCD_Clear(LCD_COLOR_BLACK);
    LCD_SetColors(LCD_COLOR_WHITE, LCD_COLOR_BLACK);
    LCD_SetFont(&Font16x24);

    /* 触摸屏初始化（之后 EXTI 下降沿中断进入就绪状态） */
    if (GTP_Init_Panel() != 0)
        printf("[WARN] Touch init failed!\r\n");
    else
        printf("[OK] Touch panel ready\r\n");

    LED_BLUE;
}

/* ── 记录传感器数据 ───────────────────────────── */
static void DHT11_Record(uint8_t humidity, uint8_t temperature, uint8_t valid)
{
    DHT11_History[DHT11_HistoryIndex].humidity    = humidity;
    DHT11_History[DHT11_HistoryIndex].temperature = temperature;
    DHT11_History[DHT11_HistoryIndex].valid       = valid;
    DHT11_History[DHT11_HistoryIndex].timestamp   = xTaskGetTickCount();

    DHT11_HistoryIndex = (DHT11_HistoryIndex + 1) % DHT11_HISTORY_COUNT;

    if (DHT11_HistoryCount < DHT11_HISTORY_COUNT)
        DHT11_HistoryCount++;

    DHT11_SampleCount++;
}

/* ── 判断触摸是否在按钮区域内 ──────────────────── */
static uint8_t GUI_IsTouchInButton(TouchEvent_t *e)
{
    return (e->x >= BTN_X1 && e->x <= BTN_X2 &&
            e->y >= BTN_Y1 && e->y <= BTN_Y2) ? 1 : 0;
}

/*
 * 以下所有显示函数使用 LCD_DisplayStringLine(像素Y, 文字)
 * Y 坐标为硬编码像素值，不依赖 LINE 宏（避免字体切换时漂移）。
 * 全程使用 Font16x24，不再混用字体。
 */

/* ── 绘制触摸按钮 ─────────────────────────────── */
static void GUI_DrawButton(void)
{
    /* 按钮底色 (BTN_X1, BTN_Y1, BTN_WIDTH, BTN_HEIGHT) */
    LCD_SetColors(LCD_COLOR_WHITE, LCD_COLOR_BLUE);
    LCD_DrawFullRect(BTN_X1, BTN_Y1, BTN_WIDTH, BTN_HEIGHT);

    /* 按钮边框 */
    LCD_SetColors(LCD_COLOR_WHITE, LCD_COLOR_BLUE);
    LCD_DrawRect(BTN_X1, BTN_Y1, BTN_WIDTH, BTN_HEIGHT);

    /* 文字居中 */
    {
        const char *text = (screen_mode == SCREEN_MAIN) ? " History >>" : "  << Back";
        LCD_SetColors(LCD_COLOR_WHITE, LCD_COLOR_BLUE);
        LCD_SetFont(&Font16x24);
        LCD_DisplayStringLine(BTN_Y1 + 16, (uint8_t *)text);
    }
}

/* ── 主界面 ────────────────────────────────────── */
static void GUI_DrawMainScreen(void)
{
    char buf[50];
    TickType_t now = xTaskGetTickCount();
    uint32_t seconds_ago;
    uint16_t y;

    LCD_SetLayer(LCD_FOREGROUND_LAYER);
    LCD_Clear(LCD_COLOR_BLACK);
    LCD_SetFont(&Font16x24);

    /* 标题栏: Y=0 */
    LCD_SetColors(LCD_COLOR_WHITE, LCD_COLOR_BLUE2);
    LCD_DrawFullRect(0, 0, 800, 30);
    LCD_SetColors(LCD_COLOR_WHITE, LCD_COLOR_BLUE2);
    LCD_DisplayStringLine(0, (uint8_t *)"  = DHT11 Temperature & Humidity  #02 =");

    /* 温度: Y=80 */
    LCD_SetColors(LCD_COLOR_WHITE, LCD_COLOR_BLACK);
    sprintf(buf, "       Temperature :  %2d  C", g_CurrentTemperature);
    LCD_DisplayStringLine(80, (uint8_t *)buf);

    /* 湿度: Y=120 */
    sprintf(buf, "        Humidity  :  %2d  %%", g_CurrentHumidity);
    LCD_DisplayStringLine(120, (uint8_t *)buf);

    /* 分隔线: Y=180 */
    LCD_SetColors(LCD_COLOR_GREY, LCD_COLOR_BLACK);
    LCD_DrawFullRect(60, 180, 680, 2);

    /* 状态: Y=210 */
    y = 210;
    if (DHT11_HistoryCount > 0) {
        uint8_t lastIdx = (DHT11_HistoryIndex + DHT11_HISTORY_COUNT - 1) % DHT11_HISTORY_COUNT;
        if (DHT11_History[lastIdx].valid) {
            LCD_SetColors(LCD_COLOR_GREEN, LCD_COLOR_BLACK);
            sprintf(buf, "  Status: OK    Samples: %lu", DHT11_SampleCount);
        } else {
            LCD_SetColors(LCD_COLOR_YELLOW, LCD_COLOR_BLACK);
            sprintf(buf, "  Status: Last read failed");
        }
        LCD_DisplayStringLine(y, (uint8_t *)buf);

        /* 更新时间: Y=248 */
        y = 248;
        LCD_SetColors(LCD_COLOR_GREY, LCD_COLOR_BLACK);
        seconds_ago = (now - DHT11_History[lastIdx].timestamp) / configTICK_RATE_HZ;
        if (seconds_ago < 60)
            sprintf(buf, "  Updated: %lu sec ago", seconds_ago);
        else
            sprintf(buf, "  Updated: %lu min %lu sec ago", seconds_ago / 60, seconds_ago % 60);
        LCD_DisplayStringLine(y, (uint8_t *)buf);
    } else {
        LCD_SetColors(LCD_COLOR_YELLOW, LCD_COLOR_BLACK);
        LCD_DisplayStringLine(y, (uint8_t *)"  Waiting for first sample...");
    }

    /* 按钮: Y=340~400, 提示: Y=430 */
    GUI_DrawButton();

    LCD_SetColors(LCD_COLOR_GREY, LCD_COLOR_BLACK);
    LCD_DisplayStringLine(430, (uint8_t *)"  [ Touch BLUE button to view History ]");
}

/* ── 历史记录界面 ──────────────────────────────── */
static void GUI_DrawHistoryScreen(void)
{
    char buf[60];
    uint8_t dispIdx, recIdx, startIdx;
    TickType_t now = xTaskGetTickCount();
    uint32_t sec;
    uint16_t y;

    LCD_SetLayer(LCD_FOREGROUND_LAYER);
    LCD_Clear(LCD_COLOR_BLACK);
    LCD_SetFont(&Font16x24);

    /* 标题栏: Y=0 */
    LCD_SetColors(LCD_COLOR_WHITE, LCD_COLOR_BLUE2);
    LCD_DrawFullRect(0, 0, 800, 30);
    LCD_SetColors(LCD_COLOR_WHITE, LCD_COLOR_BLUE2);
    LCD_DisplayStringLine(0, (uint8_t *)"  = History Records (latest 10) =");

    /* 表头: Y=48 */
    LCD_SetColors(LCD_COLOR_CYAN, LCD_COLOR_BLACK);
    LCD_DisplayStringLine(48, (uint8_t *)"  No  Humi   Temp      When");

    /* 10 条记录: Y=72 + n*24 */
    if (DHT11_HistoryCount == DHT11_HISTORY_COUNT)
        startIdx = DHT11_HistoryIndex;
    else
        startIdx = 0;

    for (dispIdx = 0; dispIdx < DHT11_HISTORY_COUNT; dispIdx++)
    {
        y = 72 + dispIdx * 24;
        recIdx = (startIdx + dispIdx) % DHT11_HISTORY_COUNT;

        if (dispIdx < DHT11_HistoryCount)
        {
            if (DHT11_History[recIdx].valid)
            {
                sec = (now - DHT11_History[recIdx].timestamp) / configTICK_RATE_HZ;
                if (sec < 60)
                    sprintf(buf, "%2d:   %3d%%    %2dC     %lus ago",
                            dispIdx + 1, DHT11_History[recIdx].humidity,
                            DHT11_History[recIdx].temperature, sec);
                else if (sec < 3600)
                    sprintf(buf, "%2d:   %3d%%    %2dC     %lum%lus ago",
                            dispIdx + 1, DHT11_History[recIdx].humidity,
                            DHT11_History[recIdx].temperature, sec / 60, sec % 60);
                else
                    sprintf(buf, "%2d:   %3d%%    %2dC     %luh%lum ago",
                            dispIdx + 1, DHT11_History[recIdx].humidity,
                            DHT11_History[recIdx].temperature,
                            sec / 3600, (sec % 3600) / 60);
                LCD_SetColors(LCD_COLOR_WHITE, LCD_COLOR_BLACK);
            }
            else
            {
                sprintf(buf, "%2d:    ERR       --       (failed)", dispIdx + 1);
                LCD_SetColors(LCD_COLOR_RED, LCD_COLOR_BLACK);
            }
        }
        else
        {
            sprintf(buf, "%2d:    ---       --       (empty)", dispIdx + 1);
            LCD_SetColors(LCD_COLOR_GREY, LCD_COLOR_BLACK);
        }
        LCD_DisplayStringLine(y, (uint8_t *)buf);
    }

    /* 按钮: Y=340~400, 提示: Y=430 */
    GUI_DrawButton();

    LCD_SetColors(LCD_COLOR_GREY, LCD_COLOR_BLACK);
    LCD_DisplayStringLine(430, (uint8_t *)"  [ Touch BLUE button to return to Main ]");
}
