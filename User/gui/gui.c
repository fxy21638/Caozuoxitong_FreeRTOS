/* ========================================================================== */
/*  gui.c �?Manager LCD GUI (left-right card layout + blue-white theme)       */
/*                                                                             */
/*  Layout: Title | DHT card (L) + MPU card (R) | History (L+R cols) | Status  */
/*  Theme:  Blue-white, clean modern look                                       */
/*  Input:  KEY1(PA0)→DHT  KEY2(PC13)→MPU  非阻塞消�?                          */
/* ========================================================================== */

#include "./gui.h"
#include "../lcd/bsp_lcd.h"
#include "../key/bsp_key.h"
#include "../protocol/protocol.h"
#include "../config.h"
#include "../touch/gt9xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>

/* ========================================================================== */
/*  Blue-White Theme Palette (ARGB1555)                                        */
/*  ARGB1555: bit15 = 0=不透明, 1=透明                                         */
/* ========================================================================== */
#define COL_BLACK 0x0000   /* 不透明�?(0x8000 = 透明�? 不能�?              */
#define COL_WHITE 0x7FFF   /* 不透明�?                                      */
#define COL_BLUE  0x001F   /* 不透明�?(bit15=0)                            */
#define COL_LBLUE 0x1D1F   /* 不透明淡蓝                                     */
#define COL_PALE  0x4EBF   /* 不透明淡色 (用于历史行交�?                    */
#define COL_DIM   0x630C   /* 暗灰 (非透明)                                  */
#define COL_DATA  0x001F   /* 数据�?= �?                                   */
#define COL_GREEN 0x03E0   /* 绿色                                           */

/* ========================================================================== */
/*  Layout geometry (pixels)                                                   */
/* ========================================================================== */
#define TITLE_Y 0
#define TITLE_H 40

#define HDR_Y 40
#define HDR_H 28
#define BODY_Y 68
#define BODY_H 188
#define BODY_END (BODY_Y + BODY_H)

#define DIV_X 397
#define DIV_W 6

#define L_X 12
#define R_X 416
#define LBL_X 12
#define VAL_X 180

#define HIST_HDR_Y BODY_END
#define HIST_HDR_H 28
#define HIST_BODY_Y (HIST_HDR_Y + HIST_HDR_H)
#define HIST_BODY_H 144
#define HIST_PER_COL 6

#define STAT_Y (HIST_BODY_Y + HIST_BODY_H)
#define STAT_H 52

/* ---- Touch buttons (centered in each card) ---- */
#define BTN_H     28
#define BTN_DHT_X 134
#define BTN_DHT_Y 218
#define BTN_DHT_W 140
#define BTN_MPU_X 532
#define BTN_MPU_Y 218
#define BTN_MPU_W 140
#define TOUCH_COOLDOWN_MS  400

#define KEY_DEBOUNCE_MS    40
#define KEY_LONG_PRESS_MS  500

/* ========================================================================== */
/*  History entry                                                              */
/* ========================================================================== */
typedef struct
{
    GUISensorType_t type;
    TickType_t tick;
    union
    {
        struct
        {
            float temp;
            float humi;
        } dht;
        struct
        {
            float pitch;
            float roll;
            float yaw;
        } mpu;
    } data;
} HistoryEntry_t;

typedef struct
{
    uint8_t dest_addr;
    uint8_t msg_type;
    uint8_t data[RS485_MAX_PAYLOAD];
    uint8_t data_len;
} RS485TxRequest_t;

extern QueueHandle_t g_tx_queue;
extern QueueHandle_t xTouchQueue;
extern QueueHandle_t xTouchSemaphore;
typedef struct { int32_t x; int32_t y; } TouchEvent_t;

/* ========================================================================== */
/*  GUI internal state                                                         */
/* ========================================================================== */
static HistoryEntry_t g_history[GUI_HISTORY_DEPTH];
static uint8_t g_hist_idx;
static uint32_t g_update_count;
static QueueHandle_t g_gui_queue;

static uint8_t g_dht_valid, g_mpu_valid;
static float g_dht_temp, g_dht_humi;
static float g_mpu_pitch, g_mpu_roll, g_mpu_yaw;
static uint32_t g_dht_tick, g_mpu_tick;

static char g_buf[64];

/* ========================================================================== */
/*  Helpers                                                                    */
/* ========================================================================== */
static void FillR(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t c)
{
    LCD_SetTextColor(c);
    LCD_DrawFullRect(x, y, w, h);
}

static void Put(uint16_t x, uint16_t y, uint16_t fg, uint16_t bg, const char *s)
{
    LCD_SetTextColor(fg);
    LCD_SetBackColor(bg);
    /* LCD_DispString_EN_CH(Line=y, Column=x, str) */
    LCD_DispString_EN_CH(y, x, (const uint8_t *)s);
}

/* ========================================================================== */
/*  Draw solid button                                                            */
/* ========================================================================== */
static void DrawButton(int x, int y, int w, int h, uint16_t fill)
{
    LCD_SetTextColor(fill);
    LCD_DrawFullRect(x, y, w, h);
}

/* ========================================================================== */
/*  Draw touch buttons                                                          */
/* ========================================================================== */
static void DrawButtons(void)
{
    DrawButton(BTN_DHT_X, BTN_DHT_Y, BTN_DHT_W, BTN_H, COL_BLUE);
    /* 8x16 字体: 8 字符=64px, 按钮宽140 → (140-64)/2=38; 按钮高28 → (28-16)/2=6 */
    Put(BTN_DHT_X + 38, BTN_DHT_Y + 6, COL_WHITE, COL_BLUE, "REQ  DHT");
    DrawButton(BTN_MPU_X, BTN_MPU_Y, BTN_MPU_W, BTN_H, COL_BLUE);
    Put(BTN_MPU_X + 38, BTN_MPU_Y + 6, COL_WHITE, COL_BLUE, "REQ  MPU");
}

/* ========================================================================== */
/*  Touch handler                                                               */
/* ========================================================================== */
static void HandleTouch(uint16_t x, uint16_t y)
{
    static uint32_t last_touch_tick = 0;
    uint32_t now = xTaskGetTickCount();

    /* 消抖: 400ms 内忽略重复触摸 */
    if ((now - last_touch_tick) < pdMS_TO_TICKS(TOUCH_COOLDOWN_MS)) return;
    last_touch_tick = now;

    RS485TxRequest_t req;
    if (x >= BTN_DHT_X && x < BTN_DHT_X + BTN_DHT_W &&
        y >= BTN_DHT_Y && y < BTN_DHT_Y + BTN_H) {
        printf("[TOUCH] DHT\n");
        req.dest_addr = ADDR_COLLECTOR_1;
        req.msg_type  = MSG_TYPE_TEMP_HUMI;
        req.data_len  = 0;
        xQueueSend(g_tx_queue, &req, 0);
    } else if (x >= BTN_MPU_X && x < BTN_MPU_X + BTN_MPU_W &&
               y >= BTN_MPU_Y && y < BTN_MPU_Y + BTN_H) {
        printf("[TOUCH] MPU\n");
        req.dest_addr = ADDR_COLLECTOR_2;
        req.msg_type  = MSG_TYPE_MPU6050;
        req.data_len = 0;
        xQueueSend(g_tx_queue, &req, 0);
    }
}

static void QueueSensorRequest(uint8_t dest_addr, uint8_t msg_type)
{
    RS485TxRequest_t req;

    req.dest_addr = dest_addr;
    req.msg_type  = msg_type;
    req.data_len  = 0;
    xQueueSend(g_tx_queue, &req, 0);
}

/* ========================================================================== */
/*  Draw the static UI once at startup                                         */
/* ========================================================================== */
static void DrawStaticUI(void)
{
    FillR(0, TITLE_Y, LCD_PIXEL_WIDTH, TITLE_H, COL_BLUE);
    Put(12, 8, COL_WHITE, COL_BLUE, "***  RS485 Manager  [0x01]  ***");

    FillR(0, HDR_Y, LCD_PIXEL_WIDTH, HDR_H, COL_BLUE);
    Put(L_X, HDR_Y + 4, COL_WHITE, COL_BLUE, "DHT22  T/H  0x02");
    Put(R_X, HDR_Y + 4, COL_WHITE, COL_BLUE, "MPU6050  Angle  0x03");

    FillR(0, BODY_Y, DIV_X, BODY_H, COL_LBLUE);
    FillR(DIV_X + DIV_W, BODY_Y, LCD_PIXEL_WIDTH - DIV_X - DIV_W, BODY_H, COL_LBLUE);
    FillR(DIV_X, BODY_Y, DIV_W, BODY_H, COL_BLUE);

    /* Left card labels */
    Put(L_X + LBL_X, 80, COL_BLACK, COL_LBLUE, "Temp (C) :");
    Put(L_X + LBL_X, 108, COL_BLACK, COL_LBLUE, "Humi (%) :");
    Put(L_X + LBL_X, 136, COL_BLACK, COL_LBLUE, "Status      :");
    Put(L_X + LBL_X, 164, COL_BLACK, COL_LBLUE, "Updated     :");

    /* Right card labels */
    Put(R_X + LBL_X, 80, COL_BLACK, COL_LBLUE, "Pitch :");
    Put(R_X + LBL_X, 108, COL_BLACK, COL_LBLUE, "Roll  :");
    Put(R_X + LBL_X, 136, COL_BLACK, COL_LBLUE, "Yaw   :");
    Put(R_X + LBL_X, 164, COL_BLACK, COL_LBLUE, "Status      :");
    Put(R_X + LBL_X, 192, COL_BLACK, COL_LBLUE, "Updated     :");

    /* Placeholder values */
    Put(L_X + VAL_X, 80, COL_DIM, COL_LBLUE, "---.-");
    Put(L_X + VAL_X, 108, COL_DIM, COL_LBLUE, "---.-");
    Put(L_X + VAL_X, 136, COL_DIM, COL_LBLUE, "Wait...");
    Put(L_X + VAL_X, 164, COL_DIM, COL_LBLUE, "--     ");

    Put(R_X + VAL_X, 80, COL_DIM, COL_LBLUE, "---.-");
    Put(R_X + VAL_X, 108, COL_DIM, COL_LBLUE, "---.-");
    Put(R_X + VAL_X, 136, COL_DIM, COL_LBLUE, "---.-");
    Put(R_X + VAL_X, 164, COL_DIM, COL_LBLUE, "Wait...");
    Put(R_X + VAL_X, 192, COL_DIM, COL_LBLUE, "--     ");

    /* (按键提示已删�? */

    /* Touch buttons */
    DrawButtons();

    /* History section */
    FillR(0, HIST_HDR_Y, LCD_PIXEL_WIDTH, HIST_HDR_H, COL_BLUE);
    Put(12, HIST_HDR_Y + 4, COL_WHITE, COL_BLUE,
        "History  (Recent 6 per sensor)");
    FillR(0, HIST_BODY_Y, LCD_PIXEL_WIDTH, HIST_BODY_H, COL_WHITE);

    /* Status bar */
    FillR(0, STAT_Y, LCD_PIXEL_WIDTH, STAT_H, COL_BLUE);
    Put(12, STAT_Y + 12, COL_WHITE, COL_BLUE,
        "Updates: 0  |  DHT: --   MPU: --");
}

/* ========================================================================== */
/*  Refresh sensor data area                                                   */
/* ========================================================================== */
static void RefreshSensors(void)
{
    uint32_t now = xTaskGetTickCount();
    uint32_t age;
    uint16_t fg;

    if (g_dht_valid)
    {
        age = (now - g_dht_tick) * portTICK_PERIOD_MS / 1000;
        fg = COL_DATA;
        snprintf(g_buf, sizeof(g_buf), "%.1f", g_dht_temp);
        Put(L_X + VAL_X, 80, fg, COL_LBLUE, g_buf);
        snprintf(g_buf, sizeof(g_buf), "%.1f", g_dht_humi);
        Put(L_X + VAL_X, 108, fg, COL_LBLUE, g_buf);
        Put(L_X + VAL_X, 136, COL_GREEN, COL_LBLUE, "Online");
        snprintf(g_buf, sizeof(g_buf), "%lus", age);
        Put(L_X + VAL_X, 164, fg, COL_LBLUE, g_buf);
    }
    else
    {
        Put(L_X + VAL_X, 80, COL_DIM, COL_LBLUE, "---");
        Put(L_X + VAL_X, 108, COL_DIM, COL_LBLUE, "---");
        Put(L_X + VAL_X, 136, COL_DIM, COL_LBLUE, "--");
        Put(L_X + VAL_X, 164, COL_DIM, COL_LBLUE, "--");
    }

    if (g_mpu_valid)
    {
        age = (now - g_mpu_tick) * portTICK_PERIOD_MS / 1000;
        fg = COL_DATA;
        snprintf(g_buf, sizeof(g_buf), "%.1f", g_mpu_pitch);
        Put(R_X + VAL_X, 80, fg, COL_LBLUE, g_buf);
        snprintf(g_buf, sizeof(g_buf), "%.1f", g_mpu_roll);
        Put(R_X + VAL_X, 108, fg, COL_LBLUE, g_buf);
        snprintf(g_buf, sizeof(g_buf), "%.1f", g_mpu_yaw);
        Put(R_X + VAL_X, 136, fg, COL_LBLUE, g_buf);
        Put(R_X + VAL_X, 164, COL_GREEN, COL_LBLUE, "Online");
        snprintf(g_buf, sizeof(g_buf), "%lus", age);
        Put(R_X + VAL_X, 192, fg, COL_LBLUE, g_buf);
    }
    else
    {
        Put(R_X + VAL_X, 80, COL_DIM, COL_LBLUE, "---");
        Put(R_X + VAL_X, 108, COL_DIM, COL_LBLUE, "---");
        Put(R_X + VAL_X, 136, COL_DIM, COL_LBLUE, "---");
        Put(R_X + VAL_X, 164, COL_DIM, COL_LBLUE, "--");
        Put(R_X + VAL_X, 192, COL_DIM, COL_LBLUE, "--");
    }

    FillR(DIV_X, BODY_Y, DIV_W, BODY_H, COL_BLUE);
}

/* ========================================================================== */
/*  Refresh history                                                             */
/* ========================================================================== */
static void RefreshHistory(void)
{
    uint8_t i, idx, dht_cnt = 0, mpu_cnt = 0, entry;
    uint16_t line_y, bg;
    uint32_t now = xTaskGetTickCount();
    uint32_t age;

    for (i = 0; i < GUI_HISTORY_DEPTH && (dht_cnt < HIST_PER_COL || mpu_cnt < HIST_PER_COL); i++)
    {
        idx = (g_hist_idx - 1 - i + GUI_HISTORY_DEPTH) % GUI_HISTORY_DEPTH;
        if (g_history[idx].tick == 0)
            break;

        age = (now - g_history[idx].tick) * portTICK_PERIOD_MS / 1000;

        if (g_history[idx].type == GUI_SENSOR_DHT && dht_cnt < HIST_PER_COL)
        {
            entry = dht_cnt + 1;
            line_y = HIST_BODY_Y + dht_cnt * 24;
            bg = (dht_cnt & 1) ? COL_WHITE : COL_PALE;
            snprintf(g_buf, sizeof(g_buf), "%2d %4.1fC %4.1f%% %3lus",
                     entry, g_history[idx].data.dht.temp,
                     g_history[idx].data.dht.humi, age);
            Put(L_X, line_y, COL_DATA, bg, g_buf);
            dht_cnt++;
        }
        else if (g_history[idx].type == GUI_SENSOR_MPU6050 && mpu_cnt < HIST_PER_COL)
        {
            entry = mpu_cnt + 1;
            line_y = HIST_BODY_Y + mpu_cnt * 24;
            bg = (mpu_cnt & 1) ? COL_WHITE : COL_PALE;
            snprintf(g_buf, sizeof(g_buf), "%2d %4.0f %4.0f %4.0f %3lus",
                     entry, g_history[idx].data.mpu.pitch,
                     g_history[idx].data.mpu.roll,
                     g_history[idx].data.mpu.yaw, age);
            Put(R_X, line_y, COL_DATA, bg, g_buf);
            mpu_cnt++;
        }
    }

    for (i = dht_cnt; i < HIST_PER_COL; i++)
    {
        line_y = HIST_BODY_Y + i * 24;
        bg = (i & 1) ? COL_WHITE : COL_PALE;
        snprintf(g_buf, sizeof(g_buf), "%2d  ---", i + 1);
        Put(L_X, line_y, COL_DIM, bg, g_buf);
    }
    for (i = mpu_cnt; i < HIST_PER_COL; i++)
    {
        line_y = HIST_BODY_Y + i * 24;
        bg = (i & 1) ? COL_WHITE : COL_PALE;
        snprintf(g_buf, sizeof(g_buf), "%2d  ---", i + 1);
        Put(R_X, line_y, COL_DIM, bg, g_buf);
    }

    FillR(DIV_X, HIST_BODY_Y, DIV_W, HIST_BODY_H, COL_BLUE);
}

static void RefreshStatusBar(void)
{
    snprintf(g_buf, sizeof(g_buf),
             "Updates:%-4lu | DHT:%-3s  MPU:%-3s",
             g_update_count,
             g_dht_valid ? "OK" : "--",
             g_mpu_valid ? "OK" : "--");
    Put(12, STAT_Y + 12, COL_WHITE, COL_BLUE, g_buf);
}

static void HistoryAdd(const GUIMsg_t *msg)
{
    g_history[g_hist_idx].type = msg->type;
    g_history[g_hist_idx].tick = xTaskGetTickCount();
    if (msg->type == GUI_SENSOR_DHT)
    {
        g_history[g_hist_idx].data.dht.temp = msg->data.dht.temp;
        g_history[g_hist_idx].data.dht.humi = msg->data.dht.humi;
    }
    else
    {
        g_history[g_hist_idx].data.mpu.pitch = msg->data.mpu.pitch;
        g_history[g_hist_idx].data.mpu.roll = msg->data.mpu.roll;
        g_history[g_hist_idx].data.mpu.yaw = msg->data.mpu.yaw;
    }
    g_hist_idx = (g_hist_idx + 1) % GUI_HISTORY_DEPTH;
}

/* ========================================================================== */
/*  GUI Task �?非阻塞按�?+ 传感器显�?+ 历史记录                                */
/*                                                                             */
/*  按键检�? �?20ms 读一�?GPIO, 下降�?(1�?) 触发, 无阻�?                   */
/*  KEY1(PA0) �?请求 DHT    KEY2(PC13) �?请求 MPU                              */
/* ========================================================================== */
void GUI_Task(void *pvParameters)
{
    GUIMsg_t msg;
    g_gui_queue = (QueueHandle_t)pvParameters;

    LCD_SetLayer(LCD_FOREGROUND_LAYER);
    LCD_Clear(COL_BLACK);

    /* 按键消抖: KEY1=PA0 短按DHT/长按MPU, KEY2=PC13 仅诊�?*/
    /* KEY1: short press DHT, long press MPU. Active level follows KEY_ON. */
    uint8_t  k1_raw = GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_PIN);
    uint8_t  k1_last_raw = k1_raw;
    uint8_t  k1_stable = k1_raw;
    uint32_t k1_raw_change_tick = 0;

    uint8_t  k2_raw = GPIO_ReadInputDataBit(KEY2_GPIO_PORT, KEY2_PIN);
    uint8_t  k2_last_raw = k2_raw;
    uint8_t  k2_stable = k2_raw;
    uint32_t k2_raw_change_tick = 0;

    /* 触摸屏初始化已禁�?�?软件 I2C 操作 PA8/PC9 + I2C3 干扰 LTDC 导致全黑 */
    // Touch_Init();

    DrawStaticUI();

    while (1)
    {
        TickType_t now = xTaskGetTickCount();
        uint8_t k1 = GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_PIN);
        uint8_t k2 = GPIO_ReadInputDataBit(KEY2_GPIO_PORT, KEY2_PIN);

        if (k1 != k1_last_raw) {
            k1_last_raw = k1;
            k1_raw_change_tick = now;
        }

        if ((k1 != k1_stable) &&
            ((now - k1_raw_change_tick) >= pdMS_TO_TICKS(KEY_DEBOUNCE_MS))) {
            k1_stable = k1;
            if (k1_stable == KEY_ON) {
                /* KEY1 press -> DHT */
                printf("[KEY] KEY1 pressed -> request DHT\n");
                QueueSensorRequest(ADDR_COLLECTOR_1, MSG_TYPE_TEMP_HUMI);
            }
        }

        /* KEY2 (PC13) �?请求 MPU6050 (短按 50ms 消抖) */
        if (k2 != k2_last_raw) {
            k2_last_raw = k2;
            k2_raw_change_tick = now;
        }
        if ((k2 != k2_stable) &&
            ((now - k2_raw_change_tick) >= pdMS_TO_TICKS(KEY_DEBOUNCE_MS))) {
            k2_stable = k2;
            if (k2_stable == KEY_ON) {
                printf("[KEY] KEY2 pressed -> request MPU\n");
                QueueSensorRequest(ADDR_COLLECTOR_2, MSG_TYPE_MPU6050);
            }
        }

        /* 非阻塞排空传感器数据队列 */
        while (xQueueReceive(g_gui_queue, &msg, 0) == pdPASS)
        {
            if (msg.type == GUI_SENSOR_DHT)
            {
                g_dht_valid = 1;
                g_dht_temp = msg.data.dht.temp;
                g_dht_humi = msg.data.dht.humi;
                g_dht_tick = xTaskGetTickCount();
            }
            else
            {
                g_mpu_valid = 1;
                g_mpu_pitch = msg.data.mpu.pitch;
                g_mpu_roll = msg.data.mpu.roll;
                g_mpu_yaw = msg.data.mpu.yaw;
                g_mpu_tick = xTaskGetTickCount();
            }
            g_update_count++;
            HistoryAdd(&msg);
        }

        /* 触摸屏已禁用 �?�?GUI_Task 入口 */
        if (xSemaphoreTake(xTouchSemaphore, 0) == pdTRUE) {
            GTP_TouchProcess();
            TouchEvent_t ev;
            while (xQueueReceive(xTouchQueue, &ev, 0) == pdTRUE) {
                HandleTouch((uint16_t)ev.x, (uint16_t)ev.y);
            }
        }


        RefreshSensors();
        RefreshHistory();
        RefreshStatusBar();

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ========================================================================== */
/*  LCD_TestTask �?测试模式: 绕过 RS485, 每秒�?GUI 队列喂虚拟传感器数据     */
/*  用于在没有采集前端时, 验证 LCD 更新流程                                    */
/* ========================================================================== */
void LCD_TestTask(void *pvParameters)
{
    QueueHandle_t gui_queue = (QueueHandle_t)pvParameters;
    GUIMsg_t msg;
    int cnt = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        cnt++;

        /* DHT 测试: 温度 25.0+cnt*0.1, 湿度 60.0+cnt*0.5 */
        msg.type = GUI_SENSOR_DHT;
        msg.data.dht.temp = 25.0f + (cnt % 100) * 0.1f;
        msg.data.dht.humi = 60.0f + (cnt % 100) * 0.5f;
        xQueueSend(gui_queue, &msg, 0);

        /* MPU 测试: pitch=10+cnt, roll=-20-cnt, yaw=cnt*2 */
        msg.type = GUI_SENSOR_MPU6050;
        msg.data.mpu.pitch = 10.0f + (cnt % 360);
        msg.data.mpu.roll  = -20.0f - (cnt % 360);
        msg.data.mpu.yaw   = (cnt % 360) * 2.0f;
        xQueueSend(gui_queue, &msg, 0);

        printf("[TEST] #%d: DHT=%.1f/%.1f  MPU=%.1f/%.1f/%.1f\n",
               cnt, msg.data.dht.temp, msg.data.dht.humi,
               msg.data.mpu.pitch, msg.data.mpu.roll, msg.data.mpu.yaw);
    }
}
