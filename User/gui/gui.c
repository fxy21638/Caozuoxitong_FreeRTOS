/* ========================================================================== */
/*  gui.c — Manager LCD GUI (left-right card layout + blue-white theme)       */
/*                                                                             */
/*  Layout: Title | DHT card (L) + MPU card (R) | History (L+R cols) | Status  */
/*  Theme:  Blue-white, clean modern look                                       */
/*  Input:  KEY1(PA0)→DHT  KEY2(PC13)→MPU  非阻塞消抖                           */
/* ========================================================================== */

#include "./gui.h"
#include "../lcd/bsp_lcd.h"
#include "../key/bsp_key.h"
#include "../protocol/protocol.h"
#include "../config.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

/* ========================================================================== */
/*  Blue-White Theme Palette (ARGB1555)                                        */
/*  ARGB1555: bit15 = 0=不透明, 1=透明                                         */
/* ========================================================================== */
#define COL_BLACK 0x0000   /* 不透明黑 (0x8000 = 透明黑, 不能用)              */
#define COL_WHITE 0x7FFF   /* 不透明白                                       */
#define COL_BLUE  0x001F   /* 不透明蓝 (bit15=0)                            */
#define COL_LBLUE 0x1D1F   /* 不透明淡蓝                                     */
#define COL_PALE  0x4EBF   /* 不透明淡色 (用于历史行交替)                    */
#define COL_DIM   0x630C   /* 暗灰 (非透明)                                  */
#define COL_DATA  0x001F   /* 数据色 = 蓝                                    */
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
    Put(L_X + LBL_X, 80, COL_BLACK, COL_LBLUE, "Temperature :");
    Put(L_X + LBL_X, 108, COL_BLACK, COL_LBLUE, "Humidity    :");
    Put(L_X + LBL_X, 136, COL_BLACK, COL_LBLUE, "Status      :");
    Put(L_X + LBL_X, 164, COL_BLACK, COL_LBLUE, "Updated     :");

    /* Right card labels */
    Put(R_X + LBL_X, 80, COL_BLACK, COL_LBLUE, "Pitch :");
    Put(R_X + LBL_X, 108, COL_BLACK, COL_LBLUE, "Roll  :");
    Put(R_X + LBL_X, 136, COL_BLACK, COL_LBLUE, "Yaw   :");
    Put(R_X + LBL_X, 164, COL_BLACK, COL_LBLUE, "Status      :");
    Put(R_X + LBL_X, 192, COL_BLACK, COL_LBLUE, "Updated     :");

    /* Placeholder values */
    Put(L_X + VAL_X, 80, COL_DIM, COL_LBLUE, "---.-C ");
    Put(L_X + VAL_X, 108, COL_DIM, COL_LBLUE, "---.-% ");
    Put(L_X + VAL_X, 136, COL_DIM, COL_LBLUE, "Wait...");
    Put(L_X + VAL_X, 164, COL_DIM, COL_LBLUE, "--     ");

    Put(R_X + VAL_X, 80, COL_DIM, COL_LBLUE, "---.-  ");
    Put(R_X + VAL_X, 108, COL_DIM, COL_LBLUE, "---.-  ");
    Put(R_X + VAL_X, 136, COL_DIM, COL_LBLUE, "---.-  ");
    Put(R_X + VAL_X, 164, COL_DIM, COL_LBLUE, "Wait...");
    Put(R_X + VAL_X, 192, COL_DIM, COL_LBLUE, "--     ");

    /* Key hints — show which key does what */
    Put(L_X, BODY_END - 14, COL_BLUE, COL_LBLUE, "KEY1: REQ DHT");
    Put(R_X, BODY_END - 14, COL_BLUE, COL_LBLUE, "KEY2: REQ MPU");

    /* History section */
    FillR(0, HIST_HDR_Y, LCD_PIXEL_WIDTH, HIST_HDR_H, COL_BLUE);
    Put(12, HIST_HDR_Y + 4, COL_WHITE, COL_BLUE,
        "History  (Recent 6 per sensor)");
    FillR(0, HIST_BODY_Y, LCD_PIXEL_WIDTH, HIST_BODY_H, COL_WHITE);

    /* Status bar */
    FillR(0, STAT_Y, LCD_PIXEL_WIDTH, STAT_H, COL_BLUE);
    Put(12, STAT_Y + 12, COL_WHITE, COL_BLUE,
        "Updates: 0  |  DHT: --   MPU: --   |  KEY1/KEY2");
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
        snprintf(g_buf, sizeof(g_buf), "%5.1fC ", g_dht_temp);
        Put(L_X + VAL_X, 80, fg, COL_LBLUE, g_buf);
        snprintf(g_buf, sizeof(g_buf), "%5.1f%% ", g_dht_humi);
        Put(L_X + VAL_X, 108, fg, COL_LBLUE, g_buf);
        Put(L_X + VAL_X, 136, COL_GREEN, COL_LBLUE, "Online ");
        snprintf(g_buf, sizeof(g_buf), "%lus     ", age);
        Put(L_X + VAL_X, 164, fg, COL_LBLUE, g_buf);
    }
    else
    {
        Put(L_X + VAL_X, 80, COL_DIM, COL_LBLUE, "---.-C ");
        Put(L_X + VAL_X, 108, COL_DIM, COL_LBLUE, "---.-% ");
        Put(L_X + VAL_X, 136, COL_DIM, COL_LBLUE, "Wait...");
        Put(L_X + VAL_X, 164, COL_DIM, COL_LBLUE, "--     ");
    }

    if (g_mpu_valid)
    {
        age = (now - g_mpu_tick) * portTICK_PERIOD_MS / 1000;
        fg = COL_DATA;
        snprintf(g_buf, sizeof(g_buf), "%6.1f  ", g_mpu_pitch);
        Put(R_X + VAL_X, 80, fg, COL_LBLUE, g_buf);
        snprintf(g_buf, sizeof(g_buf), "%6.1f  ", g_mpu_roll);
        Put(R_X + VAL_X, 108, fg, COL_LBLUE, g_buf);
        snprintf(g_buf, sizeof(g_buf), "%6.1f  ", g_mpu_yaw);
        Put(R_X + VAL_X, 136, fg, COL_LBLUE, g_buf);
        Put(R_X + VAL_X, 164, COL_GREEN, COL_LBLUE, "Online ");
        snprintf(g_buf, sizeof(g_buf), "%lus     ", age);
        Put(R_X + VAL_X, 192, fg, COL_LBLUE, g_buf);
    }
    else
    {
        Put(R_X + VAL_X, 80, COL_DIM, COL_LBLUE, "---.-  ");
        Put(R_X + VAL_X, 108, COL_DIM, COL_LBLUE, "---.-  ");
        Put(R_X + VAL_X, 136, COL_DIM, COL_LBLUE, "---.-  ");
        Put(R_X + VAL_X, 164, COL_DIM, COL_LBLUE, "Wait...");
        Put(R_X + VAL_X, 192, COL_DIM, COL_LBLUE, "--     ");
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
             "Updates:%-4lu | DHT:%-3s  MPU:%-3s | KEY1/KEY2",
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
/*  GUI Task — 非阻塞按键 + 传感器显示 + 历史记录                                */
/*                                                                             */
/*  按键检测: 每 20ms 读一次 GPIO, 下降沿 (1→0) 触发, 无阻塞                    */
/*  KEY1(PA0) → 请求 DHT    KEY2(PC13) → 请求 MPU                              */
/* ========================================================================== */
void GUI_Task(void *pvParameters)
{
    GUIMsg_t msg;
    g_gui_queue = (QueueHandle_t)pvParameters;

    LCD_SetLayer(LCD_FOREGROUND_LAYER);
    LCD_Clear(COL_BLACK);
    DrawStaticUI();

    /* 按键消抖: 上次电平 (1=按下, 0=未按, 适配野火 KEY_ON=1) */
    uint8_t k1_last = GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_PIN);
    uint8_t k2_last = GPIO_ReadInputDataBit(KEY2_GPIO_PORT, KEY2_PIN);

    while (1)
    {
        uint8_t k1 = GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_PIN);
        uint8_t k2 = GPIO_ReadInputDataBit(KEY2_GPIO_PORT, KEY2_PIN);

        /* KEY1 上升沿 (0→1 = 按下) — 野火 KEY_ON=1, 高电平按下 */
        if (k1 == 1 && k1_last == 0)
        {
            RS485TxRequest_t req;
            printf("[KEY] KEY1 pressed -> request DHT\n");
            req.dest_addr = ADDR_COLLECTOR_1;
            req.msg_type = MSG_TYPE_TEMP_HUMI;
            req.data_len = 0;
            xQueueSend(g_tx_queue, &req, 0);
        }
        k1_last = k1;

        /* KEY2 上升沿 */
        if (k2 == 1 && k2_last == 0)
        {
            RS485TxRequest_t req;
            printf("[KEY] KEY2 pressed -> request MPU\n");
            req.dest_addr = ADDR_COLLECTOR_2;
            req.msg_type = MSG_TYPE_MPU6050;
            req.data_len = 0;
            xQueueSend(g_tx_queue, &req, 0);
        }
        k2_last = k2;

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

        RefreshSensors();
        RefreshHistory();
        RefreshStatusBar();

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ========================================================================== */
/*  LCD_TestTask — 测试模式: 绕过 RS485, 每秒向 GUI 队列喂虚拟传感器数据     */
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
