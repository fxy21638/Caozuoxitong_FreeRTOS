/* ========================================================================== */
/*  gui.c  ?Manager LCD GUI (left-right card layout + blue-white theme)       */
/*                                                                             */
/*  Layout: Title | DHT card (L) + MPU card (R) | History (L+R cols) | Status  */
/*  Theme:  Blue-white, clean modern look                                       */
/*  Input:  KEY1(PA0)→DHT  KEY2(PC13)→MPU  非阻塞消 ?                          */
/* ========================================================================== */

#include "./gui.h"
#include "../lcd/bsp_lcd.h"
#include "../key/bsp_key.h"
#include "../protocol/protocol.h"
#include "../config.h"
#include "../led/bsp_led.h"
#include "../touch/gt9xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>

extern void GTP_TouchProcess(void);

/* ========================================================================== */
/*  Minimalist Light Theme Palette (RGB565)                                    */
/* ========================================================================== */
#define COL_BG 0xE75E       /* #E5E9F0 Morandi Light Blue-Gray Background */
#define COL_CARD 0xFFFF     /* #FFFFFF White Card */
#define COL_BORDER 0xBDF7   /* #B0B0B0 Medium Gray Border for higher contrast */
#define COL_TEXT_PRI 0x2988 /* #2D3142 Dark Slate Text */
#define COL_TEXT_SEC 0x8C94 /* #8C92A6 Secondary Gray Text */
#define COL_ACCENT 0x03DF   /* #007AFF iOS Blue Accent */
#define COL_GREEN 0x362B    /* #34C759 Green Status */
#define COL_RED 0xF9C6      /* #FF3B30 Red Status */
#define COL_PALE 0xF79E     /* #F0F0F3 Alternate Row in history list */
#define COL_WHITE 0xFFFF    /* Pure White */
#define COL_BLACK 0x0000    /* Pure Black */
#define COL_TAB_ACT 0xE79F  /* #E6F2FF Light Blue for active tab */
#define COL_TAB_INA 0xEFBE  /* #F5F5F7 Light Gray for inactive tab */

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
#define HIST_PER_COL 5

#define STAT_Y (HIST_BODY_Y + HIST_BODY_H)
#define STAT_H 52

/* ---- Touch buttons (centered in each card) ---- */
#define BTN_H 28
#define BTN_DHT_X 24
#define BTN_DHT_Y 218
#define BTN_DHT_W 140
#define BTN_MPU_X (DIV_X + 18)
#define BTN_MPU_Y 218
#define BTN_MPU_W 140
#define TOUCH_COOLDOWN_MS 400

#define KEY_DEBOUNCE_MS 40
#define KEY_LONG_PRESS_MS 500

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
typedef struct
{
    int32_t x;
    int32_t y;
} TouchEvent_t;

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

static uint8_t g_current_page = 0; /* 0: Dashboard, 1: DHT22, 2: MPU6050 */

static float g_dht_temp_max = -999.0f, g_dht_temp_min = 999.0f;
static float g_dht_humi_max = -999.0f, g_dht_humi_min = 999.0f;
static float g_mpu_pitch_max = -999.0f, g_mpu_pitch_min = 999.0f;
static float g_mpu_roll_max = -999.0f, g_mpu_roll_min = 999.0f;
static float g_mpu_yaw_max = -999.0f, g_mpu_yaw_min = 999.0f;

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

static void DrawMenuBar(void)
{
    /* Fill bottom menu background with Morandi background for visual contrast */
    FillR(0, 428, LCD_PIXEL_WIDTH, 52, COL_BG);
    FillR(0, 428, LCD_PIXEL_WIDTH, 1, COL_BORDER);

    int tab_w = 246;
    int tab_h = 36;
    int tab_y = 436;
    int x0 = 12;
    int x1 = 277;
    int x2 = 542;

    /* Tab 0: Dashboard */
    uint8_t act0 = (g_current_page == 0);
    uint16_t bg0 = act0 ? COL_TAB_ACT : COL_TAB_INA;
    uint16_t border0 = act0 ? COL_ACCENT : COL_BORDER;
    uint16_t fg0 = act0 ? COL_ACCENT : COL_TEXT_SEC;
    FillR(x0, tab_y, tab_w, tab_h, bg0);
    LCD_SetTextColor(border0);
    LCD_DrawRect(x0, tab_y, tab_w, tab_h);
    Put(x0 + 50, tab_y + 6, fg0, bg0, "Dashboard");

    /* Tab 1: DHT22 Detail */
    uint8_t act1 = (g_current_page == 1);
    uint16_t bg1 = act1 ? COL_TAB_ACT : COL_TAB_INA;
    uint16_t border1 = act1 ? COL_ACCENT : COL_BORDER;
    uint16_t fg1 = act1 ? COL_ACCENT : COL_TEXT_SEC;
    FillR(x1, tab_y, tab_w, tab_h, bg1);
    LCD_SetTextColor(border1);
    LCD_DrawRect(x1, tab_y, tab_w, tab_h);
    Put(x1 + 44, tab_y + 6, fg1, bg1, "DHT22");

    /* Tab 2: MPU6050 Detail */
    uint8_t act2 = (g_current_page == 2);
    uint16_t bg2 = act2 ? COL_TAB_ACT : COL_TAB_INA;
    uint16_t border2 = act2 ? COL_ACCENT : COL_BORDER;
    uint16_t fg2 = act2 ? COL_ACCENT : COL_TEXT_SEC;
    FillR(x2, tab_y, tab_w, tab_h, bg2);
    LCD_SetTextColor(border2);
    LCD_DrawRect(x2, tab_y, tab_w, tab_h);
    Put(x2 + 42, tab_y + 6, fg2, bg2, "MPU6050");
}

static void RefreshSensors(void);
static void RefreshHistory(void);
static void DrawStaticUI(void);

static void SwitchPage(uint8_t page)
{
    g_current_page = page;
    LCD_Clear(COL_BG);
    DrawStaticUI();
    RefreshSensors();
    RefreshHistory();
}

/* ========================================================================== */
/*  Draw solid button                                                            */
/* ========================================================================== */
static void DrawButton(int x, int y, int w, int h, uint16_t fill, uint16_t border)
{
    LCD_SetTextColor(fill);
    LCD_DrawFullRect(x, y, w, h);
    LCD_SetTextColor(border);
    LCD_DrawRect(x, y, w, h);
}

/* ========================================================================== */
/*  Draw touch buttons                                                          */
/* ========================================================================== */
static void DrawButtons(void)
{
    /* iOS style filled blue buttons with white text */
    DrawButton(BTN_DHT_X, BTN_DHT_Y, BTN_DHT_W, BTN_H, COL_ACCENT, COL_ACCENT);
    Put(BTN_DHT_X + 14, BTN_DHT_Y + 5, COL_WHITE, COL_ACCENT, "REQ DHT");

    DrawButton(BTN_MPU_X, BTN_MPU_Y, BTN_MPU_W, BTN_H, COL_ACCENT, COL_ACCENT);
    Put(BTN_MPU_X + 14, BTN_MPU_Y + 5, COL_WHITE, COL_ACCENT, "REQ MPU");
}

/* ========================================================================== */
/*  Touch handler                                                               */
/* ========================================================================== */
static void HandleTouch(uint16_t x, uint16_t y)
{
    static uint32_t last_touch_tick = 0;
    uint32_t now = xTaskGetTickCount();

    /* 消抖: 400ms 内忽略重复触摸 */
    if ((now - last_touch_tick) < pdMS_TO_TICKS(TOUCH_COOLDOWN_MS))
        return;
    last_touch_tick = now;

    /* 1. 优先判断底部 Tab 菜单栏的点击 (Y >= 428) */
    if (y >= 428 && y <= 480)
    {
        if (x >= 12 && x <= 258)
        {
            if (g_current_page != 0)
            {
                printf("[TOUCH] Switch to Page 0 (Dashboard)\n");
                SwitchPage(0);
            }
        }
        else if (x >= 277 && x <= 523)
        {
            if (g_current_page != 1)
            {
                printf("[TOUCH] Switch to Page 1 (DHT22)\n");
                SwitchPage(1);
            }
        }
        else if (x >= 542 && x <= 788)
        {
            if (g_current_page != 2)
            {
                printf("[TOUCH] Switch to Page 2 (MPU6050)\n");
                SwitchPage(2);
            }
        }
        return;
    }

    /* 2. 根据当前页面判断其他按钮的点击 (GT9157 实测坐标) */
    RS485TxRequest_t req;
    if (g_current_page == 0)
    {
        if (x >= 241 && x < 381 && y >= 210 && y < 250)
        {
            printf("[TOUCH] DHT (Dashboard)\n");
            req.dest_addr = ADDR_COLLECTOR_1;
            req.msg_type = MSG_TYPE_TEMP_HUMI;
            req.data_len = 0;
            xQueueSend(g_tx_queue, &req, 0);
        }
        else if (x >= 418 && x < 558 && y >= 210 && y < 250)
        {
            printf("[TOUCH] MPU (Dashboard)\n");
            req.dest_addr = ADDR_COLLECTOR_2;
            req.msg_type = MSG_TYPE_MPU6050;
            req.data_len = 0;
            xQueueSend(g_tx_queue, &req, 0);
        }
    }
    else if (g_current_page == 1)
    {
        if (x >= 500 && x < 700 && y >= 210 && y < 250)
        {
            printf("[TOUCH] DHT (Detail)\n");
            req.dest_addr = ADDR_COLLECTOR_1;
            req.msg_type = MSG_TYPE_TEMP_HUMI;
            req.data_len = 0;
            xQueueSend(g_tx_queue, &req, 0);
        }
    }
    else if (g_current_page == 2)
    {
        if (x >= 500 && x < 700 && y >= 210 && y < 250)
        {
            printf("[TOUCH] MPU (Detail)\n");
            req.dest_addr = ADDR_COLLECTOR_2;
            req.msg_type = MSG_TYPE_MPU6050;
            req.data_len = 0;
            xQueueSend(g_tx_queue, &req, 0);
        }
    }
}

static void QueueSensorRequest(uint8_t dest_addr, uint8_t msg_type)
{
    RS485TxRequest_t req;

    req.dest_addr = dest_addr;
    req.msg_type = msg_type;
    req.data_len = 0;
    xQueueSend(g_tx_queue, &req, 0);
}

/* ========================================================================== */
/*  Draw the static UI once at startup                                         */
/* ========================================================================== */
static void DrawStaticUI(void)
{
    /* Background */
    FillR(0, 0, LCD_PIXEL_WIDTH, 480, COL_BG);

    /* Top Title Bar */
    FillR(0, 0, LCD_PIXEL_WIDTH, TITLE_H, COL_CARD);
    if (g_current_page == 0)
    {
        Put(16, 12, COL_TEXT_PRI, COL_CARD, "SYSTEM TERMINAL");
    }
    else if (g_current_page == 1)
    {
        Put(16, 12, COL_TEXT_PRI, COL_CARD, "DHT22 SENSOR DETAILS");
    }
    else
    {
        Put(16, 12, COL_TEXT_PRI, COL_CARD, "MPU6050 SENSOR DETAILS");
    }
    Put(420, 12, COL_TEXT_SEC, COL_CARD, "|  NODE [0x01]");

    snprintf(g_buf, sizeof(g_buf), "PACKETS: %lu", g_update_count);
    Put(580, 12, COL_TEXT_SEC, COL_CARD, g_buf);

    FillR(0, TITLE_H - 1, LCD_PIXEL_WIDTH, 1, COL_BORDER);

    if (g_current_page == 0)
    {
        /* Left Card: DHT22 */
        FillR(12, HDR_Y + 12, DIV_X - 18, BODY_H + HDR_H - 12, COL_CARD);
        LCD_SetTextColor(COL_BORDER);
        LCD_DrawRect(12, HDR_Y + 12, DIV_X - 18, BODY_H + HDR_H - 12);

        Put(24, HDR_Y + 24, COL_ACCENT, COL_CARD, "DHT22");
        Put(120, HDR_Y + 24, COL_TEXT_SEC, COL_CARD, "[0x02]");
        Put(260, HDR_Y + 24, g_dht_valid ? COL_GREEN : COL_TEXT_SEC, COL_CARD, g_dht_valid ? "ONLINE" : "OFFLINE");
        FillR(12, HDR_Y + 48, DIV_X - 18, 1, COL_BORDER);

        /* Grid Lines for Left Card */
        FillR(12, 126, DIV_X - 18, 1, COL_BORDER);
        FillR(12, 154, DIV_X - 18, 1, COL_BORDER);
        FillR(12, 182, DIV_X - 18, 1, COL_BORDER);
        FillR(12, 210, DIV_X - 18, 1, COL_BORDER);
        FillR(148, 88, 1, 122, COL_BORDER);

        /* Right Card: MPU6050 */
        FillR(DIV_X + 6, HDR_Y + 12, LCD_PIXEL_WIDTH - DIV_X - 18, BODY_H + HDR_H - 12, COL_CARD);
        LCD_SetTextColor(COL_BORDER);
        LCD_DrawRect(DIV_X + 6, HDR_Y + 12, LCD_PIXEL_WIDTH - DIV_X - 18, BODY_H + HDR_H - 12);

        Put(DIV_X + 18, HDR_Y + 24, COL_ACCENT, COL_CARD, "MPU6050");
        Put(DIV_X + 140, HDR_Y + 24, COL_TEXT_SEC, COL_CARD, "[0x03]");
        Put(DIV_X + 260, HDR_Y + 24, g_mpu_valid ? COL_GREEN : COL_TEXT_SEC, COL_CARD, g_mpu_valid ? "ONLINE" : "OFFLINE");
        FillR(DIV_X + 6, HDR_Y + 48, LCD_PIXEL_WIDTH - DIV_X - 18, 1, COL_BORDER);

        /* Grid Lines for Right Card */
        FillR(DIV_X + 6, 126, LCD_PIXEL_WIDTH - DIV_X - 18, 1, COL_BORDER);
        FillR(DIV_X + 6, 154, LCD_PIXEL_WIDTH - DIV_X - 18, 1, COL_BORDER);
        FillR(DIV_X + 6, 182, LCD_PIXEL_WIDTH - DIV_X - 18, 1, COL_BORDER);
        FillR(DIV_X + 6, 210, LCD_PIXEL_WIDTH - DIV_X - 18, 1, COL_BORDER);
        FillR(DIV_X + 148, 88, 1, 122, COL_BORDER);

        /* Left card labels */
        Put(24, 96, COL_TEXT_SEC, COL_CARD, "Temp");
        Put(24, 128, COL_TEXT_SEC, COL_CARD, "Humi");
        Put(24, 156, COL_TEXT_SEC, COL_CARD, "Updated");

        /* Right card labels */
        Put(DIV_X + 18, 96, COL_TEXT_SEC, COL_CARD, "Pitch");
        Put(DIV_X + 18, 128, COL_TEXT_SEC, COL_CARD, "Roll");
        Put(DIV_X + 18, 156, COL_TEXT_SEC, COL_CARD, "Yaw");
        Put(DIV_X + 18, 184, COL_TEXT_SEC, COL_CARD, "Updated");

        /* Placeholder values */
        Put(164, 96, COL_TEXT_SEC, COL_CARD, "---.- C");
        Put(164, 128, COL_TEXT_SEC, COL_CARD, "---.- %");
        Put(164, 156, COL_TEXT_SEC, COL_CARD, "-- s   ");

        Put(DIV_X + 164, 96, COL_TEXT_SEC, COL_CARD, "---.- deg");
        Put(DIV_X + 164, 128, COL_TEXT_SEC, COL_CARD, "---.- deg");
        Put(DIV_X + 164, 156, COL_TEXT_SEC, COL_CARD, "---.- deg");
        Put(DIV_X + 164, 184, COL_TEXT_SEC, COL_CARD, "-- s   ");

        /* Touch buttons */
        DrawButtons();

        /* History section */
        FillR(12, HIST_HDR_Y + 12, LCD_PIXEL_WIDTH - 24, HIST_BODY_H + HIST_HDR_H - 12, COL_CARD);
        LCD_SetTextColor(COL_BORDER);
        LCD_DrawRect(12, HIST_HDR_Y + 12, LCD_PIXEL_WIDTH - 24, HIST_BODY_H + HIST_HDR_H - 12);

        Put(24, HIST_HDR_Y + 24, COL_ACCENT, COL_CARD, "SYSTEM LOG (Recent 5)");
        FillR(12, HIST_HDR_Y + 48, LCD_PIXEL_WIDTH - 24, 1, COL_BORDER);
    }
    else if (g_current_page == 1)
    {
        /* Temp Large Card */
        FillR(12, 52, DIV_X - 18, 148, COL_CARD);
        LCD_SetTextColor(COL_BORDER);
        LCD_DrawRect(12, 52, DIV_X - 18, 148);
        Put(24, 64, COL_ACCENT, COL_CARD, "Temperature Stat");
        Put(24, 98, COL_TEXT_SEC, COL_CARD, "Current:");
        Put(24, 154, COL_TEXT_SEC, COL_CARD, "Range  :");
        /* Grid Lines for Temp Detail */
        FillR(12, 132, DIV_X - 18, 1, COL_BORDER);

        /* Humi Large Card */
        FillR(DIV_X + 6, 52, LCD_PIXEL_WIDTH - DIV_X - 18, 148, COL_CARD);
        LCD_SetTextColor(COL_BORDER);
        LCD_DrawRect(DIV_X + 6, 52, LCD_PIXEL_WIDTH - DIV_X - 18, 148);
        Put(DIV_X + 18, 64, COL_ACCENT, COL_CARD, "Humidity Stat");
        FillR(DIV_X + 6, 88, LCD_PIXEL_WIDTH - DIV_X - 18, 1, COL_BORDER);
        Put(DIV_X + 18, 98, COL_TEXT_SEC, COL_CARD, "Current:");
        Put(DIV_X + 18, 154, COL_TEXT_SEC, COL_CARD, "Range  :");
        /* Grid Lines for Humi Detail */
        FillR(DIV_X + 6, 132, LCD_PIXEL_WIDTH - DIV_X - 18, 1, COL_BORDER);
        FillR(DIV_X + 148, 88, 1, 112, COL_BORDER);

        /* History log card for DHT22 only */
        FillR(12, 212, LCD_PIXEL_WIDTH - 24, 208, COL_CARD);
        LCD_SetTextColor(COL_BORDER);
        LCD_DrawRect(12, 212, LCD_PIXEL_WIDTH - 24, 208);
        Put(24, 224, COL_ACCENT, COL_CARD, "DHT22 DETAILED LOGS (Recent 7)");
        FillR(12, 248, LCD_PIXEL_WIDTH - 24, 1, COL_BORDER);

        /* Touch Button inside details history header (right-aligned) */
        DrawButton(636, 218, 140, 28, COL_ACCENT, COL_ACCENT);
        Put(636 + 14, 218 + 5, COL_WHITE, COL_ACCENT, "REQ");
    }
    else if (g_current_page == 2)
    {
        /* Pitch, Roll, Yaw Cards */
        int col_w = (LCD_PIXEL_WIDTH - 36) / 3;
        int x1 = 12;
        int x2 = 12 + col_w + 6;
        int x3 = 12 + 2 * col_w + 12;

        FillR(x1, 52, col_w, 148, COL_CARD);
        LCD_SetTextColor(COL_BORDER);
        LCD_DrawRect(x1, 52, col_w, 148);
        Put(x1 + 12, 64, COL_ACCENT, COL_CARD, "Pitch");
        FillR(x1, 88, col_w, 1, COL_BORDER);
        Put(x1 + 12, 98, COL_TEXT_SEC, COL_CARD, "Cur:");
        Put(x1 + 12, 154, COL_TEXT_SEC, COL_CARD, "Rng:");
        /* Grid Lines for Pitch Card */
        FillR(x1, 132, col_w, 1, COL_BORDER);
        FillR(x1 + 72, 88, 1, 112, COL_BORDER);

        FillR(x2, 52, col_w, 148, COL_CARD);
        LCD_SetTextColor(COL_BORDER);
        LCD_DrawRect(x2, 52, col_w, 148);
        Put(x2 + 12, 64, COL_ACCENT, COL_CARD, "Roll");
        FillR(x2, 88, col_w, 1, COL_BORDER);
        Put(x2 + 12, 98, COL_TEXT_SEC, COL_CARD, "Cur:");
        Put(x2 + 12, 154, COL_TEXT_SEC, COL_CARD, "Rng:");
        /* Grid Lines for Roll Card */
        FillR(x2, 132, col_w, 1, COL_BORDER);
        FillR(x2 + 72, 88, 1, 112, COL_BORDER);

        FillR(x3, 52, col_w, 148, COL_CARD);
        LCD_SetTextColor(COL_BORDER);
        LCD_DrawRect(x3, 52, col_w, 148);
        Put(x3 + 12, 64, COL_ACCENT, COL_CARD, "Yaw");
        FillR(x3, 88, col_w, 1, COL_BORDER);
        Put(x3 + 12, 98, COL_TEXT_SEC, COL_CARD, "Cur:");
        Put(x3 + 12, 154, COL_TEXT_SEC, COL_CARD, "Rng:");
        /* Grid Lines for Yaw Card */
        FillR(x3, 132, col_w, 1, COL_BORDER);
        FillR(x3 + 72, 88, 1, 112, COL_BORDER);

        /* History log card for MPU6050 only */
        FillR(12, 212, LCD_PIXEL_WIDTH - 24, 208, COL_CARD);
        LCD_SetTextColor(COL_BORDER);
        LCD_DrawRect(12, 212, LCD_PIXEL_WIDTH - 24, 208);
        Put(24, 224, COL_ACCENT, COL_CARD, "MPU6050 DETAILED LOGS (Recent 7)");
        FillR(12, 248, LCD_PIXEL_WIDTH - 24, 1, COL_BORDER);

        /* Touch Button inside details history header (right-aligned) */
        DrawButton(636, 218, 140, 28, COL_ACCENT, COL_ACCENT);
        Put(636 + 14, 218 + 5, COL_WHITE, COL_ACCENT, "REQ");
    }

    /* Bottom Tab Bar */
    DrawMenuBar();
}

/* ========================================================================== */
/*  Refresh sensor data area                                                   */
/* ========================================================================== */
static void RefreshSensors(void)
{
    uint32_t now = xTaskGetTickCount();
    uint32_t age;
    uint16_t fg;

    if (g_current_page == 0)
    {
        if (g_dht_valid)
        {
            age = (now - g_dht_tick) * portTICK_PERIOD_MS / 1000;
            fg = COL_TEXT_PRI;
            snprintf(g_buf, sizeof(g_buf), "%5.1f C      ", g_dht_temp);
            Put(164, 96, fg, COL_CARD, g_buf);
            snprintf(g_buf, sizeof(g_buf), "%5.1f %%      ", g_dht_humi);
            Put(164, 128, fg, COL_CARD, g_buf);
            Put(260, HDR_Y + 24, COL_GREEN, COL_CARD, "ONLINE ");
            snprintf(g_buf, sizeof(g_buf), "%3lu s      ", age);
            Put(164, 156, COL_TEXT_SEC, COL_CARD, g_buf);
        }
        else
        {
            Put(164, 96, COL_TEXT_SEC, COL_CARD, "---.- C     ");
            Put(164, 128, COL_TEXT_SEC, COL_CARD, "---.- %     ");
            Put(260, HDR_Y + 24, COL_RED, COL_CARD, "OFFLINE");
            Put(164, 156, COL_TEXT_SEC, COL_CARD, "-- s        ");
        }

        if (g_mpu_valid)
        {
            age = (now - g_mpu_tick) * portTICK_PERIOD_MS / 1000;
            fg = COL_TEXT_PRI;
            snprintf(g_buf, sizeof(g_buf), "%5.1f deg  ", g_mpu_pitch);
            Put(DIV_X + 164, 96, fg, COL_CARD, g_buf);
            snprintf(g_buf, sizeof(g_buf), "%5.1f deg  ", g_mpu_roll);
            Put(DIV_X + 164, 128, fg, COL_CARD, g_buf);
            snprintf(g_buf, sizeof(g_buf), "%5.1f deg  ", g_mpu_yaw);
            Put(DIV_X + 164, 156, fg, COL_CARD, g_buf);
            Put(DIV_X + 260, HDR_Y + 24, COL_GREEN, COL_CARD, "ONLINE ");
            snprintf(g_buf, sizeof(g_buf), "%3lu s      ", age);
            Put(DIV_X + 164, 184, COL_TEXT_SEC, COL_CARD, g_buf);
        }
        else
        {
            Put(DIV_X + 164, 96, COL_TEXT_SEC, COL_CARD, "---.- deg  ");
            Put(DIV_X + 164, 128, COL_TEXT_SEC, COL_CARD, "---.- deg  ");
            Put(DIV_X + 164, 156, COL_TEXT_SEC, COL_CARD, "---.- deg  ");
            Put(DIV_X + 260, HDR_Y + 24, COL_RED, COL_CARD, "OFFLINE");
            Put(DIV_X + 164, 184, COL_TEXT_SEC, COL_CARD, "-- s        ");
        }
    }
    else if (g_current_page == 1)
    {
        if (g_dht_valid)
        {
            fg = COL_TEXT_PRI;
            snprintf(g_buf, sizeof(g_buf), "%5.1f C", g_dht_temp);
            Put(160, 98, fg, COL_CARD, g_buf);
            snprintf(g_buf, sizeof(g_buf), "%.1f~%.1f C", g_dht_temp_min, g_dht_temp_max);
            Put(160, 154, COL_TEXT_SEC, COL_CARD, g_buf);

            snprintf(g_buf, sizeof(g_buf), "%5.1f %%", g_dht_humi);
            Put(DIV_X + 160, 98, fg, COL_CARD, g_buf);
            snprintf(g_buf, sizeof(g_buf), "%.1f~%.1f %%", g_dht_humi_min, g_dht_humi_max);
            Put(DIV_X + 160, 154, COL_TEXT_SEC, COL_CARD, g_buf);
        }
        else
        {
            Put(160, 98, COL_TEXT_SEC, COL_CARD, "---.- C");
            Put(160, 154, COL_TEXT_SEC, COL_CARD, "---~--- C");
            Put(DIV_X + 160, 98, COL_TEXT_SEC, COL_CARD, "---.- %");
            Put(DIV_X + 160, 154, COL_TEXT_SEC, COL_CARD, "---~--- %");
        }
    }
    else if (g_current_page == 2)
    {
        int col_w = (LCD_PIXEL_WIDTH - 36) / 3;
        int x1 = 12;
        int x2 = 12 + col_w + 6;
        int x3 = 12 + 2 * col_w + 12;

        if (g_mpu_valid)
        {
            fg = COL_TEXT_PRI;
            snprintf(g_buf, sizeof(g_buf), "%5.1f", g_mpu_pitch);
            Put(x1 + 80, 98, fg, COL_CARD, g_buf);
            snprintf(g_buf, sizeof(g_buf), "%.0f/%.0f", g_mpu_pitch_min, g_mpu_pitch_max);
            Put(x1 + 80, 154, COL_TEXT_SEC, COL_CARD, g_buf);

            snprintf(g_buf, sizeof(g_buf), "%5.1f", g_mpu_roll);
            Put(x2 + 80, 98, fg, COL_CARD, g_buf);
            snprintf(g_buf, sizeof(g_buf), "%.0f/%.0f", g_mpu_roll_min, g_mpu_roll_max);
            Put(x2 + 80, 154, COL_TEXT_SEC, COL_CARD, g_buf);

            snprintf(g_buf, sizeof(g_buf), "%5.1f", g_mpu_yaw);
            Put(x3 + 80, 98, fg, COL_CARD, g_buf);
            snprintf(g_buf, sizeof(g_buf), "%.0f/%.0f", g_mpu_yaw_min, g_mpu_yaw_max);
            Put(x3 + 80, 154, COL_TEXT_SEC, COL_CARD, g_buf);
        }
        else
        {
            Put(x1 + 80, 98, COL_TEXT_SEC, COL_CARD, "---.-");
            Put(x1 + 80, 154, COL_TEXT_SEC, COL_CARD, "---/---");
            Put(x2 + 80, 98, COL_TEXT_SEC, COL_CARD, "---.-");
            Put(x2 + 80, 154, COL_TEXT_SEC, COL_CARD, "---/---");
            Put(x3 + 80, 98, COL_TEXT_SEC, COL_CARD, "---.-");
            Put(x3 + 80, 154, COL_TEXT_SEC, COL_CARD, "---/---");
        }
    }
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

    if (g_current_page == 0)
    {
        for (i = 0; i < GUI_HISTORY_DEPTH && (dht_cnt < HIST_PER_COL || mpu_cnt < HIST_PER_COL); i++)
        {
            idx = (g_hist_idx - 1 - i + GUI_HISTORY_DEPTH) % GUI_HISTORY_DEPTH;
            if (g_history[idx].tick == 0)
                break;

            age = (now - g_history[idx].tick) * portTICK_PERIOD_MS / 1000;

            if (g_history[idx].type == GUI_SENSOR_DHT && dht_cnt < HIST_PER_COL)
            {
                entry = dht_cnt + 1;
                line_y = HIST_HDR_Y + 50 + dht_cnt * 25;
                bg = (dht_cnt & 1) ? COL_CARD : COL_PALE;
                FillR(14, line_y, 220, 20, bg);

                /* 分列对齐: 序号 | 温度 | 湿度 | 时间 */
                snprintf(g_buf, sizeof(g_buf), "%2d", entry);
                Put(18, line_y + 2, COL_TEXT_PRI, bg, g_buf);
                snprintf(g_buf, sizeof(g_buf), "%.1fC", g_history[idx].data.dht.temp);
                Put(56, line_y + 2, COL_TEXT_PRI, bg, g_buf);
                snprintf(g_buf, sizeof(g_buf), "%.0f%%", g_history[idx].data.dht.humi);
                Put(120, line_y + 2, COL_TEXT_PRI, bg, g_buf);
                snprintf(g_buf, sizeof(g_buf), "%lus", age);
                Put(170, line_y + 2, COL_TEXT_SEC, bg, g_buf);

                /* Draw row bottom divider line */
                FillR(14, line_y + 20, 220, 1, COL_BORDER);
                dht_cnt++;
            }
            else if (g_history[idx].type == GUI_SENSOR_MPU6050 && mpu_cnt < HIST_PER_COL)
            {
                entry = mpu_cnt + 1;
                line_y = HIST_HDR_Y + 50 + mpu_cnt * 25;
                bg = (mpu_cnt & 1) ? COL_CARD : COL_PALE;
                FillR(DIV_X + 10, line_y, 320, 20, bg);

                /* 分列对齐: 序号 | Pitch | Roll | Yaw | 时间 */
                snprintf(g_buf, sizeof(g_buf), "%2d", entry);
                Put(DIV_X + 16, line_y + 2, COL_TEXT_PRI, bg, g_buf);
                snprintf(g_buf, sizeof(g_buf), "P%.0f", g_history[idx].data.mpu.pitch);
                Put(DIV_X + 50, line_y + 2, COL_TEXT_PRI, bg, g_buf);
                snprintf(g_buf, sizeof(g_buf), "R%.0f", g_history[idx].data.mpu.roll);
                Put(DIV_X + 120, line_y + 2, COL_TEXT_PRI, bg, g_buf);
                snprintf(g_buf, sizeof(g_buf), "Y%.0f", g_history[idx].data.mpu.yaw);
                Put(DIV_X + 190, line_y + 2, COL_TEXT_PRI, bg, g_buf);
                snprintf(g_buf, sizeof(g_buf), "%lus", age);
                Put(DIV_X + 260, line_y + 2, COL_TEXT_SEC, bg, g_buf);

                FillR(DIV_X + 10, line_y + 20, 320, 1, COL_BORDER);
                mpu_cnt++;
            }
        }

        for (i = dht_cnt; i < HIST_PER_COL; i++)
        {
            line_y = HIST_HDR_Y + 50 + i * 25;
            bg = (i & 1) ? COL_CARD : COL_PALE;
            FillR(14, line_y, 210, 20, bg);
            snprintf(g_buf, sizeof(g_buf), " %d:  ---                  ", i + 1);
            Put(20, line_y, COL_TEXT_SEC, bg, g_buf);
            FillR(14, line_y + 20, 220, 1, COL_BORDER);
        }
        for (i = mpu_cnt; i < HIST_PER_COL; i++)
        {
            line_y = HIST_HDR_Y + 50 + i * 25;
            bg = (i & 1) ? COL_CARD : COL_PALE;
            FillR(DIV_X + 10, line_y, 320, 20, bg);
            snprintf(g_buf, sizeof(g_buf), " %d:  ---                  ", i + 1);
            Put(DIV_X + 16, line_y, COL_TEXT_SEC, bg, g_buf);
            FillR(DIV_X + 10, line_y + 20, 320, 1, COL_BORDER);
        }

        /* Draw center divider line for history */
        FillR(DIV_X, HIST_HDR_Y + 48, 1, HIST_BODY_H - 24, COL_BORDER);
    }
    else if (g_current_page == 1)
    {
        for (i = 0; i < GUI_HISTORY_DEPTH && dht_cnt < 7; i++)
        {
            idx = (g_hist_idx - 1 - i + GUI_HISTORY_DEPTH) % GUI_HISTORY_DEPTH;
            if (g_history[idx].tick == 0)
                break;

            if (g_history[idx].type == GUI_SENSOR_DHT)
            {
                age = (now - g_history[idx].tick) * portTICK_PERIOD_MS / 1000;
                entry = dht_cnt + 1;
                line_y = 250 + dht_cnt * 24;
                bg = (dht_cnt & 1) ? COL_CARD : COL_PALE;
                FillR(14, line_y, LCD_PIXEL_WIDTH - 28, 20, bg);

                snprintf(g_buf, sizeof(g_buf), " #%d: Temp: %.1fC | Humi: %.1f%% | %lus ago",
                         entry, g_history[idx].data.dht.temp,
                         g_history[idx].data.dht.humi, age);
                Put(20, line_y, COL_TEXT_PRI, bg, g_buf);
                dht_cnt++;
            }
        }
        for (i = dht_cnt; i < 7; i++)
        {
            line_y = 250 + i * 24;
            bg = (i & 1) ? COL_CARD : COL_PALE;
            FillR(14, line_y, LCD_PIXEL_WIDTH - 28, 20, bg);
            snprintf(g_buf, sizeof(g_buf), " #%d: ---", i + 1);
            Put(20, line_y, COL_TEXT_SEC, bg, g_buf);
        }
    }
    else if (g_current_page == 2)
    {
        for (i = 0; i < GUI_HISTORY_DEPTH && mpu_cnt < 7; i++)
        {
            idx = (g_hist_idx - 1 - i + GUI_HISTORY_DEPTH) % GUI_HISTORY_DEPTH;
            if (g_history[idx].tick == 0)
                break;

            if (g_history[idx].type == GUI_SENSOR_MPU6050)
            {
                age = (now - g_history[idx].tick) * portTICK_PERIOD_MS / 1000;
                entry = mpu_cnt + 1;
                line_y = 250 + mpu_cnt * 24;
                bg = (mpu_cnt & 1) ? COL_CARD : COL_PALE;
                FillR(14, line_y, LCD_PIXEL_WIDTH - 28, 20, bg);

                snprintf(g_buf, sizeof(g_buf), " #%d: P:%.1f | R:%.1f | Y:%.1f | %lus ago",
                         entry, g_history[idx].data.mpu.pitch,
                         g_history[idx].data.mpu.roll,
                         g_history[idx].data.mpu.yaw, age);
                Put(20, line_y, COL_TEXT_PRI, bg, g_buf);
                mpu_cnt++;
            }
        }
        for (i = mpu_cnt; i < 7; i++)
        {
            line_y = 250 + i * 24;
            bg = (i & 1) ? COL_CARD : COL_PALE;
            FillR(14, line_y, LCD_PIXEL_WIDTH - 28, 20, bg);
            snprintf(g_buf, sizeof(g_buf), " #%d: ---", i + 1);
            Put(20, line_y, COL_TEXT_SEC, bg, g_buf);
        }
    }
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
/*  GUI Task ?非阻塞按?+ 传感器显?+ 历史记录                                */
/*                                                                             */
/*  按键检? ?20ms 读一?GPIO, 下降?(1?) 触发, 无阻?                   */
/*  KEY1(PA0) ?请求 DHT    KEY2(PC13) ?请求 MPU                              */
/* ========================================================================== */
void GUI_Task(void *pvParameters)
{
    GUIMsg_t msg;
    g_gui_queue = (QueueHandle_t)pvParameters;

    LCD_SetLayer(LCD_FOREGROUND_LAYER);
    LCD_Clear(COL_BG);

    /* 按键消抖: KEY1=PA0 短按DHT/长按MPU, KEY2=PC13 仅诊?*/
    /* KEY1: short press DHT, long press MPU. Active level follows KEY_ON. */
    uint8_t k1_raw = GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_PIN);
    uint8_t k1_last_raw = k1_raw;
    uint8_t k1_stable = k1_raw;
    uint32_t k1_raw_change_tick = 0;

    uint8_t k2_raw = GPIO_ReadInputDataBit(KEY2_GPIO_PORT, KEY2_PIN);
    uint8_t k2_last_raw = k2_raw;
    uint8_t k2_stable = k2_raw;
    uint32_t k2_raw_change_tick = 0;

    /* 触摸屏初始化已禁??软件 I2C 操作 PA8/PC9 + I2C3 干扰 LTDC 导致全黑 */
    // Touch_Init();

    DrawStaticUI();

    while (1)
    {
        TickType_t now = xTaskGetTickCount();
        uint8_t k1 = GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_PIN);
        uint8_t k2 = GPIO_ReadInputDataBit(KEY2_GPIO_PORT, KEY2_PIN);

        if (k1 != k1_last_raw)
        {
            k1_last_raw = k1;
            k1_raw_change_tick = now;
        }

        if ((k1 != k1_stable) &&
            ((now - k1_raw_change_tick) >= pdMS_TO_TICKS(KEY_DEBOUNCE_MS)))
        {
            k1_stable = k1;
            if (k1_stable == KEY_ON)
            {
                /* KEY1: 翻转 DHT22 前端 LED */
                static uint8_t dht_led = 0;
                dht_led = !dht_led;
                printf("[KEY] KEY1 pressed -> toggle DHT LED %s\n", dht_led ? "ON" : "OFF");
                {
                    RS485TxRequest_t led_req;
                    led_req.dest_addr = ADDR_COLLECTOR_1;
                    led_req.msg_type = MSG_TYPE_LED_CTRL;
                    led_req.data[0] = dht_led ? 1 : 0;
                    led_req.data_len = 1;
                    xQueueSend(g_tx_queue, &led_req, 0);
                }
            }
        }

        /* KEY2 (PC13): 翻转 MPU6050 前端 LED */
        if (k2 != k2_last_raw)
        {
            k2_last_raw = k2;
            k2_raw_change_tick = now;
        }
        if ((k2 != k2_stable) &&
            ((now - k2_raw_change_tick) >= pdMS_TO_TICKS(KEY_DEBOUNCE_MS)))
        {
            k2_stable = k2;
            if (k2_stable == KEY_ON)
            {
                static uint8_t mpu_led = 0;
                mpu_led = !mpu_led;
                printf("[KEY] KEY2 pressed -> toggle MPU LED %s\n", mpu_led ? "ON" : "OFF");
                {
                    RS485TxRequest_t led_req;
                    led_req.dest_addr = ADDR_COLLECTOR_2;
                    led_req.msg_type = MSG_TYPE_LED_CTRL;
                    led_req.data[0] = mpu_led ? 1 : 0;
                    led_req.data_len = 1;
                    xQueueSend(g_tx_queue, &led_req, 0);
                }
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

                if (g_dht_temp_max < -900.0f)
                {
                    g_dht_temp_max = g_dht_temp;
                    g_dht_temp_min = g_dht_temp;
                    g_dht_humi_max = g_dht_humi;
                    g_dht_humi_min = g_dht_humi;
                }
                else
                {
                    if (g_dht_temp > g_dht_temp_max)
                        g_dht_temp_max = g_dht_temp;
                    if (g_dht_temp < g_dht_temp_min)
                        g_dht_temp_min = g_dht_temp;
                    if (g_dht_humi > g_dht_humi_max)
                        g_dht_humi_max = g_dht_humi;
                    if (g_dht_humi < g_dht_humi_min)
                        g_dht_humi_min = g_dht_humi;
                }
            }
            else
            {
                g_mpu_valid = 1;
                g_mpu_pitch = msg.data.mpu.pitch;
                g_mpu_roll = msg.data.mpu.roll;
                g_mpu_yaw = msg.data.mpu.yaw;
                g_mpu_tick = xTaskGetTickCount();

                if (g_mpu_pitch_max < -900.0f)
                {
                    g_mpu_pitch_max = g_mpu_pitch;
                    g_mpu_pitch_min = g_mpu_pitch;
                    g_mpu_roll_max = g_mpu_roll;
                    g_mpu_roll_min = g_mpu_roll;
                    g_mpu_yaw_max = g_mpu_yaw;
                    g_mpu_yaw_min = g_mpu_yaw;
                }
                else
                {
                    if (g_mpu_pitch > g_mpu_pitch_max)
                        g_mpu_pitch_max = g_mpu_pitch;
                    if (g_mpu_pitch < g_mpu_pitch_min)
                        g_mpu_pitch_min = g_mpu_pitch;
                    if (g_mpu_roll > g_mpu_roll_max)
                        g_mpu_roll_max = g_mpu_roll;
                    if (g_mpu_roll < g_mpu_roll_min)
                        g_mpu_roll_min = g_mpu_roll;
                    if (g_mpu_yaw > g_mpu_yaw_max)
                        g_mpu_yaw_max = g_mpu_yaw;
                    if (g_mpu_yaw < g_mpu_yaw_min)
                        g_mpu_yaw_min = g_mpu_yaw;
                }
            }
            g_update_count++;
            HistoryAdd(&msg);
        }

        /* 触摸屏扫描 (lantian ISR→信号量→GTP_TouchProcess) */
        if (xSemaphoreTake(xTouchSemaphore, 0) == pdTRUE)
        {
            GTP_TouchProcess();
            TouchEvent_t ev;
            while (xQueueReceive(xTouchQueue, &ev, 0) == pdTRUE)
            {
                HandleTouch((uint16_t)ev.x, (uint16_t)ev.y);
            }
        }

        RefreshSensors();
        RefreshHistory();

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ========================================================================== */
/*  LCD_TestTask  ?测试模式: 绕过 RS485, 每秒 ?GUI 队列喂虚拟传感器数据     */
/*  用于在没有采集前端时, 验证 LCD 更新流程                                    */
/* ========================================================================== */
void LCD_TestTask(void *pvParameters)
{
    QueueHandle_t gui_queue = (QueueHandle_t)pvParameters;
    GUIMsg_t msg;
    int cnt = 0;

    while (1)
    {
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
        msg.data.mpu.roll = -20.0f - (cnt % 360);
        msg.data.mpu.yaw = (cnt % 360) * 2.0f;
        xQueueSend(gui_queue, &msg, 0);

        printf("[TEST] #%d: DHT=%.1f/%.1f  MPU=%.1f/%.1f/%.1f\n",
               cnt, msg.data.dht.temp, msg.data.dht.humi,
               msg.data.mpu.pitch, msg.data.mpu.roll, msg.data.mpu.yaw);
    }
}
