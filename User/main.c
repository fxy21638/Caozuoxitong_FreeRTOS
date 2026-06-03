/**
  *********************************************************************
  * @file    main.c
  * @brief   FreeRTOS UART Protocol Parser with LCD Display
  *
  *          Task1: UART protocol analysis + LCD display
  *          Task2: UART response
  *
  *          Protocol frame: 0xAA + length + data[length] + 0x55
  *
  *          Frame boundary = 200ms inter-byte timeout.
  *          All bytes within 200ms of each other form one frame.
  *          After 200ms idle, the accumulated frame is validated.
  *********************************************************************
  */

/* FreeRTOS headers */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/* BSP headers */
#include "bsp_led.h"
#include "bsp_debug_usart.h"
#include "./lcd/bsp_lcd.h"
#include "protocol.h"

/* Standard headers */
#include <string.h>
#include <stdio.h>

/**************************** Global Handles ********************************/
TaskHandle_t UartRxTask_Handle = NULL;
TaskHandle_t UartTxTask_Handle = NULL;
TimerHandle_t RxTimeoutTimer_Handle = NULL;

/******************** ISR-to-Task Byte Ring Buffer **************************/
#define RX_RING_SIZE  64
volatile uint8_t g_rx_ring[RX_RING_SIZE];
volatile uint8_t g_rx_wr;
static uint8_t g_rx_rd;

/**************************** Task Notification Bits *************************/
#define NOTIFY_BIT_BYTES    0x01  /* ISR: new bytes in ring buffer */
#define NOTIFY_BIT_TIMEOUT  0x02  /* Timer: 200ms timeout, validate frame */

/**************************** Protocol Parser *******************************/
static ProtocolParser_t g_parser;

/* Display buffer */
static char lcd_disp_buf[64];

/* Rolling line pairs for multi-frame LCD display.
 * Computed at runtime because ARM CC5 C90 forbids function calls in static init.
 * LINE(x) = x * LCD_GetFont()->Height  (pixel Y coordinate) */
static uint16_t g_frame_line_pairs[13][2];
#define FRAME_PAIR_COUNT  13
static uint8_t g_frame_idx = 0;
static uint8_t g_frame_count = 0;

/**************************************************************************
  * @brief  200ms timeout: notify Task1 to validate accumulated frame
  **************************************************************************/
static void RxTimeoutCallback(TimerHandle_t xTimer)
{
    (void)xTimer;

    if (g_parser.byte_count > 0)
    {
        xTaskNotify(UartRxTask_Handle, NOTIFY_BIT_TIMEOUT, eSetBits);
    }
}

/**************************************************************************
  * @brief  Task1: UART protocol analysis + LCD display.
  *         Waits on task notification bits from ISR and timer.
  *         Drains ring buffer bytes into parser.
  *         On timeout: validates frame, displays data on 2 LCD lines,
  *         scrolls downward, notifies Task2 for response.
  **************************************************************************/
static void UartRxTask(void *pvParameters)
{
    (void)pvParameters;
    uint32_t ulNotifiedBits;
    uint8_t byte;
    uint16_t font_h;
    int i;

    Protocol_Init(&g_parser);

    /* Build LCD line-pair lookup table at runtime */
    font_h = LCD_GetFont()->Height;
    for (i = 0; i < FRAME_PAIR_COUNT; i++)
    {
        g_frame_line_pairs[i][0] = (uint16_t)((2U + (uint16_t)i * 2U) * font_h);
        g_frame_line_pairs[i][1] = (uint16_t)((3U + (uint16_t)i * 2U) * font_h);
    }

    /* LCD header */
    LCD_SetTextColor(LCD_COLOR_CYAN);
    LCD_SetBackColor(LCD_COLOR_BLACK);
    LCD_DisplayStringLine(LCD_LINE_0, (uint8_t *)"UART Protocol RX");

    while (1)
    {
        /* Wait for notification from ISR (bytes) or timer (timeout) */
        xTaskNotifyWait(0, 0xFFFFFFFF, &ulNotifiedBits, portMAX_DELAY);

        /* Drain ring buffer: accumulate all bytes into parser */
        while (g_rx_rd != g_rx_wr)
        {
            byte = g_rx_ring[g_rx_rd & (RX_RING_SIZE - 1)];
            g_rx_rd++;

            /* Diagnostic: show last byte on fixed LCD line */
            LCD_SetTextColor(LCD_COLOR_CYAN);
            sprintf(lcd_disp_buf, "Rx[%d]=0x%02X      ",
                    (int)g_parser.byte_count, (unsigned int)byte);
            LCD_DisplayStringLine(LCD_LINE_28, (uint8_t *)lcd_disp_buf);

            Protocol_AddByte(&g_parser, byte);
        }

        /* Timeout bit set -> validate accumulated frame */
        if (ulNotifiedBits & NOTIFY_BIT_TIMEOUT)
        {
            if (Protocol_Validate(&g_parser))
            {
                uint8_t data_len = Protocol_GetDataLen(&g_parser);
                uint8_t *data = Protocol_GetData(&g_parser);
                uint16_t line1, line2;
                int pos;

                line1 = g_frame_line_pairs[g_frame_idx][0];
                line2 = g_frame_line_pairs[g_frame_idx][1];

                LCD_ClearLine(line1);
                LCD_ClearLine(line2);

                /* Line 1: frame info (green) */
                LCD_SetTextColor(LCD_COLOR_GREEN);
                LCD_SetBackColor(LCD_COLOR_BLACK);
                sprintf(lcd_disp_buf, "F%d  len=%d", (int)g_frame_count, data_len);
                LCD_DisplayStringLine(line1, (uint8_t *)lcd_disp_buf);

                /* Line 2: hex data (white), up to 16 bytes */
                LCD_SetTextColor(LCD_COLOR_WHITE);
                pos = 0;
                for (i = 0; i < data_len && i < 16; i++)
                {
                    pos += sprintf(&lcd_disp_buf[pos], "%02X ", data[i]);
                }
                if (data_len > 16)
                {
                    pos += sprintf(&lcd_disp_buf[pos], "..");
                }
                LCD_DisplayStringLine(line2, (uint8_t *)lcd_disp_buf);

                g_frame_idx = (g_frame_idx + 1) % FRAME_PAIR_COUNT;
                g_frame_count++;

                xTaskNotifyGive(UartTxTask_Handle);
                LED2_TOGGLE;
            }

            /* Reset parser for next frame (valid or invalid) */
            Protocol_Reset(&g_parser);
        }
    }
}

/**************************************************************************
  * @brief  Task2: UART response task.
  *         Sends fixed response frame: 0xAA 0x01 0x01 0x55
  **************************************************************************/
static void UartTxTask(void *pvParameters)
{
    (void)pvParameters;
    static const uint8_t response_frame[] = {0xAA, 0x01, 0x01, 0x55};
    int i;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        for (i = 0; i < 4; i++)
        {
            Usart_SendByte(DEBUG_USART, response_frame[i]);
        }

        LCD_SetTextColor(LCD_COLOR_YELLOW);
        LCD_DisplayStringLine(LCD_LINE_29, (uint8_t *)"Response sent: AA 01 01 55");
    }
}


/**************************************************************************
  * @brief  Board Support Package initialization
  **************************************************************************/
static void BSP_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    LED_GPIO_Config();

    Debug_USART_Config();

    LCD_Init();
    LCD_LayerInit();
    LTDC_Cmd(ENABLE);

    LCD_SetLayer(LCD_BACKGROUND_LAYER);
    LCD_Clear(LCD_COLOR_BLACK);
    LCD_SetLayer(LCD_FOREGROUND_LAYER);
    LCD_SetTransparency(0xFF);
    LCD_Clear(LCD_COLOR_BLACK);
}

/**************************************************************************
  * @brief  Main entry point
  **************************************************************************/
int main(void)
{
    BaseType_t xReturn = pdPASS;

    BSP_Init();

    printf("=== FreeRTOS UART Protocol Parser ===\n");
    printf("Frame format: AA + Len + Data[N] + 55\n");
    printf("Timeout: 200ms inter-byte\n");

    RxTimeoutTimer_Handle = xTimerCreate(
        "RxTimeout",
        pdMS_TO_TICKS(200),
        pdFALSE,
        (void *)0,
        RxTimeoutCallback
    );

    if (RxTimeoutTimer_Handle == NULL)
    {
        printf("ERROR: Failed to create timeout timer!\n");
    }

    xReturn = xTaskCreate((TaskFunction_t)UartRxTask,
                          "UartRxTask",
                          512, NULL, 5,
                          &UartRxTask_Handle);
    if (xReturn == pdPASS) printf("UartRxTask created OK\n");

    xReturn = xTaskCreate((TaskFunction_t)UartTxTask,
                          "UartTxTask",
                          256, NULL, 4,
                          &UartTxTask_Handle);
    if (xReturn == pdPASS) printf("UartTxTask created OK\n");

    g_rx_wr = 0;
    g_rx_rd = 0;
    vTaskStartScheduler();

    while (1) {}
}
