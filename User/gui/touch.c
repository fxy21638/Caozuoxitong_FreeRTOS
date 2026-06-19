/* ========================================================================== */
/*  touch.c — GT9157 电容触摸屏驱动 (适配野火挑战者 F429)                      */
/*                                                                             */
/*  I2C3 (PA8=SCL, PC9=SDA), RST=PC0, INT=PC1                                     */
/*  软件 I2C bit-bang, vTaskDelay 让出 CPU, ACK 超时返回                          */
/* ========================================================================== */

#include "touch.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

/* ---- 引脚定义 ---- */
#define GTP_I2C_SCL_PORT   GPIOA
#define GTP_I2C_SCL_PIN    GPIO_Pin_8
#define GTP_I2C_SCL_CLK    RCC_AHB1Periph_GPIOA
#define GTP_I2C_SDA_PORT   GPIOC
#define GTP_I2C_SDA_PIN    GPIO_Pin_9
#define GTP_I2C_SDA_CLK    RCC_AHB1Periph_GPIOC

#define GTP_RST_PORT       GPIOC
#define GTP_RST_PIN        GPIO_Pin_0
#define GTP_RST_CLK        RCC_AHB1Periph_GPIOC

#define GTP_INT_PORT       GPIOC
#define GTP_INT_PIN        GPIO_Pin_1
#define GTP_INT_CLK        RCC_AHB1Periph_GPIOC

/* ---- I2C 位操作宏 (毫秒级延时, vTaskDelay 让出 CPU) ---- */
#define I2C_SCL_1()   GPIO_SetBits(GTP_I2C_SCL_PORT, GTP_I2C_SCL_PIN)
#define I2C_SCL_0()   GPIO_ResetBits(GTP_I2C_SCL_PORT, GTP_I2C_SCL_PIN)
#define I2C_SDA_1()   GPIO_SetBits(GTP_I2C_SDA_PORT, GTP_I2C_SDA_PIN)
#define I2C_SDA_0()   GPIO_ResetBits(GTP_I2C_SDA_PORT, GTP_I2C_SDA_PIN)
#define I2C_SDA_READ()  GPIO_ReadInputDataBit(GTP_I2C_SDA_PORT, GTP_I2C_SDA_PIN)

/* GT9157 I2C 地址 */
#define GTP_ADDR_W   0xBA
#define GTP_ADDR_R   0xBB

/* GT9157 寄存器 */
#define GTP_READ_COOR_ADDR   0x814E
#define GTP_REG_SENSOR_ID    0x814A
#define GTP_REG_VERSION       0x8140
#define GTP_REG_COMMAND       0x8040

/* 触摸状态 */
static volatile uint16_t touch_x = 0, touch_y = 0;
static volatile uint8_t  touch_ready = 0;

/* 超时时间 (ms) */
#define I2C_TIMEOUT_MS   5     /* 单字节超时 */
#define RESET_DELAY_MS    10    /* 复位序列单步延时 */

/* ---- 延时函数: 调用 vTaskDelay 让出 CPU ---- */
static void i2c_delay(uint32_t ms)
{
    if (ms > 0) vTaskDelay(pdMS_TO_TICKS(ms));
}

/* ---- SDA 输入/输出模式切换 (一次初始化, 之后用 ODR 寄存器) ---- */
static void sda_to_input(void)
{
    GPIO_InitTypeDef gpio = {
        .GPIO_Pin   = GTP_I2C_SDA_PIN,
        .GPIO_Mode  = GPIO_Mode_IN,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_PuPd  = GPIO_PuPd_NOPULL,
    };
    GPIO_Init(GTP_I2C_SDA_PORT, &gpio);
}

static void sda_to_output(void)
{
    GPIO_InitTypeDef gpio = {
        .GPIO_Pin   = GTP_I2C_SDA_PIN,
        .GPIO_Mode  = GPIO_Mode_OUT,
        .GPIO_OType = GPIO_OType_OD,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_PuPd  = GPIO_PuPd_NOPULL,
    };
    GPIO_Init(GTP_I2C_SDA_PORT, &gpio);
}

/* ---- 软件 I2C 基础 (毫秒级) ---- */
static void i2c_start(void)
{
    I2C_SDA_1(); i2c_delay(1);
    I2C_SCL_1(); i2c_delay(1);
    I2C_SDA_0(); i2c_delay(1);
    I2C_SCL_0(); i2c_delay(1);
}

static void i2c_stop(void)
{
    I2C_SDA_0(); i2c_delay(1);
    I2C_SCL_1(); i2c_delay(1);
    I2C_SDA_1(); i2c_delay(1);
}

/* ACK: 0=ACK, 1=NACK */
static uint8_t i2c_wait_ack(void)
{
    uint8_t ack = 1;
    TickType_t t0 = xTaskGetTickCount();
    sda_to_input();

    I2C_SCL_1(); i2c_delay(1);
    ack = I2C_SDA_READ();
    I2C_SCL_0(); i2c_delay(1);

    sda_to_output();

    /* 超时检测: 超时则强制 NACK 返回 */
    if ((xTaskGetTickCount() - t0) > pdMS_TO_TICKS(I2C_TIMEOUT_MS)) {
        return 1;   /* NACK */
    }
    return ack;
}

static void i2c_write_byte(uint8_t byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        if (byte & 0x80) I2C_SDA_1(); else I2C_SDA_0();
        i2c_delay(1);
        I2C_SCL_1(); i2c_delay(1);
        I2C_SCL_0(); i2c_delay(1);
        byte <<= 1;
    }
}

static uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t i, byte = 0;
    sda_to_input();
    for (i = 0; i < 8; i++) {
        byte <<= 1;
        I2C_SCL_1(); i2c_delay(1);
        if (I2C_SDA_READ()) byte |= 1;
        I2C_SCL_0(); i2c_delay(1);
    }
    sda_to_output();
    if (ack) I2C_SDA_1(); else I2C_SDA_0();
    i2c_delay(1);
    I2C_SCL_1(); i2c_delay(1);
    I2C_SCL_0(); i2c_delay(1);
    return byte;
}

/* ---- GT9157 寄存器读写 (16-bit 地址, 大端序) ---- */
static uint8_t gtp_write_reg(uint16_t reg, const uint8_t *buf, uint8_t len)
{
    i2c_start();
    i2c_write_byte(GTP_ADDR_W);
    if (i2c_wait_ack()) goto fail;
    i2c_write_byte(reg >> 8);
    if (i2c_wait_ack()) goto fail;
    i2c_write_byte(reg & 0xFF);
    if (i2c_wait_ack()) goto fail;

    while (len--) {
        i2c_write_byte(*buf++);
        if (i2c_wait_ack()) goto fail;
    }
    i2c_stop();
    return 1;
fail:
    i2c_stop();
    return 0;
}

static uint8_t gtp_read_reg(uint16_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;
    i2c_start();
    i2c_write_byte(GTP_ADDR_W);
    if (i2c_wait_ack()) goto fail;
    i2c_write_byte(reg >> 8);
    if (i2c_wait_ack()) goto fail;
    i2c_write_byte(reg & 0xFF);
    if (i2c_wait_ack()) goto fail;

    i2c_start();
    i2c_write_byte(GTP_ADDR_R);
    if (i2c_wait_ack()) goto fail;

    for (i = 0; i < len; i++) {
        buf[i] = i2c_read_byte(i == len - 1 ? 0 : 1);
    }
    i2c_stop();
    return 1;
fail:
    i2c_stop();
    return 0;
}

/* ---- GT9157 复位 (短延时) ---- */
static void gtp_reset(void)
{
    GPIO_InitTypeDef gpio;
    /* INT 输出低 */
    gpio.GPIO_Pin = GTP_INT_PIN;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GTP_INT_PORT, &gpio);
    GPIO_ResetBits(GTP_INT_PORT, GTP_INT_PIN);
    i2c_delay(RESET_DELAY_MS);

    /* RST 拉低 → 拉高 */
    gpio.GPIO_Pin = GTP_RST_PIN;
    GPIO_Init(GTP_RST_PORT, &gpio);
    GPIO_ResetBits(GTP_RST_PORT, GTP_RST_PIN);
    i2c_delay(RESET_DELAY_MS);
    GPIO_SetBits(GTP_RST_PORT, GTP_RST_PIN);
    i2c_delay(RESET_DELAY_MS);

    /* INT 输入 */
    gpio.GPIO_Pin = GTP_INT_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GTP_INT_PORT, &gpio);
    i2c_delay(RESET_DELAY_MS);
}

/* ---- 初始化 ---- */
uint8_t Touch_Init(void)
{
    GPIO_InitTypeDef gpio;

    /* 时钟 */
    RCC_AHB1PeriphClockCmd(GTP_I2C_SCL_CLK | GTP_I2C_SDA_CLK |
                            GTP_RST_CLK | GTP_INT_CLK, ENABLE);

    /* SCL 开漏 */
    gpio.GPIO_Pin   = GTP_I2C_SCL_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GTP_I2C_SCL_PORT, &gpio);

    /* SDA 开漏初始高 */
    gpio.GPIO_Pin   = GTP_I2C_SDA_PIN;
    GPIO_Init(GTP_I2C_SDA_PORT, &gpio);
    I2C_SDA_1();

    /* 复位 */
    gtp_reset();

    /* 验证通信 */
    uint8_t ver[4] = {0};
    if (!gtp_read_reg(GTP_REG_VERSION, ver, 4)) {
        printf("[Touch] I2C FAIL (no ACK)\n");
        return 0;
    }
    printf("[Touch] OK ver=%02X %02X %02X %02X\n", ver[0], ver[1], ver[2], ver[3]);

    touch_ready = 1;
    return 1;
}

/* ---- 扫描触摸点 ---- */
uint8_t Touch_Scan(void)
{
    uint8_t buf[8];
    uint8_t points;

    if (!touch_ready) return 0;

    /* 读触摸状态寄存器 (1 字节) */
    if (!gtp_read_reg(GTP_READ_COOR_ADDR, buf, 1)) return 0;
    points = buf[0] & 0x0F;
    if (points == 0) return 0;
    if (points > 5) points = 5;

    /* 读第一个触摸点 (8 字节) */
    if (!gtp_read_reg(GTP_READ_COOR_ADDR + 1, buf, 8)) return 0;

    /* 坐标: X = (x_h[3:0] << 8) | x_l */
    touch_x = ((uint16_t)(buf[1] & 0x0F) << 8) | buf[2];
    touch_y = ((uint16_t)(buf[3] & 0x0F) << 8) | buf[4];

    /* 清状态寄存器 */
    buf[0] = 0;
    gtp_write_reg(GTP_READ_COOR_ADDR, buf, 1);

    if (touch_x >= 800 || touch_y >= 480) return 0;
    return 1;
}

uint16_t Touch_GetX(void) { return touch_x; }
uint16_t Touch_GetY(void) { return touch_y; }
