#include "touch.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

/* ========================================================================== */
/*  touch.c — 触摸屏驱动 (I2C3: PA8=SCL, PC9=SDA)                              */
/*                                                                             */
/*  自动探测 GT9157 (0x14 / 0x5D) 和 FT5x06 (0x38)                             */
/*  I2C 模式匹配 bsp_mpu6050_i2c.c (已验证可用)                                  */
/* ========================================================================== */

/* ---- I2C3 引脚 ---- */
#define TOUCH_I2C               I2C3
#define TOUCH_I2C_CLK           RCC_APB1Periph_I2C3

#define TOUCH_SCL_PORT          GPIOA
#define TOUCH_SCL_PIN           GPIO_Pin_8
#define TOUCH_SCL_PIN_SRC       GPIO_PinSource8
#define TOUCH_SCL_AF            GPIO_AF_I2C3
#define TOUCH_SCL_PORT_CLK      RCC_AHB1Periph_GPIOA

#define TOUCH_SDA_PORT          GPIOC
#define TOUCH_SDA_PIN           GPIO_Pin_9
#define TOUCH_SDA_PIN_SRC       GPIO_PinSource9
#define TOUCH_SDA_AF            GPIO_AF_I2C3
#define TOUCH_SDA_PORT_CLK      RCC_AHB1Periph_GPIOC

#define I2C_TIMEOUT             10000

/* ---- GT9157 寄存器 (16-bit 地址, 大端序传输) ---- */
#define GT9157_REG_CFG          0x8047
#define GT9157_REG_TOUCH_STATUS 0x814E
#define GT9157_REG_POINT1       0x814F

/* ---- FT5x06 寄存器 (8-bit 地址) ---- */
#define FT5X06_REG_MODE          0x00
#define FT5X06_REG_TD_STATUS     0x02
#define FT5X06_REG_P1_XH         0x03
#define FT5X06_REG_P1_XL         0x04
#define FT5X06_REG_P1_YH         0x05
#define FT5X06_REG_P1_YL         0x06

/* ---- 触摸点数据偏移 ---- */
#define PT_OFF_TRACK_ID   0
#define PT_OFF_X_H        1
#define PT_OFF_X_L        2
#define PT_OFF_Y_H        3
#define PT_OFF_Y_L        4

/* ---- 内部状态 ---- */
static uint16_t touch_x = 0;
static uint16_t touch_y = 0;
static uint8_t  touch_ready = 0;
static uint8_t  touch_type  = 0;          /* 1=GT9157, 2=FT5x06              */

/* ---- 当前探测到的 I2C 7-bit 地址 ---- */
static uint8_t  touch_i2c_addr = 0;

/* ========================================================================== */
/*  I2C3 操作 (匹配 bsp_mpu6050_i2c.c 风格)                                     */
/* ========================================================================== */

/** @brief I2C3 写: dev_addr(7-bit) + reg(8-bit) + data */
static uint8_t I2C3_WriteReg8(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    uint32_t timeout;

    I2C_GenerateSTART(TOUCH_I2C, ENABLE);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(TOUCH_I2C, I2C_EVENT_MASTER_MODE_SELECT) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(TOUCH_I2C, ENABLE); return 1; }
    }

    I2C_Send7bitAddress(TOUCH_I2C, addr << 1, I2C_Direction_Transmitter);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(TOUCH_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(TOUCH_I2C, ENABLE); return 1; }
    }

    I2C_SendData(TOUCH_I2C, reg);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(TOUCH_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(TOUCH_I2C, ENABLE); return 1; }
    }

    while (len--) {
        I2C_SendData(TOUCH_I2C, *buf++);
        timeout = I2C_TIMEOUT;
        while (I2C_CheckEvent(TOUCH_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR) {
            if (--timeout == 0) { I2C_GenerateSTOP(TOUCH_I2C, ENABLE); return 1; }
        }
    }

    I2C_GenerateSTOP(TOUCH_I2C, ENABLE);
    return 0;
}

/** @brief I2C3 读: dev_addr(7-bit) + reg(8-bit) → data */
static uint8_t I2C3_ReadReg8(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    uint32_t timeout;

    /* Phase 1: write register address */
    I2C_GenerateSTART(TOUCH_I2C, ENABLE);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(TOUCH_I2C, I2C_EVENT_MASTER_MODE_SELECT) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(TOUCH_I2C, ENABLE); return 1; }
    }

    I2C_Send7bitAddress(TOUCH_I2C, addr << 1, I2C_Direction_Transmitter);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(TOUCH_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(TOUCH_I2C, ENABLE); return 1; }
    }

    I2C_SendData(TOUCH_I2C, reg);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(TOUCH_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(TOUCH_I2C, ENABLE); return 1; }
    }

    /* Phase 2: repeated START + read */
    I2C_GenerateSTART(TOUCH_I2C, ENABLE);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(TOUCH_I2C, I2C_EVENT_MASTER_MODE_SELECT) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(TOUCH_I2C, ENABLE); return 1; }
    }

    I2C_Send7bitAddress(TOUCH_I2C, addr << 1, I2C_Direction_Receiver);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(TOUCH_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(TOUCH_I2C, ENABLE); return 1; }
    }

    while (len) {
        if (len == 1) {
            I2C_AcknowledgeConfig(TOUCH_I2C, DISABLE);
        }
        timeout = I2C_TIMEOUT;
        while (I2C_CheckEvent(TOUCH_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED) == ERROR) {
            if (--timeout == 0) { I2C_GenerateSTOP(TOUCH_I2C, ENABLE); return 1; }
        }
        *buf++ = I2C_ReceiveData(TOUCH_I2C);
        len--;
    }

    I2C_GenerateSTOP(TOUCH_I2C, ENABLE);
    I2C_AcknowledgeConfig(TOUCH_I2C, ENABLE);
    return 0;
}

/* ========================================================================== */
/*  GT9157 寄存器读写 (16-bit 寄存器地址, 大端序)                                  */
/* ========================================================================== */

/** @brief GT9157 写: 16-bit reg + 8-bit data[] */
static uint8_t GT9157_WriteReg(uint16_t reg, const uint8_t *data, uint8_t len)
{
    uint32_t timeout;
    uint8_t addr_hi = (uint8_t)(reg >> 8);
    uint8_t addr_lo = (uint8_t)(reg & 0xFF);

    I2C_GenerateSTART(TOUCH_I2C, ENABLE);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(TOUCH_I2C, I2C_EVENT_MASTER_MODE_SELECT) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(TOUCH_I2C, ENABLE); return 1; }
    }

    I2C_Send7bitAddress(TOUCH_I2C, touch_i2c_addr << 1, I2C_Direction_Transmitter);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(TOUCH_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(TOUCH_I2C, ENABLE); return 1; }
    }

    /* 16-bit 寄存器地址 (高字节在前) */
    I2C_SendData(TOUCH_I2C, addr_hi);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(TOUCH_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(TOUCH_I2C, ENABLE); return 1; }
    }
    I2C_SendData(TOUCH_I2C, addr_lo);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(TOUCH_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(TOUCH_I2C, ENABLE); return 1; }
    }

    while (len--) {
        I2C_SendData(TOUCH_I2C, *data++);
        timeout = I2C_TIMEOUT;
        while (I2C_CheckEvent(TOUCH_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR) {
            if (--timeout == 0) { I2C_GenerateSTOP(TOUCH_I2C, ENABLE); return 1; }
        }
    }

    I2C_GenerateSTOP(TOUCH_I2C, ENABLE);
    return 0;
}

/** @brief GT9157 读: 16-bit reg → data[] */
static uint8_t GT9157_ReadReg(uint16_t reg, uint8_t *data, uint8_t len)
{
    uint32_t timeout;
    uint8_t addr_hi = (uint8_t)(reg >> 8);
    uint8_t addr_lo = (uint8_t)(reg & 0xFF);

    /* Phase 1: write 16-bit register address */
    I2C_GenerateSTART(TOUCH_I2C, ENABLE);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(TOUCH_I2C, I2C_EVENT_MASTER_MODE_SELECT) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(TOUCH_I2C, ENABLE); return 1; }
    }

    I2C_Send7bitAddress(TOUCH_I2C, touch_i2c_addr << 1, I2C_Direction_Transmitter);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(TOUCH_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(TOUCH_I2C, ENABLE); return 1; }
    }

    I2C_SendData(TOUCH_I2C, addr_hi);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(TOUCH_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(TOUCH_I2C, ENABLE); return 1; }
    }
    I2C_SendData(TOUCH_I2C, addr_lo);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(TOUCH_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(TOUCH_I2C, ENABLE); return 1; }
    }

    /* Phase 2: repeated START + read */
    I2C_GenerateSTART(TOUCH_I2C, ENABLE);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(TOUCH_I2C, I2C_EVENT_MASTER_MODE_SELECT) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(TOUCH_I2C, ENABLE); return 1; }
    }

    I2C_Send7bitAddress(TOUCH_I2C, touch_i2c_addr << 1, I2C_Direction_Receiver);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(TOUCH_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(TOUCH_I2C, ENABLE); return 1; }
    }

    while (len) {
        if (len == 1) {
            I2C_AcknowledgeConfig(TOUCH_I2C, DISABLE);
        }
        timeout = I2C_TIMEOUT;
        while (I2C_CheckEvent(TOUCH_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED) == ERROR) {
            if (--timeout == 0) { I2C_GenerateSTOP(TOUCH_I2C, ENABLE); return 1; }
        }
        *data++ = I2C_ReceiveData(TOUCH_I2C);
        len--;
    }

    I2C_GenerateSTOP(TOUCH_I2C, ENABLE);
    I2C_AcknowledgeConfig(TOUCH_I2C, ENABLE);
    return 0;
}

/* ========================================================================== */
/*  芯片探测: 尝试读取指定地址的寄存器, 返回 1=芯片在线                           */
/* ========================================================================== */
static uint8_t Touch_ProbeAddr(uint8_t addr, uint8_t is_gt9157)
{
    uint8_t buf[2];
    uint8_t ret;

    if (is_gt9157) {
        touch_i2c_addr = addr;
        ret = GT9157_ReadReg(GT9157_REG_CFG, buf, 2);
    } else {
        ret = I2C3_ReadReg8(addr, FT5X06_REG_MODE, 1, buf);
    }
    return (ret == 0) ? 1 : 0;
}

/* ========================================================================== */
/*  I2C3 硬件初始化                                                              */
/*                                                                             */
/*  !!! 关键: PA8(GPIOA) 和 PC9(GPIOC) 在不同端口, 必须分开 GPIO_Init,            */
/*      否则 Pin 合并 (PA8|PC9) 会误配置 PA9(调试串口 TX) 和 PC8(MPU6050 INT)    */
/* ========================================================================== */
static void I2C3_HW_Init(void)
{
    GPIO_InitTypeDef  gpio;
    I2C_InitTypeDef   i2c;

    RCC_AHB1PeriphClockCmd(TOUCH_SCL_PORT_CLK | TOUCH_SDA_PORT_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(TOUCH_I2C_CLK, ENABLE);

    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    /* SCL: PA8 (单独初始化, 避免误触 PA9) */
    gpio.GPIO_Pin = TOUCH_SCL_PIN;
    GPIO_Init(TOUCH_SCL_PORT, &gpio);
    GPIO_PinAFConfig(TOUCH_SCL_PORT, TOUCH_SCL_PIN_SRC, TOUCH_SCL_AF);

    /* SDA: PC9 (单独初始化, 避免误触 PC8) */
    gpio.GPIO_Pin = TOUCH_SDA_PIN;
    GPIO_Init(TOUCH_SDA_PORT, &gpio);
    GPIO_PinAFConfig(TOUCH_SDA_PORT, TOUCH_SDA_PIN_SRC, TOUCH_SDA_AF);

    I2C_DeInit(TOUCH_I2C);
    i2c.I2C_Mode                = I2C_Mode_I2C;
    i2c.I2C_DutyCycle           = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1         = 0x00;
    i2c.I2C_Ack                 = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed          = 400000;
    I2C_Init(TOUCH_I2C, &i2c);
    I2C_Cmd(TOUCH_I2C, ENABLE);
}

/* ========================================================================== */
/*  公开接口                                                                     */
/* ========================================================================== */

/**
  * @brief  初始化触摸控制器 — 自动探测 GT9157 / FT5x06
  * @retval 1 = 成功, 0 = 失败
  */
uint8_t Touch_Init(void)
{
    I2C3_HW_Init();

    /* 上电后触摸芯片需要时间启动 (GT9157 从内部 flash 加载配置) */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* 1) 尝试 GT9157 主地址 0x14 */
    printf("[Touch] probe GT9157 @ 0x14... ");
    if (Touch_ProbeAddr(0x14, 1)) {
        printf("OK\n");
        touch_type  = 1;
        touch_ready = 1;
        return 1;
    }
    printf("FAIL\n");

    /* 2) 尝试 GT9157 备选地址 0x5D */
    printf("[Touch] probe GT9157 @ 0x5D... ");
    if (Touch_ProbeAddr(0x5D, 1)) {
        printf("OK\n");
        touch_type  = 1;
        touch_ready = 1;
        return 1;
    }
    printf("FAIL\n");

    /* 3) 尝试 FT5x06 地址 0x38 */
    printf("[Touch] probe FT5x06 @ 0x38... ");
    if (Touch_ProbeAddr(0x38, 0)) {
        printf("OK\n");
        touch_type  = 2;
        touch_ready = 1;
        return 1;
    }
    printf("FAIL\n");

    /* 均失败 — 触摸不可用, 仅用按键 */
    printf("[Touch] no controller found, using keys only\n");
    touch_ready = 0;
    return 0;
}

/**
  * @brief  扫描触摸状态
  * @retval 1 = 检测到触摸, 0 = 无触摸
  */
uint8_t Touch_Scan(void)
{
    uint8_t buf[8];
    uint8_t points;

    if (!touch_ready) return 0;

    if (touch_type == 1) {
        /* ---- GT9157 ---- */
        if (GT9157_ReadReg(GT9157_REG_TOUCH_STATUS, &points, 1) != 0)
            return 0;

        points &= 0x0F;
        if (points == 0) return 0;

        if (GT9157_ReadReg(GT9157_REG_POINT1, buf, 8) != 0)
            return 0;

        touch_x = (uint16_t)((buf[PT_OFF_X_H] & 0x0F) << 8) | buf[PT_OFF_X_L];
        touch_y = (uint16_t)((buf[PT_OFF_Y_H] & 0x0F) << 8) | buf[PT_OFF_Y_L];

        /* 清除状态, 通知 GT9157 数据已读 */
        buf[0] = 0;
        GT9157_WriteReg(GT9157_REG_TOUCH_STATUS, buf, 1);

        if (touch_x >= 800 || touch_y >= 480) return 0;
        return 1;

    } else if (touch_type == 2) {
        /* ---- FT5x06 ---- */
        if (I2C3_ReadReg8(touch_i2c_addr, FT5X06_REG_TD_STATUS, 1, &points) != 0)
            return 0;

        points &= 0x0F;
        if (points == 0) return 0;

        /* 读第 1 个触摸点: 4 字节 (XH, XL, YH, YL) */
        if (I2C3_ReadReg8(touch_i2c_addr, FT5X06_REG_P1_XH, 4, buf) != 0)
            return 0;

        touch_x = (uint16_t)((buf[0] & 0x0F) << 8) | buf[1];
        touch_y = (uint16_t)((buf[2] & 0x0F) << 8) | buf[3];

        /* FT5x06 坐标可能与屏幕旋转相关, 需要时可以交换/翻转 */
        if (touch_x >= 800 || touch_y >= 480) {
            uint16_t tmp = touch_x;
            touch_x = touch_y;
            touch_y = tmp;
            if (touch_x >= 800 || touch_y >= 480) return 0;
        }
        return 1;
    }

    return 0;
}

uint16_t Touch_GetX(void) { return touch_x; }
uint16_t Touch_GetY(void) { return touch_y; }
