#include "bsp_mpu6050_i2c.h"

/* FreeRTOS header for vTaskDelay / xTaskGetTickCount */
#include "FreeRTOS.h"
#include "task.h"

/* Timeout for I2C events (prevent bus hang) */
#define I2C_TIMEOUT  10000

static uint8_t I2C_WriteBytes(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    uint32_t timeout;

    /* Generate START */
    I2C_GenerateSTART(MPU6050_I2C, ENABLE);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(MPU6050_I2C, ENABLE); return 1; }
    }

    /* Send device address (write direction) */
    I2C_Send7bitAddress(MPU6050_I2C, addr << 1, I2C_Direction_Transmitter);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(MPU6050_I2C, ENABLE); return 1; }
    }

    /* Send register address */
    I2C_SendData(MPU6050_I2C, reg);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(MPU6050_I2C, ENABLE); return 1; }
    }

    /* Send data bytes */
    while (len--) {
        I2C_SendData(MPU6050_I2C, *buf++);
        timeout = I2C_TIMEOUT;
        while (I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR) {
            if (--timeout == 0) { I2C_GenerateSTOP(MPU6050_I2C, ENABLE); return 1; }
        }
    }

    /* Generate STOP */
    I2C_GenerateSTOP(MPU6050_I2C, ENABLE);
    return 0;
}

static uint8_t I2C_ReadBytes(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    uint32_t timeout;

    /* Generate START */
    I2C_GenerateSTART(MPU6050_I2C, ENABLE);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(MPU6050_I2C, ENABLE); return 1; }
    }

    /* Send device address (write direction) to set register pointer */
    I2C_Send7bitAddress(MPU6050_I2C, addr << 1, I2C_Direction_Transmitter);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(MPU6050_I2C, ENABLE); return 1; }
    }

    /* Send register address */
    I2C_SendData(MPU6050_I2C, reg);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(MPU6050_I2C, ENABLE); return 1; }
    }

    /* Repeated START */
    I2C_GenerateSTART(MPU6050_I2C, ENABLE);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(MPU6050_I2C, ENABLE); return 1; }
    }

    /* Send device address (read direction) */
    I2C_Send7bitAddress(MPU6050_I2C, addr << 1, I2C_Direction_Receiver);
    timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == ERROR) {
        if (--timeout == 0) { I2C_GenerateSTOP(MPU6050_I2C, ENABLE); return 1; }
    }

    /* Read bytes with ACK/NACK */
    while (len) {
        if (len == 1) {
            /* Last byte: NACK */
            I2C_AcknowledgeConfig(MPU6050_I2C, DISABLE);
        }
        timeout = I2C_TIMEOUT;
        while (I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED) == ERROR) {
            if (--timeout == 0) { I2C_GenerateSTOP(MPU6050_I2C, ENABLE); return 1; }
        }
        *buf++ = I2C_ReceiveData(MPU6050_I2C);
        len--;
    }

    /* Generate STOP */
    I2C_GenerateSTOP(MPU6050_I2C, ENABLE);
    I2C_AcknowledgeConfig(MPU6050_I2C, ENABLE);
    return 0;
}

void MPU6050_I2C_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    I2C_InitTypeDef   I2C_InitStructure;

    /* Enable clocks */
    RCC_AHB1PeriphClockCmd(MPU6050_I2C_SCL_CLK | MPU6050_I2C_SDA_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(MPU6050_I2C_CLK, ENABLE);

    /* Configure SCL (PB6) and SDA (PB7) as AF open-drain */
    GPIO_InitStructure.GPIO_Pin   = MPU6050_I2C_SCL_PIN | MPU6050_I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(MPU6050_I2C_SCL_GPIO, &GPIO_InitStructure);

    /* Connect pins to I2C1 AF */
    GPIO_PinAFConfig(MPU6050_I2C_SCL_GPIO, MPU6050_I2C_SCL_SOURCE, MPU6050_I2C_AF);
    GPIO_PinAFConfig(MPU6050_I2C_SDA_GPIO, MPU6050_I2C_SDA_SOURCE, MPU6050_I2C_AF);

    /* I2C configuration: 400kHz */
    I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1         = 0x00;
    I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed          = 400000;
    I2C_Init(MPU6050_I2C, &I2C_InitStructure);

    I2C_Cmd(MPU6050_I2C, ENABLE);
}

uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    return I2C_WriteBytes(addr, reg, len, buf);
}

uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    return I2C_ReadBytes(addr, reg, len, buf);
}

void Delay_ms(unsigned long ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void mget_ms(unsigned long *time)
{
    *time = xTaskGetTickCount();
}
