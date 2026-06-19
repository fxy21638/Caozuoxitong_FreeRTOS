/**
  *********************************************************************
  * @file    bsp_dht11.c
  * @author  fire
  * @version V1.0
  * @date    2026-06-10
  * @brief   DHT11 温湿度传感器驱动实现
  *********************************************************************
  */

#include "bsp_dht11.h"
#include "stm32f4xx.h"
#define DHT11_DATA_PIN            GPIO_Pin_2
#define DHT11_DATA_GPIO_PORT      GPIOE
#define DHT11_DATA_GPIO_CLK       RCC_AHB1Periph_GPIOE

static void DHT11_Delay_us(uint32_t us);
static void DHT11_Delay_ms(uint32_t ms);
static void DHT11_SetPinOutput(void);
static void DHT11_SetPinInput(void);
static void DHT11_SetPinLow(void);
static void DHT11_SetPinHigh(void);
static uint8_t DHT11_CheckResponse(void);
static uint8_t DHT11_ReadBit(void);
static void DHT11_DWT_Init(void);

void DHT11_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(DHT11_DATA_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = DHT11_DATA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(DHT11_DATA_GPIO_PORT, &GPIO_InitStructure);

    GPIO_SetBits(DHT11_DATA_GPIO_PORT, DHT11_DATA_PIN);
    DHT11_DWT_Init();
}

uint8_t DHT11_Read(uint8_t *pHumidity, uint8_t *pTemperature)
{
    uint8_t data[5] = {0};
    uint8_t i, j;

    if (pHumidity == NULL || pTemperature == NULL)
    {
        return DHT11_ERROR_TIMEOUT;
    }

    DHT11_SetPinOutput();
    DHT11_SetPinLow();
    DHT11_Delay_us(18000);
    DHT11_SetPinHigh();
    DHT11_Delay_us(30);
    DHT11_SetPinInput();

    if (DHT11_CheckResponse() == 0)
    {
        return DHT11_ERROR_TIMEOUT;
    }

    for (i = 0; i < 5; i++)
    {
        for (j = 0; j < 8; j++)
        {
            data[i] <<= 1;
            data[i] |= DHT11_ReadBit();
        }
    }

    if (data[4] != (uint8_t)(data[0] + data[1] + data[2] + data[3]))
    {
        return DHT11_ERROR_CHECKSUM;
    }

    *pHumidity = data[0];
    *pTemperature = data[2];

    return DHT11_OK;
}

uint8_t DHT11_ReadAverage(uint8_t *pHumidity, uint8_t *pTemperature, uint8_t sampleCount)
{
    uint32_t sumHumidity = 0;
    uint32_t sumTemperature = 0;
    uint8_t validCount = 0;
    uint8_t humidity = 0;
    uint8_t temperature = 0;
    uint8_t i = 0;

    if (pHumidity == NULL || pTemperature == NULL || sampleCount == 0)
    {
        return DHT11_ERROR_TIMEOUT;
    }

    for (i = 0; i < sampleCount; i++)
    {
        if (DHT11_Read(&humidity, &temperature) == DHT11_OK)
        {
            sumHumidity += humidity;
            sumTemperature += temperature;
            validCount++;
        }

        DHT11_Delay_ms(120);
    }

    if (validCount == 0)
    {
        return DHT11_ERROR_TIMEOUT;
    }

    *pHumidity = (uint8_t)(sumHumidity / validCount);
    *pTemperature = (uint8_t)(sumTemperature / validCount);

    return DHT11_OK;
}

static void DHT11_SetPinOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = DHT11_DATA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(DHT11_DATA_GPIO_PORT, &GPIO_InitStructure);
}

static void DHT11_SetPinInput(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = DHT11_DATA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(DHT11_DATA_GPIO_PORT, &GPIO_InitStructure);
}

static void DHT11_SetPinLow(void)
{
    GPIO_ResetBits(DHT11_DATA_GPIO_PORT, DHT11_DATA_PIN);
}

static void DHT11_SetPinHigh(void)
{
    GPIO_SetBits(DHT11_DATA_GPIO_PORT, DHT11_DATA_PIN);
}

static uint8_t DHT11_CheckResponse(void)
{
    uint32_t timeout = 0;

    /* 等待 DHT11 拉低应答 */
    while (GPIO_ReadInputDataBit(DHT11_DATA_GPIO_PORT, DHT11_DATA_PIN) == Bit_SET)
    {
        DHT11_Delay_us(1);
        if (++timeout > 100)
        {
            return 0;
        }
    }

    timeout = 0;
    while (GPIO_ReadInputDataBit(DHT11_DATA_GPIO_PORT, DHT11_DATA_PIN) == Bit_RESET)
    {
        DHT11_Delay_us(1);
        if (++timeout > 100)
        {
            return 0;
        }
    }

    timeout = 0;
    while (GPIO_ReadInputDataBit(DHT11_DATA_GPIO_PORT, DHT11_DATA_PIN) == Bit_SET)
    {
        DHT11_Delay_us(1);
        if (++timeout > 100)
        {
            return 0;
        }
    }

    return 1;
}

static uint8_t DHT11_ReadBit(void)
{
    uint32_t timeout = 0;

    while (GPIO_ReadInputDataBit(DHT11_DATA_GPIO_PORT, DHT11_DATA_PIN) == Bit_RESET)
    {
        DHT11_Delay_us(1);
        if (++timeout > 100)
        {
            return 0;
        }
    }

    DHT11_Delay_us(40);

    if (GPIO_ReadInputDataBit(DHT11_DATA_GPIO_PORT, DHT11_DATA_PIN) == Bit_SET)
    {
        while (GPIO_ReadInputDataBit(DHT11_DATA_GPIO_PORT, DHT11_DATA_PIN) == Bit_SET)
        {
            DHT11_Delay_us(1);
        }
        return 1;
    }

    return 0;
}

static void DHT11_DWT_Init(void)
{
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }

    DWT->CYCCNT = 0;
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0)
    {
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

static void DHT11_Delay_us(uint32_t us)
{
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < ticks)
    {
    }
}

static void DHT11_Delay_ms(uint32_t ms)
{
    while (ms--)
    {
        DHT11_Delay_us(1000);
    }
}
