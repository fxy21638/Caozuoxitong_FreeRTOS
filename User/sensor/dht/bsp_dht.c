#include "bsp_dht.h"
#include "FreeRTOS.h"
#include "task.h"

/* -------------------------------------------------------------------------- */
/*  Microsecond delay using DWT cycle counter (Cortex-M4, 180MHz)             */
/* -------------------------------------------------------------------------- */
static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void DWT_Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

/* -------------------------------------------------------------------------- */
/*  GPIO mode switch (single-wire protocol)                                   */
/* -------------------------------------------------------------------------- */
static void DHT_Mode_Out(void)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin   = DHT_GPIO_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(DHT_GPIO_PORT, &gpio);
}

static void DHT_Mode_In(void)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin   = DHT_GPIO_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_IN;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(DHT_GPIO_PORT, &gpio);
}

/* -------------------------------------------------------------------------- */
/*  DHT11 low-level protocol                                                  */
/* -------------------------------------------------------------------------- */
static void DHT_Rst(void)
{
    DHT_Mode_Out();
    GPIO_ResetBits(DHT_GPIO_PORT, DHT_GPIO_PIN);   /* low 20ms */
    DWT_Delay_us(20000);
    GPIO_SetBits(DHT_GPIO_PORT, DHT_GPIO_PIN);     /* high 13us */
    DWT_Delay_us(13);
}

static uint8_t DHT_Check(void)
{
    uint8_t retry = 0;
    DHT_Mode_In();
    /* Wait for DHT11 to pull low (response, 40-80us) */
    while (GPIO_ReadInputDataBit(DHT_GPIO_PORT, DHT_GPIO_PIN) && retry < 100) {
        retry++;
        DWT_Delay_us(1);
    }
    if (retry >= 100) return 1;

    retry = 0;
    /* Wait for DHT11 to pull high (ready) */
    while (!GPIO_ReadInputDataBit(DHT_GPIO_PORT, DHT_GPIO_PIN) && retry < 100) {
        retry++;
        DWT_Delay_us(1);
    }
    if (retry >= 100) return 1;

    return 0;
}

static uint8_t DHT_Read_Bit(void)
{
    uint8_t retry = 0;
    /* Wait for signal to go low (start of bit) */
    while (GPIO_ReadInputDataBit(DHT_GPIO_PORT, DHT_GPIO_PIN) && retry < 100) {
        retry++;
        DWT_Delay_us(1);
    }
    retry = 0;
    /* Wait for signal to go high */
    while (!GPIO_ReadInputDataBit(DHT_GPIO_PORT, DHT_GPIO_PIN) && retry < 100) {
        retry++;
        DWT_Delay_us(1);
    }
    DWT_Delay_us(40);   /* Sample after 40us */
    if (GPIO_ReadInputDataBit(DHT_GPIO_PORT, DHT_GPIO_PIN))
        return 1;
    else
        return 0;
}

static uint8_t DHT_Read_Byte(void)
{
    uint8_t i, dat = 0;
    for (i = 0; i < 8; i++) {
        dat <<= 1;
        dat |= DHT_Read_Bit();
    }
    return dat;
}

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */
uint8_t DHT_Init(void)
{
    GPIO_InitTypeDef gpio;

    DWT_Init();

    RCC_AHB1PeriphClockCmd(DHT_GPIO_CLK, ENABLE);

    gpio.GPIO_Pin   = DHT_GPIO_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(DHT_GPIO_PORT, &gpio);
    GPIO_SetBits(DHT_GPIO_PORT, DHT_GPIO_PIN);

    /* Initial reset + check */
    DHT_Rst();
    return DHT_Check();   /* 0 = DHT present, 1 = not found */
}

uint8_t DHT_Read_Data(uint8_t *temp, uint8_t *humi)
{
    uint8_t buf[5], i;

    taskENTER_CRITICAL();   /* protect DHT timing */
    DHT_Rst();
    if (DHT_Check() == 0) {
        for (i = 0; i < 5; i++) buf[i] = DHT_Read_Byte();
        taskEXIT_CRITICAL();

        if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4]) {
            *humi = buf[0];    /* integer humidity */
            *temp = buf[2];    /* integer temperature */
            return 0;
        }
        return 2;   /* checksum error */
    }
    taskEXIT_CRITICAL();
    return 1;       /* DHT not responding */
}

uint8_t DHT_Read_Data_Float(float *temp, float *humi)
{
    uint8_t buf[5], i;

    taskENTER_CRITICAL();
    DHT_Rst();
    if (DHT_Check() == 0) {
        for (i = 0; i < 5; i++) buf[i] = DHT_Read_Byte();
        taskEXIT_CRITICAL();

        if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4]) {
            /* DHT11: buf[0]=humi_int, buf[1]=humi_dec (always 0 for DHT11) */
            /*        buf[2]=temp_int, buf[3]=temp_dec (always 0 for DHT11) */
            *humi = (float)buf[0] + (float)buf[1] * 0.1f;
            *temp = (float)buf[2] + (float)buf[3] * 0.1f;
            return 0;
        }
        return 2;
    }
    taskEXIT_CRITICAL();
    return 1;
}
