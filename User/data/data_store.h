#ifndef __DATA_STORE_H
#define __DATA_STORE_H

#include "stm32f4xx.h"
#include <stdint.h>

/* ========================================================================== */
/*  data_store.h — 传感器数据环形缓冲存储                                       */
/*                                                                             */
/*  每个采集前端本地保存最近 STORE_DEPTH 组传感器数据                             */
/*  管理端保存两个前端各 STORE_DEPTH 组数据                                      */
/* ========================================================================== */

/* 存储最大深度 (来自 config.h) */
#ifndef STORE_DEPTH
#define STORE_DEPTH  10
#endif

/* ========================================================================== */
/*  传感器数据类型                                                               */
/* ========================================================================== */
typedef enum {
    SENSOR_TYPE_DHT = 1,
    SENSOR_TYPE_MPU6050 = 2,
} SensorType_t;

/* ========================================================================== */
/*  单条传感器记录                                                               */
/* ========================================================================== */
typedef struct {
    SensorType_t type;
    uint32_t     tick;               /* xTaskGetTickCount() 记录时刻          */
    union {
        struct { float temp; float humi; } dht;
        struct { float pitch; float roll; float yaw; } mpu;
    } data;
} SensorRecord_t;

/* ========================================================================== */
/*  数据存储上下文 (每个传感器一个实例)                                            */
/* ========================================================================== */
typedef struct {
    SensorRecord_t records[STORE_DEPTH];
    uint8_t        head;             /* 写入位置 (环形)                       */
    uint8_t        count;            /* 当前存储数量 (0 ~ STORE_DEPTH)        */
} DataStore_t;

/* ========================================================================== */
/*  公开接口                                                                     */
/* ========================================================================== */
void     DataStore_Init(DataStore_t *store);
void     DataStore_Push(DataStore_t *store, const SensorRecord_t *rec);
uint8_t  DataStore_GetLatest(DataStore_t *store, SensorRecord_t *out);
uint8_t  DataStore_Get(DataStore_t *store, uint8_t index, SensorRecord_t *out);
uint8_t  DataStore_Count(DataStore_t *store);
void     DataStore_Clear(DataStore_t *store);

#endif /* __DATA_STORE_H */
