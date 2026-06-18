#include "data_store.h"
#include <string.h>

/* ========================================================================== */
/*  初始化存储上下文                                                              */
/* ========================================================================== */
void DataStore_Init(DataStore_t *store)
{
    memset(store, 0, sizeof(DataStore_t));
}

/* ========================================================================== */
/*  追加一条记录 (环形覆盖)                                                      */
/* ========================================================================== */
void DataStore_Push(DataStore_t *store, const SensorRecord_t *rec)
{
    store->records[store->head] = *rec;
    store->head = (store->head + 1) % STORE_DEPTH;
    if (store->count < STORE_DEPTH) {
        store->count++;
    }
}

/* ========================================================================== */
/*  获取最新一条记录                                                              */
/* ========================================================================== */
uint8_t DataStore_GetLatest(DataStore_t *store, SensorRecord_t *out)
{
    if (store->count == 0) return 0;

    /* 最新记录在 head 前一个位置 */
    uint8_t idx = (store->head == 0) ? (STORE_DEPTH - 1) : (store->head - 1);
    *out = store->records[idx];
    return 1;
}

/* ========================================================================== */
/*  按索引获取历史记录 (0 = 最新, count-1 = 最旧)                                  */
/* ========================================================================== */
uint8_t DataStore_Get(DataStore_t *store, uint8_t index, SensorRecord_t *out)
{
    if (index >= store->count) return 0;

    /* 最新 = head-1, 往前 index 条 */
    int16_t idx = (int16_t)store->head - 1 - (int16_t)index;
    while (idx < 0) idx += STORE_DEPTH;
    *out = store->records[(uint8_t)idx];
    return 1;
}

/* ========================================================================== */
/*  返回当前存储数量                                                              */
/* ========================================================================== */
uint8_t DataStore_Count(DataStore_t *store)
{
    return store->count;
}

/* ========================================================================== */
/*  清空存储                                                                     */
/* ========================================================================== */
void DataStore_Clear(DataStore_t *store)
{
    memset(store, 0, sizeof(DataStore_t));
}
