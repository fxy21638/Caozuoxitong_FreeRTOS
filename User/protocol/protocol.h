/**
  *********************************************************************
  * @file    protocol.h
  * @brief   串口协议解析模块 —— 类型定义与接口声明
  *
  *          超时分包机制：字节间间隔 < 200ms 视为同一帧，
  *          200ms 无新字节后校验帧格式 (AA + Len + Data + 55)
  *********************************************************************
  */

#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "stm32f4xx.h"
#include <stdint.h>

/* 协议帧头与帧尾 */
#define PROTOCOL_HEADER     0xAA
#define PROTOCOL_TAIL       0x55

/* 最大用户数据长度 */
#define PROTOCOL_MAX_DATA   256

/* 帧缓冲区大小（帧头+长度+数据+帧尾） */
#define PROTOCOL_BUF_SIZE   (PROTOCOL_MAX_DATA + 3)

/* 协议解析器结构体 */
typedef struct {
    uint8_t  rx_buf[PROTOCOL_BUF_SIZE];  /* 帧缓冲区 */
    uint16_t byte_count;                  /* 已接收字节总数 */
    uint8_t  frame_valid;                 /* 校验通过标志 */
} ProtocolParser_t;

/* 接口函数 */
void     Protocol_Init(ProtocolParser_t *parser);
void     Protocol_Reset(ProtocolParser_t *parser);
void     Protocol_AddByte(ProtocolParser_t *parser, uint8_t byte);
uint8_t  Protocol_Validate(ProtocolParser_t *parser);
uint8_t  Protocol_GetDataLen(ProtocolParser_t *parser);
uint8_t* Protocol_GetData(ProtocolParser_t *parser);

#endif /* __PROTOCOL_H */
