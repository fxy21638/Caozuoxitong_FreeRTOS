/**
  *********************************************************************
  * @file    protocol.c
  * @brief   串口协议解析模块
  *
  *          超时分包机制：持续累积字节（每个字节重置 200ms 定时器），
  *          定时器超时后校验累积的帧是否合法。
  *          合法帧格式: 0xAA + 长度(N) + 数据[N 字节] + 0x55
  *********************************************************************
  */

#include "protocol.h"
#include <string.h>

/**
  * @brief  初始化协议解析器
  * @param  parser: 解析器指针
  * @retval 无
  */
void Protocol_Init(ProtocolParser_t *parser)
{
    memset(parser, 0, sizeof(ProtocolParser_t));
}

/**
  * @brief  复位协议解析器（超时后丢弃当前帧）
  * @param  parser: 解析器指针
  * @retval 无
  */
void Protocol_Reset(ProtocolParser_t *parser)
{
    parser->byte_count = 0;
    parser->frame_valid = 0;
}

/**
  * @brief  向解析器累积一个字节
  * @param  parser: 解析器指针
  * @param  byte: 收到的字节
  * @retval 无
  */
void Protocol_AddByte(ProtocolParser_t *parser, uint8_t byte)
{
    if (parser->byte_count < PROTOCOL_BUF_SIZE)
    {
        parser->rx_buf[parser->byte_count] = byte;
        parser->byte_count++;
    }
}

/**
  * @brief  校验累积的帧是否合法
  *         检查: 帧头=0xAA, 帧尾=0x55, 长度字段匹配实际数据长度
  * @param  parser: 解析器指针
  * @retval 1 = 帧合法, 0 = 帧非法
  */
uint8_t Protocol_Validate(ProtocolParser_t *parser)
{
    uint8_t len_field;

    /* 至少需要帧头(1) + 长度(1) + 帧尾(1) = 3 字节 */
    if (parser->byte_count < 3)
        return 0;

    /* 校验帧头 */
    if (parser->rx_buf[0] != PROTOCOL_HEADER)
        return 0;

    /* 校验帧尾 */
    if (parser->rx_buf[parser->byte_count - 1] != PROTOCOL_TAIL)
        return 0;

    /* 长度字段应等于数据字节数（总长 - 帧头 - 长度 - 帧尾） */
    len_field = parser->rx_buf[1];
    if (len_field != (parser->byte_count - 3))
        return 0;

    parser->frame_valid = 1;
    return 1;
}

/**
  * @brief  获取帧数据长度
  * @param  parser: 解析器指针
  * @retval 数据字节数
  */
uint8_t Protocol_GetDataLen(ProtocolParser_t *parser)
{
    return parser->rx_buf[1];
}

/**
  * @brief  获取帧数据指针
  * @param  parser: 解析器指针
  * @retval 指向数据首字节的指针
  */
uint8_t* Protocol_GetData(ProtocolParser_t *parser)
{
    return &parser->rx_buf[2];
}