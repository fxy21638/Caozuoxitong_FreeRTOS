#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "stm32f4xx.h"
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/*  帧格式常量：AA + Addr + Len + Type + Data[N] + CRC + 55                  */
/* -------------------------------------------------------------------------- */
#define PROTOCOL_HEADER     0xAA
#define PROTOCOL_TAIL       0x55

/* 最大载荷 = Type(1) + Data，不含帧头/地址/长度/CRC/帧尾 */
#define PROTOCOL_MAX_DATA   32
/* 缓冲区大小 = 帧头(1) + 地址(1) + 长度(1) + 载荷(32) + CRC(1) + 帧尾(1) */
#define PROTOCOL_BUF_SIZE   37

/* -------------------------------------------------------------------------- */
/*  消息类型                                                                   */
/* -------------------------------------------------------------------------- */
#define MSG_TYPE_TEMP_HUMI  0x01    /* DHT 温湿度                               */
#define MSG_TYPE_MPU6050    0x02    /* MPU6050 姿态角 (pitch/roll/yaw)          */
#define MSG_TYPE_LED_CTRL   0x03    /* LED 亮灭控制                             */

/* -------------------------------------------------------------------------- */
/*  RS485 地址                                                                */
/* -------------------------------------------------------------------------- */
#define ADDR_MANAGER        0x01
#define ADDR_COLLECTOR_1    0x02
#define ADDR_COLLECTOR_2    0x03

/* -------------------------------------------------------------------------- */
/*  协议状态机                                                                 */
/* -------------------------------------------------------------------------- */
typedef enum {
    STATE_WAIT_HEADER = 0,  /* 等待帧头 0xAA                                   */
    STATE_WAIT_ADDR,        /* 等待地址字节                                     */
    STATE_WAIT_LEN,         /* 等待长度字节                                     */
    STATE_WAIT_TYPE,        /* 等待消息类型                                     */
    STATE_WAIT_DATA,        /* 等待数据载荷                                     */
    STATE_WAIT_CRC,         /* 等待 CRC 校验字节                                */
    STATE_WAIT_TAIL         /* 等待帧尾 0x55                                   */
} ProtocolState_t;

/* -------------------------------------------------------------------------- */
/*  解析后的帧信息                                                             */
/* -------------------------------------------------------------------------- */
typedef struct {
    uint8_t  addr;          /* 帧中的目标地址                                   */
    uint8_t  type;          /* 消息类型                                        */
    uint8_t  data[PROTOCOL_MAX_DATA];  /* 数据载荷（不含 Type 字节）              */
    uint8_t  data_len;      /* 数据载荷字节数                                   */
} ProtocolFrame_t;

/* -------------------------------------------------------------------------- */
/*  协议解析器上下文                                                            */
/* -------------------------------------------------------------------------- */
typedef struct {
    ProtocolState_t state;          /* 当前状态机状态                            */
    uint8_t  rx_buf[PROTOCOL_BUF_SIZE]; /* 接收缓冲                             */
    uint16_t byte_count;            /* 已接收字节数                              */
    uint8_t  exp_data_len;         /* 还需接收的数据字节数                        */
    uint8_t  frame_ready;          /* 1 = 已有完整有效帧等待处理                  */
    ProtocolFrame_t frame;         /* 最近解析成功的帧                           */
    uint8_t  local_addr;           /* 本机 RS485 地址                           */
    uint8_t  is_manager;           /* 1 = 管理端（接收所有地址）                   */
} ProtocolParser_t;

/* -------------------------------------------------------------------------- */
/*  公开接口                                                                   */
/* -------------------------------------------------------------------------- */
void     Protocol_Init(ProtocolParser_t *parser, uint8_t local_addr, uint8_t is_manager);
void     Protocol_Reset(ProtocolParser_t *parser);
uint8_t  Protocol_FeedByte(ProtocolParser_t *parser, uint8_t byte);  /* 返回 1=帧就绪 */
uint8_t  Protocol_IsFrameReady(ProtocolParser_t *parser);
void     Protocol_ClearFrame(ProtocolParser_t *parser);

/* CRC-8 校验（多项式 0x07） */
uint8_t  CRC8_Compute(const uint8_t *data, uint16_t len);

/* 组帧函数：返回帧总长度，写入 out_buf */
uint8_t  Protocol_BuildRequest(uint8_t *out_buf, uint8_t dest_addr, uint8_t msg_type);
uint8_t  Protocol_BuildResponse(uint8_t *out_buf, uint8_t dest_addr,
                                uint8_t msg_type, const uint8_t *data, uint8_t data_len);

#endif /* __PROTOCOL_H */
