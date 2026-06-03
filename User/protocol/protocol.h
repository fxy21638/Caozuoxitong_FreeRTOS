#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "stm32f4xx.h"
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/*  Frame constants:  AA + Addr + Len + Type + Data[N] + CRC + 55           */
/* -------------------------------------------------------------------------- */
#define PROTOCOL_HEADER     0xAA
#define PROTOCOL_TAIL       0x55

/* Max payload = Type(1) + Data, not counting header/addr/len/crc/tail */
#define PROTOCOL_MAX_DATA   32
/* Buffer size = Header(1) + Addr(1) + Len(1) + Payload(32) + CRC(1) + Tail(1) */
#define PROTOCOL_BUF_SIZE   37

/* -------------------------------------------------------------------------- */
/*  Message Types                                                            */
/* -------------------------------------------------------------------------- */
#define MSG_TYPE_TEMP_HUMI  0x01    /* DHT temperature + humidity             */
#define MSG_TYPE_MPU6050    0x02    /* MPU6050 pitch/roll/yaw                 */
#define MSG_TYPE_LED_CTRL   0x03    /* LED on/off control                     */

/* -------------------------------------------------------------------------- */
/*  Addresses                                                                */
/* -------------------------------------------------------------------------- */
#define ADDR_MANAGER        0x01
#define ADDR_COLLECTOR_1    0x02
#define ADDR_COLLECTOR_2    0x03

/* -------------------------------------------------------------------------- */
/*  Protocol State Machine States                                            */
/* -------------------------------------------------------------------------- */
typedef enum {
    STATE_WAIT_HEADER = 0,
    STATE_WAIT_ADDR,
    STATE_WAIT_LEN,
    STATE_WAIT_TYPE,
    STATE_WAIT_DATA,
    STATE_WAIT_CRC,
    STATE_WAIT_TAIL
} ProtocolState_t;

/* -------------------------------------------------------------------------- */
/*  Parsed Frame Info                                                        */
/* -------------------------------------------------------------------------- */
typedef struct {
    uint8_t  addr;          /* Destination address from frame                 */
    uint8_t  type;          /* Message type                                  */
    uint8_t  data[PROTOCOL_MAX_DATA];  /* Payload data (excludes Type byte)   */
    uint8_t  data_len;      /* Length of payload data                        */
} ProtocolFrame_t;

/* -------------------------------------------------------------------------- */
/*  Parser Context                                                           */
/* -------------------------------------------------------------------------- */
typedef struct {
    ProtocolState_t state;
    uint8_t  rx_buf[PROTOCOL_BUF_SIZE];
    uint16_t byte_count;
    uint8_t  exp_data_len;  /* Expected payload bytes (Len field - 1)         */
    uint8_t  frame_ready;   /* 1 = complete valid frame in 'frame'            */
    ProtocolFrame_t frame;  /* Last successfully parsed frame                 */
    uint8_t  local_addr;    /* This device's RS485 address                    */
    uint8_t  is_manager;    /* 1 = accept frames for any address              */
} ProtocolParser_t;

/* -------------------------------------------------------------------------- */
/*  Public API                                                               */
/* -------------------------------------------------------------------------- */
void     Protocol_Init(ProtocolParser_t *parser, uint8_t local_addr, uint8_t is_manager);
void     Protocol_Reset(ProtocolParser_t *parser);
uint8_t  Protocol_FeedByte(ProtocolParser_t *parser, uint8_t byte);  /* ret 1=frame ready */
uint8_t  Protocol_IsFrameReady(ProtocolParser_t *parser);
void     Protocol_ClearFrame(ProtocolParser_t *parser);

/* CRC-8 (polynomial 0x07) */
uint8_t  CRC8_Compute(const uint8_t *data, uint16_t len);

/* Frame building: returns total frame length, writes to out_buf */
uint8_t  Protocol_BuildRequest(uint8_t *out_buf, uint8_t dest_addr, uint8_t msg_type);
uint8_t  Protocol_BuildResponse(uint8_t *out_buf, uint8_t dest_addr,
                                uint8_t msg_type, const uint8_t *data, uint8_t data_len);

#endif /* __PROTOCOL_H */
