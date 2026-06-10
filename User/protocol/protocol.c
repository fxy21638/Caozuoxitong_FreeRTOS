#include "protocol.h"
#include <string.h>

/* ========================================================================== */
/*  CRC-8 查找表 (多项式 0x07 = x^8 + x^2 + x + 1)                            */
/* ========================================================================== */
static const uint8_t crc8_table[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

/**
  * @brief  计算 CRC-8 校验值 (多项式 0x07)
  * @param  data : 待校验数据指针
  * @param  len  : 数据长度 (字节)
  * @retval CRC-8 校验值
  */
uint8_t CRC8_Compute(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    while (len--) {
        crc = crc8_table[crc ^ *data++];
    }
    return crc;
}

/* ========================================================================== */
/*  协议解析器                                                                 */
/* ========================================================================== */

/**
  * @brief  初始化协议解析器
  * @param  parser     : 解析器指针
  * @param  local_addr : 本机 RS485 地址
  * @param  is_manager : 1=管理端 (接收所有地址), 0=采集前端 (仅接收本机地址)
  */
void Protocol_Init(ProtocolParser_t *parser, uint8_t local_addr, uint8_t is_manager)
{
    memset(parser, 0, sizeof(ProtocolParser_t));
    parser->local_addr = local_addr;
    parser->is_manager = is_manager;
    parser->state = STATE_WAIT_HEADER;
}

/**
  * @brief  重置解析器到初始状态（帧超时时调用）
  */
void Protocol_Reset(ProtocolParser_t *parser)
{
    parser->state       = STATE_WAIT_HEADER;
    parser->byte_count  = 0;
    parser->frame_ready = 0;
}

/**
  * @brief  清除帧就绪标志（任务处理完帧后调用）
  */
void Protocol_ClearFrame(ProtocolParser_t *parser)
{
    parser->frame_ready = 0;
}

/**
  * @brief  喂入一个字节到状态机
  * @retval 1 = 完整合法帧已解析 (通过 parser->frame 访问)
  *         0 = 仍在接收或帧已被丢弃
  *
  *  状态机流程:
  *    WAIT_HEADER → 收到 0xAA → WAIT_ADDR
  *    WAIT_ADDR   → 地址过滤    → WAIT_LEN
  *    WAIT_LEN    → 长度校验    → WAIT_TYPE
  *    WAIT_TYPE   → 记录类型    → WAIT_DATA 或 WAIT_CRC
  *    WAIT_DATA   → 收 N 字节   → WAIT_CRC
  *    WAIT_CRC    → CRC 校验    → WAIT_TAIL 或丢弃复位
  *    WAIT_TAIL   → 确认 0x55   → 帧就绪 → 回到 WAIT_HEADER
  *
  *  校验范围: 从帧头 (0xAA) 到 CRC 前一字节
  *  地址过滤: 管理端接收所有地址，前端仅处理匹配自身地址的帧
  */
uint8_t Protocol_FeedByte(ProtocolParser_t *parser, uint8_t byte)
{
    switch (parser->state) {

    case STATE_WAIT_HEADER:
        if (byte == PROTOCOL_HEADER) {
            parser->rx_buf[0] = byte;
            parser->byte_count = 1;
            parser->state = STATE_WAIT_ADDR;
        }
        break;

    case STATE_WAIT_ADDR:
        parser->rx_buf[1] = byte;
        parser->byte_count = 2;

        /* 地址过滤：管理端接收所有地址；前端仅处理本机地址 */
        if (parser->is_manager) {
            parser->state = STATE_WAIT_LEN;
        } else if (byte == parser->local_addr) {
            parser->state = STATE_WAIT_LEN;
        } else {
            parser->state = STATE_WAIT_HEADER;   /* 非本机地址，丢弃 */
        }
        break;

    case STATE_WAIT_LEN:
        /* 长度合法性检查：Len = Type + Data 字节数 */
        if (byte > PROTOCOL_MAX_DATA) {
            parser->state = STATE_WAIT_HEADER;
            break;
        }
        parser->rx_buf[2] = byte;
        parser->byte_count = 3;
        parser->exp_data_len = byte;          /* Len 包含 Type 字节 */
        parser->state = STATE_WAIT_TYPE;
        break;

    case STATE_WAIT_TYPE:
        parser->rx_buf[3]        = byte;
        parser->frame.type       = byte;
        parser->frame.data_len   = 0;
        parser->byte_count       = 4;
        parser->exp_data_len   -= 1;          /* 减去 Type 字节 */
        if (parser->exp_data_len > 0) {
            parser->state = STATE_WAIT_DATA;
        } else {
            parser->state = STATE_WAIT_CRC;   /* 仅 Type 无数据，直接校验 CRC */
        }
        break;

    case STATE_WAIT_DATA:
        /* 逐字节接收数据载荷 */
        parser->rx_buf[parser->byte_count] = byte;
        parser->frame.data[parser->frame.data_len++] = byte;
        parser->byte_count++;
        parser->exp_data_len--;
        if (parser->exp_data_len == 0) {
            parser->state = STATE_WAIT_CRC;
        }
        break;

    case STATE_WAIT_CRC: {
        uint8_t expected_crc;
        parser->rx_buf[parser->byte_count] = byte;
        parser->byte_count++;
        /* 计算 CRC 范围: rx_buf[0] ~ rx_buf[byte_count-2] (不含 CRC 字节) */
        expected_crc = CRC8_Compute(parser->rx_buf, parser->byte_count - 1);
        if (byte == expected_crc) {
            parser->state = STATE_WAIT_TAIL;
        } else {
            parser->state = STATE_WAIT_HEADER;   /* CRC 校验失败，丢弃整帧 */
        }
        break;
    }

    case STATE_WAIT_TAIL:
        if (byte == PROTOCOL_TAIL) {
            parser->frame.addr = parser->rx_buf[1];
            parser->frame_ready = 1;             /* 完整合法帧就绪 */
        }
        parser->state = STATE_WAIT_HEADER;
        break;

    default:
        parser->state = STATE_WAIT_HEADER;
        break;
    }

    return parser->frame_ready;
}

/**
  * @brief  查询是否有完整帧就绪
  */
uint8_t Protocol_IsFrameReady(ProtocolParser_t *parser)
{
    return parser->frame_ready;
}

/* ========================================================================== */
/*  帧组包                                                                     */
/* ========================================================================== */

/**
  * @brief  构建请求帧（仅 Type，无额外数据）
  * @param  out_buf   : 输出缓冲区
  * @param  dest_addr : 目标地址
  * @param  msg_type  : 消息类型
  * @retval 完整帧长度
  */
uint8_t Protocol_BuildRequest(uint8_t *out_buf, uint8_t dest_addr, uint8_t msg_type)
{
    return Protocol_BuildResponse(out_buf, dest_addr, msg_type, NULL, 0);
}

/**
  * @brief  构建应答/控制帧（Type + Data）
  *         帧结构: AA + Addr + Len + Type + Data[] + CRC + 55
  * @param  out_buf   : 输出缓冲区
  * @param  dest_addr : 目标地址
  * @param  msg_type  : 消息类型
  * @param  data      : 数据载荷指针 (可为 NULL)
  * @param  data_len  : 数据载荷长度
  * @retval 完整帧长度
  */
uint8_t Protocol_BuildResponse(uint8_t *out_buf, uint8_t dest_addr,
                                uint8_t msg_type, const uint8_t *data, uint8_t data_len)
{
    uint8_t idx = 0;
    uint8_t len;
    uint8_t crc;

    len = data_len + 1;                /* Len = Type 字节 + Data 字节 */

    out_buf[idx++] = PROTOCOL_HEADER;  /* 帧头   */
    out_buf[idx++] = dest_addr;        /* 地址   */
    out_buf[idx++] = len;              /* 长度   */
    out_buf[idx++] = msg_type;         /* 类型   */

    if (data_len > 0 && data != NULL) {
        memcpy(&out_buf[idx], data, data_len);
        idx += data_len;
    }

    crc = CRC8_Compute(out_buf, idx);  /* CRC 覆盖帧头~数据末 */
    out_buf[idx++] = crc;
    out_buf[idx++] = PROTOCOL_TAIL;    /* 帧尾   */

    return idx;
}
