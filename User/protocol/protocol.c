#include "protocol.h"
#include <string.h>

/* ========================================================================== */
/*  CRC-8 Lookup Table (polynomial 0x07)                                     */
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

uint8_t CRC8_Compute(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    while (len--) {
        crc = crc8_table[crc ^ *data++];
    }
    return crc;
}

/* ========================================================================== */
/*  Protocol Parser                                                          */
/* ========================================================================== */

void Protocol_Init(ProtocolParser_t *parser, uint8_t local_addr, uint8_t is_manager)
{
    memset(parser, 0, sizeof(ProtocolParser_t));
    parser->local_addr = local_addr;
    parser->is_manager = is_manager;
    parser->state = STATE_WAIT_HEADER;
}

void Protocol_Reset(ProtocolParser_t *parser)
{
    parser->state      = STATE_WAIT_HEADER;
    parser->byte_count = 0;
    parser->frame_ready = 0;
}

void Protocol_ClearFrame(ProtocolParser_t *parser)
{
    parser->frame_ready = 0;
}

/**
  * Feed one byte into the state machine.
  * Returns 1 when a complete, valid frame has been parsed (check parser->frame).
  * Returns 0 otherwise (still receiving or invalid frame discarded).
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

        /* Address filter (collectors only process frames addressed to them) */
        if (parser->is_manager) {
            parser->state = STATE_WAIT_LEN;
        } else if (byte == parser->local_addr) {
            parser->state = STATE_WAIT_LEN;
        } else {
            /* Not for us, discard */
            parser->state = STATE_WAIT_HEADER;
        }
        break;

    case STATE_WAIT_LEN:
        if (byte > PROTOCOL_MAX_DATA) {
            /* Payload too large, discard */
            parser->state = STATE_WAIT_HEADER;
            break;
        }
        parser->rx_buf[2] = byte;
        parser->byte_count = 3;
        parser->exp_data_len = byte;       /* Len = Type + Data bytes count */
        parser->state = STATE_WAIT_TYPE;
        break;

    case STATE_WAIT_TYPE:
        parser->rx_buf[3]            = byte;
        parser->frame.type           = byte;
        parser->frame.data_len       = 0;
        parser->byte_count           = 4;
        parser->exp_data_len        -= 1;  /* Subtract Type byte */
        if (parser->exp_data_len > 0) {
            parser->state = STATE_WAIT_DATA;
        } else {
            parser->state = STATE_WAIT_CRC;
        }
        break;

    case STATE_WAIT_DATA:
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
        /* Compute CRC over header through last data byte */
        expected_crc = CRC8_Compute(parser->rx_buf, parser->byte_count - 1);
        if (byte == expected_crc) {
            parser->state = STATE_WAIT_TAIL;
        } else {
            /* CRC mismatch, discard */
            parser->state = STATE_WAIT_HEADER;
        }
        break;
    }

    case STATE_WAIT_TAIL:
        if (byte == PROTOCOL_TAIL) {
            /* Complete valid frame */
            parser->frame.addr = parser->rx_buf[1];
            parser->frame_ready = 1;
        }
        parser->state = STATE_WAIT_HEADER;
        break;

    default:
        parser->state = STATE_WAIT_HEADER;
        break;
    }

    return parser->frame_ready;
}

uint8_t Protocol_IsFrameReady(ProtocolParser_t *parser)
{
    return parser->frame_ready;
}

/* ========================================================================== */
/*  Frame Building                                                           */
/* ========================================================================== */

/**
  * Build a request frame (Type only, no extra data).
  * Returns total frame length.
  */
uint8_t Protocol_BuildRequest(uint8_t *out_buf, uint8_t dest_addr, uint8_t msg_type)
{
    return Protocol_BuildResponse(out_buf, dest_addr, msg_type, NULL, 0);
}

/**
  * Build a response/command frame with optional data payload.
  * Frame: AA + Addr + Len + Type + Data[] + CRC + 55
  * Returns total frame length.
  */
uint8_t Protocol_BuildResponse(uint8_t *out_buf, uint8_t dest_addr,
                                uint8_t msg_type, const uint8_t *data, uint8_t data_len)
{
    uint8_t idx = 0;
    uint8_t len;
    uint8_t crc;

    len = data_len + 1;  /* Type byte + data bytes */

    out_buf[idx++] = PROTOCOL_HEADER;   /* Header  */
    out_buf[idx++] = dest_addr;         /* Address */
    out_buf[idx++] = len;              /* Length  */
    out_buf[idx++] = msg_type;         /* Type    */

    if (data_len > 0 && data != NULL) {
        memcpy(&out_buf[idx], data, data_len);
        idx += data_len;
    }

    crc = CRC8_Compute(out_buf, idx);
    out_buf[idx++] = crc;
    out_buf[idx++] = PROTOCOL_TAIL;

    return idx;
}
