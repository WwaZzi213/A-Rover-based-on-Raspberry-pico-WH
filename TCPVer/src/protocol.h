#pragma once
#include <stdint.h>
#include <stdbool.h>

// -------------------------------------------------------
//  CMD 包  (客户端 → Pico)  14 字节，little-endian
//  与 UDP 版协议完全相同（对照实验公平性）
// -------------------------------------------------------
#define PROTO_MAGIC0  'R'
#define PROTO_MAGIC1  'V'
#define PROTO_VER      1

#pragma pack(push, 1)
typedef struct {
    uint8_t  magic[2];   // 'R','V'
    uint8_t  ver;        // 1
    uint8_t  flags;      // bit0=enable, bit1=brake
    uint32_t seq;        // little-endian
    int16_t  left;       // -1000..1000
    int16_t  right;      // -1000..1000
    uint16_t crc16;      // CRC16-CCITT over bytes[0..11]
} CmdPacket;             // sizeof == 14
#pragma pack(pop)

#define CMD_SIZE  ((uint16_t)sizeof(CmdPacket))   // 14

#define CMD_FLAG_ENABLE  (1u << 0)
#define CMD_FLAG_BRAKE   (1u << 1)

// -------------------------------------------------------
//  ACK 包  (Pico → 客户端)  16 字节
// -------------------------------------------------------
#pragma pack(push, 1)
typedef struct {
    uint8_t  magic[2];      // 'R','A'
    uint8_t  ver;           // 1
    uint8_t  status;        // 0=ok, 1=link_lost
    uint32_t seq_echo;
    int16_t  applied_left;
    int16_t  applied_right;
    uint32_t t_us;
} AckPacket;                // sizeof == 16
#pragma pack(pop)

// -------------------------------------------------------
//  CRC16-CCITT  (poly 0x1021, init 0xFFFF)
// -------------------------------------------------------
uint16_t crc16_ccitt(const uint8_t *data, uint16_t len);

// -------------------------------------------------------
//  解析并校验 CMD 包，通过返回 true
// -------------------------------------------------------
bool proto_parse_cmd(const uint8_t *buf, uint16_t len, CmdPacket *out);
