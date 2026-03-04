#include "protocol.h"
#include <string.h>

uint16_t crc16_ccitt(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

bool proto_parse_cmd(const uint8_t *buf, uint16_t len, CmdPacket *out) {
    if (len < CMD_SIZE) return false;

    // magic
    if (buf[0] != PROTO_MAGIC0 || buf[1] != PROTO_MAGIC1) return false;
    // ver
    if (buf[2] != PROTO_VER) return false;

    // CRC：对前 12 字节（CMD_SIZE - 2）计算
    uint16_t expected = crc16_ccitt(buf, CMD_SIZE - 2);
    uint16_t received;
    memcpy(&received, buf + CMD_SIZE - 2, 2);  // little-endian，RP2040 原生匹配
    if (expected != received) return false;

    memcpy(out, buf, CMD_SIZE);
    return true;
}
