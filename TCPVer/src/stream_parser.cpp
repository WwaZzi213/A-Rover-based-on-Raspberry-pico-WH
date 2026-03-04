#include "stream_parser.h"
#include <string.h>

void rb_init(RingBuf *rb) {
    memset(rb, 0, sizeof(*rb));
}

uint16_t rb_push(RingBuf *rb, const uint8_t *data, uint16_t len) {
    uint16_t space = RING_BUF_SIZE - rb->count;
    if (len > space) len = space;
    for (uint16_t i = 0; i < len; i++) {
        rb->buf[rb->head] = data[i];
        rb->head = (rb->head + 1) % RING_BUF_SIZE;
    }
    rb->count += len;
    return len;
}

bool rb_peek(const RingBuf *rb, uint16_t offset, uint8_t *out) {
    if (offset >= rb->count) return false;
    *out = rb->buf[(rb->tail + offset) % RING_BUF_SIZE];
    return true;
}

void rb_consume(RingBuf *rb, uint16_t n) {
    if (n > rb->count) n = rb->count;
    rb->tail  = (rb->tail + n) % RING_BUF_SIZE;
    rb->count -= n;
}

bool sp_try_parse(RingBuf *rb, CmdPacket *pkt_out, bool *crc_err) {
    *crc_err = false;

    // Step 1: 向前扫描，定位 magic 'R','V'
    while (rb->count >= 2) {
        uint8_t b0, b1;
        rb_peek(rb, 0, &b0);
        rb_peek(rb, 1, &b1);
        if (b0 == PROTO_MAGIC0 && b1 == PROTO_MAGIC1) break;
        rb_consume(rb, 1);   // 不是 magic，丢弃一字节
    }

    // Step 2: 数据是否足够一个完整包
    if (rb->count < CMD_SIZE) return false;

    // Step 3: 线性化到临时缓冲区
    uint8_t tmp[CMD_SIZE];
    for (uint16_t i = 0; i < CMD_SIZE; i++) {
        rb_peek(rb, i, &tmp[i]);
    }

    // Step 4: 校验（magic+ver+crc）
    if (proto_parse_cmd(tmp, CMD_SIZE, pkt_out)) {
        rb_consume(rb, CMD_SIZE);
        return true;
    }

    // CRC 失败：丢弃 1 字节，让调用者重试（防止错位锁死）
    rb_consume(rb, 1);
    *crc_err = true;
    return false;
}
