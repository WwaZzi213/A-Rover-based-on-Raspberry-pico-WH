#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "protocol.h"

// -------------------------------------------------------
//  TCP 流式解包器
//  TCP 是字节流，不保证一次 recv 恰好是一个完整包。
//  维护一个 ring buffer，从中搜索 magic 'R','V'，
//  凑够 CMD_SIZE(14) 字节后校验 CRC；
//  CRC 失败则丢弃 1 字节继续搜索（防粘包/错位）。
// -------------------------------------------------------

typedef struct {
    uint8_t  buf[RING_BUF_SIZE];
    uint16_t head;    // 写指针
    uint16_t tail;    // 读指针
    uint16_t count;   // 可读字节数
} RingBuf;

void     rb_init(RingBuf *rb);

// 将 data[0..len-1] 推入 ring buffer，返回实际写入字节数
uint16_t rb_push(RingBuf *rb, const uint8_t *data, uint16_t len);

// 从 tail 偏移 offset 处取一个字节（不消耗），越界返回 false
bool     rb_peek(const RingBuf *rb, uint16_t offset, uint8_t *out);

// 消耗（丢弃）n 个字节
void     rb_consume(RingBuf *rb, uint16_t n);

// -------------------------------------------------------
//  尝试从 ring buffer 中解析一个完整的 CmdPacket。
//  返回值：
//    true  → 解析成功，*pkt_out 有效，*crc_err = false
//    false + *crc_err=true  → 找到 magic+ver 但 CRC 失败，
//                              已丢弃 1 字节，调用者应重试
//    false + *crc_err=false → 数据不足，等待更多数据
// -------------------------------------------------------
bool sp_try_parse(RingBuf *rb, CmdPacket *pkt_out, bool *crc_err);
