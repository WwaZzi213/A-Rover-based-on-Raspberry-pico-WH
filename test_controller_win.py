#!/usr/bin/env python3
"""
Pico Wi-Fi Car - Windows 测试控制器
用法: py test_controller_win.py --ip 192.168.5.29
"""

import socket
import struct
import time
import argparse
import msvcrt
import sys

# -------------------------------------------------------
#  协议常量
# -------------------------------------------------------
MAGIC_CMD = b'RV'
MAGIC_ACK = b'RA'
VER = 1
FLAG_ENABLE = 0x01
FLAG_BRAKE  = 0x02
CMD_FMT = '<2sBBIhh H'
ACK_FMT = '<2sBBIhhI'
CMD_SIZE = struct.calcsize(CMD_FMT)
ACK_SIZE = struct.calcsize(ACK_FMT)

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
        crc &= 0xFFFF
    return crc

def build_cmd(seq, left, right, enable=True, brake=False):
    flags = (FLAG_ENABLE if enable else 0) | (FLAG_BRAKE if brake else 0)
    left  = max(-1000, min(1000, left))
    right = max(-1000, min(1000, right))
    body = struct.pack('<2sBBIhh', MAGIC_CMD, VER, flags, seq, left, right)
    crc  = crc16_ccitt(body)
    return body + struct.pack('<H', crc)

def parse_ack(data):
    if len(data) < ACK_SIZE:
        return None
    magic, ver, status, seq_echo, app_l, app_r, t_us = struct.unpack(ACK_FMT, data[:ACK_SIZE])
    if magic != MAGIC_ACK or ver != VER:
        return None
    return dict(status=status, seq_echo=seq_echo,
                app_left=app_l, app_right=app_r, t_us=t_us)

def get_key():
    """非阻塞读键，返回字符或空串，Windows msvcrt 版"""
    if msvcrt.kbhit():
        ch = msvcrt.getwch()
        # 方向键是两字节：先 \x00 或 \xe0，再功能码
        if ch in ('\x00', '\xe0'):
            ch2 = msvcrt.getwch()
            return {'H': 'UP', 'P': 'DOWN', 'K': 'LEFT', 'M': 'RIGHT'}.get(ch2, '')
        return ch
    return ''

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ip',       required=True,       help='Pico IP 地址')
    ap.add_argument('--port',     type=int, default=5005)
    ap.add_argument('--interval', type=float, default=0.05, help='发包间隔(秒)')
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.04)
    pico_addr = (args.ip, args.port)

    seq      = 0
    throttle = 0    # -1000..1000
    steer    = 0    # -1000..1000
    brake    = False
    running  = True

    print(f"=== Pico Car Controller (Windows) ===")
    print(f"目标: {args.ip}:{args.port}")
    print(f"W/↑=前进  S/↓=后退  A/←=左转  D/→=右转")
    print(f"空格=刹车切换  R=复位  Q=退出")
    print(f"{'─'*55}")

    last_send = time.monotonic()

    while running:
        # 读键（可连续读多个，避免积压）
        k = get_key()
        if k:
            ku = k.upper()
            if ku in ('Q', '\x03', '\x1b'):
                running = False
                break
            elif ku in ('W', 'UP'):
                throttle = min(1000, throttle + 100)
            elif ku in ('S', 'DOWN'):
                throttle = max(-1000, throttle - 100)
            elif ku in ('A', 'LEFT'):
                steer = max(-1000, steer - 100)
            elif ku in ('D', 'RIGHT'):
                steer = min(1000, steer + 100)
            elif k == ' ':
                brake = not brake
                if brake:
                    throttle = 0
                    steer    = 0
            elif ku == 'R':
                throttle = 0
                steer    = 0
                brake    = False

        # 按间隔发包
        now = time.monotonic()
        if now - last_send >= args.interval:
            last_send = now

            left  = max(-1000, min(1000, throttle + steer))
            right = max(-1000, min(1000, throttle - steer))

            pkt = build_cmd(seq, left, right, enable=True, brake=brake)
            t0  = time.monotonic()
            sock.sendto(pkt, pico_addr)
            seq += 1

            # 接收 ACK
            rtt_str = "---"
            ack_str = "等待..."
            try:
                data, _ = sock.recvfrom(64)
                rtt_ms  = (time.monotonic() - t0) * 1000
                ack = parse_ack(data)
                if ack:
                    status = {0: 'OK', 1: 'LINK_LOST'}.get(ack['status'], f"?{ack['status']}")
                    rtt_str = f"{rtt_ms:.1f}ms"
                    ack_str = f"seq={ack['seq_echo']:5d} L={ack['app_left']:+5d} R={ack['app_right']:+5d} {status}"
            except socket.timeout:
                ack_str = "无ACK(超时)"

            brake_str = " [BRAKE]" if brake else ""
            sys.stdout.write(
                f"\r发送: L={left:+5d} R={right:+5d}{brake_str:<8}"
                f"  RTT={rtt_str:<8}  ACK: {ack_str}          "
            )
            sys.stdout.flush()

        time.sleep(0.005)  # 5ms 轮询，降低 CPU 占用

    # 退出前发停止命令
    pkt = build_cmd(seq, 0, 0, enable=False, brake=True)
    sock.sendto(pkt, pico_addr)
    sock.close()
    print("\n已停止。")

if __name__ == '__main__':
    main()