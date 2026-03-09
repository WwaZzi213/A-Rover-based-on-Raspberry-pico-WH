#!/usr/bin/env python3
"""
Pico W TCP Car — Windows 测试控制器
依赖：Python 3.7+，无第三方库，Windows/Linux/Mac 均可用

用法：
    py test_controller.py --ip 192.168.x.x [--port 5001] [--interval 0.05]

键盘控制（按键立即生效，无需回车）：
    W / S       前进 / 后退（每按一次 +100/-100）
    A / D       左转 / 右转
    SPACE       刹车切换
    R           重置油门和转向为 0
    Q / Ctrl+C  退出
"""

import socket
import struct
import time
import argparse
import sys
import os
import threading

# -------------------------------------------------------
#  协议常量（与 Pico 端 protocol.h 完全一致）
# -------------------------------------------------------
MAGIC_CMD  = b'RV'
MAGIC_ACK  = b'RA'
VER        = 1
FLAG_ENABLE = 0x01
FLAG_BRAKE  = 0x02

# CmdPacket: magic(2) ver(B) flags(B) seq(I) left(h) right(h) crc16(H) = 14字节
CMD_FMT  = '<2sBBIhhH'
CMD_SIZE = struct.calcsize(CMD_FMT)   # 14

# AckPacket: magic(2) ver(B) status(B) seq_echo(I) app_left(h) app_right(h) t_us(I) = 16字节
ACK_FMT  = '<2sBBIhhI'
ACK_SIZE = struct.calcsize(ACK_FMT)  # 16

# -------------------------------------------------------
#  CRC16-CCITT
# -------------------------------------------------------
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
        crc &= 0xFFFF
    return crc

# -------------------------------------------------------
#  构建 CMD 包（14 字节）
# -------------------------------------------------------
_seq = 0
def build_cmd(left: int, right: int,
              enable: bool = True, brake: bool = False) -> bytes:
    global _seq
    _seq += 1
    flags = (FLAG_ENABLE if enable else 0) | (FLAG_BRAKE if brake else 0)
    left  = max(-1000, min(1000, left))
    right = max(-1000, min(1000, right))
    body  = struct.pack('<2sBBIhh', MAGIC_CMD, VER, flags, _seq, left, right)
    crc   = crc16_ccitt(body)
    return body + struct.pack('<H', crc)

# -------------------------------------------------------
#  解析 ACK 包（16 字节）
# -------------------------------------------------------
def parse_ack(data: bytes):
    if len(data) < ACK_SIZE:
        return None
    magic, ver, status, seq_echo, app_l, app_r, t_us = struct.unpack(
        ACK_FMT, data[:ACK_SIZE])
    if magic != MAGIC_ACK or ver != VER:
        return None
    return dict(status=status, seq_echo=seq_echo,
                app_left=app_l, app_right=app_r, t_us=t_us)

# -------------------------------------------------------
#  跨平台键盘输入（非阻塞）
# -------------------------------------------------------
if os.name == 'nt':
    # Windows
    import msvcrt
    def get_key() -> str:
        if msvcrt.kbhit():
            ch = msvcrt.getwch()
            if ch in ('\x00', '\xe0'):   # 功能键前缀
                ch2 = msvcrt.getwch()
                return {
                    'H': 'w',   # 上箭头
                    'P': 's',   # 下箭头
                    'K': 'a',   # 左箭头
                    'M': 'd',   # 右箭头
                }.get(ch2, '')
            return ch.lower()
        return ''
else:
    # Linux / Mac
    import tty, termios, select
    def get_key() -> str:
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            r, _, _ = select.select([sys.stdin], [], [], 0.0)
            if r:
                ch = sys.stdin.read(1)
                if ch == '\x1b':
                    r2, _, _ = select.select([sys.stdin], [], [], 0.05)
                    if r2:
                        ch += sys.stdin.read(2)
                        return {
                            '\x1b[A': 'w',
                            '\x1b[B': 's',
                            '\x1b[D': 'a',
                            '\x1b[C': 'd',
                        }.get(ch, '')
                return ch.lower()
            return ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

# -------------------------------------------------------
#  主逻辑
# -------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ip',       required=True,  help='Pico IP 地址')
    ap.add_argument('--port',     type=int, default=5001)
    ap.add_argument('--interval', type=float, default=0.05,
                    help='发送间隔秒数（默认 0.05 = 20Hz）')
    args = ap.parse_args()

    hz = 1.0 / args.interval
    print(f"连接到 {args.ip}:{args.port}  发送频率={hz:.0f}Hz")
    print("W/S=前后  A/D=转向  箭头键同效  SPACE=刹车  R=归零  Q=退出")
    print(f"CMD包大小={CMD_SIZE}字节  ACK包大小={ACK_SIZE}字节")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)
    try:
        sock.connect((args.ip, args.port))
    except Exception as e:
        print(f"\n[错误] 连接失败: {e}")
        print("\n排查建议：")
        print("  1. 确认 Pico 串口有输出（设备管理器找 COM 口，用 Putty/Thonny 查看）")
        print("  2. 串口应输出 '[WIFI] Connected! IP: ...' 和 '[TCP] Listening on port 5001'")
        print("  3. 确认 IP 地址与串口输出一致")
        print("  4. 确认 PC 和 Pico 在同一 Wi-Fi 网络")
        print("  5. 防火墙是否阻断了 5001 端口（Windows 防火墙可能拦截入站规则）")
        return

    sock.settimeout(0.002)
    print("已连接！\n")

    throttle  = 0
    steer     = 0
    brake     = False
    running   = True
    ack_buf   = b''
    rtt_sum   = 0.0
    rtt_count = 0
    rtt_max   = 0.0
    last_ack_str = "等待ACK..."

    t_next    = time.monotonic()
    t_last_send = time.monotonic()

    try:
        while running:
            # ── 键盘 ──
            k = get_key()
            if k in ('q', '\x03', '\x1a'):
                running = False
                break
            elif k == 'w':   throttle = min(1000, throttle + 100)
            elif k == 's':   throttle = max(-1000, throttle - 100)
            elif k == 'a':   steer    = max(-1000, steer - 100)
            elif k == 'd':   steer    = min(1000, steer + 100)
            elif k == ' ':
                brake = not brake
                if brake: throttle = 0
            elif k == 'r':
                throttle = 0; steer = 0; brake = False

            # 差速混合
            left  = max(-1000, min(1000, throttle + steer))
            right = max(-1000, min(1000, throttle - steer))

            # ── 定时发送 ──
            now = time.monotonic()
            if now >= t_next:
                t_next += args.interval
                pkt = build_cmd(left, right, enable=not brake, brake=brake)
                t_send = time.monotonic()
                try:
                    sock.sendall(pkt)
                    t_last_send = t_send
                except Exception as e:
                    print(f"\n[错误] 发送失败: {e}")
                    break

                # ── 接收 ACK ──
                try:
                    chunk = sock.recv(128)
                    if not chunk:
                        print("\n[错误] 连接已被Pico关闭")
                        break
                    ack_buf += chunk
                    while len(ack_buf) >= ACK_SIZE:
                        ack = parse_ack(ack_buf[:ACK_SIZE])
                        ack_buf = ack_buf[ACK_SIZE:]
                        if ack:
                            rtt_ms = (time.monotonic() - t_last_send) * 1000
                            rtt_sum += rtt_ms
                            rtt_count += 1
                            if rtt_ms > rtt_max: rtt_max = rtt_ms
                            avg = rtt_sum / rtt_count
                            status_str = {0:'OK', 1:'LINK_LOST'}.get(
                                ack['status'], f"?{ack['status']}")
                            last_ack_str = (
                                f"seq={ack['seq_echo']:5d} "
                                f"L={ack['app_left']:+5d} R={ack['app_right']:+5d} "
                                f"RTT={rtt_ms:5.1f}ms avg={avg:5.1f}ms "
                                f"max={rtt_max:5.1f}ms [{status_str}]"
                            )
                except (socket.timeout, BlockingIOError, OSError):
                    pass

                # 刷新显示
                brake_str = " [BRAKE]" if brake else ""
                sys.stdout.write(
                    f"\r油门={throttle:+5d} 转向={steer:+5d}{brake_str} | "
                    f"L={left:+5d} R={right:+5d} | {last_ack_str}    "
                )
                sys.stdout.flush()

            time.sleep(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        print("\n发送停止指令...")
        try:
            sock.sendall(build_cmd(0, 0, enable=False, brake=True))
            time.sleep(0.1)
        except Exception:
            pass
        sock.close()
        avg = rtt_sum / rtt_count if rtt_count else 0
        print(f"结束。发包={_seq}  收ACK={rtt_count}"
              f"  avg_rtt={avg:.1f}ms  max_rtt={rtt_max:.1f}ms")

if __name__ == '__main__':
    main()