#!/usr/bin/env python3
"""
Pico BLE Car — Windows 测试控制器
依赖：py -m pip install bleak
用法：py test_controller_win.py [--name ROVER_BLE] [--interval 0.05]
W/S/上下方向键=前进/后退  A/D/左右方向键=转向  SPACE=刹车  Q=退出
"""

import asyncio
import struct
import time
import argparse
import sys
import msvcrt
from bleak import BleakScanner, BleakClient

# -------------------------------------------------------
#  协议常量
# -------------------------------------------------------
MAGIC_CMD   = b'RV'
MAGIC_ACK   = b'RA'
VER         = 1
FLAG_ENABLE = 0x01
FLAG_BRAKE  = 0x02

CMD_FMT  = '<2sBBIhhH'
ACK_FMT  = '<2sBBIhhI'
CMD_SIZE = struct.calcsize(CMD_FMT)   # 14
ACK_SIZE = struct.calcsize(ACK_FMT)  # 16

SERVICE_UUID = "12345678-1234-1234-1234-1234567890ab"
CMD_UUID     = "12345678-1234-1234-1234-1234567890ac"
ACK_UUID     = "12345678-1234-1234-1234-1234567890ad"

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
    body  = struct.pack('<2sBBIhh', MAGIC_CMD, VER, flags, seq,
                        max(-1000, min(1000, left)),
                        max(-1000, min(1000, right)))
    return body + struct.pack('<H', crc16_ccitt(body))

def parse_ack(data):
    if len(data) < ACK_SIZE:
        return None
    magic, ver, status, seq_echo, app_l, app_r, t_us = struct.unpack(ACK_FMT, data[:ACK_SIZE])
    if magic != MAGIC_ACK or ver != VER:
        return None
    return dict(status=status, seq_echo=seq_echo,
                app_left=app_l, app_right=app_r, t_us=t_us)

# -------------------------------------------------------
#  Windows 非阻塞键盘
# -------------------------------------------------------
def get_key():
    if msvcrt.kbhit():
        ch = msvcrt.getwch()
        if ch in ('\x00', '\xe0'):
            ch2 = msvcrt.getwch()
            return {'H': 'UP', 'P': 'DOWN', 'K': 'LEFT', 'M': 'RIGHT'}.get(ch2, '')
        return ch
    return ''

# -------------------------------------------------------
#  共享状态
# -------------------------------------------------------
last_ack  = None
ack_times = {}
ack_count = 0

def ack_handler(_, data):
    global last_ack, ack_count
    ack = parse_ack(bytes(data))
    if ack:
        send_t = ack_times.pop(ack['seq_echo'], None)
        ack['rtt_ms'] = (time.monotonic() - send_t) * 1000 if send_t else -1.0
        last_ack = ack
        ack_count += 1

# -------------------------------------------------------
#  主逻辑
# -------------------------------------------------------
async def run(device_name, interval):
    global last_ack, ack_count

    print(f"Scanning for '{device_name}'...")
    device = await BleakScanner.find_device_by_name(device_name, timeout=10.0)
    if not device:
        print(f"[ERROR] '{device_name}' not found.")
        return

    print(f"Found: {device.name}  [{device.address}]")

    async with BleakClient(device) as client:
        print(f"Connected!")

        # 列出所有特征，确认 UUID 正确
        print("--- GATT Services ---")
        for svc in client.services:
            print(f"  Service: {svc.uuid}")
            for char in svc.characteristics:
                print(f"    Char: {char.uuid}  props={char.properties}")
        print("---------------------")

        # 订阅 ACK Notify
        await client.start_notify(ACK_UUID, ack_handler)
        print(f"Subscribed to ACK notify ({ACK_UUID})")

        seq      = 0
        throttle = 0
        steer    = 0
        brake    = False
        running  = True
        last_status_t = time.monotonic()

        print("W/S/方向键=油门  A/D/方向键=转向  SPACE=急停  1-9=挡位  T=弹射  Q=退出\n")

        while running:
            k = get_key()
            if k in ('q', 'Q', '\x03'):
                running = False; break
            elif k in ('w', 'W', 'UP'):
                throttle = max(-1000, throttle - 100)
            elif k in ('s', 'S', 'DOWN'):
                throttle = min(1000, throttle + 100)
            elif k in ('a', 'A', 'LEFT'):
                steer = min(1000, steer + 100)
            elif k in ('d', 'D', 'RIGHT'):
                steer = max(-1000, steer - 100)
            elif k == ' ':
                brake = not brake
                throttle = 0
                steer = 0
            elif k and k in '123456789':
                # 数字挡位：保持方向，设置速度绝对值
                speed = int(k) * 100
                throttle = -speed if throttle <= 0 else speed
                brake = False
            elif k in ('t', 'T'):
                # 弹射模式：1000 全速前进
                throttle = -1000
                steer = 0
                brake = False

            if throttle == 0:
                # 原地坦克转：左右反向
                left  = max(-1000, min(1000,  steer))
                right = max(-1000, min(1000, -steer))
            else:
                # 行进中转向：
                #   外侧 = throttle（保持不变）
                #   内侧 = throttle * (1 - |steer|/1000)，线性减速到0，不反转
                t = abs(throttle)
                s = abs(steer)
                inner = int(t * (1.0 - s / 1000.0))  # 0..t，永远不反转
                outer = t
                if throttle < 0:  # 前进
                    if steer > 0:   # 左转：左内右外
                        left  = -inner
                        right = -outer
                    elif steer < 0: # 右转：右内左外
                        left  = -outer
                        right = -inner
                    else:
                        left  = -t
                        right = -t
                else:             # 后退
                    if steer > 0:   # 左转
                        left  = inner
                        right = outer
                    elif steer < 0: # 右转
                        left  = outer
                        right = inner
                    else:
                        left  = t
                        right = t
                left  = max(-1000, min(1000, left))
                right = max(-1000, min(1000, right))

            pkt = build_cmd(seq, left, right, enable=True, brake=brake)
            ack_times[seq] = time.monotonic()
            try:
                await client.write_gatt_char(CMD_UUID, pkt, response=False)
            except Exception as e:
                print(f"\n[ERR] write failed: {e}")
                break
            seq += 1

            # 每 0.2 秒强制刷新一次状态行
            now = time.monotonic()
            if now - last_status_t >= 0.2 or last_ack:
                last_status_t = now
                if last_ack:
                    rtt = last_ack['rtt_ms']
                    rtt_str = f"{rtt:5.1f}ms" if rtt >= 0 else "  N/A "
                    status_str = {0: 'OK', 1: 'LINK_LOST'}.get(last_ack['status'], '?')
                    sys.stdout.write(
                        f"\r[seq={last_ack['seq_echo']:6d}] "
                        f"applied L={last_ack['app_left']:+5d} R={last_ack['app_right']:+5d}"
                        f"  RTT={rtt_str}  [{status_str}]"
                        f"  cmd thr={throttle:+5d} steer={steer:+5d}"
                        f"  ack_total={ack_count}   "
                    )
                else:
                    sys.stdout.write(
                        f"\r[seq={seq:6d}] waiting for ACK..."
                        f"  cmd thr={throttle:+5d} steer={steer:+5d}   "
                    )
                sys.stdout.flush()

            await asyncio.sleep(interval)

        # 发送停止命令
        pkt = build_cmd(seq, 0, 0, enable=False, brake=True)
        await client.write_gatt_char(CMD_UUID, pkt, response=False)
        await client.stop_notify(ACK_UUID)
        print(f"\nStopped. Total ACKs received: {ack_count}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--name',     default='ROVER_BLE')
    ap.add_argument('--interval', type=float, default=0.05)
    args = ap.parse_args()
    try:
        asyncio.run(run(args.name, args.interval))
    except KeyboardInterrupt:
        print("\nInterrupted.")

if __name__ == '__main__':
    main()