#!/usr/bin/env python3
"""
Pico BLE Car — Windows 测试控制器
依赖：pip install bleak
用法：py test_controller_win.py [--name ROVER_BLE] [--interval 0.05]
W/S=前进/后退  A/D=左转/右转  SPACE=刹车  Q=退出
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
#  Windows 非阻塞键盘（msvcrt）
# -------------------------------------------------------
def get_key():
    """返回按下的键，无输入返回空字符串"""
    if msvcrt.kbhit():
        ch = msvcrt.getwch()
        if ch in ('\x00', '\xe0'):   # 方向键前缀
            ch2 = msvcrt.getwch()
            return {'H': 'UP', 'P': 'DOWN', 'K': 'LEFT', 'M': 'RIGHT'}.get(ch2, '')
        return ch
    return ''

# -------------------------------------------------------
#  ACK 回调
# -------------------------------------------------------
last_ack  = None
ack_times = {}

def ack_handler(_, data):
    global last_ack
    ack = parse_ack(bytes(data))
    if ack:
        send_t = ack_times.pop(ack['seq_echo'], None)
        ack['rtt_ms'] = (time.monotonic() - send_t) * 1000 if send_t else 0.0
        last_ack = ack

# -------------------------------------------------------
#  主逻辑
# -------------------------------------------------------
async def run(device_name, interval):
    global last_ack

    print(f"Scanning for '{device_name}'...")
    device = await BleakScanner.find_device_by_name(device_name, timeout=10.0)
    if not device:
        print(f"[ERROR] '{device_name}' not found. Is Pico advertising?")
        return

    print(f"Found: {device.name}  [{device.address}]")

    async with BleakClient(device) as client:
        print("Connected!")
        await client.start_notify(ACK_UUID, ack_handler)

        seq      = 0
        throttle = 0
        steer    = 0
        brake    = False
        running  = True

        print("W/S or UP/DOWN=前进/后退  A/D or LEFT/RIGHT=转向  SPACE=刹车  Q=退出\n")

        while running:
            k = get_key()
            if k in ('q', 'Q', '\x03'):
                running = False; break
            elif k in ('w', 'W', 'UP'):
                throttle = min(1000, throttle + 100)
            elif k in ('s', 'S', 'DOWN'):
                throttle = max(-1000, throttle - 100)
            elif k in ('a', 'A', 'LEFT'):
                steer = max(-1000, steer - 100)
            elif k in ('d', 'D', 'RIGHT'):
                steer = min(1000, steer + 100)
            elif k == ' ':
                brake = not brake
                throttle = 0

            left  = max(-1000, min(1000, throttle + steer))
            right = max(-1000, min(1000, throttle - steer))

            pkt = build_cmd(seq, left, right, enable=True, brake=brake)
            ack_times[seq] = time.monotonic()
            await client.write_gatt_char(CMD_UUID, pkt, response=False)
            seq += 1

            if last_ack:
                status_str = {0: 'OK', 1: 'LINK_LOST'}.get(last_ack['status'], '?')
                sys.stdout.write(
                    f"\r[seq={last_ack['seq_echo']:6d}] "
                    f"L={last_ack['app_left']:+5d} R={last_ack['app_right']:+5d}"
                    f"  RTT={last_ack['rtt_ms']:5.1f}ms  [{status_str}]"
                    f"  thr={throttle:+5d} steer={steer:+5d}   "
                )
                sys.stdout.flush()

            await asyncio.sleep(interval)

        pkt = build_cmd(seq, 0, 0, enable=False, brake=True)
        await client.write_gatt_char(CMD_UUID, pkt, response=False)
        await client.stop_notify(ACK_UUID)
        print("\nStopped.")

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