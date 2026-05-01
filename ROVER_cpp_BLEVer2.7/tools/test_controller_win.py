#!/usr/bin/env python3
"""
Pico BLE Car — Windows 键盘遥控器
====================================
依赖安装：py -m pip install bleak
运行方式：py test_controller_win.py --name ROVER_BLE

键位说明：
  W               前进（每按一次加速 100）
  S               后退（每按一次加速 100）
  A / 方向键←     左转（每按一次转向量 +200）
  D / 方向键→     右转（每按一次转向量 +200）
  方向键↑         避障阈值 +5cm（最大 200cm）
  方向键↓         避障阈值 -5cm（最小 5cm）
  1 ~ 9           速度挡位（1=100, 2=200 ... 9=900，保持当前方向）
  T               弹射模式（直接全速前进 1000）
  空格 SPACE      急停（速度和转向全部清零）
  K               切换手动/自动模式
  Q               退出程序
"""

import asyncio
import struct
import time
import argparse
import sys
import msvcrt
from bleak import BleakScanner, BleakClient

# ============================================================
#  ★ 参数区 ★ ^_^
# ============================================================

# 油门每次按键的步进量（越大加速越猛，范围 1~1000）
THROTTLE_STEP = 100

# 转向每次按键的步进量（越大转向越急，范围 1~1000）
STEER_STEP = 200

# 发送命令的频率（秒），0.05 = 每秒 20 次
SEND_INTERVAL = 0.05

# ============================================================
#  BLE / 协议常量（与固件 protocol.h 完全一致，不要修改）
# ============================================================

SERVICE_UUID = "12345678-1234-1234-1234-1234567890ab"
CMD_UUID     = "12345678-1234-1234-1234-1234567890ac"   # 发送控制命令
ACK_UUID     = "12345678-1234-1234-1234-1234567890ad"   # 接收回应数据
CFG_UUID     = "12345678-1234-1234-1234-1234567890ae"   # 运行时配置（阈值/模式）

# CONFIG 命令类型
CFG_SET_THRESHOLD = 0x01   # [1]=避障阈值 cm
CFG_SET_MODE      = 0x02   # [1]=0:手动  1:自动

MAGIC_CMD   = b'RV'
MAGIC_ACK   = b'RA'
VER         = 1

FLAG_ENABLE = 0x01
FLAG_BRAKE  = 0x02

CMD_FMT  = '<2sBBIhhH'   # 命令包：14 字节
ACK_FMT  = '<2sBBIhhI'   # 回应包：16 字节
CMD_SIZE = struct.calcsize(CMD_FMT)
ACK_SIZE = struct.calcsize(ACK_FMT)

# ============================================================
#  CRC16-CCITT
# ============================================================
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
    body = struct.pack('<2sBBIhh',
                       MAGIC_CMD, VER, flags, seq,
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

# ============================================================
#  Windows 非阻塞键盘读取（msvcrt）
#  方向键发两字节：前缀 \x00/\xe0 + 方向码
# ============================================================
def get_key():
    if msvcrt.kbhit():
        ch = msvcrt.getwch()
        if ch in ('\x00', '\xe0'):
            ch2 = msvcrt.getwch()
            return {'H': 'UP', 'P': 'DOWN', 'K': 'LEFT', 'M': 'RIGHT'}.get(ch2, '')
        return ch
    return ''

# ============================================================
#  全局 ACK 状态
# ============================================================
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

# ============================================================
#  差速混合（静止=坦克转，行进=内侧减速）
# ============================================================
def mix(throttle, steer):
    if throttle == 0:
        left  = max(-1000, min(1000,  steer))
        right = max(-1000, min(1000, -steer))
    else:
        t = abs(throttle)
        s = abs(steer)
        inner = int(t * (1.0 - s / 1000.0))
        outer = t
        if throttle < 0:
            if steer > 0:   left, right = -inner, -outer
            elif steer < 0: left, right = -outer, -inner
            else:           left = right = -t
        else:
            if steer > 0:   left, right = inner, outer
            elif steer < 0: left, right = outer, inner
            else:           left = right = t
        left  = max(-1000, min(1000, left))
        right = max(-1000, min(1000, right))
    return left, right

# ============================================================
#  主程序
# ============================================================
async def run(device_name, interval):
    global last_ack, ack_count

    print(f"正在扫描 '{device_name}'，请确保 Pico 已上电...")
    device = await BleakScanner.find_device_by_name(device_name, timeout=10.0)
    if not device:
        print(f"[错误] 找不到 '{device_name}'，请检查 Pico 是否在广播。")
        return

    print(f"找到设备：{device.name}  [{device.address}]")

    async with BleakClient(device) as client:
        print("已连接！")
        await client.start_notify(ACK_UUID, ack_handler)
        print("已订阅 ACK 通知\n")

        seq        = 0
        throttle   = 0
        steer      = 0
        brake      = False
        drive_mode = 0        # 0=手动，1=自动
        obs_thresh = 20       # 本地显示用的避障阈值（与固件同步）
        running    = True
        last_display_t = time.monotonic()

        print("W/S=油门  A/D/←→=转向  ↑↓=调避障阈值  K=切模式  SPACE=急停  1-9=挡位  T=弹射  Q=退出\n")

        while running:
            k = get_key()

            if k in ('q', 'Q', '\x03'):
                running = False
                break

            elif k in ('w', 'W'):
                throttle = max(-1000, throttle - THROTTLE_STEP)
                brake = False

            elif k in ('s', 'S'):
                throttle = min(1000, throttle + THROTTLE_STEP)
                brake = False

            elif k in ('a', 'A', 'LEFT'):
                steer = min(1000, steer + STEER_STEP)

            elif k in ('d', 'D', 'RIGHT'):
                steer = max(-1000, steer - STEER_STEP)

            elif k == 'UP':
                obs_thresh = min(200, obs_thresh + 5)
                await client.write_gatt_char(
                    CFG_UUID, bytes([CFG_SET_THRESHOLD, obs_thresh]), response=False)
                print(f"\n[CFG] 避障阈值 -> {obs_thresh}cm")

            elif k == 'DOWN':
                obs_thresh = max(5, obs_thresh - 5)
                await client.write_gatt_char(
                    CFG_UUID, bytes([CFG_SET_THRESHOLD, obs_thresh]), response=False)
                print(f"\n[CFG] 避障阈值 -> {obs_thresh}cm")

            elif k in ('k', 'K'):
                drive_mode = 1 - drive_mode
                await client.write_gatt_char(
                    CFG_UUID, bytes([CFG_SET_MODE, drive_mode]), response=False)
                print(f"\n[CFG] 驾驶模式 -> {'AUTO' if drive_mode else 'MANUAL'}")

            elif k == ' ':
                brake    = not brake
                throttle = 0
                steer    = 0

            elif k and k in '123456789':
                speed = int(k) * 100
                throttle = -speed if throttle <= 0 else speed
                brake = False

            elif k in ('t', 'T'):
                throttle = -1000
                steer    = 0
                brake    = False

            left, right = mix(throttle, steer)

            pkt = build_cmd(seq, left, right, enable=not brake, brake=brake)
            ack_times[seq] = time.monotonic()
            try:
                await client.write_gatt_char(CMD_UUID, pkt, response=False)
            except Exception as e:
                print(f"\n[错误] 发送失败：{e}")
                break
            seq += 1

            now = time.monotonic()
            if now - last_display_t >= 0.1:
                last_display_t = now
                mode_str = 'AUTO  ' if drive_mode else 'MANUAL'
                if last_ack:
                    rtt = last_ack['rtt_ms']
                    rtt_str    = f"{rtt:5.1f}ms" if rtt >= 0 else "  N/A "
                    status_str = {0: 'OK', 1: 'LINK_LOST'}.get(last_ack['status'], '?')
                    sys.stdout.write(
                        f"\r[{mode_str}] thr={obs_thresh:3d}cm "
                        f"[seq={last_ack['seq_echo']:6d}] "
                        f"L={last_ack['app_left']:+5d} R={last_ack['app_right']:+5d}"
                        f"  RTT={rtt_str}  [{status_str}]"
                        f"  油门={throttle:+5d} 转向={steer:+5d}   "
                    )
                else:
                    sys.stdout.write(
                        f"\r[{mode_str}] thr={obs_thresh:3d}cm "
                        f"[seq={seq:6d}] 等待 ACK..."
                        f"  油门={throttle:+5d} 转向={steer:+5d}   "
                    )
                sys.stdout.flush()

            await asyncio.sleep(interval)

        pkt = build_cmd(seq, 0, 0, enable=False, brake=True)
        await client.write_gatt_char(CMD_UUID, pkt, response=False)
        await client.stop_notify(ACK_UUID)
        print(f"\n已停止。累计收到 ACK：{ack_count} 个")

# ============================================================
#  命令行参数入口
# ============================================================
def main():
    ap = argparse.ArgumentParser(description='Pico BLE Car Windows 控制器')
    ap.add_argument('--name',     default='ROVER_BLE',
                    help='BLE 设备名称（默认 ROVER_BLE）')
    ap.add_argument('--interval', type=float, default=SEND_INTERVAL,
                    help=f'发包间隔秒数（默认 {SEND_INTERVAL}）')
    args = ap.parse_args()
    try:
        asyncio.run(run(args.name, args.interval))
    except KeyboardInterrupt:
        print("\n已中断。")

if __name__ == '__main__':
    main()
