#!/usr/bin/env python3
"""
Pico BLE Car — Windows 键盘遥控器
====================================
依赖安装：py -m pip install bleak
运行方式：py test_controller_win.py --name ROVER_BLE

键位说明：
  W / 方向键↑     前进（每按一次加速 100）
  S / 方向键↓     后退（每按一次加速 100）
  A / 方向键←     左转（每按一次转向量 +200）
  D / 方向键→     右转（每按一次转向量 +200）
  1 ~ 9           速度挡位（1=100, 2=200 ... 9=900，保持当前方向）
  T               弹射模式（直接全速前进 1000）
  空格 SPACE      急停（速度和转向全部清零）
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

# 蓝牙服务和特征的 UUID（固件里定义的，必须对应）
SERVICE_UUID = "12345678-1234-1234-1234-1234567890ab"
CMD_UUID     = "12345678-1234-1234-1234-1234567890ac"   # 发送控制命令
ACK_UUID     = "12345678-1234-1234-1234-1234567890ad"   # 接收回应数据

# 协议魔数和版本（固件用来校验数据包合法性）
MAGIC_CMD   = b'RV'
MAGIC_ACK   = b'RA'
VER         = 1

# 命令标志位
FLAG_ENABLE = 0x01   # bit0=1 表示"允许电机运转"
FLAG_BRAKE  = 0x02   # bit1=1 表示"刹车"

# 数据包格式（< 表示小端，2s=2字节字符串，B=无符号字节，I=无符号32位整数，h=有符号16位整数，H=无符号16位整数）
CMD_FMT  = '<2sBBIhhH'   # 命令包：14 字节
ACK_FMT  = '<2sBBIhhI'   # 回应包：16 字节
CMD_SIZE = struct.calcsize(CMD_FMT)
ACK_SIZE = struct.calcsize(ACK_FMT)

# ============================================================
#  CRC16 校验（数据完整性检验，固件会核对这个值）
# ============================================================
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
        crc &= 0xFFFF
    return crc

# ============================================================
#  构建命令数据包
#  seq    : 序列号（每发一包自动 +1，固件用来检测丢包）
#  left   : 左轮速度，范围 -1000~1000（负=前进，正=后退，取决于接线）
#  right  : 右轮速度，同上
#  enable : True=允许电机转动，False=停止
#  brake  : True=主动刹车
# ============================================================
def build_cmd(seq, left, right, enable=True, brake=False):
    flags = (FLAG_ENABLE if enable else 0) | (FLAG_BRAKE if brake else 0)
    # 先打包前 12 字节（不含 CRC）
    body = struct.pack('<2sBBIhh',
                       MAGIC_CMD, VER, flags, seq,
                       max(-1000, min(1000, left)),
                       max(-1000, min(1000, right)))
    # 计算 CRC 后追加到末尾
    return body + struct.pack('<H', crc16_ccitt(body))

# ============================================================
#  解析固件回传的 ACK 数据包
#  返回字典，包含：固件实际输出的左右轮速、序列号回显、时间戳
# ============================================================
def parse_ack(data):
    if len(data) < ACK_SIZE:
        return None
    magic, ver, status, seq_echo, app_l, app_r, t_us = struct.unpack(ACK_FMT, data[:ACK_SIZE])
    if magic != MAGIC_ACK or ver != VER:
        return None
    return dict(status=status, seq_echo=seq_echo,
                app_left=app_l, app_right=app_r, t_us=t_us)

# ============================================================
#  Windows 非阻塞键盘读取（不按键时立即返回空字符串）
#  msvcrt 是 Windows 内置模块，不需要安装
# ============================================================
def get_key():
    if msvcrt.kbhit():
        ch = msvcrt.getwch()
        # 方向键会发出两个字节：前缀 \x00 或 \xe0，再跟方向码
        if ch in ('\x00', '\xe0'):
            ch2 = msvcrt.getwch()
            return {'H': 'UP', 'P': 'DOWN', 'K': 'LEFT', 'M': 'RIGHT'}.get(ch2, '')
        return ch
    return ''   # 没有按键

# ============================================================
#  全局状态：ACK 回调会从 BLE 中断里更新这两个变量
# ============================================================
last_ack  = None    # 最近收到的 ACK 解析结果
ack_times = {}      # 记录每个 seq 的发送时间，用来计算 RTT（往返延迟）
ack_count = 0       # 累计收到的 ACK 数量

def ack_handler(_, data):
    """BLE Notify 回调：每次固件发来 ACK 时自动触发"""
    global last_ack, ack_count
    ack = parse_ack(bytes(data))
    if ack:
        send_t = ack_times.pop(ack['seq_echo'], None)
        # RTT = 当前时间 - 发送时间（毫秒）
        ack['rtt_ms'] = (time.monotonic() - send_t) * 1000 if send_t else -1.0
        last_ack = ack
        ack_count += 1

# ============================================================
#  混合计算左右轮速
#  throttle : 油门，负=前进，正=后退，0=静止
#  steer    : 转向，正=左转，负=右转
#
#  静止时（throttle==0）：坦克转，左右电机反向旋转
#  行进时：外侧保持原速，内侧按转向比例线性减速至 0，不反转
#    例：throttle=-500（前进），steer=+600（左转）
#      → 右轮（外侧）= 500，左轮（内侧）= 500*(1-600/1000) = 200
# ============================================================
def mix(throttle, steer):
    if throttle == 0:
        # 原地坦克转
        left  = max(-1000, min(1000,  steer))
        right = max(-1000, min(1000, -steer))
    else:
        t = abs(throttle)
        s = abs(steer)
        # 内侧电机速度 = 原速 × (1 - 转向量/满量程)，最小为 0
        inner = int(t * (1.0 - s / 1000.0))
        outer = t

        if throttle < 0:        # 前进方向
            if steer > 0:       # 左转：左轮内侧，右轮外侧
                left, right = -inner, -outer
            elif steer < 0:     # 右转：右轮内侧，左轮外侧
                left, right = -outer, -inner
            else:               # 直行
                left = right = -t
        else:                   # 后退方向
            if steer > 0:       # 左转
                left, right = inner, outer
            elif steer < 0:     # 右转
                left, right = outer, inner
            else:               # 直行
                left = right = t

        left  = max(-1000, min(1000, left))
        right = max(-1000, min(1000, right))
    return left, right

# ============================================================
#  主程序：扫描 BLE 设备、连接、循环发送命令
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

        # 订阅 ACK Notify，之后固件每发一包就会触发 ack_handler
        await client.start_notify(ACK_UUID, ack_handler)
        print(f"已订阅 ACK 通知\n")

        # ---------- 控制状态 ----------
        seq      = 0        # 数据包序列号，自动递增
        throttle = 0        # 当前油门值（负=前进，正=后退，0=静止）
        steer    = 0        # 当前转向值（正=左，负=右，0=直行）
        brake    = False    # 是否处于刹车状态
        running  = True
        last_display_t = time.monotonic()

        print("W/S/方向键=油门  A/D/方向键=转向  SPACE=急停  1-9=挡位  T=弹射  Q=退出\n")

        while running:
            # ---- 读取键盘输入 ----
            k = get_key()

            if k in ('q', 'Q', '\x03'):         # Q 或 Ctrl+C：退出
                running = False
                break

            elif k in ('w', 'W', 'UP'):          # 前进：throttle 向负方向步进
                throttle = max(-1000, throttle - THROTTLE_STEP)
                brake = False

            elif k in ('s', 'S', 'DOWN'):        # 后退：throttle 向正方向步进
                throttle = min(1000, throttle + THROTTLE_STEP)
                brake = False

            elif k in ('a', 'A', 'LEFT'):        # 左转
                steer = min(1000, steer + STEER_STEP)

            elif k in ('d', 'D', 'RIGHT'):       # 右转
                steer = max(-1000, steer - STEER_STEP)

            elif k == ' ':                       # 急停：清零所有运动状态
                brake    = not brake
                throttle = 0
                steer    = 0

            elif k and k in '123456789':         # 数字挡位：保持方向，设置速度
                speed = int(k) * 100
                # 保持当前前进/后退方向，只改变速度绝对值
                throttle = -speed if throttle <= 0 else speed
                brake = False

            elif k in ('t', 'T'):               # 弹射：全速前进
                throttle = -1000
                steer    = 0
                brake    = False

            # ---- 计算左右轮速 ----
            left, right = mix(throttle, steer)

            # ---- 发送命令包 ----
            pkt = build_cmd(seq, left, right, enable=not brake, brake=brake)
            ack_times[seq] = time.monotonic()
            try:
                await client.write_gatt_char(CMD_UUID, pkt, response=False)
            except Exception as e:
                print(f"\n[错误] 发送失败：{e}")
                break
            seq += 1

            # ---- 刷新显示（每 0.1 秒或收到新 ACK 时更新）----
            now = time.monotonic()
            if now - last_display_t >= 0.1:
                last_display_t = now
                if last_ack:
                    rtt = last_ack['rtt_ms']
                    rtt_str   = f"{rtt:5.1f}ms" if rtt >= 0 else "  N/A "
                    status_str = {0: 'OK', 1: 'LINK_LOST'}.get(last_ack['status'], '?')
                    sys.stdout.write(
                        f"\r[seq={last_ack['seq_echo']:6d}] "
                        f"实际输出 L={last_ack['app_left']:+5d} R={last_ack['app_right']:+5d}"
                        f"  RTT={rtt_str}  [{status_str}]"
                        f"  油门={throttle:+5d} 转向={steer:+5d}"
                        f"  ACK总数={ack_count}   "
                    )
                else:
                    sys.stdout.write(
                        f"\r[seq={seq:6d}] 等待 ACK..."
                        f"  油门={throttle:+5d} 转向={steer:+5d}   "
                    )
                sys.stdout.flush()

            await asyncio.sleep(interval)

        # ---- 退出前发送停止命令 ----
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