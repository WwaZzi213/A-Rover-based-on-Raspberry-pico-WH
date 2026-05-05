# Pico BLE Car — BLE 版本

基于 Raspberry Pi Pico WH (RP2040 + CYW43439) 的 BLE 遥控小车固件。

与 UDP 版本保持完全相同的**协议格式**和**代码结构**，便于直接对比两种传输方式的性能差异。

---

## 文件结构

```
pico_ble_rover/
├── CMakeLists.txt
├── pico_sdk_import.cmake
├── src/
│   ├── config.h            # 所有配置宏（引脚、PWM、BLE 名称等）
│   ├── btstack_config.h    # BTstack 编译期配置
│   ├── rover.gatt          # GATT 数据库源文件（编译期生成 rover_gatt.h）
│   ├── main.cpp            # 入口，100Hz 控制循环
│   ├── ble_server.h/.cpp   # BTstack 初始化、GATT 回调、ACK Notify（对标 udp_server）
│   ├── protocol.h/.cpp     # CRC16-CCITT、CMD/ACK 解析（与 UDP 版完全相同）
│   ├── motor_control.h/.cpp # PWM 初始化、slew rate、deadband（与 UDP 版完全相同）
│   └── failsafe.h/.cpp     # 失联保护状态机（与 UDP 版完全相同）
└── tools/
    └── test_controller.py  # PC 测试控制器（使用 bleak，对标 UDP 版）
```

---

## 协议（与 UDP 版完全兼容）

### CMD 包（14 字节，手机 → Pico，Write Without Response）

| 字节 | 字段       | 说明                        |
|------|------------|-----------------------------|
| 0-1  | magic      | `'R'`, `'V'`                |
| 2    | ver        | `1`                         |
| 3    | flags      | bit0=enable, bit1=brake     |
| 4-7  | seq        | 序列号，uint32 LE           |
| 8-9  | left       | 左轮 -1000..1000，int16 LE  |
| 10-11| right      | 右轮 -1000..1000，int16 LE  |
| 12-13| crc16      | CRC16-CCITT over [0..11]    |

### ACK 包（16 字节，Pico → 手机，Notify）

| 字节 | 字段         | 说明                  |
|------|--------------|-----------------------|
| 0-1  | magic        | `'R'`, `'A'`          |
| 2    | ver          | `1`                   |
| 3    | status       | 0=ok, 1=link_lost     |
| 4-7  | seq_echo     | 回显 CMD 的 seq       |
| 8-9  | applied_left | 实际输出（含 slew）   |
| 10-11| applied_right|                       |
| 12-15| t_us         | Pico 时间戳 us        |

---

## BLE 服务 UUID

| 角色    | UUID                                 | 属性                  |
|---------|--------------------------------------|-----------------------|
| Service | `12345678-1234-1234-1234-1234567890AB` | —                   |
| CMD     | `12345678-1234-1234-1234-1234567890AC` | Write Without Resp  |
| ACK     | `12345678-1234-1234-1234-1234567890AD` | Notify              |

---

## 构建

```bash
export PICO_SDK_PATH=/path/to/pico-sdk
cd pico_ble_rover
mkdir build && cd build
cmake ..
make -j$(nproc)
```

烧录：按住 BOOTSEL，将 `build/pico_ble_rover.uf2` 拖入 RPI-RP2 驱动器。

---

## 硬件接线（默认，可在 config.h 修改）

| 信号        | GPIO |
|-------------|------|
| 左轮 PWM    | 2    |
| 左轮 DIR    | 3    |
| 右轮 PWM    | 0    |
| 右轮 DIR    | 5    |

右轮默认反向（`RIGHT_MOTOR_INVERT 1`），如需正向请改为 `0`。

---

## 测试控制器

```bash
pip install bleak
python tools/test_controller.py --name ROVER_BLE
# W/S=前进/后退  A/D=左转/右转  SPACE=刹车  Q=退出
```

---

## 串口日志格式（1Hz，与 UDP 版相同）

```
[STAT t=5s] state=CONNECTED    rx= 50 drop_crc= 0 seq_lost= 0 | arr_avg=20012us arr_max=21500us | apply_avg=  85us apply_max= 120us | fs_total=0
```

| 字段          | 说明                            |
|---------------|---------------------------------|
| state         | ADVERTISING / CONNECTED / LINK_LOST |
| rx            | 本秒有效 CMD 包数               |
| drop_crc      | CRC/格式错误丢弃包数            |
| seq_lost      | 根据序列号推算的丢包数          |
| arr_avg/max   | CMD 到达间隔均值/峰值（us）     |
| apply_avg/max | rx→电机输出延迟均值/峰值（us）  |
| fs_total      | 累计触发失联次数                |

---

## 与 UDP 版性能对比

| 指标             | UDP/Wi-Fi | BLE        |
|------------------|-----------|------------|
| 典型 RTT         | 1–10 ms   | 10–50 ms   |
| 吞吐量上限       | 高        | 中（BLE 5.0 ~ 2 Mbps） |
| 配网步骤         | 需连 Wi-Fi | 免配网，直连 |
| 传输距离         | ~30 m     | ~10–30 m   |
| 协议格式         | 完全相同  | 完全相同   |
| 统计日志格式     | 完全相同  | 完全相同   |
