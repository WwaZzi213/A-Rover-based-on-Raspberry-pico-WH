#pragma once

// ============================================================
//  BLE
// ============================================================
#define BLE_DEVICE_NAME             "ROVER_BLE"

// 自定义 128-bit Service UUID: 12345678-1234-1234-1234-1234567890AB
// CMD  特征 (Write Without Response): ...90AC
// ACK  特征 (Notify):                 ...90AD
// UUID 以小端字节数组形式在 ble_server.cpp 中定义

// ============================================================
//  电机引脚  (Cytron DIR+PWM 单路模式)
//  Cytron MDDS20 / MDD20A 都支持 IN1=DIR, IN2=PWM
// ============================================================
#define LEFT_DIR_PIN    3
#define LEFT_PWM_PIN    2
#define RIGHT_DIR_PIN   5
#define RIGHT_PWM_PIN   0

// ============================================================
//  PWM
// ============================================================
#define PWM_FREQ_HZ         20000u   // 20 kHz，超出电机音频范围
#define PWM_CMD_MAX         1000     // 命令满量程
// PWM wrap 值由 motor_control.cpp 根据系统时钟计算

// ============================================================
//  控制循环
// ============================================================
#define CONTROL_LOOP_HZ     100      // 主循环 100 Hz
#define CONTROL_LOOP_US     (1000000u / CONTROL_LOOP_HZ)

// ============================================================
//  Slew rate：每个控制周期允许的最大 PWM 命令变化量
//  满量程 1000，100Hz 下 20/tick = 0→1000 需 50 tick = 0.5 s
// ============================================================
#define SLEW_RATE_PER_TICK  20

// ============================================================
//  失联保护
// ============================================================
#define FAILSAFE_TIMEOUT_MS 250

// ============================================================
//  电机方向
// ============================================================
#define RIGHT_MOTOR_INVERT  1   // 1=反向, 0=正向

// ============================================================
//  HC-SR04 超声波传感器
//  注意：HC-SR04 ECHO 输出 5V，需串联 1kΩ+2kΩ 分压至 3.3V
//        或使用 3.3V 供电版本，否则可能损坏 RP2040 IO
// ============================================================
// 传感器 0：正前方（固定）
#define ULTRASONIC_TRIG_PIN          9    // GPIO9  → 前方传感器 TRIG
#define ULTRASONIC_ECHO_PIN          10   // GPIO10 → 前方传感器 ECHO
// 传感器 1：左侧（安装在左舵机上）
#define ULTRASONIC_L_TRIG_PIN        11   // GPIO11 → 左侧传感器 TRIG
#define ULTRASONIC_L_ECHO_PIN        12   // GPIO12 → 左侧传感器 ECHO
// 传感器 2：右侧（安装在右舵机上）
#define ULTRASONIC_R_TRIG_PIN        13   // GPIO13 → 右侧传感器 TRIG
#define ULTRASONIC_R_ECHO_PIN        16   // GPIO16 → 右侧传感器 ECHO

#define OBSTACLE_STOP_DISTANCE_CM    20   // 手动模式避障阈值默认值（cm）

// ============================================================
//  Parallax Standard Servo（50Hz，500-2500µs = 0°-180°）
//  左舵机：GPIO14 (PWM7A)  0°=正后方  180°=正前方  90°=左侧
//  右舵机：GPIO15 (PWM7B)  0°=正前方  180°=正后方  90°=右侧
// ============================================================
#define LEFT_SERVO_PIN               14
#define RIGHT_SERVO_PIN              15

#define SERVO_MIN_US                 500u   // 0°对应脉宽
#define SERVO_MAX_US                 2500u  // 180°对应脉宽
#define SERVO_FREQ_HZ                50u

// 手动模式：两舵机均朝向正前方
#define SERVO_MANUAL_LEFT_DEG        180
#define SERVO_MANUAL_RIGHT_DEG       0
// 自动模式：两舵机均朝向侧边 90°
#define SERVO_AUTO_LEFT_DEG          90
#define SERVO_AUTO_RIGHT_DEG         90

// ============================================================
//  自动驾驶参数
// ============================================================
#define AUTO_DRIVE_SPEED             200   // 直行速度（程序中取负值 = 前进）
#define AUTO_OBSTACLE_CM             30    // 前方障碍触发转向阈值（cm）
#define AUTO_TURN_SPEED              400   // 原地坦克转速度
#define AUTO_TURN_TICKS              30    // 转向持续 tick 数（×10ms = 300ms）
#define AUTO_TURN_SIDE_TOLERANCE_CM  5     // 两侧距离差 ≤ 此值视为相等 → 默认左转
