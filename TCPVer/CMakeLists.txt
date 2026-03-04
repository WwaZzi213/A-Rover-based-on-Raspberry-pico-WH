cmake_minimum_required(VERSION 3.13)

# pico_sdk_import.cmake 与工程同级（从SDK复制或软链接）
include(pico_sdk_import.cmake)

project(pico_tcp_car C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pico_tcp_car
    src/main.cpp
    src/protocol.cpp
    src/motor_control.cpp
    src/failsafe.cpp
    src/tcp_server.cpp
    src/stream_parser.cpp
)

target_include_directories(pico_tcp_car PRIVATE src)

# -------------------------------------------------------
# 与 UDP 版选用相同 arch：
#   pico_cyw43_arch_lwip_threadsafe_background
#   lwIP 跑在后台中断驱动，TCP 回调在 lwIP 锁上下文触发
# -------------------------------------------------------
target_link_libraries(pico_tcp_car
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    hardware_pwm
    hardware_gpio
    hardware_clocks
    hardware_sync          # spin_lock / critical section
)

# 串口走 USB CDC（与UDP版一致，无需 UART 线）
pico_enable_stdio_usb(pico_tcp_car 1)
pico_enable_stdio_uart(pico_tcp_car 0)

pico_add_extra_outputs(pico_tcp_car)
