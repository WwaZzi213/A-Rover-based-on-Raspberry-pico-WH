cmake_minimum_required(VERSION 3.13)

# pico_sdk_import.cmake 需在 set(PICO_BOARD) 前 include
include(pico_sdk_import.cmake)

project(pico_wifi_car C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pico_wifi_car
    src/main.cpp
    src/protocol.cpp
    src/motor_control.cpp
    src/failsafe.cpp
    src/udp_server.cpp
)

target_include_directories(pico_wifi_car PRIVATE src)

# -------------------------------------------------------
# 选用 pico_cyw43_arch_lwip_threadsafe_background 理由：
#   - lwIP 跑在后台中断驱动，主循环无需手动 poll
#   - UDP 回调在 lwIP 锁上下文触发，需用 cyw43_arch_lwip_begin/end
#     保护对共享数据的访问，安全性有保证
#   - 相比 poll 模式延迟更低、代码更简单
# -------------------------------------------------------
target_link_libraries(pico_wifi_car
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    hardware_pwm
    hardware_gpio
    hardware_sync          # spin_lock / critical section
)

# 串口输出走 USB CDC (方便调试，无需 UART 线)
pico_enable_stdio_usb(pico_wifi_car 1)
pico_enable_stdio_uart(pico_wifi_car 0)

pico_add_extra_outputs(pico_wifi_car)
