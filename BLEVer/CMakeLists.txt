cmake_minimum_required(VERSION 3.13)

# pico_sdk_import.cmake 需在 set(PICO_BOARD) 前 include
include(pico_sdk_import.cmake)

project(pico_ble_rover C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# -------------------------------------------------------
#  从 rover.gatt 生成 rover_gatt.h
#  使用 execute_process 在 cmake 配置阶段直接执行，
#  避免 Windows NMake 对 add_custom_command 的依赖追踪问题
# -------------------------------------------------------
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(GATT_FILE    "${CMAKE_CURRENT_SOURCE_DIR}/src/rover.gatt")
set(GATT_HEADER  "${CMAKE_CURRENT_BINARY_DIR}/rover_gatt.h")
set(COMPILE_GATT "${PICO_SDK_PATH}/lib/btstack/tool/compile_gatt.py")

message(STATUS "Compiling GATT database: rover.gatt -> rover_gatt.h")
execute_process(
    COMMAND "${Python3_EXECUTABLE}" "${COMPILE_GATT}" "${GATT_FILE}" "${GATT_HEADER}"
    RESULT_VARIABLE GATT_RESULT
    OUTPUT_VARIABLE GATT_OUTPUT
    ERROR_VARIABLE  GATT_ERROR
)
if(NOT GATT_RESULT EQUAL 0)
    message(FATAL_ERROR "compile_gatt.py failed (ret=${GATT_RESULT}):\n${GATT_ERROR}")
endif()
message(STATUS "rover_gatt.h -> ${GATT_HEADER}")

# -------------------------------------------------------
#  可执行文件
# -------------------------------------------------------
add_executable(pico_ble_rover
    src/main.cpp
    src/ble_server.cpp
    src/protocol.cpp
    src/motor_control.cpp
    src/failsafe.cpp
)

target_include_directories(pico_ble_rover PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_BINARY_DIR}          # rover_gatt.h 生成到 build 目录
    ${PICO_SDK_PATH}/lib/btstack/src
    ${PICO_SDK_PATH}/lib/btstack/platform/embedded
)

# -------------------------------------------------------
#  链接库
#  pico_cyw43_arch_none：主循环手动 poll，适合 BLE 单任务场景
# -------------------------------------------------------
target_link_libraries(pico_ble_rover
    pico_stdlib
    pico_btstack_ble
    pico_btstack_cyw43
    pico_cyw43_arch_none
    pico_async_context_poll
    hardware_pwm
    hardware_gpio
    hardware_sync          # spin_lock / critical section
)

# 串口输出走 USB CDC (方便调试，无需 UART 线)
pico_enable_stdio_usb(pico_ble_rover 1)
pico_enable_stdio_uart(pico_ble_rover 0)

# 通过命令行强制定义关键宏，确保无论哪个 btstack_config.h 被读到都生效
target_compile_definitions(pico_ble_rover PRIVATE
    ENABLE_LE_PERIPHERAL=1
    ENABLE_GATT_SERVER=1
    MAX_ATT_DB_SIZE=512
    MAX_NR_LE_DEVICE_DB_ENTRIES=4
    NVM_NUM_DEVICE_DB_ENTRIES=4
    ENABLE_SM_MINIMAL=1
    ENABLE_PRINTF_HEXDUMP=1
    HCI_OUTGOING_PRE_BUFFER_SIZE=4
    HCI_ACL_CHUNK_SIZE_ALIGNMENT=4
)

pico_add_extra_outputs(pico_ble_rover)