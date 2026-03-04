#pragma once

// -------------------------------------------------------
//  BTstack 编译期配置
//  注意：pico-sdk 的 CMake 已通过命令行 -D 定义了部分宏，
//  此处全部用 ifndef 保护，避免重定义 warning
// -------------------------------------------------------

#ifndef ENABLE_BLE
#define ENABLE_BLE
#endif

#ifndef ENABLE_GATT_SERVER
#define ENABLE_GATT_SERVER
#endif

#ifndef ENABLE_LE_PERIPHERAL
#define ENABLE_LE_PERIPHERAL
#endif

#ifndef ENABLE_SM_MINIMAL
#define ENABLE_SM_MINIMAL
#endif

#ifndef ENABLE_PRINTF_HEXDUMP
#define ENABLE_PRINTF_HEXDUMP
#endif

// ATT DB 静态分配大小
#ifndef MAX_ATT_DB_SIZE
#define MAX_ATT_DB_SIZE                     512
#endif

// 连接数
#ifndef MAX_NR_HCI_CONNECTIONS
#define MAX_NR_HCI_CONNECTIONS              1
#endif
#ifndef MAX_NR_BLE_CONNECTIONS
#define MAX_NR_BLE_CONNECTIONS              1
#endif
#ifndef MAX_NR_GATT_CLIENTS
#define MAX_NR_GATT_CLIENTS                 1
#endif

// LE Device DB
#ifndef MAX_NR_LE_DEVICE_DB_ENTRIES
#define MAX_NR_LE_DEVICE_DB_ENTRIES         4
#endif
#ifndef NVM_NUM_DEVICE_DB_ENTRIES
#define NVM_NUM_DEVICE_DB_ENTRIES           4
#endif

// L2CAP
#ifndef MAX_NR_L2CAP_CHANNELS
#define MAX_NR_L2CAP_CHANNELS               4
#endif
#ifndef MAX_NR_L2CAP_SERVICES
#define MAX_NR_L2CAP_SERVICES               2
#endif

// 日志
#ifndef ENABLE_LOG_INFO
#define ENABLE_LOG_INFO
#endif
#ifndef ENABLE_LOG_ERROR
#define ENABLE_LOG_ERROR
#endif

// BTstack 内存池
#ifndef MAX_NR_BTSTACK_LINK_KEY_DB_MEMORY_ENTRIES
#define MAX_NR_BTSTACK_LINK_KEY_DB_MEMORY_ENTRIES  2
#endif

// HCI ACL buffer
#ifndef HCI_ACL_PAYLOAD_SIZE
#define HCI_ACL_PAYLOAD_SIZE                255
#endif