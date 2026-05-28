#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// Build / feature switches
// ─────────────────────────────────────────────────────────────────────────────

#define RUN_DEMO_HTTP_REQUEST           0
#define DEBUG_BUTTONS                   0
#define USE_SOCKET_SSL                  0
#define SHOW_SOCKET_OFFLINE_ON_OLED     0
#define ENABLE_LOW_POWER_MODE           1

// Plain WebSocket path — no EIO query string, no Socket.IO namespace overhead.
#define WS_PATH   "/ws"

// ─────────────────────────────────────────────────────────────────────────────
// Timing settings
// ─────────────────────────────────────────────────────────────────────────────

#define BUTTON_DEBOUNCE_MS                     35UL
#define BUTTON_TASK_PERIOD_MS                  5UL
#define WIFI_RETRY_INTERVAL_MS                 30000UL
#define WIFI_STATUS_PRINT_INTERVAL_MS          2000UL
#define SOCKET_DISCONNECT_MESSAGE_INTERVAL_MS  10000UL
#define SOCKET_RECONNECT_INTERVAL_MS           10000UL
#define INACTIVITY_SLEEP_MS                    180000UL

// ─────────────────────────────────────────────────────────────────────────────
// OLED settings
// ─────────────────────────────────────────────────────────────────────────────

#define I2C_ADDRESS                     0x3C
#define OLED_SDA                        21
#define OLED_SCL                        22

// ─────────────────────────────────────────────────────────────────────────────
// Button / sequence settings
// ─────────────────────────────────────────────────────────────────────────────

#define RANDOM_BUTTON_PIN               27
#define SOCKET_BUTTON_PIN               23
#define RF_RECEIVER_PIN                 35   // 433 MHz data line — also used as light-sleep wake source
#define MAX_BACKEND_SEQUENCE_STEPS      32


#define KEYPAD_ROWS 4
#define KEYPAD_COLS 4
#define GYRO_ACTIVITY_THRESHOLD_MS2     0.05f
