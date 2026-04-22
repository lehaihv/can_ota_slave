#pragma once

#include <stdint.h>

#define CAN_TX_GPIO         GPIO_NUM_41
#define CAN_RX_GPIO         GPIO_NUM_42
#define CAN_BITRATE         TWAI_TIMING_CONFIG_500KBITS()

#define CAN_ID_TRIGGER      0x100
#define CAN_ID_OTA_START    0x101
#define CAN_ID_DATA         0x102
#define CAN_ID_OTA_END      0x103
#define CAN_ID_ABORT        0x104

#define CAN_ID_ACK          0x110
#define CAN_ID_OTA_OK       0x111
#define CAN_ID_OTA_FAIL     0x112
#define CAN_ID_WINDOW_SYNC  0x113
#define CAN_ID_WINDOW_ACK   0x114

#define OTA_TRIGGER_BYTE0   0xAA
#define OTA_TRIGGER_BYTE1   0xBB
#define OTA_TRIGGER_BYTE2   0xCC
#define OTA_TRIGGER_BYTE3   0xDD

#define CAN_HEADER_BYTES        2
#define CAN_MAX_FIRMWARE_BYTES  6

#define OTA_ACK_TIMEOUT_MS      100
#define OTA_WINDOW_TIMEOUT_MS   400
#define OTA_MAX_RETRIES         10
#define CAN_TX_TIMEOUT_MS       20
#define PROGRESS_LOG_INTERVAL   2000

static inline uint16_t crc16_update(uint16_t crc, uint8_t byte) {
    crc ^= (uint16_t)byte << 8;
    for (int i = 0; i < 8; i++)
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    return crc;
}
static inline uint16_t crc16_buf(const uint8_t *buf, uint32_t len) {
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++) crc = crc16_update(crc, buf[i]);
    return crc;
}