#pragma once

#include <stdint.h>

// ─── TWAI (CAN) pin config ────────────────────────────────────────────────────
#define CAN_TX_PIN          17
#define CAN_RX_PIN          16
// Bitrate: 500 kbps — both boards must match
#define CAN_BITRATE         TWAI_TIMING_CONFIG_500KBITS()

// ─── CAN message IDs (11-bit standard frame) ─────────────────────────────────
// Master → Slave
#define CAN_ID_TRIGGER      0x100   // 4-byte trigger payload
#define CAN_ID_OTA_START    0x101   // payload: total_size (4B) + seq_total (4B)
#define CAN_ID_DATA         0x102   // payload: [seq_hi][seq_lo][chunk_idx][data 5B]
#define CAN_ID_OTA_END      0x103   // no payload
#define CAN_ID_ABORT        0x104   // no payload

// Slave → Master
#define CAN_ID_ACK          0x110   // payload: [seq_hi][seq_lo][status]
//   status: 0x00 = ACK, 0xFF = NACK
#define CAN_ID_OTA_OK       0x111   // OTA written, will reboot
#define CAN_ID_OTA_FAIL     0x112   // OTA failed, kept old firmware

// ─── Trigger bytes ────────────────────────────────────────────────────────────
#define OTA_TRIGGER_BYTE0   0xAA
#define OTA_TRIGGER_BYTE1   0xBB
#define OTA_TRIGGER_BYTE2   0xCC
#define OTA_TRIGGER_BYTE3   0xDD

// ─── Packet fragmentation ─────────────────────────────────────────────────────
// Each CAN DATA frame carries 5 bytes of firmware payload.
// A logical "packet" is 1 CAN frame (seq = frame index in the firmware stream).
#define CAN_PAYLOAD_BYTES   5       // firmware bytes per CAN frame
// Frame layout for CAN_ID_DATA:
//   byte[0] = seq >> 8
//   byte[1] = seq & 0xFF
//   byte[2] = chunk length (1-5, last frame may be shorter)
//   byte[3..7] = firmware data (up to 5 bytes)

// ─── Timeouts & retries ───────────────────────────────────────────────────────
#define OTA_ACK_TIMEOUT_MS  2000
#define OTA_MAX_RETRIES     5
#define CAN_TX_TIMEOUT_MS   100     // TWAI transmit queue timeout

// ─── CRC-16/CCITT-FALSE (run over entire received firmware at the end) ────────
static inline uint16_t crc16_update(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t)byte << 8;
    for (int i = 0; i < 8; i++)
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    return crc;
}

static inline uint16_t crc16_buf(const uint8_t *buf, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++)
        crc = crc16_update(crc, buf[i]);
    return crc;
}
