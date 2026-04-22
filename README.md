# CAN OTA Slave (ESP32-S3)

ESP32-S3 firmware that receives OTA updates over a CAN (TWAI) bus. Designed to pair with a CAN OTA master.

## Hardware

- **MCU:** ESP32-S3 (4 MB flash)
- **CAN transceiver:** GPIO41 (TX) / GPIO42 (RX)
- **CAN bitrate:** 500 kbps
- **Partition layout:**

| Name    | Type | SubType | Offset   | Size     |
|---------|------|---------|----------|----------|
| nvs     | data | nvs     | 0x9000   | 20 KB    |
| otadata | data | ota     | 0xE000   | 8 KB     |
| ota_0   | app  | ota_0   | 0x10000  | ~1.87 MB |
| ota_1   | app  | ota_1   | 0x1F0000 | ~1.87 MB |

## OTA Protocol

The slave listens on the CAN bus for a specific trigger sequence, then receives firmware data frames with per-frame ACK and windowed retry logic.

| CAN ID             | Direction    | Purpose                                          |
|--------------------|--------------|--------------------------------------------------|
| `CAN_ID_TRIGGER`   | Master→Slave | Start OTA handshake (magic bytes `AA BB CC DD`)  |
| `CAN_ID_OTA_START` | Master→Slave | Announce total firmware size (8-byte payload)    |
| `CAN_ID_DATA`      | Master→Slave | Firmware chunk `[seq_hi][seq_lo][payload…]`      |
| `CAN_ID_OTA_END`   | Master→Slave | Signal end of data transfer                      |
| `CAN_ID_ABORT`     | Either       | Abort the transfer                               |
| `CAN_ID_ACK`       | Slave→Master | Per-frame ACK/NACK `[seq_hi][seq_lo][0x00/0xFF]` |
| `CAN_ID_WINDOW_SYNC` | Master→Slave | Window boundary sync probe                     |
| `CAN_ID_WINDOW_ACK`  | Slave→Master | Window-level ACK/NACK with expected seq         |
| `CAN_ID_OTA_OK`    | Slave→Master | OTA success, slave will reboot                   |
| `CAN_ID_OTA_FAIL`  | Slave→Master | OTA failed                                       |

### CAN IDs

```
CAN_ID_TRIGGER     0x100
CAN_ID_OTA_START   0x101
CAN_ID_DATA        0x102
CAN_ID_OTA_END     0x103
CAN_ID_ABORT       0x104
CAN_ID_ACK         0x110
CAN_ID_OTA_OK      0x111
CAN_ID_OTA_FAIL    0x112
CAN_ID_WINDOW_SYNC 0x113
CAN_ID_WINDOW_ACK  0x114
```

### Data Frame Format

Each `CAN_ID_DATA` frame carries a 2-byte sequence header followed by up to 6 bytes of firmware payload:

```
[seq_hi] [seq_lo] [byte0] … [byte5]   (max 8 bytes total)
```

### Windowed Flow Control

The slave uses a window-based ACK scheme alongside per-frame ACKs:
- `CAN_ID_WINDOW_SYNC` probes from the master trigger a `CAN_ID_WINDOW_ACK` reply containing the next expected sequence number.
- On timeout or out-of-order frames the slave sends a NACK window ACK so the master can retransmit from the correct sequence.
- Up to `OTA_MAX_RETRIES` (10) consecutive timeouts before the transfer is aborted.

### Rollback Safety

`app_main` follows the ESP-IDF recommended diagnostic pattern:

1. Check if the running partition is in `ESP_OTA_IMG_PENDING_VERIFY` state.
2. Run a TWAI bus diagnostic (checks driver state, bus-off, and error counters).
3. If diagnostics pass → `esp_ota_mark_app_valid_cancel_rollback()`.
4. If diagnostics fail → `esp_ota_mark_app_invalid_rollback_and_reboot()` (automatic rollback to previous firmware).

## Application Tasks

`app_tasks_start()` launches background tasks that run alongside the OTA monitor:

| Task       | GPIO | Description                                      |
|------------|------|--------------------------------------------------|
| `pwm_task` | 4    | LEDC PWM breathing effect, 1 kHz, 13-bit resolution |

Additional tasks can be registered in `app_tasks.c`.

## Build & Flash

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## Project Structure

```
├── main/
│   ├── main.c                  # Entry point, OTA diagnostic, starts tasks
│   └── OTA/
│       ├── ota_can_slave.c     # CAN OTA receive logic (TWAI + esp_ota_*)
│       ├── ota_can_slave.h     # Public API: ota_can_init / ota_can_slave_start
│       ├── ota_can_protocol.h  # CAN IDs, GPIO pins, timing & protocol constants
│       ├── app_tasks.c         # Application tasks (PWM demo, …)
│       └── app_tasks.h
├── main/CMakeLists.txt
├── partitions.csv              # Partition table for 4 MB flash
├── partitions8MB.csv           # Partition table for 8 MB flash
├── sdkconfig.defaults
├── CMakeLists.txt
└── README.md
```

## Configuration

Key constants are defined in `main/OTA/ota_can_protocol.h`:

| Constant               | Default          | Description                        |
|------------------------|------------------|------------------------------------|
| `CAN_TX_GPIO`          | `GPIO_NUM_41`    | TWAI TX pin                        |
| `CAN_RX_GPIO`          | `GPIO_NUM_42`    | TWAI RX pin                        |
| `CAN_BITRATE`          | 500 kbps         | TWAI timing config                 |
| `OTA_ACK_TIMEOUT_MS`   | 100 ms           | Per-frame receive timeout          |
| `OTA_WINDOW_TIMEOUT_MS`| 400 ms           | Window-level timeout               |
| `OTA_MAX_RETRIES`      | 10               | Max consecutive timeouts before abort |
| `CAN_TX_TIMEOUT_MS`    | 20 ms            | Transmit queue timeout             |
| `PROGRESS_LOG_INTERVAL`| 2000 frames      | Log progress every N frames        |
| `CAN_MAX_FIRMWARE_BYTES`| 6 bytes         | Max firmware payload per CAN frame |
