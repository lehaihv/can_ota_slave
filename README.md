# CAN OTA Slave (ESP32-S3)

ESP32-S3 firmware that receives OTA updates over a CAN (TWAI) bus. Designed to pair with a CAN OTA master.

## Hardware

- **MCU:** ESP32-S3 (4 MB flash)
- **CAN transceiver:** Connected to configurable TX/RX pins
- **Partition layout:**

| Name    | Type | SubType | Offset   | Size     |
|---------|------|---------|----------|----------|
| nvs     | data | nvs     | 0x9000   | 20 KB    |
| otadata | data | ota     | 0xE000   | 8 KB     |
| ota_0   | app  | ota_0   | 0x10000  | ~1.87 MB |
| ota_1   | app  | ota_1   | 0x1F0000 | ~1.87 MB |

## OTA Protocol

The slave listens on the CAN bus for a specific trigger sequence, then receives firmware data frames with per-frame ACK and retry logic.

| CAN ID          | Direction   | Purpose                          |
|-----------------|-------------|----------------------------------|
| `CAN_ID_TRIGGER`| Master→Slave| Start OTA handshake (4 magic bytes) |
| `CAN_ID_OTA_START`| Master→Slave| Announce total size & frame count |
| `CAN_ID_DATA`   | Master→Slave| Firmware chunk `[seq_hi][seq_lo][len][payload]` |
| `CAN_ID_ACK`    | Slave→Master| ACK/NAACK for each data frame    |
| `CAN_ID_OTA_END`| Master→Slave| Signal end of data transfer      |
| `CAN_ID_OTA_OK` | Slave→Master| OTA success, slave will reboot   |
| `CAN_ID_OTA_FAIL`| Slave→Master| OTA failed                       |
| `CAN_ID_ABORT`  | Either      | Abort the transfer              |

### Rollback Safety

After rebooting into the new firmware, `esp_ota_mark_app_valid_cancel_rollback()` is called in `app_main()` to confirm the image and prevent automatic rollback to the previous partition.

## Build & Flash

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## Project Structure

```
├── main/
│   ├── main.c              # Entry point, marks firmware valid after OTA
│   ├── ota_can_slave.c     # CAN OTA receive logic (TWAI + esp_ota_*)
│   ├── ota_can_slave.h     # Pin & timing configuration
│   ├── ota_can_protocol.h  # CAN IDs and protocol constants
│   ├── app_tasks.c         # Application tasks (PWM demo, etc.)
│   ├── app_tasks.h
│   └── CMakeLists.txt
├── partitions.csv          # Custom partition table for 4 MB flash
├── sdkconfig.defaults
├── CMakeLists.txt
└── README.md
```

## Configuration

Edit `sdkconfig.defaults` for:
- Flash size (default: 4 MB)
- Task stack sizes
- OTA rollback enable

Edit `main/ota_can_slave.h` for:
- CAN TX/RX GPIO pins
- CAN bitrate
- Timeout and retry constants
