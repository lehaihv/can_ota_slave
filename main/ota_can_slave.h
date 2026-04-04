#pragma once

/**
 * Install and start the TWAI driver. Call once from app_main before any task
 * that needs the CAN bus (including ota_can_slave_start).
 */
void ota_can_init(void);

/**
 * Start the background OTA-slave monitor task. Watches for an OTA trigger on
 * the CAN bus; the TWAI driver must already be running (call ota_can_init first).
 * While idle the task yields frequently so other tasks can transmit freely.
 */
void ota_can_slave_start(void);
