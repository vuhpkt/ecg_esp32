#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <stdint.h>

typedef void (*ecg_data_cb_t)(int32_t *ecg_samples, int sample_count);
// Khởi tạo và bắt đầu quét BLE
void ble_manager_init(ecg_data_cb_t callback);

#endif