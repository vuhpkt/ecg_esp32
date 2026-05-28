#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <stdint.h>

/*
 * Callback nhận dữ liệu ECG đã được tách từ BLE notification.
 *
 * ecg_samples: mảng mẫu ECG dạng int32_t.
 * sample_count: số mẫu trong mảng.
 *
 * Callback được gọi từ luồng BLE, nên xử lý nhanh. Nếu cần dùng dữ liệu sau
 * khi callback trả về, hãy copy mẫu ra buffer riêng thay vì giữ con trỏ này.
 */
typedef void (*ecg_data_cb_t)(int32_t *ecg_samples, int sample_count);

/*
 * Khởi tạo NimBLE, đăng ký callback nhận ECG và bắt đầu quy trình scan/connect.
 */
void ble_manager_init(ecg_data_cb_t callback);

#endif
