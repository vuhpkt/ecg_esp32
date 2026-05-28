#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "ble_manager.h" 

static const char *TAG = "ECG_MAIN";

// Biến toàn cục lưu tổng số mẫu đã nhận được
volatile uint32_t total_samples = 0;

// --- 1. HÀM CALLBACK: IN GIÁ TRỊ ĐO ---
void process_ecg_data(int32_t *samples, int count) {
    // Cộng dồn số mẫu vừa nhận được vào biến tổng
    total_samples += count;

    // In giá trị đo ra Terminal (Định dạng tương thích Arduino Serial Plotter)
    for (int i = 0; i < count; i++) {
        printf(">ECG:%" PRId32 "\n", samples[i]);
    }
}

// --- 2. TASK GIÁM SÁT: TÍNH TOÁN VÀ IN SPS ---
void monitor_sps_task(void *arg) {
    uint32_t last_sample_count = 0;
    
    for (;;) {
        // Ngủ đúng 1 giây (1000ms)
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Lấy số đếm hiện tại tại thời điểm chốt sổ
        uint32_t current_count = total_samples;
        
        // Tính số mẫu nhận được trong đúng 1 giây qua
        uint32_t sps = current_count - last_sample_count;
        last_sample_count = current_count;

        // Chỉ in log nếu có dữ liệu truyền về, tránh spam Terminal khi đang mất kết nối
        if (sps > 0) {
            ESP_LOGI(TAG, "Tốc độ nhận: %" PRIu32 " SPS (Mẫu/giây)", sps);
        }
    }
}

void app_main(void) {
    // Khởi tạo NVS Flash
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "\n--- KHỞI ĐỘNG GATEWAY Y TẾ ESP32 ---\n");

    // Khởi tạo Task đếm SPS chạy độc lập ở Core 0 (độ ưu tiên trung bình)
    xTaskCreate(monitor_sps_task, "monitor_sps", 2048, NULL, 4, NULL);

    // Kích hoạt Component Vô tuyến và truyền hàm xử lý dữ liệu vào
    ble_manager_init(process_ecg_data);

    // Main Task nghỉ ngơi hoàn toàn
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}