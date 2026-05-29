#include "ble_ecg.h"
#include "max30003_ecg.h"

void setup() {
  Serial.begin(115200);

  // Chờ hệ thống nguồn và USB CDC ổn định
  delay(2000);
  Serial.println("\n===== KHỞI ĐỘNG HỆ THỐNG Y TẾ ECG THỜI GIAN THỰC =====");

  setupBLE();      // Khởi động BLE, bắt đầu quảng bá
  initECGSystem(); // Cấu hình MAX30003, tạo các FreeRTOS task

  Serial.println("[MAIN] Hệ thống sẵn sàng.");
}

void loop() {
  // Toàn bộ xử lý chạy trong FreeRTOS tasks — luồng chính nhường CPU
  vTaskDelay(pdMS_TO_TICKS(5000));
}