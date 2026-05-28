#include "max30003_ecg.h"

// Địa chỉ các thanh ghi nội bộ MAX30003 (tra datasheet mục 6)
#define MAX30003_STATUS    0x01
#define MAX30003_EN_INT    0x02
#define MAX30003_MNGR_INT  0x04
#define MAX30003_MNGR_DYN  0x05
#define MAX30003_SW_RST    0x08
#define MAX30003_SYNCH     0x09
#define MAX30003_CNFG_GEN  0x10
#define MAX30003_CNFG_CAL  0x12
#define MAX30003_CNFG_EMUX 0x14
#define MAX30003_CNFG_ECG  0x15
#define MAX30003_FIFO_ECG  0x21

// Đặt 1 để in dữ liệu lên Serial Plotter khi debug (tắt khi chạy thực tế)
#define ENABLE_SERIAL_PLOTTER 0

// Số mẫu tối đa chứa trong FreeRTOS Queue giữa 2 task: 2 giây tại 256 SPS
#define QUEUE_SIZE 512

// Kích thước batch BLE cố định: 32 mẫu = 128 bytes = 1/8 giây tại 256 SPS
#define ECG_BATCH_SAMPLES 32

QueueHandle_t ecgQueue;
TaskHandle_t xTaskReadECGHandle = NULL;
static uint32_t dropped_samples = 0;

// Khai báo nội bộ
void max30003_isr();
void TaskReadECG(void *pvParameters);
void TaskSendBLE(void *pvParameters);
void MAX30003_Write(uint8_t reg, uint32_t data);
uint32_t MAX30003_Read(uint8_t reg);

// ISR phần cứng: dùng Task Notification để đánh thức TaskReadECG tức thì
// Không xử lý dữ liệu trong ISR — chỉ báo hiệu để task xử lý
void max30003_isr() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(xTaskReadECGHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void initECGSystem() {
  ecgQueue = xQueueCreate(QUEUE_SIZE, sizeof(int32_t));
  if (ecgQueue == NULL) {
    Serial.println("[ERR] Không thể tạo FreeRTOS Queue!");
    while(1);
  }

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  pinMode(INTB_PIN, INPUT_PULLUP);
  pinMode(INT2B_PIN, INPUT_PULLUP);

  SPI.setPins(SPI_MISO_PIN, SPI_SCLK_PIN, SPI_MOSI_PIN);
  SPI.begin();

  // Reset chip và cấu hình thanh ghi theo thứ tự bắt buộc của datasheet
  MAX30003_Write(MAX30003_SW_RST, 0x000000);
  delay(100);
  MAX30003_Write(MAX30003_CNFG_GEN, 0x080000);
  MAX30003_Write(MAX30003_CNFG_ECG, 0x415000);
  MAX30003_Write(MAX30003_EN_INT,   0x800002); // Chỉ bật ngắt EINT (FIFO đầy)
  MAX30003_Write(MAX30003_MNGR_INT, 0x980004); // EFIT=19: ngắt khi FIFO tích đủ 20 mẫu
  MAX30003_Write(MAX30003_MNGR_DYN, 0xBF0000);

  // Cấu hình theo chế độ test (xem TEST_MODE trong max30003_ecg.h)
  switch (TEST_MODE) {
    case 1: // Tín hiệu vuông nội bộ để kiểm tra hoạt động chip
      MAX30003_Write(MAX30003_CNFG_ECG,  0x401000);
      MAX30003_Write(MAX30003_CNFG_EMUX, 0x3B0000);
      MAX30003_Write(MAX30003_CNFG_CAL,  0x703800);
      break;
    case 2: // Đo điện cực ngoài, tắt hiệu chỉnh nội
      MAX30003_Write(MAX30003_CNFG_EMUX, 0x350000);
      MAX30003_Write(MAX30003_CNFG_CAL,  0x000000);
      break;
    case 3: // Đo thực tế qua mạch PCB (chế độ sản xuất)
      MAX30003_Write(MAX30003_CNFG_EMUX, 0x000000);
      MAX30003_Write(MAX30003_CNFG_CAL,  0x000000);
      break;
  }

  MAX30003_Write(MAX30003_SYNCH, 0x000000);
  MAX30003_Read(MAX30003_STATUS); // Xóa cờ lỗi ban đầu trước khi bật ngắt

  attachInterrupt(digitalPinToInterrupt(INTB_PIN), max30003_isr, FALLING);

  Serial.println("[ECG] Cấu hình MAX30003 thành công.");

  // Ưu tiên: TaskReadECG (3) > TaskSendBLE (2) để không bao giờ bỏ sót mẫu
  xTaskCreate(TaskReadECG, "ReadECG", 512, NULL, 3, &xTaskReadECGHandle);
  xTaskCreate(TaskSendBLE, "SendBLE", 512, NULL, 2, NULL);
}

// Task 1: Đọc FIFO sau mỗi ngắt, đẩy mẫu vào Queue
// Task ngủ hoàn toàn (0% CPU) cho đến khi ISR báo đủ 20 mẫu
void TaskReadECG(void *pvParameters) {
  (void) pvParameters;
  uint8_t stuck_counter = 0;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Đọc cạn FIFO phần cứng (tối đa 32 mẫu mỗi lần)
    for (int i = 0; i < 32; i++) {
      uint32_t fifo_data = MAX30003_Read(MAX30003_FIFO_ECG);
      uint8_t etag = (fifo_data >> 3) & 0x07;

      if (etag == 6) break; // ETAG=6: FIFO trống

      // Trích 18-bit giá trị ECG từ bits[23:6], sau đó sign-extend lên int32_t
      int32_t ecg_sample = (fifo_data >> 6) & 0x0003FFFF;
      if (ecg_sample & 0x00020000) {
        ecg_sample |= 0xFFFC0000;
      }

      // etag 0–3: mẫu hợp lệ; etag 4–5: mẫu lỗi (bỏ qua)
      if (etag <= 3) {
        if (xQueueSend(ecgQueue, &ecg_sample, 0) != pdPASS) {
          dropped_samples++;
          if ((dropped_samples % 256) == 1) {
            Serial.print("[WARN] Queue ECG đầy, tổng mẫu bị mất: ");
            Serial.println(dropped_samples);
          }
        }

        if (ENABLE_SERIAL_PLOTTER) {
          Serial.print(">ECG: ");
          Serial.println(ecg_sample);
        }
      }
    }

    // Bảo vệ: nếu chân INTB vẫn LOW sau khi đọc xong, có thể chip bị kẹt
    // Đọc STATUS register để reset cờ ngắt bên trong chip
    if (digitalRead(INTB_PIN) == LOW) {
      stuck_counter++;
      if (stuck_counter > 3) {
        MAX30003_Read(MAX30003_STATUS);
        stuck_counter = 0;
      }
    } else {
      stuck_counter = 0;
    }
  }
}

// Task 2: Gom đủ đúng 32 mẫu rồi gửi một lần qua BLE (raw binary, 128 bytes/batch)
// Nếu BLE chưa kết nối: xả sạch Queue để tránh gửi dữ liệu cũ khi kết nối lại
void TaskSendBLE(void *pvParameters) {
  (void) pvParameters;
  int32_t received_sample;
  int32_t tx_buffer[ECG_BATCH_SAMPLES];
  int sample_count = 0;

  for (;;) {
    if (xQueueReceive(ecgQueue, &received_sample, portMAX_DELAY) == pdPASS) {

      if (!isBLEConnected()) {
        xQueueReset(ecgQueue);
        sample_count = 0;
        continue;
      }

      tx_buffer[sample_count++] = received_sample;

      if (sample_count == ECG_BATCH_SAMPLES) {
        uint16_t retry_count = 0;
        while (isBLEConnected() && !sendECGBatchBLE((uint8_t*)tx_buffer, sizeof(tx_buffer))) {
          if ((retry_count++ % 200) == 0) {
            Serial.println("[BLE] Batch ECG 32 mẫu chưa gửi được, đang giữ lại và thử lại...");
          }
          vTaskDelay(pdMS_TO_TICKS(5));
        }

        sample_count = 0;
      }
    }
  }
}

// Giao tiếp SPI với MAX30003 — tốc độ 2MHz, Mode 0, MSB first
void MAX30003_Write(uint8_t reg, uint32_t data) {
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);
  SPI.transfer((reg << 1) | 0x00); // Bit LSB = 0: ghi
  SPI.transfer((data >> 16) & 0xFF);
  SPI.transfer((data >> 8)  & 0xFF);
  SPI.transfer(data & 0xFF);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
}

uint32_t MAX30003_Read(uint8_t reg) {
  uint32_t data = 0;
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);
  SPI.transfer((reg << 1) | 0x01); // Bit LSB = 1: đọc
  data |= (uint32_t)SPI.transfer(0x00) << 16;
  data |= (uint32_t)SPI.transfer(0x00) << 8;
  data |= (uint32_t)SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  return data;
}
