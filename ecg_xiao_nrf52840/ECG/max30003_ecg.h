#ifndef MAX30003_ECG_H
#define MAX30003_ECG_H

#include <Arduino.h>
#include <SPI.h>
#include "ble_ecg.h"

// Pinout nRF52840 (XIAO BLE) — theo thiết kế PCB
#define SPI_MOSI_PIN 10
#define SPI_MISO_PIN 9
#define SPI_SCLK_PIN 8
#define CS_PIN       7
#define INTB_PIN     6   // Ngắt chính (FIFO đầy)
#define INT2B_PIN    5   // Ngắt phụ (hiện chưa dùng)

// Chế độ hoạt động:
//   1 = Tín hiệu vuông nội bộ (kiểm tra chip không cần điện cực)
//   2 = Điện cực ngoài, tắt hiệu chỉnh
//   3 = Đo thực tế qua mạch PCB (chế độ sản xuất)
#define TEST_MODE 3

void initECGSystem();

#endif