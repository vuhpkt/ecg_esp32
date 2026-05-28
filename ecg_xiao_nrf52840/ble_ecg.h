#ifndef BLE_ECG_H
#define BLE_ECG_H

#include <Arduino.h>
#include <bluefruit.h>

void setupBLE();
void sendECGDataBLE(int32_t ecg_sample);
bool isBLEConnected();

// Gửi batch ECG dưới dạng raw binary; trả về true nếu ghi đủ length byte
bool sendECGBatchBLE(uint8_t* buffer, int length);

#endif
