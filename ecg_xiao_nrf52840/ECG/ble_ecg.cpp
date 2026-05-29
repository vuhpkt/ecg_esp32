#include "ble_ecg.h"

BLEUart bleuart;
bool _ble_connected = false;

void connect_callback(uint16_t conn_handle) {
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));
  Serial.print("\n[BLE] Đã kết nối: ");
  Serial.println(central_name);
  _ble_connected = true;
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void) conn_handle;
  Serial.print("\n[BLE] Ngắt kết nối! Mã lỗi GAP: 0x");
  Serial.println(reason, HEX);
  _ble_connected = false;
}

void setupBLE() {
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("ECG_XIAO");

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Connection Interval 7.5–15ms để đảm bảo băng thông truyền ECG liên tục
  Bluefruit.Periph.setConnInterval(6, 12);

  bleuart.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);

  Serial.println("[BLE] Khởi tạo thành công, đang quảng bá...");
}

void sendECGDataBLE(int32_t ecg_sample) {
  if (_ble_connected && bleuart.notifyEnabled()) {
    bleuart.println(ecg_sample);
  }
}

bool isBLEConnected() {
  return _ble_connected;
}

// Gửi một batch dữ liệu nhị phân (raw binary) để tối ưu băng thông BLE
bool sendECGBatchBLE(uint8_t* buffer, int length) {
  if (!_ble_connected || !bleuart.notifyEnabled()) {
    return false;
  }

  return bleuart.write(buffer, length) == (size_t)length;
}
