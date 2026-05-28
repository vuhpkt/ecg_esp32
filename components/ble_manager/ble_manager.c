#include "ble_manager.h"
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"

static const char *TAG = "ECG_GATEWAY";

// Mảng đảo ngược của UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e" (Chuẩn Little-Endian)
static const ble_uuid128_t nus_tx_uuid = 
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 
                     0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t nus_tx_val_handle = 0;
static ecg_data_cb_t user_ecg_cb = NULL;

// 5. NƠI HỨNG DỮ LIỆU: Bắn Callback về cho main.c
static int ble_on_notify(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (user_ecg_cb != NULL) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        uint8_t *raw_data = (uint8_t *)ctxt->om->om_data;

        // Phép thuật Zero-Copy: Ép thẳng 80 bytes nhị phân thành mảng 20 số int32_t
        if (len % 4 == 0) {
            int num_samples = len / 4;
            user_ecg_cb((int32_t *)raw_data, num_samples);
        }
    }
    return 0;
}

// 4. TÌM ĐƯỢC ỐNG DẪN DỮ LIỆU: Bật công tắc (Subscribe) để mạch XIAO bắt đầu gửi
static int ble_on_disc_chr(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg) {
    if (error->status == 0) {
        if (ble_uuid_cmp(&chr->uuid.u, &nus_tx_uuid.u) == 0) {
            ESP_LOGI(TAG, "Đã tìm thấy ống dẫn dữ liệu NUS TX!");
            nus_tx_val_handle = chr->val_handle;
        }
    } else if (error->status == BLE_HS_EDONE) {
        if (nus_tx_val_handle != 0) {
            // Viết 0x0001 vào cờ CCCD (ngay sau value handle) để kích hoạt Notification
            uint8_t value[2] = {1, 0}; 
            ble_gattc_write_flat(conn_handle, nus_tx_val_handle + 1, value, sizeof(value), NULL, NULL);
            ESP_LOGI(TAG, "===> ĐÃ KÍCH HOẠT NHẬN DỮ LIỆU ĐIỆN TIM <===");
        }
    }
    return 0;
}

// 1. MÁY TRẠNG THÁI CHÍNH: Xử lý các sự kiện Vô tuyến
static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_DISC: {
            struct ble_hs_adv_fields fields;
            ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
            
            if (fields.name != NULL && fields.name_len > 0) {
                if (strncmp((char *)fields.name, "ECG_XIAO", fields.name_len) == 0) {
                    ESP_LOGI(TAG, "Tìm thấy mục tiêu! Hủy quét và thực hiện kết nối...");
                    ble_gap_disc_cancel(); 
                    ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &event->disc.addr, 30000, NULL, ble_gap_event, NULL);
                }
            }
            break;
        }
        case BLE_GAP_EVENT_CONNECT: {
            if (event->connect.status == 0) {
                ESP_LOGI(TAG, "Kết nối vât lý THÀNH CÔNG! Đang thỏa thuận MTU...");
                conn_handle = event->connect.conn_handle;
                
                // 2. MỞ RỘNG BĂNG THÔNG: Yêu cầu XIAO dùng gói tin lớn (247 bytes)
                ble_gattc_exchange_mtu(conn_handle, NULL, NULL);
            } else {
                ESP_LOGE(TAG, "Kết nối thất bại. Tự động quét lại...");
                struct ble_gap_disc_params disc_params = {0};
                ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params, ble_gap_event, NULL);
            }
            break;
        }
        case BLE_GAP_EVENT_MTU: {
            ESP_LOGI(TAG, "MTU đã đổi thành: %d bytes. Đang khám phá bộ nhớ XIAO...", event->mtu.value);
            // 3. KHÁM PHÁ: Tìm toàn bộ các Characteristic trên XIAO
            ble_gattc_disc_all_chrs(conn_handle, 1, 0xFFFF, ble_on_disc_chr, NULL);
            break;
        }
        case BLE_GAP_EVENT_NOTIFY_RX: {
            // Khi có dữ liệu mới bắn tới, chuyển tiếp vào hàm xử lý Notify
            struct ble_gatt_access_ctxt ctxt = { .om = event->notify_rx.om };
            ble_on_notify(event->notify_rx.conn_handle, event->notify_rx.attr_handle, &ctxt, NULL);
            break;
        }
        case BLE_GAP_EVENT_DISCONNECT: {
            ESP_LOGW(TAG, "Mất kết nối với XIAO! Bật lại chế độ rình mồi...");
            conn_handle = BLE_HS_CONN_HANDLE_NONE;
            struct ble_gap_disc_params disc_params = {0};
            ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params, ble_gap_event, NULL);
            break;
        }
    }
    return 0;
}

static void ble_app_on_sync(void) {
    ESP_LOGI(TAG, "Hệ thống vô tuyến đã khởi động. Đang quét...");
    struct ble_gap_disc_params disc_params = {0};
    ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params, ble_gap_event, NULL);
}

static void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_manager_init(ecg_data_cb_t callback) {
    user_ecg_cb = callback;
    nimble_port_init();
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(ble_host_task);
}