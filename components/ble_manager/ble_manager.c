#include "ble_manager.h"
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"

static const char *TAG = "ECG_GATEWAY";

/*
 * Characteristic NUS TX mà thiết bị ECG dùng để gửi dữ liệu.
 * NimBLE khai báo UUID128 theo thứ tự little-endian, nên byte UUID được đảo
 * từ chuỗi chuẩn: 6e400003-b5a3-f393-e0a9-e50e24dcca9e.
 */
static const ble_uuid128_t nus_tx_uuid = 
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 
                     0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

// Trạng thái kết nối hiện tại và handle của characteristic nhận dữ liệu ECG.
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t nus_tx_val_handle = 0;
static ecg_data_cb_t user_ecg_cb = NULL;

/*
 * Xử lý BLE notification chứa payload ECG.
 * Payload là các mẫu int32_t liên tiếp, nên độ dài hợp lệ phải chia hết cho 4.
 */
static int ble_on_notify(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (user_ecg_cb != NULL) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        uint8_t *raw_data = (uint8_t *)ctxt->om->om_data;

        if (len % 4 == 0) {
            int num_samples = len / 4;
            // Gọi callback ngay trong vòng đời mbuf; không giữ raw_data sau khi trả về.
            user_ecg_cb((int32_t *)raw_data, num_samples);
        }
    }
    return 0;
}

/*
 * Callback khi discover characteristic.
 * Khi tìm thấy NUS TX, lưu value handle; khi quá trình discover kết thúc,
 * ghi CCCD để bật notification từ thiết bị ECG.
 */
static int ble_on_disc_chr(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg) {
    if (error->status == 0) {
        if (ble_uuid_cmp(&chr->uuid.u, &nus_tx_uuid.u) == 0) {
            ESP_LOGI(TAG, "Đã tìm thấy characteristic dữ liệu NUS TX!");
            nus_tx_val_handle = chr->val_handle;
        }
    } else if (error->status == BLE_HS_EDONE) {
        if (nus_tx_val_handle != 0) {
            // CCCD nằm ngay sau value handle; 0x0001 nghĩa là bật notification.
            uint8_t value[2] = {1, 0}; 
            ble_gattc_write_flat(conn_handle, nus_tx_val_handle + 1, value, sizeof(value), NULL, NULL);
            ESP_LOGI(TAG, "===> ĐÃ KÍCH HOẠT NHẬN DỮ LIỆU ĐIỆN TIM <===");
        }
    }
    return 0;
}

/*
 * State machine BLE chính:
 * scan thiết bị ECG_XIAO, kết nối, thỏa thuận MTU, discover characteristic,
 * nhận notification và tự quét lại khi mất kết nối.
 */
static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_DISC: {
            // Mỗi gói quảng bá được parse để tìm tên thiết bị ECG cần kết nối.
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
                ESP_LOGI(TAG, "Kết nối vật lý THÀNH CÔNG! Đang thỏa thuận MTU...");
                conn_handle = event->connect.conn_handle;
                
                // Tăng MTU giúp mỗi notification có thể chứa nhiều mẫu ECG hơn.
                ble_gattc_exchange_mtu(conn_handle, NULL, NULL);
            } else {
                ESP_LOGE(TAG, "Kết nối thất bại. Tự động quét lại...");
                struct ble_gap_disc_params disc_params = {0};
                ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params, ble_gap_event, NULL);
            }
            break;
        }
        case BLE_GAP_EVENT_MTU: {
            ESP_LOGI(TAG, "MTU đã đổi thành: %d bytes. Đang khám phá characteristic trên XIAO...", event->mtu.value);
            // Sau khi có MTU mới, tìm toàn bộ characteristic để lấy handle NUS TX.
            ble_gattc_disc_all_chrs(conn_handle, 1, 0xFFFF, ble_on_disc_chr, NULL);
            break;
        }
        case BLE_GAP_EVENT_NOTIFY_RX: {
            // Chuyển notification của GAP event sang hàm xử lý payload ECG chung.
            struct ble_gatt_access_ctxt ctxt = { .om = event->notify_rx.om };
            ble_on_notify(event->notify_rx.conn_handle, event->notify_rx.attr_handle, &ctxt, NULL);
            break;
        }
        case BLE_GAP_EVENT_DISCONNECT: {
            ESP_LOGW(TAG, "Mất kết nối với XIAO! Bắt đầu quét lại...");
            conn_handle = BLE_HS_CONN_HANDLE_NONE;
            struct ble_gap_disc_params disc_params = {0};
            ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params, ble_gap_event, NULL);
            break;
        }
    }
    return 0;
}

// NimBLE gọi hàm này khi host và controller đã sẵn sàng để bắt đầu scan.
static void ble_app_on_sync(void) {
    ESP_LOGI(TAG, "Hệ thống BLE đã khởi động. Đang quét...");
    struct ble_gap_disc_params disc_params = {0};
    ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params, ble_gap_event, NULL);
}

// Task chạy event loop của NimBLE; hàm này block cho đến khi NimBLE dừng.
static void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_manager_init(ecg_data_cb_t callback) {
    // Lưu callback của tầng ứng dụng trước khi bật BLE để không mất gói đầu tiên.
    user_ecg_cb = callback;
    nimble_port_init();
    // Sau khi stack đồng bộ xong, ble_app_on_sync sẽ bắt đầu quét thiết bị ECG.
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(ble_host_task);
}
