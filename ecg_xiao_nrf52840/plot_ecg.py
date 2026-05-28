import asyncio
import threading
import logging
import time
import struct
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from bleak import BleakClient, BleakScanner
from collections import deque

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)

# UUID của Nordic UART Service (NUS) — chuẩn cố định, không đổi
UART_TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
DEVICE_NAME = "ECG_XIAO"

SPS = 256          # Tần số lấy mẫu cấu hình trên MAX30003 (Hz)
BUFFER_SIZE = 1000 # Số mẫu hiển thị (~3.9 giây tại 256 SPS)

data_buffer = deque([0] * BUFFER_SIZE, maxlen=BUFFER_SIZE)
rx_buffer = bytearray() # Đệm byte thô để ghép gói tin BLE bị chẻ nhỏ

# Trục X cố định theo thời gian; dữ liệu cuộn từ phải sang trái
time_axis = [i / SPS for i in range(BUFFER_SIZE)]

# Biến đo tốc độ nhận thực tế
sample_count = 0
last_time = time.time()


def handle_disconnect(client: BleakClient):
    logging.warning(f"Mất kết nối với {client.address}! Kiểm tra nguồn mạch.")


def handle_rx(sender, data: bytearray):
    """Giải mã luồng raw binary từ BLE thành các mẫu ECG int32_t."""
    global rx_buffer, sample_count

    rx_buffer.extend(data)

    # Mỗi mẫu ECG = 4 bytes (int32_t, little-endian)
    # Vòng lặp bảo vệ trường hợp BLE stack chẻ gói tin ra nhiều notify
    while len(rx_buffer) >= 4:
        chunk = rx_buffer[:4]
        del rx_buffer[:4]
        try:
            val = struct.unpack('<i', chunk)[0]
            data_buffer.append(val)
            sample_count += 1
        except struct.error as e:
            logging.error(f"Lỗi giải mã: {e}")


async def monitor_sps():
    """Chạy ngầm, in tốc độ nhận thực tế mỗi giây để kiểm tra đường truyền."""
    global sample_count, last_time
    while True:
        await asyncio.sleep(1.0)
        current_time = time.time()
        sps = sample_count / (current_time - last_time)
        if sample_count > 0:
            logging.info(f"Tốc độ nhận: {sps:.1f} SPS")
        sample_count = 0
        last_time = current_time


async def run_ble():
    logging.info(f"Đang quét thiết bị '{DEVICE_NAME}'...")
    devices = await BleakScanner.discover(timeout=5.0)
    target_device = next((d for d in devices if d.name == DEVICE_NAME), None)

    if not target_device:
        logging.error("Không tìm thấy thiết bị! Đảm bảo mạch XIAO đang bật.")
        return

    client = BleakClient(target_device, disconnected_callback=handle_disconnect)
    try:
        await client.connect()
        logging.info("Kết nối thành công! Bắt đầu nhận dữ liệu...")

        asyncio.create_task(monitor_sps())
        await client.start_notify(UART_TX_CHAR_UUID, handle_rx)

        while client.is_connected:
            await asyncio.sleep(1.0)

    except Exception as e:
        logging.error(f"Lỗi BLE: {e}")
    finally:
        if client.is_connected:
            await client.disconnect()
            logging.info("Đã ngắt kết nối an toàn.")


def start_asyncio_thread():
    """Chạy vòng lặp BLE trên thread riêng để không block giao diện đồ thị."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(run_ble())


if __name__ == "__main__":
    ble_thread = threading.Thread(target=start_asyncio_thread, daemon=True)
    ble_thread.start()

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.set_title("Đồ thị Điện tâm đồ (ECG) Thời gian thực", fontsize=14, fontweight='bold')
    ax.set_ylabel("Biên độ (Đơn vị ADC)", fontsize=11)
    ax.set_xlabel("Thời gian (Giây)", fontsize=11)
    ax.grid(True, which='both', linestyle='--', color='gray', alpha=0.5)
    ax.set_xlim(0, max(time_axis))

    # Điều chỉnh y_lim nếu cần, phụ thuộc vào cấu hình Gain của MAX30003
    ax.set_ylim(-50000, 50000)

    line, = ax.plot(time_axis, data_buffer, color='crimson', linewidth=1.5)

    def update_plot(frame):
        line.set_ydata(data_buffer)
        return line,

    # ~33 FPS để đồ thị mượt mà mà không chiếm CPU
    ani = FuncAnimation(fig, update_plot, interval=30, blit=True, cache_frame_data=False)
    plt.tight_layout()

    try:
        plt.show()
    except KeyboardInterrupt:
        logging.info("Đang đóng chương trình...")