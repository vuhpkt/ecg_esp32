import serial
import threading
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# --- CẤU HÌNH CỔNG COM ---
PORT = 'COM5' 
BAUDRATE = 115200

SPS = 256 
BUFFER_SIZE = 1000

data_buffer = deque([0] * BUFFER_SIZE, maxlen=BUFFER_SIZE)
time_axis = [i / SPS for i in range(BUFFER_SIZE)]

def read_serial_data():
    """Tác vụ chạy ngầm để hứng dữ liệu từ cổng COM"""
    try:
        with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
            print(f"Đã mở cổng {PORT} thành công! Đang đọc dữ liệu...")
            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                # Chỉ lọc những dòng có chứa tiền tố đồ thị >ECG:
                if line.startswith(">ECG:"):
                    try:
                        # Tách lấy phần số đằng sau dấu hai chấm
                        val = int(line.split(":")[1])
                        data_buffer.append(val)
                    except ValueError:
                        pass
    except serial.SerialException as e:
        print(f"Lỗi cổng Serial: {e}. Vui lòng tắt terminal trên VSCode hoặc Arduino trước khi chạy.")

# Chạy luồng đọc Serial độc lập với luồng vẽ đồ thị
thread = threading.Thread(target=read_serial_data, daemon=True)
thread.start()

# --- THIẾT LẬP GIAO DIỆN ĐỒ THỊ ---
fig, ax = plt.subplots(figsize=(10, 5))
ax.set_title("Đồ thị Điện tâm đồ (ECG) qua ESP32 Gateway", fontsize=14, fontweight='bold')
ax.set_ylabel("Biên độ (ADC)", fontsize=11)
ax.set_xlabel("Thời gian (Giây)", fontsize=11) 
ax.grid(True, which='both', linestyle='--', color='gray', alpha=0.5)

ax.set_xlim(0, max(time_axis)) 
ax.set_ylim(-50000, 50000) 

line, = ax.plot(time_axis, data_buffer, color='crimson', linewidth=1.5)

def update_plot(frame):
    line.set_ydata(data_buffer)
    return line,

ani = FuncAnimation(fig, update_plot, interval=30, blit=True, cache_frame_data=False)
plt.tight_layout()
plt.show()