"""
plot_serial.py  –  Hiển thị đồ thị ECG real-time từ cổng USB-Serial (ESP32).
                   Firmware gửi dữ liệu đã lọc theo format:
                       >ECG_FILT:<value_mV>

Yêu cầu:
    pip install pyserial matplotlib numpy

Cách chạy:
    python plot_serial.py
"""

import serial
import threading
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
from collections import deque

# ─────────────────────────────────────────────
# CẤU HÌNH CỔNG SERIAL
# ─────────────────────────────────────────────
PORT     = 'COM5'
BAUDRATE = 115200

# ─────────────────────────────────────────────
# CẤU HÌNH HIỂN THỊ
# ─────────────────────────────────────────────
ECG_FS_HZ       = 256           # sampling rate (Hz) — phải khớp với firmware
DISPLAY_SECONDS = 10            # cửa sổ hiển thị (giây)
DISPLAY_SAMPLES = ECG_FS_HZ * DISPLAY_SECONDS

# Tiền tố gói dữ liệu ECG đã lọc — phải khớp với ECG_SERIAL_PREFIX trong main.c
ECG_FILT_PREFIX = ">ECG_FILT:"

# ─────────────────────────────────────────────
# BUFFER & THỐNG KÊ
# ─────────────────────────────────────────────
ecg_buffer = deque([0.0] * DISPLAY_SAMPLES, maxlen=DISPLAY_SAMPLES)

stats = {
    "total_samples": 0,
    "last_update":   "–",
    "sps_estimate":  0.0,
    "_t0":           time.monotonic(),
    "_count_since":  0,
}


# ═════════════════════════════════════════════
# LUỒNG ĐỌC SERIAL
# ═════════════════════════════════════════════

def read_serial_data():
    """Tác vụ nền: mở cổng COM và đọc từng dòng dữ liệu ECG đã lọc."""
    try:
        with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
            print(f"[Serial] Đã mở cổng {PORT}  –  đang đọc dữ liệu ECG…")
            while True:
                raw_line = ser.readline()
                try:
                    line = raw_line.decode('utf-8', errors='ignore').strip()
                except Exception:
                    continue

                if not line.startswith(ECG_FILT_PREFIX):
                    # Bỏ qua các dòng log ESP_LOGx
                    continue

                try:
                    val = float(line[len(ECG_FILT_PREFIX):])
                except ValueError:
                    continue

                ecg_buffer.append(val)
                stats["total_samples"]  += 1
                stats["_count_since"]   += 1

                # Ước tính SPS mỗi giây
                now = time.monotonic()
                elapsed = now - stats["_t0"]
                if elapsed >= 1.0:
                    stats["sps_estimate"] = stats["_count_since"] / elapsed
                    stats["_count_since"] = 0
                    stats["_t0"]          = now
                    stats["last_update"]  = time.strftime("%H:%M:%S")

    except serial.SerialException as exc:
        print(
            f"\n[Serial] LỖI: {exc}\n"
            f"  → Kiểm tra cổng COM ({PORT}) và đảm bảo không có terminal\n"
            f"    (VSCode Serial Monitor, Arduino IDE…) nào đang mở cổng đó.\n"
        )


# Chạy luồng đọc độc lập, tự kết thúc khi chương trình chính thoát
_reader_thread = threading.Thread(target=read_serial_data, daemon=True)
_reader_thread.start()


# ═════════════════════════════════════════════
# THIẾT LẬP GIAO DIỆN ĐỒ THỊ (dark theme)
# ═════════════════════════════════════════════

BG_COLOR    = "#0d1117"
PANEL_COLOR = "#161b22"
ECG_COLOR   = "#3fb950"    # xanh lá neon
GRID_COLOR  = "#21262d"
TEXT_COLOR  = "#e6edf3"
ACCENT      = "#58a6ff"    # xanh lam nhạt
WARN_COLOR  = "#f85149"    # đỏ

plt.rcParams.update({
    "figure.facecolor": BG_COLOR,
    "axes.facecolor":   PANEL_COLOR,
    "axes.edgecolor":   GRID_COLOR,
    "axes.labelcolor":  TEXT_COLOR,
    "xtick.color":      TEXT_COLOR,
    "ytick.color":      TEXT_COLOR,
    "text.color":       TEXT_COLOR,
    "grid.color":       GRID_COLOR,
    "font.family":      "DejaVu Sans",
})

fig = plt.figure(figsize=(14, 7), facecolor=BG_COLOR)
try:
    fig.canvas.manager.set_window_title("ECG Monitor – Serial Plot")
except Exception:
    pass

gs = gridspec.GridSpec(2, 1, figure=fig, height_ratios=[6, 1], hspace=0.06)

# ── Panel chính: dạng sóng ECG ──────────────
ax_ecg = fig.add_subplot(gs[0])
ax_ecg.set_facecolor(PANEL_COLOR)
ax_ecg.set_ylabel("Biên độ (mV)", color=TEXT_COLOR, fontsize=11)
ax_ecg.tick_params(labelbottom=False)
ax_ecg.grid(True, linestyle="--", linewidth=0.5, color=GRID_COLOR, alpha=0.8)

time_axis = np.linspace(0, DISPLAY_SECONDS, DISPLAY_SAMPLES)
ax_ecg.set_xlim(0, DISPLAY_SECONDS)
ax_ecg.set_ylim(-2.0, 2.0)

(ecg_line,) = ax_ecg.plot(
    time_axis,
    list(ecg_buffer),
    color=ECG_COLOR,
    linewidth=1.2,
    antialiased=True,
)

# Tiêu đề
ax_ecg.text(
    0.5, 0.97,
    "Đồ thị Điện tâm đồ (ECG) – Serial Real-time",
    transform=ax_ecg.transAxes,
    ha="center", va="top",
    fontsize=14, fontweight="bold",
    color=TEXT_COLOR,
)

# Badge trạng thái (góc trên trái)
status_dot = ax_ecg.text(
    0.01, 0.97, "●",
    transform=ax_ecg.transAxes,
    ha="left", va="top",
    fontsize=14, color=WARN_COLOR,
)
status_label = ax_ecg.text(
    0.028, 0.97, f"Đang chờ  {PORT}…",
    transform=ax_ecg.transAxes,
    ha="left", va="top",
    fontsize=10, color=TEXT_COLOR,
)

# ── Panel dưới: trục thời gian + thống kê ───
ax_info = fig.add_subplot(gs[1])
ax_info.set_facecolor(PANEL_COLOR)
ax_info.set_xlabel("Thời gian (Giây)", color=TEXT_COLOR, fontsize=11)
ax_info.set_xlim(0, DISPLAY_SECONDS)
ax_info.set_ylim(0, 1)
ax_info.tick_params(left=False, labelleft=False)
ax_info.grid(True, linestyle="--", linewidth=0.5, color=GRID_COLOR, alpha=0.8)
ax_info.set_xticks(np.arange(0, DISPLAY_SECONDS + 1, 1))

info_text = ax_info.text(
    0.5, 0.5,
    f"Chưa nhận dữ liệu — đang chờ cổng {PORT}…",
    transform=ax_info.transAxes,
    ha="center", va="center",
    fontsize=10, color=TEXT_COLOR, family="monospace",
)

fig.tight_layout(pad=1.5)


# ═════════════════════════════════════════════
# CALLBACK CẬP NHẬT ĐỒ THỊ
# ═════════════════════════════════════════════

def update_plot(frame):
    data = list(ecg_buffer)
    ecg_line.set_ydata(data)

    # Tự động scale trục Y theo dữ liệu thực
    valid = [v for v in data if v != 0.0]
    if len(valid) > 20:
        lo, hi = min(valid), max(valid)
        margin = max((hi - lo) * 0.15, 0.05)
        ax_ecg.set_ylim(lo - margin, hi + margin)

    # Cập nhật trạng thái badge
    if stats["total_samples"] > 0:
        status_dot.set_color(ECG_COLOR)
        status_label.set_text(f"{PORT}  ✓ đang nhận")
    else:
        status_dot.set_color(WARN_COLOR)
        status_label.set_text(f"Đang chờ {PORT}…")

    # Cập nhật dòng thống kê
    if stats["total_samples"] > 0:
        info_text.set_text(
            f"Cổng: {PORT}  @{BAUDRATE} baud   │   "
            f"Mẫu nhận: {stats['total_samples']:,}   │   "
            f"SPS ước tính: {stats['sps_estimate']:.0f} Hz   │   "
            f"Cập nhật: {stats['last_update']}   │   "
            f"fs = {ECG_FS_HZ} Hz   │   Cửa sổ: {DISPLAY_SECONDS} s"
        )

    return ecg_line, status_dot, status_label, info_text


# ═════════════════════════════════════════════
# KHỞI ĐỘNG
# ═════════════════════════════════════════════

if __name__ == "__main__":
    print("=" * 60)
    print(" ECG Serial Plotter – Python")
    print(f" Cổng   : {PORT}  @{BAUDRATE} baud")
    print(f" Prefix : {ECG_FILT_PREFIX}")
    print(f" Cửa sổ : {DISPLAY_SECONDS} s  |  fs = {ECG_FS_HZ} Hz")
    print("=" * 60)

    ani = FuncAnimation(
        fig,
        update_plot,
        interval=50,           # cập nhật mỗi 50 ms (20 FPS)
        blit=True,
        cache_frame_data=False,
    )

    plt.show()