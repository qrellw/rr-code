# RR-Laser & PPG Signal Suite — Firmware + GUI Phân Tích + Ghi Dữ Liệu

```
Code/
├── RR_Serial/RR_Serial.ino       ← Firmware chính ESP32-C3 (Adafruit_ADS1X15)
├── rr_app/                       ← Gói module xử lý tín hiệu lõi
│   ├── core/dsp_filters.py       ← IIR, FIR Equiripple, FIR Window, Hampel, Savitzky-Golay, Kalman, EMA
│   └── core/serial_worker.py     ← QThread đọc Serial + Ghi CSV 11 cột + Ước lượng nhịp trực tiếp
├── rr_suite.py                   ← Ứng dụng GUI chính (PyQt6 + pyqtgraph)
├── ppg_suite.py                  ← Launcher/alias tương thích ngược
├── rr_logger.py                  ← Script dòng lệnh vẽ đồ thị matplotlib & ghi CSV
├── rr_tools.py                   ← Script dòng lệnh thu dữ liệu theo thời gian
├── Test_ADS1115/                 ← Sketch Arduino kiểm tra I2C
└── README.md
```

## 1. Ứng dụng Phân tích Toàn diện (`rr_suite.py`)

Chạy ứng dụng bằng lệnh:
```bash
python rr_suite.py
# hoặc
python ppg_suite.py
```

Ứng dụng đồng bộ hoàn toàn với dữ liệu trong thư mục `rrdata/` và bao gồm 4 tab chuyên sâu:

1. **Live Plotting (Thu thập trực tiếp):**
   - Tự động nhận diện cổng ESP32-C3 / USB-Serial / CH340.
   - Hỗ trợ chọn **Mode 1** (~50 Hz liên tục) hoặc **Mode 0** (~8 Hz 2 pha trừ tối).
   - Tự động gửi lệnh `1`/`0` và `z` để bắt đầu truyền CSV.
   - Các nút điều khiển nhanh phần cứng: Hiệu chuẩn `k` (`c`), Bật/Tắt Laser (`l`), Reset `k=1` (`r`), Chẩn đoán (`d`), Quét I2C (`s`).
   - Vẽ thời gian thực tỉ số vị trí `r` (nhịp thở), điện áp quang `Va` & `Vb` (mV).
   - Hiển thị nhịp thở tức thời (`bpm`), cờ trạng thái (`flags`) và tự động lưu trực tiếp file CSV 11 cột chuẩn vào `rrdata/`.

2. **Offline Analysis (Phân tích ngoại tuyến):**
   - Mở và phân tích trực tiếp các file CSV trong `rrdata/` (`rr_1_*.csv`) hoặc định dạng PPG cũ.
   - Cho phép chọn kênh phân tích: `r`, `Va`, `Vb`, `Vb - Va (Hiệu quang)`, `rr`.
   - Tự động ước lượng tần số lấy mẫu thực tế $F_s$ từ nhãn thời gian `t_ms` / `host_s`.
   - Chuyển đổi linh hoạt trục hoành: **Thời gian (Giây)** hoặc **Số mẫu (Samples)**.
   - Phân tích phổ Fourier (FFT) với các dải sinh lý học được tô màu trực quan (Baseline drift, Nhịp thở 0.1–0.5 Hz, Sóng hài).
   - Tính đạo hàm bậc 1 (**VPG**) và bậc 2 (**APG**).
   - Tự động tìm đỉnh sóng thích ứng và tính nhịp thở trung bình (Breaths/min).
   - Thước đo và đánh dấu điểm tương tác (Ctrl+D, Ctrl+Z, Ctrl+Y).

3. **Bộ lọc số (DSP Filter Pipeline):**
   - Thiết kế chuỗi lọc đa tầng: IIR Butterworth, FIR Equiripple (Parks-McClellan có cơ chế chống phân kỳ & Kaiser fallback), FIR Window, Hampel (loại bỏ xung dị biệt), Savitzky-Golay (làm mượt bảo toàn đỉnh), Kalman Filter, EMA.
   - Các mẫu cấu hình nhanh (Presets): Nhịp thở (0.1–0.5 Hz), Khử nhiễu thông thấp, Khử trôi nền, Nhịp tim PPG...
   - Xem đáp ứng tần số biên độ/pha và xem trước tín hiệu lọc theo thời gian thực.
   - Xuất mã nguồn C/C++ (mảng hệ số SOS cho IIR hoặc Taps cho FIR) để nạp thẳng vào vi điều khiển.
   - Xuất file CSV tín hiệu đã lọc.

4. **Phân tích Biến đổi Sóng (Wavelet DWT):**
   - Phân rã DWT 6 mức với các họ sóng `sym4`, `sym8`, `db4`, `db8`, `coif4`, `haar`.
   - Tự động gán dải tần số tương ứng theo $F_s$ thực tế (cA6, cD6..cD1).
   - Tùy chọn triệt tiêu từng dải tần để loại bỏ nhiễu/trôi nền và tái tạo tín hiệu sạch bằng IDWT.

---

## 2. Phần cứng / Kết nối chân

| Tín hiệu | GPIO | Ghi chú |
|---|---|---|
| SDA | 8 | I2C Data cho ADS1115 |
| SCL | 7 | I2C Clock (**không dùng chân 9**) |
| LASER_EN | 0 | Qua MOSFET 2N7002, trở kéo xuống R14 100k |
| VBAT_SENSE | 21 | C3 không có ADC ở chân này → giá trị luôn 0 |

ADS1115 địa chỉ `0x48` (ADDR nối GND):
- `AIN0` = Va (Photodiode A)
- `AIN1` = Vb (Photodiode B)
- `AIN2` = Vbias (Điện áp phân cực ~228 mV)
- `AIN3` = GND

---

## 3. Định dạng dữ liệu CSV (Đồng bộ với `rrdata/`)

Mỗi dòng dữ liệu trong các file CSV gồm 11 trường:
```
host_s, t_ms, Va, Vb, Va0, Vb0, Vbias, r, Vbat, flags, rr
```
- `host_s`: Thời gian trên máy tính (giây).
- `t_ms`: Thời gian `millis()` trên vi điều khiển ESP32 (ms).
- `Va`, `Vb`: Điện áp đọc được từ hai photodiode (mV).
- `Va0`, `Vb0`: Mẫu tối (khi laser tắt).
- `Vbias`: Điện áp phân cực tham chiếu (mV).
- `r`: Tỉ số vị trí chuẩn hóa $r = \frac{k \cdot Vb - Va}{Va + k \cdot Vb}$ (tín hiệu nhịp thở chính). Chiều: **hít vào → r tăng, thở ra → r giảm** (dấu đã đảo so với công thức gốc vì thực đo Vb > Va).
- `Vbat`: Điện áp pin (mV).
- `flags`: Cờ trạng thái lỗi (bit0: lỗi ADS, bit1: bão hòa, bit2: Vbias mặc định, bit3: pin yếu).
- `rr`: Nhịp thở ước lượng (lần/phút).
