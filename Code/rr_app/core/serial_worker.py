"""
serial_worker.py - Luồng đọc Serial thời gian thực từ ESP32 RR-Laser / PPG.
Tương thích định dạng CSV 11 cột trong rrdata/:
    host_s, t_ms, Va, Vb, Va0, Vb0, Vbias, r, Vbat, flags, rr
"""

import os
import csv
import time
import math
from datetime import datetime
from collections import deque
import numpy as np
import serial
import serial.tools.list_ports
from PyQt6.QtCore import QThread, pyqtSignal


class BandpassRBJ:
    """Biquad band-pass RBJ dùng cho ước lượng nhịp thở."""
    def __init__(self, fs=45.0, f0=0.25, q=0.8):
        self.z1 = self.z2 = 0.0
        self.set_params(fs, f0, q)

    def set_params(self, fs, f0=0.25, q=0.8):
        self.fs = max(fs, 1.0)
        w0 = 2 * math.pi * f0 / self.fs
        al = math.sin(w0) / (2 * q)
        a0 = 1 + al
        self.b0 = al / a0
        self.b1 = 0.0
        self.b2 = -al / a0
        self.a1 = -2 * math.cos(w0) / a0
        self.a2 = (1 - al) / a0

    def step(self, x):
        y = self.b0 * x + self.z1
        self.z1 = self.b1 * x - self.a1 * y + self.z2
        self.z2 = self.b2 * x - self.a2 * y
        return y


class LiveBreathRate:
    """Ước lượng nhịp thở tức thời từ tỉ số r."""
    def __init__(self, fs=45.0):
        self.bp = BandpassRBJ(fs)
        self.env = 0.0
        self.rr = 0.0
        self.last_cross = None
        self.armed = True
        self.periods = deque(maxlen=3)
        self.t = 0.0
        self.dt = 1.0 / max(fs, 1.0)
        self.skip = 4.0

    def add(self, r):
        self.t += self.dt
        y = self.bp.step(r)
        a = abs(y)
        self.env = a if a > self.env else self.env * 0.98
        if self.t < self.skip or self.env < 0.008:
            return self.rr
        h = 0.35 * self.env
        if self.armed and y > h:
            self.armed = False
            if self.last_cross is not None:
                per = self.t - self.last_cross
                if 1.5 < per < 12.0:
                    self.periods.append(per)
                    if len(self.periods) >= 2:
                        med = sorted(self.periods)[len(self.periods) // 2]
                        self.rr = 60.0 / med
            self.last_cross = self.t
        elif not self.armed and y < -h:
            self.armed = True
        return self.rr


class SerialThread(QThread):
    """
    QThread thu nhận dữ liệu Serial liên tục từ ESP32-C3 / bo vi điều khiển.
    Ghi trực tiếp ra file CSV chuẩn trong thư mục rrdata/
    """
    # dict chứa {host_s, t_ms, va, vb, va0, vb0, vbias, r, vbat, flags, rr}
    new_data = pyqtSignal(dict)
    # Signal tương thích ngược cho ppg live plot cũ: (t_ms, va, vb, r)
    legacy_data = pyqtSignal(float, float, float, float)
    diagnostic_msg = pyqtSignal(str)
    error = pyqtSignal(str)
    connection_status = pyqtSignal(bool, str)

    def __init__(self):
        super().__init__()
        self.port = ""
        self.baudrate = 115200
        self.mode = "1"  # "1": ~50Hz, "0": ~8Hz trừ tối
        self.auto_record = True
        self.is_running = False
        self.ser = None
        self.save_file = None
        self.csv_writer = None
        self.save_path = ""
        self.breath_est = LiveBreathRate(45.0)

    @staticmethod
    def get_available_ports():
        """Tìm và chấm điểm các cổng COM khả dụng."""
        ports = serial.tools.list_ports.comports()
        result = []
        for p in ports:
            d = ((p.description or '') + ' ' + (p.manufacturer or '')).lower()
            score = 0
            if any(k in d for k in ('jtag', 'esp32', 'usb-serial', 'usb serial')):
                score = 3
            elif 'ch340' in d or 'cp210' in d:
                score = 2
            result.append((p.device, p.description, score))
        result.sort(key=lambda x: x[2], reverse=True)
        return result

    def send_command(self, cmd):
        """Gửi lệnh ký tự đơn hoặc chuỗi xuống firmware (c, l, r, d, s, x, z...)."""
        if self.ser and self.ser.is_open:
            try:
                if isinstance(cmd, str):
                    cmd = cmd.encode('utf-8')
                self.ser.write(cmd)
                self.ser.flush()
                return True
            except Exception as e:
                self.error.emit(f"Lỗi gửi lệnh '{cmd}': {str(e)}")
        return False

    def run(self):
        self.is_running = True
        self.t0 = time.monotonic()
        self.sample_count = 0
        buf = ""

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            time.sleep(1.2)  # Đợi ESP32 khởi động / boot CDC
            self.ser.reset_input_buffer()

            # Gửi chế độ và lệnh 'z' để firmware bắt đầu in CSV
            self.ser.write(self.mode.encode('utf-8'))
            time.sleep(0.05)
            self.ser.write(b'z')
            time.sleep(0.05)

            # Tạo thư mục rrdata (nằm cạnh Code)
            current_dir = os.path.dirname(os.path.abspath(__file__))
            # Code/rr_app/core -> Code/ -> NCKH/rrdata
            rrdata_dir = os.path.abspath(os.path.join(current_dir, "..", "..", "..", "rrdata"))
            os.makedirs(rrdata_dir, exist_ok=True)

            if self.auto_record:
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                filename = f"rr_{self.mode}_{timestamp}.csv"
                self.save_path = os.path.join(rrdata_dir, filename)
                self.save_file = open(self.save_path, mode='w', newline='', encoding='utf-8')
                self.csv_writer = csv.writer(self.save_file)
                self.csv_writer.writerow([
                    'host_s', 't_ms', 'Va', 'Vb', 'Va0', 'Vb0', 'Vbias',
                    'r', 'Vbat', 'flags', 'rr'
                ])

            self.connection_status.emit(True, f"Đã kết nối {self.port} @ {self.baudrate}")

            while self.is_running:
                try:
                    avail = self.ser.in_waiting
                except Exception:
                    avail = 1

                chunk = self.ser.read(max(1, avail))
                if chunk:
                    buf += chunk.decode('utf-8', errors='ignore')
                    while '\n' in buf:
                        line, buf = buf.split('\n', 1)
                        line = line.strip()
                        if not line:
                            continue

                        # Dòng chẩn đoán (#...)
                        if line.startswith('#'):
                            self.diagnostic_msg.emit(line)
                            continue

                        parts = line.split(',')
                        # Định dạng RR-Laser 9 giá trị:
                        # t_ms, Va, Vb, Va0, Vb0, Vbias, r, Vbat, flags
                        if len(parts) >= 9:
                            try:
                                t_ms = int(float(parts[0]))
                                va = float(parts[1])
                                vb = float(parts[2])
                                va0 = float(parts[3])
                                vb0 = float(parts[4])
                                vbias = float(parts[5])
                                r = float(parts[6])
                                vbat = int(float(parts[7]))
                                flags = int(float(parts[8]))
                            except ValueError:
                                continue

                            self.sample_count += 1
                            host_s = time.monotonic() - self.t0
                            rr = self.breath_est.add(r)

                            data_dict = {
                                'host_s': round(host_s, 4),
                                't_ms': t_ms,
                                'va': va,
                                'vb': vb,
                                'va0': va0,
                                'vb0': vb0,
                                'vbias': vbias,
                                'r': r,
                                'vbat': vbat,
                                'flags': flags,
                                'rr': round(rr, 1)
                            }
                            self.new_data.emit(data_dict)
                            self.legacy_data.emit(float(t_ms), float(va), float(vb), float(r))

                            if self.csv_writer and self.save_file:
                                self.csv_writer.writerow([
                                    round(host_s, 4), t_ms, int(va), int(vb),
                                    int(va0), int(vb0), int(vbias),
                                    round(r, 5), vbat, flags, round(rr, 1)
                                ])
                                if self.sample_count % 25 == 0:
                                    self.save_file.flush()

                        elif len(parts) == 4:
                            # Định dạng cũ: tran, san, ppg, truc_0
                            try:
                                t, s, p, z = map(float, parts)
                                host_s = time.monotonic() - self.t0
                                data_dict = {
                                    'host_s': round(host_s, 4),
                                    't_ms': int(host_s * 1000),
                                    'va': p,
                                    'vb': 0.0,
                                    'va0': 0.0,
                                    'vb0': 0.0,
                                    'vbias': 0.0,
                                    'r': p,
                                    'vbat': 0,
                                    'flags': 0,
                                    'rr': 0.0
                                }
                                self.new_data.emit(data_dict)
                                self.legacy_data.emit(t, s, p, z)
                            except ValueError:
                                pass
                        elif len(parts) == 1:
                            try:
                                p = float(parts[0])
                                host_s = time.monotonic() - self.t0
                                data_dict = {
                                    'host_s': round(host_s, 4),
                                    't_ms': int(host_s * 1000),
                                    'va': p,
                                    'vb': 0.0,
                                    'va0': 0.0,
                                    'vb0': 0.0,
                                    'vbias': 0.0,
                                    'r': p,
                                    'vbat': 0,
                                    'flags': 0,
                                    'rr': 0.0
                                }
                                self.new_data.emit(data_dict)
                                self.legacy_data.emit(0.0, 0.0, p, 0.0)
                            except ValueError:
                                pass

        except Exception as e:
            self.error.emit(str(e))
        finally:
            if self.save_file:
                try:
                    self.save_file.flush()
                    self.save_file.close()
                except Exception:
                    pass
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(b'x')  # Trả firmware về chế độ in AIN
                    self.ser.close()
                except Exception:
                    pass
            self.connection_status.emit(False, "Đã ngắt kết nối")

    def stop(self):
        self.is_running = False
        self.wait(1500)
