#!/usr/bin/env python3
"""
rr_suite.py - Bộ công cụ thu nhận, phân tích tín hiệu RR-Laser & PPG (Advanced Suite).
Tương thích hoàn toàn với file CSV trong rrdata/:
    host_s, t_ms, Va, Vb, Va0, Vb0, Vbias, r, Vbat, flags, rr
Đồng thời hỗ trợ định dạng PPG cũ:
    Gia_tri_PPG, Raw_IR, Original, Filtered...

Bao gồm 4 Tab chức năng:
1. Thu nhận trực tiếp (Live Plotting & Serial Streaming)
2. Phân tích ngoại tuyến (Offline Analysis: FFT, VPG/APG, Tìm đỉnh & Nhịp thở)
3. Phát triển Bộ lọc số (DSP Filter Pipeline, Đáp ứng tần số, Xuất mã C/C++)
4. Phân tích Biến đổi Sóng (Wavelet DWT/IDWT đa mức)
"""

import sys
import os

# Đảm bảo thư mục chứa file script nằm trong sys.path
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

import csv
from datetime import datetime
from collections import deque
import numpy as np
import pandas as pd
import scipy.fftpack
from scipy.signal import find_peaks
import pywt

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QComboBox, QLabel, QTabWidget, QFileDialog, QMessageBox,
    QSplitter, QSpinBox, QDoubleSpinBox, QCheckBox, QListWidget, QListWidgetItem,
    QGroupBox, QGridLayout, QFormLayout, QTextEdit, QDialog, QInputDialog
)
from PyQt6.QtCore import QThread, pyqtSignal, QTimer, Qt
from PyQt6.QtGui import QShortcut, QKeySequence, QFont, QColor
import pyqtgraph as pg
import pyqtgraph.exporters

# Import core modules (hỗ trợ cả rr_app và ppg_app)
try:
    from rr_app.core.serial_worker import SerialThread, LiveBreathRate
    from rr_app.core.dsp_filters import calculate_frequency_response, apply_pipeline_filters
except ImportError:
    from ppg_app.core.serial_worker import SerialThread, LiveBreathRate
    from ppg_app.core.dsp_filters import calculate_frequency_response, apply_pipeline_filters


class RRAnalyzerSuite(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RR-Laser & PPG Signal Suite - Advanced Analyzer")
        self.resize(1280, 850)

        # Style PyQtGraph nền trắng, viền rõ nét
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')
        pg.setConfigOptions(antialias=True)

        # --- KHỞI TẠO LUỒNG SERIAL ---
        self.serial_thread = SerialThread()
        self.serial_thread.new_data.connect(self.on_new_live_data)
        self.serial_thread.diagnostic_msg.connect(self.on_diagnostic_msg)
        self.serial_thread.error.connect(self.on_serial_error)
        self.serial_thread.connection_status.connect(self.on_connection_status)

        # Buffer cho Live Plotting
        self.max_samples = 1000
        self.live_count = 0            # số mẫu thật đã nhận trong phiên hiện tại
        self.live_total = 0            # tổng số mẫu đã nhận (trục X chạy 0 -> vo han)
        self.live_buf_r = np.zeros(self.max_samples)
        self.live_buf_va = np.zeros(self.max_samples)
        self.live_buf_vb = np.zeros(self.max_samples)
        self.live_buf_time = np.zeros(self.max_samples)

        # Trạng thái dữ liệu ngoại tuyến (Offline)
        self.offline_df = None
        self.offline_signal = None
        self.pure_offline_signal = None
        self.offline_time = None
        self.offline_fs = 47.6  # Tần số lấy mẫu mặc định của RR-Laser (~45-55 Hz)
        self.offline_filename = ""
        self.offline_channel = "r"

        # Thước đo tạm (Rulers & Markers)
        self.temp_rulers = []
        self.redo_rulers = []
        self.peaks_markers = []

        # Pipeline bộ lọc DSP
        self.filter_pipeline = []
        self.current_filter_idx = -1
        self._is_populating = False
        self.filtered_signal_cache = None

        # Khởi tạo giao diện
        self.init_ui()

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)

        self.tab_live = QWidget()
        self.tab_offline = QWidget()
        self.tab_filter = QWidget()
        self.tab_wavelet = QWidget()

        self.tabs.addTab(self.tab_live, "1. Live Plotting (Trực tiếp)")
        self.tabs.addTab(self.tab_offline, "2. Offline Analysis (Phân tích CSV)")
        self.tabs.addTab(self.tab_filter, "3. Bộ lọc số (Filter Pipeline)")
        self.tabs.addTab(self.tab_wavelet, "4. Phân tích Sóng (Wavelet DWT)")

        self.setup_live_tab()
        self.setup_offline_tab()
        self.setup_filter_tab()
        self.setup_wavelet_tab()

        # Phím tắt Hoàn tác / Làm lại thước đo
        self.shortcut_undo = QShortcut(QKeySequence("Ctrl+Z"), self)
        self.shortcut_undo.activated.connect(self.undo_temp_ruler)
        self.shortcut_redo = QShortcut(QKeySequence("Ctrl+Y"), self)
        self.shortcut_redo.activated.connect(self.redo_temp_ruler)
        self.shortcut_dot = QShortcut(QKeySequence("Ctrl+D"), self)
        self.shortcut_dot.activated.connect(self.toggle_dot_mode)

    # =========================================================================
    # TAB 1: LIVE PLOTTING & SERIAL ACQUISITION
    # =========================================================================
    def setup_live_tab(self):
        layout = QVBoxLayout(self.tab_live)

        # --- Bảng điều khiển kết nối ---
        ctrl_box = QGroupBox("Điều khiển Kết nối & Phần cứng")
        ctrl_layout = QVBoxLayout(ctrl_box)

        row1 = QHBoxLayout()
        self.cb_ports = QComboBox()
        self.cb_ports.setMinimumWidth(220)
        self.update_ports()

        self.btn_refresh = QPushButton("Làm mới cổng")
        self.btn_refresh.clicked.connect(self.update_ports)

        row1.addWidget(QLabel("Cổng COM:"))
        row1.addWidget(self.cb_ports)
        row1.addWidget(self.btn_refresh)

        row1.addWidget(QLabel("Chế độ:"))
        self.cb_live_mode = QComboBox()
        self.cb_live_mode.addItems([
            "Mode 1: Laser liên tục (~50 Hz)",
            "Mode 0: 2 pha trừ tối (~8 Hz)"
        ])
        row1.addWidget(self.cb_live_mode)

        self.btn_connect = QPushButton("Kết nối & Thu thập")
        self.btn_connect.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 6px;")
        self.btn_connect.clicked.connect(self.toggle_connection)
        row1.addWidget(self.btn_connect)

        self.cb_auto_record = QCheckBox("Lưu CSV vào rrdata/")
        self.cb_auto_record.setChecked(True)
        row1.addWidget(self.cb_auto_record)

        row1.addStretch()
        ctrl_layout.addLayout(row1)

        # Hàng nút lệnh tương tác với firmware
        row2 = QHBoxLayout()
        self.btn_cal_k = QPushButton("Hiệu chuẩn k (c)")
        self.btn_cal_k.setToolTip(
            "Đo 3 giây tại vị trí tia hiện tại rồi chọn k sao cho r dao động quanh 0.\n"
            "Giữ yên và thở đều trong lúc hiệu chuẩn.\n"
            "k được lưu vào bộ nhớ flash, giữ nguyên qua lần khởi động lại."
        )
        self.btn_cal_k.clicked.connect(self.on_calibrate_k)

        self.btn_toggle_laser = QPushButton("Bật/Tắt Laser (l)")
        self.btn_toggle_laser.clicked.connect(lambda: self.send_serial_cmd('l'))

        self.btn_reset_k = QPushButton("Đặt lại k=1 (r)")
        self.btn_reset_k.clicked.connect(lambda: self.send_serial_cmd('r'))

        self.btn_diag = QPushButton("Chẩn đoán (d)")
        self.btn_diag.clicked.connect(lambda: self.send_serial_cmd('d'))

        self.btn_scan_i2c = QPushButton("Quét I2C (s)")
        self.btn_scan_i2c.clicked.connect(lambda: self.send_serial_cmd('s'))

        row2.addWidget(QLabel("Lệnh nhanh:"))
        row2.addWidget(self.btn_cal_k)
        row2.addWidget(self.btn_toggle_laser)
        row2.addWidget(self.btn_reset_k)
        row2.addWidget(self.btn_diag)
        row2.addWidget(self.btn_scan_i2c)

        row2.addWidget(QLabel(" | Khung hiển thị:"))
        self.spin_window = QSpinBox()
        self.spin_window.setRange(100, 5000)
        self.spin_window.setValue(1000)
        self.spin_window.setSingleStep(100)
        self.spin_window.setSuffix(" mẫu")
        self.spin_window.valueChanged.connect(self.change_window_size)
        row2.addWidget(self.spin_window)

        self.cb_invert_live = QCheckBox("Lật ngược")
        row2.addWidget(self.cb_invert_live)

        self.cb_show_vavb = QCheckBox("Hiện Va / Vb")
        self.cb_show_vavb.setChecked(True)
        self.cb_show_vavb.stateChanged.connect(self.on_toggle_live_vavb)
        row2.addWidget(self.cb_show_vavb)

        self.btn_reset_plot = QPushButton(" Làm mới đồ thị")
        self.btn_reset_plot.setStyleSheet(
            "background-color: #FF7043; color: white; font-weight: bold; padding: 5px 10px; border-radius: 4px;"
        )
        self.btn_reset_plot.setToolTip("Xóa sạch đồ thị và buffer để bắt đầu phiên đo mới")
        self.btn_reset_plot.clicked.connect(self.reset_live_plot)
        row2.addWidget(self.btn_reset_plot)

        row2.addStretch()
        ctrl_layout.addLayout(row2)
        layout.addWidget(ctrl_box)

        # --- Thanh thẻ số đo thời gian thực (Status Cards) ---
        metric_layout = QHBoxLayout()
        self.lbl_card_r = QLabel("r (Vị trí): --")
        self.lbl_card_r.setStyleSheet("font-size: 14px; font-weight: bold; color: #D81B60; background: #FCE4EC; padding: 6px 12px; border-radius: 4px;")

        self.lbl_card_rr = QLabel("Nhịp thở: -- bpm")
        self.lbl_card_rr.setStyleSheet("font-size: 14px; font-weight: bold; color: #1E88E5; background: #E3F2FD; padding: 6px 12px; border-radius: 4px;")

        self.lbl_card_va = QLabel("Va: -- mV")
        self.lbl_card_va.setStyleSheet("font-size: 13px; color: #0D47A1; background: #E8EAF6; padding: 6px 10px; border-radius: 4px;")

        self.lbl_card_vb = QLabel("Vb: -- mV")
        self.lbl_card_vb.setStyleSheet("font-size: 13px; color: #1B5E20; background: #E8F5E9; padding: 6px 10px; border-radius: 4px;")

        self.lbl_card_flags = QLabel("Trạng thái: Sẵn sàng")
        self.lbl_card_flags.setStyleSheet("font-size: 13px; color: #37474F; background: #ECEFF1; padding: 6px 10px; border-radius: 4px;")

        self.lbl_save_status = QLabel("File: Chưa ghi")
        self.lbl_save_status.setStyleSheet("font-size: 12px; color: #555; padding: 6px;")

        metric_layout.addWidget(self.lbl_card_r)
        metric_layout.addWidget(self.lbl_card_rr)
        metric_layout.addWidget(self.lbl_card_va)
        metric_layout.addWidget(self.lbl_card_vb)
        metric_layout.addWidget(self.lbl_card_flags)
        metric_layout.addWidget(self.lbl_save_status)
        metric_layout.addStretch()
        layout.addLayout(metric_layout)

        # --- Khu vực đồ thị trực tiếp (Graphics Splitter) ---
        splitter_live = QSplitter(Qt.Orientation.Vertical)

        # Đồ thị r (tỉ số vị trí / nhịp thở)
        self.live_plot_r = pg.PlotWidget(title="Tín hiệu Nhịp thở r = (k·Vb - Va)/(Va + k·Vb)   [hít vào ↑ · thở ra ↓]")
        self.live_plot_r.showGrid(x=True, y=True, alpha=0.3)
        self.live_plot_r.setLabel('bottom', "Mẫu (Samples)")
        self.live_plot_r.setLabel('left', "Tỉ số r")
        self.live_plot_r.enableAutoRange(axis='y')
        # Trục X luôn tự bám theo số mẫu thật (0 -> vô hạn, cuộn theo khung hiển thị).
        # Nếu để autoscale mặc định, TextItem ở gốc toạ độ sẽ ghim khung nhìn ở 0..1000.
        self.live_plot_r.setXRange(0, self.max_samples, padding=0)
        # r có miền xác định cố định [-1, 1] -> khoá trục Y, KHÔNG để autoscale.
        # Autoscale trước đây bị label_live/vLine_live kéo theo toạ độ chuột, đẩy
        # trục Y lên hàng trăm, làm đường r (biên độ ~0.02) trông như đường phẳng.
        self.live_plot_r.disableAutoRange(axis='y')
        self.live_plot_r.setYRange(-1.0, 1.0, padding=0)
        self.curve_live_r = self.live_plot_r.plot(pen=pg.mkPen('#ff69b4', width=2.0), name="r")
        self.zero_line_r = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('#ff69b4', width=1, style=Qt.PenStyle.DashLine))
        self.live_plot_r.addItem(self.zero_line_r)

        # Crosshair cho đồ thị r
        self.vLine_live = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen('gray', style=Qt.PenStyle.DashLine))
        self.live_plot_r.addItem(self.vLine_live, ignoreBounds=True)
        self.label_live = pg.TextItem(anchor=(0, 1), color='k', fill=pg.mkBrush(255, 255, 255, 200))
        self.live_plot_r.addItem(self.label_live, ignoreBounds=True)
        self.proxy_live = pg.SignalProxy(self.live_plot_r.scene().sigMouseMoved, rateLimit=60, slot=self.mouseMoved_live)

        splitter_live.addWidget(self.live_plot_r)

        # Đồ thị Va và Vb
        self.live_plot_vavb = pg.PlotWidget(title="Điện áp cảm biến quang AIN0 (Va) & AIN1 (Vb)")
        self.live_plot_vavb.showGrid(x=True, y=True, alpha=0.3)
        self.live_plot_vavb.setLabel('bottom', "Mẫu (Samples)")
        self.live_plot_vavb.setLabel('left', "Điện áp (mV)")
        self.live_plot_vavb.addLegend()
        self.curve_live_va = self.live_plot_vavb.plot(pen=pg.mkPen('#1976D2', width=1.5), name="Va (AIN0)")
        self.curve_live_vb = self.live_plot_vavb.plot(pen=pg.mkPen('#388E3C', width=1.5), name="Vb (AIN1)")
        self.live_plot_vavb.setXLink(self.live_plot_r)
        splitter_live.addWidget(self.live_plot_vavb)

        splitter_live.setSizes([500, 250])
        layout.addWidget(splitter_live)

        # Nhật ký phản hồi từ bo vi điều khiển
        self.live_log_text = QTextEdit()
        self.live_log_text.setReadOnly(True)
        self.live_log_text.setMaximumHeight(70)
        self.live_log_text.setPlaceholderText("Thông báo phản hồi từ ESP32 (# CAL, # DIAG, # I2C)...")
        layout.addWidget(self.live_log_text)

        # Timer cập nhật đồ thị mượt mà (30 FPS)
        self.live_timer = QTimer()
        self.live_timer.timeout.connect(self.update_live_plot)
        self.live_timer.start(33)

    def on_toggle_live_vavb(self, state):
        if self.cb_show_vavb.isChecked():
            self.live_plot_vavb.show()
        else:
            self.live_plot_vavb.hide()

    def update_ports(self):
        self.cb_ports.clear()
        ports = SerialThread.get_available_ports()
        if not ports:
            self.cb_ports.addItem("Không tìm thấy cổng COM")
            return
        for dev, desc, score in ports:
            mark = "★ " if score >= 3 else ""
            self.cb_ports.addItem(f"{mark}{dev} ({desc})", dev)

    def toggle_connection(self):
        if self.serial_thread.is_running:
            self.serial_thread.stop()
            self.btn_connect.setText("Kết nối & Thu thập")
            self.btn_connect.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 6px;")
            self.cb_ports.setEnabled(True)
            self.cb_live_mode.setEnabled(True)
        else:
            port_data = self.cb_ports.currentData()
            if not port_data:
                port_text = self.cb_ports.currentText()
                if "COM" in port_text:
                    port_data = port_text.split(" ")[0].replace("★", "").strip()
                else:
                    QMessageBox.warning(self, "Lỗi cổng", "Vui lòng chọn cổng COM hợp lệ.")
                    return

            self.serial_thread.port = port_data
            self.serial_thread.mode = "1" if "Mode 1" in self.cb_live_mode.currentText() else "0"
            self.serial_thread.auto_record = self.cb_auto_record.isChecked()
            self.serial_thread.start()

            self.btn_connect.setText("Dừng thu (Disconnect)")
            self.btn_connect.setStyleSheet("background-color: #f44336; color: white; font-weight: bold; padding: 6px;")
            self.cb_ports.setEnabled(False)
            self.cb_live_mode.setEnabled(False)

    def on_connection_status(self, connected, msg):
        if connected:
            self.lbl_card_flags.setText(f"Trạng thái: {msg}")
            if self.serial_thread.save_path:
                self.lbl_save_status.setText(f"Ghi: {os.path.basename(self.serial_thread.save_path)}")
        else:
            self.lbl_card_flags.setText("Trạng thái: Đã ngắt kết nối")

    def send_serial_cmd(self, cmd):
        if not self.serial_thread.is_running:
            QMessageBox.information(self, "Chưa kết nối", "Vui lòng kết nối với thiết bị trước.")
            return
        self.serial_thread.send_command(cmd)

    def on_calibrate_k(self):
        """Hiệu chuẩn k. Firmware đo 3 giây nên chặn luồng đọc một lúc;
        hiện trạng thái để người dùng biết đang chạy chứ không tưởng bị treo."""
        if not self.serial_thread.is_running:
            QMessageBox.information(self, "Chưa kết nối",
                                    "Vui lòng kết nối với thiết bị trước khi hiệu chuẩn.")
            return
        self.btn_cal_k.setEnabled(False)
        self.btn_cal_k.setText("Đang hiệu chuẩn (3s)...")
        self.lbl_card_flags.setText("Trạng thái: đang hiệu chuẩn k, giữ yên và thở đều")
        self.log_diag("[GUI] bắt đầu hiệu chuẩn k — đo 3 giây tại vị trí tia hiện tại")
        self.serial_thread.send_command('c')

        def _done():
            self.btn_cal_k.setEnabled(True)
            self.btn_cal_k.setText("Hiệu chuẩn k (c)")
            self.lbl_card_flags.setText("Trạng thái: hiệu chuẩn xong")

        QTimer.singleShot(4200, _done)

    def log_diag(self, msg):
        """Ghi một dòng vào nhật ký phản hồi firmware (nếu đã dựng xong)."""
        if getattr(self, 'live_log_text', None) is not None:
            self.live_log_text.append(msg)

    def change_window_size(self, size):
        self.max_samples = size
        self.live_count = 0
        self.live_total = 0
        self.live_buf_r = np.zeros(self.max_samples)
        self.live_buf_va = np.zeros(self.max_samples)
        self.live_buf_vb = np.zeros(self.max_samples)
        self.live_buf_time = np.zeros(self.max_samples)
        self.live_plot_r.setXRange(0, self.max_samples, padding=0)

    def on_new_live_data(self, d):
        r = d['r']
        if self.cb_invert_live.isChecked():
            r = -r

        # Đẩy mẫu mới vào cuối buffer kiểu FIFO trượt. live_count đếm số mẫu thật
        # để lúc khởi động chưa có mẫu vẫn không vẽ dãy 0 phẳng trước dữ liệu thật.
        self.live_buf_r[:-1] = self.live_buf_r[1:]
        self.live_buf_r[-1] = r

        self.live_buf_va[:-1] = self.live_buf_va[1:]
        self.live_buf_va[-1] = d['va']

        self.live_buf_vb[:-1] = self.live_buf_vb[1:]
        self.live_buf_vb[-1] = d['vb']

        self.live_count = min(self.live_count + 1, self.max_samples)
        self.live_total += 1

        self.lbl_card_r.setText(f"r: {r:+.4f}")
        self.lbl_card_rr.setText(f"Nhịp thở: {d['rr']:.1f} bpm" if d['rr'] > 0 else "Nhịp thở: Đang tính...")
        self.lbl_card_va.setText(f"Va: {d['va']:.0f} mV")
        self.lbl_card_vb.setText(f"Vb: {d['vb']:.0f} mV")

        flags = d['flags']
        if flags == 0:
            self.lbl_card_flags.setText("Cờ: 0x00 (Bình thường)")
        else:
            errs = []
            if flags & 1: errs.append("Lỗi ADS")
            if flags & 2: errs.append("Bão hòa")
            if flags & 4: errs.append("Vbias mặc định")
            if flags & 8: errs.append("Pin yếu")
            self.lbl_card_flags.setText(f"Cờ: 0x{flags:02X} ({', '.join(errs)})")

    def on_diagnostic_msg(self, msg):
        self.live_log_text.append(msg)
        # Tự động cuộn xuống dòng mới nhất
        self.live_log_text.moveCursor(self.live_log_text.textCursor().MoveOperation.End)

    def on_serial_error(self, err_msg):
        self.toggle_connection()
        QMessageBox.critical(self, "Lỗi Serial", f"Lỗi cổng kết nối: {err_msg}")

    def update_live_plot(self):
        if self.serial_thread.is_running:
            # Chỉ vẽ phần mẫu thật đã nhận (tránh dãy 0 flat lúc mới khởi động)
            n = self.live_count
            if n < 2:
                return
            # Trục X luôn bắt đầu từ 0 và tăng mãi: hiển thị theo số mẫu tổng
            # [total-n, total]. Khi total < max_samples thì thấy [0, total].
            total = self.live_total
            x = np.arange(total - n, total)
            self.curve_live_r.setData(x, self.live_buf_r[-n:])
            if self.cb_show_vavb.isChecked():
                self.curve_live_va.setData(x, self.live_buf_va[-n:])
                self.curve_live_vb.setData(x, self.live_buf_vb[-n:])
            # Khung nhìn trượt theo cửa sổ hiển thị, không còn vùng trống bên phải
            self.live_plot_r.setXRange(max(0, total - self.max_samples), total, padding=0)

    def mouseMoved_live(self, evt):
        pos = evt[0]
        if self.live_plot_r.sceneBoundingRect().contains(pos):
            mousePoint = self.live_plot_r.plotItem.vb.mapSceneToView(pos)
            x = mousePoint.x()
            # Đọc r tại đúng mẫu dưới con trỏ, KHÔNG lấy toạ độ Y của chuột:
            # nếu lấy Y thì nhãn trôi tới chỗ trống và (khi bật autoscale) kéo
            # cả khung nhìn theo, làm line r trông phẳng.
            n = self.live_count
            total = self.live_total
            y = None
            if n > 0:
                i = int(x) - (total - n)
                if 0 <= i < n:
                    y = float(self.live_buf_r[-n:][i])
            self.vLine_live.setPos(x)
            if y is None:
                self.label_live.setHtml("Mẫu: %d<br>r: --" % int(x))
            else:
                self.label_live.setHtml("Mẫu: %d<br>r: %+.4f" % (int(x), y))
            # Neo nhãn cao hơn đỉnh trục một chút để không chen vào đường r
            self.label_live.setPos(x, self.live_plot_r.viewRange()[1][1] * 0.88)

    # =========================================================================
    # TAB 2: OFFLINE ANALYSIS (PHÂN TÍCH CSV ĐỒNG BỘ RRDATA)
    # =========================================================================
    def setup_offline_tab(self):
        layout = QVBoxLayout(self.tab_offline)

        # Thanh điều khiển tải và chọn dữ liệu
        ctrl_bar = QHBoxLayout()

        self.btn_load_csv = QPushButton("1. Chọn file CSV")
        self.btn_load_csv.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; padding: 5px 12px;")
        self.btn_load_csv.clicked.connect(self.load_csv)
        ctrl_bar.addWidget(self.btn_load_csv)

        ctrl_bar.addWidget(QLabel("Kênh phân tích:"))
        self.cb_channel = QComboBox()
        self.cb_channel.setMinimumWidth(150)
        self.cb_channel.currentIndexChanged.connect(self.on_channel_changed)
        ctrl_bar.addWidget(self.cb_channel)

        ctrl_bar.addWidget(QLabel("Trục hoành:"))
        self.cb_x_axis = QComboBox()
        self.cb_x_axis.addItems(["Thời gian (Giây)", "Số mẫu (Samples)"])
        self.cb_x_axis.currentIndexChanged.connect(self.redraw_offline_raw)
        ctrl_bar.addWidget(self.cb_x_axis)

        ctrl_bar.addWidget(QLabel("Fs (Hz):"))
        self.spin_fs = QDoubleSpinBox()
        self.spin_fs.setRange(1.0, 2000.0)
        self.spin_fs.setValue(47.6)
        self.spin_fs.setDecimals(1)
        self.spin_fs.valueChanged.connect(self.on_fs_changed)
        ctrl_bar.addWidget(self.spin_fs)

        self.cb_invert_offline = QCheckBox("Lật ngược")
        self.cb_invert_offline.stateChanged.connect(self.on_invert_changed)
        ctrl_bar.addWidget(self.cb_invert_offline)

        self.cb_signal_type = QComboBox()
        self.cb_signal_type.addItems(["Dải Nhịp thở (RR: 0.1-0.5 Hz)", "Dải Nhịp tim (PPG: 0.8-3.0 Hz)"])
        self.cb_signal_type.currentIndexChanged.connect(self.update_fft_zones)
        ctrl_bar.addWidget(self.cb_signal_type)

        self.btn_analyze = QPushButton("2. Phân tích (FFT & Đạo hàm)")
        self.btn_analyze.setEnabled(False)
        self.btn_analyze.clicked.connect(self.analyze_data)
        ctrl_bar.addWidget(self.btn_analyze)

        self.btn_find_peaks = QPushButton("3. Tìm Đỉnh & Nhịp")
        self.btn_find_peaks.setEnabled(False)
        self.btn_find_peaks.setStyleSheet("background-color: #FF9800; color: white; font-weight: bold; padding: 5px 12px;")
        self.btn_find_peaks.clicked.connect(self.find_peaks_rate)
        ctrl_bar.addWidget(self.btn_find_peaks)

        ctrl_bar.addStretch()

        self.btn_export_fft = QPushButton("Lưu ảnh FFT")
        self.btn_export_fft.clicked.connect(self.export_fft)
        ctrl_bar.addWidget(self.btn_export_fft)

        self.btn_export_deriv = QPushButton("Lưu ảnh Đạo hàm")
        self.btn_export_deriv.clicked.connect(self.export_deriv_plot)
        ctrl_bar.addWidget(self.btn_export_deriv)

        layout.addLayout(ctrl_bar)

        # Sub-tabs: Fourier vs Đạo hàm
        self.offline_subtabs = QTabWidget()

        # --- SUB-TAB 1: TIME DOMAIN & FFT ---
        self.sub_tab_fft = QWidget()
        layout_fft = QVBoxLayout(self.sub_tab_fft)
        splitter_fft = QSplitter(Qt.Orientation.Vertical)

        self.fft_time_plot = pg.PlotWidget(title="Tín hiệu Miền thời gian (Time Domain)")
        self.fft_time_plot.showGrid(x=True, y=True, alpha=0.3)
        self.fft_time_plot.setLabel('bottom', "Thời gian (s)")
        self.fft_time_plot.setLabel('left', "Biên độ")
        splitter_fft.addWidget(self.fft_time_plot)

        self.fft_widget = pg.PlotWidget(title="Phân tích Phổ Tần số (FFT Spectrum)")
        self.fft_widget.showGrid(x=True, y=True, alpha=0.3)
        self.fft_widget.setLabel('bottom', "Tần số (Hz)")
        self.fft_widget.setLabel('left', "Biên độ (Magnitude)")
        self.fft_widget.addLegend()

        # Dải màu sinh lý học
        self.lr_zone1 = pg.LinearRegionItem([0, 0.1], brush=(255, 0, 0, 30), movable=False)
        self.lr_zone2 = pg.LinearRegionItem([0.1, 0.5], brush=(0, 255, 0, 30), movable=False)
        self.lr_zone3 = pg.LinearRegionItem([0.5, 2.5], brush=(255, 165, 0, 30), movable=False)
        self.fft_widget.addItem(self.lr_zone1)
        self.fft_widget.addItem(self.lr_zone2)
        self.fft_widget.addItem(self.lr_zone3)

        self.dummy_zone1 = self.fft_widget.plot([], [], pen=pg.mkPen(color=(255, 100, 100), width=3), name="Trôi nền Baseline (<0.1Hz)")
        self.dummy_zone2 = self.fft_widget.plot([], [], pen=pg.mkPen(color=(100, 255, 100), width=3), name="Dải Nhịp thở chính (0.1-0.5Hz)")
        self.dummy_zone3 = self.fft_widget.plot([], [], pen=pg.mkPen(color=(255, 200, 100), width=3), name="Hài / Nhiễu (>0.5Hz)")

        self.fft_curve = self.fft_widget.plot([], [], pen=pg.mkPen('#1E88E5', width=1.8))
        splitter_fft.addWidget(self.fft_widget)
        splitter_fft.setSizes([400, 450])
        layout_fft.addWidget(splitter_fft)

        # Crosshairs cho FFT
        self.vLine_fft_time = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen('gray', style=Qt.PenStyle.DashLine))
        self.vLine_fft = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen('gray', style=Qt.PenStyle.DashLine))
        self.fft_time_plot.addItem(self.vLine_fft_time, ignoreBounds=True)
        self.fft_widget.addItem(self.vLine_fft, ignoreBounds=True)

        self.label_fft_time = pg.TextItem(anchor=(0, 1), color='k', fill=pg.mkBrush(255, 255, 255, 200))
        self.label_fft = pg.TextItem(anchor=(0, 1), color='k', fill=pg.mkBrush(255, 255, 255, 200))
        self.fft_time_plot.addItem(self.label_fft_time, ignoreBounds=True)
        self.fft_widget.addItem(self.label_fft, ignoreBounds=True)

        self.proxy_fft_time = pg.SignalProxy(self.fft_time_plot.scene().sigMouseMoved, rateLimit=60, slot=self.mouseMoved_fft_time)
        self.proxy_fft_widget = pg.SignalProxy(self.fft_widget.scene().sigMouseMoved, rateLimit=60, slot=self.mouseMoved_fft_widget)

        # --- SUB-TAB 2: TIME DOMAIN & ĐẠO HÀM (VPG / APG) ---
        self.sub_tab_deriv = QWidget()
        layout_deriv = QVBoxLayout(self.sub_tab_deriv)

        deriv_tools = QHBoxLayout()
        self.cb_show_vpg = QCheckBox("Hiển thị VPG (Bậc 1)")
        self.cb_show_vpg.setChecked(True)
        self.cb_show_vpg.stateChanged.connect(self.toggle_vpg)

        self.cb_show_apg = QCheckBox("Hiển thị APG (Bậc 2)")
        self.cb_show_apg.setChecked(True)
        self.cb_show_apg.stateChanged.connect(self.toggle_apg)

        self.cb_dot_mode = QCheckBox("Chấm đỏ (Ctrl+D)")
        self.cb_dot_mode.setChecked(False)

        self.btn_clear_rulers = QPushButton("Xóa các đánh dấu tạm")
        self.btn_clear_rulers.clicked.connect(self.clear_temp_rulers)

        deriv_tools.addWidget(self.cb_show_vpg)
        deriv_tools.addWidget(self.cb_show_apg)
        deriv_tools.addWidget(self.cb_dot_mode)
        deriv_tools.addStretch()
        deriv_tools.addWidget(self.btn_clear_rulers)
        layout_deriv.addLayout(deriv_tools)

        self.deriv_layout_widget = pg.GraphicsLayoutWidget()

        self.deriv_raw_plot = self.deriv_layout_widget.addPlot(title="Tín hiệu Gốc (Time Domain)")
        self.deriv_raw_plot.showGrid(x=True, y=True, alpha=0.3)
        self.deriv_raw_plot.setLabel('left', "Biên độ")

        self.deriv_layout_widget.nextRow()
        self.vpg_plot = self.deriv_layout_widget.addPlot(title="VPG (Đạo hàm Bậc 1: ds/dt)")
        self.vpg_plot.showGrid(x=True, y=True, alpha=0.3)
        self.vpg_plot.setLabel('left', "VPG")
        self.vpg_plot.setXLink(self.deriv_raw_plot)

        self.deriv_layout_widget.nextRow()
        self.apg_plot = self.deriv_layout_widget.addPlot(title="APG (Đạo hàm Bậc 2: d2s/dt2)")
        self.apg_plot.showGrid(x=True, y=True, alpha=0.3)
        self.apg_plot.setLabel('bottom', "Thời gian (s)")
        self.apg_plot.setLabel('left', "APG")
        self.apg_plot.setXLink(self.deriv_raw_plot)

        layout_deriv.addWidget(self.deriv_layout_widget)

        # Crosshairs cho Đạo hàm
        self.vLine_deriv_raw = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen('gray', style=Qt.PenStyle.DashLine))
        self.vLine_vpg = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen('gray', style=Qt.PenStyle.DashLine))
        self.vLine_apg = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen('gray', style=Qt.PenStyle.DashLine))

        self.deriv_raw_plot.addItem(self.vLine_deriv_raw, ignoreBounds=True)
        self.vpg_plot.addItem(self.vLine_vpg, ignoreBounds=True)
        self.apg_plot.addItem(self.vLine_apg, ignoreBounds=True)

        self.label_deriv_raw = pg.TextItem(anchor=(0, 1), color='k', fill=pg.mkBrush(255, 255, 255, 200))
        self.label_vpg = pg.TextItem(anchor=(0, 1), color='k', fill=pg.mkBrush(255, 255, 255, 200))
        self.label_apg = pg.TextItem(anchor=(0, 1), color='k', fill=pg.mkBrush(255, 255, 255, 200))

        self.deriv_raw_plot.addItem(self.label_deriv_raw, ignoreBounds=True)
        self.vpg_plot.addItem(self.label_vpg, ignoreBounds=True)
        self.apg_plot.addItem(self.label_apg, ignoreBounds=True)

        self.proxy_deriv = pg.SignalProxy(self.deriv_layout_widget.scene().sigMouseMoved, rateLimit=60, slot=self.mouseMoved_deriv)
        self.deriv_layout_widget.scene().sigMouseClicked.connect(self.mouseClicked_deriv)

        self.offline_subtabs.addTab(self.sub_tab_fft, "1. Miền thời gian & Phổ Fourier (FFT)")
        self.offline_subtabs.addTab(self.sub_tab_deriv, "2. Miền thời gian & Đạo hàm (VPG/APG)")
        layout.addWidget(self.offline_subtabs)

    def update_fft_zones(self):
        is_rr = "Nhịp thở" in self.cb_signal_type.currentText()
        if is_rr:
            self.lr_zone1.setRegion([0, 0.1])
            self.lr_zone2.setRegion([0.1, 0.5])
            self.lr_zone3.setRegion([0.5, 2.5])
            self.fft_widget.plotItem.legend.items[0][1].setText("Trôi nền Baseline (<0.1Hz)")
            self.fft_widget.plotItem.legend.items[1][1].setText("Dải Nhịp thở chính (0.1-0.5Hz)")
            self.fft_widget.plotItem.legend.items[2][1].setText("Hài / Nhiễu (>0.5Hz)")
            self.fft_widget.setXRange(0, min(5.0, self.offline_fs / 2.0))
        else:
            self.lr_zone1.setRegion([0, 0.5])
            self.lr_zone2.setRegion([0.8, 3.0])
            self.lr_zone3.setRegion([3.0, 10.0])
            self.fft_widget.plotItem.legend.items[0][1].setText("Trôi nền Baseline (<0.5Hz)")
            self.fft_widget.plotItem.legend.items[1][1].setText("Dải Nhịp tim (0.8-3.0Hz)")
            self.fft_widget.plotItem.legend.items[2][1].setText("Đặc trưng / Hài (>3.0Hz)")
            self.fft_widget.setXRange(0, min(12.0, self.offline_fs / 2.0))

    def load_csv(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        # Mặc định mở thư mục rrdata (nằm cùng cấp với thư mục Code hoặc con của NCKH)
        target_dir = os.path.abspath(os.path.join(current_dir, "..", "rrdata"))
        if not os.path.exists(target_dir):
            target_dir = os.path.abspath(os.path.join(current_dir, "rrdata"))
        if not os.path.exists(target_dir):
            target_dir = current_dir

        file_path, _ = QFileDialog.getOpenFileName(self, "Chọn file CSV", target_dir, "CSV Files (*.csv)")
        if not file_path:
            return

        try:
            df = pd.read_csv(file_path)
            # Chuẩn hóa tên cột viết hoa / thường
            cols_map = {col.lower(): col for col in df.columns}

            # Tự động tính tần số lấy mẫu Fs
            computed_fs = 47.6
            if 't_ms' in cols_map:
                col_tms = cols_map['t_ms']
                diffs = df[col_tms].diff().dropna()
                diffs = diffs[diffs > 0]
                if len(diffs) > 0:
                    med_dt_ms = diffs.median()
                    if med_dt_ms > 0:
                        computed_fs = round(1000.0 / med_dt_ms, 1)
            elif 'host_s' in cols_map:
                col_hosts = cols_map['host_s']
                diffs = df[col_hosts].diff().dropna()
                diffs = diffs[diffs > 0]
                if len(diffs) > 0:
                    med_dt_s = diffs.median()
                    if med_dt_s > 0:
                        computed_fs = round(1.0 / med_dt_s, 1)

            self.offline_fs = computed_fs
            self.spin_fs.blockSignals(True)
            self.spin_fs.setValue(self.offline_fs)
            self.spin_fs.blockSignals(False)

            self.offline_df = df
            self.offline_filename = os.path.basename(file_path)

            # Cập nhật danh sách các kênh khả dụng vào combobox
            self.cb_channel.blockSignals(True)
            self.cb_channel.clear()

            # Các cột ưu tiên của RR-Laser
            priority_channels = ['r', 'Va', 'Vb', 'va', 'vb', 'rr', 'Va0', 'Vb0', 'Gia_tri_PPG', 'Raw_IR', 'Original', 'Filtered']
            added = set()

            # Thêm các cột ưu tiên nếu có trong file
            for pc in priority_channels:
                for c in df.columns:
                    if c.lower() == pc.lower() and c not in added:
                        self.cb_channel.addItem(c)
                        added.add(c)

            # Thêm cột tổng hợp Vb - Va nếu có cả Va và Vb
            # (Vb lớn hơn Va trên phần lớn dải -> lấy Vb - Va để hiệu quang không âm)
            has_va = any(c.lower() == 'va' for c in df.columns)
            has_vb = any(c.lower() == 'vb' for c in df.columns)
            if has_va and has_vb:
                self.cb_channel.addItem("Vb - Va (Hiệu quang)")

            # Thêm các cột số còn lại
            for c in df.columns:
                if c not in added and np.issubdtype(df[c].dtype, np.number):
                    if c.lower() not in ('host_s', 't_ms', 'flags', 'vbat'):
                        self.cb_channel.addItem(c)
                        added.add(c)

            self.cb_channel.blockSignals(False)

            # Chọn kênh mặc định: ưu tiên 'r', sau đó 'Gia_tri_PPG' hoặc kênh đầu tiên
            idx_r = -1
            for i in range(self.cb_channel.count()):
                if self.cb_channel.itemText(i).lower() == 'r':
                    idx_r = i
                    break
            if idx_r >= 0:
                self.cb_channel.setCurrentIndex(idx_r)
            else:
                self.cb_channel.setCurrentIndex(0)

            # Trích xuất vector thời gian
            if 'host_s' in cols_map:
                self.offline_time = df[cols_map['host_s']].values
            elif 't_ms' in cols_map:
                t_ms_arr = df[cols_map['t_ms']].values
                self.offline_time = (t_ms_arr - t_ms_arr[0]) / 1000.0
            else:
                self.offline_time = np.arange(len(df)) / self.offline_fs

            self.on_channel_changed()

            self.btn_analyze.setEnabled(True)
            self.btn_analyze.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; padding: 5px 12px;")
            self.btn_find_peaks.setEnabled(True)

            # Tự động cập nhật đồ thị Tab Filter nếu đang mở
            if hasattr(self, 'orig_curve'):
                self.orig_curve.setData(self.pure_offline_signal)
                if len(self.filter_pipeline) > 0 and self.cb_auto_preview.isChecked():
                    self.apply_filter(silent=True)

            QMessageBox.information(
                self, "Đã tải file thành công",
                f"File: {self.offline_filename}\n"
                f"Số mẫu: {len(df)}\n"
                f"Fs ước lượng: {self.offline_fs:.1f} Hz\n"
                f"Kênh đang chọn: {self.offline_channel}"
            )

        except Exception as e:
            QMessageBox.critical(self, "Lỗi đọc file CSV", f"Chi tiết lỗi: {str(e)}")

    def on_channel_changed(self):
        if self.offline_df is None:
            return
        channel_name = self.cb_channel.currentText()
        if not channel_name:
            return

        self.offline_channel = channel_name
        df = self.offline_df

        if channel_name == "Vb - Va (Hiệu quang)":
            col_va = next(c for c in df.columns if c.lower() == 'va')
            col_vb = next(c for c in df.columns if c.lower() == 'vb')
            sig = df[col_vb].values.astype(float) - df[col_va].values.astype(float)
        else:
            sig = df[channel_name].values.astype(float)

        self.offline_signal = sig
        self.pure_offline_signal = np.copy(sig)

        # Cập nhật preset dải tần nếu kênh là r (nhịp thở)
        if channel_name.lower() == 'r':
            self.cb_signal_type.setCurrentIndex(0)  # RR
        else:
            self.cb_signal_type.setCurrentIndex(1)  # PPG

        self.redraw_offline_raw()

    def on_fs_changed(self, val):
        self.offline_fs = val
        if self.offline_df is not None:
            self.redraw_offline_raw()

    def on_invert_changed(self):
        if self.offline_signal is not None:
            self.redraw_offline_raw()
            if self.btn_analyze.isEnabled():
                self.analyze_data()

    def redraw_offline_raw(self):
        if self.offline_signal is None:
            return

        self.fft_time_plot.clear()
        self.deriv_raw_plot.clear()
        self.vpg_plot.clear()
        self.apg_plot.clear()
        self.fft_curve.setData([], [])
        self.clear_temp_rulers()

        signal = -self.offline_signal if self.cb_invert_offline.isChecked() else self.offline_signal

        use_seconds = "Giây" in self.cb_x_axis.currentText()
        if use_seconds and self.offline_time is not None and len(self.offline_time) == len(signal):
            x_data = self.offline_time
            x_label = "Thời gian (s)"
        else:
            x_data = np.arange(len(signal))
            x_label = "Số mẫu (Samples)"

        self.fft_time_plot.setLabel('bottom', x_label)
        self.deriv_raw_plot.setLabel('bottom', x_label)
        self.apg_plot.setLabel('bottom', x_label)

        pen_color = '#E91E63' if self.offline_channel.lower() == 'r' else '#673AB7'
        self.fft_time_plot.plot(x_data, signal, pen=pg.mkPen(pen_color, width=1.6))
        self.fft_time_plot.setTitle(f"Tín hiệu [{self.offline_channel}] - {self.offline_filename} ({len(signal)} mẫu, Fs={self.offline_fs:.1f}Hz)")

        self.deriv_raw_plot.plot(x_data, signal, pen=pg.mkPen(pen_color, width=1.6))
        self.deriv_raw_plot.setTitle(f"Tín hiệu Gốc [{self.offline_channel}]")

        self.fft_time_plot.addItem(self.vLine_fft_time, ignoreBounds=True)
        self.fft_time_plot.addItem(self.label_fft_time, ignoreBounds=True)
        self.deriv_raw_plot.addItem(self.vLine_deriv_raw, ignoreBounds=True)
        self.deriv_raw_plot.addItem(self.label_deriv_raw, ignoreBounds=True)

    def analyze_data(self, *args, silent=False):
        if not isinstance(silent, bool):
            silent = False
        if self.offline_signal is None:
            return

        signal = self.offline_signal
        if len(signal) < 20:
            if not silent:
                QMessageBox.warning(self, "Dữ liệu quá ngắn", "Tín hiệu cần ít nhất 20 mẫu để phân tích FFT.")
            return

        signal_no_dc = signal - np.mean(signal)

        # Xác định khoảng chia thời gian dt
        use_seconds = "Giây" in self.cb_x_axis.currentText()
        if use_seconds and self.offline_time is not None and len(self.offline_time) == len(signal):
            x_data = self.offline_time
            diff_t = np.diff(self.offline_time)
            dt = np.median(diff_t[diff_t > 0]) if len(diff_t[diff_t > 0]) > 0 else (1.0 / self.offline_fs)
        else:
            x_data = np.arange(len(signal))
            dt = 1.0 / self.offline_fs

        # 1. Tính toán Đạo hàm Bậc 1 (VPG) và Bậc 2 (APG)
        vpg = np.gradient(signal_no_dc, dt)
        apg = np.gradient(vpg, dt)

        self.vpg_plot.clear()
        self.apg_plot.clear()

        self.vpg_plot.plot(x_data, vpg, pen=pg.mkPen('#2E7D32', width=1.5))
        self.apg_plot.plot(x_data, apg, pen=pg.mkPen('#EF6C00', width=1.5))

        self.vpg_plot.addItem(self.vLine_vpg, ignoreBounds=True)
        self.vpg_plot.addItem(self.label_vpg, ignoreBounds=True)
        self.apg_plot.addItem(self.vLine_apg, ignoreBounds=True)
        self.apg_plot.addItem(self.label_apg, ignoreBounds=True)

        # 2. Tính toán Phổ Fourier (FFT)
        N = len(signal_no_dc)
        Fs = self.offline_fs
        T = 1.0 / Fs

        yf = scipy.fftpack.fft(signal_no_dc)
        xf = np.linspace(0.0, 1.0 / (2.0 * T), N // 2)
        yf_abs = 2.0 / N * np.abs(yf[:N // 2])

        self.fft_curve.setData(xf, yf_abs)
        self.update_fft_zones()

        if not silent:
            QMessageBox.information(self, "Hoàn tất phân tích", "Đã phân tích xong Phổ tần số (FFT) và Đạo hàm (VPG / APG).")

    def find_peaks_rate(self, *args, silent=False):
        """Tìm đỉnh và ước lượng nhịp thở (hoặc nhịp tim) thích ứng với dải tần sinh lý."""
        if not isinstance(silent, bool):
            silent = False
        if self.offline_signal is None:
            if not silent:
                QMessageBox.warning(self, "Chưa có dữ liệu", "Vui lòng chọn file CSV trước.")
            return

        signal = self.offline_signal
        if len(signal) < 30:
            if not silent:
                QMessageBox.warning(self, "Dữ liệu quá ngắn", "Cần ít nhất 30 mẫu để tìm đỉnh.")
            return

        # offline_signal đã mang sẵn chiều: hít vào -> đi lên, thở ra -> đi xuống
        # (redraw_offline_raw()/apply_filter() đã áp dụng "Lật ngược" nếu được tick).
        # KHÔNG đảo lại ở đây, tránh đảo hai lần làm đỉnh bị gắn ngược chỗ.
        signal_no_dc = signal - np.mean(signal)
        Fs = self.offline_fs
        is_rr = "Nhịp thở" in self.cb_signal_type.currentText() or self.offline_channel.lower() == 'r'

        # Tham số tìm đỉnh thích ứng
        if is_rr:
            # Nhịp thở người lớn: 10 - 30 bpm -> chu kỳ 2 - 6 giây
            min_distance = max(5, int(Fs * 1.5))
            prom = max(np.ptp(signal_no_dc) * 0.12, 1e-4)
        else:
            # Nhịp tim: 45 - 180 bpm -> chu kỳ 0.33 - 1.33 giây
            min_distance = max(5, int(Fs * 0.35))
            prom = max(np.ptp(signal_no_dc) * 0.15, 1e-4)

        peaks, _ = find_peaks(signal_no_dc, distance=min_distance, prominence=prom)

        use_seconds = "Giây" in self.cb_x_axis.currentText()
        if use_seconds and self.offline_time is not None and len(self.offline_time) == len(signal):
            x_data = self.offline_time
        else:
            x_data = np.arange(len(signal))

        # Vẽ lại đồ thị
        self.redraw_offline_raw()
        if len(peaks) > 0:
            peak_x = x_data[peaks]
            peak_y = signal[peaks]

            scatter1 = pg.ScatterPlotItem(x=peak_x, y=peak_y, size=9, pen=pg.mkPen(None), brush=pg.mkBrush(255, 0, 0, 255))
            scatter2 = pg.ScatterPlotItem(x=peak_x, y=peak_y, size=9, pen=pg.mkPen(None), brush=pg.mkBrush(255, 0, 0, 255))
            self.fft_time_plot.addItem(scatter1)
            self.deriv_raw_plot.addItem(scatter2)

            for px in peak_x:
                l_pen = pg.mkPen('#FF1744', width=1, style=Qt.PenStyle.DashLine)
                self.deriv_raw_plot.addItem(pg.InfiniteLine(pos=px, angle=90, pen=l_pen))
                self.vpg_plot.addItem(pg.InfiniteLine(pos=px, angle=90, pen=l_pen))
                self.apg_plot.addItem(pg.InfiniteLine(pos=px, angle=90, pen=l_pen))

        rate = 0.0
        if len(peaks) > 1:
            if use_seconds and self.offline_time is not None:
                intervals_s = np.diff(self.offline_time[peaks])
            else:
                intervals_s = np.diff(peaks) / Fs
            med_interval = np.median(intervals_s)
            if med_interval > 0:
                rate = 60.0 / med_interval

        unit_str = "lần/phút (Breaths/min)" if is_rr else "BPM"
        title_tag = f"Đỉnh: {len(peaks)} | {'Nhịp thở' if is_rr else 'Nhịp tim'}: {rate:.1f} {unit_str}"
        self.fft_time_plot.setTitle(f"Tín hiệu [{self.offline_channel}] | {title_tag}")
        self.deriv_raw_plot.setTitle(f"Tín hiệu Gốc [{self.offline_channel}] | {title_tag}")

        if not silent:
            QMessageBox.information(
                self, "Kết quả phát hiện đỉnh",
                f"Tìm thấy: {len(peaks)} đỉnh sóng.\n"
                f"Khoảng cách chu kỳ trung vị: {np.median(np.diff(peaks))/Fs:.2f} s\n"
                f"{'Nhịp thở ước lượng' if is_rr else 'Nhịp tim ước lượng'}: {rate:.1f} {unit_str}"
            )

    def toggle_vpg(self, state):
        if self.cb_show_vpg.isChecked():
            self.vpg_plot.show()
        else:
            self.vpg_plot.hide()

    def toggle_apg(self, state):
        if self.cb_show_apg.isChecked():
            self.apg_plot.show()
        else:
            self.apg_plot.hide()

    def toggle_dot_mode(self):
        self.cb_dot_mode.setChecked(not self.cb_dot_mode.isChecked())

    def mouseMoved_fft_time(self, evt):
        pos = evt[0]
        if self.fft_time_plot.sceneBoundingRect().contains(pos):
            mousePoint = self.fft_time_plot.plotItem.vb.mapSceneToView(pos)
            x, y = mousePoint.x(), mousePoint.y()
            self.vLine_fft_time.setPos(x)
            lbl = f"T: {x:.2f}s" if "Giây" in self.cb_x_axis.currentText() else f"Mẫu: {int(x)}"
            self.label_fft_time.setHtml(f"{lbl}<br>Biên độ: {y:.4f}")
            self.label_fft_time.setPos(x, y)

    def mouseMoved_fft_widget(self, evt):
        pos = evt[0]
        if self.fft_widget.sceneBoundingRect().contains(pos):
            mousePoint = self.fft_widget.plotItem.vb.mapSceneToView(pos)
            x, y = mousePoint.x(), mousePoint.y()
            self.vLine_fft.setPos(x)
            self.label_fft.setHtml(f"Tần số: {x:.2f} Hz<br>Biên độ: {y:.5f}")
            self.label_fft.setPos(x, y)

    def mouseMoved_deriv(self, evt):
        pos = evt[0]
        x = None
        if self.deriv_raw_plot.isVisible() and self.deriv_raw_plot.vb.sceneBoundingRect().contains(pos):
            mousePoint = self.deriv_raw_plot.vb.mapSceneToView(pos)
            x, y = mousePoint.x(), mousePoint.y()
            self.label_deriv_raw.setHtml(f"X: {x:.2f}<br>Tín hiệu: {y:.4f}")
            self.label_deriv_raw.setPos(x, y)
            self.label_deriv_raw.show()
            self.label_vpg.hide()
            self.label_apg.hide()
        elif self.vpg_plot.isVisible() and self.vpg_plot.vb.sceneBoundingRect().contains(pos):
            mousePoint = self.vpg_plot.vb.mapSceneToView(pos)
            x, y = mousePoint.x(), mousePoint.y()
            self.label_vpg.setHtml(f"X: {x:.2f}<br>VPG: {y:.4f}")
            self.label_vpg.setPos(x, y)
            self.label_vpg.show()
            self.label_deriv_raw.hide()
            self.label_apg.hide()
        elif self.apg_plot.isVisible() and self.apg_plot.vb.sceneBoundingRect().contains(pos):
            mousePoint = self.apg_plot.vb.mapSceneToView(pos)
            x, y = mousePoint.x(), mousePoint.y()
            self.label_apg.setHtml(f"X: {x:.2f}<br>APG: {y:.4f}")
            self.label_apg.setPos(x, y)
            self.label_apg.show()
            self.label_deriv_raw.hide()
            self.label_vpg.hide()

        if x is not None:
            self.vLine_deriv_raw.setPos(x)
            self.vLine_vpg.setPos(x)
            self.vLine_apg.setPos(x)

    def mouseClicked_deriv(self, evt):
        if evt.button() == Qt.MouseButton.LeftButton and self.cb_dot_mode.isChecked():
            pos = evt.scenePos()
            plots = [self.deriv_raw_plot, self.vpg_plot, self.apg_plot]
            for p in plots:
                if p.isVisible() and p.vb.sceneBoundingRect().contains(pos):
                    mousePoint = p.vb.mapSceneToView(pos)
                    x, y = mousePoint.x(), mousePoint.y()
                    scatter = pg.ScatterPlotItem(x=[x], y=[y], size=8, brush=pg.mkBrush(255, 0, 0, 255))
                    p.addItem(scatter)
                    v_line = pg.InfiniteLine(pos=x, angle=90, pen=pg.mkPen('#FF5722', width=1, style=Qt.PenStyle.DashLine))
                    for target_p in plots:
                        if target_p.isVisible():
                            target_p.addItem(v_line)
                    self.temp_rulers.append((scatter, v_line, p))
                    break

    def undo_temp_ruler(self):
        if self.temp_rulers:
            item = self.temp_rulers.pop()
            scatter, v_line, p = item
            p.removeItem(scatter)
            for target_p in [self.deriv_raw_plot, self.vpg_plot, self.apg_plot]:
                target_p.removeItem(v_line)
            self.redo_rulers.append(item)

    def redo_temp_ruler(self):
        if self.redo_rulers:
            item = self.redo_rulers.pop()
            scatter, v_line, p = item
            p.addItem(scatter)
            for target_p in [self.deriv_raw_plot, self.vpg_plot, self.apg_plot]:
                target_p.addItem(v_line)
            self.temp_rulers.append(item)

    def clear_temp_rulers(self):
        for scatter, v_line, p in self.temp_rulers:
            try:
                p.removeItem(scatter)
            except Exception:
                pass
            for target_p in [self.deriv_raw_plot, self.vpg_plot, self.apg_plot]:
                try:
                    target_p.removeItem(v_line)
                except Exception:
                    pass
        self.temp_rulers.clear()
        self.redo_rulers.clear()

    def export_fft(self):
        path, _ = QFileDialog.getSaveFileName(self, "Lưu ảnh FFT", "fft_analysis.png", "PNG Files (*.png)")
        if path:
            exporter = pg.exporters.ImageExporter(self.fft_widget.plotItem)
            exporter.export(path)
            QMessageBox.information(self, "Thành công", f"Đã lưu ảnh tại:\n{path}")

    def export_deriv_plot(self):
        path, _ = QFileDialog.getSaveFileName(self, "Lưu ảnh Đạo hàm", "deriv_analysis.png", "PNG Files (*.png)")
        if path:
            exporter = pg.exporters.ImageExporter(self.deriv_layout_widget.scene())
            exporter.export(path)
            QMessageBox.information(self, "Thành công", f"Đã lưu ảnh tại:\n{path}")

    # =========================================================================
    # TAB 3: BỘ LỌC SỐ (DSP FILTER PIPELINE)
    # =========================================================================
    def setup_filter_tab(self):
        layout = QHBoxLayout(self.tab_filter)

        # Cột trái: Cài đặt và danh sách lớp lọc
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_widget.setFixedWidth(380)

        # Hộp chọn Preset nhanh
        preset_box = QGroupBox("Cấu hình Mẫu (Presets)")
        preset_layout = QHBoxLayout(preset_box)
        self.cb_preset = QComboBox()
        self.cb_preset.addItems([
            "--- Chọn Preset nhanh ---",
            "RR: Bandpass Butterworth (0.1 - 0.5 Hz)",
            "RR: Dual-EMA Bandpass (0.1 - 0.5 Hz)",
            "RR: Thông thấp Khử nhiễu (0.8 Hz)",
            "RR: Khử trôi nền (Highpass 0.08 Hz)",
            "PPG: Bandpass Nhịp tim (0.5 - 5.0 Hz)",
            "Lọc Savitzky-Golay làm mượt",
            "Lọc Hampel khử xung dị biệt"
        ])
        self.cb_preset.currentIndexChanged.connect(self.apply_preset)
        preset_layout.addWidget(self.cb_preset)
        left_layout.addWidget(preset_box)

        # Hộp Pipeline
        pipe_box = QGroupBox("Chuỗi Bộ Lọc (Pipeline Layers)")
        pipe_layout = QVBoxLayout(pipe_box)

        self.list_pipeline = QListWidget()
        self.list_pipeline.currentRowChanged.connect(self.on_pipeline_selection_changed)
        self.list_pipeline.itemChanged.connect(self.on_pipeline_item_changed)
        pipe_layout.addWidget(self.list_pipeline)

        btn_row1 = QHBoxLayout()
        self.btn_add_layer = QPushButton("+ Thêm Layer")
        self.btn_add_layer.clicked.connect(self.add_pipeline_filter)
        self.btn_remove_layer = QPushButton("- Xóa")
        self.btn_remove_layer.clicked.connect(self.remove_pipeline_filter)
        self.btn_up_layer = QPushButton("▲ Lên")
        self.btn_up_layer.clicked.connect(self.move_pipeline_filter_up)
        self.btn_down_layer = QPushButton("▼ Xuống")
        self.btn_down_layer.clicked.connect(self.move_pipeline_filter_down)

        btn_row1.addWidget(self.btn_add_layer)
        btn_row1.addWidget(self.btn_remove_layer)
        btn_row1.addWidget(self.btn_up_layer)
        btn_row1.addWidget(self.btn_down_layer)
        pipe_layout.addLayout(btn_row1)
        left_layout.addWidget(pipe_box)

        # Hộp thông số lớp đang chọn
        param_box = QGroupBox("Tham số của Layer đang chọn")
        self.param_form = QFormLayout(param_box)

        self.cb_filter_impl = QComboBox()
        self.cb_filter_impl.addItems([
            "IIR (Butterworth)",
            "EMA (Trung bình trượt số mũ)",
            "FIR (Equiripple)",
            "FIR (Cửa sổ Kaiser/Hamming)",
            "Savitzky-Golay (Làm mượt)",
            "Hampel (Lọc xung dị biệt)",
            "Kalman Filter"
        ])
        self.cb_filter_impl.currentIndexChanged.connect(self.on_filter_impl_changed)
        self.param_form.addRow("Thuật toán:", self.cb_filter_impl)

        self.lbl_filter_type = QLabel("Loại bộ lọc:")
        self.cb_filter_type = QComboBox()
        self.cb_filter_type.addItems(["Lowpass", "Highpass", "Bandpass", "Bandstop"])
        self.cb_filter_type.currentIndexChanged.connect(self.on_filter_type_changed)
        self.param_form.addRow(self.lbl_filter_type, self.cb_filter_type)

        self.lbl_order = QLabel("Bậc / Số Taps:")
        self.spin_order = QSpinBox()
        self.spin_order.setRange(1, 200)
        self.spin_order.setValue(4)
        self.spin_order.valueChanged.connect(self.on_param_changed)
        self.param_form.addRow(self.lbl_order, self.spin_order)

        self.lbl_cut1 = QLabel("Tần số cắt 1:")
        self.spin_cut1 = QDoubleSpinBox()
        self.spin_cut1.setRange(0.01, 500.0)
        self.spin_cut1.setValue(0.1)
        self.spin_cut1.setSingleStep(0.05)
        self.spin_cut1.setSuffix(" Hz")
        self.spin_cut1.valueChanged.connect(self.on_param_changed)
        self.param_form.addRow(self.lbl_cut1, self.spin_cut1)

        self.lbl_cut2 = QLabel("Tần số cắt 2:")
        self.spin_cut2 = QDoubleSpinBox()
        self.spin_cut2.setRange(0.01, 500.0)
        self.spin_cut2.setValue(0.5)
        self.spin_cut2.setSingleStep(0.05)
        self.spin_cut2.setSuffix(" Hz")
        self.spin_cut2.valueChanged.connect(self.on_param_changed)
        self.param_form.addRow(self.lbl_cut2, self.spin_cut2)

        # Label hiển thị thông tin động cho EMA
        self.lbl_ema_info = QLabel("")
        self.lbl_ema_info.setStyleSheet("color: #00838F; font-size: 11px; font-style: italic;")
        self.param_form.addRow(self.lbl_ema_info)

        # Tham số cho Savitzky-Golay & Hampel
        self.lbl_sg_win = QLabel("Cửa sổ (Window):")
        self.spin_sg_win = QSpinBox()
        self.spin_sg_win.setRange(3, 151)
        self.spin_sg_win.setSingleStep(2)
        self.spin_sg_win.setValue(11)
        self.spin_sg_win.setSuffix(" mẫu")
        self.spin_sg_win.valueChanged.connect(self.on_sg_hampel_changed)
        self.param_form.addRow(self.lbl_sg_win, self.spin_sg_win)

        self.lbl_sg_poly = QLabel("Bậc đa thức:")
        self.spin_sg_poly = QSpinBox()
        self.spin_sg_poly.setRange(1, 6)
        self.spin_sg_poly.setValue(3)
        self.spin_sg_poly.valueChanged.connect(self.on_sg_hampel_changed)
        self.param_form.addRow(self.lbl_sg_poly, self.spin_sg_poly)

        # Tham số cho Hampel
        self.lbl_hampel_sigma = QLabel("Ngưỡng Sigma:")
        self.spin_hampel_sigma = QDoubleSpinBox()
        self.spin_hampel_sigma.setRange(0.5, 10.0)
        self.spin_hampel_sigma.setValue(3.0)
        self.spin_hampel_sigma.setSingleStep(0.5)
        self.spin_hampel_sigma.valueChanged.connect(self.on_sg_hampel_changed)
        self.param_form.addRow(self.lbl_hampel_sigma, self.spin_hampel_sigma)

        # Tham số cho Kalman
        self.lbl_kalman_r = QLabel("Nhiễu đo R:")
        self.spin_kalman_r = QDoubleSpinBox()
        self.spin_kalman_r.setRange(0.0001, 100.0)
        self.spin_kalman_r.setValue(0.1)
        self.spin_kalman_r.setDecimals(4)
        self.spin_kalman_r.valueChanged.connect(self.on_kalman_changed)
        self.param_form.addRow(self.lbl_kalman_r, self.spin_kalman_r)

        self.lbl_kalman_q = QLabel("Nhiễu quá trình Q:")
        self.spin_kalman_q = QDoubleSpinBox()
        self.spin_kalman_q.setRange(0.00001, 10.0)
        self.spin_kalman_q.setValue(0.01)
        self.spin_kalman_q.setDecimals(5)
        self.spin_kalman_q.valueChanged.connect(self.on_kalman_changed)
        self.param_form.addRow(self.lbl_kalman_q, self.spin_kalman_q)

        left_layout.addWidget(param_box)

        # Các nút xuất kết quả
        act_box = QGroupBox("Thao tác")
        act_layout = QVBoxLayout(act_box)

        self.cb_auto_preview = QCheckBox("Tự động xem trước (Auto-preview)")
        self.cb_auto_preview.setChecked(True)
        act_layout.addWidget(self.cb_auto_preview)

        self.cb_show_response = QCheckBox("Hiện Đáp ứng Tần số / Pha")
        self.cb_show_response.setChecked(True)
        act_layout.addWidget(self.cb_show_response)

        self.btn_apply_filter = QPushButton("Áp dụng Chuỗi Lọc")
        self.btn_apply_filter.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 6px;")
        self.btn_apply_filter.clicked.connect(self.apply_filter)
        act_layout.addWidget(self.btn_apply_filter)

        export_row = QHBoxLayout()
        self.btn_export_c = QPushButton("Xuất C/C++ Code")
        self.btn_export_c.clicked.connect(self.export_c_code)
        self.btn_export_csv = QPushButton("Xuất CSV Đã Lọc")
        self.btn_export_csv.clicked.connect(self.export_filtered_csv)
        export_row.addWidget(self.btn_export_c)
        export_row.addWidget(self.btn_export_csv)
        act_layout.addLayout(export_row)

        left_layout.addWidget(act_box)
        left_layout.addStretch()
        layout.addWidget(left_widget)

        # Cột phải: Đồ thị đáp ứng và xem trước tín hiệu
        plots_splitter = QSplitter(Qt.Orientation.Vertical)

        # Đồ thị Đáp ứng tần số & pha
        self.resp_plot_widget = pg.PlotWidget(title="Đáp ứng Biên độ Tần số |H(f)| (dB)")
        self.resp_plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.resp_plot_widget.setLabel('bottom', "Tần số (Hz)")
        self.resp_plot_widget.setLabel('left', "Độ lợi (dB)")
        self.freq_curve = self.resp_plot_widget.plot(pen=pg.mkPen('#00897B', width=2.0))
        plots_splitter.addWidget(self.resp_plot_widget)

        # Đồ thị Tín hiệu Gốc vs Đã Lọc
        self.filt_time_plot = pg.PlotWidget(title="So sánh Tín hiệu Gốc và Đã Lọc")
        self.filt_time_plot.showGrid(x=True, y=True, alpha=0.3)
        self.filt_time_plot.setLabel('bottom', "Thời gian / Mẫu")
        self.filt_time_plot.setLabel('left', "Biên độ")
        self.filt_time_plot.addLegend()
        self.orig_curve = self.filt_time_plot.plot(pen=pg.mkPen('#B0BEC5', width=1.5), name="Gốc (Original)")
        self.filt_curve = self.filt_time_plot.plot(pen=pg.mkPen('#E91E63', width=1.8), name="Đã Lọc (Filtered)")
        plots_splitter.addWidget(self.filt_time_plot)

        plots_splitter.setSizes([350, 450])
        layout.addWidget(plots_splitter)

        # Khởi tạo 1 layer mặc định
        self.add_pipeline_filter()

    def create_default_layer(self, impl="IIR (Butterworth)"):
        return {
            'impl': impl,
            'type': "Bandpass",
            'order': 4,
            'cutoff1': 0.1,
            'cutoff2': 0.5,
            'is_active': True,
            'sos': None,
            'b': None,
            'a': None
        }

    def add_pipeline_filter(self):
        layer = self.create_default_layer()
        self.filter_pipeline.append(layer)
        self.update_pipeline_list()
        self.list_pipeline.setCurrentRow(len(self.filter_pipeline) - 1)
        if self.cb_auto_preview.isChecked():
            self.apply_filter(silent=True)

    def remove_pipeline_filter(self):
        idx = self.list_pipeline.currentRow()
        if 0 <= idx < len(self.filter_pipeline):
            self.filter_pipeline.pop(idx)
            self.update_pipeline_list()
            new_idx = min(idx, len(self.filter_pipeline) - 1)
            self.list_pipeline.setCurrentRow(new_idx)
            if self.cb_auto_preview.isChecked():
                self.apply_filter(silent=True)

    def move_pipeline_filter_up(self):
        idx = self.list_pipeline.currentRow()
        if idx > 0:
            self.filter_pipeline[idx], self.filter_pipeline[idx - 1] = self.filter_pipeline[idx - 1], self.filter_pipeline[idx]
            self.update_pipeline_list()
            self.list_pipeline.setCurrentRow(idx - 1)
            if self.cb_auto_preview.isChecked():
                self.apply_filter(silent=True)

    def move_pipeline_filter_down(self):
        idx = self.list_pipeline.currentRow()
        if 0 <= idx < len(self.filter_pipeline) - 1:
            self.filter_pipeline[idx], self.filter_pipeline[idx + 1] = self.filter_pipeline[idx + 1], self.filter_pipeline[idx]
            self.update_pipeline_list()
            self.list_pipeline.setCurrentRow(idx + 1)
            if self.cb_auto_preview.isChecked():
                self.apply_filter(silent=True)

    def update_pipeline_list(self):
        self._is_populating = True
        self.list_pipeline.clear()
        for i, layer in enumerate(self.filter_pipeline):
            item_text = f"L{i+1}: {layer['impl']} ({layer['type']})"
            item = QListWidgetItem(item_text)
            item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
            item.setCheckState(Qt.CheckState.Checked if layer.get('is_active', True) else Qt.CheckState.Unchecked)
            self.list_pipeline.addItem(item)
        self._is_populating = False

    def update_param_visibility(self):
        """Ẩn/hiện các ô tham số tương ứng với từng loại thuật toán bộ lọc."""
        impl = self.cb_filter_impl.currentText()
        f_type = self.cb_filter_type.currentText()
        is_iir_fir = ("IIR" in impl or "FIR" in impl)
        is_ema = ("EMA" in impl)
        is_sg = ("Savitzky" in impl)
        is_hampel = ("Hampel" in impl)
        is_kalman = ("Kalman" in impl)

        # 1. Loại bộ lọc (Lowpass, Highpass, Bandpass, Bandstop)
        show_type = is_iir_fir or is_ema
        self.lbl_filter_type.setVisible(show_type)
        self.cb_filter_type.setVisible(show_type)

        # 2. Bậc / Số taps / Số lần lặp
        show_order = is_iir_fir or is_ema
        self.lbl_order.setVisible(show_order)
        self.spin_order.setVisible(show_order)
        if "IIR" in impl:
            self.lbl_order.setText("Bậc lọc (Poles):")
            self.spin_order.setRange(1, 12)
        elif "FIR" in impl:
            self.lbl_order.setText("Số Taps (Độ dài):")
            self.spin_order.setRange(3, 301)
        elif is_ema:
            self.lbl_order.setText("Số lần lặp (Cascades):")
            self.spin_order.setRange(1, 10)

        # 3. Tần số cắt 1 & 2
        show_cut = is_iir_fir or is_ema
        self.lbl_cut1.setVisible(show_cut)
        self.spin_cut1.setVisible(show_cut)
        is_band = f_type in ["Bandpass", "Bandstop"]
        self.lbl_cut2.setVisible(show_cut and is_band)
        self.spin_cut2.setVisible(show_cut and is_band)

        # 4. EMA Info Label
        self.lbl_ema_info.setVisible(is_ema)
        if is_ema:
            self.update_ema_info()

        # 5. Savitzky-Golay & Hampel Window
        show_sg = is_sg
        show_hampel = is_hampel
        self.lbl_sg_win.setVisible(show_sg or show_hampel)
        self.spin_sg_win.setVisible(show_sg or show_hampel)
        self.lbl_sg_poly.setVisible(show_sg)
        self.spin_sg_poly.setVisible(show_sg)
        self.lbl_hampel_sigma.setVisible(show_hampel)
        self.spin_hampel_sigma.setVisible(show_hampel)

        # 6. Kalman
        self.lbl_kalman_r.setVisible(is_kalman)
        self.spin_kalman_r.setVisible(is_kalman)
        self.lbl_kalman_q.setVisible(is_kalman)
        self.spin_kalman_q.setVisible(is_kalman)

    def update_ema_info(self):
        """Hiển thị thông số alpha và span tính toán được từ tần số cắt cho EMA."""
        Fs = max(self.offline_fs, 1.0)
        c1 = self.spin_cut1.value()
        c2 = self.spin_cut2.value()
        f_type = self.cb_filter_type.currentText()

        a1 = 1.0 - np.exp(-2.0 * np.pi * c1 / Fs)
        span1 = max(1, int(round(2.0 / max(a1, 1e-4) - 1.0)))

        if f_type in ["Bandpass", "Bandstop"]:
            a2 = 1.0 - np.exp(-2.0 * np.pi * c2 / Fs)
            span2 = max(1, int(round(2.0 / max(a2, 1e-4) - 1.0)))
            self.lbl_ema_info.setText(
                f"Dual-EMA: α1(chậm)={a1:.4f} (Span {span1}m) | α2(nhanh)={a2:.4f} (Span {span2}m)"
            )
        else:
            self.lbl_ema_info.setText(
                f"EMA: α={a1:.4f} (Span tương đương: {span1} mẫu @ {Fs:.1f}Hz)"
            )

    def on_pipeline_selection_changed(self, idx):
        if self._is_populating or idx < 0 or idx >= len(self.filter_pipeline):
            return
        self.current_filter_idx = idx
        layer = self.filter_pipeline[idx]

        self._is_populating = True
        self.cb_filter_impl.setCurrentText(layer.get('impl', "IIR (Butterworth)"))
        self.cb_filter_type.setCurrentText(layer.get('type', "Bandpass"))
        self.spin_order.setValue(int(layer.get('order', 4)))
        self.spin_cut1.setValue(float(layer.get('cutoff1', 0.1)))
        self.spin_cut2.setValue(float(layer.get('cutoff2', 0.5)))
        self.spin_sg_win.setValue(int(layer.get('sg_window', layer.get('hampel_window', 11))))
        self.spin_sg_poly.setValue(int(layer.get('sg_poly', 3)))
        self.spin_hampel_sigma.setValue(float(layer.get('hampel_sigma', 3.0)))
        self.spin_kalman_r.setValue(float(layer.get('kalman_r', 0.1)))
        self.spin_kalman_q.setValue(float(layer.get('kalman_q', 0.01)))
        self._is_populating = False

        self.update_param_visibility()

    def on_pipeline_item_changed(self, item):
        if self._is_populating:
            return
        idx = self.list_pipeline.row(item)
        if 0 <= idx < len(self.filter_pipeline):
            is_active = (item.checkState() == Qt.CheckState.Checked)
            self.filter_pipeline[idx]['is_active'] = is_active
            if self.cb_auto_preview.isChecked():
                self.apply_filter(silent=True)

    def on_filter_impl_changed(self, idx):
        if self._is_populating or self.current_filter_idx < 0:
            return
        layer = self.filter_pipeline[self.current_filter_idx]
        layer['impl'] = self.cb_filter_impl.currentText()
        self.update_param_visibility()
        self.update_pipeline_list()
        self.list_pipeline.setCurrentRow(self.current_filter_idx)
        if self.cb_auto_preview.isChecked():
            self.apply_filter(silent=True)

    def on_filter_type_changed(self, idx):
        if self._is_populating or self.current_filter_idx < 0:
            return
        layer = self.filter_pipeline[self.current_filter_idx]
        layer['type'] = self.cb_filter_type.currentText()
        self.update_param_visibility()
        self.update_pipeline_list()
        self.list_pipeline.setCurrentRow(self.current_filter_idx)
        if self.cb_auto_preview.isChecked():
            self.apply_filter(silent=True)

    def on_param_changed(self):
        if self._is_populating or self.current_filter_idx < 0:
            return
        layer = self.filter_pipeline[self.current_filter_idx]
        layer['order'] = self.spin_order.value()
        layer['cutoff1'] = self.spin_cut1.value()
        layer['cutoff2'] = self.spin_cut2.value()
        if "EMA" in layer.get('impl', ''):
            self.update_ema_info()
        if self.cb_auto_preview.isChecked():
            self.apply_filter(silent=True)

    def on_sg_hampel_changed(self):
        if self._is_populating or self.current_filter_idx < 0:
            return
        layer = self.filter_pipeline[self.current_filter_idx]
        w = self.spin_sg_win.value()
        if w % 2 == 0:
            w += 1
            self.spin_sg_win.blockSignals(True)
            self.spin_sg_win.setValue(w)
            self.spin_sg_win.blockSignals(False)
        layer['sg_window'] = w
        layer['hampel_window'] = w
        layer['sg_poly'] = self.spin_sg_poly.value()
        layer['hampel_sigma'] = self.spin_hampel_sigma.value()
        if self.cb_auto_preview.isChecked():
            self.apply_filter(silent=True)

    def on_kalman_changed(self):
        if self._is_populating or self.current_filter_idx < 0:
            return
        layer = self.filter_pipeline[self.current_filter_idx]
        layer['kalman_r'] = self.spin_kalman_r.value()
        layer['kalman_q'] = self.spin_kalman_q.value()
        if self.cb_auto_preview.isChecked():
            self.apply_filter(silent=True)

    def apply_preset(self, idx):
        if idx <= 0:
            return
        preset_name = self.cb_preset.currentText()
        if "Bandpass Butterworth" in preset_name:
            self.filter_pipeline = [{
                'impl': "IIR (Butterworth)",
                'type': "Bandpass",
                'order': 3,
                'cutoff1': 0.1,
                'cutoff2': 0.5,
                'is_active': True
            }]
        elif "Dual-EMA Bandpass" in preset_name:
            self.filter_pipeline = [{
                'impl': "EMA (Trung bình trượt số mũ)",
                'type': "Bandpass",
                'order': 1,
                'cutoff1': 0.1,
                'cutoff2': 0.5,
                'is_active': True
            }]
        elif "RR: Thông thấp" in preset_name:
            self.filter_pipeline = [{
                'impl': "IIR (Butterworth)",
                'type': "Lowpass",
                'order': 3,
                'cutoff1': 0.8,
                'cutoff2': 1.5,
                'is_active': True
            }]
        elif "RR: Khử trôi nền" in preset_name:
            self.filter_pipeline = [{
                'impl': "IIR (Butterworth)",
                'type': "Highpass",
                'order': 2,
                'cutoff1': 0.08,
                'cutoff2': 1.0,
                'is_active': True
            }]
        elif "PPG: Bandpass" in preset_name:
            self.filter_pipeline = [{
                'impl': "IIR (Butterworth)",
                'type': "Bandpass",
                'order': 4,
                'cutoff1': 0.5,
                'cutoff2': 5.0,
                'is_active': True
            }]
        elif "Savitzky-Golay" in preset_name:
            self.filter_pipeline = [{
                'impl': "Savitzky-Golay (Làm mượt)",
                'type': "Lowpass",
                'order': 3,
                'sg_window': 11,
                'sg_poly': 3,
                'is_active': True
            }]
        elif "Hampel" in preset_name:
            self.filter_pipeline = [{
                'impl': "Hampel (Lọc xung dị biệt)",
                'type': "Lowpass",
                'order': 1,
                'hampel_window': 7,
                'hampel_sigma': 3.0,
                'is_active': True
            }]

        self.update_pipeline_list()
        self.list_pipeline.setCurrentRow(0)
        self.apply_filter(silent=True)
        self.cb_preset.setCurrentIndex(0)

    def apply_filter(self, *args, silent=False):
        if not isinstance(silent, bool):
            silent = False

        if self.pure_offline_signal is None:
            if not silent:
                QMessageBox.warning(self, "Chưa có dữ liệu", "Vui lòng tải file CSV ở Tab 'Offline Analysis' trước.")
            return

        signal = -self.pure_offline_signal if self.cb_invert_offline.isChecked() else self.pure_offline_signal
        Fs = self.offline_fs

        # 1. Tính đáp ứng tần số
        if self.cb_show_response.isChecked():
            f_hz, mag_db, phase_rad, ideal_f, ideal_db = calculate_frequency_response(self.filter_pipeline, fs=Fs)
            if f_hz is not None:
                self.freq_curve.setData(f_hz, mag_db)
                self.resp_plot_widget.setXRange(0, Fs / 2.0)
            else:
                self.freq_curve.setData([], [])

        # 2. Lọc tín hiệu qua chuỗi pipeline (truyền đúng Fs)
        try:
            filtered = apply_pipeline_filters(signal, self.filter_pipeline, fs=Fs)
            self.filtered_signal_cache = filtered
            self.offline_signal = filtered

            self.orig_curve.setData(signal)
            self.filt_curve.setData(filtered)
            self.filt_time_plot.setTitle("So sánh Tín hiệu Gốc vs Đã Lọc (Pipeline OK)")

            if not silent:
                QMessageBox.information(self, "Hoàn tất lọc", "Đã áp dụng toàn bộ chuỗi lọc thành công.")
        except Exception as e:
            self.filt_curve.setData([])
            self.filt_time_plot.setTitle(f"Lỗi lọc: {str(e)}")
            if not silent:
                QMessageBox.critical(self, "Lỗi bộ lọc", str(e))

    def export_c_code(self):
        Fs = self.offline_fs
        code = "// ================================================================\n"
        code += f"// C/C++ DSP Filter Header - Generated by RR-Laser Suite\n"
        code += f"// Tan so lay mau thiet ke Fs: {Fs:.1f} Hz\n"
        code += f"// Thoi gian tao: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
        code += "// ================================================================\n\n"

        count = 0
        for i, layer in enumerate(self.filter_pipeline):
            if not layer.get('is_active', True):
                continue
            impl = layer.get('impl', '')
            f_type = layer.get('type', 'Lowpass')
            cut1 = float(layer.get('cutoff1', 0.1))
            cut2 = float(layer.get('cutoff2', 0.5))

            if "IIR" in impl:
                sos = layer.get('sos')
                if sos is not None:
                    count += 1
                    code += f"// --- Layer {i+1}: IIR Butterworth ({f_type}) ---\n"
                    code += f"#define IIR_L{i+1}_SECTIONS {len(sos)}\n"
                    code += f"const float iir_l{i+1}_sos[IIR_L{i+1}_SECTIONS][6] = {{\n"
                    for sec in sos:
                        code += "    {" + ", ".join([f"{x:.6e}f" for x in sec]) + "},\n"
                    code += "};\n\n"

            elif "FIR" in impl:
                b = layer.get('b')
                if b is not None:
                    count += 1
                    code += f"// --- Layer {i+1}: FIR Filter ({f_type}) ---\n"
                    code += f"#define FIR_L{i+1}_TAPS {len(b)}\n"
                    code += f"const float fir_l{i+1}_taps[FIR_L{i+1}_TAPS] = {{\n    "
                    code += ", ".join([f"{x:.6e}f" for x in b])
                    code += "\n};\n\n"

            elif "EMA" in impl:
                count += 1
                a1 = 1.0 - np.exp(-2.0 * np.pi * cut1 / Fs)
                a2 = 1.0 - np.exp(-2.0 * np.pi * cut2 / Fs)
                code += f"// --- Layer {i+1}: EMA ({f_type}) ---\n"
                if f_type == "Bandpass":
                    code += f"const float ema_alpha_slow_{i+1} = {a1:.6e}f; // fc1 = {cut1:.2f} Hz\n"
                    code += f"const float ema_alpha_fast_{i+1} = {a2:.6e}f; // fc2 = {cut2:.2f} Hz\n"
                    code += f"static float ema_slow_{i+1} = 0.0f;\n"
                    code += f"static float ema_fast_{i+1} = 0.0f;\n\n"
                    code += f"float apply_ema_bandpass_{i+1}(float x) {{\n"
                    code += f"    ema_slow_{i+1} += ema_alpha_slow_{i+1} * (x - ema_slow_{i+1});\n"
                    code += f"    ema_fast_{i+1} += ema_alpha_fast_{i+1} * (x - ema_fast_{i+1});\n"
                    code += f"    return ema_fast_{i+1} - ema_slow_{i+1};\n"
                    code += "}\n\n"
                elif f_type == "Highpass":
                    code += f"const float ema_alpha_{i+1} = {a1:.6e}f; // fc = {cut1:.2f} Hz\n"
                    code += f"static float ema_state_{i+1} = 0.0f;\n\n"
                    code += f"float apply_ema_highpass_{i+1}(float x) {{\n"
                    code += f"    ema_state_{i+1} += ema_alpha_{i+1} * (x - ema_state_{i+1});\n"
                    code += f"    return x - ema_state_{i+1};\n"
                    code += "}\n\n"
                else:  # Lowpass
                    code += f"const float ema_alpha_{i+1} = {a1:.6e}f; // fc = {cut1:.2f} Hz\n"
                    code += f"static float ema_state_{i+1} = 0.0f;\n\n"
                    code += f"float apply_ema_lowpass_{i+1}(float x) {{\n"
                    code += f"    ema_state_{i+1} += ema_alpha_{i+1} * (x - ema_state_{i+1});\n"
                    code += f"    return ema_state_{i+1};\n"
                    code += "}\n\n"

            elif "Kalman" in impl:
                count += 1
                r_val = float(layer.get('kalman_r', 0.1))
                q_val = float(layer.get('kalman_q', 0.01))
                code += f"// --- Layer {i+1}: 1D Kalman Filter (R={r_val}, Q={q_val}) ---\n"
                code += f"static float k_x_{i+1} = 0.0f, k_p_{i+1} = 1.0f;\n"
                code += f"const float k_r_{i+1} = {r_val:.6e}f, k_q_{i+1} = {q_val:.6e}f;\n"
                code += f"float apply_kalman_{i+1}(float z) {{\n"
                code += f"    float p_pred = k_p_{i+1} + k_q_{i+1};\n"
                code += f"    float k_gain = p_pred / (p_pred + k_r_{i+1});\n"
                code += f"    k_x_{i+1} = k_x_{i+1} + k_gain * (z - k_x_{i+1});\n"
                code += f"    k_p_{i+1} = (1.0f - k_gain) * p_pred;\n"
                code += f"    return k_x_{i+1};\n"
                code += "}\n\n"

        if count == 0:
            QMessageBox.information(self, "Lưu ý", "Không có bộ lọc nào đang Active để xuất code.")
            return

        dlg = QDialog(self)
        dlg.setWindowTitle("C/C++ Code (DSP Pipeline)")
        dlg.resize(650, 450)
        vbox = QVBoxLayout(dlg)
        txt = QTextEdit()
        txt.setPlainText(code)
        txt.setReadOnly(True)
        txt.setFont(QFont("Courier New", 9))
        vbox.addWidget(txt)
        btn_close = QPushButton("Đóng")
        btn_close.clicked.connect(dlg.accept)
        vbox.addWidget(btn_close)
        dlg.exec()

    def export_filtered_csv(self):
        if self.filtered_signal_cache is None:
            QMessageBox.warning(self, "Chưa lọc", "Vui lòng áp dụng bộ lọc trước khi xuất CSV.")
            return

        path, _ = QFileDialog.getSaveFileName(self, "Lưu file CSV đã lọc", "filtered_signal.csv", "CSV Files (*.csv)")
        if path:
            try:
                orig = -self.pure_offline_signal if self.cb_invert_offline.isChecked() else self.pure_offline_signal
                df_out = pd.DataFrame({
                    'Original': orig,
                    'Filtered': self.filtered_signal_cache
                })
                if self.offline_time is not None and len(self.offline_time) == len(orig):
                    df_out.insert(0, 'Time_s', self.offline_time)
                df_out.to_csv(path, index=False)
                QMessageBox.information(self, "Thành công", f"Đã lưu kết quả tại:\n{path}")
            except Exception as e:
                QMessageBox.critical(self, "Lỗi lưu file", str(e))

    # =========================================================================
    # TAB 4: PHÂN TÍCH SÓNG (WAVELET ANALYSIS - DWT/IDWT)
    # =========================================================================
    def setup_wavelet_tab(self):
        layout = QHBoxLayout(self.tab_wavelet)

        # Cột điều khiển
        ctrl_panel = QWidget()
        ctrl_layout = QVBoxLayout(ctrl_panel)
        ctrl_panel.setFixedWidth(280)

        family_row = QHBoxLayout()
        family_row.addWidget(QLabel("Họ sóng:"))
        self.cb_wavelet_family = QComboBox()
        self.cb_wavelet_family.addItems(['sym4', 'sym8', 'db4', 'db8', 'coif4', 'haar'])
        family_row.addWidget(self.cb_wavelet_family)
        ctrl_layout.addLayout(family_row)

        self.btn_wavelet_analyze = QPushButton("Phân tách Wavelet (DWT 6 mức)")
        self.btn_wavelet_analyze.setStyleSheet("background-color: #8E24AA; color: white; font-weight: bold; padding: 6px;")
        self.btn_wavelet_analyze.clicked.connect(self.perform_wavelet_analysis)
        ctrl_layout.addWidget(self.btn_wavelet_analyze)

        self.btn_wavelet_overlay = QPushButton("Chế độ: Chồng phổ (Overlay)")
        self.btn_wavelet_overlay.setCheckable(True)
        self.btn_wavelet_overlay.clicked.connect(self.toggle_wavelet_overlay)
        ctrl_layout.addWidget(self.btn_wavelet_overlay)

        ctrl_layout.addWidget(QLabel("Tích chọn để LOẠI BỎ dải tần:"))
        self.wavelet_checkboxes = []
        labels = [
            ("cA6 (Trôi nền cực chậm)", 0),
            ("cD6 (Nhịp thở chính)", 1),
            ("cD5 (Nhịp tim / Dao động nhỏ)", 2),
            ("cD4 (Dải trung tần thấp)", 3),
            ("cD3 (Dải trung tần)", 4),
            ("cD2 (Dải trung cao)", 5),
            ("cD1 (Nhiễu cao tần)", 6)
        ]
        for desc, idx in labels:
            cb = QCheckBox(desc)
            cb.stateChanged.connect(self.update_wavelet_reconstruction)
            self.wavelet_checkboxes.append((cb, idx))
            ctrl_layout.addWidget(cb)

        ctrl_layout.addStretch()
        layout.addWidget(ctrl_panel)

        # Cột hiển thị phổ và tín hiệu tái tạo
        plots_splitter = QSplitter(Qt.Orientation.Vertical)

        self.wavelet_bands_widget = pg.GraphicsLayoutWidget()
        plots_splitter.addWidget(self.wavelet_bands_widget)

        self.wavelet_recon_plot = pg.PlotWidget(title="Tín hiệu Tái tạo sau Triệt tiêu Dải (IDWT)")
        self.wavelet_recon_plot.showGrid(x=True, y=True, alpha=0.3)
        self.wavelet_recon_plot.setLabel('bottom', "Mẫu (Samples)")
        self.wavelet_recon_plot.setLabel('left', "Biên độ")
        self.wavelet_recon_plot.addLegend()

        self.curve_recon_orig = self.wavelet_recon_plot.plot([], [], pen=pg.mkPen('#B0BEC5', width=2.0), name="Gốc")
        self.curve_recon_new = self.wavelet_recon_plot.plot([], [], pen=pg.mkPen('#1E88E5', width=1.8), name="Tái tạo (IDWT)")
        plots_splitter.addWidget(self.wavelet_recon_plot)

        plots_splitter.setSizes([500, 300])
        layout.addWidget(plots_splitter)

        self.wavelet_coeffs = None
        self.wavelet_bands_plots = []
        self.wavelet_bands_curves = []

    def perform_wavelet_analysis(self, *args, silent=False):
        if not isinstance(silent, bool):
            silent = False
        if self.pure_offline_signal is None or len(self.pure_offline_signal) == 0:
            if not silent:
                QMessageBox.warning(self, "Chưa có dữ liệu", "Vui lòng tải file CSV ở Tab 'Offline Analysis' trước.")
            return

        wavelet_name = self.cb_wavelet_family.currentText()
        signal = -self.pure_offline_signal if self.cb_invert_offline.isChecked() else self.pure_offline_signal

        try:
            self.wavelet_coeffs = pywt.wavedec(signal, wavelet_name, level=6)
        except Exception as e:
            if not silent:
                QMessageBox.critical(self, "Lỗi Wavelet", str(e))
            return

        # Cập nhật nhãn tần số động theo Fs thực tế
        Fs = self.offline_fs
        freq_ranges = [
            f"cA6 (< {Fs/128.0:.2f} Hz - Trôi nền)",
            f"cD6 ({Fs/128.0:.2f} - {Fs/64.0:.2f} Hz - Nhịp thở)",
            f"cD5 ({Fs/64.0:.2f} - {Fs/32.0:.2f} Hz - Tim/Dao động)",
            f"cD4 ({Fs/32.0:.2f} - {Fs/16.0:.2f} Hz)",
            f"cD3 ({Fs/16.0:.2f} - {Fs/8.0:.2f} Hz)",
            f"cD2 ({Fs/8.0:.2f} - {Fs/4.0:.2f} Hz)",
            f"cD1 (> {Fs/4.0:.2f} Hz - Nhiễu cao)"
        ]
        for i, (cb, _) in enumerate(self.wavelet_checkboxes):
            cb.blockSignals(True)
            cb.setText(freq_ranges[i])
            cb.setChecked(False)
            cb.blockSignals(False)

        self.update_wavelet_plots()
        self.update_wavelet_reconstruction()
        if not silent:
            QMessageBox.information(self, "Wavelet thành công", "Đã phân tách tín hiệu thành 7 dải tần (1 xấp xỉ cA6 + 6 chi tiết cD1..cD6).")

    def toggle_wavelet_overlay(self):
        if self.btn_wavelet_overlay.isChecked():
            self.btn_wavelet_overlay.setText("Chế độ: Tách phổ (Subplots)")
        else:
            self.btn_wavelet_overlay.setText("Chế độ: Chồng phổ (Overlay)")
        if self.wavelet_coeffs is not None:
            self.update_wavelet_plots()

    def update_wavelet_plots(self):
        if self.wavelet_coeffs is None:
            return

        self.wavelet_bands_widget.clear()
        self.wavelet_bands_plots.clear()
        self.wavelet_bands_curves.clear()

        is_overlay = self.btn_wavelet_overlay.isChecked()
        wavelet_name = self.cb_wavelet_family.currentText()
        colors = ['#D32F2F', '#C2185B', '#7B1FA2', '#512DA8', '#303F9F', '#1976D2', '#0288D1']
        labels = ['cA6', 'cD6', 'cD5', 'cD4', 'cD3', 'cD2', 'cD1']
        n_orig = len(self.pure_offline_signal)

        if is_overlay:
            plot = self.wavelet_bands_widget.addPlot(title="Phổ Wavelet 7 Dải (Chồng phổ Overlay)")
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.addLegend()
            for i, coeff in enumerate(self.wavelet_coeffs):
                single_band = [np.zeros_like(c) for c in self.wavelet_coeffs]
                single_band[i] = coeff
                recon = pywt.waverec(single_band, wavelet_name)[:n_orig]
                curve = plot.plot(recon, pen=pg.mkPen(colors[i % len(colors)], width=1.5), name=labels[i])
                self.wavelet_bands_curves.append(curve)
        else:
            for i, coeff in enumerate(self.wavelet_coeffs):
                plot = self.wavelet_bands_widget.addPlot(title=labels[i])
                plot.showGrid(x=True, y=True, alpha=0.2)
                if i > 0 and self.wavelet_bands_plots:
                    plot.setXLink(self.wavelet_bands_plots[0])
                single_band = [np.zeros_like(c) for c in self.wavelet_coeffs]
                single_band[i] = coeff
                recon = pywt.waverec(single_band, wavelet_name)[:n_orig]
                curve = plot.plot(recon, pen=pg.mkPen(colors[i % len(colors)], width=1.5))
                self.wavelet_bands_plots.append(plot)
                self.wavelet_bands_curves.append(curve)
                self.wavelet_bands_widget.nextRow()

    def update_wavelet_reconstruction(self):
        from copy import deepcopy
        if self.wavelet_coeffs is None:
            return

        coeffs_mod = deepcopy(self.wavelet_coeffs)
        wavelet_name = self.cb_wavelet_family.currentText()
        n_orig = len(self.pure_offline_signal)

        for cb, idx in self.wavelet_checkboxes:
            if cb.isChecked():
                coeffs_mod[idx] = np.zeros_like(coeffs_mod[idx])

        recon = pywt.waverec(coeffs_mod, wavelet_name)[:n_orig]
        orig = -self.pure_offline_signal if self.cb_invert_offline.isChecked() else self.pure_offline_signal

        self.curve_recon_orig.setData(orig)
        self.curve_recon_new.setData(recon)

    # ------------------------------------------------------------------
    # Reset live plot
    # ------------------------------------------------------------------
    def reset_live_plot(self):
        """Xóa toàn bộ buffer và đồ thị live để chuẩn bị cho phiên đo mới."""
        # Reset numpy buffers
        self.live_count = 0
        self.live_total = 0
        self.live_buf_r  = np.zeros(self.max_samples)
        self.live_buf_va = np.zeros(self.max_samples)
        self.live_buf_vb = np.zeros(self.max_samples)

        # Clear plot curves
        self.curve_live_r.setData([])
        self.curve_live_va.setData([])
        self.curve_live_vb.setData([])
        self.live_plot_r.setXRange(0, self.max_samples, padding=0)

        # Reset status cards
        self.lbl_card_r.setText("r (Vị trí): --")
        self.lbl_card_rr.setText("Nhịp thở: -- bpm")
        self.lbl_card_va.setText("Va: -- mV")
        self.lbl_card_vb.setText("Vb: -- mV")
        self.lbl_card_flags.setText("Trạng thái: Sẵn sàng")
        self.lbl_save_status.setText("File: Chưa ghi")

        # Clear diagnostic log
        self.live_log_text.clear()

        # Reset breath estimator nếu không đang chạy
        if hasattr(self, 'serial_thread') and not self.serial_thread.is_running:
            self.serial_thread.breath_est = LiveBreathRate(45.0)

        self.live_log_text.append("[Làm mới] Đồ thị và buffer đã được xóa. Sẵn sàng cho phiên đo mới.")


def main():
    app = QApplication(sys.argv)
    window = RRAnalyzerSuite()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
