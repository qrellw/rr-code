"""
dsp_filters.py - Module xử lý tín hiệu số (DSP) toàn diện cho RR-Laser & PPG Suite.
Hỗ trợ:
- IIR (Butterworth) [Lowpass, Highpass, Bandpass, Bandstop]
- FIR (Equiripple / Parks-McClellan có fallback Kaiser)
- FIR (Window: Kaiser, Hamming, Hann)
- EMA (Exponential Moving Average: Lowpass, Highpass, Dual-EMA Bandpass, Bandstop, Cascaded Orders)
- Savitzky-Golay (Làm mượt bảo toàn hình thái sóng)
- Hampel Filter (Khử nhiễu gai ngoại lai dựa trên Median & MAD)
- Kalman Filter (Bộ lọc trạng thái 1D tối ưu)
"""

import numpy as np
from scipy.signal import (
    butter, firwin, freqz, filtfilt, sosfreqz, sosfiltfilt,
    savgol_filter, savgol_coeffs, remez
)
from scipy.ndimage import median_filter


def alpha_from_cutoff(fc, fs):
    """Tính hệ số alpha của EMA từ tần số cắt fc và tần số lấy mẫu fs."""
    fc = min(max(fc, 1e-4), 0.49 * fs)
    return float(1.0 - np.exp(-2.0 * np.pi * fc / fs))


def cutoff_from_alpha(alpha, fs):
    """Tính tần số cắt fc từ hệ số alpha của EMA."""
    alpha = min(max(alpha, 1e-5), 0.9999)
    return float(-np.log(1.0 - alpha) * fs / (2.0 * np.pi))


def run_single_ema(x, alpha, passes=1):
    """
    Chạy bộ lọc EMA xuôi-ngược (zero-phase forward-backward) với số lần lặp passes.
    """
    alpha = min(max(float(alpha), 1e-5), 0.9999)
    y = np.copy(x).astype(float)
    n = len(y)
    if n == 0:
        return y

    for _ in range(max(1, int(passes))):
        # Chiều xuôi
        fwd = np.empty(n, dtype=float)
        fwd[0] = y[0]
        a = alpha
        one_minus_a = 1.0 - a
        for i in range(1, n):
            fwd[i] = a * y[i] + one_minus_a * fwd[i - 1]

        # Chiều ngược (khử trễ pha)
        bwd = np.empty(n, dtype=float)
        bwd[-1] = fwd[-1]
        for i in range(n - 2, -1, -1):
            bwd[i] = a * fwd[i] + one_minus_a * bwd[i + 1]

        y = bwd
    return y


def calculate_frequency_response(filter_pipeline, fs=50.0, num_points=8000):
    """
    Tính đáp ứng biên độ |H(f)| và pha của toàn bộ chuỗi lọc trong pipeline.
    Hỗ trợ IIR, FIR, EMA, và Savitzky-Golay.
    """
    fs = max(float(fs), 1.0)
    total_h = np.ones(num_points, dtype=complex)
    w_axis = np.linspace(0, np.pi, num_points)
    has_freq_response = False
    ideal_f = None
    ideal_db = None

    nyq = 0.5 * fs

    for layer in filter_pipeline:
        if not layer.get('is_active', True):
            continue

        impl = layer.get('impl', '')
        f_type = layer.get('type', 'Lowpass')
        order = int(layer.get('order', 4))
        cut1 = min(float(layer.get('cutoff1', 0.1)), nyq * 0.98)
        cut2 = min(float(layer.get('cutoff2', 0.5)), nyq * 0.98)

        # ---------------- 1. IIR (Butterworth) ----------------
        if "IIR" in impl:
            try:
                if f_type in ["Bandpass", "Bandstop"]:
                    if cut1 >= cut2:
                        continue
                    Wn = [cut1 / nyq, cut2 / nyq]
                else:
                    Wn = cut1 / nyq

                sos = butter(min(order, 12), Wn, btype=f_type.lower(), output='sos')
                layer['sos'] = sos
                w, h = sosfreqz(sos, worN=num_points)
                # Vì sử dụng sosfiltfilt (zero-phase 2 chiều): đáp ứng là |h|^2
                total_h *= (np.abs(h) ** 2)
                has_freq_response = True
            except Exception as e:
                print(f"IIR response error: {e}")

        # ---------------- 2. FIR (Equiripple / Window) ----------------
        elif "FIR" in impl:
            try:
                numtaps = max(3, order)
                if numtaps % 2 == 0 and f_type in ["Highpass", "Bandstop"]:
                    numtaps += 1

                if "Equiripple" in impl:
                    f_stop1 = min(float(layer.get('eq_stop1', max(0.01, cut1 * 0.6))), nyq * 0.95)
                    f_pass1 = min(float(layer.get('eq_pass1', cut1)), nyq * 0.96)
                    f_pass2 = min(float(layer.get('eq_pass2', cut2)), nyq * 0.97)
                    f_stop2 = min(float(layer.get('eq_stop2', cut2 * 1.4)), nyq * 0.98)
                    att1 = float(layer.get('eq_att1', 40.0))
                    rip = float(layer.get('eq_rip', 1.0))
                    att2 = float(layer.get('eq_att2', 40.0))

                    if f_type == "Lowpass":
                        delta_f = (f_stop2 - f_pass2) / fs
                        bands = [0, f_pass2, f_stop2, nyq]
                        desired = [1, 0]
                        weights = [1.0, 1.0]
                        fallback_cut = f_pass2
                        pass_zero = True
                    elif f_type == "Highpass":
                        delta_f = (f_pass1 - f_stop1) / fs
                        bands = [0, f_stop1, f_pass1, nyq]
                        desired = [0, 1]
                        weights = [1.0, 1.0]
                        fallback_cut = f_pass1
                        pass_zero = False
                    elif f_type == "Bandpass":
                        delta_f = min((f_pass1 - f_stop1) / fs, (f_stop2 - f_pass2) / fs)
                        bands = [0, f_stop1, f_pass1, f_pass2, f_stop2, nyq]
                        desired = [0, 1, 0]
                        weights = [1.0, 1.0, 1.0]
                        fallback_cut = [f_pass1, f_pass2]
                        pass_zero = False
                    else:  # Bandstop
                        delta_f = min((f_stop1 - f_pass1) / fs, (f_pass2 - f_stop2) / fs)
                        bands = [0, f_pass1, f_stop1, f_stop2, f_pass2, nyq]
                        desired = [1, 0, 1]
                        weights = [1.0, 1.0, 1.0]
                        fallback_cut = [f_stop1, f_stop2]
                        pass_zero = True

                    try:
                        b = remez(numtaps, bands, desired, weight=weights, fs=fs)
                    except Exception:
                        b = firwin(numtaps, fallback_cut, pass_zero=pass_zero, fs=fs)
                else:
                    # FIR Window
                    cutoff = [cut1, cut2] if f_type in ["Bandpass", "Bandstop"] else cut1
                    pass_zero = (f_type not in ["Highpass", "Bandpass"])
                    b = firwin(numtaps, cutoff, pass_zero=pass_zero, fs=fs)

                layer['b'] = b
                layer['a'] = [1.0]
                w, h = freqz(b, [1.0], worN=num_points)
                # Zero-phase filtfilt: |h|^2
                total_h *= (np.abs(h) ** 2)
                has_freq_response = True
            except Exception as e:
                print(f"FIR response error: {e}")

        # ---------------- 3. EMA (Exponential Moving Average) ----------------
        elif "EMA" in impl:
            try:
                # Tính alpha từ cutoff frequency hoặc ema_alpha
                if 'ema_alpha' in layer and layer['ema_alpha'] > 0:
                    a1 = layer['ema_alpha']
                else:
                    a1 = alpha_from_cutoff(cut1, fs)

                a2 = alpha_from_cutoff(cut2, fs)
                passes = max(1, int(layer.get('order', 1)))

                # Đáp ứng tần số H_LP(e^jw) của EMA đơn lẻ
                # H_LP(z) = alpha / (1 - (1 - alpha)*z^-1)
                exp_jw = np.exp(-1j * w_axis)
                H_lp1 = a1 / (1.0 - (1.0 - a1) * exp_jw)
                H_lp2 = a2 / (1.0 - (1.0 - a2) * exp_jw)

                if f_type == "Lowpass":
                    H_ema = H_lp1
                elif f_type == "Highpass":
                    H_ema = 1.0 - H_lp1
                elif f_type == "Bandpass":
                    # Dual-EMA Difference
                    H_ema = H_lp2 - H_lp1
                else:  # Bandstop
                    H_ema = 1.0 - (H_lp2 - H_lp1)

                # Zero-phase bidirectional (forward-backward) -> |H|^2 qua mỗi pass
                H_ema_resp = (np.abs(H_ema) ** 2) ** passes
                total_h *= H_ema_resp
                has_freq_response = True
            except Exception as e:
                print(f"EMA response error: {e}")

        # ---------------- 4. Savitzky-Golay ----------------
        elif "Savitzky" in impl:
            try:
                win_len = int(layer.get('sg_window', 11))
                if win_len % 2 == 0:
                    win_len += 1
                poly = int(layer.get('sg_poly', 3))
                if poly >= win_len:
                    poly = win_len - 1
                b_sg = savgol_coeffs(win_len, poly)
                w, h = freqz(b_sg, [1.0], worN=num_points)
                total_h *= (np.abs(h) ** 2)
                has_freq_response = True
            except Exception as e:
                print(f"Savitzky response error: {e}")

    if has_freq_response:
        f_hz = (w_axis * fs) / (2.0 * np.pi)
        mag_db = 20.0 * np.log10(np.maximum(np.abs(total_h), 1e-5))
        phase_rad = np.unwrap(np.angle(total_h))
        return f_hz, mag_db, phase_rad, ideal_f, ideal_db
    return None, None, None, None, None


def apply_pipeline_filters(signal, filter_pipeline, fs=50.0):
    """
    Áp dụng toàn bộ chuỗi lọc trong pipeline lên mảng tín hiệu 1D numpy.
    Đảm bảo EMA, IIR, FIR, Savitzky, Hampel, Kalman đều hoạt động chuẩn xác theo fs và tham số.
    """
    filtered_signal = np.copy(signal).astype(float)
    n = len(filtered_signal)
    if n == 0:
        return filtered_signal

    fs = max(float(fs), 1.0)
    nyq = 0.5 * fs

    for layer in filter_pipeline:
        if not layer.get('is_active', True):
            continue

        impl = layer.get('impl', '')
        f_type = layer.get('type', 'Lowpass')
        order = int(layer.get('order', 4))
        cut1 = min(float(layer.get('cutoff1', 0.1)), nyq * 0.98)
        cut2 = min(float(layer.get('cutoff2', 0.5)), nyq * 0.98)

        # ---------------- 1. IIR (Butterworth) ----------------
        if "IIR" in impl:
            if layer.get('sos') is not None:
                padlen = min(3 * 2 * len(layer['sos']), len(filtered_signal) - 2)
                if padlen < 0:
                    padlen = 0
                filtered_signal = sosfiltfilt(layer['sos'], filtered_signal, padlen=padlen)

        # ---------------- 2. FIR ----------------
        elif "FIR" in impl:
            if layer.get('b') is not None:
                try:
                    filtered_signal = filtfilt(layer['b'], layer.get('a', [1.0]), filtered_signal)
                except ValueError:
                    padlen = max(0, len(filtered_signal) - 2)
                    if padlen > 0:
                        filtered_signal = filtfilt(layer['b'], layer.get('a', [1.0]), filtered_signal, padlen=padlen)

        # ---------------- 3. EMA (Exponential Moving Average) ----------------
        elif "EMA" in impl:
            # Lấy alpha từ tần số cắt hoặc từ cấu hình
            if 'ema_alpha' in layer and layer['ema_alpha'] > 0:
                a1 = layer['ema_alpha']
            else:
                a1 = alpha_from_cutoff(cut1, fs)

            a2 = alpha_from_cutoff(cut2, fs)
            passes = max(1, int(layer.get('order', 1)))

            if f_type == "Lowpass":
                filtered_signal = run_single_ema(filtered_signal, a1, passes=passes)
            elif f_type == "Highpass":
                ema_baseline = run_single_ema(filtered_signal, a1, passes=passes)
                filtered_signal = filtered_signal - ema_baseline
            elif f_type == "Bandpass":
                # Dual-EMA Bandpass: lọc thông nhanh trừ lọc thông chậm (khử DC + khử nhiễu)
                ema_slow = run_single_ema(filtered_signal, a1, passes=passes)
                ema_fast = run_single_ema(filtered_signal, a2, passes=passes)
                filtered_signal = ema_fast - ema_slow
            elif f_type == "Bandstop":
                ema_slow = run_single_ema(filtered_signal, a1, passes=passes)
                ema_fast = run_single_ema(filtered_signal, a2, passes=passes)
                filtered_signal = filtered_signal - (ema_fast - ema_slow)

        # ---------------- 4. Hampel Filter (Khử xung gai) ----------------
        elif "Hampel" in impl:
            window_size = int(layer.get('hampel_window', 7))
            if window_size % 2 == 0:
                window_size += 1
            n_sigmas = float(layer.get('hampel_sigma', 3.0))
            rolling_median = median_filter(filtered_signal, size=window_size)
            diff = np.abs(filtered_signal - rolling_median)
            rolling_mad = median_filter(diff, size=window_size)
            outliers = diff > (n_sigmas * 1.4826 * rolling_mad)
            filtered_signal = np.where(outliers, rolling_median, filtered_signal)

        # ---------------- 5. Kalman Filter ----------------
        elif "Kalman" in impl:
            R = float(layer.get('kalman_r', 0.1))
            Q = float(layer.get('kalman_q', 0.01))
            temp_sig = np.empty(n, dtype=float)
            x_est = filtered_signal[0]
            P_est = 1.0
            for i in range(n):
                x_pred = x_est
                P_pred = P_est + Q
                K = P_pred / (P_pred + R)
                x_est = x_pred + K * (filtered_signal[i] - x_pred)
                P_est = (1.0 - K) * P_pred
                temp_sig[i] = x_est
            filtered_signal = temp_sig

        # ---------------- 6. Savitzky-Golay (Làm mượt) ----------------
        elif "Savitzky" in impl:
            window_length = int(layer.get('sg_window', 11))
            if window_length % 2 == 0:
                window_length += 1
            if window_length >= n:
                window_length = max(3, n - 1)
                if window_length % 2 == 0:
                    window_length -= 1
            polyorder = int(layer.get('sg_poly', 3))
            if polyorder >= window_length:
                polyorder = max(1, window_length - 1)
            filtered_signal = savgol_filter(filtered_signal, window_length, polyorder)

    if np.any(np.isnan(filtered_signal)) or np.any(np.isinf(filtered_signal)):
        raise ValueError("Tín hiệu sau lọc chứa giá trị NaN hoặc Inf. Bộ lọc không ổn định.")

    return filtered_signal
