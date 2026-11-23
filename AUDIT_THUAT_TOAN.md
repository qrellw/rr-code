# AUDIT THUẬT TOÁN RR-Laser — Báo cáo kiểm tra toàn bộ

Ngày: 2026-11-21
Phạm vi: `Code/RR_Serial/RR_Serial.ino`, `Code/rr_app/core/serial_worker.py`,
`Code/rr_suite.py`, dữ liệu thật trong `rrdata/`.
Trạng thái: **Đã sửa hướng C** (§10) — ưu tiên 1 (làm `k` hoạt động đúng).
Các mục 1, 2, 5, 6, 7 vẫn còn, chờ quyết định hướng A hoặc B.

---

## 0. Kết luận ngắn

| # | Vấn đề | Mức độ | Bằng chứng |
|---|---|---|---|
| 1 | `k` bị kẹp `[0.5, 2.0]`, nhưng `k` tối ưu thực tế là **0,12 – 0,68** → hiệu chuẩn bị vô hiệu | **Nặng** | §3 |
| 2 | Công thức `k` hiện lấy từ phép sáng/tối, cho `k = 20,66` — sai 80 lần so với dữ liệu vận hành | **Nặng** | §4 |
| 3 | Chẩn đoán "Vb > Va" trong comment firmware là **sai bản chất** — không phải Vb lớn hơn, mà là **nền tối hai kênh khác nhau** | **Nặng** | §2 |
| 4 | Mẫu số `\|a\| + k·\|b\|` khác `a + k·b` khi `a,b` trái dấu — chỉ đúng khi cùng dấu. Dữ liệu hiện tại luôn cùng dấu nên vô hại, nhưng là bom hẹn giờ | Trung bình | §5 |
| 5 | Thiết bị **không phải bicell cân bằng**: `ptp(Va+Vb)/ptp(Va−Vb) = 0,4–1,6` (bicell lý tưởng phải ≪ 1) | Trung bình | §2.3 |
| 6 | Nhãn "[hít vào ↑ · thở ra ↓]" trong GUI là **khẳng định chưa kiểm chứng** | Trung bình | §6 |
| 7 | `LiveBreathRate(fs=45.0)` hard-code, nhưng Fs thực **46,6 Hz** → sai nhịp thở 3,5% | Nhẹ | §7 |
| 8 | `offline_fs` mặc định 47,6 Hz trong khi Fs thật 62,5 Hz (do `host_s` trùng lặp) | Nhẹ | §7 |

---

## 1. Công thức lý thuyết chuẩn

Nguồn: [Laser Focus World — Position-sensing detectors](https://www.laserfocusworld.com/detectors-imaging/article/16546909/position-sensing-detectors-fill-photonics-test-and-measurement-needs),
[US6624403 — bicell normalization](https://patentimages.storage.googleapis.com/33/6e/87/78db3d05510738/US6624403.pdf).

Với bi-cell (2 photodiode):

```
r = (A − B) / (A + B)
```

Trong đó **A, B là tín hiệu đã trừ nền tối** (photocurrent do ánh sáng, luôn ≥ 0), KHÔNG phải
điện áp thô.

Điều kiện để công thức có nghĩa:
1. `A + B` ≈ hằng số (tổng năng lượng không đổi khi tia di chuyển)
2. `A, B ≥ 0`
3. Nền tối phải trừ **riêng từng kênh**

Nguồn Ý ([malus_2015](https://osiris.df.unipi.it/~fuso/dida/malus_2015.pdf)) nói rõ: nền có thể
đo bằng cách **tắt laser và đọc photocurrent**.

---

## 2. Đối chiếu với code hiện tại

### 2.1 Firmware dùng `vbias` chung thay vì nền tối riêng

`RR_Serial.ino:310-312` (mode 1):

```c
float a = va1 - vbias;    // vbias = 0.228 V, đọc từ AIN2
float b = vb1 - vbias;    // DÙNG CHUNG một hằng số cho cả hai kênh
```

Nhưng nền tối **thực tế của từng kênh khác nhau** (đo trực tiếp, laser TẮT):

| | AIN0 (Va) | AIN1 (Vb) |
|---|---|---|
| Nền tối (laser TẮT) | 0,2304 V | **0,3276 V** |
| Khi sáng (laser BẬT) | 0,2636 V | 0,3292 V |
| Hiệu (tín hiệu thật) | 0,0332 V | 0,0016 V |

`vbias = 0,228` tình cờ gần đúng nền tối của Va (0,2304 → sai 1,1×),
nhưng **sai 63× với nền tối của Vb** (0,3276).

**Đây là lỗi gốc.** Comment firmware (`RR_Serial.ino:13-15`) ghi
*"thực đo cho thấy Vb lớn hơn Va"* — cách diễn giải đó **sai bản chất**. Vb không "lớn hơn" vì
nhận nhiều sáng hơn; Vb lớn hơn vì **offset nền của kênh B cao hơn 97 mV**. Đó là sai lệch
mạch (op-amp offset, dòng tối), không phải tín hiệu.

### 2.2 Hệ quả số học

Với `a = Va − 0,228`, `b = Vb − 0,228`:

```
b bị phóng đại 63 lần so với tín hiệu thật
→ r bị kéo về −0,48 … −0,95 thay vì ≈ 0 tại vị trí cân bằng
```

Đo trên file `rr_1_20261121_172952.csv`: `a ∈ [0,003; 0,039]`, `b ∈ [0,098; 0,141]`
→ `|b|/|a| = 3,79`. Cả hai đều **luôn dương**, nên trông "hợp lệ" nhưng tỉ lệ sai hoàn toàn.

### 2.3 Kiểm định mô hình bicell — KHÔNG đạt

Với bicell lý tưởng: `ptp(A+B) ≪ ptp(A−B)`.

Kết quả trên 9 file (300+ mẫu):

| File | ptp(Va+Vb) | ptp(Va−Vb) | Tỉ số | Kết luận |
|---|---|---|---|---|
| `rr_1_20261120_135357` | 0,0570 | 0,1430 | **0,399** | bicell |
| `rr_1_20261121_172952` | 0,0460 | 0,0710 | 0,648 | không rõ |
| `rr_1_20261121_140334` | 0,9730 | 1,7150 | 0,567 | không rõ |
| `rr_1_20261121_173310` | 0,0080 | 0,0070 | **1,143** | không phải |
| `rr_1_20261121_173601` | 0,0110 | 0,0070 | **1,571** | không phải |

Chỉ **1/9 file** hành xử như bicell thật. Phần lớn tổng năng lượng biến thiên mạnh
ngang hiệu → nguồn sáng thay đổi cường độ, hoặc cả hai ptd nhận cùng biến thiên.
**Không thể coi `A+B` là hằng số**, nên công thức chuẩn hóa mất cơ sở.

---

## 3. Lỗi then chốt: dải kẹp `[0.5, 2.0]` vô hiệu hóa hiệu chuẩn

`RR_Serial.ino:194-198`:

```c
if (k < 0.5f || k > 2.0f) {
    // ... giu k = 1.0
    k = 1.0f;
}
```

Dải này **do tôi tự đặt, không có cơ sở đo**. Giá trị `k` đúng (tính từ dữ liệu vận hành)
nằm NGOÀI dải này ở mọi file:

| File | mean(a) | mean(b) | **k tối ưu** | k bị kẹp thành |
|---|---|---|---|---|
| `rr_1_20261121_172952` | +0,0282 | +0,1067 | **0,2640** | 1,0 |
| `rr_1_20261121_173104` | +0,0308 | +0,1051 | **0,2929** | 1,0 |
| `rr_1_20261120_090426` | +0,0755 | +0,6269 | **0,1204** | 1,0 |
| `rr_1_20261121_140334` | +0,2058 | +0,3020 | **0,6814** | 1,0 |

→ **Mọi lần bấm "Hiệu chuẩn" đều bị đẩy về `k = 1,0`**, tức không có tác dụng gì.
Đây chính là hiện tượng bạn quan sát và gọi là "làm màu".

### Hiệu quả của `k` đúng

Với `k = mean(a)/mean(b)` — tức `k` làm trung bình tử số bằng 0:

| File | k=1,0 (hiện tại) | | k tối ưu | |
|---|---|---|---|---|
| | r tb | ptp | r tb | ptp |
| `172952` | **+0,5900** | 0,496 | **+0,0616** | **1,007** |
| `173104` | +0,5509 | 0,544 | **+0,0323** | **1,054** |
| `090426` | +0,7482 | 0,811 | **+0,0700** | **1,416** |
| `140334` | +0,2339 | 1,724 | +0,1237 | 1,782 |

`r` trung bình về gần 0, **độ phân giải tăng gấp đôi**, bão hoà về 0%.

**Kết luận:** câu hỏi của bạn *"ấn hiệu chuẩn thì nó phải về 0 bất kể tia sáng đang đâu chứ?"* —
**ĐÚNG**, và điều đó **khả thi**, nhưng chỉ khi `k` được tính đúng và không bị kẹp.

---

## 4. `k` từ phép sáng/tối là sai 80 lần

`calibrateK()` (`RR_Serial.ino:162-202`) tính:

```
ia = Va_sáng − Va_tối = 0,0332 V
ib = Vb_sáng − Vb_tối = 0,0016 V
k = ia/ib = 20,66
```

Nhưng từ dữ liệu vận hành: `k ≈ 0,26`. **Chênh 80 lần.**

Nguyên nhân `ib` không đáng tin:

- `ib = 0,0016 V` = **13 LSB** của ADS1115 (gain ±4,096 V → LSB 125 µV). Nằm trong nhiễu.
- `SETTLE_MS = 45` ms; mạch RC 15 Hz → τ = 10,6 ms → 45 ms = 4,2τ. Đủ về lý thuyết,
  nhưng khi tín hiệu chỉ 13 LSB thì nhiễu chi phối.
- `calibrateK()` chỉ lấy **N = 16 mẫu** mỗi pha, không lọc, không loại outlier.
- Đo ở hai thời điểm khác nhau → ánh sáng môi trường có thể đã đổi.

→ `k = ia/ib` là phép chia hai số nhỏ, **khuếch đại sai số cực mạnh**. Không dùng được.

**Khuyến nghị:** tính `k = mean(a)/mean(b)` từ chính dữ liệu vận hành, trung bình trên
vài giây (1–2 chu kỳ thở). Đây là cách tự hiệu chỉnh về 0 đúng nghĩa.

---

## 5. Mẫu số `|a| + k·|b|` — đúng hay sai?

`RR_Serial.ino:78-87`:

```c
float num = kFactor * b - a;
float den = fabsf(a) + kFactor * fabsf(b);
```

So với lý thuyết `(A − B)/(A + B)` với `A,B ≥ 0`: nếu `a, b` cùng dương thì
`|a| + k|b| = a + k·b` → khớp. **Dữ liệu hiện tại luôn cùng dương** (`a ∈ [0,003; 0,039]`,
`b ∈ [0,098; 0,141]`) nên hiện **không gây hại**.

Nhưng khi tia lệch hẳn sang một bên, `a` hoặc `b` có thể âm, lúc đó `|·|` làm sai nghĩa vật lý
của "tổng năng lượng". Đây là **bom hẹn giờ**, không phải lỗi đang hoạt động.

**Khuyến nghị:** sau khi đã trừ đúng nền tối, `a` và `b` sẽ luôn ≥ 0, khi đó bỏ dấu `|·|`
và dùng `den = a + k*b` cho đúng lý thuyết; chỉ cần bảo vệ `den < ε`.

---

## 6. Chiều "hít vào / thở ra" — CHƯA KIỂM CHỨNG

GUI hiển thị tiêu đề:

```
Tín hiệu Nhịp thở r = (k·Vb - Va)/(Va + k·Vb)   [hít vào ↑ · thở ra ↓]
```

Nhãn này **do tôi thêm vào**, dựa trên suy đoán, **không có ground-truth**. Không có
spirometer hay bất kỳ tham chiếu độc lập nào trong dữ liệu.

Những gì **đã** kiểm chứng được:
- `corr(r, Va) ≈ +0,98` → `r` tăng khi `Va` tăng (theo định nghĩa công thức).
- Nhưng **`Va` tăng khi hít vào hay thở ra thì chưa ai đo.**

**Khuyến nghị:** bỏ nhãn khẳng định, hoặc đổi thành trung tính. Chỉ ghi "↑/↓" sau khi
đối chiếu với một phép đo độc lập (băng ngực, spirometer, hoặc đếm thở bằng mắt).

---

## 7. Sai số tần số lấy mẫu

| Chỗ | Khai báo | Thực tế | Sai |
|---|---|---|---|
| `LiveBreathRate(fs=45.0)` (`serial_worker.py:103`) | 45,0 Hz | **46,63 Hz** | −3,5% |
| `BandpassRBJ(fs=45.0)` (`serial_worker.py:46`) | 45,0 Hz | 46,63 Hz | −3,5% → tần số cắt lệch |
| `offline_fs = 47.6` (`rr_suite.py:83`) | 47,6 Hz | **62,5 Hz** | −24% |

Đo thực: 234 dòng / 5,02 s = **46,63 mẫu/s**, trung vị Δt = 21 ms.

Riêng `offline_fs = 47,6` bị lệch nhiều vì `host_s` trong CSV có giá trị trùng lặp
(nhiều mẫu cùng timestamp) → `1/median(diff)` cho 62,5 Hz. Cần dùng `t_ms` thay vì `host_s`.

---

## 8. Đề xuất sửa (theo thứ tự ưu tiên)

### Ưu tiên 1 — làm `k` hoạt động đúng (giải quyết trực tiếp yêu cầu của bạn)

1. **Bỏ dải kẹp** `[0.5, 2.0]`. Thay bằng kiểm tra hợp lệ rộng: `0.02 < k < 50`,
   ngoài dải đó mới đặt 1,0 và **báo lỗi rõ ràng** (không im lặng).
2. **Đổi cách tính `k`**: thay phép sáng/tối bằng trung bình dữ liệu vận hành:

   ```c
   // Trong calibrateK(): đo ~3 giây ở mode 1, laser đang bật
   float sum_a = 0, sum_b = 0;
   int n = 0;
   for (uint32_t t0 = millis(); millis() - t0 < 3000; ) {
       float va = readCh(0), vb = readCh(1);
       sum_a += va - vbias;
       sum_b += vb - vbias;
       n++;
       delay(PERIOD_1_MS);
   }
   float k = sum_a / sum_b;   // lam trung binh tu so = 0 -> r trung binh = 0
   ```

3. Kết quả: `r` trung bình ≈ 0 tại vị trí hiện tại → **đúng điều bạn muốn**, và độ phân giải
   tăng gấp đôi (§3).

### Ưu tiên 2 — sửa gốc: nền tối riêng từng kênh

1. Thêm `va_dark`, `vb_dark` lưu vào NVS.
2. Hiệu chuẩn: tắt laser → đo `va_dark, vb_dark`; bật → đo `va_lit, vb_lit`.
3. Mode 1 dùng `a = Va − va_dark`, `b = Vb − vb_dark` thay cho `vbias` chung.
4. Sửa lại comment đầu file: bỏ khẳng định sai "Vb > Va", ghi đúng bản chất
   **offset nền kênh B cao hơn 97 mV**.

### Ưu tiên 3 — tính chính trực

1. Bỏ hoặc sửa nhãn "[hít vào ↑ · thở ra ↓]" (§6).
2. Nếu chọn hướng "thiết bị là single-cell" (vì `ib` chỉ 5% `ia`), thì phải nói rõ trong
   báo cáo khoa học rằng **thiết bị không đo được vị trí bicell**, chỉ đo cường độ phản xạ.
   Đây là khác biệt bản chất, ảnh hưởng đến phần "phương pháp" của luận văn.

### Ưu tiên 4 — chỉnh nhỏ

1. `LiveBreathRate(fs)` và `BandpassRBJ(fs)` nên nhận Fs thật từ GUI (46,6 Hz), không hard-code 45.
2. `offline_fs` nên tính từ `t_ms` (ms → s) thay vì `host_s`.

---

## 9. Câu hỏi cần bạn quyết định

Trước khi tôi sửa, cần biết **hướng thiết kế** vì nó đổi bản chất luận văn:

**Hướng A — Sửa để bicell hoạt động thật.**
- Cần chỉnh quang học cho chùm tia phủ đều hai ptd.
- Chỉ khả thi nếu `ptp(A+B) ≪ ptp(A−B)` — hiện chỉ 1/9 file đạt.
- Nếu chọn: tôi sửa firmware theo Ưu tiên 1 + 2, sau đó bạn chỉnh cơ khí, đo lại.

**Hướng B — Công nhận là cảm biến cường độ (single-cell).**
- Bỏ `k`, bỏ bicell. Dùng trực tiếp biến thiên `Va` (hoặc `Va+Vb`) làm tín hiệu nhịp thở.
- Đơn giản, trung thực với phần cứng thực tế, không cần chỉnh quang.
- Nhưng mất chức năng "đo vị trí" — phải viết lại phần phương pháp trong báo cáo.

**Hướng C — Chỉ sửa `k` (Ưu tiên 1), giữ nguyên phần còn lại.**
- `r` sẽ về 0 khi bấm hiệu chuẩn, độ phân giải tăng gấp đôi.
- Nhanh, đúng yêu cầu trước mắt, nhưng không sửa gốc (§2.1) — nền tối vẫn sai,
  chỉ được `k` bù tạm.

Tôi khuyến nghị **C trước** (để bạn có dữ liệu dùng được ngay), rồi **A hoặc B** sau khi
bạn xác định hướng luận văn.

---

## 10. ĐÃ SỬA — Hướng C (ưu tiên 1)

Thực hiện ngày 2026-11-21, đã nạp và kiểm chứng trên board thật.

### Thay đổi trong `RR_Serial.ino`

| Chỗ | Trước | Sau |
|---|---|---|
| `calibrateK()` | Đo sáng/tối, `k = ia/ib` → ra 20,66 | Đo 3 s dữ liệu vận hành, `k = mean(a)/mean(b)` |
| Dải kẹp khi hiệu chuẩn | `[0.5, 2.0]` → đẩy về 1,0 | Hợp lệ `(0.02, 50)`; ngoài đó **báo lỗi**, không âm thầm đổi |
| Kiểm tra `k` lúc khởi động | `[0.5, 2.0]` → xoá về 1,0 | `(0.02, 50)` → giữ `k` đã hiệu chuẩn |
| Cảnh báo `k` lệch 1 | "CANH BAO: k lech xa 1 → gõ r" (sai: `k` đúng *phải* lệch 1) | Chỉ nhắc bấm `c` khi `k` vẫn = 1,0 |
| Comment đầu file | "thực đo cho thấy Vb > Va" (sai bản chất) | Ghi rõ: offset nền kênh B cao hơn 97 mV |
| Hàm mới `sampleAB()` | — | Lấy `(a,b)` đúng như `loop()` đang tính, hỗ trợ cả mode 0/1 |

### Thay đổi trong `rr_suite.py`

- Thêm `on_calibrate_k()`: khoá nút, hiện "Đang hiệu chuẩn (3s)...", ghi log, mở khoá sau 4,2 s.
  Trước đây bấm nút không có phản hồi nào trong 3 giây firmware bận.
- Nút "Hiệu chuẩn k" có tooltip giải thích.

### Kết quả đo thật trên board (COM8)

```
TRUOC hieu chuan : r tb = -0.2584
                # CAL OK: k = 1.5214  (mean(a)=0.1415 mean(b)=0.0930 n=75)
SAU hieu chuan   : r tb = +0.0546
```

Lặp lại 2 lần liên tiếp — đều cho `r` quanh 0:

| Lần | `k` tính được | `r` tb sau đó |
|---|---|---|
| 1 | 1,7236 | +0,0864 |
| 2 | 1,3255 | +0,0334 |

**Lưu qua reset: ĐẠT.** `k = 1,3255` trước và sau khi reset board giống hệt.

Kiểm chứng qua GUI (bấm nút thật, chụp màn hình): `r` tb `+0,1492 → +0,0368`,
đồ thị dao động quanh đường 0, không bão hoà (0/361 mẫu).

### Việc còn lại (chưa làm)

- **§2.1** Nền tối riêng từng kênh (`va_dark`, `vb_dark`) — ưu tiên 2, thuộc hướng A.
  Hiện `k` đang **bù tạm** cho sai lệch này, không sửa gốc.
- **§6** Nhãn "[hít vào ↑ · thở ra ↓]" vẫn là quy ước chưa kiểm chứng.
- **§7** `LiveBreathRate(fs=45.0)` hard-code (Fs thật 46,63 Hz) → nhịp thở sai 3,5%.
- **§7** `offline_fs = 47.6` nên tính từ `t_ms` thay vì `host_s`.

---

## 10b. ĐÃ SỬA — Hướng A (ưu tiên 2): nền tối riêng từng kênh

Thực hiện ngày 2026-11-21 sau khi đo được nền tối thật (nhờ đèn chiếu thử).

### Số đo quyết định

| | Nền tối (không ánh sáng) |
|---|---|
| `Va` (AIN0) | **228,0 mV** |
| `Vb` (AIN1) | **309,6 mV** |
| `vbias` firmware cũ dùng chung | **226,6 mV** |

Hai kênh lệch nhau **81,6 mV**, nhưng code cũ trừ **cùng một** `vbias`. Nó khớp AIN0
nhưng sai **83 mV** với AIN1 → `b` luôn dương giả 83 mV.

### Hậu quả đo được trên dữ liệu thật (`vb_test.csv`)

| | Lúc KHÔNG có tín hiệu | Lúc có ánh sáng |
|---|---|---|
| Cách cũ (`vbias` chung) | `a`=1,4 mV, `b`=83,0 mV → **r = 0,9668** | r = [−0,156 … +0,975], **0 %** mẫu r<−0,9 |
| Cách mới (nền riêng) | `a`=0,0 mV, `b`=0,0 mV → **r = 0,0288** | r = [−1,000 … **+1,000**], **14,7 %** mẫu r<−0,9 |

Đối chiếu lý thuyết: tia chỉ vào A → `r = −1`; giữa → `r = 0`; chỉ vào B → `r = +1`. Cách mới đúng.

### Thay đổi trong `RR_Serial.ino`

- Thêm `vaDark`, `vbDark`, `darkValid`, lưu NVS (`vaD`, `vbD`, `darkOK`).
- `baselineAB()`: lấy `a = Va − vaDark`, `b = Vb − vbDark`; nếu chưa hiệu chuẩn thì
  tạm dùng `vbias` chung như cũ.
- `calibrateK()` thành 2 pha: **pha 1** tắt laser đo nền tối từng kênh (48 mẫu);
  **pha 2** bật laser đo 3 s dữ liệu vận hành, `k = mean(a)/mean(b)`.
- `DIAG` in thêm nền tối và độ lệch giữa hai kênh.

### Kiểm chứng trên board

```
#      nen toi: AIN0=0.2282 AIN1=0.3095 V  (da hieu chuan, lech 81.2 mV)
r : tb=+0.0000  [+0.000 .. +0.000]   (943 mau, khi khong co anh sang)
```

`r = 0,0000` chính xác khi không có tín hiệu — trước khi sửa là **0,97**.

### Chặn lại: laser không phát sáng

```
Bật/tắt laser -> Va đổi 0,0 mV, Vb đổi 0,2 mV
Va = 228,0 = đúng nền tối, Vb = 309,5 ≈ nền 309,6
# CAL THAT BAI: k = -0.0066  (mean(a)=0.0000 mean(b)=-0.0013)
```

Không có ánh sáng nào tới ptd → không hiệu chuẩn `k` được, không có tín hiệu thở.
Cần kiểm tra laser của thiết bị (GPIO0, module laser, pin).

---

## 11. Vì sao `r` bám `Va` chứ không theo `Vb` (đo ngày 2026-11-21, file `rr_1_20261121_231549.csv`)

Quan sát trên GUI: `Vb` dao động nhanh (ringing) trong khi `r` đi theo dạng bậc của `Va`.

### Phổ tần — kênh nào mang tín hiệu thở

| Kênh | Năng lượng 0,1–0,6 Hz (thở) | Năng lượng 2–10 Hz (ringing) | Tỉ số ringing/thở |
|---|---|---|---|
| `Va` | 1,00e-4 (đỉnh 0,27 Hz) | 3,19e-6 | **0,03** → sạch |
| `Vb` | 1,27e-6 | 8,47e-5 (đỉnh 5,07 Hz) | **66,6** → toàn nhiễu |
| `r` | 2,46e-2 | 1,93e-3 | 0,08 |

`Vb` có năng lượng ringing **gấp 66 lần** năng lượng dải thở → `Vb` hầu như **không chứa
tín hiệu thở**, chỉ chứa nhiễu 5,07 Hz.

### Ringing của `Vb` là nhiễu điện riêng kênh B, KHÔNG phải chùm tia rung

Nếu là chùm tia di chuyển trên bicell thì `Va` và `Vb` phải **ngược pha** trong dải đó.
Đo được:

```
corr(Va, Vb) trong dải 2–8 Hz = −0,018     -> độc lập hoàn toàn
biên độ tại 5,07 Hz: Va = 1,06 mV, Vb = 42,46 mV  -> Vb gấp 40 lần
```

Tương quan ≈ 0 → hai kênh dao động **độc lập** → đây là nhiễu riêng của kênh B
(dây/che chắn/trở kháng đầu vào), không phải tín hiệu quang.

### Vì sao `r` bám `Va` — hai lý do cộng lại

**Lý do 1 (vật lý):** `Va` mang tín hiệu thở thật, `Vb` chỉ mang nhiễu.

**Lý do 2 (toán học):** `a = Va − vbias` là **số dư nhỏ** nên độ nhạy tương đối rất cao:

| | khoảng giá trị | trung bình | biến thiên tương đối | đóng góp vào `r` (ptp) |
|---|---|---|---|---|
| `a` | 0,0020 – 0,0550 V | 0,036 V | **147 %** | **1,102** |
| `b` | 0,0640 – 0,1120 V | 0,086 V | 56 % | 0,278 |

`a` ảnh hưởng lên `r` **gấp 4 lần** `b`.

Hệ quả: `corr(r, Va) = −0,987`, `corr(r, Vb) = +0,191`.

### Kết luận

`r` bám `Va` là **hành vi đúng**, không phải lỗi. Nếu `r` bám `Vb` thì đồ thị sẽ toàn
nhiễu 5 Hz. Nhưng nó xác nhận thêm **§2.3**: kênh B không mang thông tin vị trí, nên
thiết bị hiện **không hoạt động như bicell**.

Hướng xử lý (chờ quyết định):
1. **Kiểm tra phần cứng kênh B** — dây AIN1, che chắn, tụ lọc tại chân ADS1115.
   Nếu hết nhiễu thì mới đánh giá lại được bicell.
2. **Bỏ `Vb` khỏi công thức** (hướng B) — `r` sẽ sạch hơn vì không còn cộng nhiễu 5 Hz.
3. **Lọc thông thấp `Vb`** trước khi tính `r` — rẻ, giảm nhiễu nhưng không khôi phục
   thông tin vị trí.
