/* =====================================================================
   RR-Laser v5 - doc tin hieu, in CSV ra Serial
   ESP32-C3 SuperMini | ADS1115 | 2x BPW34S (bi-cell) | laser 650nm

   Dung thu vien Adafruit_ADS1X15 (giong ESP32_Firmware.ino cua repo Firebase).

   Chan (theo schematic):  SDA=GPIO8  SCL=GPIO7  LASER_EN=GPIO0  VBAT=GPIO21
   ADS1115  0x48 (ADDR=GND):  AIN0=Va  AIN1=Vb  AIN2=Vbias  AIN3=GND

   Thuat toan:
       a = Va - Vbias ,  b = Vb - Vbias
       r = (k*b - a) / (|a| + k*|b|)
     k duoc hieu chuan bang lenh 'c': chon k = mean(a)/mean(b) tren 3s du lieu
     dang chay, lam r dao dong quanh 0 tai vi tri tia hien tai.

   LUU Y VE NEN (da do thuc te, xem AUDIT_THUAT_TOAN.md):
     nen toi AIN0 = 0.2304 V, nen toi AIN1 = 0.3276 V -> kenh B co offset cao
     hon 97 mV. Vb lon hon Va KHONG PHAI vi nhan nhieu sang hon (tin hieu laser
     that cua B chi ~0.0016 V), ma do offset nen cua kenh B. Dung vbias chung
     0.228 V lam b bi phong dai ~63 lan; k bu duoc phan lech nay.
     mode 1 (mac dinh): laser sang lien tuc, ~45 Hz
     mode 0           : 2 pha sang/toi, ~8 Hz, tru mau toi (dung nen that)

   Moi dong ra Serial 115200, 9 so:
       t_ms, Va, Vb, Va0, Vb0, Vbias, r, Vbat_mV, flags
   flags: bit0 loi ADS, bit1 bao hoa (>3.25V), bit2 Vbias chua do duoc,
          bit3 pin yeu

   Phim:
       1  laser lien tuc (~55 Hz)     0  2 pha (tru mau toi)
       l  bat/tat laser               c  hieu chuan k
       k  in k                        r  dat lai k = 1
       d  chan doan                   b  tim bus I2C
       s  quet I2C                    ?  tro giup
   ===================================================================== */

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Preferences.h>

// ---------------- chan ----------------
static const int PIN_SDA      = 8;
static const int PIN_SCL      = 7;
static const int PIN_LASER_EN = 0;
static const int PIN_VBAT     = 21;
static const bool VBAT_MEASURABLE = false;   // C3 khong co ADC o GPIO21

// ---------------- thong so ----------------
static const uint16_t SETTLE_MS    = 45;     // cho RC 15 Hz xac lap
static const uint16_t PERIOD_1_MS  = 22;     // chu ky mode 1 (~45 Hz)
static const float MIN_SIGNAL_V = 0.020f;
static const float SAT_V        = 3.25f;
static const float VBIAS_NOM    = 0.228f;

Adafruit_ADS1115 ads;

static float   kFactor = 1.0f;
static int     mode    = 1;
static bool    laserOn = true;
static bool    rawMode = false;   // false = in CSV 9 so (mac dinh); true = in AIN0..3
static bool    quadMode = false;  // true = in nhanh ca 4 chan, de chan doan nhieu
static float   vaDark  = 0.228f;   // nen toi AIN0 (do khi tat laser)
static float   vbDark  = 0.310f;   // nen toi AIN1 (do khi tat laser)
static bool    darkValid = false;  // da do duoc nen that chua
static float   vbias   = VBIAS_NOM;
static bool    vbiasValid = false;
static bool    batLow  = false;
static uint16_t vbat   = 0;
static bool    adsOk   = false;
static Preferences prefs;

// ---------------- doc kenh ----------------
static float readCh(uint8_t ch) {
  return ads.computeVolts(ads.readADC_SingleEnded(ch));
}

// ---------------- ti so chuan hoa r ----------------
// a = Va - NEN_RIENG_A ,  b = Vb - NEN_RIENG_B
//   r = (k*b - a) / (|a| + k*|b|)
//
// QUAN TRONG: phai tru nen RIENG tung kenh. Do thuc te (2026-11-21):
//     nen toi AIN0 = 228.0 mV,  nen toi AIN1 = 309.6 mV
//   hai kenh lech nhau 81.6 mV. Ban cu tru CHUNG vbias = 226.6 mV, khop voi
//   AIN0 nhung sai 83 mV voi AIN1 -> b luon duong gia ~83 mV -> r khong bao
//   gio ve am, khong bao gio dat -1, va luon lech duong.
//   Do la nguyen nhan goc, khong phai do cong thuc.
static float calcRatio(float a, float b) {
  if (fabsf(a) + fabsf(b) < MIN_SIGNAL_V) return 0.0f;   // chua du tin hieu
  float num = kFactor * b - a;
  float den = fabsf(a) + kFactor * fabsf(b);
  if (den < 1e-6f) return 0.0f;
  float v = num / den;
  if (v > 1.0f)  v = 1.0f;
  if (v < -1.0f) v = -1.0f;
  return v;
}

// ---------------- chan doan ----------------
static void i2cScan() {
  Serial.print("# I2C:");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02X", a);
      n++;
    }
  }
  if (n == 0) {
    Serial.print(" (khong thay thiet bi nao!)");
  } else {
    Wire.beginTransmission(0x48);
    if (Wire.endTransmission() != 0) Serial.print("  <- thieu 0x48 (ADS1115)");
  }
  Serial.println();
}

static void help() {
  Serial.println("# 1 = lien tuc | 0 = 2 pha | l = laser | c = cal k | k = in k");
  Serial.println("# r = k ve 1 | d = chan doan | t = test laser 5s | s = quet I2C | x = in AIN | q = doc nhanh 4 chan | z = in CSV | ? = tro giup");
}

static void dumpDiag() {
  if (!adsOk) {
    Serial.println("# DIAG: ADS1115 khong tra loi (go 's' de quet I2C)");
    return;
  }
  bool wasOn = laserOn;
  laserOn = true;
  digitalWrite(PIN_LASER_EN, HIGH);
  delay(SETTLE_MS + 5);

  float va = readCh(0);
  float vb = readCh(1);
  float vc = readCh(2);

  laserOn = wasOn;
  if (!wasOn) digitalWrite(PIN_LASER_EN, LOW);

  float a, b;
  baselineAB(va, vb, a, b);
  float r = calcRatio(a, b);

  Serial.printf("# DIAG va=%.4f vb=%.4f  a=%.4f b=%.4f  k=%.4f  r=%+.4f\n",
                va, vb, a, b, kFactor, r);
  Serial.printf("#      nen toi: AIN0=%.4f AIN1=%.4f V  (%s, lech %.1f mV)\n",
                vaDark, vbDark, darkValid ? "da hieu chuan" : "MAC DINH",
                (vbDark - vaDark) * 1000.0f);
  Serial.printf("#      AIN2 doc truc tiep = %.4f V (phai ~0.228)\n", vc);
  Serial.printf("#      vbias dang dung    = %.4f V  (%s)\n", vbias,
                vbiasValid ? "da do duoc" : "MAC DINH, chua do duoc");
}

// Bat/tat laser theo nhip 5s, in AIN0/AIN1 -> tach "laser yeu dan" khoi "ro nhiet"
static void laserStepTest() {
  Serial.println("# TEST 60s: laser BAT 5s / TAT 5s. Xem AIN0 co bam theo khong.");
  for (int k = 0; k < 6; k++) {
    digitalWrite(PIN_LASER_EN, HIGH);
    for (int i = 0; i < 5; i++) {
      Serial.printf("ON   AIN0=%+.4f  AIN1=%+.4f\n", readCh(0), readCh(1));
      delay(1000);
    }
    digitalWrite(PIN_LASER_EN, LOW);
    for (int i = 0; i < 5; i++) {
      Serial.printf("OFF  AIN0=%+.4f  AIN1=%+.4f\n", readCh(0), readCh(1));
      delay(1000);
    }
  }
  digitalWrite(PIN_LASER_EN, laserOn ? HIGH : LOW);
  Serial.println("# TEST xong");
}

// ---------------- hieu chuan k + nen toi ----------------
// Thay (a,b) cua kenh theo nen toi rieng tung kenh.
static void baselineAB(float va1, float vb1, float &a, float &b) {
  if (darkValid) {
    a = va1 - vaDark;
    b = vb1 - vbDark;
  } else {
    // Chua do nen that: tam dung vbias chung nhu ban cu (kem chinh xac hon)
    a = va1 - vbias;
    b = vb1 - vbias;
  }
}

// Lay mau (a, b) dung nhu loop() dang tinh, de k tinh ra ap dung duoc ngay.
static void sampleAB(float &a, float &b) {
  float va1, vb1;
  if (mode == 0) {
    digitalWrite(PIN_LASER_EN, LOW);
    delay(SETTLE_MS);
    float va0 = readCh(0), vb0 = readCh(1);
    digitalWrite(PIN_LASER_EN, HIGH);
    delay(SETTLE_MS);
    va1 = readCh(0); vb1 = readCh(1);
    digitalWrite(PIN_LASER_EN, laserOn ? HIGH : LOW);
    a = va1 - va0;
    b = vb1 - vb0;
  } else {
    digitalWrite(PIN_LASER_EN, laserOn ? HIGH : LOW);
    va1 = readCh(0); vb1 = readCh(1);
    baselineAB(va1, vb1, a, b);
  }
}

// Hieu chuan gom 2 pha:
//   Pha 1 - TAT laser, do nen toi RIENG tung kenh -> vaDark, vbDark.
//   Pha 2 - BAT laser, do 3s du lieu van hanh -> chon k = mean(a)/mean(b),
//           tuc lam trung binh tu so bang 0, r dao dong quanh 0 tai vi tri
//           tia hien tai (bat ke tia dang chieu vao dau bicell).
static void calibrateK() {
  if (!adsOk) {
    Serial.println("# CAL THAT BAI: ADS1115 khong tra loi");
    return;
  }

  const int      DARK_N    = 48;     // so mau do nen toi
  const uint32_t WINDOW_MS = 3000;   // thoi gian do du lieu van hanh
  const int      MAX_N     = 400;
  const int      MIN_N     = 20;

  // ---------- Pha 1: nen toi ----------
  Serial.println("# CAL: pha 1/2 - tat laser de do nen toi tung kenh...");
  bool wasOn = laserOn;
  digitalWrite(PIN_LASER_EN, LOW);
  delay(SETTLE_MS + 30);            // cho RC xac lap sau khi tat laser

  float sva = 0, svb = 0;
  for (int i = 0; i < DARK_N; i++) {
    sva += readCh(0);
    svb += readCh(1);
    delay(2);
  }
  float nva = sva / DARK_N;
  float nvb = svb / DARK_N;

  vaDark = nva;
  vbDark = nvb;
  darkValid = true;

  // ---------- Pha 2: du lieu van hanh ----------
  digitalWrite(PIN_LASER_EN, wasOn ? HIGH : LOW);
  delay(SETTLE_MS + 30);
  Serial.println("# CAL: pha 2/2 - do 3s du lieu (giu yen, tho deu)...");

  float sumA = 0.0f, sumB = 0.0f;
  int   n    = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < WINDOW_MS && n < MAX_N) {
    float a, b;
    sampleAB(a, b);
    sumA += a;
    sumB += b;
    n++;
    if (mode == 1) delay(PERIOD_1_MS);
  }

  if (n < MIN_N) {
    Serial.printf("# CAL THAT BAI: chi lay duoc %d mau (can >= %d)\n", n, MIN_N);
    return;
  }

  float ma = sumA / n;
  float mb = sumB / n;

  if (fabsf(mb) < 1e-4f) {
    Serial.printf("# CAL THAT BAI: mean(b) = %.6f qua nho, khong chia duoc\n", mb);
    Serial.printf("#   nen toi: AIN0=%.4f AIN1=%.4f\n", vaDark, vbDark);
    Serial.println("#   -> kiem tra AIN1 (Vb) co tin hieu khong, bang lenh 'd'");
    return;
  }

  float k = ma / mb;

  if (!(k > 0.02f && k < 50.0f)) {
    Serial.printf("# CAL THAT BAI: k = %.4f ngoai vung hop le (0.02 .. 50)\n", k);
    Serial.printf("#   mean(a)=%.4f  mean(b)=%.4f  n=%d\n", ma, mb, n);
    return;
  }

  kFactor = k;
  prefs.putFloat("k",     kFactor);
  prefs.putFloat("vaD",   vaDark);
  prefs.putFloat("vbD",   vbDark);
  prefs.putBool ("darkOK", darkValid);

  Serial.printf("# CAL OK: k = %.4f\n", kFactor);
  Serial.printf("#   nen toi: AIN0=%.4f V  AIN1=%.4f V  (lech %.1f mV)\n",
                vaDark, vbDark, (vbDark - vaDark) * 1000.0f);
  Serial.printf("#   mean(a)=%.4f V  mean(b)=%.4f V  n=%d\n", ma, mb, n);
  Serial.println("#   a,b da tru nen rieng -> r chay duoc ca am lan duong");
}

// ---------------- setup ----------------
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_LASER_EN, OUTPUT);
  digitalWrite(PIN_LASER_EN, LOW);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  delay(10);

  prefs.begin("rrlaser", false);
  kFactor = prefs.getFloat("k", 1.0f);
  vaDark  = prefs.getFloat("vaD", 0.228f);
  vbDark  = prefs.getFloat("vbD", 0.310f);
  darkValid = prefs.getBool("darkOK", false);
  // Ban cu kep k trong [0.5, 2.0] -> moi gia tri hieu chuan dung deu bi day ve
  // 1.0. Gio chi loai gia tri vo ly, giu nguyen k da hieu chuan.
  if (!(kFactor > 0.02f && kFactor < 50.0f)) {
    Serial.printf("# CANH BAO: k luu trong NVS = %.4f khong hop le -> dat lai 1.0\n", kFactor);
    kFactor = 1.0f;
    prefs.putFloat("k", kFactor);
  }

  Serial.println();
  Serial.println("# ===== RR-Laser v5 (Adafruit_ADS1X15) =====");
  Serial.println("# SDA=GPIO8  SCL=GPIO7  LASER=GPIO0  VBAT=GPIO21");
  Serial.printf("# laser: %s  ('l' de bat/tat)\n", laserOn ? "BAT" : "TAT");
  Serial.println("# Dang in CSV 9 so (go 'x' de xem AIN0..3, 'z' de quay lai CSV)");
  help();

  // co mat tren bus?
  Wire.beginTransmission(0x48);
  adsOk = (Wire.endTransmission() == 0);

  ads.begin(0x48);                 // ban thu vien cu: 2 tham so
  Wire.begin(PIN_SDA, PIN_SCL);    // phong khi begin() doi chan ve mac dinh
  Wire.setClock(100000);
  ads.setGain(GAIN_ONE);           // +-4.096V

  i2cScan();

  if (adsOk) {
    Serial.printf("# ADS1115 OK, k = %.4f\n", kFactor);
    if (darkValid) {
      Serial.printf("# nen toi da hieu chuan: AIN0=%.4f  AIN1=%.4f V (lech %.1f mV)\n",
                    vaDark, vbDark, (vbDark - vaDark) * 1000.0f);
    } else {
      Serial.println("# CHUA hieu chuan nen -> dang dung vbias chung (kem chinh xac)");
      Serial.println("#   Go 'c' de hieu chuan ca nen toi lan k");
    }
  } else {
    Serial.println("# ADS1115 KHONG TRA LOI!");
    Serial.println("#   kiem tra 3V3_ANA, SDA=GPIO8, SCL=GPIO7, ADDR noi GND");
    Serial.println("#   LUU Y: cot Vbias trong CSV la hang so 0.228, KHONG phai so do");
  }
}

// ---------------- loop ----------------
void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '0') { mode = 0; Serial.println("# mode 0: 2 pha"); }
    else if (c == '1') { mode = 1; Serial.println("# mode 1: lien tuc"); }
    else if (c == 'l' || c == 'L') {
      laserOn = !laserOn;
      Serial.printf("# laser %s\n", laserOn ? "BAT" : "TAT");
    }
    else if (c == 'c' || c == 'C') calibrateK();
    else if (c == 'k' || c == 'K') Serial.printf("# k = %.4f\n", kFactor);
    else if (c == 'r' || c == 'R') {
      kFactor = 1.0f;
      prefs.putFloat("k", kFactor);
      Serial.println("# k da dat lai = 1.0000");
    }
    else if (c == 'd' || c == 'D') dumpDiag();
    else if (c == 't' || c == 'T') laserStepTest();
    else if (c == 'x' || c == 'X') {
      rawMode = true;
      quadMode = false;
      Serial.println("# in AIN0..3 (1 giay/lan)");
    }
    else if (c == 'q' || c == 'Q') {
      quadMode = true;
      rawMode = false;
      Serial.println("# Q: doc nhanh ca 4 chan (~45 Hz) de chan doan nhieu");
    }
    else if (c == 'z' || c == 'Z') {
      rawMode = false;
      quadMode = false;
      Serial.println("# in CSV");
    }
    else if (c == 's' || c == 'S') i2cScan();
    else if (c == '?' || c == 'h' || c == 'H') help();
  }

  uint32_t t0 = millis();
  float va1 = 0, vb1 = 0, va0 = 0, vb0 = 0, r = 0;
  uint8_t f = 0;

  if (!adsOk) {
    f |= 0x01;
  } else if (mode == 0) {
    digitalWrite(PIN_LASER_EN, LOW);
    delay(SETTLE_MS);
    va0 = readCh(0); vb0 = readCh(1);

    digitalWrite(PIN_LASER_EN, HIGH);
    delay(SETTLE_MS);
    va1 = readCh(0); vb1 = readCh(1);
    digitalWrite(PIN_LASER_EN, laserOn ? HIGH : LOW);

    float a = va1 - va0;
    float b = vb1 - vb0;
    r = calcRatio(a, b);
  } else {
    digitalWrite(PIN_LASER_EN, laserOn ? HIGH : LOW);
    va1 = readCh(0); vb1 = readCh(1);

    float a, b;
    baselineAB(va1, vb1, a, b);
    r = calcRatio(a, b);
  }

  if (r > 1.0f)  r = 1.0f;
  if (r < -1.0f) r = -1.0f;
  if (va1 > SAT_V || vb1 > SAT_V) f |= 0x02;
  if (!vbiasValid) f |= 0x04;
  if (batLow)      f |= 0x08;

  // ----- Vbias + pin: 1 giay/lan -----
  static uint32_t lastSlow = 0;
  if (millis() - lastSlow > 1000) {
    lastSlow = millis();

    if (adsOk) {
      float v = readCh(2);
      if (v > 0.15f && v < 0.35f) { vbias = v; vbiasValid = true; }
    }

    if (VBAT_MEASURABLE) {
      uint32_t mv = 0;
      for (int i = 0; i < 4; i++) mv += analogReadMilliVolts(PIN_VBAT);
      vbat = (uint16_t)((mv / 4) * 2);
    } else {
      vbat = 0;
    }
    batLow = (VBAT_MEASURABLE && vbat < 3400);
  }

  // ----- in -----
  if (rawMode) {
    // in AIN0..3 moi giay, giong het file Test_ADS_Lib
    static uint32_t lastRaw = 0;
    if (millis() - lastRaw > 1000) {
      lastRaw = millis();
      float v2 = adsOk ? readCh(2) : 0.0f;
      float v3 = adsOk ? readCh(3) : 0.0f;
      Serial.printf("AIN0=%+.4f  AIN1=%+.4f  AIN2=%+.4f  AIN3=%+.4f\n",
                    va1, vb1, v2, v3);
    }
  } else if (quadMode) {
    // Che do chan doan nhanh: doc DU 4 chan moi vong (~45 Hz) de so sanh.
    // AIN2 = Vbias (nguon on dinh), AIN3 = GND. Neu ca 4 chan cung dao dong
    // mot tan so -> nhieu nam trong chip/board. Neu chi AIN1 dao dong -> nhieu
    // nam o day/nguon photodiode B.
    float q1 = adsOk ? readCh(1) : 0.0f;
    float q2 = adsOk ? readCh(2) : 0.0f;
    float q3 = adsOk ? readCh(3) : 0.0f;
    Serial.printf("Q,%lu,%.4f,%.4f,%.4f,%.4f\n",
                  (unsigned long)millis(), va1, q1, q2, q3);
  } else {
    Serial.printf("%lu,%u,%u,%u,%u,%u,%.4f,%u,%u\n",
                  (unsigned long)millis(),
                  (unsigned)lroundf(va1 * 1000.0f),
                  (unsigned)lroundf(vb1 * 1000.0f),
                  (unsigned)lroundf(va0 * 1000.0f),
                  (unsigned)lroundf(vb0 * 1000.0f),
                  (unsigned)lroundf(vbias * 1000.0f),
                  r, (unsigned)vbat, (unsigned)f);
  }

  // ----- giu nhip -----
  if (mode == 0) {
    uint32_t period = 2 * SETTLE_MS + 2 * 25;   // 2 lan settle + 4 lan doc
    uint32_t elapsed = millis() - t0;
    if (elapsed < period) delay(period - elapsed);
  } else {
    uint32_t elapsed = millis() - t0;
    if (elapsed < PERIOD_1_MS) delay(PERIOD_1_MS - elapsed);
  }
}
