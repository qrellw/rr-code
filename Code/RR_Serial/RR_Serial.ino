/* =====================================================================
   RR-Laser v5 - doc tin hieu, in CSV ra Serial
   ESP32-C3 SuperMini | ADS1115 | 2x BPW34S (bi-cell) | laser 650nm

   Dung thu vien Adafruit_ADS1X15 (giong ESP32_Firmware.ino cua repo Firebase).

   Chan (theo schematic):  SDA=GPIO8  SCL=GPIO7  LASER_EN=GPIO0  VBAT=GPIO21
   ADS1115  0x48 (ADDR=GND):  AIN0=Va  AIN1=Vb  AIN2=Vbias  AIN3=GND

   Thuat toan:
       a = Va - Vbias ,  b = Vb - Vbias
       r = (a - k*b) / (a + k*b)        voi k ~ 1
     mode 1 (mac dinh): laser sang lien tuc, ~55 Hz
     mode 0           : 2 pha sang/toi, ~8 Hz, tru mau toi

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
  Serial.println("# r = k ve 1 | d = chan doan | t = test laser 5s | s = quet I2C | x = in AIN | z = in CSV | ? = tro giup");
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

  float a = va - vbias;
  float b = vb - vbias;
  float den = a + kFactor * b;
  float r = (fabsf(den) > 1e-6f) ? (a - kFactor * b) / den : 0.0f;

  Serial.printf("# DIAG va=%.4f vb=%.4f  a=%.4f b=%.4f  vbias=%.4f  den=%.4f  k=%.4f  r=%+.4f\n",
                va, vb, a, b, vbias, den, kFactor, r);
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

// ---------------- hieu chuan k ----------------
static void calibrateK() {
  const int N = 16;
  if (!adsOk) {
    Serial.println("# CAL THAT BAI: ADS1115 khong tra loi");
    return;
  }
  Serial.println("# CAL: bat dau (giu chum giua khe)...");

  float da = 0, db = 0;
  digitalWrite(PIN_LASER_EN, LOW);
  delay(SETTLE_MS + 20);
  for (int i = 0; i < N; i++) { da += readCh(0); db += readCh(1); }
  da /= N; db /= N;

  float oa = 0, ob = 0;
  digitalWrite(PIN_LASER_EN, HIGH);
  delay(SETTLE_MS + 20);
  for (int i = 0; i < N; i++) { oa += readCh(0); ob += readCh(1); }
  oa /= N; ob /= N;
  digitalWrite(PIN_LASER_EN, laserOn ? HIGH : LOW);

  float ia = oa - da, ib = ob - db;
  if (ia <= 0.002f || ib <= 0.002f) {
    Serial.printf("# CAL THAT BAI: tin hieu qua nho (Va=%.3f Vb=%.3f, nen=%.3f/%.3f)\n",
                  oa, ob, da, db);
    Serial.println("#   -> chum phai phu CA HAI ptd, laser phai sang");
    return;
  }
  float k = ia / ib;
  if (k < 0.2f) k = 0.2f;
  if (k > 5.0f) k = 5.0f;
  kFactor = k;
  prefs.putFloat("k", kFactor);
  Serial.printf("# CAL OK: k = %.4f  (Ia=%.4f V, Ib=%.4f V)\n", kFactor, ia, ib);
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
  if (!(kFactor > 0.1f && kFactor < 10.0f)) kFactor = 1.0f;

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
    if (fabsf(kFactor - 1.0f) > 0.25f)
      Serial.println("# CANH BAO: k lech xa 1 -> go 'r' de dat lai");
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
      Serial.println("# in AIN0..3");
    }
    else if (c == 'z' || c == 'Z') {
      rawMode = false;
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
    float den = a + kFactor * b;
    if (fabsf(a) + fabsf(b) < MIN_SIGNAL_V || fabsf(den) < 1e-6f) r = 0;
    else r = (a - kFactor * b) / den;
  } else {
    digitalWrite(PIN_LASER_EN, laserOn ? HIGH : LOW);
    va1 = readCh(0); vb1 = readCh(1);

    float a = va1 - vbias;
    float b = vb1 - vbias;
    float den = a + kFactor * b;
    if (fabsf(a) + fabsf(b) < MIN_SIGNAL_V || fabsf(den) < 1e-6f) r = 0;
    else r = (a - kFactor * b) / den;
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
