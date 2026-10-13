// Network: use WiFiS3 for UNO R4, WiFi for ESP32
#ifdef ESP32
#include <WiFi.h>
#include <WiFiUdp.h>
#else
#include <WiFiS3.h>  // for Arduino UNO R4 WiFi
#include <WiFiUdp.h>
#endif
#include <EEPROM.h>

// Watchdog: WDT is Renesas UNO R4 specific — exclude on ESP32
#ifndef ESP32
#include <WDT.h>   // Renesas UNO R4 Watchdog
#include <WiFiUdp.h>
#endif

// WiFi access data
const char* ssid     = "your_ssid";
const char* password = "your_password";
WiFiServer server(80);

WiFiUDP udp;

// NTP
const char* NTP_SERVER = "pool.ntp.org";
const unsigned int NTP_PORT = 123;
const int NTP_PACKET_SIZE = 48;
byte ntpPacket[NTP_PACKET_SIZE];

bool timeValid = false;
uint32_t epochUtc = 0;                 // UTC seconds
uint32_t epochAtSyncUtc = 0;
unsigned long msAtSync = 0;
unsigned long lastNtpSyncMs = 0;
unsigned long lastNtpAttemptMs = 0;

const unsigned long NTP_SYNC_INTERVAL_MS = 60UL * 60UL * 1000UL; // 1 hour

// --- relay board pin assignment ---
const int RELAY_PUMPE   = 2;
const int RELAY_BRENNER = 3;

// --- NTC sensors pin assignment ---
const int PIN_VORLAUF_ADC = A0; 
const int PIN_AUSSEN_ADC  = A1; 
const int PIN_ROOM_ADC    = A2;      // neuer NTC für Raumtemperatur
const int PIN_SUPPLY_A    = 4;  
const int PIN_SUPPLY_V    = 5;  
const int PIN_SUPPLY_ROOM = 6;       // Spannung für Raum-NTC

// --- measuring intervals ---
unsigned long lastMeasure = 0;
const unsigned long MEASURE_INTERVAL = 3000;

// --- ADC Setup ---
const float ADC_MAX = 4095.0;

// --- fixed resistors (defaults) ---
#define DEFAULT_R_FIXED_AUSSEN  (1508.0 * 1.019)
#define DEFAULT_R_FIXED_VORLAUF (2240.0 * 1.014)
#define DEFAULT_R_FIXED_ROOM    (1800.0 * 1.019)

// runtime variables (may be loaded from EEPROM on boot)
float R_fixed_Aussen  = DEFAULT_R_FIXED_AUSSEN;
float R_fixed_Vorlauf = DEFAULT_R_FIXED_VORLAUF;
float R_fixed_Room    = DEFAULT_R_FIXED_ROOM;

// --- VT min/max defaults (use same define style) ---
#define DEFAULT_VT_MIN  12.0f
#define DEFAULT_VT_MAX  90.0f

// runtime VT bounds (loaded from EEPROM at boot)
float VT_min_runtime = DEFAULT_VT_MIN;
float VT_max_runtime = DEFAULT_VT_MAX;

// --- Hampel thresholds defaults (same define style) ---
#define DEFAULT_HAMPEL_A 1.3f
#define DEFAULT_HAMPEL_V 2.7f
#define DEFAULT_HAMPEL_R 1.5f

// runtime hampel thresholds (loaded from EEPROM at boot)
float hampelA_runtime = DEFAULT_HAMPEL_A;
float hampelV_runtime = DEFAULT_HAMPEL_V;
float hampelR_runtime = DEFAULT_HAMPEL_R;

// --- Steinhart-Hart coefficients ---
const float SH_A_AUSSEN = 0.016207324099999998;
const float SH_B_AUSSEN = -0.00300974648;
const float SH_C_AUSSEN = 0.0000240442873;

const float SH_A_VORLAUF = 0.008139253781875624;
const float SH_B_VORLAUF = -0.0012092224103120086;
const float SH_C_VORLAUF = 0.00000932119289641137;

const float SH_A_ROOM = 0.0014976326882427393f;
const float SH_B_ROOM = 0.0002557655307135374f;
const float SH_C_ROOM = -0.000000036013063680625744f; // ≈ -3.6013e-08

// --- smoothing ---
#define SMOOTH_COUNT 5
float aussenBuffer[SMOOTH_COUNT];
float vorlaufBuffer[SMOOTH_COUNT];
float roomBuffer[SMOOTH_COUNT];
int smoothIndex = 0;

// --- global variables ---
float tAussen = NAN, rAussen = NAN;
float tVorlauf = NAN, rVorlauf = NAN;
float tRoom = NAN, rRoom = NAN;
float vtCurve = NAN;
float vtCorrected = NAN;

// --- states ---
bool pumpeAn   = true;
bool brennerAn = false;

// --- operating mode ---
bool manualOverride = false;   

// --- Heat curve selection (8 curves) ---
struct Curve {
  float A, B, C, D;
};

// Curves I, II, III, 1, 3, 5, 7, 8
const Curve curves[8] = {
   { 0.00004, -0.0056,  -0.2979, 28.573 },   // 0 = I
   { -0.0002, -0.0096,  -0.4874, 35.011 },   // 1 = II
   { -0.0002, -0.0121,  -0.6602, 40.203 },   // 2 = III
   { -0.0004, -0.0163,  -0.7421, 44.851 },   // 3 = 1
   { -0.0003, -0.0174,  -0.9478, 48.892 },   // 4 = 3 (your old default)
   { 0.00002, -0.0153,  -1.1745, 52.816 },   // 5 = 5
   { -0.0003, -0.0181,  -1.4718, 61.491 },   // 6 = 7
   { 0.004,   -0.12,    -1.9,     82.0   }   // 7 = 8
};

uint8_t curveIndex = 4;  // Start with curve 3 (index 4)
const int EEPROM_ADDR_CURVE = 4;

// --- room temperature control ---
float roomTarget = 21.0;      
const float ROOM_KP = 3.0;    
const float ROOM_CORR_MAX = 63.0;

// --- Timer schedule (inside automatic mode) ---
bool timerEnabled = false;

// defaults
#define DEFAULT_TIMER_DAY_TARGET   21.0f
#define DEFAULT_TIMER_NIGHT_TARGET 18.0f
#define DEFAULT_TIMER_DAY_START_MIN   (6*60)   // 06:00
#define DEFAULT_TIMER_NIGHT_START_MIN (22*60)  // 22:00

float    timerDayTarget   = DEFAULT_TIMER_DAY_TARGET;
float    timerNightTarget = DEFAULT_TIMER_NIGHT_TARGET;
uint16_t timerDayStartMin   = DEFAULT_TIMER_DAY_START_MIN;
uint16_t timerNightStartMin = DEFAULT_TIMER_NIGHT_START_MIN;

// computed effective target used by control
float roomTarget_effective = 21.0f;   // will be updated regularly

// --- control parameters ---
#define DEFAULT_BURNER_HYST 4.0f   // 4 K standard

// runtime hysteresis (loaded from EEPROM at boot)
float burnerHyst_runtime = DEFAULT_BURNER_HYST;

const float PUMP_OFF_OUTDOOR_DELTA = 1.0;   // pump OFF if tAussen >= roomTarget + 1°C
const float PUMP_ON_OUTDOOR_DELTA  = 1.0;   // pump ON  if tAussen <= roomTarget - 1°C

// --- EEPROM (addresses) ---
// note: avoid overlapping with existing addresses used elsewhere (0.. etc.)
const int EEPROM_ADDR_ROOMTARGET = 0; // existing
const int EEPROM_ADDR_CURVE_IDX  = EEPROM_ADDR_CURVE; // 4 existing
const int EEPROM_ADDR_ROOMMODE   = 5;   // single byte existing

// calibration floats: use addresses far enough (4 bytes each)
const int EEPROM_ADDR_RFIX_A = 10; // float at 10..13
const int EEPROM_ADDR_RFIX_V = 14; // float at 14..17
const int EEPROM_ADDR_RFIX_R = 18; // float at 18..21

// VT min/max floats: follow calibration addresses
const int EEPROM_ADDR_VT_MIN = 22; // 22..25
const int EEPROM_ADDR_VT_MAX = 26; // 26..29

// Hampel thresholds addresses (user approved)
const int EEPROM_ADDR_HAMP_A = 30; // 30..33
const int EEPROM_ADDR_HAMP_V = 34; // 34..37
const int EEPROM_ADDR_HAMP_R = 38; // 38..41

// Burner hysteresis EEPROM
const int EEPROM_ADDR_BURNER_HYST = 42; // float at 42..45

// --- Timer EEPROM (new) ---
// 46: enabled byte
// 47..50: day target float
// 51..54: night target float
// 55..56: day start minutes uint16
// 57..58: night start minutes uint16
const int EEPROM_ADDR_TIMER_EN      = 46;
const int EEPROM_ADDR_TIMER_DAY_T   = 47;
const int EEPROM_ADDR_TIMER_NIGHT_T = 51;
const int EEPROM_ADDR_TIMER_DAY_M   = 55;
const int EEPROM_ADDR_TIMER_NIGHT_M = 57;

float lastSavedRoomTarget = NAN;
bool roomMode = true; // true = use room sensor (default ON)

struct TimerParams {
  bool en;
  float day;
  float night;
  uint16_t dm;
  uint16_t nm;
};

bool parseTimerParams(char* req, TimerParams& out) {
  out.en = timerEnabled;              // fallback = current values
  out.day = timerDayTarget;
  out.night = timerNightTarget;
  out.dm = timerDayStartMin;
  out.nm = timerNightStartMin;

  char* pen = strstr(req, "EN=");
  if (pen) out.en = (atoi(pen + 3) != 0);

  char* pday = strstr(req, "DAY=");
  if (pday) {
    float v = atof(pday + 4);
    if (v >= 2.0f && v <= 30.0f) out.day = v;
  }

  char* pnight = strstr(req, "NIGHT=");
  if (pnight) {
    float v = atof(pnight + 6);
    if (v >= 2.0f && v <= 30.0f) out.night = v;
  }

  char* pdm = strstr(req, "DSTART=");
  if (pdm) {
    int m = atoi(pdm + 7);
    if (m >= 0 && m <= 1439) out.dm = (uint16_t)m;
  }

  char* pnm = strstr(req, "NSTART=");
  if (pnm) {
    int m = atoi(pnm + 7);
    if (m >= 0 && m <= 1439) out.nm = (uint16_t)m;
  }

  return true;
}

// --- debug memory display ---
#ifdef ESP32
#include <esp_heap_caps.h>
int freeMemory() {
  // Use heap_caps_get_free_size for accurate free heap on ESP32
  return (int)heap_caps_get_free_size(MALLOC_CAP_8BIT);
}
#else
extern "C" char *sbrk(int);
int freeMemory() {
  char top;
  return &top - sbrk(0);
}
#endif

// ---------- EEPROM helpers ----------
float eeprom_getFloatOrNan(int addr) {
  float val = NAN;
  EEPROM.get(addr, val);
  return val;
}

// If savedVal is invalid (NaN) or too small (< 1.0), return default
float loadOrDefault(float savedVal, float defaultVal) {
  if (isnan(savedVal)) return defaultVal;
  // for resistances and temps we treat <1.0 as "not set"
  if (savedVal < 1.0f) return defaultVal;
  return savedVal;
}

// --- load/save/reset calibration (resistors) ---
void loadCalibrationFromEEPROM() {
  float sa = eeprom_getFloatOrNan(EEPROM_ADDR_RFIX_A);
  float sv = eeprom_getFloatOrNan(EEPROM_ADDR_RFIX_V);
  float sr = eeprom_getFloatOrNan(EEPROM_ADDR_RFIX_R);
  R_fixed_Aussen  = loadOrDefault(sa, DEFAULT_R_FIXED_AUSSEN);
  R_fixed_Vorlauf = loadOrDefault(sv, DEFAULT_R_FIXED_VORLAUF);
  R_fixed_Room    = loadOrDefault(sr, DEFAULT_R_FIXED_ROOM);
}

// **Option 1 behavior:** save to EEPROM only — do NOT change runtime copies immediately.
// They get applied on next reboot because loadCalibrationFromEEPROM() runs in setup().
void saveCalibrationToEEPROM(float aVal, float vVal, float rVal) {
  EEPROM.put(EEPROM_ADDR_RFIX_A, aVal);
  EEPROM.put(EEPROM_ADDR_RFIX_V, vVal);
  EEPROM.put(EEPROM_ADDR_RFIX_R, rVal);
  // do NOT update runtime variables here (Option 1)
  // On ESP32 EEPROM writes require commit()
#ifdef ESP32
  EEPROM.commit();
#endif
}

void resetCalibrationInEEPROM() {
  float marker = 0.0f; // indicates "unset"
  EEPROM.put(EEPROM_ADDR_RFIX_A, marker);
  EEPROM.put(EEPROM_ADDR_RFIX_V, marker);
  EEPROM.put(EEPROM_ADDR_RFIX_R, marker);
  // do NOT change runtime variables now; changes take effect after next reboot
#ifdef ESP32
  EEPROM.commit();
#endif
}

// --- load/save/reset VT bounds ---
// load at boot
void loadVTboundsFromEEPROM() {
  float smin = eeprom_getFloatOrNan(EEPROM_ADDR_VT_MIN);
  float smax = eeprom_getFloatOrNan(EEPROM_ADDR_VT_MAX);
  VT_min_runtime = loadOrDefault(smin, DEFAULT_VT_MIN);
  VT_max_runtime = loadOrDefault(smax, DEFAULT_VT_MAX);
  // Ensure runtime values are within allowed absolute constraints
  if (VT_min_runtime < 12.0f) VT_min_runtime = DEFAULT_VT_MIN;
  if (VT_max_runtime > 90.0f) VT_max_runtime = DEFAULT_VT_MAX;
  if (VT_min_runtime >= VT_max_runtime) { // fallback to defaults if inconsistent
    VT_min_runtime = DEFAULT_VT_MIN;
    VT_max_runtime = DEFAULT_VT_MAX;
  }
}

// Option 1: write to EEPROM only; do not update runtime values here.
void saveVTboundsToEEPROM(float minVal, float maxVal) {
  EEPROM.put(EEPROM_ADDR_VT_MIN, minVal);
  EEPROM.put(EEPROM_ADDR_VT_MAX, maxVal);
  // do NOT call loadVTboundsFromEEPROM() here (Option 1)
#ifdef ESP32
  EEPROM.commit();
#endif
}

void resetVTboundsInEEPROM() {
  float marker = 0.0f;
  EEPROM.put(EEPROM_ADDR_VT_MIN, marker);
  EEPROM.put(EEPROM_ADDR_VT_MAX, marker);
  // do NOT change runtime bounds now; changes will apply after reboot
#ifdef ESP32
  EEPROM.commit();
#endif
}

// --- load/save/reset Hampel thresholds ---
// load at boot
void loadHampelFromEEPROM() {
  float sa = eeprom_getFloatOrNan(EEPROM_ADDR_HAMP_A);
  float sv = eeprom_getFloatOrNan(EEPROM_ADDR_HAMP_V);
  float sr = eeprom_getFloatOrNan(EEPROM_ADDR_HAMP_R);
  // Use loadOrDefault but note thresholds could be small (<1) so we treat <0.05 as unset.
  // We'll implement a loader that accepts reasonable thresholds (>=0.05).
  auto useOrDefault = [](float saved, float def) {
    if (isnan(saved)) return def;
    if (saved < 0.05f) return def; // treat too small as unset
    return saved;
  };
  hampelA_runtime = useOrDefault(sa, DEFAULT_HAMPEL_A);
  hampelV_runtime = useOrDefault(sv, DEFAULT_HAMPEL_V);
  hampelR_runtime = useOrDefault(sr, DEFAULT_HAMPEL_R);
}

// Option 1 save: write to EEPROM only; do not update runtime copies here.
void saveHampelToEEPROM(float aVal, float vVal, float rVal) {
  EEPROM.put(EEPROM_ADDR_HAMP_A, aVal);
  EEPROM.put(EEPROM_ADDR_HAMP_V, vVal);
  EEPROM.put(EEPROM_ADDR_HAMP_R, rVal);
  // do NOT update runtime values now (Option 1)
#ifdef ESP32
  EEPROM.commit();
#endif
}

void resetHampelInEEPROM() {
  float marker = 0.0f;
  EEPROM.put(EEPROM_ADDR_HAMP_A, marker);
  EEPROM.put(EEPROM_ADDR_HAMP_V, marker);
  EEPROM.put(EEPROM_ADDR_HAMP_R, marker);
  // do NOT change runtime values now; changes apply after reboot
#ifdef ESP32
  EEPROM.commit();
#endif
}

// --- load/save/reset burner hysteresis ---
// load at boot
void loadBurnerHystFromEEPROM() {
  float s = eeprom_getFloatOrNan(EEPROM_ADDR_BURNER_HYST);
  burnerHyst_runtime = loadOrDefault(s, DEFAULT_BURNER_HYST);

  // enforce allowed range
  if (burnerHyst_runtime < 4.0f || burnerHyst_runtime > 15.0f) {
    burnerHyst_runtime = DEFAULT_BURNER_HYST;
  }
}

// save EEPROM only (Option 1)
void saveBurnerHystToEEPROM(float val) {
  EEPROM.put(EEPROM_ADDR_BURNER_HYST, val);
#ifdef ESP32
  EEPROM.commit();
#endif
}

// reset EEPROM (defaults apply after reboot)
void resetBurnerHystInEEPROM() {
  float marker = 0.0f;
  EEPROM.put(EEPROM_ADDR_BURNER_HYST, marker);
#ifdef ESP32
  EEPROM.commit();
#endif
}

// ---------- existing EEPROM save/load functions (room target etc.) ----------
void saveRoomTarget(float val) {
  float saved = NAN;
  EEPROM.get(EEPROM_ADDR_ROOMTARGET, saved);
  bool storedInvalid = isnan(saved) || (saved < 2.0) || (saved > 30.0);
  if (storedInvalid || fabs(saved - val) > 0.05) {
    EEPROM.put(EEPROM_ADDR_ROOMTARGET, val);
    lastSavedRoomTarget = val;
#ifdef ESP32
    EEPROM.commit();
#endif
  } else {
    lastSavedRoomTarget = saved;
  }
}

void loadRoomTarget() {
  float saved = NAN;
  EEPROM.get(EEPROM_ADDR_ROOMTARGET, saved);
  if (saved >= 2.0 && saved <= 30.0) {
    roomTarget = saved;
    lastSavedRoomTarget = saved;
  } else {
    lastSavedRoomTarget = NAN;
  }
}

void saveCurveIndex(uint8_t idx) {
#ifdef ESP32
  if (idx <= 7) EEPROM.write(EEPROM_ADDR_CURVE_IDX, idx);
  EEPROM.commit();
#else
  if (idx <= 7) EEPROM.update(EEPROM_ADDR_CURVE_IDX, idx);
#endif
}

void loadCurveIndex() {
  uint8_t idx = EEPROM.read(EEPROM_ADDR_CURVE_IDX);
  if (idx <= 7) curveIndex = idx;
}

void saveRoomMode(bool mode) {
#ifdef ESP32
  EEPROM.write(EEPROM_ADDR_ROOMMODE, mode ? 1 : 0);
  EEPROM.commit();
#else
  EEPROM.update(EEPROM_ADDR_ROOMMODE, mode ? 1 : 0);
#endif
}

void loadRoomMode() {
  uint8_t v = EEPROM.read(EEPROM_ADDR_ROOMMODE);
  if (v == 0) roomMode = false;
  else if (v == 1) roomMode = true;
  else roomMode = true; // default ON if no valid value
}

// ---------- Timer EEPROM helpers ----------
void loadTimerFromEEPROM() {
  uint8_t en = EEPROM.read(EEPROM_ADDR_TIMER_EN);
  if (en == 0 || en == 1) timerEnabled = (en == 1);
  else timerEnabled = false; // default OFF if invalid

  float d = eeprom_getFloatOrNan(EEPROM_ADDR_TIMER_DAY_T);
  float n = eeprom_getFloatOrNan(EEPROM_ADDR_TIMER_NIGHT_T);

  // temps valid range
  if (!isnan(d) && d >= 2.0f && d <= 30.0f) timerDayTarget = d;
  else timerDayTarget = DEFAULT_TIMER_DAY_TARGET;

  if (!isnan(n) && n >= 2.0f && n <= 30.0f) timerNightTarget = n;
  else timerNightTarget = DEFAULT_TIMER_NIGHT_TARGET;

  uint16_t dm, nm;
  EEPROM.get(EEPROM_ADDR_TIMER_DAY_M, dm);
  EEPROM.get(EEPROM_ADDR_TIMER_NIGHT_M, nm);

  if (dm != 0xFFFF && dm <= 1439) timerDayStartMin = dm;
  else timerDayStartMin = DEFAULT_TIMER_DAY_START_MIN;

  if (nm != 0xFFFF && nm <= 1439) timerNightStartMin = nm;
  else timerNightStartMin = DEFAULT_TIMER_NIGHT_START_MIN;
  }

void saveTimerToEEPROM(int en, float day, float night, uint16_t dm, uint16_t nm) {

  // enable flag (always safe)
#ifdef ESP32
  EEPROM.write(EEPROM_ADDR_TIMER_EN, en ? 1 : 0);
#else
  EEPROM.update(EEPROM_ADDR_TIMER_EN, en ? 1 : 0);
#endif

  // floats: use marker 0.0 like calibration
  if (isnan(day)) day = 0.0f;
  if (isnan(night)) night = 0.0f;

  EEPROM.put(EEPROM_ADDR_TIMER_DAY_T, day);
  EEPROM.put(EEPROM_ADDR_TIMER_NIGHT_T, night);

  // minutes: use 0xFFFF as marker
  if (dm == 0xFFFF) EEPROM.put(EEPROM_ADDR_TIMER_DAY_M, (uint16_t)0xFFFF);
  else EEPROM.put(EEPROM_ADDR_TIMER_DAY_M, dm);

  if (nm == 0xFFFF) EEPROM.put(EEPROM_ADDR_TIMER_NIGHT_M, (uint16_t)0xFFFF);
  else EEPROM.put(EEPROM_ADDR_TIMER_NIGHT_M, nm);
#ifdef ESP32
  EEPROM.commit();
#endif
}

// --- Hilfsfunktionen ---
void setPump(bool on) {
  pumpeAn = on;
  digitalWrite(RELAY_PUMPE, on ? HIGH : LOW);
}

void setBrenner(bool on) {
  brennerAn = on;
  digitalWrite(RELAY_BRENNER, on ? LOW : HIGH);
}

float resistanceFromADC(int adcValue, float Rfixed) {
  if (adcValue <= 0) return 1e12;
  if (adcValue >= ADC_MAX) return 0.0;
  float ratio = (float)adcValue / (ADC_MAX - (float)adcValue);
  return Rfixed * ratio;
}

float ntcTempC_from_R_SH(float Rntc, float A, float B, float C) {
  if (Rntc <= 0.0) return NAN;
  float lnR = log(Rntc);
  float invT = A + B * lnR + C * lnR * lnR * lnR;
  if (invT <= 0.0) return NAN;
  return (1.0f / invT) - 273.15f;
}

int readADCwithSupply(int supplyPin, int adcPin, int settleMs = 100) {
  pinMode(supplyPin, OUTPUT);
  digitalWrite(supplyPin, HIGH);
  delay(settleMs);
  analogRead(adcPin);
  long sum=0;
  const int N=6;
  for (int i=0;i<N;i++) { sum += analogRead(adcPin); delay(5); }
  int raw = sum / N;
  digitalWrite(supplyPin, LOW);
  delay(5);
  pinMode(supplyPin, INPUT);
  return raw;
}

// --- smoothing helpers ---
float smoothMedian(float *buf, int count) {
  float tmp[count];
  memcpy(tmp, buf, count * sizeof(float));
  for (int i = 0; i < count - 1; i++)
    for (int j = i + 1; j < count; j++)
      if (tmp[j] < tmp[i]) { float t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t; }
  return tmp[count / 2];
}

float hampelFilter(float current, float *buf, int count, float threshold) {
  int validCount = 0;
  float tmp[count];
  for (int i=0;i<count;i++) if (!isnan(buf[i])) tmp[validCount++] = buf[i];
  if (validCount == 0) return current;
  for (int i=0;i<validCount-1;i++)
    for (int j=i+1;j<validCount;j++)
      if (tmp[j]<tmp[i]) { float t=tmp[i]; tmp[i]=tmp[j]; tmp[j]=t; }
  float med = tmp[validCount/2];
  if (fabs(current - med) > threshold) return med;
  return current;
}

// --- Temperaturmessung ---
void measureTemps() {
  // --- Außenfühler ---
  int rawA = readADCwithSupply(PIN_SUPPLY_A, PIN_AUSSEN_ADC);
  rAussen  = resistanceFromADC(rawA, R_fixed_Aussen);
  float tA = ntcTempC_from_R_SH(rAussen, SH_A_AUSSEN, SH_B_AUSSEN, SH_C_AUSSEN);

  // --- Vorlauffühler ---
  int rawV = readADCwithSupply(PIN_SUPPLY_V, PIN_VORLAUF_ADC);
  rVorlauf = resistanceFromADC(rawV, R_fixed_Vorlauf);
  float tV = ntcTempC_from_R_SH(rVorlauf, SH_A_VORLAUF, SH_B_VORLAUF, SH_C_VORLAUF);

  // --- Raumfühler ---
  int rawR = readADCwithSupply(PIN_SUPPLY_ROOM, PIN_ROOM_ADC);
  rRoom = resistanceFromADC(rawR, R_fixed_Room);
  float tR = ntcTempC_from_R_SH(rRoom, SH_A_ROOM, SH_B_ROOM, SH_C_ROOM);

  // --- robust smoothing ---
  tA = hampelFilter(tA, aussenBuffer, SMOOTH_COUNT, hampelA_runtime);
  tV = hampelFilter(tV, vorlaufBuffer, SMOOTH_COUNT, hampelV_runtime);
  tR = hampelFilter(tR, roomBuffer,   SMOOTH_COUNT, hampelR_runtime);

  aussenBuffer[smoothIndex] = tA;
  vorlaufBuffer[smoothIndex] = tV;
  roomBuffer[smoothIndex]   = tR;
  smoothIndex = (smoothIndex + 1) % SMOOTH_COUNT;

  tAussen  = smoothMedian(aussenBuffer, SMOOTH_COUNT);
  tVorlauf = smoothMedian(vorlaufBuffer, SMOOTH_COUNT);
  tRoom    = smoothMedian(roomBuffer,   SMOOTH_COUNT);

  // --- Heizkurve ---
  if (!isnan(tAussen)) {
    float x = tAussen;
    const Curve &c = curves[curveIndex];
    vtCurve = c.A*x*x*x + c.B*x*x + c.C*x + c.D;
    vtCurve = constrain(vtCurve, VT_min_runtime, VT_max_runtime);
  }

  // --- Raumkorrektur / Curve shift mode ---
  float target = roomTarget_effective;

  if (roomMode) {
    // room-dependent (existing behavior)
    if (!isnan(vtCurve) && !isnan(tRoom)) {
      float error = target - tRoom;
      float corr = ROOM_KP * error;
      corr = constrain(corr, -ROOM_CORR_MAX, ROOM_CORR_MAX);
      vtCorrected = constrain(vtCurve + corr, VT_min_runtime, VT_max_runtime);
    } else {
      vtCorrected = vtCurve;
    }
  } else {
    // room-independent: parallel shift around 20°C with 1.25 K per °C difference
    float delta = target - 20.0f;
    float shift = delta * 1.25f;
    vtCorrected = constrain(vtCurve + shift, VT_min_runtime, VT_max_runtime);
  }
}

// --- automatic control ---
void controlHeating() {
  if (isnan(tVorlauf) || isnan(vtCorrected)) return;
  if (!brennerAn && tVorlauf < vtCorrected) {
    if (pumpeAn) setBrenner(true);
  } 
  else if (brennerAn && tVorlauf > vtCorrected + burnerHyst_runtime) {
    setBrenner(false);
  }
  // --- Pump control based only on outside temperature ---
  if (!isnan(tAussen)) {

    // Pump OFF if outside temp is high enough
    if (pumpeAn && tAussen >= roomTarget_effective + PUMP_OFF_OUTDOOR_DELTA) {
      setPump(false);
      if (brennerAn) setBrenner(false);  // safety: burner must stop
    }

    // Pump ON if outside temp drops
    else if (!pumpeAn && tAussen <= roomTarget_effective - PUMP_ON_OUTDOOR_DELTA) {
      setPump(true);
    }
  }
}

// --- Cooldown ---
unsigned long lastAction = 0;
bool actionAllowed() {
  if (millis() - lastAction < 500) return false;
  lastAction = millis();
  return true;
}

// ---------- small helper to parse simple query params ----------
float parseParamAtoiFloat(const char* req, const char* key) {
  char *p = strstr((char*)req, key);
  if (!p) return NAN;
  // key is like "A=" so p points at start of "A="
  return atof(p + strlen(key));
}

void sendNtpPacket(const char* address) {
  memset(ntpPacket, 0, NTP_PACKET_SIZE);
  ntpPacket[0] = 0b11100011; // LI, Version, Mode
  ntpPacket[1] = 0;          // Stratum
  ntpPacket[2] = 6;          // Polling Interval
  ntpPacket[3] = 0xEC;       // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  ntpPacket[12] = 49;
  ntpPacket[13] = 0x4E;
  ntpPacket[14] = 49;
  ntpPacket[15] = 52;

  udp.beginPacket(address, NTP_PORT);
  udp.write(ntpPacket, NTP_PACKET_SIZE);
  udp.endPacket();
}

// returns true on success and updates epochUtc + timeValid
bool syncTimeWithNtp(unsigned long timeoutMs = 1500) {
  if (WiFi.status() != WL_CONNECTED) return false;

  udp.begin(2390);

  sendNtpPacket(NTP_SERVER);

  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    int size = udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      udp.read(ntpPacket, NTP_PACKET_SIZE);

      // NTP timestamp starts at byte 40 (Transmit Timestamp)
      uint32_t highWord = (uint32_t)ntpPacket[40] << 24 |
                          (uint32_t)ntpPacket[41] << 16 |
                          (uint32_t)ntpPacket[42] << 8  |
                          (uint32_t)ntpPacket[43];

      const uint32_t seventyYears = 2208988800UL; // seconds 1900->1970
      if (highWord > seventyYears) {
          epochAtSyncUtc = highWord - seventyYears;
          msAtSync = millis();

          timeValid = true;
          lastNtpSyncMs = msAtSync;
        udp.stop();
        return true;
      }
    }
    delay(10);
  }

  udp.stop();
  return false;
}

bool isLeapYear(int y) {
  return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
}

int daysInMonth(int y, int m) {
  static const int dm[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (m == 2) return dm[m-1] + (isLeapYear(y) ? 1 : 0);
  return dm[m-1];
}

// Zeller-ish helper: weekday (0=Sun..6=Sat)
int weekdayFromDate(int y, int m, int d) {
  if (m < 3) { m += 12; y -= 1; }
  int K = y % 100;
  int J = y / 100;
  int h = (d + (13*(m+1))/5 + K + (K/4) + (J/4) + 5*J) % 7;
  // h: 0=Sat,1=Sun,...6=Fri
  int dow = (h + 6) % 7; // 0=Sun..6=Sat
  return dow;
}

int lastSundayOfMonth(int y, int m) {
  int dim = daysInMonth(y, m);
  int dow = weekdayFromDate(y, m, dim); // 0=Sun
  int offset = dow; // days since last Sunday
  return dim - offset;
}

// EU DST: starts last Sunday in March 01:00 UTC, ends last Sunday in Oct 01:00 UTC
bool isDstEuropeUtc(uint32_t epoch) {
  // Convert epoch to date/time in UTC (lightweight)
  uint32_t t = epoch;
  uint32_t sec = t % 60; t /= 60;
  uint32_t min = t % 60; t /= 60;
  uint32_t hour = t % 24; t /= 24;

  int y = 1970;
  while (true) {
    uint32_t dy = isLeapYear(y) ? 366 : 365;
    if (t >= dy) { t -= dy; y++; }
    else break;
  }
  int m = 1;
  while (true) {
    int dim = daysInMonth(y, m);
    if (t >= (uint32_t)dim) { t -= dim; m++; }
    else break;
  }
  int d = (int)t + 1;

  int startDay = lastSundayOfMonth(y, 3);
  int endDay   = lastSundayOfMonth(y, 10);

  // compare (y,m,d,h) to DST window in UTC:
  // start: Mar startDay 01:00 UTC
  // end:   Oct endDay   01:00 UTC
  if (m < 3 || m > 10) return false;
  if (m > 3 && m < 10) return true;

  if (m == 3) {
    if (d > startDay) return true;
    if (d < startDay) return false;
    return hour >= 1;
  }
  // m == 10
  if (d < endDay) return true;
  if (d > endDay) return false;
  return hour < 1;
}

void formatLocalTimeDE(char* out, size_t outSize, uint32_t epochUtcNow) {
  if (!timeValid) { snprintf(out, outSize, "invalid"); return; }

  bool dst = isDstEuropeUtc(epochUtcNow);
  int offsetSec = dst ? 2*3600 : 1*3600;
  uint32_t local = epochUtcNow + offsetSec;

  // Convert local epoch to Y-M-D H:M:S (reuse simple conversion)
  uint32_t t = local;
  uint32_t sec = t % 60; t /= 60;
  uint32_t min = t % 60; t /= 60;
  uint32_t hour = t % 24; t /= 24;

  int y = 1970;
  while (true) {
    uint32_t dy = isLeapYear(y) ? 366 : 365;
    if (t >= dy) { t -= dy; y++; }
    else break;
  }
  int m = 1;
  while (true) {
    int dim = daysInMonth(y, m);
    if (t >= (uint32_t)dim) { t -= dim; m++; }
    else break;
  }
  int d = (int)t + 1;

  snprintf(out, outSize, "%04d-%02d-%02d %02lu:%02lu:%02lu %s",
           y, m, d, (unsigned long)hour, (unsigned long)min, (unsigned long)sec,
           dst ? "CEST" : "CET");
}

uint32_t currentEpochUtc() {
  if (!timeValid) return 0;
  unsigned long ageMs = millis() - msAtSync;
  return epochAtSyncUtc + (ageMs / 1000UL);
}

// ---------- Timer logic ----------
uint16_t localMinutesNow(uint32_t epochUtcNow) {
  bool dst = isDstEuropeUtc(epochUtcNow);
  int offsetSec = dst ? 2*3600 : 1*3600;
  uint32_t local = epochUtcNow + offsetSec;

  uint32_t t = local / 60UL;       // minutes since epoch
  uint16_t minute = t % 60;
  uint16_t hour = (t / 60UL) % 24;
  return hour * 60 + minute;
}

float timerTargetNow(uint32_t epochUtcNow, bool* outIsDay = nullptr) {
  uint16_t m = localMinutesNow(epochUtcNow);

  bool isDay;
  if (timerDayStartMin < timerNightStartMin) {
    isDay = (m >= timerDayStartMin && m < timerNightStartMin);
  } else {
    // if user ever sets weird crossing times
    isDay = (m >= timerDayStartMin || m < timerNightStartMin);
  }

  if (outIsDay) *outIsDay = isDay;
  return isDay ? timerDayTarget : timerNightTarget;
}

// compute effective target (safe fallback if time invalid)
float computeEffectiveRoomTarget(bool* outTimerUsed = nullptr, bool* outIsDay = nullptr) {
  if (outTimerUsed) *outTimerUsed = false;
  if (outIsDay) *outIsDay = false;

  if (manualOverride) return roomTarget;           // manual mode: don’t care
  if (!timerEnabled) return roomTarget;           // timer off
  if (!timeValid) return roomTarget;              // no time -> safe fallback

  uint32_t nowUtc = currentEpochUtc();
  if (outTimerUsed) *outTimerUsed = true;
  return timerTargetNow(nowUtc, outIsDay);
}

// --- Setup ---
void setup() {
  analogReadResolution(12);
  // Initialize EEPROM early on ESP32 before any EEPROM.get/read calls
#ifdef ESP32
  EEPROM.begin(512);
#endif
  // For ESP32 ADC on 3.3V boards: set full-scale attenuation (~0..3.3V)
  // This ensures analogRead uses the 0..3.3V range and maps to 0..4095
  // Guard the calls so the sketch remains compatible with UNO R4.
#ifdef ESP32
  analogSetPinAttenuation(PIN_AUSSEN_ADC, ADC_11db);
  analogSetPinAttenuation(PIN_VORLAUF_ADC, ADC_11db);
  analogSetPinAttenuation(PIN_ROOM_ADC, ADC_11db);
#endif
  pinMode(RELAY_PUMPE, OUTPUT);
  pinMode(RELAY_BRENNER, OUTPUT);
  setPump(true);
  setBrenner(false);
  pinMode(PIN_SUPPLY_A, INPUT);
  pinMode(PIN_SUPPLY_V, INPUT);
  pinMode(PIN_SUPPLY_ROOM, INPUT);

  for (int i=0; i<SMOOTH_COUNT; i++) {
    aussenBuffer[i] = NAN;
    vorlaufBuffer[i] = NAN;
    roomBuffer[i]   = NAN;
  }

  loadCurveIndex();
  loadRoomTarget();
  loadRoomMode();     // load mode (default ON if not set)

  // Load persisted calibration, VT bounds and Hampel thresholds from EEPROM — applied to runtime now.
  loadCalibrationFromEEPROM(); // NEW: load calibration (or defaults)
  loadVTboundsFromEEPROM();    // NEW: load VT bounds (or defaults)
  loadHampelFromEEPROM();      // NEW: load Hampel thresholds (or defaults)
  loadBurnerHystFromEEPROM();   // NEW
  loadTimerFromEEPROM();
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }
  server.begin();
#ifndef ESP32
  WDT.begin(8000);
#endif
  syncTimeWithNtp(); // try once at boot (non-fatal if it fails)
}

// --- Hauptloop ---
void loop() {
#ifndef ESP32
  WDT.refresh();
#endif

  static unsigned long lastWiFiCheck = 0;
  static unsigned long lastWiFiConnected = millis();
  uint8_t status = WiFi.status();
  if (status == WL_CONNECTED) lastWiFiConnected = millis();
  else if (millis() - lastWiFiConnected > 60000UL) {
    WiFi.disconnect(); delay(1000); WiFi.begin(ssid, password);
    lastWiFiConnected = millis();
  }
  if (millis() - lastWiFiCheck > 10000UL) lastWiFiCheck = millis();

  if (millis() - lastMeasure >= MEASURE_INTERVAL) {
    roomTarget_effective = computeEffectiveRoomTarget(); // <-- NEW
    measureTemps();
    lastMeasure = millis();
    if (!manualOverride) controlHeating();
  }

  // periodic NTP sync (non-blocking-ish: only tries once per interval)
  if (WiFi.status() == WL_CONNECTED) {
    if (!timeValid) {
      // if we don't have time yet, try more often (e.g., every 10s)
      if (millis() - lastNtpAttemptMs > 10000UL) {
        lastNtpAttemptMs = millis();
        syncTimeWithNtp(1200);
      }
    } else {
      // normal maintenance sync
      if (millis() - lastNtpSyncMs > NTP_SYNC_INTERVAL_MS) {
        syncTimeWithNtp(1200);
      }
    }
  }

  WiFiClient client = server.available();
  if (!client) return;

  char req[600];
  unsigned long startRead = millis();
  while (!client.available()) {
    if (millis() - startRead > 3000) { client.stop(); return; }
    delay(1);
  }

  int len = client.readBytesUntil('\r', req, sizeof(req) - 1);
  if (len <= 0 || !strstr(req, "GET")) { client.stop(); return; }
  req[len] = 0; client.flush();

  // --- basic commands ---
  if (strstr(req, "GET /auto/on") && actionAllowed()) manualOverride = false;
  if (strstr(req, "GET /auto/off") && actionAllowed()) manualOverride = true;

  if (manualOverride) {
    if (strstr(req, "GET /pump/on") && actionAllowed()) setPump(true);
    if (strstr(req, "GET /pump/off") && actionAllowed()) { setPump(false); if (brennerAn) setBrenner(false); }
    if (strstr(req, "GET /brenner/on") && actionAllowed()) { if (!pumpeAn) setPump(true); setBrenner(true); }
    if (strstr(req, "GET /brenner/off") && actionAllowed()) setBrenner(false);
  }

  char *p = strstr(req, "/set?room=");
  if (p) {
    float newTarget = atof(p + 10);
    if (newTarget >= 2 && newTarget <= 30) {
      roomTarget = newTarget;
      measureTemps();
    }
  }

  // --- Room mode set/save (room control ON/OFF)
  char *rmSet = strstr(req, "/roommode/set?m=");
  if (rmSet) {
    int m = atoi(rmSet + 16);
    roomMode = (m != 0);
    roomTarget_effective = computeEffectiveRoomTarget();
    measureTemps();
    client.stop();
    return;
  }
  if (strstr(req, "/roommode/save")) {
    saveRoomMode(roomMode);
    client.stop();
    return;
  }

  // --- Heat curve selection (Set / Save) ---
  char *cpSet = strstr(req, "/curve/set?i=");
  if (cpSet) {
    int idx = atoi(cpSet + 13);
    if (idx >= 0 && idx <= 7) {
      curveIndex = idx;
    }
    client.stop();
    return;
  }
  if (strstr(req, "/curve/save")) {
    saveCurveIndex(curveIndex);
    client.stop();
    return;
  }

  // --- Room mode (Set / Save) ---
  char* tpSet = strstr(req, "/timer/set?");
  if (tpSet) {
    TimerParams p;
    parseTimerParams(req, p);

    timerEnabled = p.en;
    timerDayTarget = p.day;
    timerNightTarget = p.night;
    timerDayStartMin = p.dm;
    timerNightStartMin = p.nm;

    roomTarget_effective = computeEffectiveRoomTarget();
    measureTemps();

    client.stop();
    return;
  }

  char* tpSave = strstr(req, "/timer/save?");
  if (tpSave) {
    TimerParams p;
    parseTimerParams(req, p);

    saveTimerToEEPROM(p.en, p.day, p.night, p.dm, p.nm);

    client.stop();
    return;
  }

  // --- Calibration: save/reset (EEPROM-only mode) ---
  // Save to EEPROM permanently (applied on next boot) -- no immediate runtime update (Option 1)
  char *cpCalSave = strstr(req, "/cal/save?");
  if (cpCalSave) {
    float a = parseParamAtoiFloat(req, "A=");
    float v = parseParamAtoiFloat(req, "V=");
    float r = parseParamAtoiFloat(req, "R=");
    // if NaN, write marker 0.0 to clear that entry; caller should provide values they want stored
    if (isnan(a)) a = 0.0f;
    if (isnan(v)) v = 0.0f;
    if (isnan(r)) r = 0.0f;
    saveCalibrationToEEPROM(a, v, r);
    client.stop();
    return;
  }
  // Reset calibration (erase) so defaults will be used on next boot
  if (strstr(req, "/cal/reset")) {
    resetCalibrationInEEPROM();
    client.stop();
    return;
  }

  // --- VT bounds: save/reset (EEPROM-only mode) ---
  char *cpVTSave = strstr(req, "/vt/save?");
  if (cpVTSave) {
    float vmin = parseParamAtoiFloat(req, "MIN=");
    float vmax = parseParamAtoiFloat(req, "MAX=");
    // If NaN, write marker 0.0 to clear that entry (so defaults will apply after reboot)
    if (isnan(vmin)) vmin = 0.0f;
    if (isnan(vmax)) vmax = 0.0f;
    // validate ranges before saving: if provided and invalid, ignore saving that particular value (write 0.0 to clear)
    if (!(vmin >= 12.0f && vmin <= 65.0f)) vmin = 0.0f;
    if (!(vmax >= 70.0f && vmax <= 90.0f)) vmax = 0.0f;
    saveVTboundsToEEPROM(vmin, vmax);
    client.stop();
    return;
  }
  // Reset VT bounds in EEPROM -> defaults apply on next boot
  if (strstr(req, "/vt/reset")) {
    resetVTboundsInEEPROM();
    client.stop();
    return;
  }

  // --- Hampel thresholds: save/reset (EEPROM-only mode) ---
  // Save to EEPROM permanently (applied on next boot) -- Option 1
  char *cpHampSave = strstr(req, "/hampel/save?");
  if (cpHampSave) {
    float a = parseParamAtoiFloat(req, "A=");
    float v = parseParamAtoiFloat(req, "V=");
    float r = parseParamAtoiFloat(req, "R=");
    // If NaN, write marker 0.0
    if (isnan(a)) a = 0.0f;
    if (isnan(v)) v = 0.0f;
    if (isnan(r)) r = 0.0f;
    // Validate reasonable ranges: A 0.05..20, V 0.05..20, R 0.05..20
    if (!(a >= 0.05f && a <= 20.0f)) a = 0.0f;
    if (!(v >= 0.05f && v <= 20.0f)) v = 0.0f;
    if (!(r >= 0.05f && r <= 20.0f)) r = 0.0f;
    saveHampelToEEPROM(a, v, r);
    client.stop();
    return;
  }
  // Reset hampel thresholds in EEPROM -> defaults apply on next boot
  if (strstr(req, "/hampel/reset")) {
    resetHampelInEEPROM();
    client.stop();
    return;
  }

  // --- Burner hysteresis: save/reset (EEPROM-only mode) ---
  char *cpHystSave = strstr(req, "/hyst/save?");
  if (cpHystSave) {
    float h = parseParamAtoiFloat(req, "H=");
    // valid range: 4 .. 15 K
    if (!(h >= 4.0f && h <= 15.0f)) h = 0.0f;
    saveBurnerHystToEEPROM(h);
    client.stop();
    return;
  }

  if (strstr(req, "/hyst/reset")) {
    resetBurnerHystInEEPROM();
    client.stop();
    return;
  }

  // --- Save room target to EEPROM ---
  if (strstr(req, "GET /save")) saveRoomTarget(roomTarget);

  // --- JSON API ---
  if (strstr(req, "GET /status")) {
    client.println(F("HTTP/1.1 200 OK"));
    client.println(F("Content-Type: application/json"));
    client.println(F("Connection: close"));
    client.println();

    char timeStr[32];
    uint32_t nowUtc = currentEpochUtc();
    formatLocalTimeDE(timeStr, sizeof(timeStr), nowUtc);

    // START JSON OBJECT FIRST
    client.print(F("{"));

    // time fields
    client.print(F("\"timeValid\":"));
    client.print(timeValid ? F("true") : F("false"));

    client.print(F(",\"timeLocal\":\""));
    client.print(timeStr);
    client.print(F("\""));

    client.print(F(",\"ntpAgeSec\":"));
    if (timeValid) client.print((millis() - lastNtpSyncMs) / 1000UL);
    else client.print(-1);

    client.print(F(",\"tAussen\":")); client.print(isnan(tAussen)?0:tAussen,1);
    client.print(F(",\"tVorlauf\":")); client.print(isnan(tVorlauf)?0:tVorlauf,1);
    client.print(F(",\"tRoom\":"));   client.print(isnan(tRoom)?0:tRoom,1);
    client.print(F(",\"vtCurve\":")); client.print(isnan(vtCurve)?0:vtCurve,1);
    client.print(F(",\"vtCorrected\":")); client.print(isnan(vtCorrected)?0:vtCorrected,1);
    client.print(F(",\"pump\":"));    client.print(pumpeAn ? F("true") : F("false"));
    client.print(F(",\"burner\":"));  client.print(brennerAn ? F("true") : F("false"));
    client.print(F(",\"mode\":\""));  client.print(manualOverride ? F("Manuell") : F("Automatik")); client.print(F("\""));
    client.print(F(",\"curveIndex\":")); client.print(curveIndex);
    client.print(F(",\"roomMode\":")); client.print(roomMode ? F("true") : F("false"));
    client.print(F(",\"roomTarget\":")); client.print(roomTarget,1);
    
    bool timerUsed = false;
    bool timerIsDay = false;
    float eff = computeEffectiveRoomTarget(&timerUsed, &timerIsDay);

    client.print(F(",\"roomTargetEffective\":")); client.print(eff,1);
    client.print(F(",\"timerEnabled\":")); client.print(timerEnabled ? F("true") : F("false"));
    client.print(F(",\"timerUsed\":")); client.print(timerUsed ? F("true") : F("false"));
    client.print(F(",\"timerIsDay\":")); client.print(timerIsDay ? F("true") : F("false"));
    client.print(F(",\"timerDayTarget\":")); client.print(timerDayTarget,1);
    client.print(F(",\"timerNightTarget\":")); client.print(timerNightTarget,1);
    client.print(F(",\"timerDayStartMin\":")); client.print(timerDayStartMin);
    client.print(F(",\"timerNightStartMin\":")); client.print(timerNightStartMin);
    // add calibration fields to JSON (currently active runtime copies)
    client.print(F(",\"R_fixed_Aussen\":")); client.print(R_fixed_Aussen,1);
    client.print(F(",\"R_fixed_Vorlauf\":")); client.print(R_fixed_Vorlauf,1);
    client.print(F(",\"R_fixed_Room\":")); client.print(R_fixed_Room,1);
    // resistances (unfiltered)
    client.print(F(",\"rAussen\":")); client.print(rAussen,1);
    client.print(F(",\"rVorlauf\":")); client.print(rVorlauf,1);
    client.print(F(",\"rRoom\":")); client.print(rRoom,1);
    // VT runtime bounds
    client.print(F(",\"vtMinRuntime\":")); client.print(VT_min_runtime,1);
    client.print(F(",\"vtMaxRuntime\":")); client.print(VT_max_runtime,1);
    // Hampel runtime thresholds
    client.print(F(",\"hampelA_runtime\":")); client.print(hampelA_runtime,2);
    client.print(F(",\"hampelV_runtime\":")); client.print(hampelV_runtime,2);
    client.print(F(",\"hampelR_runtime\":")); client.print(hampelR_runtime,2);
    // Burner hysteresis runtime
    client.print(F(",\"burnerHyst\":"));
    client.print(burnerHyst_runtime,1);
    
    // Also include whether EEPROM currently stores values (helpful to the user)
    float saved_a = eeprom_getFloatOrNan(EEPROM_ADDR_RFIX_A);
    float saved_v = eeprom_getFloatOrNan(EEPROM_ADDR_RFIX_V);
    float saved_r = eeprom_getFloatOrNan(EEPROM_ADDR_RFIX_R);
    client.print(F(",\"saved_R_fixed_Aussen\":")); client.print(!isnan(saved_a) && saved_a >= 1.0f ? 1 : 0);
    client.print(F(",\"saved_R_fixed_Vorlauf\":")); client.print(!isnan(saved_v) && saved_v >= 1.0f ? 1 : 0);
    client.print(F(",\"saved_R_fixed_Room\":")); client.print(!isnan(saved_r) && saved_r >= 1.0f ? 1 : 0);

    float saved_min = eeprom_getFloatOrNan(EEPROM_ADDR_VT_MIN);
    float saved_max = eeprom_getFloatOrNan(EEPROM_ADDR_VT_MAX);
    client.print(F(",\"saved_vtMin\":")); client.print(!isnan(saved_min) && saved_min >= 1.0f ? 1 : 0);
    client.print(F(",\"saved_vtMax\":")); client.print(!isnan(saved_max) && saved_max >= 1.0f ? 1 : 0);

    float sh_a = eeprom_getFloatOrNan(EEPROM_ADDR_HAMP_A);
    float sh_v = eeprom_getFloatOrNan(EEPROM_ADDR_HAMP_V);
    float sh_r = eeprom_getFloatOrNan(EEPROM_ADDR_HAMP_R);
    client.print(F(",\"saved_hampelA\":")); client.print(!isnan(sh_a) && sh_a >= 0.05f ? 1 : 0);
    client.print(F(",\"saved_hampelV\":")); client.print(!isnan(sh_v) && sh_v >= 0.05f ? 1 : 0);
    client.print(F(",\"saved_hampelR\":")); client.print(!isnan(sh_r) && sh_r >= 0.05f ? 1 : 0);

    float saved_h = eeprom_getFloatOrNan(EEPROM_ADDR_BURNER_HYST);
    client.print(F(",\"saved_burnerHyst\":"));
    client.print(!isnan(saved_h) && saved_h >= 4.0f ? 1 : 0);

    client.print(F(",\"lastSaved\":"));
    if (!isnan(lastSavedRoomTarget)) client.print(lastSavedRoomTarget,1); else client.print(-1);
    client.print(F(",\"freeMem\":")); client.print(freeMemory());
    client.print(F(",\"wifiStatus\":")); client.print(WiFi.status());
    client.println(F("}"));
    client.flush(); client.stop(); delay(10); return;
  }

  // --- HTML UI ---
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: text/html; charset=utf-8"));
  client.println(F("Connection: close"));
  client.println();
  client.println(F(
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<title>Heating Controller</title>"
    "<meta name='viewport' content='width=device-width, initial-scale=1'>"
    "<style>"
      ".expert{"
        "border-top:1px solid #ccc;"
        "margin-top:10px;"
        "padding-top:10px;"
      "}"
      "body{font-family:sans-serif;max-width:600px;margin:auto;padding:10px;}"
      "h2{color:#333;}"
      ".on{color:green;font-weight:bold;}.off{color:red;font-weight:bold;}"
      "button{padding:8px 12px;margin:4px;}"
    "</style></head><body>"
  ));
  client.println(F(
    "<h2>Heating Controller</h2>"
  ));
  client.println(F("<div id='status'>Loading...</div>"));
  // --- Room Target block (UI Option C: put the toggle here) ---
  client.println(F(
    "<h3>Room target</h3>"
    "<input type='number' step='0.1' min='2' max='30' id='roomTarget'> °C "
    "<button onclick=\"setTarget()\">Set</button>"
    "<button onclick=\"sendCommand('/save')\">Save</button>"
    "<h3>Room control (Mode)</h3>"
    "<select id='roomModeSel'>"
      "<option value='1'>ON</option>"
      "<option value='0'>OFF (Curve-Shift)</option>"
    "</select>"
    "<button onclick=\"setRoomMode()\">Set</button>"
    "<button onclick=\"sendCommand('/roommode/save')\">Save</button>"
    "<p style='font-size:0.9em;color:#666;'>Note: When room control is disabled, the curve is shifted by (target - 20°C) * 1.25 K.</p>"
  ));
  client.println(F(
    "<h3>Control (mode)</h3>"
    "<p><button onclick=\"sendCommand('/auto/on')\">Automatic</button>"
    "<button onclick=\"sendCommand('/auto/off')\">Manual</button></p>"
    "<div id='manualControls'></div>"
  ));

  client.println(F(
    "<h3>Timer (Schedule)</h3>"
    "<p>Timer in automatic mode:</p>"
    "<select id='timerEnSel'>"
      "<option value='0'>OFF</option>"
      "<option value='1'>ON</option>"
    "</select>"
      "<button onclick=\"timerSetAll()\">Set</button>"
      "<button onclick=\"timerSaveAll()\">Save</button>"

    "<p>Day target: <input id='tDay' type='number' step='0.1' min='2' max='30'> °C "
    "Night target: <input id='tNight' type='number' step='0.1' min='2' max='30'> °C</p>"

    "<p>Day Start: <input id='tDayStart' type='time'> "
    "Night Start: <input id='tNightStart' type='time'></p>"

    "<p style='font-size:0.9em;color:#666;'>"
    "Note: If time is invalid (no NTP), the timer is ignored and the manual target is used."
    "</p>"
  ));

  client.println(F(
    "<h3>Heating Curve</h3>"
    "<select id='curveSel'>"
      "<option value='0'>I</option>"
      "<option value='1'>II</option>"
      "<option value='2'>III</option>"
      "<option value='3'>1</option>"
      "<option value='4'>3</option>"
      "<option value='5'>5</option>"
      "<option value='6'>7</option>"
      "<option value='7'>8</option>"
    "</select>"
    "<button onclick=\"setCurve()\">Set</button>"
    "<button onclick=\"sendCommand('/curve/save')\">Save</button>"
  ));

  // --- Calibration UI (Option A: under Heizkurve) ---
  client.println(F("<div class='expert'>"));
  client.println(F(
  "<h3>Calibration (Fixed Resistances)</h3>"
  "<p>Enter the total R_fixed * factor number (e.g. 1536.7)</p>"
  "<p>"
    "Outside: <input id='calA' type='number' step='0.1' style='width:60px;'> Ω "
    "Flow: <input id='calV' type='number' step='0.1' style='width:60px;'> Ω "
    "Room: <input id='calR' type='number' step='0.1' style='width:60px;'> Ω "
    "<button onclick=\"calSave()\">Save</button> "
    "<button onclick=\"calReset()\">Reset</button>"
  "</p>"
  ));
  client.println(F("</div>"));

  // --- VT bounds UI (same behavior as calibration) ---
  client.println(F("<div class='expert'>"));
  client.println(F(
  "<h3>Flow Temperature Limits (VT_MIN / VT_MAX)</h3>"
  "<p>VT_MIN allowed 12°C .. 65°C; VT_MAX allowed 70°C .. 90°C.</p>"
  "<p>"
      "VT_MIN: <input id='vtMin' type='number' step='0.1' style='width:60px;'> °C "
      "VT_MAX: <input id='vtMax' type='number' step='0.1' style='width:60px;'> °C "
      "<button onclick=\"vtSave()\">Save</button> "
      "<button onclick=\"vtReset()\">Reset</button>"
    "</p>"
  ));
  client.println(F("</div>"));

  // --- Hampel thresholds UI ---
  client.println(F("<div class='expert'>"));
  client.println(F(
    "<h3>Hampel Filter Thresholds</h3>"
    "<p>"
      "O: <input id='hampA' type='number' step='0.01' style='width:60px;'> "
      "F: <input id='hampV' type='number' step='0.01' style='width:60px;'> "
      "R: <input id='hampR' type='number' step='0.01' style='width:60px;'> "
      "<button onclick=\"hampelSave()\">Save</button> "
      "<button onclick=\"hampelReset()\">Reset</button>"
    "</p>"
  ));
  client.println(F("</div>"));

  // --- Hyst threshold UI ---
  client.println(F("<div class='expert'>"));
  client.println(F(
    "<h3>Burner Hysteresis</h3>"
    "<p>"
      "Range: 4 K .. 15 K "
      "<input id='hystVal' type='number' step='0.1' style='width:60px;'> K "
      "<button onclick=\"hystSave()\">Save</button> "
      "<button onclick=\"hystReset()\">Reset</button>"
    "</p>"
  ));

client.println(F("</div>"));

  client.println(F(
    "<script>"
      "let expertMode = false;"
      "let userEditingUntil = 0;"  // typing grace timer

        "document.addEventListener('DOMContentLoaded',()=>{"
          "expertMode = localStorage.getItem(\"expertMode\") === \"1\";"
          "applyExpertMode();"
        "document.getElementById('roomTarget').addEventListener('input',()=>{userEditingUntil=Date.now()+5000;});"
        "document.getElementById('timerEnSel').addEventListener('input',()=>{userEditingUntil=Date.now()+5000;});"
        "document.getElementById('tDay').addEventListener('input',()=>{userEditingUntil=Date.now()+5000;});"
        "document.getElementById('tNight').addEventListener('input',()=>{userEditingUntil=Date.now()+5000;});"
        "document.getElementById('tDayStart').addEventListener('input',()=>{userEditingUntil=Date.now()+5000;});"
        "document.getElementById('tNightStart').addEventListener('input',()=>{userEditingUntil=Date.now()+5000;});"
        "document.getElementById('curveSel').addEventListener('input',()=>{userEditingUntil=Date.now()+5000;});"
        "document.getElementById('roomModeSel').addEventListener('input',()=>{userEditingUntil=Date.now()+5000;});"
        "document.getElementById('calA').addEventListener('input',()=>{userEditingUntil=Date.now()+5000;});"
        "document.getElementById('calV').addEventListener('input',()=>{userEditingUntil=Date.now()+5000;});"
        "document.getElementById('calR').addEventListener('input',()=>{userEditingUntil=Date.now()+5000;});"
        "document.getElementById('vtMin').addEventListener('input',()=>{userEditingUntil=Date.now()+5000;});"
        "document.getElementById('vtMax').addEventListener('input',()=>{userEditingUntil=Date.now()+5000;});"
        "document.getElementById('hampA').addEventListener('input',()=>{userEditingUntil=Date.now()+5000;});"
        "document.getElementById('hampV').addEventListener('input',()=>{userEditingUntil=Date.now()+5000;});"
        "document.getElementById('hampR').addEventListener('input',()=>{userEditingUntil=Date.now()+5000;});"
      "});"

      "async function sendCommand(cmd){await fetch(cmd);update();}"

      "async function setTarget(){"
        "let val=document.getElementById('roomTarget').value;"
        "if(val){await fetch('/set?room='+val);update();}"
      "}"

      "function toggleExpert(){"
        "expertMode = !expertMode;"
        "localStorage.setItem(\"expertMode\", expertMode ? \"1\" : \"0\");"
        "applyExpertMode();"
      "}"

      "function applyExpertMode(){"
        "document.querySelectorAll('.expert').forEach(el=>{"
          "el.style.display = expertMode ? 'block' : 'none';"
        "});"

        "document.getElementById('modeLabel').innerText ="
          "expertMode ? 'Expert Mode' : 'Basic Mode';"
      "}"

      "function minToHHMM(m){"
        "let h = Math.floor(m / 60);"
        "let min = m % 60;"
        "return String(h).padStart(2,'0') + ':' + String(min).padStart(2,'0');"
      "}"

      "function hhmmToMin(s){"
        "if(!s || s.length < 4) return null;"
        "let parts = s.split(':');"
        "let h = parseInt(parts[0],10);"
        "let m = parseInt(parts[1],10);"
        "if(isNaN(h)||isNaN(m)) return null;"
        "return h*60+m;"
      "}"

      "function minToHhmm(min){"
        "min = Math.max(0, Math.min(1439, min|0));"
        "let h = Math.floor(min/60);"
        "let m = min%60;"
        "return (h<10?'0':'')+h+':' + (m<10?'0':'')+m;"
      "}"

      "function getTimerParams(){"
        "let en = document.getElementById('timerEnSel').value;"
        "let day = document.getElementById('tDay').value;"
        "let night = document.getElementById('tNight').value;"
        "let dm = hhmmToMin(document.getElementById('tDayStart').value);"
        "let nm = hhmmToMin(document.getElementById('tNightStart').value);"

        "if(dm===null || nm===null) return null;"

        "return 'EN='+en+'&DAY='+day+'&NIGHT='+night+'&DSTART='+dm+'&NSTART='+nm;"
      "}"
      
      "async function timerSetAll(){"
        "let params = getTimerParams();"
        "if(!params) return;"

        "await fetch('/timer/set?' + params);"
        "update();"
      "}"

      "async function timerSaveAll(){"
        "let params = getTimerParams();"
        "if(!params) return;"

        "await fetch('/timer/save?' + params);"
        "update();"
      "}"

      "async function setCurve(){"
        "let idx=document.getElementById('curveSel').value;"
        "await fetch('/curve/set?i='+idx);"
        "update();"
      "}"

      "async function setRoomMode(){"
        "let m = document.getElementById('roomModeSel').value;"
        "await fetch('/roommode/set?m='+m);"
        "update();"
      "}"

      "async function calSave(){"
        "let a=document.getElementById('calA').value;"
        "let v=document.getElementById('calV').value;"
        "let r=document.getElementById('calR').value;"
        "await fetch('/cal/save?A='+a+'&V='+v+'&R='+r);"
        "update();"
      "}"

      "async function calReset(){"
        "await fetch('/cal/reset');"
        "update();"
      "}"

      "async function vtSave(){"
        "let min=document.getElementById('vtMin').value;"
        "let max=document.getElementById('vtMax').value;"
        "await fetch('/vt/save?MIN='+min+'&MAX='+max);"
        "update();"
      "}"
      "async function vtReset(){"
        "await fetch('/vt/reset');"
        "update();"
      "}"

      "async function hampelSave(){"
        "let a=document.getElementById('hampA').value;"
        "let v=document.getElementById('hampV').value;"
        "let r=document.getElementById('hampR').value;"
        "await fetch('/hampel/save?A='+a+'&V='+v+'&R='+r);"
        "update();"
      "}"
      "async function hampelReset(){"
        "await fetch('/hampel/reset');"
        "update();"
      "}"

      "async function hystSave(){"
      "let h=document.getElementById('hystVal').value;"
      "await fetch('/hyst/save?H='+h);"
      "update();"
      "}"
      "async function hystReset(){"
      "await fetch('/hyst/reset');"
      "update();"
      "}"

      "async function update(){"
        "let r=await fetch('/status');"
        "let d=await r.json();"
        "let html='';"

        "html+=''+"
        "'<p><b>'+d.timeLocal+'</b> '+(d.timeValid?'':'(invalid)')+'</p>';"
        "html+='<p>Room: '+d.tRoom+' °C → <b>'+d.roomTargetEffective+' °C</b> ' +"
        "'<span style=\"color:#666;font-size:0.9em;\">' +"
        "(d.timerEnabled"
        "  ? (d.timerUsed"
        "      ? (d.timerIsDay ? '(Day)' : '(Night)')"
        "      : '(Time invalid → Fallback)')"
        "  : '(Timer OFF)') +"
        "'</span></p>';"
        "html+='<p>Flow: '+d.tVorlauf+' °C → <b>'+d.vtCorrected+' °C</b></p>';"
        "html+='<p>Outside: '+d.tAussen+' °C</p>';"
        "html+='<p>Pump: <span class=\"'+(d.pump?'on':'off')+'\">'+(d.pump?'ON':'OFF')+'</span></p>';"
        "html+='<p>Burner: <span class=\"'+(d.burner?'on':'off')+'\">'+(d.burner?'ON':'OFF')+'</span></p>';"

        "if(expertMode){"
          "html+='<p>Mode: <b>'+d.mode+'</b></p>';"
          "html += '<p>NTP age: ' + d.ntpAgeSec + ' s</p>';"
          "html+='<p>Flow target: '+d.vtCurve+' °C</p>';"
          "html+='<p>Flow target (Room temperature correction): '+d.vtCorrected+' °C</p>';"
          "html+='<p>Room target (manual): '+d.roomTarget+' °C</p>';"
          "html+='<p>Current heating curve: '+d.curveIndex+'</p>';"
          "html += '<p>Timer: '"
            "+ (d.timerEnabled ? 'ON' : 'OFF')"
            "+ '</p>';"
          "html += '<p>Day target: ' + d.timerDayTarget + ' °C at '"
            "+ minToHHMM(d.timerDayStartMin) + '</p>';"
          "html += '<p>Night target: ' + d.timerNightTarget + ' °C at '"
            "+ minToHHMM(d.timerNightStartMin) + '</p>';"
          "html+='<p>R_fixed Outside: '+d.R_fixed_Aussen+' Ω</p>';"
          "html+='<p>R_fixed Flow: '+d.R_fixed_Vorlauf+' Ω</p>';"
          "html+='<p>R_fixed Room: '+d.R_fixed_Room+' Ω</p>';"
          "html+='<p>R Outside (NTC): '+d.rAussen+' Ω</p>';"
          "html+='<p>R Flow (NTC): '+d.rVorlauf+' Ω</p>';"
          "html+='<p>R Room (NTC): '+d.rRoom+' Ω</p>';"
          "html+='<p>VT_MIN: '+d.vtMinRuntime+' °C</p>';"
          "html+='<p>VT_MAX: '+d.vtMaxRuntime+' °C</p>';"
          "html+='<p>Hampel O: '+d.hampelA_runtime+'</p>';"
          "html+='<p>Hampel F: '+d.hampelV_runtime+'</p>';"
          "html+='<p>Hampel R: '+d.hampelR_runtime+'</p>';"
          "html+='<p>Hysteresis: '+d.burnerHyst+' K</p>';"
          "if(d.lastSaved>0) html+='<p>Last target: '+d.lastSaved+' °C</p>';"
          "html+='<p>Free memory: '+d.freeMem+' bytes</p>';"
          "html+='<p>WiFi status: '+d.wifiStatus+'</p>';"
        "}"
        "document.getElementById('status').innerHTML=html;"

        // ---- Sync input fields unless user is typing ----
        "if(Date.now() > userEditingUntil){"
          "let rt=document.getElementById('roomTarget');"
          "if(document.activeElement!==rt){rt.value=d.roomTarget;}"

          "let te=document.getElementById('timerEnSel');"
          "if(document.activeElement!==te){te.value=d.timerEnabled?'1':'0';}"

          "let td=document.getElementById('tDay');"
          "if(document.activeElement!==td){td.value=d.timerDayTarget;}"

          "let tn=document.getElementById('tNight');"
          "if(document.activeElement!==tn){tn.value=d.timerNightTarget;}"

          "let tds=document.getElementById('tDayStart');"
          "if(document.activeElement!==tds){tds.value=minToHhmm(d.timerDayStartMin);}"

          "let tns=document.getElementById('tNightStart');"
          "if(document.activeElement!==tns){tns.value=minToHhmm(d.timerNightStartMin);}"

          "let cs=document.getElementById('curveSel');"
          "if(document.activeElement!==cs){cs.value=d.curveIndex;}"

          "let rms=document.getElementById('roomModeSel');"
          "if(document.activeElement!==rms){rms.value=d.roomMode?'1':'0';}"

          // sync calibration inputs with current values (runtime active)
          "let ca=document.getElementById('calA');"
          "if(document.activeElement!==ca){ca.value=d.R_fixed_Aussen;}"
          "let cv=document.getElementById('calV');"
          "if(document.activeElement!==cv){cv.value=d.R_fixed_Vorlauf;}"
          "let cr=document.getElementById('calR');"
          "if(document.activeElement!==cr){cr.value=d.R_fixed_Room;}"

          // sync VT inputs with current active runtime values (these were loaded at boot)
          "let vmin=document.getElementById('vtMin');"
          "if(document.activeElement!==vmin){vmin.value=d.vtMinRuntime;}"
          "let vmax=document.getElementById('vtMax');"
          "if(document.activeElement!==vmax){vmax.value=d.vtMaxRuntime;}"

          // sync Hampel inputs with current runtime values (loaded at boot)
          "let ha=document.getElementById('hampA');"
          "if(document.activeElement!==ha){ha.value=d.hampelA_runtime;}"
          "let hv=document.getElementById('hampV');"
          "if(document.activeElement!==hv){hv.value=d.hampelV_runtime;}"
          "let hr=document.getElementById('hampR');"
          "if(document.activeElement!==hr){hr.value=d.hampelR_runtime;}"

          // sync Hyst input
          "let hh=document.getElementById('hystVal');"
          "if(document.activeElement!==hh){hh.value=d.burnerHyst;}"
        "}"

        // Manual controls
        "let mc='';"
        "if(d.mode==='Manuell'){"
          "mc+='<p><button onclick=\"sendCommand(\\'/pump/on\\')\">Pump ON</button>';"
          "mc+='<button onclick=\"sendCommand(\\'/pump/off\\')\">Pump OFF</button></p>';"
          "mc+='<p><button onclick=\"sendCommand(\\'/brenner/on\\')\">Burner ON</button>';"
          "mc+='<button onclick=\"sendCommand(\\'/brenner/off\\')\">Burner OFF</button></p>';}"
        "document.getElementById('manualControls').innerHTML=mc;"
      "}"
      "setInterval(update,3000);"
      "update();"
    "</script>"
  ));
  client.println(F(
    "<hr style='margin-top:30px;margin-bottom:10px;border:0;border-top:1px solid #ccc;'>"
    "<div style='text-align:center;color:#666;font-size:0.9em;'>"
      "<button onclick=\"toggleExpert()\">Toggle Expert Mode</button><br>"
      "<span id='modeLabel' style='font-weight:bold;'></span>"
    "</div>"
  ));
  client.println(F("</body></html>"));
  client.flush();
  client.stop();
  delay(10);
}