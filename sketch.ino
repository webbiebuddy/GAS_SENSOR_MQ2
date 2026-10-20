/******************************************************************************
 * PS6 - SILENT LEAK PRO  ·  one sketch for Wokwi (ESP32) and real ESP8266
 *
 * SIMULATION_BUILD 1 (default) = Wokwi ESP32 simulation of the full Pro
 *   logic, accelerated timings, no Wi-Fi/Blynk/MQTT. Parts wired to match
 *   the project kit: SG90 servo = gas-valve shutoff, buzzer = local audible
 *   alarm; the INJECT GAS button is a sim-only demo convenience.
 * SIMULATION_BUILD 0 = the real ESP8266 build (Wi-Fi + Blynk + MQTT +
 *   EEPROM, exactly the behavior of silent_leak/silent_leak.ino).
 *
 * Detection engine (identical on both targets -- keep in sync with
 * silent_leak/silent_leak.ino):
 *   median-of-15 + EMA filtering -> slow adaptive baseline that FREEZES
 *   during events -> WARNING (held elevation) / ALARM (deeper level or fast
 *   rise) with hysteresis+dwell clear -> sensor fault detection.
 *
 * HARDWARE SAFETY: the MQ-2 DOUT still drives the relay/siren directly,
 * independent of this firmware. Nothing here can disable that path.
 *
 * Wokwi setup:
 *   - Run the simulation; press the INJECT GAS button (or key G) to demo
 *     the full warning -> alarm -> valve-close -> clear cycle without gas.
 *   - Command 'a' (MQTT cmd "silence") mutes the buzzer during an alarm
 *     for 20 s (LEDs + relay keep alarming); otherwise sends a test event.
 *   - Serial monitor: T test · I inject · A acknowledge · C recalibrate ·
 *     P report · S status · R reboot · H help.
 ******************************************************************************/

#define SIMULATION_BUILD 1

#if SIMULATION_BUILD
// ------------------------------- Wokwi ESP32 ------------------------------
const int GAS_PIN        = 34;   // MQ-2 AOUT (ADC1, input only)
const int STATUS_LED_PIN = 12;   // yellow software-status LED
const int BUZZER_PIN     = 14;   // piezo (hardware relay also has its own path)
const int SERVO_PIN      = 18;   // SG90 signal = gas-valve actuator
const int BTN_INJECT     = 32;   // sim-only INJECT GAS pushbutton (key G)

const unsigned long WARMUP_MS       = 1000UL;   // accelerated for simulation
const unsigned long CALIB_MS        = 2500UL;
const unsigned long WARN_CONFIRM_MS = 1000UL;
const unsigned long ALARM_CONFIRM_MS= 1500UL;
const unsigned long CLEAR_MS        = 2000UL;
const unsigned long EVENT_GAP_MS    = 8000UL;
const unsigned long URGENT_GAP_MS   = 4000UL;
const unsigned long WARMUP_PRINT_MS = 500UL;
#else
// ------------------------------ Real ESP8266 ------------------------------
#define BLYNK_TEMPLATE_ID   "TMPLxxxx"
#define BLYNK_TEMPLATE_NAME "Silent Leak"
#define BLYNK_AUTH_TOKEN    "YOUR_DEVICE_AUTH_TOKEN"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <Servo.h>

static Servo servoValve;    // SG90 gas-valve actuator

const int GAS_PIN        = A0;   // MQ-2 AOUT
const int STATUS_LED_PIN = D6;   // yellow software-status LED
const int BUZZER_PIN     = -1;   // buzzer is relay-driven (hardware path)
const int SERVO_PIN      = D7;   // SG90 signal = gas-valve actuator
const int BTN_INJECT     = -1;   // injection via serial 'i' on hardware

const unsigned long WARMUP_MS       = 10UL * 60UL * 1000UL;
const unsigned long CALIB_MS        = 30UL * 1000UL;
const unsigned long WARN_CONFIRM_MS = 2000UL;
const unsigned long ALARM_CONFIRM_MS= 3000UL;
const unsigned long CLEAR_MS        = 8000UL;
const unsigned long EVENT_GAP_MS    = 30000UL;
const unsigned long URGENT_GAP_MS   = 10000UL;
const unsigned long WARMUP_PRINT_MS = 5000UL;

static const char* WIFI_SSID = "NAZI";
static const char* WIFI_PASS = "HAILHITLER";
static const char*  MQTT_HOST  = "broker.hivemq.com";
static const uint16_t MQTT_PORT = 1883;
static const char*  MQTT_USER  = nullptr;
static const char*  MQTT_PASS  = nullptr;
static const char*  TOPIC_ROOT = "silentleak";
#define FW_VERSION "2.2.0"
#endif

// Forward-declare so servoDriveDeg() (SIMULATION_BUILD) can call it before
// the definition appears later in the file — the .ino auto-prototype hoist
// does not reliably cross #if/#else boundaries.
#if SIMULATION_BUILD
static void servoWriteDeg(int deg);
#endif

// ---------------------------- Detection config ----------------------------
const int ALARM_DELTA   = 45;    // ADC counts above threshold -> ALARM
const int WARN_DELTA    = 25;    // ADC counts above threshold -> WARNING
const int CLEAR_DELTA   = 12;    // hysteresis floor for clearing
const float SLOPE_RISE_1S = 15.0f;  // ADC counts per second -> immediate ALARM

const unsigned long SAMPLE_MS   = 50UL;    // 20 Hz
const unsigned long SERIAL_MS   = 1000UL;
const unsigned long PUBLISH_MS  = 2000UL;
const unsigned long ACK_MS      = 20000UL; // silence-buzzer window
const unsigned long INJECT_MS   = 12000UL; // test-leak duration
const unsigned long INJECT_RAMP_MS = 6000UL;
const int  INJECT_DELTA         = 60;      // counts above baseline at full ramp

const uint8_t MEDIAN_N  = 15;const float  EMA_ALPHA = 0.25f;
const float   BASELINE_ALPHA = 0.0008f;    // ~10 min drift tracking

const int VALVE_OPEN   = 0;                // SG90 angle, gas flowing
const int VALVE_CLOSED = 180;              // SG90 angle, gas shut off
// If the horn hits the valve's mechanical stop before the servo reaches the
// angle above, shorten VALVE_CLOSED here (e.g. 170): a servo commanded past
// its stop stalls -- hot gearbox, brown-out, "stuck" valve.
const int      VALVE_STEP_MS    = 10;      // 1 deg / 10 ms -> 180 deg in 1.8 s
const int      VALVE_REFRESH_MS = 4000;    // idle poll cadence once parked
const int      VALVE_SETTLE_MS  = 1500;    // keep pulses this long after arrival

// ------------------------------- State ------------------------------------
enum State : uint8_t {
  ST_BOOT, ST_WARMUP, ST_CALIBRATING, ST_SAFE, ST_WARNING, ST_ALARM, ST_FAULT
};
static State gState = ST_BOOT;

static uint32_t bootTime = 0;
static uint32_t lastSampleAt = 1, lastSerialAt = 1, lastPublishAt = 1;  // =1: reached() never fires for since=0
static uint32_t warmupStart = 0, calibStart = 0, lastSlopeWin = 1;  // warmup/calib assigned in setup; slope same since=0 rule
static uint32_t highSince = 0, severeSince = 0, lowSince = 0;
static uint32_t faultSince = 0, faultClearSince = 0, lastFastRiseAt = 0;
static uint32_t lastWarmupPrint = 1;  // since=0 would silence the warm-up countdown forever
static uint32_t lastEventAt = 0, eventCount = 0, bootCount = 0;

static int   lastRaw = 0;
static float ema = 0.0f;
static bool  emaValid = false;
static float slopeRef = 0.0f;

static float    baseline = 0.0f;
static bool     baselineReady = false;
static float    calibSum = 0.0f;
static uint32_t calibCount = 0;
static int     calibMin = 1023, calibMax = 0;
static uint8_t calibRetries = 0;

static uint32_t buzzerMutedUntil = 0;
static uint32_t injectUntil = 0, injectStart = 0;
static bool    injectActive = false;

static int    valvePos = VALVE_OPEN;       // commanded state (telemetry)
static int    valveWanted = VALVE_OPEN;    // sweep target
static int    valveCmd = VALVE_OPEN;       // last angle written to the servo
static uint32_t valveNextWriteAt = 0;
static bool   servoEngaged = false;        // pulse train on (real build parks when idle)
static uint32_t valveArrivedAt = 0;        // when the sweep reached its target
static bool   valveHoldClosed = false;  // remote hold: keep the valve shut (dashboard/CLI)
static int    lastBuzFreq = -1;

static int    medBuf[MEDIAN_N];
static uint8_t medIdx = 0;

// ------------------------------ Helpers -----------------------------------
static inline bool reached(uint32_t now, uint32_t since, uint32_t interval) {
  return since != 0 && (uint32_t)(now - since) >= interval;
}

static const char* stateName(State s) {
  switch (s) {
    case ST_WARMUP:      return "WARMUP";
    case ST_CALIBRATING: return "CALIBRATING";
    case ST_SAFE:        return "SAFE";
    case ST_WARNING:     return "WARNING";
    case ST_ALARM:       return "ALARM";
    case ST_FAULT:       return "FAULT";
    default:             return "BOOT";
  }
}

// Median-of-N: a single ADC spike can no longer trip a threshold.
static int medianPush(int v) {
  medBuf[medIdx] = v;
  medIdx = (medIdx + 1) % MEDIAN_N;
  int tmp[MEDIAN_N];
  memcpy(tmp, medBuf, sizeof(tmp));
  for (uint8_t i = 1; i < MEDIAN_N; i++) {
    int key = tmp[i];
    int j = i - 1;
    while (j >= 0 && tmp[j] > key) { tmp[j + 1] = tmp[j]; j--; }
    tmp[j + 1] = key;
  }
  return tmp[MEDIAN_N / 2];
}

// Threshold anchor. Kept as a single chokepoint so a compensation model
// can be re-introduced later without touching the ~40 call sites.
static float activeBaseline() {
  return baseline;
}

// Rough, UNCALIBRATED ppm estimate for trend display only.
static float estimatePpm() {
  if (!baselineReady) return 0.0f;
  float denom = ema < 1.0f ? 1.0f : ema;
  float r  = (1023.0f - ema) / denom;
  float b  = activeBaseline();
  float r0 = (1023.0f - b) / (b < 1.0f ? 1.0f : b);
  float x  = r0 < 0.001f ? 1.0f : (r / r0);
  if (x < 0.02f) x = 0.02f;
  if (x > 50.0f) x = 50.0f;
  float ppm = powf(10.0f, -0.47f * log10f(x) + 1.33f);
  if (ppm < 0.0f) ppm = 0.0f;
  if (ppm > 10000.0f) ppm = 10000.0f;
  return ppm;
}

#if !SIMULATION_BUILD
// ============================= PERSISTENCE ================================
struct Persist {
  uint32_t magic;
  float    baseline;
  uint32_t bootCount;
  uint32_t eventCount;
  uint32_t crc;
};
static const uint32_t PERSIST_MAGIC = 0x534C4B33UL;  // "SLK3"

// Explicit declaration: the .ino preprocessor otherwise hoists its own
// prototype for this function above the struct, which breaks the build.
static uint32_t persistCrc(const Persist& p);

static uint32_t persistCrc(const Persist& p) {
  uint32_t b;
  memcpy(&b, &p.baseline, 4);
  return p.magic ^ b ^ p.bootCount ^ p.eventCount;
}

static void persistLoad() {
  EEPROM.begin(64);
  Persist p;
  EEPROM.get(0, p);
  if (p.magic == PERSIST_MAGIC && p.crc == persistCrc(p)) {
    baseline   = p.baseline;
    bootCount  = p.bootCount + 1;
    eventCount = p.eventCount;
    Serial.printf("Stored baseline from last run: %.1f\n", baseline);
  } else {
    bootCount = 1;
    Serial.println("No valid stored state; starting fresh.");
  }
  Serial.printf("Boot count: %lu\n", (unsigned long)bootCount);
}

static void persistSave() {
  Persist p;
  p.magic = PERSIST_MAGIC;
  p.baseline = baseline;
  p.bootCount = bootCount;
  p.eventCount = eventCount;
  p.crc = persistCrc(p);
  EEPROM.put(0, p);
  EEPROM.commit();
}
#endif  // !SIMULATION_BUILD

#if !SIMULATION_BUILD
// Network globals declared early: sendEvent() below publishes through them.
static WiFiClient   net;
static PubSubClient mqtt(net);
static char deviceId[12];
static char topicTelemetry[48], topicStatus[48], topicEvent[48], topicCmd[48];
#endif

// ============================ EVENT SINKS ==================================
// Rate-limited everywhere; urgent events (alarm/fault/clear) get a shorter
// floor than routine warnings so a developing incident is never muted.
static void sendEvent(const char* code, const char* msg, bool urgent) {
  uint32_t now = millis();
  uint32_t floorMs = urgent ? URGENT_GAP_MS : EVENT_GAP_MS;
  if (lastEventAt != 0 && (uint32_t)(now - lastEventAt) < floorMs) {
    Serial.println("[event] suppressed (rate limit)");
    return;
  }
  lastEventAt = now;
  eventCount++;
  Serial.printf("[event] %s: %s\n", code, msg);

#if !SIMULATION_BUILD
  if (Blynk.connected()) Blynk.logEvent(code, msg);
  if (mqtt.connected()) {
    char json[192];
    snprintf(json, sizeof(json),
             "{\"type\":\"%s\",\"msg\":\"%s\",\"t\":%lu}",
             code, msg, (unsigned long)(now / 1000UL));
    mqtt.publish(topicEvent, json);
  }
  persistSave();
#endif
}

#if !SIMULATION_BUILD
static void publishTelemetry();
#endif

// ============================ CONNECTIVITY ================================
#if !SIMULATION_BUILD
static uint32_t lastWifiTry = 1, lastMqttTry = 1, lastBlynkTry = 1;  // =1 so the first gate fires at t=interval; 0 would deadlock reached()

static void buildTopics() {
  snprintf(deviceId, sizeof(deviceId), "%08X", ESP.getChipId());
  snprintf(topicTelemetry, sizeof(topicTelemetry), "%s/%s/telemetry", TOPIC_ROOT, deviceId);
  snprintf(topicStatus,    sizeof(topicStatus),    "%s/%s/status",    TOPIC_ROOT, deviceId);
  snprintf(topicEvent,     sizeof(topicEvent),     "%s/%s/event",     TOPIC_ROOT, deviceId);
  snprintf(topicCmd,       sizeof(topicCmd),       "%s/%s/cmd",       TOPIC_ROOT, deviceId);
}

static void handleCommand(const char* cmd);

static void mqttCallback(char* topic, byte* payload, unsigned int len) {
  char cmd[24];
  if (len >= sizeof(cmd)) len = sizeof(cmd) - 1;
  memcpy(cmd, payload, len);
  cmd[len] = '\0';
  for (unsigned int i = 0; cmd[i]; i++) cmd[i] = tolower(cmd[i]);
  handleCommand(cmd);
}

static void connectivity(uint32_t now) {
  static bool wifiWasUp = false;
  bool wifiUp = WiFi.status() == WL_CONNECTED;
  if (wifiUp && !wifiWasUp) {
    wifiWasUp = true;
    Serial.printf("[wifi] connected to %s  ip=%s\n",
                  WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
  } else if (!wifiUp && wifiWasUp) {
    wifiWasUp = false;
    Serial.println("[wifi] lost connection");
  }

  if (!wifiUp && reached(now, lastWifiTry, 10000UL)) {
    lastWifiTry = now;
    WiFi.reconnect();
    Serial.println("[wifi] reconnect requested");
  }

  if (Blynk.connected()) {
    Blynk.run();
  } else if (reached(now, lastBlynkTry, 5000UL)) {
    lastBlynkTry = now;
    if (WiFi.status() == WL_CONNECTED) Blynk.connect(2000);  // bounded wait
  }

  if (mqtt.connected()) {
    mqtt.loop();
  } else if (reached(now, lastMqttTry, 5000UL)) {
    lastMqttTry = now;
    if (WiFi.status() == WL_CONNECTED) {
      char cid[20];
      snprintf(cid, sizeof(cid), "slk-%s", deviceId);
      // Last-Will: the broker marks us offline if we vanish without goodbye.
      if (mqtt.connect(cid, MQTT_USER, MQTT_PASS, topicStatus, 0, true, "offline")) {
        mqtt.publish(topicStatus, "online", true);
        mqtt.subscribe(topicCmd);
        Serial.printf("[mqtt] connected as %s\n", cid);
        publishTelemetry();
      } else {
        Serial.printf("[mqtt] connect failed, rc=%d\n", mqtt.state());
      }
    }
  }
}
#endif  // !SIMULATION_BUILD

// ============================== REPORTING =================================
static void publishTelemetry() {
#if !SIMULATION_BUILD
  char json[384];
  int rssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
  snprintf(json, sizeof(json),
    "{\"id\":\"%s\",\"fw\":\"%s\",\"uptime\":%lu,\"state\":\"%s\","
    "\"raw\":%d,\"smooth\":%.1f,\"baseline\":%.1f,\"delta\":%.1f,"
    "\"ppm\":%.0f,\"valve\":%d,\"valveHold\":%d,\"rssi\":%d,\"heap\":%lu,"
    "\"blynk\":%d,\"mqtt\":%d,\"events\":%lu}",
    deviceId, FW_VERSION, (unsigned long)((millis() - bootTime) / 1000UL),
    stateName(gState), lastRaw,
    emaValid ? ema : 0.0f, baselineReady ? activeBaseline() : 0.0f,
    baselineReady ? (ema - activeBaseline()) : 0.0f, estimatePpm(),
    valvePos == VALVE_CLOSED ? 1 : 0, valveHoldClosed ? 1 : 0;
    rssi, (unsigned long)ESP.getFreeHeap(),
    Blynk.connected() ? 1 : 0, mqtt.connected() ? 1 : 0,
    (unsigned long)eventCount);
  if (mqtt.connected()) mqtt.publish(topicTelemetry, json);
  if (Blynk.connected()) {
    Blynk.virtualWrite(V0, lastRaw);
    Blynk.virtualWrite(V1, emaValid ? ema : 0.0);
    Blynk.virtualWrite(V2, baselineReady ? activeBaseline() : 0.0);
    Blynk.virtualWrite(V3, stateName(gState));
    Blynk.virtualWrite(V4, estimatePpm());
    Blynk.virtualWrite(V5, baselineReady ? (ema - activeBaseline()) : 0.0);
  }
#endif
}

static void reportTick(uint32_t now) {
  if (reached(now, lastSerialAt, SERIAL_MS)) {
    lastSerialAt = now;
    Serial.printf(
      "ADC=%d sm=%.1f base=%.1f thr=%.1f state=%s",
      lastRaw, ema, baseline, activeBaseline(), stateName(gState));
    Serial.printf(" valve=%s%s",
                  valvePos == VALVE_CLOSED ? "CLOSED" : "open",
                  valveHoldClosed ? "(hold)" : "");
    if (now < buzzerMutedUntil) Serial.printf(" muted=%lus",
                  (unsigned long)((buzzerMutedUntil - now) / 1000UL));
#if !SIMULATION_BUILD
    Serial.printf(" heap=%lu wifi=%d blynk=%d mqtt=%d",
                  (unsigned long)ESP.getFreeHeap(),
                  WiFi.status() == WL_CONNECTED,
                  Blynk.connected(), mqtt.connected());
#endif
    Serial.println();
  }
  if (reached(now, lastPublishAt, PUBLISH_MS)) {
    lastPublishAt = now;
    publishTelemetry();
  }
}

// ============================ STATE ACTIONS ===============================
static void enterCalibrating(uint32_t now) {
  gState = ST_CALIBRATING;
  calibStart = now;
  calibSum = 0.0f;
  calibCount = 0;
  calibMin = 1023;
  calibMax = 0;
  Serial.println("Baseline calibration: keep the air clean and still.");
}

static void enterSafe(uint32_t now, bool announce) {
  bool wasBad = (gState == ST_WARNING || gState == ST_ALARM);
  gState = ST_SAFE;
  highSince = severeSince = lowSince = 0;
  lastFastRiseAt = 0;          // stale slope must not leak into next event
  slopeRef = ema;
  lastSlopeWin = now;
  if (wasBad && announce) {
    sendEvent("gas_clear",
              valveHoldClosed
                ? "Air normal again -- valve stays CLOSED (remote hold)."
                : "Signal normal again. Warning cleared.",
              true);
  }
}

static void enterWarning(uint32_t now) {
  gState = ST_WARNING;
  lowSince = 0;
  char msg[96];
  snprintf(msg, sizeof(msg),
           "Elevated gas signal: ADC %.0f (thr %.0f, +%0.f).",
           ema, activeBaseline(), ema - activeBaseline());
  sendEvent("gas_warning", msg, false);
}

static void enterAlarm(uint32_t now) {
  gState = ST_ALARM;
  lowSince = 0;
  char msg[112];
  snprintf(msg, sizeof(msg),
           "GAS ALARM: ADC %.0f (thr %.0f, +%0.f, est %.0f ppm).",
           ema, activeBaseline(), ema - activeBaseline(), estimatePpm());
  sendEvent("gas_alarm", msg, true);
}

static void enterFault() {
  gState = ST_FAULT;
  faultClearSince = 0;
  char msg[96];
  snprintf(msg, sizeof(msg),
           "Sensor fault: ADC pinned at %d. Check wiring.", lastRaw);
  sendEvent("device_fault", msg, true);
}

// =========================== SENSING PIPELINE =============================
static void calibrationTick(uint32_t now) {
  calibSum += ema;
  calibCount++;
  if ((int)ema < calibMin) calibMin = (int)ema;
  if ((int)ema > calibMax) calibMax = (int)ema;

  if (!reached(now, calibStart, CALIB_MS)) return;

  if (calibCount < 20) {
    Serial.println("Calibration retry: too few samples.");
    enterCalibrating(now);
    return;
  }
  if (calibMax - calibMin > 40 && calibRetries < 1) {
    calibRetries++;
    Serial.println("Calibration retry: air unstable.");
    enterCalibrating(now);
    return;
  }

  baseline = calibSum / calibCount;
  baselineReady = true;
  ema = baseline;
#if !SIMULATION_BUILD
  persistSave();
#endif
  Serial.printf("Clean-air baseline (ADC): %.1f\n", baseline);
  enterSafe(now, false);
  Serial.println("Monitoring. Press INJECT GAS (or 'i') for a test leak.");
}

static void faultTick(uint32_t now) {
  if (lastRaw <= 3 || lastRaw >= 1020) {
    if (!faultSince) faultSince = now;
    if (reached(now, faultSince, 10000UL)) enterFault();
  } else {
    faultSince = 0;
    if (gState == ST_FAULT) {
      if (!faultClearSince) faultClearSince = now;
      if (reached(now, faultClearSince, 5000UL)) {
        sendEvent("device_ok", "Sensor reading restored; recalibrating.", true);
        enterCalibrating(now);
      }
    }
  }
}

static void monitorTick(uint32_t now) {
  float thr = activeBaseline();

  // One-second slope window: a fast rise escalates without waiting.
  if (reached(now, lastSlopeWin, 1000UL)) {
    if (lastSlopeWin != 0) {
      float rise = ema - slopeRef;
      if (rise >= SLOPE_RISE_1S) lastFastRiseAt = now;
    }
    slopeRef = ema;
    lastSlopeWin = now;
  }
  bool fastRise = lastFastRiseAt != 0 &&
                  (uint32_t)(now - lastFastRiseAt) <= 2000UL;

  bool elevated = ema >= thr + WARN_DELTA;
  bool severe   = ema >= thr + ALARM_DELTA;

  if (gState == ST_SAFE || gState == ST_WARNING) {
    if (elevated) { if (!highSince) highSince = now; }
    else { highSince = 0; }

    if (severe) { if (!severeSince) severeSince = now; }
    else { severeSince = 0; }

    if (severe && (reached(now, severeSince, ALARM_CONFIRM_MS) || fastRise)) {
      enterAlarm(now);
    } else if (gState == ST_SAFE &&
               elevated && reached(now, highSince, WARN_CONFIRM_MS)) {
      enterWarning(now);
    }
  } else if (gState == ST_ALARM || gState == ST_WARNING) {
    if (gState == ST_WARNING) {
      if (severe) { if (!severeSince) severeSince = now; }
      if (severe && (reached(now, severeSince, ALARM_CONFIRM_MS) || fastRise)) {
        enterAlarm(now);
        return;
      }
    }
    // Hysteresis + dwell: sit below thr+CLEAR_DELTA for CLEAR_MS to clear.
    if (ema < thr + CLEAR_DELTA) {
      if (!lowSince) lowSince = now;
      if (reached(now, lowSince, CLEAR_MS)) enterSafe(now, true);
    } else {
      lowSince = 0;
    }
  }
}

static void baselineDrift() {
  // Adapt only while clearly safe, so the baseline can never creep up
  // into a real leak. ~10 minute time constant.
  if (gState != ST_SAFE) return;
  if (ema >= activeBaseline() + 10.0f) return;
  baseline += BASELINE_ALPHA * (ema - baseline);
}

static void sampleTick(uint32_t now) {
  if (!reached(now, lastSampleAt, SAMPLE_MS)) return;
  lastSampleAt = now;

  lastRaw = analogRead(GAS_PIN);

  // Test injection sits at the input layer: every downstream stage
  // (median, EMA, slope, thresholds) processes it exactly like real gas.
  bool injecting = injectUntil != 0 && now <= injectUntil;
  if (injecting) {
    if (!injectActive) {
      injectActive = true;
      injectStart = now;
      Serial.println("*** TEST INJECTION: simulated gas ramp for 12 s ***");
    }
    float p = (float)(now - injectStart) / (float)INJECT_RAMP_MS;
    if (p > 1.0f) p = 1.0f;
    lastRaw = (int)(activeBaseline() + INJECT_DELTA * p);
  } else if (injectActive) {
    injectActive = false;
    Serial.println("*** TEST INJECTION ended ***");
  }

  int med = medianPush(lastRaw);
  if (!emaValid) { ema = med; emaValid = true; }
  else { ema += EMA_ALPHA * (med - ema); }

  if (!baselineReady) {
    if (gState == ST_WARMUP) {
      if (reached(now, lastWarmupPrint, WARMUP_PRINT_MS)) {
        lastWarmupPrint = now;
        uint32_t left = (now - warmupStart < WARMUP_MS)
                          ? (WARMUP_MS - (now - warmupStart)) / 1000UL : 0;
        Serial.printf("Warming up (%lus left); hardware path remains active.\n",
                      (unsigned long)left);
      }
      if (reached(now, warmupStart, WARMUP_MS)) enterCalibrating(now);
    } else if (gState == ST_CALIBRATING) {
      calibrationTick(now);
    }
    return;
  }

  if (gState == ST_CALIBRATING) {
    calibrationTick(now);
    faultTick(now);
    return;
  }

  faultTick(now);
  if (gState == ST_FAULT) return;
  monitorTick(now);
  baselineDrift();
}

// ============================== ACTUATORS =================================
// Yellow status LED: every state has its own signature.
static void updateLed(uint32_t now) {
  bool on = false;
  switch (gState) {
    case ST_WARMUP:      on = (now % 1000UL) < 500UL; break;  // slow blink
    case ST_CALIBRATING: on = (now % 250UL)  < 125UL; break;  // fast blink
    case ST_SAFE:        on = (now % 5000UL) < 80UL;  break;  // heartbeat
    case ST_WARNING:     on = (now % 400UL)  < 200UL; break;  // blink
    case ST_ALARM:       on = true;                           break;
    case ST_FAULT:       on = (now % 2000UL) < 1400UL; break; // long on
    default:             on = false;
  }
  digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
}

// Buzzer: software tone where wired (Wokwi). On the real build the buzzer
// hangs off the relay, so this function stays inert and the hardware path
// cannot be muted from software.
static void updateBuzzer(uint32_t now) {
  if (BUZZER_PIN < 0) return;
  int want = -1;  // -1 = silent
  if (now >= buzzerMutedUntil) {
    if (gState == ST_ALARM) want = 2200;
    else if (gState == ST_WARNING && (now % 1000UL) < 250UL) want = 1800;
    else if (gState == ST_FAULT && (now % 2000UL) < 200UL) want = 1000;
  }
  if (want != lastBuzFreq) {
    lastBuzFreq = want;
    if (want < 0) noTone(BUZZER_PIN);
    else tone(BUZZER_PIN, want);
  }
}

// Gas valve: closed only in a confirmed ALARM, reopened when safe.
// A REMOTE HOLD (dashboard button / CLI 'V') keeps it shut even after the
// air clears -- you are away and the gas must not come back on without you.
// FAULT leaves the valve where it is -- a wiring fault must not yank the
// valve either way. Boot fails open: you do not want a reboot cutting gas.
static void updateValve() {
  int want;
  if (gState == ST_ALARM) want = VALVE_CLOSED;
  else if (gState == ST_FAULT) return;
  else if (valveHoldClosed) want = VALVE_CLOSED;
  else want = VALVE_OPEN;
  if (want == valveWanted) return;
  valveWanted = want;
  valvePos = want;
  // Kick-start the sweep immediately instead of waiting up to
  // VALVE_REFRESH_MS — eliminates the 4-second stall before the
  // servo begins moving after a state change.
  valveNextWriteAt = millis();
  valveArrivedAt = 0;
  Serial.printf("Gas valve %s (sweeping %d deg)\n",
                want == VALVE_CLOSED ? "CLOSED (servo)" : "OPEN",
                abs(want - valveCmd));
}

// One write at a chosen angle, from whatever attach/park state we are in.
static void servoDriveDeg(int deg) {
#if SIMULATION_BUILD
  servoWriteDeg(deg);
#else
  if (!servoEngaged) { servoValve.attach(SERVO_PIN, 544, 2400); servoEngaged = true; }
  servoValve.write(deg);
#endif
}

// Non-blocking 1-degree sweep toward valveWanted. A single 0->180 slam
// draws a stall-grade current spike that can brown the board out mid-move
// and leave the valve half-shut ("stuck"); stepping keeps the current
// sane. Once in position the pulses are held VALVE_SETTLE_MS and then the
// servo is PARKED (real build): a servo grinding into its mechanical
// stop draws 0.5-1 A forever, which sags the 5 V rail, kills the USB link
// and reboots the board mid-alarm -- the "valve got stuck" failure mode.
// Parked = zero holding current; the valve holds on gear friction.
static void valveTick(uint32_t now) {
  if (now < valveNextWriteAt) return;
  if (valveCmd != valveWanted) {
    valveCmd += (valveWanted > valveCmd) ? 1 : -1;
    servoDriveDeg(valveCmd);
    valveNextWriteAt = now + VALVE_STEP_MS;
    valveArrivedAt = 0;                      // moving again: restart settle
    return;
  }
  if (!valveArrivedAt) {                     // just arrived at target
    valveArrivedAt = now;
    valveNextWriteAt = now + VALVE_SETTLE_MS;
    return;
  }
#if !SIMULATION_BUILD
  if (servoEngaged && (now - valveArrivedAt) >= VALVE_SETTLE_MS) {
    servoValve.detach();                     // park: 0 holding current
    servoEngaged = false;
    Serial.println("Valve in position; servo parked (no holding current).");
    valveNextWriteAt = now + VALVE_REFRESH_MS;
    return;
  }
#endif
  valveNextWriteAt = now + VALVE_REFRESH_MS; // idle poll (sim never parks)
}

// =========================== INPUTS / ACTIONS =============================
static void doAcknowledgeOrTest(uint32_t now) {
  if (gState == ST_ALARM || gState == ST_WARNING) {
    buzzerMutedUntil = now + ACK_MS;
    Serial.printf("Acknowledged: buzzer muted %lu s. "
                  "LEDs, relay and valve keep alarming.\n",
                  (unsigned long)(ACK_MS / 1000UL));
  } else {
    sendEvent("test", "TEST ONLY: Silent Leak notification path check.", true);
  }
}

static void startInjection(uint32_t now) {
  injectUntil = now + INJECT_MS;
  if (!injectActive && baselineReady) {
    injectActive = true;
    injectStart = now;
    Serial.println("*** TEST INJECTION: simulated gas ramp for 12 s ***");
  }
}

static void readInputs(uint32_t now) {
  // Sim-only INJECT GAS pushbutton: held = keep the ramp alive.
  // The hardware build has no buttons; silence/ack and inject arrive via
  // the serial CLI or the MQTT cmd topic (same code path, both targets).
  if (BTN_INJECT >= 0 && digitalRead(BTN_INJECT) == LOW && baselineReady) {
    injectUntil = now + INJECT_MS;
  }
}

// ============================== CLI =======================================
static void handleCommand(const char* cmd) {
  uint32_t now = millis();
  if (strcmp(cmd, "test") == 0 || strcmp(cmd, "t") == 0) {
    lastEventAt = 0;
    sendEvent("test", "TEST ONLY: Silent Leak notification path check.", true);
  } else if (strcmp(cmd, "inject") == 0 || strcmp(cmd, "i") == 0) {
    if (baselineReady) startInjection(now);
    else Serial.println("Not ready: finish calibration first.");
  } else if (strcmp(cmd, "silence") == 0 || strcmp(cmd, "ack") == 0 ||
             strcmp(cmd, "a") == 0) {
    doAcknowledgeOrTest(now);
  } else if (strcmp(cmd, "vclose") == 0 || strcmp(cmd, "valveclose") == 0 ||
             strcmp(cmd, "v") == 0) {
    // Manual shutoff from anywhere: the dashboard button sends 'vclose'.
    valveHoldClosed = true;
    updateValve();
    Serial.println("Valve CLOSED by remote request; held until released.");
    sendEvent("valve_cmd", "Valve CLOSED by remote request (held until released).", true);
  } else if (strcmp(cmd, "vopen") == 0 || strcmp(cmd, "valveopen") == 0 ||
             strcmp(cmd, "o") == 0) {
    if (gState == ST_ALARM) {
      // Safety interlock: nobody reopens the valve over the internet
      // while gas is actually detected.
      Serial.println("Refused: the valve stays shut during a GAS ALARM.");
      sendEvent("valve_refused", "Remote OPEN refused: GAS ALARM active.", true);
    } else {
      valveHoldClosed = false;
      updateValve();
      Serial.println("Valve released to automatic control.");
      sendEvent("valve_cmd", "Valve released to automatic control.", false);
    }
  } else if (strcmp(cmd, "cal") == 0 || strcmp(cmd, "c") == 0) {
    baselineReady = false;
    warmupStart = millis() - WARMUP_MS;  // sensor already hot: skip warm-up
    gState = ST_WARMUP;
    Serial.println("Recalibration requested.");
  } else if (strcmp(cmd, "status") == 0 || strcmp(cmd, "s") == 0) {
    Serial.printf(
      "{\"state\":\"%s\",\"raw\":%d,\"smooth\":%.1f,\"baseline\":%.1f,"
      "\"ppm\":%.0f,\"events\":%lu,\"valve\":%d,\"hold\":%d}\n",
      stateName(gState), lastRaw, ema, baseline, estimatePpm(),
      (unsigned long)eventCount,
      valvePos == VALVE_CLOSED ? 1 : 0, valveHoldClosed ? 1 : 0);
  } else if (strcmp(cmd, "report") == 0 || strcmp(cmd, "p") == 0) {
    Serial.println("--- Silent Leak Pro report ---");
#if !SIMULATION_BUILD
    Serial.printf("device id : %s\n", deviceId);
    Serial.printf("firmware  : %s\n", FW_VERSION);
#endif
    Serial.printf("mode      : %s\n",
#if SIMULATION_BUILD
                  "Wokwi simulation (ESP32)"
#else
                  "ESP8266 hardware"
#endif
                  );
    Serial.printf("uptime s  : %lu\n",
                  (unsigned long)((millis() - bootTime) / 1000UL));
    Serial.printf("state     : %s\n", stateName(gState));
    Serial.printf("raw/ema   : %d / %.1f\n", lastRaw, ema);
    Serial.printf("baseline  : %.1f (thr %.1f)\n", baseline, activeBaseline());
    Serial.printf("valve     : %s%s\n",
                  valvePos == VALVE_CLOSED ? "CLOSED" : "open",
                  valveHoldClosed ? " (remote hold)" : "");
    Serial.printf("ppm est   : %.0f\n", estimatePpm());
    Serial.printf("events    : %lu\n", (unsigned long)eventCount);
  } else if (strcmp(cmd, "reboot") == 0 || strcmp(cmd, "r") == 0) {
    Serial.println("Rebooting...");
#if !SIMULATION_BUILD
    if (mqtt.connected()) mqtt.disconnect();
#endif
    ESP.restart();
  } else if (strcmp(cmd, "help") == 0 || strcmp(cmd, "h") == 0 ||
             strcmp(cmd, "?") == 0) {
    Serial.println("T=test  I=inject test gas  A=acknowledge/silence 20 s");
    Serial.println("V=close valve (remote hold)  O=open valve (release hold)");
    Serial.println("C=recalibrate  P=report  S=status JSON  R=reboot  H=help");
  }
}

static void serialCli() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r' || c == ' ') continue;
    char cmd[2] = {(char)tolower(c), '\0'};
    handleCommand(cmd);
  }
}

// ============================== SETUP/LOOP ================================
#if !SIMULATION_BUILD
BLYNK_CONNECTED() {
  publishTelemetry();
}

static void servoInit() {
  // 544..2400 us: the SG90's spec pulse window (0..180 deg). 500 us drove
  // slightly past nominal-zero and pushed the horn into the stop harder.
  servoValve.attach(SERVO_PIN, 544, 2400);
  servoValve.write(VALVE_OPEN);
  servoEngaged = true;   // first valveTick parks it after the settle window
}
#else
// LEDC PWM without a servo library: arduino-esp32 3.x changed the API,
// so pick the right one at compile time (Wokwi's builder and the local
// arduino-cli install can differ).
static void servoInit() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(SERVO_PIN, 50, 16);
#else
  ledcSetup(0, 50, 16);
  ledcAttachPin(SERVO_PIN, 0);
#endif
  servoWriteDeg(VALVE_OPEN);
  servoEngaged = true;
}

static void servoWriteDeg(int deg) {
  // 50 Hz, 16-bit: 1 ms = 3277 counts, 2 ms = 6553 counts.
  uint32_t duty = (uint32_t)map(deg, 0, 180, 3277, 6553);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(SERVO_PIN, duty);
#else
  ledcWrite(0, duty);
#endif
}
#endif

void setup() {
  Serial.begin(115200);
  delay(50);

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  if (BUZZER_PIN >= 0) pinMode(BUZZER_PIN, OUTPUT);
  if (BTN_INJECT >= 0)  pinMode(BTN_INJECT, INPUT_PULLUP);

  Serial.println();
  Serial.println("=== PS6 Silent Leak Pro v2.2 ===");
  Serial.println("Hardware alarm remains wired independently of this code.");

#if SIMULATION_BUILD
  analogReadResolution(10);   // keep 0-1023 thresholds comparable
  Serial.println("Mode: Wokwi ESP32 simulation. Timings accelerated.");
  Serial.println("Press INJECT GAS (key G) for a full alarm cycle.");
#else
  buildTopics();
  Serial.printf("Device id: %s\n", deviceId);
  persistLoad();

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.hostname(deviceId);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Blynk.config(BLYNK_AUTH_TOKEN);
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(512);
  mqtt.setKeepAlive(30);

  // Crash/WDT restarts deserve eyes at the bench: a servo grinding into its
  // end stop sags the 5 V rail and can reboot the chip mid-alarm.
  {
    const rst_info* ri = ESP.getResetInfoPtr();
    if (ri->reason == REASON_WDT_RST || ri->reason == REASON_EXCEPTION_RST ||
        ri->reason == REASON_SOFT_WDT_RST) {
      Serial.printf("*** Unexpected restart (reason: %s). If it followed a "
                    "valve move, power the SG90 separately or add "
                    "470-1000 uF across its 5V/GND. ***\n",
                    ESP.getResetReason().c_str());
    }
  }
#endif

  servoInit();

  bootTime = millis();
  warmupStart = millis();
  gState = ST_WARMUP;

  Serial.printf("Warm-up started (%lu s). Keep sensor in clean air.\n",
                (unsigned long)(WARMUP_MS / 1000UL));
  Serial.println("Send 'H' for commands.");
}

void loop() {
#if !SIMULATION_BUILD
  ESP.wdtFeed();
  uint32_t now = millis();
  connectivity(now);
#else
  delay(10);                 // let the simulator idle between samples
  uint32_t now = millis();
#endif

  readInputs(now);
  sampleTick(now);
  reportTick(now);
  updateLed(now);
  updateBuzzer(now);
  updateValve();
  valveTick(now);
  serialCli();
}
