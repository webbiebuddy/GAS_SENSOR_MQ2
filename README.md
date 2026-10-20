# GAS_SENSOR_MQ2
A MQ2-ESP8266-SERVO90(S90)-Relay based project that simply detects the gas leakage, updates the IP based webpage about the concentration and simply closes the main valve after a limit.

# Silent Leak Pro — MQ-2 gas leak detector + live web dashboard

An ESP8266 gas-leak monitor with **two independent alert paths**, plus a browser
dashboard that shows live status from anywhere.

```
MQ-2 sensor ──► comparator ──► relay ──► siren       (HARD path: always works)
                  │
                  └──► ESP8266 A0 ──► this firmware ──► Blynk app  (phone push)
                                         │           └─► MQTT      (this dashboard)
                                         └──────────► SG90 servo closes the gas valve
```

The hardware path is wired around the chip on purpose: if Wi-Fi, Blynk, the
MQTT broker or the ESP itself dies, the siren still sounds. The firmware here
is the *secondary* layer — smarter detection, phone notifications, and the
live dashboard.

## What's in here

| Path | What it is |
|---|---|
| `silent_leak/silent_leak.ino` | **Silent Leak Pro v2.1** — the full hardware firmware (flash this): gas + servo valve, Wi-Fi/Blynk/MQTT |
| `simulation/sketch.ino` | The same engine for **Wokwi** (`SIMULATION_BUILD 1`, ESP32, accelerated timings, no Wi-Fi) |
| `simulation/diagram.json` | Wokwi wiring: MQ-2, servo valve, relay, LEDs, INJECT GAS button |
| `dashboard/index.html` | Live web dashboard — open in a browser, no build step |
| `legacy/silent_leak_v1/` | Your original v1 sketch, kept for reference |
| `WIRING.md` | **Full connection guide**: pin map, power rules, build order, flash procedure, troubleshooting |
| `The_Silent_Leak_Presentation.pptx` | EMBED-X final-round deck (11 slides, speaker notes with a 5-min budget) |
| `ppt/build.js` | Script that regenerates the deck (`node ppt/build.js`) |
| `IDEAS.md` | Hackathon idea bank — what to build/demo next |
| `README.md` | This file |

The two `.ino` files are the **same detection engine**, identical apart from the
`SIMULATION_BUILD` flag (`0` = real ESP8266, `1` = Wokwi ESP32). If you change
the detection logic, change it in both.

## Firmware v2 vs v1

| | v1 | v2 (Pro) |
|---|---|---|
| Wi-Fi / Blynk start | `Blynk.begin()` — **blocks forever** if Wi-Fi is down | Non-blocking, throttled reconnects; sensing continues offline |
| Filtering | raw `analogRead()` — one spike can fire the alarm | median-of-15 + exponential smoother |
| Baseline | one-shot average at boot, never updated | slow adaptive drift **frozen during events** so it can't creep into a leak |
| Alarm levels | single threshold | WARNING + ALARM; a **fast rise escalates immediately** |
| Sensor fault | none | ADC pinned at rails for 10 s raises `device_fault` |
| Reporting | Blynk only | Blynk **and** MQTT (with Last-Will, so the dashboard shows OFFLINE the moment the device dies) |
| Persistence | none | baseline, boot count, event count in EEPROM |
| Autonomous actuation | none | **SG90 closes the gas valve on confirmed ALARM**; reopens only after the clear path, `FAULT` never moves it, boot fails open |
| Extra inputs | none | none on purpose — a **minimal core** (gas + valve + alarm + telemetry) where every part earns its place |
| Control | serial `T` only | serial CLI (`T I A C P S R H`) **and** remote commands over MQTT |
| Memory | `String` heap churn in the hot path | fixed buffers only, no allocation |

### States

`WARMUP` (10 min heater) → `CALIBRATING` (30 s clean-air baseline) → `SAFE` ⇄
`WARNING` ⇄ `ALARM`, with `FAULT` on sensor-wiring failure. In `ALARM` the
**servo valve closes the supply**; it reopens only after the hysteresis clear,
a `FAULT` leaves it where it is, and boot fails open — a reboot must never cut
the gas. A **remote hold** (`V` / dashboard **Close valve**) keeps the valve
shut even after the air clears: away from home, the gas does not come back on
without an explicit release.

### Serial CLI

`T` test event · `I` inject a simulated leak (full alarm demo, no gas) ·
`A` acknowledge (silence the buzzer 20 s during an alarm; test event when
safe) · `V` close the valve (remote hold) · `O` open the valve (release) ·
`C` recalibrate · `P` human report · `S` status JSON · `R` reboot ·
`H` help. The same words work over MQTT on `silentleak/<id>/cmd` — so the
dashboard's **Silence** and **valve** buttons drive the same code path.
Acknowledge only mutes the buzzer: LEDs, relay and the closed valve keep
alarming. Remote close always works; remote open is **refused during a live
ALARM** — the internet cannot defeat the safety loop.

## MQTT topics

Device id is the ESP chip id, e.g. `A1B2C3D4`.

| Topic | Direction | Payload |
|---|---|---|
| `silentleak/<id>/telemetry` | device → | JSON every 2 s: state, raw, smooth, baseline, delta, ppm, **valve**, **valveHold**, rssi, heap, events |
| `silentleak/<id>/status` | device → | retained `online` / `offline` (offline is the Last-Will) |
| `silentleak/<id>/event` | device → | `gas_warning` / `gas_alarm` / `gas_clear` / `device_fault` / `valve_cmd` / `valve_refused` / `test` |
| `silentleak/<id>/cmd` | → device | `test` \| `inject` \| `silence` \| **`vclose`** \| **`vopen`** \| `cal` \| `status` \| `reboot` |

The firmware defaults to the **public HiveMQ test broker** so the dashboard
works with zero setup. That topic is readable by anyone — fine for a demo, so
switch `MQTT_HOST` to your own broker (Mosquitto, EMQX, HiveMQ Cloud…) for a
real installation, and set `MQTT_USER` / `MQTT_PASS`.

## Dashboard

Open `dashboard/index.html` in any browser — no server, no build. Then:

1. Pick a broker (EMQX or HiveMQ public), keep topic root `silentleak`, hit **Connect**.
2. Flash the firmware with your Wi-Fi credentials; within seconds the device
   appears under *Devices seen* with live values, chart and event log.
3. No hardware yet? Flip **Simulate device** — a fake `SIM-DEMO` device runs the
   same thresholds so you can see a leak ramp up, alarm, and clear. **Simulate
   gas leak** injects one on demand.
4. Turn on **Sound** — the dashboard synthesizes three voices (no audio files):
   paired **warning beeps**, a continuous warbling **siren** during ALARM, and
   low **honk** triads for watchdog/offline. The siren dies the instant
   `gas_clear` arrives.
5. Turn on **Notify** — OS-level **desktop notifications** (permission prompt
   on first arm) for alarm, warning, fault, device-offline, watchdog and test
   events: you get an OS toast even when the tab is minimized. The tab title
   flips to 🔴 ALARM across the room too.
6. **Not at home?** The **Gas valve · remote control** panel sends `vclose` /
   `vopen` to the device over MQTT — slam the valve shut from your phone, or
   release it later. Closing always works; opening is refused by the firmware
   while gas is detected, and the card shows `CLOSED ▸ HELD` whenever a
   remote hold is keeping the valve shut.
6. **Watchdog** — if the device goes silent for 12 s while the link is up, the
   dashboard shouts: banner + honks + notification + logged evidence, and
   clears itself when telemetry resumes. Complements the MQTT Last-Will.
7. **Second opinion** — the page recomputes the thresholds itself; if the
   device claims SAFE/WARNING while the signal has sat above the ALARM level
   for 8+ s, it flags the disagreement instead of trusting blindly.
8. **Export** — downloads device state + the full event log as JSON evidence.
   The event log also survives page reloads (localStorage).
9. Command buttons (Test · **Silence 20 s** · **Inject test** · Recalibrate ·
   Status · Reboot) publish straight to the device's `cmd` topic.

The chart shows raw, smoothed, baseline, and the alarm level. Everything is
client-side; the page talks directly to the broker over WebSocket.

## Your kit (BOM) → what each part does

**In the firmware now:**

| Kit part | Role |
|---|---|
| ESP8266 + cable | Controller (the Wokwi sim uses an ESP32 — simulator's platform — one flag apart) |
| MQ-2 sensor | Core gas detector → A0; DOUT → relay → buzzer = the always-alive hardware alarm |
| **SG90 servo** | **Autonomous actuator**: closes the main gas valve on confirmed ALARM, reopens when the air is safe |
| Relay module 5V | Fail-safe alarm rail between the MQ-2 comparator and the buzzer |
| Buzzer | "Warn people nearby" sound, fed from the relay so it works with the ESP dead |
| LED ×5 / resistor ×3 | Green safe, red alarm, yellow status (3 in use), 2 spare for valve-position indication |
| Breadboard, jumpers | Assembly |

**Deliberately dropped from the build** (they diluted the safety story):
DHT11, IR flame module, TTP224 touch. Nothing else in the kit is wired, and
every wired part now maps 1:1 to the problem statement: detect · act · warn.

**Not yet wired — proposed in [IDEAS.md](IDEAS.md):**

| Kit part | Proposed role |
|---|---|
| IC 555 | Hardware monostable/latch that stretches the comparator pulse so the siren can't chatter off — the "controller hangs" proof |
| LDR module | Night mode (raise sensitivity after dark) + alarm-LED proving (verify the red LED actually lit) |
| Tilt sensor | Anti-tamper: guardian knocked over or displaced → event |
| MPU6050 | Knock-to-silence gesture + orientation/tamper diagnostics |
| Soil moisture sensor | Water-leak channel → gas + fire + flood triple-hazard guardian |

## Simulation (Wokwi)

`simulation/` is a ready-to-run Wokwi project — open it in the Wokwi VS Code
extension (or wokwi.com) and press Start. No hardware needed.

- **INJECT GAS** button (or key `G`) ramps a fake leak through the real
  pipeline (median → EMA → slope → thresholds): full WARNING → ALARM →
  valve-close → clear demo in ~15 s. Timings are accelerated (1 s warm-up,
  2.5 s calibration); the hardware build uses the real 10 min / 30 s.
- Serial CLI works: `T I A C P S R H` (send keys through the Wokwi serial
  monitor; `A` silences the buzzer 20 s during an alarm).
- No extra libraries needed — the sim is dependency-free now.
- The sim targets ESP32 (Wokwi has no ESP8266 board). The real firmware is the
  same file with `SIMULATION_BUILD 0`.

## Blynk setup

1. Create a template (ESP8266) and a device, copy the auth token.
2. Paste `TMPLxxxx`, the template name and the token at the top of the sketch.
3. In the template's **Events** tab add codes: `gas_warning`, `gas_alarm`,
   `gas_clear`, `device_fault`, `device_ok`, `test` — `Blynk.logEvent` needs
   them to exist.
4. Optional gauges on virtual pins: `V0` raw, `V1` smoothed, `V2` baseline,
   `V3` state, `V4` ppm estimate, `V5` delta.

## Wiring (NodeMCU / Wemos D1 mini)

| MQ-2 pin | ESP8266 | Note |
|---|---|---|
| VCC | 5V | heater needs 5 V |
| GND | GND | common ground |
| AOUT | A0 | analog reading |
| DOUT | relay IN | hardware alarm: drives the buzzer/siren **directly** — no firmware can disable it |
| — | D6 | status LED (yellow, per-state blink signature) |
| — | D7 | **SG90 servo signal** — gas-valve actuator (power the servo from 5 V, common GND; add a capacitor across the servo supply if it browns out the board) |
| — | — | buzzer is relay-fed (hardware path), not a GPIO |
| — | D1 D2 D5 | **unused by design** — the dropped touch/flame/DHT pins stay empty |

The MQ-2 draws ~150 mA while heating. Power it from the USB 5 V rail, not the
NodeMCU's 3V3 regulator.

## Safety note

This is a demonstration-grade software monitor. The ppm figure is an
uncalibrated estimate for trending, thresholds are room-dependent, and a
single MQ-2 channel cannot tell LPG from smoke or steam. For a real install:
keep the hardware relay path, add a proper sensor (MQ-4 for natural gas,
MQ-135 for air quality, or a real LEL sensor), and never rely on a cloud
service as your only alarm.
