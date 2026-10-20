# PS6 — "The Silent Leak": Idea Bank

Problem statement, decoded:

> Gas leaks and smoldering fires ... noticed too late, especially at night.
> Build a guardian that **detects** the hazard, **acts on its own** to reduce
> danger, and **warns people nearby and far away**. The hazard response must
> **still trigger if the main controller hangs or crashes.**

That last sentence is the real test. Every idea below is graded against it.

**Legend:** ✅ already in the firmware · 🔨 build next from the BOM · 🚀 extension (small extra part)

> **Stance update — reduced build:** DHT11, the IR flame module and the TTP224
> touch were **deliberately cut from the hardware** — they diluted the safety
> story. Ideas below stay as an idea bank; anything re-added must earn its
> place against the criterion again (silence/test now runs over MQTT/serial).

---

## Criterion → answer map

| Criterion | How this project answers it |
|---|---|
| Detects the hazard | MQ-2 median/EMA pipeline with adaptive baseline ✅ + (water 🔨) |
| Acts on its own | SG90 closes the main valve on confirmed ALARM ✅ · relay siren ✅ |
| Warns people nearby | Siren/buzzer via relay ✅ · per-state LED signatures ✅ · remote ack UX (MQTT/serial) ✅ |
| Warns people far away | Blynk push ✅ · live MQTT dashboard with offline detection ✅ · ESP-NOW no-internet peer 🚀 |
| **Survives controller hang/crash** | Hardware rail: MQ-2 DOUT → (IC 555 latch 🔨) → relay → siren — **the ESP is not in the path** ✅ (+ LWT shows OFFLINE the moment it dies ✅) |

---

## Tier 1 — working now (demo these first)

### 1. Autonomous gas-valve shutoff (the servo idea) ✅
On a **confirmed** ALARM the SG90 rotates to `VALVE_CLOSED` (180°) and cuts the
supply; a fast rise (≥15 ADC/s) skips the confirmation wait entirely. Reopen
only after the air sits below the clear threshold for the full dwell — no
flapping on a whiff. `FAULT` never moves the valve (a wiring fault must not
yank the supply), and **boot fails open** (a reboot must never cut gas).
Wokwi demo: press INJECT GAS and watch the arm sweep shut.

### 2. Controller-independent alarm rail ✅ (+ IC 555 🔨)
MQ-2 DOUT → relay → buzzer is wired around the chip: no firmware, Wi-Fi or
cloud can mute it. **Judge moment: rip the USB out mid-alarm — the siren keeps
sounding.** The sensor's own comparator makes that true today.

### 3. Detection engine that doesn't cry wolf ✅
Median-of-15 + EMA (one spike can't alarm) · baseline that **freezes during
events** (it can't drift up into a leak) · hysteresis + dwell clear ·
humidity-compensated threshold from the DHT11 (monsoon kitchens shift the MQ-2
baseline) · dead-sensor fault instead of a false "safe".

### 4. Flame = instant escalation ✅
The IR module bypasses confirmation timing: flame forces ALARM and closes the
valve immediately. Gas waits for evidence; fire does not.

### 5. Two-way warning split ✅
Nearby = siren + LEDs (works offline). Far = Blynk push + the MQTT dashboard,
with Last-Will so the phone/dashboard says OFFLINE the instant the board dies.

### 6. Touch acknowledge that can't defeat safety ✅
Hold the TTP224 during an alarm: **buzzer only** mutes for 20 s — LEDs, relay
and closed valve are unaffected. Acknowledge ≠ override.

---

## Tier 2 — build next from the kit (highest judge impact per part)

### 7. IC 555 hardware watchdog latch 🔨 — *the criterion-killer*
Nobody else will use the 555. Wire it as a **retriggerable monostable**
between the MQ-2 comparator and the relay: any detection pulse re-extends the
output, so a chattering sensor can't chatter the siren off, and the window
stays latched for ~60 s after the last detection. Crucially the ESP is *not*
in that path — it can only observe. Demo: hold the ESP in reset (or let the
loop hang) while injecting gas → **siren still sounds**. This is the literal
sentence from the problem statement, proven on a breadboard.

### 8. LDR: alarm-LED proving + night mode 🔨
Mount the LDR next to the red alarm LED (shrouded tube). Every boot the
firmware lights the LED and **reads back that it actually lit** — if the
warning lamp is dead, that's a `device_fault`, not a silent pass. Same sensor:
after dark (LDR low) raise reporting cadence and lengthen the audible pattern —
"especially at night" answered with a sensor we already have.

### 9. Tilt sensor: anti-tamper guardian 🔨
If the unit is displaced or knocked over (tilt open > 2 s) — especially during
WARNING/ALARM — raise `guardian_displaced`: someone moved the sensor away from
the leak, or it fell at night. The guardian watching its own watcher.

### 10. MPU6050: knock-to-silence + orientation baseline 🔨
Sharp impulse (> ~1.5 g spike in a 30 ms window) = a knock on the table does
what the touch pad does — no fumbling for a pad in the dark. Continuous
accelerometer baseline doubles as relocation/obstruction detection. Runs on
I2C (D3/D4 with the module's pull-ups keeping the strap pin high).

### 11. Soil moisture → water channel 🔨 — *triple-hazard guardian*
Repurpose the probe as a floor electrode: water on the floor = `flood_warning`.
One board now guards a hostel kitchen against **gas, fire and flood** — same
code shape (threshold + confirm + hysteresis), three hazards, entire BOM used.

### 12. DHT11 dT/dt fire pre-stage 🔨
Rapid temperature rise alone (≥ 2 °C in 10 s) while gas is low = "heating
event" warning — catches a smoldering fire **before** the IR flame sensor can
see one. The problem statement's "smoldering fires" answered two ways.

---

## Tier 3 — extensions (small extra parts, if time allows)

- **🚀 ESP-NOW corridor peer** — a second ESP8266 (~₹250) in the corridor
  repeats the alarm peer-to-peer: *far-away warning with the router dead*.
- **🚀 Evidence timeline** — NTP timestamps + a small EEPROM ring buffer; the
  dashboard exports "first rise → valve closed → cleared" for the post-mortem.
- **🚀 Leak-rate metric** — slope (ADC/min) as a severity gradient on the
  dashboard: "at this rate, alarm in ~40 s". Drives the escalation ladder:
  warn → valve → evacuate.
- **🚀 Multi-room map** — several boards, retained last-state per device id,
  one glance for a hostel warden (dashboard already groups devices).

---

## 3-minute judge demo (choreography)

1. **SAFE** heartbeat, dashboard green. *"Hostel kitchen, 2 a.m., everyone asleep."*
2. **INJECT GAS** → WARNING, yellow blink, phone event. *"A leak starts."*
3. Ramp → **ALARM**: servo sweeps the valve **shut**, red LED solid, siren via
   relay, dashboard flips red with audible browser alarm, Blynk push arrives.
   *"Supply cut and people warned — no human touched anything."*
4. **Unplug the ESP mid-alarm** → siren keeps sounding. *"If the brain dies,
   the guardian doesn't. That was the assignment."*
5. **Flame sensor** in front of the IR lamp → instant alarm with zero gas.
   *"Second hazard, no confirmation delay."*
6. **Touch-hold** → buzzer mutes, valve stays shut. *"Humans can acknowledge.
   Humans cannot override."*
7. Air clears → valve reopens only after the dwell; dashboard timeline shows
   every step with timestamps.

## Honesty boundaries (say them before the judges do)

- ppm is an **uncalibrated trend estimate**, never a safety measurement.
- A single MQ-2 channel cannot distinguish LPG from smoke or steam.
- Thresholds are room-tunable; the hardware relay path stays the real alarm.
- Wokwi has no ESP8266/DHT11/flame-injection — the sim uses ESP32, a DHT22
  stand-in, and a synthetic gas ramp; the real build uses the actual kit.
