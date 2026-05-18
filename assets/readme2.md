# OnTime Bus Tracker — ESP32 Edition

> Real-time GPS bus tracking over WiFi and MQTT, with Kalman-filtered coordinates for smooth, reliable location data.

---

## Table of Contents

- [What This Project Does](#what-this-project-does)
- [Hardware](#hardware)
- [System Architecture](#system-architecture)
- [How the Code Works](#how-the-code-works)
- [Kalman Filter](#kalman-filter)
- [Changes from G1 (SIM800L) to G2 (ESP32)](#changes-from-g1-sim800l-to-g2-esp32)
- [MQTT Topics and Payloads](#mqtt-topics-and-payloads)
- [Configuration](#configuration)
- [Wiring](#wiring)
- [Dependencies](#dependencies)
- [Error Handling and Auto-Reboot](#error-handling-and-auto-reboot)

---

## What This Project Does

A small device is installed inside a bus. Every 5 seconds it:

1. Reads the bus's GPS position from satellite signals
2. Smooths the coordinates using a **Kalman filter** to remove GPS jitter
3. Connects to a WiFi network and sends the location to an MQTT broker
4. Every 30 seconds, sends a heartbeat message so the server knows the device is alive

The backend server (G2) receives these messages and resolves which trip the bus is on — the device itself only reports position, nothing else.

---

## Hardware

| Component | Role |
|---|---|
| ESP32 Dev Board | Main processor, WiFi, logic |
| GPS Module (e.g. NEO-6M) | Reads satellite signals, outputs NMEA |
| HiveMQ Cloud | MQTT broker (receives messages) |

---

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    BUS (inside)                         │
│                                                         │
│  ┌──────────────┐     NMEA sentences     ┌───────────┐  │
│  │  GPS Module  │ ─────────────────────► │           │  │
│  │  (NEO-6M)    │   HardwareSerial(2)    │  ESP32    │  │
│  │  RX=16 TX=17 │                        │           │  │
│  └──────────────┘                        │  Kalman   │  │
│                                          │  Filter   │  │
│                                          │           │  │
└──────────────────────────────────────────┴─────┬─────┘  │
                                                 │         
                                          WiFi (TLS 8883)  
                                                 │         
                                    ┌────────────▼──────────────┐
                                    │     HiveMQ Cloud          │
                                    │     MQTT Broker           │
                                    │                           │
                                    │  transport/bus/23/location│
                                    │  transport/bus/23/heartbeat│
                                    └────────────┬──────────────┘
                                                 │
                                    ┌────────────▼──────────────┐
                                    │     G2 Backend Server     │
                                    │                           │
                                    │  Resolves active trip     │
                                    │  via Kafka trip.lifecycle │
                                    └───────────────────────────┘
```

---

## How the Code Works

### Startup (`setup()`)

```
Turn on Serial monitor (115200 baud)
Start GPS on HardwareSerial(2) pins RX=16, TX=17
Connect to WiFi → retry up to 5 times
Connect to MQTT broker (TLS port 8883)
```

### Main Loop (`loop()`) — runs every 5 seconds

```
Step 1 → Check WiFi.  If down: retry. If offline > 5 min: reboot.
Step 2 → Check MQTT.  If disconnected: reconnect.
Step 3 → Read GPS bytes. Parse NMEA. Run Kalman filter on lat/lon.
Step 4 → If 30 seconds have passed: publish heartbeat message.
Step 5 → If GPS has a fix AND time is valid: publish location message.
Step 6 → Wait 5 seconds. Repeat.
```

### Key Functions

| Function | What it does |
|---|---|
| `readGPS()` | Reads raw NMEA bytes, parses with TinyGPS++, runs Kalman filter, validates coordinates |
| `connectWiFi()` | Connects to WiFi with timeout, returns true/false |
| `ensureWiFi()` | Called every loop — reconnects if needed, up to 5 retries |
| `connectMQTT()` | Connects to HiveMQ with TLS, logs exact error reason if failed |
| `publishHeartbeatIfDue()` | Sends heartbeat every 30s with busId + timestamp |
| `kalmanUpdate()` | Runs one step of the Kalman filter on a single coordinate |
| `safeReboot()` | Logs a reason then calls `ESP.restart()` |
| `mqttStateToString()` | Converts MQTT error code to a human-readable string for debugging |

---

## Kalman Filter

### Why It Is Needed

A GPS module does not give a perfectly still reading even when the device is not moving. Satellite signals bounce off buildings and shift slightly every second. Without filtering, the bus position on the map jumps around by 5–15 metres even when stationary.

A Kalman filter solves this by blending two sources of information:

- **What we predict** — based on where the bus was last time
- **What the GPS says** — the new raw measurement

It weights them intelligently: if the GPS is very noisy, trust the prediction more. If the GPS is reliable, trust the new reading more.

### How It Works — Step by Step

Every time a new GPS coordinate arrives, the filter runs 4 steps:

```
Step 1 — Prediction
  p = p + q
  (Confidence in our estimate gets slightly worse over time
   because the bus has moved since the last reading)

Step 2 — Kalman Gain
  k = p / (p + r)
  (How much should we trust the new GPS reading?
   High noise r → small k → trust old estimate more
   Low noise r  → large k → trust new reading more)

Step 3 — Correction
  x = x + k × (new_gps_reading − x)
  (Blend old estimate with new reading, weighted by k)

Step 4 — Update Error
  p = (1 − k) × p
  (Confidence improves now that we have a new measurement)
```

### The Two Tuning Variables

```cpp
KalmanFilter kfLat = {0.0001, 0.01, 0.0, 1.0, 0.0, false};
//                     q       r
```

| Variable | Name | Effect |
|---|---|---|
| `q` | Process noise | Lower = smoother path, slower to follow sharp turns |
| `r` | Measurement noise | Higher = trust GPS less, smoother but more lag |

**Recommended values by environment:**

| Environment | q | r | Result |
|---|---|---|---|
| Open road, clear sky | `0.00001` | `0.005` | Very smooth |
| Urban streets, normal | `0.0001` | `0.01` | Balanced (default) |
| Dense city, noisy GPS | `0.001` | `0.05` | More responsive |

### Where the Filter Is Added in the Code

**1. After your `#include` block — add the struct and function:**

```cpp
// ==========================================
// KALMAN FILTER
// ==========================================
struct KalmanFilter {
  double q;           // process noise
  double r;           // measurement noise
  double x;           // current filtered value
  double p;           // estimation error
  double k;           // kalman gain
  bool   initialised;
};

KalmanFilter kfLat = {0.0001, 0.01, 0.0, 1.0, 0.0, false};
KalmanFilter kfLon = {0.0001, 0.01, 0.0, 1.0, 0.0, false};

double kalmanUpdate(KalmanFilter &kf, double measurement) {
  if (!kf.initialised) {
    kf.x = measurement;
    kf.initialised = true;
    return kf.x;
  }
  kf.p = kf.p + kf.q;
  kf.k = kf.p / (kf.p + kf.r);
  kf.x = kf.x + kf.k * (measurement - kf.x);
  kf.p = (1 - kf.k) * kf.p;
  return kf.x;
}
```

**2. Inside `readGPS()` — change 2 lines where raw lat/lon are stored:**

```cpp
// BEFORE:
lat = newLat;
lon = newLon;

// AFTER:
lat = kalmanUpdate(kfLat, newLat);
lon = kalmanUpdate(kfLon, newLon);
```

**3. Inside `readGPS()` — reset filter if GPS signal is lost:**

```cpp
if (!gps.location.isValid()) {
  kfLat.initialised = false;
  kfLon.initialised = false;
}
```

> The reset prevents the filter from slowly drifting back to an old position after a tunnel or signal gap.

### Why Two Separate Filters?

Latitude and longitude are completely independent axes. A bus moving east changes longitude only. A separate filter for each axis means neither one influences the other — they are treated as separate 1D problems, which is the correct way to apply a Kalman filter to 2D GPS coordinates.

---

## Changes from G1 (SIM800L) to G2 (ESP32)

### Overview

| Area | G1 — SIM800L | G2 — ESP32 |
|---|---|---|
| Connectivity | GSM cellular (SIM card) | WiFi (WPA2) |
| Encryption | None (port 1883) | TLS (port 8883) |
| Serial for GPS | SoftwareSerial (one at a time) | HardwareSerial(2) (simultaneous) |
| GPS filtering | None — raw coordinates | Kalman filter applied |
| Error handling | Infinite loop on failure | Retry + auto-reboot after 5 min |
| Heartbeat size | ~256 bytes (7 fields) | ~128 bytes (2 fields) |
| Timestamp precision | Seconds (`...00Z`) | Milliseconds (`...000Z`) |
| Coordinate validation | None | Range check + year ≥ 2024 check |
| Firmware version | `g1-0.1.0` | `g1-esp32-1.1.0` |

### Detail: Why SoftwareSerial Was a Problem

The old SIM800L code used `SoftwareSerial` for both the GPS module and the GSM module. Arduino's `SoftwareSerial` can only actively listen to one port at a time — you had to manually call `.listen()` to switch between them. This meant:

- GPS reading and GSM sending could not happen at the same time
- If the switch was missed, bytes were lost
- Complex sequencing in code just to manage two serial ports

The ESP32 has three hardware UART ports built in. GPS now runs on `HardwareSerial(2)` permanently — no switching needed, no byte loss, no sequencing.

### Detail: Why Heartbeat Was Shrunk

The old heartbeat sent 7 fields including `deviceId`, `gpsFix`, `satellites`, `signalQuality`, and `firmwareVersion`. These were useful during early development for debugging, but they add payload size on every single message forever.

The new heartbeat sends only `busId` and `timestamp`. If deeper diagnostics are needed, a separate debug topic can be added without bloating the production heartbeat that fires every 30 seconds from potentially hundreds of buses.

### Detail: Structured Error Recovery

Old behaviour on WiFi failure: loop forever printing dots.
New behaviour:

```
WiFi fails
  → retry up to 5 times, 20-second timeout each
  → if all 5 fail: log error, wait 5 seconds, check if offline > 5 minutes
  → if offline > 5 minutes: safeReboot("Offline too long (WiFi)")
```

Same pattern for MQTT. The 5-minute threshold (`REBOOT_AFTER_OFFLINE_MS = 300000`) means a temporary network outage recovers gracefully, but a permanent failure (dead SIM, wrong credentials) triggers a clean restart rather than hanging indefinitely.

---

## MQTT Topics and Payloads

### Location — published every 5 seconds, `retain = false`

**Topic:** `transport/bus/23/location`

```json
{
  "busId": "23",
  "lat": 6.92708,
  "lon": 79.86124,
  "speed": 42,
  "heading": 270,
  "timestamp": "2025-05-18T10:30:00.000Z"
}
```

`retain = false` because old positions must never replay. If the server restarts, it should wait for a fresh reading, not show the bus at a position from hours ago.

### Heartbeat — published every 30 seconds, `retain = true`

**Topic:** `transport/bus/23/heartbeat`

```json
{
  "busId": "23",
  "timestamp": "2025-05-18T10:30:00.000Z"
}
```

`retain = true` so the MQTT broker stores the last heartbeat. If the backend server restarts, it immediately knows this device was last seen at a specific time without waiting up to 30 seconds for the next one.

### G1 → G2 Contract

- The device sends `busId` and GPS data only. It does not send `tripId`.
- The G2 backend resolves which trip is active using `busId` combined with the Kafka `trip.lifecycle` stream.
- This keeps the device simple — all business logic lives on the server.

---

## Configuration

All settings are at the top of the `.ino` file:

```cpp
// WiFi
const char WIFI_SSID[]     = "your-network-name";
const char WIFI_PASSWORD[] = "your-password";

// MQTT (HiveMQ Cloud)
const char MQTT_BROKER[]   = "xxxx.s1.eu.hivemq.cloud";
const int  MQTT_PORT       = 8883;
const char MQTT_USERNAME[] = "your-username";
const char MQTT_PASSWORD[] = "your-password";

// Device
const char BUS_ID[]        = "23";   // change per bus

// Timing
const unsigned long LOCATION_INTERVAL_MS  = 5000;   // 5 seconds
const unsigned long HEARTBEAT_INTERVAL_MS = 30000;  // 30 seconds

// Kalman filter noise values
KalmanFilter kfLat = {0.0001, 0.01, 0.0, 1.0, 0.0, false};
KalmanFilter kfLon = {0.0001, 0.01, 0.0, 1.0, 0.0, false};
//                     q       r
```

---

## Wiring

| GPS Module Pin | ESP32 Pin |
|---|---|
| TX | GPIO 16 (RX2) |
| RX | GPIO 17 (TX2) |
| VCC | 3.3V or 5V |
| GND | GND |

The GPS baud rate is `9600`. Initialised with:

```cpp
gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
```

---

## Dependencies

Install via Arduino Library Manager:

| Library | Version | Purpose |
|---|---|---|
| TinyGPS++ | ≥ 1.0.3 | Parse NMEA sentences from GPS module |
| PubSubClient | ≥ 2.8 | MQTT client |
| WiFi | built-in (ESP32) | WiFi connection |
| WiFiClientSecure | built-in (ESP32) | TLS encrypted connection |

---

## Error Handling and Auto-Reboot

| Situation | Behaviour |
|---|---|
| WiFi not available | Retry 5 times × 20s timeout, then wait and monitor |
| MQTT connection fails | Retry 5 times, log exact reason via `mqttStateToString()` |
| Offline for 5+ minutes | `safeReboot()` — clean restart via `ESP.restart()` |
| No GPS bytes for 10s | Print warning with pin suggestion (check RX=16, TX=17) |
| GPS coordinates out of range | Skip publish, print warning |
| GPS year before 2024 | Reject timestamp — prevents epoch-zero garbage on cold boot |
| `snprintf` buffer would overflow | Skip publish, print error |
| GPS signal lost | Reset Kalman filter so first new reading is trusted fully |
