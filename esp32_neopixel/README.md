# ESP32-S3 Dual-Strip NeoPixel Flight Mode Controller (MAVLink)

Firmware for **ESP32-S3** to drive dual addressable WS2812B/NeoPixel LED strips synchronized with **ArduPilot flight modes** via MAVLink from **CUAV V6X**.

---

## Lighting Behavior by Flight Mode

| Strip | Color | GPIO Pin | LED Count | Mode Behavior |
|---|---|---|---|---|
| **Strip 1** | **TEAL** | **GPIO 5** | **11 LEDs** | 🟢 **AUTO Mode:** Solid Vibrant Teal<br>🚀 **LAND Mode:** **Dynamic Chasing Teal Light** (Running pulse with comet fade)<br>💤 **Disarmed:** Gentle Breathing Teal pulse<br>✈️ **Other Modes (Loiter/RTL):** Solid Teal |
| **Strip 2** | **WHITE** | **GPIO 6** | **10 LEDs** | ⚪ **Always Solid Bright White** (Illumination / Navigation) |

---

## Wiring Diagram: ESP32-S3 + CUAV V6X + LEDs

```
+------------------+                    +-------------------------+
|     CUAV V6X     |                    |        ESP32-S3         |
|   (TELEM Port)   |                    |                         |
|                  |                    |                         |
|   TX (Pin 2) ----+------------------->| U0_RX / GPIO 44         |
|   RX (Pin 3) <---+--------------------| U0_TX / GPIO 43         |
|   GND (Pin 6) ---+---------+--------->| GND                     |
+------------------+         |          |                         |
                             |          | GPIO 5 ----> [ DIN ]    | Strip 1 (11x TEAL)
                             |          | GPIO 6 ----> [ DIN ]    | Strip 2 (10x WHITE)
                             |          +-------------------------+
                             |
   +5V BEC (3A - 5A) --------+--------------> [ +5V ] (All LED Strips)
   GND BEC ------------------+--------------> [ GND ] (Common Ground)
```

> [!IMPORTANT]
> 1. **Common Ground:** Connect **GND of CUAV V6X, ESP32-S3, 5V BEC, and both LED strips together**.
> 2. **Power Supply:** LEDs must be powered from the **5V BEC**, **NEVER** from the 3.3V pin of ESP32 or the 5V rail of CUAV V6X.
> 3. **UART Pin Default:** On standard ESP32-S3 boards, `U0_RX` is `GPIO 44` and `U0_TX` is `GPIO 43`.

---

## ArduPilot Telemetry Port Parameters (Mission Planner)

Under Mission Planner **Config / Tuning &rarr; Full Parameter List**, verify the serial port connected to ESP32:

* If connected to **TELEM1**:
  * `SERIAL1_PROTOCOL` = **2** (MAVLink 2) or **1** (MAVLink 1)
  * `SERIAL1_BAUD` = **57** (57600 baud, matches `#define MAVLINK_BAUD 57600`)
* If connected to **TELEM2**:
  * `SERIAL2_PROTOCOL` = **2**
  * `SERIAL2_BAUD` = **57**

---

## How It Works (Zero Dependencies)

The firmware includes an internal, lightweight MAVLink state machine parser directly in the sketch:
- Listens to incoming **`HEARTBEAT`** packets from CUAV V6X (v1 or v2).
- Extracts `custom_mode` (Mode 3 = `AUTO`, Mode 9 = `LAND`).
- Extracts `base_mode` (detects if drone is Armed or Disarmed).
- **No external MAVLink library download is needed in Arduino IDE.** Only `Adafruit_NeoPixel` is required.

---

## Manual Testing (USB Serial Monitor)

You can test the animations on your desk by opening the Arduino Serial Monitor at **115200 baud** and typing:
- `LAND` &rarr; Simulates Land mode (Chasing Teal light runs immediately).
- `AUTO` &rarr; Simulates Auto mode (Solid Teal).
- `LOITER` &rarr; Simulates Loiter / Normal flight (Solid Teal).
- `DISARM` &rarr; Simulates Disarmed mode (Breathing Teal).
