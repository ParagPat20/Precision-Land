# Hexacopter Specifications (Arduino UNO Q & BL-M8812EU2 5GHz Edition)

A comprehensive hardware, avionics, power distribution, and communications specification document detailing the custom payload-carrying hexacopter equipped with the **Arduino UNO Q "Dual-Brain" Companion Computer**, **BL-M8812EU2 High-Power 5GHz Long-Range Wi-Fi Link (867 Mbps / 800 mW)**, a **Dual USB Vision Subsystem**, and **Real-Time MCU Hardware Controls (Addressable LEDs, Buzzer, and Safety Actuation)**.

---

## 📦 Master Hardware & Avionics Module Directory

| # | Subsystem | Module / Component | Model / Specs | Key Role / Interfaces | Physical Dimensions (L×W×H mm) | Mounting Location |
|---|---|---|---|---|---|---|
| **1** | **Flight Avionics** | **CUAV V6X Autopilot** | STM32H753 (480 MHz), Triple IMU, Dual Baro | Primary Flight Controller (ArduPilot Copter v4.6.3) | 45 × 45 × 17 mm | Center Top Carbon Deck |
| **2** | **Flight Avionics** | **CUAV NEO V3 GPS** | u-blox M9N Multi-GNSS + IST8310 Compass | Centimeter GNSS Position & Heading (UART/I2C) | 56 × 56 × 16 mm | Rear Top Carbon Deck |
| **3** | **Sensors** | **Benewake TF-Luna LiDAR** | 850 nm ToF Rangefinder (0.2 m – 8.0 m) | Terrain Following & Precision Auto-Landing (UART) | 35 × 35 × 24 mm | Forward Underslung Nose |
| **4** | **RC Link** | **RadioMaster RP3 Receiver** | ExpressLRS 2.4 GHz True-Diversity | Low-Latency Manual Control (CRSF @ 416 kbaud) | 22 × 13 × 6 mm | Internal Bay (Right Flank) |
| **5** | **High-Power Link**| **BL-M8812EU2 5GHz Module**| Realtek RTL8812EU-CG, 867 Mbps, 2T2R, +29 dBm | Long-Range 5.8 GHz High-Speed Video & MAVLink Link | 32 × 32 × 3.5 mm | Top Deck Shielded Bay (with Heatsink) |
| **6** | **Companion SBC**| **Arduino UNO Q (4GB/32GB)**| Qualcomm QRB2210 (Quad A53 @ 2.0GHz) + STM32U585 | Dual-Brain Companion Computer (Linux + Real-Time MCU) | 68.6 × 53.4 × 15 mm | Rear Top Carbon Deck |
| **7** | **Acoustics** | **Piezo Buzzer / Beacon** | High-Decibel Piezo Transducer (90 dB @ 10 cm) | Arming Chimes, Lost Drone Alarm (Driven by UNO Q MCU D3) | 22 × 22 × 14 mm | Top Deck Flank |
| **8** | **Lighting** | **Arm & Strobe RGB LEDs** | WS2812B / SK6812 Addressable RGB LED Strips | 6x Arm Nav Lights + Docking Strobes (Driven by UNO Q MCU D6) | 6 × (60 × 10 × 3 mm) | Underside of Arms 1 to 6 |
| **9** | **Vision (USB 1)**| **Waveshare OV9281 Mono Cam**| 1 MP Global Shutter (1280×800 @ 120 FPS, UVC) | High-Speed Tracking, Aruco Docking & Optical Flow | 32 × 32 × 22 mm | Forward Nose Bay (Left) |
| **10**| **Vision (USB 2)**| **High-Res 4K UVC USB Cam**| Sony IMX Sensor (H.264/MJPEG hardware encoder) | High-Definition Aerial Inspection & FPV Streaming | 38 × 38 × 25 mm | Forward Nose Bay (Right) |
| **11**| **BVLOS Cellular**| **CAPUF EC200U 4G LTE** | Quectel EC200U-CN Cat 1 Modem + Secondary GNSS | Beyond Visual Line of Sight (BVLOS) 4G Backup Link | 40 × 30 × 10 mm | Top Deck Flank |
| **12**| **Power System** | **6S Solid-State Battery** | 22.2 V (25.2 V Max), 39 Ah (~865.8 Wh), 7C | Primary High-Density Propulsion Power Source | 210 × 88 × 67 mm | Underslung Belly Tray |
| **13**| **Power System** | **Holybro PM08-CAN PMU** | STM32F405, 2S–14S In, 200 A Cont. / 1000 A Peak | 5.2 V / 3 A FC Supply + DroneCAN Current/Volt Telemetry | 50 × 38 × 14 mm | Internal Center Chassis Bay |
| **14**| **Power System** | **6S Smart BMS Board** | 6-Cell Passive Balancing & Thermal Cutoff | Individual Cell Health, Overcharge/Discharge Protection | 120 × 55 × 12 mm | Battery Belly Enclosure |
| **15**| **Power System** | **5V 7A Dedicated UBEC** | Synchronous Step-Down DC-DC Regulator | Dedicated 5.0 V @ 7.0 A Rail for UNO Q, 5G Wi-Fi, Vision & LEDs| 48 × 26 × 12 mm | Internal Chassis Bay |
| **16**| **Power System** | **XL4005 Buck Converter** | DC-DC Step-Down (Tuned to 7.0 V @ 5 A Peak) | Dedicated 7.0 V Regulated Rail for Robotic Servos | 43 × 21 × 14 mm | Internal Chassis Bay |
| **17**| **Propulsion** | **BotLab 55A ESC #1** | AM32 32-bit AT32F421 4-in-1 ESC (Ch 1–3) | Drives Motors 1, 2, 3 via Digital DShot600 | 44 × 44 × 9 mm | Internal Bay (Right Side) |
| **18**| **Propulsion** | **BotLab 55A ESC #2** | AM32 32-bit AT32F421 4-in-1 ESC (Ch 4–6) | Drives Motors 4, 5, 6 via Digital DShot600 | 44 × 44 × 9 mm | Internal Bay (Left Side) |
| **19**| **Propulsion** | **6 × T-Motor MN4110 KV400**| 12N14P Brushless Motors + 16×5.5 Carbon Props | 6-Rotor Heavy-Lift Propulsion System | Φ47.4 × 36.7 mm (ea.) | Arm Tips 1 to 6 |
| **20**| **Actuation** | **Waveshare Servo Driver** | Multi-Channel Serial Bus Servo Interface Board | ST/SC Protocol Command & Power Router (UART/USB) | 45 × 35 × 12 mm | Underslung Actuator Base |
| **21**| **Actuation** | **ST3215 Servo (30 kg·cm)**| High-Torque Programmable Serial Bus Servo | Heavy Payload Drop & Winch Articulation | 40 × 20 × 40 mm | Underslung Payload Rig |
| **22**| **Actuation** | **2 × SC09 Servos (2.3 kg·cm)**| Dual Micro Serial Bus Servos (Jaws A & B) | Precision Robotic Gripper Jaw Actuation | 23 × 12 × 25 mm (ea.) | Robotic Claw Mechanism |

---

## 1. System Overview & Key Metrics

* **Platform Type:** Hexacopter (6-Rotor Multirotor)
* **Configuration:** Payload Frame X (Symmetrical 60° rotor distribution)
* **Dry Weight (No Battery):** ~5.44 kg *(includes UNO Q, BL-M8812EU2, dual USB cameras, and arm LED harnesses)*
* **All-Up Weight (AUW / Takeoff Weight):** ~10.44 kg
* **Battery Weight:** 2.80 kg (6S Solid-State 39 Ah)
* **Estimated Flight Time:** ~28 minutes (at 10.44 kg AUW, benefiting from ~15 W lower companion computer draw)
* **Primary High-Speed Wireless Link:** 5.8 GHz IEEE 802.11ac (867 Mbps PHY, +29 dBm / ~800 mW EIRP) via Realtek RTL8812EU
* **Primary Manual Control Link:** 2.4 GHz ExpressLRS True-Diversity (CRSF @ 416 kbaud)
* **Frequency Deconfliction:** Complete physical separation between **2.4 GHz RC control** and **5.8 GHz Telemetry/Video**, eliminating receiver desensitization.

---

## 2. Modular Subsystem Architecture & Signal Flow

```mermaid
flowchart TD
    subgraph S4["⚡ 1. Power Storage & Distribution (PDS)"]
        BAT["6S Solid-State Battery (22.2V, 39Ah)"]
        BMS["6S Smart BMS Board"]
        PMU["Holybro PM08-CAN PMU (200A)"]
        UBEC["5V 7A Dedicated UBEC"]
        XL["XL4005 Buck (7.0V @ 5A)"]
        
        BAT --> BMS
        BAT --> PMU
        BAT --> UBEC
        BAT --> XL
    end

    subgraph S2["🕹️ 2. Primary Flight Control & Navigation"]
        FC["CUAV V6X Autopilot (ArduPilot Copter)"]
        GPS["CUAV NEO V3 GPS + Compass"]
        RP3["RadioMaster RP3 Diversity RC (2.4GHz)"]
        
        GPS -->|"UART + I2C"| FC
        RP3 -->|"CRSF @ 416k"| FC
    end

    subgraph S1["🧠 3. Companion Compute, Vision, Comms & Real-Time MCU Controls"]
        subgraph UNOQ_DUAL["Arduino UNO Q 'Dual-Brain' Subsystem"]
            MPU["Qualcomm QRB2210 MPU<br>• Debian 12 Linux & ROS 2<br>• Adreno 702 GPU & NPU"]
            BRIDGE["Arduino Bridge RPC Bus"]
            MCU["STM32U585 Real-Time MCU<br>• Arm Cortex-M33 @ 160MHz<br>• Cycle-Accurate I/O"]
            MPU <--> BRIDGE <--> MCU
        end

        WIFI["BL-M8812EU2 High-Power 5GHz Link<br>RTL8812EU (867 Mbps / 800mW / 2T2R)"]
        USB_CAM1["Waveshare OV9281 Mono Cam (120 FPS USB)"]
        USB_CAM2["4K/HD UVC Inspection Cam (USB)"]
        LTE["CAPUF EC200U 4G LTE Modem (USB)"]

        LEDS["WS2812B RGB LEDs (6 Arms + Dock Strobes)"]
        BUZZ["Piezo Acoustic Buzzer / Beacon"]
        SAFETY["Aux Controls (Parachute Eject / Drop / Touchdown)"]

        MPU <-->|"USB 2.0 (High Speed)"| WIFI
        MPU <-->|"USB 3.0/2.0 UVC"| USB_CAM1
        MPU <-->|"USB 2.0 UVC (H.264)"| USB_CAM2
        MPU <-->|"USB 2.0 CDC-ECM"| LTE

        MCU -->|"Pin D6 (Timer/DMA Data)"| LEDS
        MCU -->|"Pin D3 (Hardware PWM Tone)"| BUZZ
        MCU -->|"Pin D4 (Ejection Trigger)"| SAFETY
        MCU -->|"Pin D8 (Payload Solenoid)"| SAFETY
        MCU <--|"Pin D2 (Touchdown Interrupt)"| SAFETY
    end

    subgraph S3["🚀 4. Propulsion & Speed Control"]
        ESC1["BotLab 55A ESC #1 (Motors 1-3)"]
        ESC2["BotLab 55A ESC #2 (Motors 4-6)"]
        MOT["6x T-Motor MN4110 + 16x5.5 Props"]
        
        ESC1 --> MOT
        ESC2 --> MOT
    end

    subgraph S5["🦾 5. Robotic Actuation & Payload Mechanism"]
        SDRV["Waveshare Servo Driver Board"]
        ST["ST3215 Servo (30kg-cm Release)"]
        SC["2x SC09 Servos (Claw Jaws A/B)"]
        
        SDRV --> ST
        SDRV --> SC
    end

    %% Cross-Subsystem Interconnects
    PMU ==>|"5.2V FC Power + DroneCAN"| FC
    PMU ==>|"22.2V DC Main Bus"| ESC1
    PMU ==>|"22.2V DC Main Bus"| ESC2
    UBEC ==>|"5.0V @ 7.0A Logic Rail"| UNOQ_DUAL
    UBEC ==>|"5.0V Clean Rail"| WIFI
    UBEC ==>|"5.0V LED Power Rail"| LEDS
    XL ==>|"7.0V @ 5.0A Servo Rail"| SDRV
    FC <-->|"MAVLink2 (921.6k baud UART)"| MPU
    FC -->|"DShot600 (M1-M3)"| ESC1
    FC -->|"DShot600 (M4-M6)"| ESC2
    MPU -->|"USB / UART Control"| SDRV
```

---

## 3. Detailed Component & Subsystem Specifications

### 3.1 Companion Computer: Arduino UNO Q

The **Arduino UNO Q** replaces the Raspberry Pi 5 as the central onboard intelligence unit. It adopts a **dual-brain architecture** that pairs high-level Linux edge processing with low-level deterministic real-time microcontroller execution:

* **Microprocessor Unit (MPU - High-Level Compute):**
  * **Processor:** Qualcomm® Dragonwing™ QRB2210
  * **CPU Cores:** Quad-Core Arm® Cortex®-A53 running up to 2.0 GHz (64-bit)
  * **GPU:** Qualcomm® Adreno™ 702 (OpenGL ES 3.1, Vulkan 1.1)
  * **AI / NPU Engine:** Dedicated on-chip Neural Processing Unit capable of real-time computer vision inference (YOLOv8-nano / MobileNet for target identification and precision landing alignment)
  * **Memory:** 4 GB LPDDR4 high-speed RAM
  * **Storage:** 32 GB eMMC 5.1 onboard flash storage + MicroSD expansion
  * **Operating System:** Debian 12 Linux (Debian Bookworm ARM64) with ROS 2 (Humble/Iron) and `mavlink-router`
* **Microcontroller Unit (MCU - Real-Time Control):**
  * **Processor:** STMicroelectronics STM32U585 (Arm® Cortex®-M33 with TrustZone @ 160 MHz)
  * **Role:** Sub-millisecond deterministic control for navigation lighting, precision buzzer tones, safety interlocks, and sensor bus bridging without Linux OS preemption or jitter.
  * **Inter-Brain Communication:** Built-in high-speed **Arduino Bridge RPC** shared-memory interface allowing Python/C++ Linux services to exchange telemetry and commands directly with Arduino sketches.
* **Thermal & Power Efficiency Advantages:**
  * **Power Consumption:** ~1.5 W idle, ~3.5 W to 5 W max under full load (compared to 12 W to 25 W on Raspberry Pi 5).
  * **Thermal Performance:** Passive cooling or low-profile heatsink under prop wash; zero risk of sudden thermal throttling mid-flight.
  * **Input Power:** 5.0 V DC via USB-C or 5V pin header directly from the 5V 7A UBEC rail.

---

### 3.2 Arduino UNO Q Real-Time MCU (STM32U585) Pinout & Hardware Controls

The real-time MCU on the UNO Q exposes standard Arduino UNO header pins, dedicated entirely to deterministic tasks that require microsecond timing, zero jitter, and fail-safe operation:

#### 1. Hardware Pin Allocation Table

| Pin | Physical Name | Function / Peripheral | Direction | Signal Type & Electrical Level | Connected Device / Role | Failsafe State |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **D0** | RX | Hardware UART | Input | 3.3V TTL (5V Tolerant) | Secondary Serial / Auxiliary Debug | High-Z |
| **D1** | TX | Hardware UART | Output | 3.3V TTL | Secondary Serial / Auxiliary Debug | High-Z |
| **D2** | EXTI2 | Touchdown / Docking Switch | Input | Active LOW (Internal Pull-Up) | Microsecond Mechanical Docking Contact Sensor | Pulled HIGH |
| **D3** | TIM3_CH2 | **Acoustic Buzzer / Beacon** | Output | **Hardware PWM (0–100%, 1–5 kHz)** | **High-Decibel Piezo Alarm & Arming Chimes** | **LOW (Silent)** |
| **D4** | GPIO | Parachute Emergency Deploy | Output | Active HIGH Digital Trigger | Pyrotechnic / Spring Ejection Solenoid | LOW (Locked) |
| **D5** | TIM3_CH1 | High-Power Searchlight FET | Output | Hardware PWM (0–100% Dimming) | Underbelly High-Intensity Night Spotter LED | LOW (Off) |
| **D6** | TIM16_CH1| **Arm & Docking RGB LEDs** | Output | **Fast Cycle-Accurate 800 kHz Data**| **WS2812B / SK6812 Digital Addressable Strips**| **LOW** |
| **D7** | GPIO | Autopilot Watchdog Ping | In/Out | 10 Hz Heartbeat Square Wave | Watchdog line to CUAV V6X | Floating |
| **D8** | GPIO | Payload Drop Solenoid | Output | Active HIGH Pulse (100 ms) | Solenoid / Quick-Release Ball Latch | LOW (Held) |
| **D9** | TIM1_CH1 | Auxiliary Standard Servo | Output | 50 Hz / 333 Hz PWM (1000–2000 µs) | Auxiliary Camera Tilt or Mechanical Latch | Neutral (1500µs)|
| **D10**| SPI_CS | SPI Chip Select | Output | Active LOW | High-Speed Sensor / Flash Expansion | HIGH |
| **D11**| SPI_MOSI | SPI Master Out | Output | 3.3V SPI Bus | High-Speed Expansion Bus | LOW |
| **D12**| SPI_MISO | SPI Master In | Input | 3.3V SPI Bus | High-Speed Expansion Bus | High-Z |
| **D13**| SPI_SCK | SPI Clock | Output | 3.3V SPI Clock | High-Speed Expansion Bus | LOW |
| **A0** | ADC1_IN1 | Analog Bus Voltage Tap | Input | Analog (0–3.3V via 11:1 Divider) | Secondary Redundant Battery Voltage Sense | High-Z |
| **A1** | ADC1_IN2 | Chassis Thermistor (NTC) | Input | Analog (0–3.3V) | Internal Avionics Bay Thermal Health | High-Z |
| **A2** | ADC1_IN3 | Optical Dock Photodiode | Input | Analog (0–3.3V) | IR Docking Beacon Proximity Verification | High-Z |
| **A3** | GPIO | Spare Digital / Analog | In/Out | Configurable | General Auxiliary I/O | Floating |
| **A4** | I2C_SDA | I2C Data Bus | Bidirectional| Open-Drain (with 4.7kΩ Pull-Up) | Optional OLED Flight Status Display | Pulled HIGH |
| **A5** | I2C_SCL | I2C Clock Bus | Output | Open-Drain (with 4.7kΩ Pull-Up) | Optional OLED Flight Status Display | Pulled HIGH |

---

#### 2. Addressable RGB LED Subsystem (Pin D6)
* **Hardware Interface:** Driven by Pin **D6** via direct DMA / Hardware Timer generating precise 800 kHz timing pulses without CPU thread preemption.
* **Topology:** 6 arm strips (8 WS2812B LEDs per arm = 48 LEDs) + 4 center high-output docking alignment strobes (SK6812 RGBW).
* **Power Supply:** 5.0 V DC supplied directly from the **5V 7A UBEC rail** with common ground to UNO Q (logic level shifted to 5V if required).
* **Lighting Modes & Color Schemes:**
  * **Aviation Navigation Mode (Cruising):**
    * Arm 1 & Arm 2 (Front-Left, Left): Solid **Aviation Red** (Port side)
    * Arm 5 & Arm 6 (Front-Right, Right): Solid **Aviation Green** (Starboard side)
    * Arm 3 & Arm 4 (Aft-Left, Aft-Right): Dual **Solid Amber / Flashing White** (Aft position markers)
  * **Docking Alignment Mode:** When the QRB2210 MPU detects the ground dock via the OV9281 camera, it signals the MCU to switch all arms to **Synchronized 4 Hz Cyan / High-Contrast White Strobes**, providing optical alignment markers for the dock's stationary overhead tracking cameras.
  * **Pre-Arming / System Status Mode:**
    * Green Breathing: GPS 3D Fix Locked, EKF Ready to Arm.
    * Blue Fast Chase: Calibrating / Telemetry Active.
    * Yellow Blink: Low Battery Warning (<21.0 V on 6S pack).
    * Rapid Red Double-Flash: Critical Failsafe / Geofence Breach.

---

#### 3. Acoustic Buzzer & Melodic Beacon Subsystem (Pin D3)
* **Hardware Interface:** Driven by Pin **D3** using hardware timer PWM (TIM3_CH2).
* **Component:** High-output electromagnetic or piezo sound transducer capable of producing **90 dB @ 10 cm**.
* **Acoustic Sound Profiles:**
  * **Boot / Initialization:** Ascending 3-tone arpeggio (C5 -> E5 -> G5).
  * **Arming Warning:** Continuous 2.4 kHz tone for 1.5 seconds prior to motor spin-up.
  * **Low Battery Alarm:** Dual-frequency alternating siren (2.0 kHz / 3.2 kHz @ 2 Hz rate).
  * **Lost Drone Acoustic Locator:** Ultra-loud 3.5 kHz intermittent chirp (100 ms pulse every 1.5 seconds) triggered via RC transmitter switch or automatically if the link is severed for >60 seconds.

---

#### 4. Auxiliary Safety Controls & Interlocks (Pins D2, D4, D8)
* **Docking Touchdown Interrupt (Pin D2):**
  * Hardware microswitch or capacitive contact on the landing feet wired to Pin D2.
  * Triggers an immediate hardware interrupt (`EXTI2`) upon touch-down, instantaneously informing the MPU to cut throttle and hold motors before mechanical bounce occurs.
* **Emergency Parachute Release (Pin D4):**
  * Hardware output driving a high-current MOSFET for ballistic parachute deployment.
  * Can be triggered directly by the STM32U585 MCU if a free-fall condition or tumble is detected by onboard IMU, even if the Linux OS has encountered a kernel freeze.
* **Payload Drop Solenoid (Pin D8):**
  * Provides a deterministic 100 ms pulse to release cargo or auxiliary drop pods on exact waypoint triggers.

---

#### 5. Dual-Brain Control Integration (Arduino Bridge RPC Example)
The Linux MPU (Python/ROS 2) communicates with the STM32U585 MCU sketches using the lightweight **Arduino Bridge RPC** library:

```python
# companion_autonomy.py running on Qualcomm QRB2210 (Debian 12 Linux)
import rclpy
from arduino_bridge import ArduinoClient

# Initialize Bridge connection to on-chip STM32U585 MCU
mcu = ArduinoClient()

def on_docking_phase_entered():
    # Instantly switch arm LEDs to docking strobe mode via MCU
    mcu.call("set_led_mode", mode="DOCKING_STROBE", brightness=255)
    # Emit short audible notification tone
    mcu.call("play_tone", frequency_hz=2800, duration_ms=200)

def on_touchdown_detected():
    # Read instantaneous touchdown state from MCU interrupt buffer
    if mcu.call("get_touchdown_state"):
        publish_mavlink_disarm()

def emergency_failsafe_trigger():
    # Command MCU to deploy parachute and sound loud locator siren
    mcu.call("eject_parachute")
    mcu.call("set_buzzer_alarm", active=True)
```

---

### 3.3 High-Power Long-Range 5GHz Wi-Fi Module: BL-M8812EU2

The **BL-M8812EU2** replaces the legacy 2.4 GHz ESP32 telemetry subsystem, serving as a unified **high-throughput, long-range wireless data link** for both live HD video backhaul and bidirectional MAVLink ground telemetry:

* **RF Chipset:** Realtek **RTL8812EU-CG**
* **Wireless Standards:** IEEE 802.11a/n/ac (Wi-Fi 5)
* **Frequency Range:** 5.150 GHz – 5.850 GHz (Single-band 5 GHz design, preventing any 2.4 GHz RF contamination)
* **MIMO & Antenna Configuration:** **2T2R** (2 Transmit, 2 Receive) with dual onboard IPEX / MHF-1 connectors routed to dual 5.8 GHz circular-polarized or high-gain omnidirectional RP-SMA cloverleaf antennas.
* **PHY Data Rate:** Up to **867 Mbps** (80 MHz channel width on 802.11ac)
* **RF Transmit Power:** Integrated high-power Front-End Module (FEM) delivering up to **+29 dBm (~800 mW)** output power.
* **Long-Range / FPV Optimization:**
  * Supports standard 20/40/80 MHz channels as well as **narrowband 5 MHz and 10 MHz channel widths** for extreme range penetration and link margin.
  * Fully supports **Monitor Mode** and **Raw Packet Injection** using patched Linux kernel drivers (`rtl88x2eu`), making it natively compatible with **OpenHD**, **WFB-ng (Wi-Fi Broadcast Next Generation)**, and custom RTP/UDP zero-handshake video streaming.
* **Host Interface:** USB 2.0 (4-Pin header / micro-connector: `5V`, `D-`, `D+`, `GND`) connected to the Arduino UNO Q USB host port.
* **Electrical & Thermal Requirements:**
  * **Supply Voltage:** 5.0 V DC ± 0.25 V
  * **Peak Current:** Up to **1,800 mA (1.8 A)** during continuous high-power +29 dBm packet bursts (~9 W RF power draw).
  * **Power Routing:** Wired directly to the dedicated 5V 7A UBEC rail (never back-powered through passive unpowered USB ports).
  * **Thermal Notice:** Must be fitted with a dedicated aluminum heatsink and positioned in direct airflow from the rotor wash. **Antennas must always be securely attached before powering the module to prevent output PA damage.**

---

### 3.4 Dual USB Vision & Camera Architecture

Since the Arduino UNO Q companion computer standardizes on USB interfaces for vision (avoiding fragile, proprietary MIPI CSI flat flex cables that suffer from motor EMI and vibration fatigue), the hexacopter integrates **two specialized USB cameras**:

#### 1. High-Speed Optical Tracking & Precision Landing Camera (USB 1)
* **Model:** Waveshare OV9281 1MP Monochrome USB Camera
* **Sensor:** OmniVision OV9281 (1/4" Global Shutter CMOS)
* **Pixel Resolution:** 1280 × 800 (1 MP)
* **Shutter Type:** **True Global Shutter** (Exposes all pixels simultaneously, completely eliminating the "jello" rolling shutter distortion caused by hexacopter rotor vibrations).
* **Frame Rate:** Up to **120 FPS** @ 1280×800; up to **210 FPS** @ 640×400
* **Interface:** USB 2.0 / USB 3.0 UVC (Universal Video Class - driverless in Linux V4L2)
* **Primary Roles:**
  * Real-time AprilTag / Aruco marker detection for automated docking station centering.
  * Downward-facing visual odometry / optical flow when GPS is degraded.
  * High-speed visual servoing and precision target tracking.

#### 2. High-Definition Aerial Inspection & FPV Camera (USB 2)
* **Model:** 4K / 8MP High-Resolution UVC USB Camera Module (Sony IMX-based)
* **Sensor:** Sony IMX415 / IMX317 1/2.8" Back-Illuminated CMOS Sensor
* **Resolution:** 3840 × 2160 (4K @ 30 FPS) or 1920 × 1080 (1080p @ 60 FPS)
* **Compression Hardware:** Onboard hardware H.264 / H.265 / MJPEG video encoder engine inside the camera DSP (delivers pre-compressed elementary video frames over USB, relieving the UNO Q CPU of video encoding overhead).
* **Lens:** Low-distortion motorized autofocus or fixed-focus 85° FOV lens with IR-cut filter.
* **Interface:** Standard USB 2.0 / 3.0 UVC
* **Primary Roles:**
  * High-detail industrial infrastructure inspection (power lines, solar panels, pipeline monitoring).
  * Real-time HD FPV stream transmitted over the BL-M8812EU2 5.8 GHz wireless link to Mission Planner or QGroundControl.

---

### 3.5 RF Frequency Deconfliction & Spectrum Plan

One of the most critical design upgrades in the `HEXA_UNO_Q` configuration is the total elimination of in-band RF interference:

```
┌────────────────────────────────────────────────────────────────────────┐
│                        RF SPECTRUM ISOLATION                           │
├───────────────────┬───────────────────┬────────────────────────────────┤
│ Band              │ Link / Subsystem  │ Hardware & Role                │
├───────────────────┼───────────────────┼────────────────────────────────┤
│ 850 nm (Optical)  │ LiDAR Altitude    │ Benewake TF-Luna Rangefinder   │
│ 1.1 - 1.6 GHz     │ Satellite GNSS    │ CUAV NEO V3 (GPS/Galileo/BDS)  │
│ 1.8 - 2.1 GHz     │ 4G LTE BVLOS      │ CAPUF EC200U Modem (Cellular)  │
│ 2.400 - 2.483 GHz │ Manual RC Pilot   │ RadioMaster RP3 (ExpressLRS)   │
│ 5.150 - 5.850 GHz │ Video + Telemetry │ BL-M8812EU2 (800mW 802.11ac)   │
└───────────────────┴───────────────────┴────────────────────────────────┘
```

> [!NOTE]
> **Zero 2.4 GHz Interference:** By migrating all Wi-Fi telemetry and video streaming to the **5.8 GHz band** (BL-M8812EU2), the 2.4 GHz band is 100% reserved for the **RadioMaster RP3 ExpressLRS** control link. This eliminates the packet loss, telemetry warnings, and range degradation caused when an onboard ESP32 transmits Wi-Fi next to an ELRS receiver.

---

## 4. Flight Controller & Autopilot Integration

* **Flight Controller:** CUAV V6X Autopilot (STM32H753 @ 480 MHz, Triple IMU, Dual Baro)
* **Autopilot Firmware:** ArduPilot Copter v4.6.3 (Pixhawk 6X target)
* **Serial Port Allocation:**

| Port | Hardware UART | Connected Peripheral | Protocol & Baud Rate | ArduPilot Parameter Settings |
| :--- | :--- | :--- | :--- | :--- |
| **SERIAL0** | USB | Maintenance / Setup | MAVLink2 (115200) | Default ground config port |
| **SERIAL1** | UART7 (TELEM1)| Auxiliary / Payload | Optional / Reserved | `SERIAL1_PROTOCOL = -1` (Spare) |
| **SERIAL2** | UART5 (TELEM2)| Benewake TF-Luna LiDAR| Rangefinder (115200) | `SERIAL2_PROTOCOL = 9`, `RNGFND1_TYPE = 20` |
| **SERIAL3** | USART1 (GPS1) | CUAV NEO V3 GPS | u-blox GNSS + Compass | `SERIAL3_PROTOCOL = 5`, `GPS_TYPE = 1` |
| **SERIAL4** | UART8 (GPS2)  | RadioMaster RP3 Rx | CRSF / ExpressLRS | `SERIAL4_PROTOCOL = 23`, `SERIAL4_BAUD = 115` |
| **SERIAL5** | USART2 (TELEM3)| **Arduino UNO Q SBC** | MAVLink2 (921600 baud) | `SERIAL5_PROTOCOL = 2`, `SERIAL5_BAUD = 921` |
| **SERIAL6** | UART4 (User)  | Auxiliary Debug | N/A | Spare serial interface |

---

## 5. Power Distribution & Regulation Scheme

The hexacopter utilizes a high-efficiency power architecture fed by the **6S Solid-State 39 Ah battery (22.2 V nominal, 25.2 V max)**:

```
[6S Solid-State Battery 22.2V 39Ah]
  │
  ├─► [Holybro PM08-CAN PMU] ──(DroneCAN)──► [CUAV V6X FC (5.2V @ 3A)]
  │     │
  │     ├─► [Main DC Bus 22.2V] ──────────► [BotLab 55A ESC #1 (M1-M3)]
  │     └─► [Main DC Bus 22.2V] ──────────► [BotLab 55A ESC #2 (M4-M6)]
  │
  ├─► [5V / 7A Dedicated UBEC] 
  │     │
  │     ├─► [Arduino UNO Q Companion SBC (5.0V @ 2.0A max)]
  │     ├─► [BL-M8812EU2 5GHz Module (5.0V @ 1.8A peak)]
  │     ├─► [Waveshare OV9281 USB Camera (5.0V @ 0.3A)]
  │     ├─► [4K UVC Inspection Camera (5.0V @ 0.5A)]
  │     ├─► [WS2812B RGB Arm LEDs & Strobes (5.0V @ 0.6A avg / 1.0A peak)]
  │     └─► [CAPUF EC200U 4G Modem (5.0V @ 0.8A peak)]
  │         (Total 5V Peak Consumption: ~6.0A < 7.0A Continuous Rating)
  │
  └─► [XL4005 DC-DC Buck Converter (7.0V @ 5A)]
        └─► [Waveshare Servo Driver Board] ──► [ST3215 & 2x SC09 Servos]
```

### Power Safety & Margins
* **5V 7A UBEC Rail:** The total worst-case concurrent draw for the UNO Q, high-power BL-M8812EU2 (transmitting at full 800 mW RF), both USB cameras, addressable LEDs (all at white peak brightness), and the 4G modem is **~6.0 A**, safely operating within the **7.0 A continuous (10 A burst)** capacity of the synchronous UBEC.
* **Propulsion Current:** Hover current sits at ~15 A per motor (~90 A total at 22.2 V). The PM08-CAN PMU easily handles up to 200 A continuous and 1000 A peak bursts.

---

## 6. Software & Telemetry Pipeline

### 6.1 MAVLink Routing on Arduino UNO Q
The Arduino UNO Q runs `mavlink-routerd` on Linux Debian 12 to aggregate and route telemetry packets:
1. **Endpoint 1 (Flight Controller):** `/dev/ttyS1` (or QRB2210 UART connected to V6X TELEM3) @ `921600` baud.
2. **Endpoint 2 (5.8 GHz High-Speed Link):** UDP broadcast / unicast over `wlan0` (BL-M8812EU2) to Ground Station on port `14550`.
3. **Endpoint 3 (4G LTE BVLOS):** VPN tunnel (ZeroTier / WireGuard) over `ppp0` / `usb0` (CAPUF EC200U) to Cloud GCS on port `14551`.
4. **Endpoint 4 (Internal ROS 2):** Local UDP loopback port `14552` for ROS 2 `micro-ROS` / `mavros` companion autonomy nodes.

### 6.2 Video Streaming Pipeline
Using GStreamer hardware/software pipelines with UVC video sources:
```bash
# Stream 1: Low-Latency FPV Stream from 4K/1080p UVC Camera over BL-M8812EU2 5.8GHz Link
gst-launch-1.0 -v v4l2src device=/dev/video0 ! \
  image/jpeg,width=1920,height=1080,framerate=60/1 ! \
  jpegdec ! videoconvert ! \
  x264enc tune=zerolatency bitrate=4000 speed-preset=ultrafast ! \
  rtph264pay config-interval=1 pt=96 ! \
  udpsink host=192.168.1.100 port=5600 sync=false

# Stream 2: High-Speed OV9281 Global Shutter Stream for Onboard Vision / Edge AI
gst-launch-1.0 -v v4l2src device=/dev/video1 ! \
  video/x-raw,format=GRAY8,width=1280,height=800,framerate=120/1 ! \
  appsink name=vision_sink
```

---

## 7. Comparative Upgrade Summary (RPi 5 vs. Arduino UNO Q)

| Metric / Subsystem | Legacy Architecture (RPi 5 + ESP32) | Upgraded Architecture (`HEXA_UNO_Q.md`) | Practical Benefit |
|---|---|---|---|
| **Companion SBC** | Raspberry Pi 5 (8GB) | **Arduino UNO Q (4GB / 32GB)** | Dual-brain MPU + real-time MCU, integrated eMMC |
| **SBC Power Draw**| 12 W – 25 W (hot, requires active fan)| **2 W – 5 W (cool, efficient)** | **Saves ~15W power; adds ~1.5 min flight time** |
| **Real-Time I/O** | Soft Linux GPIO (jitter, no hard PWM)| **STM32U585 Real-Time MCU Pins** | **Cycle-accurate LED timing, hardware buzzer tones, zero jitter** |
| **Wi-Fi Telemetry**| ESP32 2.4 GHz (54 Mbps, ~20 dBm) | **BL-M8812EU2 5 GHz (867 Mbps, +29 dBm / 800 mW)** | **40× throughput, 4× RF power, true HD video** |
| **RC Deconfliction**| Shared 2.4 GHz band with ELRS (risk)| **Isolated: 2.4 GHz RC vs 5.8 GHz Video/Data** | **Zero RC packet drops, maximum pilot range** |
| **Camera Interface**| Fragile 22-pin MIPI CSI ribbon | **Dual Standard USB UVC (Plug & Play)** | **High noise immunity, robust cabling, easy swaps** |
| **Optical Flow Cam**| OV9281 (USB) | **OV9281 120 FPS Global Shutter (USB)** | Distortion-free precision landing & docking |
| **Inspection Cam** | Arducam 64MP (CSI-only) | **High-Res UVC Camera with hardware H.264 (USB)** | Native Linux support without proprietary drivers |

---

## 8. Summary Checklist & Next Steps
- [x] Integrate Arduino UNO Q dual-brain compute architecture into avionics documentation.
- [x] Map STM32U585 real-time MCU header pins for addressable WS2812B LEDs, acoustic buzzer, and safety controls.
- [x] Configure BL-M8812EU2 high-power 5.8 GHz wireless video and telemetry pipeline.
- [x] Transition camera subsystem entirely to robust USB UVC architecture (OV9281 + High-Res Inspection Cam).
- [x] Verify complete 2.4 GHz / 5.8 GHz RF frequency deconfliction.
- [x] Recalculate 5V 7A UBEC current budget incorporating addressable LEDs and high-power Wi-Fi module.
