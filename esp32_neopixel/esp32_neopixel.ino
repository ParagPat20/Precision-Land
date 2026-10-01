/*
 * ==============================================================================
 * Project: ESP32-S3 Dual-Strip NeoPixel LED Controller with MAVLink
 * Hardware: ESP32-S3
 *
 * Pin Connections:
 *   - GPIO 5 : Strip 1 (11 NeoPixel LEDs) -> TEAL
 *              * AUTO Mode : Solid Bright Teal
 *              * LAND Mode : Dynamic Chasing Teal Pulse
 *              * Other Modes: Solid Ambient Teal (Breathes when Disarmed)
 *   - GPIO 6 : Strip 2 (10 NeoPixel LEDs) -> WHITE (Always Solid Illumination)
 *
 * MAVLink Telemetry Connection to CUAV V6X:
 *   - ESP32-S3 U0_RX (GPIO 44) <--- CUAV V6X TELEM TX
 *   - ESP32-S3 U0_TX (GPIO 43) ---> CUAV V6X TELEM RX (Optional)
 *   - GND                      <--- Common Ground with CUAV V6X
 *
 * Required Arduino Library:
 *   - Adafruit_NeoPixel (Install via Arduino Library Manager)
 * ==============================================================================
 */

#include <Adafruit_NeoPixel.h>

// ------------------------------------------------------------------------------
// HARDWARE PIN DEFINITIONS & LED COUNTS
// ------------------------------------------------------------------------------
#define PIN_TEAL          5       // GPIO 5 for Teal Strip
#define NUM_LEDS_TEAL     11      // 11 NeoPixel LEDs

#define PIN_WHITE         6       // GPIO 6 for White Strip
#define NUM_LEDS_WHITE    10      // 10 NeoPixel LEDs

// ESP32-S3 Hardware UART Pins for CUAV V6X Connection
#define MAVLINK_RX_PIN    44      // Default U0 RX on ESP32-S3 (Connect to CUAV TELEM TX)
#define MAVLINK_TX_PIN    43      // Default U0 TX on ESP32-S3 (Connect to CUAV TELEM RX)
#define MAVLINK_BAUD      57600   // Standard ArduPilot TELEM baud rate (57600 or 115200)

// Brightness: 0 (Off) to 255 (Full brightness)
#define DEFAULT_BRIGHTNESS 220

// ------------------------------------------------------------------------------
// COLOR DEFINITIONS
// ------------------------------------------------------------------------------
const uint8_t TEAL_R = 0;
const uint8_t TEAL_G = 200;
const uint8_t TEAL_B = 160;

const uint8_t WHITE_R = 255;
const uint8_t WHITE_G = 255;
const uint8_t WHITE_B = 255;

// ------------------------------------------------------------------------------
// ARDUPILOT COPTER FLIGHT MODES (custom_mode in HEARTBEAT)
// ------------------------------------------------------------------------------
enum CopterMode {
    MODE_STABILIZE = 0,
    MODE_ACRO      = 1,
    MODE_ALT_HOLD  = 2,
    MODE_AUTO      = 3,
    MODE_GUIDED    = 4,
    MODE_LOITER    = 5,
    MODE_RTL       = 6,
    MODE_LAND      = 9,
    MODE_DRIFT     = 11,
    MODE_SPORT     = 13,
    MODE_FLIP      = 14,
    MODE_AUTOTUNE  = 15,
    MODE_POSHOLD   = 16,
    MODE_BRAKE     = 17,
    MODE_SMART_RTL = 21,
    MODE_UNKNOWN   = 255
};

// ------------------------------------------------------------------------------
// NEOPIXEL INSTANCES
// ------------------------------------------------------------------------------
Adafruit_NeoPixel stripTeal(NUM_LEDS_TEAL, PIN_TEAL, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripWhite(NUM_LEDS_WHITE, PIN_WHITE, NEO_GRB + NEO_KHZ800);

// Hardware Serial for MAVLink
HardwareSerial SerialFC(1);

// State tracking
uint32_t currentMode = MODE_LOITER;
bool isArmed = false;
uint32_t lastHeartbeatMs = 0;
uint8_t currentBrightness = DEFAULT_BRIGHTNESS;

// Chasing light animation state
int chaseHead = 0;
unsigned long lastChaseMs = 0;
const unsigned long CHASE_INTERVAL_MS = 60; // Speed of chasing pulse (lower = faster)

// ------------------------------------------------------------------------------
// LIGHTING PATTERN HANDLERS
// ------------------------------------------------------------------------------

// Solid White on Strip 2 (Always ON for visibility)
void updateWhiteStrip() {
    for (int i = 0; i < NUM_LEDS_WHITE; i++) {
        stripWhite.setPixelColor(i, stripWhite.Color(WHITE_R, WHITE_G, WHITE_B));
    }
    stripWhite.show();
}

// 1. AUTO MODE: Solid Vibrant Teal
void displayAutoMode() {
    for (int i = 0; i < NUM_LEDS_TEAL; i++) {
        stripTeal.setPixelColor(i, stripTeal.Color(TEAL_R, TEAL_G, TEAL_B));
    }
    stripTeal.show();
}

// 2. LAND MODE: Dynamic Chasing Teal Light with Fading Comet Tail
void displayLandChasing() {
    unsigned long now = millis();
    if (now - lastChaseMs < CHASE_INTERVAL_MS) {
        return;
    }
    lastChaseMs = now;

    // Clear all LEDs first
    stripTeal.clear();

    // Draw the chasing head and a smooth 3-pixel trailing fade
    for (int i = 0; i < 4; i++) {
        int pos = (chaseHead - i + NUM_LEDS_TEAL) % NUM_LEDS_TEAL;
        float fade = 1.0f - (i * 0.28f); // 1.0, 0.72, 0.44, 0.16
        if (fade < 0.1f) fade = 0.1f;
        stripTeal.setPixelColor(
            pos,
            stripTeal.Color(
                (uint8_t)(TEAL_R * fade),
                (uint8_t)(TEAL_G * fade),
                (uint8_t)(TEAL_B * fade)
            )
        );
    }
    stripTeal.show();

    // Advance head to the next LED
    chaseHead = (chaseHead + 1) % NUM_LEDS_TEAL;
}

// 3. NORMAL / OTHER MODES: Solid Ambient Teal
void displayNormalTeal() {
    for (int i = 0; i < NUM_LEDS_TEAL; i++) {
        stripTeal.setPixelColor(i, stripTeal.Color(TEAL_R, TEAL_G, TEAL_B));
    }
    stripTeal.show();
}

// 4. DISARMED (Breathing Pulse)
void displayDisarmedBreathing() {
    float val = (exp(sin(millis() / 2000.0 * PI)) - 0.36787944) * 0.425; // 0.0 to 1.0 breathing curve
    uint8_t r = (uint8_t)(TEAL_R * val);
    uint8_t g = (uint8_t)(TEAL_G * val);
    uint8_t b = (uint8_t)(TEAL_B * val);
    for (int i = 0; i < NUM_LEDS_TEAL; i++) {
        stripTeal.setPixelColor(i, stripTeal.Color(r, g, b));
    }
    stripTeal.show();
}

// ------------------------------------------------------------------------------
// ULTRA-LIGHTWEIGHT ZERO-DEPENDENCY MAVLINK HEARTBEAT PARSER
// ------------------------------------------------------------------------------
// Decodes ArduPilot HEARTBEAT packets (v1: 0xFE or v2: 0xFD) directly from UART stream.
void parseMavlinkByte(uint8_t b) {
    static uint8_t state = 0;
    static uint8_t isV2 = 0;
    static uint8_t payloadLen = 0;
    static uint8_t msgId = 0;
    static uint8_t bytesRead = 0;
    static uint8_t payload[64];

    switch (state) {
        case 0: // Looking for Magic packet start
            if (b == 0xFD) { // MAVLink v2
                isV2 = 1;
                state = 1;
            } else if (b == 0xFE) { // MAVLink v1
                isV2 = 0;
                state = 1;
            }
            break;

        case 1: // Payload length
            payloadLen = b;
            bytesRead = 0;
            state = 2;
            break;

        case 2: // Header skip & msgid extraction
            bytesRead++;
            if (isV2) {
                // MAVLink v2: len(1), incompat(1), compat(1), seq(1), sysid(1), compid(1), msgid(3)
                if (bytesRead == 6) msgId = b; // low byte of msgid (HEARTBEAT = 0)
                if (bytesRead >= 8) {
                    bytesRead = 0;
                    state = 3;
                }
            } else {
                // MAVLink v1: len(1), seq(1), sysid(1), compid(1), msgid(1)
                if (bytesRead == 4) msgId = b; // msgid
                if (bytesRead >= 4) {
                    bytesRead = 0;
                    state = 3;
                }
            }
            break;

        case 3: // Read payload bytes
            if (bytesRead < sizeof(payload)) {
                payload[bytesRead] = b;
            }
            bytesRead++;
            if (bytesRead >= payloadLen) {
                // We have the full payload! Check if it's HEARTBEAT (msgId == 0)
                if (msgId == 0 && payloadLen >= 9) {
                    // In HEARTBEAT payload:
                    // custom_mode is uint32_t at offset 0 (little-endian)
                    uint32_t mode = (uint32_t)payload[0] |
                                    ((uint32_t)payload[1] << 8) |
                                    ((uint32_t)payload[2] << 16) |
                                    ((uint32_t)payload[3] << 24);

                    // base_mode is at offset 6 (contains armed bit 0x80)
                    uint8_t base_mode = payload[6];
                    bool armed = (base_mode & 0x80) != 0;

                    if (mode != currentMode || armed != isArmed) {
                        Serial.printf("[MAVLINK] Mode: %u | Armed: %s\n", mode, armed ? "YES" : "NO");
                    }
                    currentMode = mode;
                    isArmed = armed;
                    lastHeartbeatMs = millis();
                }
                state = 0; // Reset for next message
            }
            break;

        default:
            state = 0;
            break;
    }
}

// ------------------------------------------------------------------------------
// SETUP
// ------------------------------------------------------------------------------
void setup() {
    // 1. USB Debug Monitor
    Serial.begin(115200);
    delay(400);

    Serial.println("\n========================================================");
    Serial.println("  ESP32-S3 MAVLink NeoPixel Flight Status Controller");
    Serial.println("========================================================");
    Serial.printf("[SETUP] GPIO 5: %d LEDs (TEAL: Solid in AUTO, Chasing in LAND)\n", NUM_LEDS_TEAL);
    Serial.printf("[SETUP] GPIO 6: %d LEDs (WHITE: Solid Navigation)\n", NUM_LEDS_WHITE);
    Serial.printf("[SETUP] MAVLink UART: RX=GPIO %d, TX=GPIO %d @ %d baud\n", MAVLINK_RX_PIN, MAVLINK_TX_PIN, MAVLINK_BAUD);

    // 2. Hardware UART connection to CUAV V6X
    SerialFC.begin(MAVLINK_BAUD, SERIAL_8N1, MAVLINK_RX_PIN, MAVLINK_TX_PIN);

    // 3. Initialize Strips
    stripTeal.begin();
    stripTeal.setBrightness(currentBrightness);
    stripTeal.show();

    stripWhite.begin();
    stripWhite.setBrightness(currentBrightness);
    updateWhiteStrip(); // Turn white strip ON immediately

    displayNormalTeal();
    Serial.println("[OK] System Active. Awaiting MAVLink telemetry...");
}

// ------------------------------------------------------------------------------
// MAIN LOOP
// ------------------------------------------------------------------------------
void loop() {
    // 1. Read MAVLink stream from CUAV V6X
    while (SerialFC.available()) {
        uint8_t b = SerialFC.read();
        parseMavlinkByte(b);
    }

    // 2. Optional: Read USB Serial commands for testing
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toUpperCase();
        if (cmd == "AUTO") {
            currentMode = MODE_AUTO;
            isArmed = true;
            lastHeartbeatMs = millis();
            Serial.println("[MANUAL TEST] Simulated AUTO mode");
        } else if (cmd == "LAND") {
            currentMode = MODE_LAND;
            isArmed = true;
            lastHeartbeatMs = millis();
            Serial.println("[MANUAL TEST] Simulated LAND mode");
        } else if (cmd == "LOITER") {
            currentMode = MODE_LOITER;
            isArmed = true;
            lastHeartbeatMs = millis();
            Serial.println("[MANUAL TEST] Simulated LOITER mode");
        } else if (cmd == "DISARM") {
            isArmed = false;
            lastHeartbeatMs = millis();
            Serial.println("[MANUAL TEST] Simulated DISARMED state");
        }
    }

    // 3. Update Teal Strip based on Flight Mode
    bool hasHeartbeat = (millis() - lastHeartbeatMs < 3500);

    if (!hasHeartbeat) {
        // No MAVLink connection yet or signal lost: Default to solid Teal
        displayNormalTeal();
    } else if (currentMode == MODE_LAND) {
        // In LAND mode: DYNAMIC CHASING TEAL PULSE
        displayLandChasing();
    } else if (currentMode == MODE_AUTO) {
        // In AUTO mode: SOLID BRIGHT TEAL
        displayAutoMode();
    } else if (!isArmed) {
        // Disarmed standby: Gentle breathing Teal
        displayDisarmedBreathing();
    } else {
        // In Flight (Loiter, Guided, RTL, PosHold): Solid Teal
        displayNormalTeal();
    }

    // White strip stays solid
    // (Only periodic refresh to keep timing optimal)
    static unsigned long lastWhiteUpdate = 0;
    if (millis() - lastWhiteUpdate > 1000) {
        updateWhiteStrip();
        lastWhiteUpdate = millis();
    }
}
