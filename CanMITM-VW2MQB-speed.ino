/*
 * ESP32 CAN Translator: 0x288 -> 0x0FD with VAG CRC8
 * Based on ESP32VWMITM hardware configuration
 * 
 * Hardware Setup:
 * - Built-in ESP32 CAN (CAN0): RX=GPIO16, TX=GPIO17 - Listens for 0x288
 * - MCP2515 SPI CAN (CAN1): CS=GPIO5, INT=GPIO27 - Transmits 0x0FD
 * - MCP2515 Crystal: 16 MHz
 * - Both buses: 500 kbps
 */

#ifndef ARDUINO_ARCH_ESP32
  #error "Select an ESP32 board" 
#endif

#include <ACAN_ESP32.h>
#include <ACAN2515.h>

#define LED_BUILTIN 2

// ===== MCP2515 PIN DEFINITIONS =====
static const byte MCP2515_SCK  = 18;
static const byte MCP2515_MOSI = 23;
static const byte MCP2515_MISO = 19;
static const byte MCP2515_CS   = 5;
static const byte MCP2515_INT  = 27;
static const uint32_t QUARTZ_FREQUENCY = 16UL * 1000UL * 1000UL; // 16 MHz

// ===== CAN MESSAGE IDs =====
#define MSG_0x288 0x288
#define MSG_0x0FD 0x0FD

// ===== GLOBAL OBJECTS =====
ACAN2515 can1(MCP2515_CS, SPI, MCP2515_INT);

// ===== GLOBAL VARIABLES =====
uint8_t counter_0FD = 0x00;
unsigned long rxCount = 0;
unsigned long txCount = 0;
unsigned long lastPrint = 0;

// ===== VAG CRC8 CALCULATION =====
const uint8_t magicBytes[16] = {
    0xA3, 0xA9, 0x66, 0x13, 0x85, 0x1F, 0x0D, 0x37,
    0xF0, 0xC2, 0x8E, 0xA1, 0x38, 0xB1, 0x4E, 0xD9
};

uint8_t calculateCRC(uint8_t *payload, uint8_t len, uint8_t counter) {
    uint8_t crc = 0x00;
    
    for (uint8_t i = 0; i < len; i++) {
        crc ^= payload[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x2F;
            else
                crc = crc << 1;
        }
    }
    
    return crc ^ magicBytes[counter & 0x0F];
}

// ===== BUILD PAYLOAD FOR CRC =====
void buildPayload(uint8_t *payload, uint8_t *frame) {
    payload[0] = frame[1] & 0xF0;  // High nibble of counter byte
    for (uint8_t i = 0; i < 6; i++) {
        payload[i + 1] = frame[i + 2];
    }
    payload[7] = 0x00;  // Virtual byte
}

// ===== TRANSLATE 0x288 TO 0x0FD =====
void translateMessage(const CANMessage &inMessage) {
    // Extract speed from byte 3 (D4) of 0x288
    uint8_t speedByte = inMessage.data[3];
    float speedKph = (float)speedByte * 1.28f;
    uint16_t speedValue = (uint16_t)(speedKph * 100.0f + 0.5f); // Round
    
    // Build new frame for 0x0FD
    CANMessage frame;
    frame.id = MSG_0x0FD;
    frame.ext = false;
    frame.rtr = false;
    frame.len = 8;
    
    frame.data[1] = 0xD0 | (counter_0FD & 0x0F);  // Counter in lower nibble
    frame.data[2] = 0x1F;
    frame.data[3] = 0x82;
    frame.data[4] = (uint8_t)(speedValue & 0xFF);        // Speed low byte
    frame.data[5] = (uint8_t)((speedValue >> 8) & 0xFF); // Speed high byte
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    
    // Calculate CRC
    uint8_t payload[8];
    buildPayload(payload, frame.data);
    frame.data[0] = calculateCRC(payload, 8, counter_0FD);
    
    // Send on CAN1 (MCP2515)
    const bool ok = can1.tryToSend(frame);
    
    if (ok) {
        txCount++;
        
        // Debug every 100 messages
        if (txCount % 100 == 0) {
            Serial.print("TX #");
            Serial.print(txCount);
            Serial.print(": 0x0FD | Speed: ");
            Serial.print(speedKph, 1);
            Serial.print(" km/h | Counter: 0x");
            Serial.print(counter_0FD, HEX);
            Serial.print(" | CRC: 0x");
            Serial.println(frame.data[0], HEX);
        }
    } else {
        Serial.println("Failed to send 0x0FD to CAN1");
    }
    
    // Increment counter (0x00-0x0F)
    counter_0FD = (counter_0FD + 1) & 0x0F;
}

// ===== CALLBACK FOR 0x288 MESSAGES =====
static void received0x288(const CANMessage &inMessage) {
    rxCount++;
    translateMessage(inMessage);
}

// ===== SETUP =====
void setup() {
    // LED indicator
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    
    // Start serial
    Serial.begin(115200);
    while (!Serial) {
        delay(50);
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
    
    Serial.println("\n========================================");
    Serial.println("ESP32 CAN Translator: 0x288 -> 0x0FD");
    Serial.println("========================================\n");
    
    // ===== Configure Built-in ESP32 CAN (CAN0) =====
    Serial.println("Initializing ESP32 CAN (CAN0)...");
    ACAN_ESP32_Settings settings0(500 * 1000); // 500 kbps
    settings0.mRxPin = GPIO_NUM_16;
    settings0.mTxPin = GPIO_NUM_17;
    
    const uint32_t errorCode0 = ACAN_ESP32::can.begin(settings0);
    if (errorCode0 == 0) {
        Serial.println("  ✓ ESP32 CAN configured at 500 kbps");
    } else {
        Serial.print("  ✗ ESP32 CAN error: 0x");
        Serial.println(errorCode0, HEX);
        while (1) delay(100);
    }
    
    // ===== Configure MCP2515 (CAN1) =====
    Serial.println("\nInitializing MCP2515 (CAN1)...");
    SPI.begin(MCP2515_SCK, MCP2515_MISO, MCP2515_MOSI);
    
    ACAN2515Settings settings1(QUARTZ_FREQUENCY, 500UL * 1000UL); // 500 kbps
    settings1.mRequestedMode = ACAN2515Settings::NormalMode;
    
    // Set up filter for 0x288 on CAN0 receive
    const ACAN2515Mask rxm0 = standard2515Mask(0x7FF, 0, 0);
    const ACAN2515AcceptanceFilter filters[] = {
        {standard2515Filter(MSG_0x288, 0, 0), received0x288}
    };
    
    const uint16_t errorCode1 = can1.begin(settings1, [] { can1.isr(); }, rxm0, rxm0, filters, 1);
    
    if (errorCode1 == 0) {
        Serial.println("  ✓ MCP2515 configured successfully");
        Serial.print("  - Bit Rate: ");
        Serial.print(settings1.actualBitRate());
        Serial.println(" bit/s");
        Serial.print("  - Exact bit rate: ");
        Serial.println(settings1.exactBitRate() ? "yes" : "no");
    } else {
        Serial.print("  ✗ MCP2515 error: 0x");
        Serial.println(errorCode1, HEX);
        Serial.println("  Check: Wiring, 16MHz crystal, power");
        while (1) delay(100);
    }
    
    Serial.println("\n✓ System Ready!");
    Serial.println("Listening for 0x288 on CAN0...");
    Serial.println("Transmitting 0x0FD on CAN1...\n");
    
    digitalWrite(LED_BUILTIN, LOW);
}

// ===== MAIN LOOP =====
void loop() {
    CANMessage frame;
    
    // Process messages from ESP32 CAN (CAN0)
    // Looking for 0x288 messages to translate
    if (ACAN_ESP32::can.receive(frame)) {
        if (frame.id == MSG_0x288 && frame.len >= 4) {
            translateMessage(frame);
        }
    }
    
    // Dispatch any received messages on MCP2515 (if using filters/callbacks)
    can1.dispatchReceivedMessage();
    
    // Status print every 5 seconds
    if (millis() - lastPrint > 5000) {
        lastPrint = millis();
        
        Serial.print("[Stats] RX: ");
        Serial.print(rxCount);
        Serial.print(" | TX: ");
        Serial.print(txCount);
        
        if (rxCount > 0) {
            float ratio = (txCount * 100.0f) / rxCount;
            Serial.print(" | Ratio: ");
            Serial.print(ratio, 1);
            Serial.println("%");
        } else {
            Serial.println(" | Waiting for messages...");
        }
    }
    
    delay(1);
}