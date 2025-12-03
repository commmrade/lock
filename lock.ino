#include "MFRC522.h"
#include <string.h>

constexpr int SS_PIN = 10;
constexpr int RST_PIN = 9; 

constexpr int UNLOCKED_PIN = 2;
constexpr int LOCKED_PIN = 4;

static MFRC522 mfrc{SS_PIN, RST_PIN};

constexpr byte VERIFIED_KEY[]{0x74, 0xE5, 0x86, 0x04};

void setup() {
    Serial.begin(9600); // Start serial so i can send info to the pc to output in a serial monitor
    SPI.begin(); // Start SPI communication
    mfrc.PCD_Init(); // Protocol handshake
    delay(10); // Wait a bit so everything is r eady
    mfrc.PCD_DumpVersionToSerial(); // Dump all the stuff i dotn care about

    // Pins for LEDs
    pinMode(UNLOCKED_PIN, OUTPUT);
    pinMode(LOCKED_PIN, OUTPUT);
}

bool card_available() {
    if (!mfrc.PICC_IsNewCardPresent()) { // Does a REQ to check if there is a card available
        // Serial.println("Not present");
        return false;
    }
    if (!mfrc.PICC_ReadCardSerial()) { // Reads UID into an internal buffer
        // Serial.println("Can't read UID");
        return false;
    }
    return true;
}

bool try_unlock() {
    return !memcmp(mfrc.uid.uidByte, VERIFIED_KEY, min(sizeof(VERIFIED_KEY), mfrc.uid.size));
}

void loop() {
    bool available = card_available();
    if (!available) {
        return;
    }
    
    bool is_unlocked = try_unlock();
    int selected_pin = is_unlocked ? UNLOCKED_PIN : LOCKED_PIN;

    Serial.println("fuck");

    digitalWrite(selected_pin, HIGH);
    delay(1000);
    digitalWrite(selected_pin, LOW);
}