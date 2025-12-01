#include "MFRC522.h"
#include <string.h>

constexpr int SS_PIN = 3;
constexpr int RST_PIN = 8; 

constexpr int UNLOCKED_PIN = 2;
constexpr int LOCKED_PIN = 4;

static MFRC522 mfrc{SS_PIN, RST_PIN};

constexpr byte VERIFIED_KEY[]{0x74, 0xE5, 0x86, 0x04};

void setup() {
    Serial.begin(9600);
    SPI.begin();
    mfrc.PCD_Init();
    delay(10);
    mfrc.PCD_DumpVersionToSerial();

    pinMode(UNLOCKED_PIN, OUTPUT);
    pinMode(LOCKED_PIN, OUTPUT);
}

void loop() {
    if (!mfrc.PICC_IsNewCardPresent()) {
        return;
    }
    if (!mfrc.PICC_ReadCardSerial()) {
        return;
    }

    int selected_pin;
    if (!memcmp(mfrc.uid.uidByte, VERIFIED_KEY, max(sizeof(VERIFIED_KEY), mfrc.uid.size))) {
        selected_pin = UNLOCKED_PIN;
    } else {
        selected_pin = LOCKED_PIN;
    }
    digitalWrite(selected_pin, HIGH);
    delay(1000);
    digitalWrite(selected_pin, LOW);
}