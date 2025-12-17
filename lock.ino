#include "MFRC522.h"
#include <string.h>
#include "uart.h"
#include "spi.h"
#include "mfrc.h"

constexpr int SS_PIN = 10;
constexpr int RST_PIN = 9; 

constexpr int UNLOCKED_PIN = 2; // PORTD
constexpr int LOCKED_PIN = 4; // PORTD

// static MFRC522 mfrc{SS_PIN, RST_PIN};


constexpr byte VERIFIED_KEY[]{0x74, 0xE5, 0x86, 0x04};

// static Uart0 serial{};
static spi spi{};
static Mfrc_522 mfrc_test{spi};

void setup() {
    Serial.begin(9600);
    delay(100);
    Serial.println("Starting");
    delay(500); // Wait a bit so everything is r eady
    // serial.init(9600);
    spi.init();


    mfrc_test.init(SS_PIN, RST_PIN);
    delay(10); // Wait a bit so everything is r eady
    auto version = mfrc_test.software_version();
    Serial.println(version);
    // serial.println(String{version});

    // Pins for LEDs
    // Set those as OUTPUTs
    DDRD |= (1 << UNLOCKED_PIN) | (1 << LOCKED_PIN);
}

bool try_unlock() {
    // return !memcmp(mfrc.uid.uidByte, VERIFIED_KEY, min(sizeof(VERIFIED_KEY), mfrc.uid.size));
    return false;
}

void loop() {
    
    /// Beginner way
    // bool available = mfrc_test.is_card_available();
    // if (available) {
    //     PORTD |= (1 << LOCKED_PIN);
    //     delay(1000);
    //     // Set to LOW
    //     PORTD &= ~(1 << LOCKED_PIN);
    // }
    /// Advanced api
    Status status = mfrc_test.PICC_REQA();
    if (status == Status::TransactionFailed) {
        Serial.println("Transactions is fucked, aborting");
    } else if (status == Status::Ok) {
        PORTD |= (1 << LOCKED_PIN);
        delay(1000);
        // Set to LOW
        PORTD &= ~(1 << LOCKED_PIN);
    }
    /// End


    // bool is_unlocked = try_unlock();
    // int selected_pin = is_unlocked ? UNLOCKED_PIN : LOCKED_PIN;
    
    // // set to HIGH
    // PORTD |= (1 << selected_pin);
    // delay(1000);
    // // Set to LOW
    // PORTD &= ~(1 << selected_pin);
}