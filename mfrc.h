#include "Arduino.h"
#include "spi.h"

enum MfrcRegisters : uint8_t {
    VersionReg = 0x37,
};

class Mfrc_522 {

public:
    Mfrc_522(spi& spi) : spi_(spi) {}
    void init(uint8_t ss_pin, uint8_t rst_pin) {
        ss_pin_ = ss_pin;
        rst_pin_ = rst_pin;

        // Unselect slave
        pinMode(ss_pin_, OUTPUT);
        digitalWrite(ss_pin_, HIGH);

        // Reset sequence
        pinMode(rst_pin_, OUTPUT);
        // Hard power down
        digitalWrite(rst_pin_, LOW);
        // "The spike filter rejects signals shorter than 10 ns"
        delayMicroseconds(5);
        // Power up
        digitalWrite(rst_pin_, HIGH);
        delay(50); // Oscillator startup delay

        // TODO: Parameters, antenna stuff and so on
    }
    bool is_powered() const {
        pinMode(rst_pin_, INPUT);
        bool val = digitalRead(rst_pin_);
        pinMode(rst_pin_, OUTPUT);
        return val;
    }
    
    int software_version() const {
        auto version = read_register(VersionReg);
        return version;
    }

private:
    spi& spi_;
    // TODO: After basic impl of mfrc use raw registers instead of abstract Pin numbers
    uint8_t ss_pin_;
    uint8_t rst_pin_;

    bool is_running{false};

    uint8_t read_register(MfrcRegisters reg) const {
        digitalWrite(ss_pin_, LOW); // poll slave

        spi_.start_transaction(F_CPU / 4, MSB_ORDER, MODE_0);
        spi_.transfer(0x80 | (static_cast<uint8_t>(reg) << 1)); // format is [mode (0 or 1) msb][6 value bits][0 lsb]
        auto result = spi_.transfer(0);
        spi_.end_transaction();

        digitalWrite(ss_pin_, HIGH);
        return result;
    }

    void write_register(MfrcRegisters reg) {
        digitalWrite(ss_pin_, LOW);

        spi_.start_transaction(F_CPU / 4, MSB_ORDER, MODE_0);
        spi_.transfer(static_cast<uint8_t>(reg) << 1);
        spi_.end_transaction();
    
        digitalWrite(ss_pin_, HIGH);
    }
};