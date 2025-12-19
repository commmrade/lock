#include <string.h>
#include <stddef.h>
#include "HardwareSerial.h"
#include <stdint.h>
#include "Arduino.h"
#include "spi.h"

enum MfrcRegisters : uint8_t {
    CommandReg = 0x01,
    ComIrqReg = 0x04,
    DivIrqReg = 0x05,
    ErrorReg = 0x06,
    FIFODataReg = 0x09,
    FIFOLevelReg = 0x0A,
    ModeReg = 0x11,
    TxModeReg = 0x12,
    RxModeReg = 0x13,
    TxControlReg = 0x14,
    TxASKReg = 0x15,
    TxSelReg = 0x16,
    CollReg = 0x0E,
    CRCResultRegH = 0x21,
    CRCResultRegL = 0x22,
    VersionReg = 0x37,
    TModeReg = 0x2A,
    TPrescalerReg = 0x2B,
    TReloadRegH = 0x2C,
    BitFramingReg = 0x0D,
    TReloadRegL = 0x2D,
};

#define FIFO_LEVEL_REG_CLEAR 0x80
#define COM_IRQ_REG_RESET 0x7F
#define COM_IRQ_REG_RXIRQ_OR_IDLEIRQ 0x30
#define COM_IRQ_REG_RXIRQ 0x20
#define COM_IRQ_REG_TXIRQ 0x40
#define COM_IRQ_REG_ERR 0x2
#define BIT_FRAM_REG_SS_SF 0x87 // Start Send, Short frame
#define BIT_FRAM_REG_SS 0x80
#define MODE_REG_DEFAULT 0x3D

enum Commands : uint8_t {
    Idle = 0x0,
    SoftReset = 0xF,
    Transceive = 0xC,

    // iso 1443 stuff
    REQA = 0x26,
    SEL_CL1 = 0x93,
    SEL_CL2 = 0x95,
    SEL_CL3 = 0x97,
};

enum Status {
    Ok,
    NoRxInterrupt,
    TransactionFailed,
    BufferTooSmall,
    TimedOut,
    Error,
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

        // Soft reset this mf
        reset();

        // timer setup
        setup_timer();

        // CRC
        setup_crc();

        // Tx
        setup_transmission();

        // Rx
        setup_reception();

        // CLear fifo and interrupt flags
        flush_fifo();

        // enable antenna
        setup_antenna();
        set_antenna(true);
        delay(10); // ISO 14443A says to wait >5 ms before sending REQA

        // Put into iddle
        write_register(CommandReg, Idle);
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

    void set_antenna(bool enable) {
        if (enable) {
            uint8_t txcontrol_val = read_register(TxControlReg);
            txcontrol_val |= 0x03; // enable tx1 and tx2
            write_register(TxControlReg, txcontrol_val);
        } else {
            uint8_t txcontrol_val = read_register(TxControlReg);
            txcontrol_val &= ~0x03; // enable tx1 and tx2
            write_register(TxControlReg, txcontrol_val);
        }
    }

    void reset() {
        write_register(MfrcRegisters::CommandReg, Commands::SoftReset);
        // TODO: Wait properly
        delay(65); // Wait for the device to reset fully (may wanna poll here but idc)
    }

    bool is_card_available() {
        auto result = PICC_REQA();
        if (result == Status::Ok) {
            return true;
        }
        return false;
    }

    [[nodiscard]] uint8_t read_register(MfrcRegisters reg) const {
        digitalWrite(ss_pin_, LOW); // poll slave

        spi_.start_transaction(F_CPU / 4, MSB_ORDER, MODE_0);
        spi_.transfer(0x80 | (static_cast<uint8_t>(reg) << 1)); // format is [mode (0 or 1) msb][6 value bits][0 lsb]
        auto result = spi_.transfer(0);
        spi_.end_transaction();

        digitalWrite(ss_pin_, HIGH);
        return result;
    }

    uint8_t write_register(MfrcRegisters reg, uint8_t value) {
        digitalWrite(ss_pin_, LOW);

        spi_.start_transaction(F_CPU / 4, MSB_ORDER, MODE_0);
        spi_.transfer(static_cast<uint8_t>(reg) << 1); // Write byte 0 which is address
        auto result = spi_.transfer(value);
        spi_.end_transaction();
    
        digitalWrite(ss_pin_, HIGH);
        return result;
    }

    void write_register(MfrcRegisters reg, const uint8_t* buf, size_t buf_size) {
        digitalWrite(ss_pin_, LOW);
        spi_.start_transaction(F_CPU / 4, MSB_ORDER, MODE_0);

        spi_.transfer(static_cast<uint8_t>(reg) << 1);
        for (size_t i = 0; i < buf_size; ++i) {
            spi_.transfer(buf[i]);
        }

        spi_.end_transaction();
        digitalWrite(ss_pin_, HIGH);
    }

    

    // PICC Commands
    // REQA checks if there is a card in the field
    // TODO: Probably should let user specify ComIrqReq error mask and ErrorReg mask
    Status transceive(const char* send_buf, int send_buf_n, char* recv_buf, int* recv_buf_n, uint8_t bitframe) {
        write_register(CommandReg, Idle);
        write_register(BitFramingReg, 0x00);
        write_register(FIFOLevelReg, FIFO_LEVEL_REG_CLEAR); // Clear fifo buffer
        write_register(ComIrqReg, COM_IRQ_REG_RESET); // нужно сбросить все IRQ-биты, они ресетаютс если 1 записать
        
        write_register(FIFODataReg, (const uint8_t*)send_buf, send_buf_n);
        write_register(CommandReg, Transceive);
        write_register(BitFramingReg, bitframe); // Start Send bit set, last 7 is because we need the short frame format


        unsigned long end_millis = millis() + 50;
        uint8_t inter_bits = 0;
        // TODO: Use timer?
        do {
            inter_bits = read_register(ComIrqReg);
            if (inter_bits & COM_IRQ_REG_RXIRQ_OR_IDLEIRQ) {
                break; // Transceive finished
            }
            if (inter_bits & 0x01) {
                return Status::TimedOut; // Timer interrupt
            }
            delay(1);
        } while (millis() < end_millis);
        if (millis() >= end_millis) {
            return Status::TimedOut;
        }

        if (!(inter_bits & COM_IRQ_REG_RXIRQ_OR_IDLEIRQ)) {
            return Status::NoRxInterrupt;
        }
        auto val = read_register(ErrorReg);
        if (val) {
            return Status::Error;
        }

        
        
        auto bytes_n = read_register(FIFOLevelReg);
        if (*recv_buf_n < bytes_n) {
            return Status::BufferTooSmall;
        }

        *recv_buf_n = bytes_n;
        for (auto i = 0; i < bytes_n; ++i) {
            auto value = read_register(FIFODataReg);
            recv_buf[i] = value;
        }

        // Clear transceive command after it is done
        write_register(CommandReg, Idle);
        return Status::Ok;
    }

    Status PICC_REQA() {
        const char command = REQA;
        char resp[2]; // ATQA response
        int resp_size = sizeof(resp);

        Status ret = transceive(&command, 1, resp, &resp_size, BIT_FRAM_REG_SS_SF);
        if (ret != Status::Ok) {
            return ret;
        }
        if (resp_size != sizeof(resp)) {
            return Status::Error;
        }

        return Status::Ok;
    }

    void clear_register_bitmask(MfrcRegisters reg, uint8_t mask) {
        uint8_t v = read_register(reg);
        v &= ~mask;
        write_register(reg, v);
    }


    Status PICC_anticollision_seq() {
        uint8_t UID_buf[10]{}; // Max uid supported is 10 bytes
        uint8_t at_response_buf[9]; // 4 uid bytes + 1 bcc
        int resp_buf_size = sizeof(at_response_buf);

        uint8_t at_send_buf[20]; // max packet size is 7

        at_send_buf[0] = SEL_CL1;
        at_send_buf[1] = 0x20;

        // UID field is 0 at this point
        clear_register_bitmask(CollReg, 0x80); // ValuesAfterColl = 1 (как делают библиотеки)
        int buf_size = resp_buf_size;
        Status ret = transceive((const char*)at_send_buf, 2, (char*)at_response_buf, &buf_size, BIT_FRAM_REG_SS);
        if (ret != Status::Ok) {
            Serial.print("Fuck off: ");
            Serial.println(static_cast<uint8_t>(ret));
            return ret;
        }
        if (buf_size != 5) {
            return Status::Error;
        }

        Serial.print("===");
        for (auto i = 0; i < 4; ++i) {
            Serial.print("0x");
            Serial.print(at_response_buf[i], HEX);
            Serial.print(" ");
        }
        Serial.print("===");

        delay(100);
        // Serial.println((const char*)at_response_buf);
        if (at_response_buf[0] != 0x88) {
            Serial.println("Starting SELECT");
            at_send_buf[0] = SEL_CL1;
            at_send_buf[1] = 0x70; // For select command
            memcpy(at_send_buf + 2, at_response_buf, 4); // 4 uid bytes + BCC
            uint8_t bcc = at_response_buf[0] ^ at_response_buf[1] ^ at_response_buf[2] ^ at_response_buf[3];
            at_send_buf[6] = bcc;

            int buf_size = resp_buf_size;
            uint8_t crc[2];
            calculate_crc(at_send_buf, 7, crc);
            at_send_buf[7] = crc[0];
            at_send_buf[8] = crc[1];
            
            // 9 because also CRC 2 bytes
            clear_register_bitmask(CollReg, 0x80); // ValuesAfterColl = 1 (как делают библиотеки)
            uint8_t rfc = read_register(static_cast<MfrcRegisters>(0x26));
            rfc |= (0x07 << 4); // RxGain = 111 -> максимальное усиление 48 dB
            write_register(static_cast<MfrcRegisters>(0x26), rfc);
            Status ret = transceive((const char*)at_send_buf, 9, (char*)at_response_buf, &buf_size, BIT_FRAM_REG_SS);
            if (ret != Status::Ok) {
                Serial.print("Select is fucked: ");
                Serial.println(static_cast<uint8_t>(ret));
                uint8_t error = read_register(ErrorReg);
                uint8_t comirq = read_register(ComIrqReg);
                Serial.print("ErrorReg = 0x"); Serial.println(error, HEX);
                Serial.print("ComIrqReg = 0x"); Serial.println(comirq, HEX);
                uint8_t fifolevel = read_register(FIFOLevelReg);
                Serial.print("FifoLevelReg = "); Serial.println(fifolevel);
                return Status::Error;
            }
            Serial.print("Got from select bytes: ");
            Serial.println(buf_size);
            // Select responds with a single byte
            if (at_response_buf[0] & 0x20) {
                Serial.println("Cascade bit set in SAK response, damn");
                return Status::Error; // TODO: Contineu cacsade
            }

            Serial.println("Successfully got UID");
            // Check cascade flag field
        }

        return Status::Ok;
    }

    void calculate_crc(const uint8_t *data, uint8_t len, uint8_t *result) {
        write_register(CommandReg, Idle);
        write_register(DivIrqReg, 0x04);          // очистить CRC IRQ
        write_register(FIFOLevelReg, 0x80);       // flush
        write_register(FIFODataReg, data, len);
        write_register(CommandReg, 0x03);         // CalcCRC
        uint32_t start = millis();
        while (!(read_register(DivIrqReg) & 0x04)) {
            if (millis() - start > 50) break;
        }

        result[0] = read_register(CRCResultRegL);
        result[1] = read_register(CRCResultRegH);
    }

private:
    spi& spi_;
    // TODO: After basic impl of mfrc use raw registers instead of abstract Pin numbers
    uint8_t ss_pin_;
    uint8_t rst_pin_;

    void setup_timer() {
        // * The timer can also be activated automatically to meet any dedicated protocol 
        // requirements by setting the TModeReg register’s TAuto bit to logic 1 *
        uint8_t tmodreg_val = 0x8F;
        write_register(TModeReg, tmodreg_val);
        uint8_t tprescaler_val = 0xFF;
        write_register(TPrescalerReg, tprescaler_val);

        // Set to max value, since idc about timer
        uint8_t resetval = 0xFF;
        write_register(TReloadRegH, resetval);
        write_register(TReloadRegL, resetval);
    }

    void setup_crc() {
        uint8_t value = MODE_REG_DEFAULT;
        write_register(ModeReg, value);
    }

    void setup_transmission() {
        uint8_t value = 0x0; // 106 KBD, crc off
        write_register(TxModeReg, value);

        uint8_t askreg_val = 0x40; // 100% ASK Modulation
        write_register(TxASKReg, askreg_val);
    }

    void setup_reception() {
        uint8_t value = 0x0; // 106 kbd, "receiver is deactivated after receiving a data frame", CRC OFF
        write_register(RxModeReg, value);
    }

    void flush_fifo() {
        uint8_t value = FIFO_LEVEL_REG_CLEAR; // Flush buffer bit set
        write_register(FIFOLevelReg, value);
    }

    void setup_antenna() {
        // IDC fo rnow
    }
};