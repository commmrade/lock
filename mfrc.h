#include "HardwareSerial.h"
#include <stdint.h>
#include "Arduino.h"
#include "spi.h"

enum MfrcRegisters : uint8_t {
    CommandReg = 0x01,
    ComIrqReg = 0x04,
    ErrorReg = 0x06,
    FIFODataReg = 0x09,
    FIFOLevelReg = 0x0A,
    ModeReg = 0x11,
    TxModeReg = 0x12,
    RxModeReg = 0x13,
    TxControlReg = 0x14,
    TxASKReg = 0x15,
    TxSelReg = 0x16,
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
#define MODE_REG_DEFAULT 0x3D

enum Commands : uint8_t {
    Idle = 0x0,
    SoftReset = 0xF,
    Transceive = 0xC,

    // iso 1443 stuff
    REQA = 0x26,
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
        delay(50); // Wait for the device to reset fully (may wanna poll here but idc)r
    }

    bool is_card_available() {
        auto result = PICC_REQA();
        if (result == Status::Ok) {
            return true;
        }
        return false;
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

    // PICC Commands
    // REQA checks if there is a card in the field
    // TODO: Probably should let user specify ComIrqReq error mask and ErrorReg mask
    Status transceive(const char* send_buf, int send_buf_n, char* recv_buf, int* recv_buf_n, uint8_t bitframe) {
        write_register(FIFOLevelReg, FIFO_LEVEL_REG_CLEAR); // Clear fifo buffer
        write_register(ComIrqReg, COM_IRQ_REG_RESET); // нужно сбросить все IRQ-биты, они ресетаютс если 1 записать

        for (auto i = 0; i < send_buf_n; ++i) {
            write_register(FIFODataReg, send_buf[i]);
        }
        write_register(CommandReg, Transceive);
        write_register(BitFramingReg, bitframe); // Start Send bit set, last 7 is because we need the short frame format


        unsigned long end_millis = millis() + 50;
        uint8_t inter_bits = 0;
        do {
            inter_bits = read_register(ComIrqReg);
            if (inter_bits & COM_IRQ_REG_RXIRQ_OR_IDLEIRQ) {
                break; // Received something
            }
            if (inter_bits & 0x01) {
                return Status::TimedOut; // Timer interrupt
            }
            delay(1);
        } while (millis() < end_millis);
        if ((inter_bits & (COM_IRQ_REG_ERR)) && (inter_bits & COM_IRQ_REG_RXIRQ)) {
            auto val = read_register(ErrorReg);
            if (val) {
                return Status::Error;
            }
        } else if (!(inter_bits & COM_IRQ_REG_RXIRQ_OR_IDLEIRQ)) {
            return Status::NoRxInterrupt;
        }
        // TODO: Check Error bit flag and then check for errors in ErrorReg

        if (millis() >= end_millis) {
            return Status::TimedOut;
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