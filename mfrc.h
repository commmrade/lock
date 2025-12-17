#include "HardwareSerial.h"
#include <stdint.h>
#include "Arduino.h"
#include "spi.h"

enum MfrcRegisters : uint8_t {
    CommandReg = 0x01,
    ComIrqReg = 0x04,
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

enum Commands : uint8_t {
    Idle = 0x0,
    SoftReset = 0xF,
    Transceive = 0xC,
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
        return picc_reqa();
    }

private:
    spi& spi_;
    // TODO: After basic impl of mfrc use raw registers instead of abstract Pin numbers
    uint8_t ss_pin_;
    uint8_t rst_pin_;

    bool is_running{false};

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

    void setup_timer() {
        // * The timer can also be activated automatically to meet any dedicated protocol 
        // requirements by setting the TModeReg register’s TAuto bit to logic 1 *
        uint8_t tmodreg_val = 0x8F;
        write_register(TModeReg, tmodreg_val);
        uint8_t tprescaler_val = 0xFF;
        write_register(TPrescalerReg, tprescaler_val);

        uint8_t resetval = 0xFF;
        write_register(TReloadRegH, resetval);
        write_register(TReloadRegL, resetval);
    }

    void setup_crc() {
        uint8_t value = 0xA3; // PolMFin 0, CRCPreset FFFFh (11)
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
        uint8_t value = 0x80; // Flush buffer bit set
        write_register(FIFOLevelReg, value);
    }

    void setup_antenna() {
        // IDC fo rnow
    }

    // PICC Commands
    bool picc_reqa() {
        const char reqa_command = 0x26;
        char resp[2]; // ATQA response
        int resp_size = sizeof(resp);

        bool result = transceive(&reqa_command, 1, resp, &resp_size);
        if (!result) {
            return false;
        }
        if (resp_size != sizeof(resp)) {
            return false; // Size is incorrect
        }

        // Print response bytes in hex
        Serial.print("ATQA: ");
        for (unsigned int i = 0; i < sizeof(resp); i++) {
            if ((uint8_t)resp[i] < 0x10) Serial.print('0'); // leading zero
            Serial.print((uint8_t)resp[i], HEX);
            Serial.print(' ');
        }

        return true;
    }

    // TODO: return status codes
    bool transceive(const char* send_buf, int send_buf_n, char* recv_buf, int* recv_buf_n) {
        write_register(FIFOLevelReg, 0x80); // Clear fifo buffer
        write_register(ComIrqReg, 0x7F); // нужно сбросить все IRQ-биты, они ресетаютс если 1 записать

        for (auto i = 0; i < send_buf_n; ++i) {
            write_register(FIFODataReg, send_buf[i]);
        }
        write_register(CommandReg, Transceive);
        write_register(BitFramingReg, 0x87); // Start Send bit set, last 7 is because we need the short frame format


        unsigned long end_millis = millis() + 50;
        uint8_t inter_bits = 0;
        do {
            inter_bits = read_register(ComIrqReg);
            if (inter_bits & 0x30) {
                break; // Received something
            }
            delay(10);
        } while (millis() < end_millis);
        if (!(inter_bits & 0x30)) {
            // TODO: How to determine if transaction failed
            // Serial.println("Didn't get an interrupt");
            return false; // Didn't get an interrupt
        }
        if (!(inter_bits & 0x40)) {
            // TxIrq - data was not transmitted if bit is not set
            Serial.println("Failed to transmit data, should abort communication");
            return false;
        }

        if (millis() >= end_millis) {
            Serial.println("Timed out");
            return false; // timed out
        }
        
        auto bytes_n = read_register(FIFOLevelReg);
        if (*recv_buf_n < bytes_n) {
            Serial.println("Buffer is too small");
            return false; // Buffer is too small
        }

        *recv_buf_n = bytes_n;
        for (auto i = 0; i < bytes_n; ++i) {
            auto value = read_register(FIFODataReg);
            recv_buf[i] = value;
        }

        // Clear transceive command after it is done
        write_register(CommandReg, Idle);
        // Serial.println("OK");
        return true; // OK
    }
    
    /*
    MFRC_REGW(CMD_REG,IDLE); //Clear command register
    MFRC_REGW(IRQ_REG,0x7F);
    MFRC_REGW(BITFRAME,0x00);
    MFRC_REGW(FIFO_LEV,0x80); //Clear FIFO buffer
    MFRC_FIFOW(sendData,sendsize); //Write data to FIFO ready for transmission
    //MFRC_FIFOR(FIFO_State, sendsize);
    //CDC_Transmit_FS(FIFO_State, sendsize);
    MFRC_REGW(CMD_REG,TRANSCEIVE); //Send FIFO data and receive PICC response
    MFRC_REGR(BITFRAME,&BIT_val);
    MFRC_REGW(BITFRAME,(BIT_val|0x80)); //Start send bit
    HAL_Delay(100);
    MFRC_REGR(IRQ_REG,&IRQval);
    if((IRQval&0x30)!=0x30){ //Return error if RXIRQ and IDLEIRQ bits are not set
        return(HAL_ERROR);
    }

    MFRC_FIFOR(recdata,recsize); //Read and store received data
    return(HAL_OK);
    
    */
};