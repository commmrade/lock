#pragma once
enum SpiDataOrder : uint8_t {
    LSB_ORDER = 0x01,
    MSB_ORDER = 0x00,
};
enum SpiDataMode : uint8_t {
    MODE_0 = 0,
    MODE_1,
    MODE_2,
    MODE_3
};

// TODOS:
// 1. Interrupt support (especially in a transaction)
class spi {
    struct State {
        uint8_t spcr_reg;
        uint8_t spsr_reg;
    };
    State old_state;
public:
    spi() = default;
    void init(uint32_t rate = F_CPU / 64, SpiDataOrder dorder = MSB_ORDER, SpiDataMode dmode = MODE_0) {
        // SPI Enable and make this controller Master, set speed rate to Fosc / 64
        SPCR |= (1 << SPE) | (1 << MSTR);
        DDRB |= (1 << PB3) | (1 << PB5); // PB3 is MOSI, PB5 is SCK, defined by datasheet
        set_rate(rate);
        set_data_order(dorder);
        set_data_mode(dmode);

        // SPCR |= (1 << DORD);
        // CPOL - SCK is low when idle, 1 - high when idle
        // CPHA - sample on start of signal, 1 - end of signal (Trailing Edge)
        // SPCR |= | (1 << CPOL) | (1 << CPHA);
        // setting up SPI clock rate
        // SPCR |= (1 << SPR1) | (1 << SPR0);
        // SPSR |= (1 << SPI2X);        
    }

    uint8_t transfer(uint8_t val) {
        SPDR = val; // Set data to SPI Data registyer
        while (!(SPSR & (1 << SPIF))); // wait untl it is passed to a slave
        return SPDR; // get slave's data
    }

    void set_rate(uint32_t rate) {
        uint8_t reg_params; // Contains smth like 00000111
        switch (rate) {
            case F_CPU / 4: reg_params = 0; break;
            case F_CPU / 16: reg_params = 1; break;
            case F_CPU / 64: reg_params = 2; break;
            case F_CPU / 128: reg_params = 3; break;
            case F_CPU / 2: reg_params = 4; break;
            case F_CPU / 8: reg_params = 5; break;
            case F_CPU / 32: reg_params = 6; break;
            default: reg_params = 0; break;
        }
        SPCR |= (reg_params & 0b00000011);
        SPSR |= (reg_params & 0b00000100);
    }

    void set_data_order(SpiDataOrder order) {
        SPCR |= ((uint8_t)order << DORD);
    }

    void set_data_mode(SpiDataMode mode) {
        SPCR |= (((mode == SpiDataMode::MODE_2 || mode == SpiDataMode::MODE_3) ? 1 : 0) << CPOL) | (((mode == SpiDataMode::MODE_1 || mode == SpiDataMode::MODE_3) ? 1 : 0) << CPHA);
    }

    void start_transaction(uint32_t rate, SpiDataOrder d_order, SpiDataMode d_mode) {
        old_state.spsr_reg = SPSR;
        old_state.spcr_reg = SPCR;

        set_rate(rate);
        set_data_order(d_order);
        set_data_mode(d_mode);
    }
    void end_transaction() {
        SPSR = old_state.spsr_reg;
        SPCR = old_state.spcr_reg;
    }
};