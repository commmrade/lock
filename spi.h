
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
    uint8_t old_spcr{};
    uint8_t old_spsr{};
public:
    spi() = default;
    void init() {
        // SPI Enable and make this controller Master, set speed rate to Fosc / 64
        SPCR |= (1 << SPE) | (1 << MSTR) | (1 << SPR0);

        // SPCR |= (1 << DORD);
        // CPOL - SCK is low when idle, 1 - high when idle
        // CPHA - sample on start of signal, 1 - end of signal (Trailing Edge)
        // SPCR |= | (1 << CPOL) | (1 << CPHA);
        // setting up SPI clock rate
        // SPCR |= (1 << SPR1) | (1 << SPR0);
        // SPSR |= (1 << SPI2X);

        DDRB |= (1 << PB3) | (1 << PB5); // PB3 is MOSI, PB5 is SCK, defined by datasheet
    }

    uint8_t transfer(uint8_t val) {
        SPDR = val; // Set data to SPI Data registyer
        while (!(SPSR & (1 << SPIF))); // wait untl it is passed to a slave
        return SPDR; // get slave's data
    }

    void set_rate(uint32_t rate) {
        uint8_t reg_params;
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
        SPCR |= (reg_params & 0x03);
        SPSR |= (reg_params & 0x01);
    }

    void set_data_order(SpiDataOrder order) {
        SPCR |= ((uint8_t)order << DORD);
    }

    void set_data_mode(SpiDataMode mode) {
        SPCR |= (((mode == SpiDataMode::MODE_2 || mode == SpiDataMode::MODE_3) ? 1 : 0) << CPOL) | (((mode == SpiDataMode::MODE_1 || mode == SpiDataMode::MODE_3) ? 1 : 0) << CPHA);
    }

    void start_transaction(uint32_t rate, SpiDataOrder d_order, SpiDataMode d_mode) {
        old_spsr = SPSR;
        old_spcr = SPCR;

        set_rate(rate);
        set_data_order(d_order);
        set_data_mode(d_mode);
    }
    void end_transaction() {
        SPSR = old_spsr;
        SPCR = old_spcr;
    }
};