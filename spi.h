class spi {
public:
    spi() = default;
    void init() {
        // SPI Enable and make this controller Master, set speed rate to Fosc / 64
        SPCR |= (1 << SPE) | (1 << MSTR) | (1 << SPR1);

        // CPOL - SCK is low when idle, 1 - high when idle
        // CPHA - sample on start of signal, 1 - end of signal (Trailing Edge)
        // SPCR |= | (1 << CPOL) | (1 << CPHA);
        // setting up SPI clock rate
        // SPCR |= (1 << SPR1) | (1 << SPR0);
        // SPSR |= (1 << SPI2X);

        DDRB |= (1 << PB3) | (1 << PB5); // PB3 is MOSI, PB5 is SCK, defined by datasheet
    }
};