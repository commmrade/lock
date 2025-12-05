
class Uart0 {
public:
    Uart0() = default;
    Uart0(unsigned long baud_rate) {
        init(baud_rate);
    }

    void init(unsigned long baud_rate) {
        // FIXME: This does not work for baud rate other than 9600
        const int baud = F_CPU / 16 / baud_rate - 1; // some magic numbers
        
        // Set up baud rate
        UBRR0 |= baud;

        // Enable transmitter TX (pin 0)
        UCSR0B |= (1 << TXEN0) | (1 << RXEN0);
        // 8 bit, parity none, 
        UCSR0C |= (3 << UCSZ00);
    }

    void write(char c) {
        // Check if UDRE0 flag is set in UCSR0A (flag is 1 when buffer is free)
        while (!(UCSR0A & (1 << UDRE0)));
        // Write to buffer (it will then move to shift register, then is gonna be sent to usb)
        UDR0 = c;
    }

    void print(const String& text) {
        for (auto c : text) {
            write(c);
        }
    }
    void println(const String& text) {
        for (auto c : text) {
            write(c);
        }
        write('\n');
    }

    bool available() const {
        // RXC flag is set when there is data to be read
        return (UCSR0A & (1 << RXC0));
    }
    char read() const {
        return UDR0;
    }
};
