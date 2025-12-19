# Arduino RFID Lock

This project implements an RFID-based lock system using an Arduino-compatible microcontroller, an MFRC522 RFID reader, and basic digital output for lock/unlock indication. The code is written in C++ and is designed for embedded environments.

## Features

- **RFID Authentication:** Uses the MFRC522 module to read RFID cards/tags.
- **Custom Key Verification:** Only cards with a specific UID (defined in code) will unlock the system.
- **Lock/Unlock Indication:** Uses digital output pins to indicate locked/unlocked status (e.g., for LEDs or relays).
- **Low-level Hardware Control:** Direct manipulation of SPI, UART, and MFRC522 registers for maximum control and efficiency.

## File Overview

- `lock.ino`: Main Arduino sketch. Handles initialization, main loop, and lock logic.
- `mfrc.h`: MFRC522 RFID reader driver. Handles communication, card detection, UID extraction, and protocol details.
- `spi.h`: SPI bus driver. Provides low-level SPI communication routines.
- `uart.h`: UART driver. Provides basic serial communication routines for debugging or external communication.
- `LICENSE`: Project license.

## Hardware Connections

- **MFRC522 Connections:** Connect the MFRC522 module to the Arduino using SPI (see your board's pinout).
- **Lock/Unlock Outputs:** Connect LEDs, relays, or other indicators to the pins defined as `UNLOCKED_PIN` and `LOCKED_PIN` in `lock.ino`.
- **Serial Debugging:** Serial output is available at 9600 baud for debugging.

## Usage

1. **Wiring:** Connect the MFRC522 and output indicators as described above.
2. **Upload:** Flash the `lock.ino` sketch to your Arduino.
3. **Power Up:** Open the serial monitor at 9600 baud to view debug output.
4. **Operation:** Present an RFID card. If the UID matches the `VERIFIED_KEY` in the code, the unlock indicator will activate; otherwise, the lock indicator will activate.

## Customization

- **Change the Verified Key:** Edit the `VERIFIED_KEY` array in `lock.ino` to match your desired RFID card/tag UID.
- **Adjust Pins:** Change `SS_PIN`, `RST_PIN`, `UNLOCKED_PIN`, and `LOCKED_PIN` as needed for your hardware setup.

## Dependencies

- Arduino core libraries.
- No external libraries required; all drivers are implemented in the project.

## License

See `LICENSE` for details.
