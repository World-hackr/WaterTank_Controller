# WaterTank_Controller

ATtiny402 water tank controller firmware and RF test code.

Current route:

- STX882 transmitter module on the tank-side TX board.
- RXB12/SRX882 receiver module on the display/relay RX board.
- Authenticated ASK/OOK RF packet link.
- Six-level probe display with dry and overflow handling.
- Manual tank-side control mode using the TX button input.

See `RF_WATER_LEVEL_LOGIC_PLAN.md` for the current design decisions.
