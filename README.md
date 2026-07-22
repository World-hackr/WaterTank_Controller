# WaterTank_Controller

ATtiny402 water tank controller firmware and RF test code.

Current route:

- STX882 transmitter module on the tank-side TX board.
- RXB12/SRX882 receiver module on the display/relay RX board.
- Authenticated ASK/OOK RF packet link.
- TX-side pump-state decision included in every normal packet.
- RX-side relay execution with no hidden auto/manual latch across reset.
- Probe display with dry and overflow handling.
- Manual tank-side control mode using the TX button input.

See `WATER_TANK_FIRMWARE_SPEC.md` for the detailed publishable firmware specification.
See `RF_WATER_LEVEL_LOGIC_PLAN.md` for the shorter architecture plan.
