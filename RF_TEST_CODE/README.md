# RF Test Code Workspace

This is the clean RF development area for the ATtiny402 water tank project.

## Current Files

- `11_auth_standalone_tx` / `11_auth_standalone_rx`
  - Stable authenticated RF counter for range testing.
  - Sends a sequence number about every 800 ms.
  - RX displays the last valid sequence number in binary on the 6 charlieplexed LEDs.

- `12_water_level_tx` / `12_water_level_rx`
  - Current product-route water-level firmware prototype.
  - TX reads the probe ladder through `PA7` drive and `PA6` sense.
  - TX uses 9 ADC samples and a median filter before reporting the level.
  - TX is event-based: no idle heartbeat; it sends on level change, while filling, dry retry, overflow repeat, and manual commands.
  - RX authenticates the packet, checks sequence/boot handling, displays level, and drives the relay on `PA2`.
  - Relay behavior: level 1 starts motor, levels 2-3 hold previous state, level 4 stops motor, level 5 is overflow/off.
  - Dry level 0 sends 5 startup packets, then one retry about every 20 minutes while still dry.
  - Manual TX mode uses PA2 button, `MANUAL_TOGGLE`, 8-second keepalive while ON, and RX-side 10-minute max runtime.

- `cli`
  - Small Arduino CLI wrappers for compile, upload, and port listing.

## Hardware Assumptions

TX board:

- `PA6` / physical pin 2: probe sense ADC
- `PA7` / physical pin 3: probe drive
- `PA1` / physical pin 4: STX882 DATA
- `PA2` / physical pin 5: sealed manual button input
- `PA3` / physical pin 7: test LED
- `PA0` / physical pin 6: UPDI, leave free

RX board:

- `PA6` / physical pin 2: charlieplex line A
- `PA7` / physical pin 3: charlieplex line B
- `PA1` / physical pin 4: RXB12 or SRX882 DATA
- `PA2` / physical pin 5: relay/buzzer driver
- `PA3` / physical pin 7: charlieplex line C
- `PA0` / physical pin 6: UPDI, leave free

Corrected LED mapping:

- LED 1: `A HIGH`, `B LOW`
- LED 2: `B HIGH`, `A LOW`
- LED 3: `B HIGH`, `C LOW`
- LED 4: `C HIGH`, `B LOW`
- LED 5: `A HIGH`, `C LOW`
- LED 6: `C HIGH`, `A LOW`

## Current Plan

1. Use `11_auth_standalone_*` for range and antenna testing.
2. Use `12_water_level_*` as the active product-route prototype.
3. Calibrate the TX ADC thresholds from real probe and float-switch measurements.
4. Replace delay-based wake simulation with real watchdog/pin-interrupt sleep code when hardware behavior is confirmed.
5. Add battery reporting and final low-battery LED policy after the water-level/manual logic is proven.

Use `cli/README.md` for Arduino CLI commands.
