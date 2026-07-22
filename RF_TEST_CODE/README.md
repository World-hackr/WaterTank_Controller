# RF Test Code Workspace

This is the clean RF development area for the ATtiny402 water tank project.

These sketches are older RF/prototype experiments. The final product logic is documented in `../RF_WATER_LEVEL_LOGIC_PLAN.md`; if anything here conflicts with that document, treat this file as historical test-code notes.

## Current Files

- `11_auth_standalone_tx` / `11_auth_standalone_rx`
  - Stable authenticated RF counter for range testing.
  - Sends a sequence number about every 800 ms.
  - RX displays the last valid sequence number in binary on the 6 charlieplexed LEDs.

- `12_water_level_tx` / `12_water_level_rx`
  - Legacy product-route water-level firmware prototype.
  - TX reads the probe ladder through `PA7` drive and `PA6` sense.
  - TX uses 9 ADC samples and a median filter before reporting the level.
  - This does not describe the final TX/RX architecture.
  - Final approach: TX sends desired pump state in every normal packet; RX obeys that state and does not infer/hold automatic relay history.
  - Final manual approach: `MANUAL_ON` and `MANUAL_KEEP` both mean relay ON; `MANUAL_OFF` means relay OFF.

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
2. Treat `12_water_level_*` as a legacy prototype, not the final source of truth.
3. Calibrate the TX ADC thresholds from real probe and float-switch measurements.
4. Replace delay-based wake simulation with real watchdog/pin-interrupt sleep code when hardware behavior is confirmed.
5. Add battery reporting and final low-battery LED policy after the water-level/manual logic is proven.
6. Use `../RF_WATER_LEVEL_LOGIC_PLAN.md` for the final architecture before changing product firmware.

Use `cli/README.md` for Arduino CLI commands.
