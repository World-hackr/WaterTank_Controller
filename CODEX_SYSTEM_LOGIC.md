# Codex Water Tank Firmware Logic

This document describes the settled final logic for the `codex_tx` and `codex_rx` firmware direction.
For the detailed publishable specification, see `WATER_TANK_FIRMWARE_SPEC.md`.

## Hardware

### TX: ATtiny204

- `PA4`: RF data output to STX882.
- `PA6`: common probe drive, active high.
- `PA7`: button input, active low, internal pull-up enabled.
- `PB0`: status LED, active high.
- `PA1`: probe 1, motor-on level.
- `PA2`: probe 2, display-only middle level.
- `PA3`: probe 3, motor-off level.
- `PA5`: probe 4, overflow warning.

### RX: ATtiny402

- `PA1`: RF data input from RXB12/SRX882.
- `PA0`: pairing button input on UPDI pin, active low, internal pull-up enabled.
- `PA2`: relay output.
- `PA6`, `PA7`, `PA3`: six charlieplexed LEDs.

PA0 is used only as an input. Since the MCU is programmed out of socket, using PA0 as the pairing button does not block normal UPDI programming.

## RF Packet

Every transmitted packet is 8 encrypted bytes, repeated four times at the RF layer.

Normal encrypted payload:

- byte 0: message type.
- byte 1: 8-bit sequence counter.
- byte 2: raw probe mask.
- byte 3: raw compressed battery ADC code from TX.
- byte 4: flags, including desired pump state.
- byte 5-7: last three silicon ID bytes of the paired TX.

Probe mask bits:

- bit 0: probe 1.
- bit 1: probe 2.
- bit 2: probe 3.
- bit 3: probe 4 overflow.

Message types:

- `0x01`: automatic level report.
- `0x02`: manual motor on.
- `0x03`: manual motor off.
- `0x05`: manual keep-alive while manual mode is active.

Flags:

- bit 0: desired pump state, `1` means relay should be ON.
- bit 6: dry/level-0 warning.
- bit 7: overflow warning.
- other bits reserved. TX sends reserved bits as `0`; RX ignores reserved bits.

## Pairing

TX sends pairing packets only after power-up, for four wake cycles. A short TX button press cancels the remaining startup pairing and makes TX start normal operation immediately.

RX enters pairing mode when:

- EEPROM has no paired ID, or
- the RX pairing button on `PA0` is held for 5 seconds.

During pairing:

- RX uses the master key only for pairing packets.
- Pairing packet must contain two magic bytes plus six TX silicon ID bytes.
- RX requires the same valid pairing payload twice before saving it.
- RX derives and stores the same unique key as TX.
- RX stores only the last three silicon ID bytes as the normal packet binding ID.
- RX writes an EEPROM valid marker last. If power fails during pairing or clearing, RX rejects the partial EEPROM data and enters pairing instead of booting half-paired.

After changing this firmware, pair again so TX and RX use the same key and packet format.

## TX Logic

On each wake:

1. Read probes by enabling the common drive pin.
2. Check the button.
3. Send the needed packet.
4. Sleep for about 8 seconds, or wake early by button.

Button:

- Short press: cancels startup pairing only.
- Hold for 3 seconds: toggles manual motor command.
- When the 3-second threshold is reached, the TX LED turns on until button release.

Automatic mode:

- TX is the single source of truth for automatic pump state.
- TX sends raw probe mask plus desired pump state.
- TX sends when the raw probe mask changes.
- TX sends when desired pump state changes.
- TX keeps sending every 8-second wake while desired pump is ON.
- TX does not send a normal idle heartbeat while desired pump is OFF.

Automatic pump decision:

- Probe 4 active: desired pump OFF, overflow warning ON.
- Probe 3 active: desired pump OFF.
- Probe 1 active: desired pump ON.
- No probe active: desired pump ON, dry warning ON.
- Probe 2 only: keep previous TX desired pump state.
- On TX reset, desired pump starts OFF.
- Damaged/impossible probe combinations use priority: probe 4 OFF, then probe 3 OFF, then probe 1 ON, then no-probe ON, then probe-2-only hold previous TX desired pump.
- Pump-start decisions while desired pump is OFF use a 3-of-5 wake filter. Pump stop from probe 3/probe 4 is not delayed while desired pump is ON.

Manual mode:

- First 3-second hold sends manual-on.
- Next 3-second hold sends manual-off.
- While manual is on, TX sends manual keep-alive every wake.
- Manual keep-alive means pump ON by itself. RX must not need previous RAM state to understand it.
- TX stops manual locally when the user turns it off or the manual runtime expires.

Status LED on TX:

- On while sending startup pairing packet.
- On after a 3-second manual-button hold, until release.
- Otherwise off.

## RX Logic

RX validates each received packet in this order:

1. RF packet framing and CRC must pass.
2. Decryption must produce a valid pairing packet when pairing, or a valid stored TX ID when not pairing.
3. Sequence must be acceptable enough to reject simple duplicates/replays.
4. RX then obeys the desired pump state or manual message type.

Sequence rule:

- First valid packet after boot is accepted.
- Exact duplicate sequence number is rejected.
- Normal forward sequence is accepted.
- During debugging, sequence resync timeout is 16 seconds.
- In production, sequence resync timeout is 2 minutes.
- After resync timeout, the next valid paired packet can establish a new baseline.
- Manual packets may be accepted more freely so manual control can recover after TX reset.

This is not high-security anti-replay, but it rejects simple duplicate repeats and old packets during normal operation without using EEPROM writes for every packet.

RX automatic relay control:

- RX does not infer pump state from probes.
- RX sets relay from the desired pump bit in the accepted automatic packet.
- Probe data is used for display/warnings.

Manual relay control:

- Manual-on turns relay on.
- Manual-off turns relay off.
- Manual keep-alive turns relay on.
- Manual state is not permanently latched in RX. It stays alive only while valid manual packets continue arriving.

Timeouts:

- If relay is on and no valid packet arrives for 24 seconds, relay turns off.
- Timeout stops are not latched. New valid automatic or manual packets can start operation again.

## RX LED Logic

Pairing mode:

- LEDs chase in sequence.

No valid packet yet:

- LED6 slow blink. This means RX is already paired and waiting for the first valid normal packet.

Normal level display:

- No probe active: LED1 fast blink for dry warning.
- Probe 1 active: LED2 on.
- Probe 2 active: LED3 on.
- Probe 3 active: LED4 on.
- Probe 4 active: LED5 fast blink for overflow.

RX display shows actual received probe bits, not inferred lower levels. This makes damaged probes and float-switch wiring visible.

Battery:

- LED6 steady on when TX battery code is above the low-battery threshold.
- LED6 slow blink when low battery is detected.
- RX uses the raw compressed ADC code from TX. It does not calculate decimal voltage.

## Intentional Simplifications

- TX owns pump-state decisions, but avoids complex dirty-probe/fault inference unless flash allows.
- RX does not latch faults or reconstruct relay state. It returns to normal when valid packets resume.
- Overflow and dry are warnings in manual mode; communication timeout still turns relay OFF if packets stop.
- EEPROM is used for pairing data only, not for every sequence counter update.
- Replay protection is practical lightweight filtering, not cryptographic freshness against an active attacker.
