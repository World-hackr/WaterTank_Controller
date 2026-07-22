# RF Water Level Logic Plan

This is the final high-level architecture for the tank-side TX and relay/display-side RX firmware.
For the detailed publishable specification, see `WATER_TANK_FIRMWARE_SPEC.md`.

## Core Principle

The TX is the controller. The RX is the executor.

```text
TX:
  read probes
  filter readings if flash allows
  decide desired pump state
  transmit complete state

RX:
  verify RF packet
  verify paired identity/key
  verify sequence enough to reject simple repeats
  obey the desired pump state in the packet
  turn relay OFF on communication timeout
```

RX must not reconstruct pump state from old probe history. After reset, RX starts fresh except for paired identity/key loaded from EEPROM.

## Hardware

### TX: ATtiny204

- `PA4`: RF data output to STX882.
- `PA6`: common probe drive, active high.
- `PA7`: button input, active low, internal pull-up.
- `PB0`: status LED, active high.
- `PA1`: probe 1, motor-start/low level.
- `PA2`: probe 2, middle/display level.
- `PA3`: probe 3, motor-stop/full level.
- `PA5`: probe 4, overflow warning level.

### RX: ATtiny402

- `PA1`: RF data input from RXB12/SRX882.
- `PA0`: pairing button input on UPDI pin, active low, internal pull-up.
- `PA2`: relay output.
- `PA6`, `PA7`, `PA3`: six charlieplexed LEDs.

## Packet Contents

Keep the encrypted payload at 8 bytes.

```text
byte 0: message type
byte 1: 8-bit sequence counter
byte 2: raw probe mask
byte 3: raw compressed TX battery ADC code
byte 4: flags
byte 5: TX ID byte 0
byte 6: TX ID byte 1
byte 7: TX ID byte 2
```

Probe mask:

```text
bit 0: probe 1
bit 1: probe 2
bit 2: probe 3
bit 3: probe 4 overflow
```

Flags:

```text
bit 0: desired pump state, 1 = relay should be ON
bit 6: dry/level-0 warning
bit 7: overflow warning
```

Other flag bits remain reserved. TX must send reserved bits as 0, and RX must ignore reserved bits.

Message types:

```text
0x01: automatic state report
0x02: manual motor ON
0x03: manual motor OFF
0x05: manual keep-alive, meaning manual pump still requested ON
```

## TX Automatic Logic

TX owns the pump decision.

On TX boot:

```text
desiredPump = OFF
manual = OFF
sequence starts from 0
```

Probe decision:

```text
if probe 4 active:
  desiredPump = OFF
  overflow warning = ON
else if probe 3 active:
  desiredPump = OFF
else if probe 1 active:
  desiredPump = ON
else if no probe active:
  desiredPump = ON
  dry warning = ON
else if only probe 2 active:
  keep previous TX desiredPump
```

This keeps probe 1 meaningful while still making TX reset fail safe. If TX resets at probe-2-only, it assumes pump OFF until water falls to probe 1 or level 0.

For damaged/impossible probe combinations, control priority is probe 4 OFF, then probe 3 OFF, then probe 1 ON, then no-probe ON, then probe-2-only hold previous TX desired pump.

TX sends automatic packet when:

- raw probe mask changes,
- desired pump state changes,
- desired pump is ON, in which case it repeats every 8-second wake so RX can keep relay ON and recover after RX reset.

No normal idle heartbeat is required while desired pump is OFF.

## Probe Filtering

Probe filtering is required unless final TX flash size proves it impossible.

Current filter:

```text
Only filter pump-start decisions while desiredPump is OFF.
Use a 5-wake window.
If at least 3 of 5 wakes request pump start, desiredPump becomes ON.
```

Pump-start readings are probe 1 active or no probe active. Pump stop from probe 3/probe 4 is not delayed while desiredPump is ON.

If memory becomes too tight, reduce or remove the filter first. Dirty probe flicker mainly causes extra transmissions and slightly more battery use.

## TX Manual Logic

TX button is active low.

```text
short press during startup pairing:
  cancel remaining pairing attempts

hold about 3 seconds:
  toggle manual requested state
```

Manual ON:

- TX sends `MSG_MANUAL_ON`.
- TX keeps sending `MSG_MANUAL_KEEP` every 8-second wake.
- `MSG_MANUAL_KEEP` means pump ON by itself. RX must not need old RAM state to understand it.

Manual OFF:

- TX sends `MSG_MANUAL_OFF`.
- TX returns to automatic logic.

TX should stop manual keep-alive after 10 minutes, unless the user turns manual off earlier. RX also has communication timeout protection.

## RX Boot And Reset Logic

On RX boot:

```text
relay OFF
manual state OFF
no current level
no old relay memory
load paired ID/key from EEPROM
if not paired, enter pairing
if paired, wait for next valid packet
```

RX EEPROM should store only:

- paired TX identity,
- derived key,
- optional pairing-valid marker/configuration.

RX must not remember previous relay state, manual state, level, timeout state, or fault state across reset.

## RX Packet Handling

RX validates each packet:

```text
RF framing/CRC must pass
decrypt with current key
paired ID must match
sequence must be acceptable
then apply message
```

Automatic packet:

```text
relay = desired pump bit from flags
display = probe mask/warnings from packet
```

Manual packet:

```text
MSG_MANUAL_ON   -> relay ON
MSG_MANUAL_KEEP -> relay ON
MSG_MANUAL_OFF  -> relay OFF
```

RX must not latch manual mode permanently. Manual stays alive only because valid manual packets continue arriving.

## Sequence Handling

For debugging:

```text
SEQUENCE_RESYNC_MS = 16000
```

For production:

```text
SEQUENCE_RESYNC_MS = 120000
```

Basic rule:

```text
first valid packet after RX boot is accepted
exact duplicate sequence is rejected
normal forward sequence is accepted
after resync timeout, next valid paired packet can establish a new baseline
manual packets may be accepted more freely because manual recovery after TX reset is more important than strict replay filtering
```

This is lightweight replay protection, not high-security cryptographic freshness. That is acceptable for this water tank controller.

## Communication Loss

If relay is ON and no valid packet arrives for the lost-link timeout:

```text
relay OFF
show communication failure indication
wait for next valid packet
```

No permanent lockout. The next valid packet decides again.

Suggested timeout: about 24 seconds, equal to three missed 8-second packets.

If relay is already OFF and TX is silent, silence is normal. RX may keep showing the last valid display instead of clearing to unknown.

## Pairing

Keep the current pairing design:

- TX sends startup pairing packets a limited number of times.
- RX enters pairing when EEPROM is unpaired or PA0 pairing button is held.
- Pairing packet uses master key.
- Normal packets use derived unique key.
- RX binds to the last three TX silicon ID bytes.
- Pairing mode always forces relay OFF.

Startup pairing should not be endless. If pairing fails, installation/debugging should fix the cause.

## RX LEDs

Use simple, visible meanings:

```text
pairing: chasing LEDs
paired but no valid packet yet: waiting indication
level 0/no probe: LED1 fast blink
probe 1 active: LED2 ON
probe 2 active: LED3 ON
probe 3 active: LED4 ON
probe 4 active/overflow: LED5 fast blink
battery: LED6 steady when OK, slow blink when low
communication lost while relay was expected ON: relay OFF and visible lost-link indication
```

Display shows actual probe bits, not inferred lower levels. This makes damaged probes and float-switch wiring obvious.

## Safety Rules

Default during uncertainty:

```text
pump OFF
```

Relay must be OFF during:

- RX boot before valid command,
- pairing mode,
- RF timeout,
- invalid packet,
- automatic packet whose desired pump bit is OFF,
- manual OFF packet.

Overflow and low battery should be clearly displayed. Overflow also makes TX request pump OFF in automatic mode. Manual mode is user-controlled, but communication timeout still turns relay OFF if packets stop.

## Test Checklist

Communication and reset:

- Reset TX at probe 1.
- Reset TX at probe 2.
- Reset RX while pump ON.
- Reset RX while pump OFF.
- Remove TX battery.
- Restore TX battery.
- Remove RX power.
- Restore RX power.
- Verify sequence wrap from 255 to 0.

RF:

- Heavy RF interference.
- Long communication loss.
- Packet corruption.
- Wrong transmitter.

Sensors:

- No probe active.
- Probe 1 active.
- Probe 2 active only.
- Probe 3 active.
- Probe 4 active.
- Disconnect each probe.
- Short each probe.
- Dirty/flickering probe.
- Water exactly between two probes.

Manual:

- Manual ON.
- Manual OFF.
- TX reset during manual.
- RX reset during manual.
- Communication loss during manual.
- Manual 10-minute timeout.

Pairing:

- Normal pairing.
- RX powered before TX.
- TX powered before RX.
- Incorrect TX nearby.
- Reset during startup pairing.

Battery:

- Low TX battery indication.
- Nearly discharged TX battery.
- RX power brownout/reset recovery.
