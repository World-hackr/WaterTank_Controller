# Automatic Water Tank Controller Firmware Specification

This document is the detailed firmware specification for the RF automatic water tank controller.

It records the current agreed design decisions, timings, packet format, reset behavior, pairing behavior, button behavior, LED behavior, and safety philosophy.

## 1. System Overview

The system has two modules:

- TX module: battery-powered tank-side controller.
- RX module: powered receiver/relay/display unit.

The final architecture is:

```text
Water probes -> TX decision logic -> encrypted RF packet -> RX verification -> relay output
```

The TX is the single source of truth for pump state.

The RX must not infer whether the pump should be ON or OFF from probe history. It only verifies packets and obeys the desired pump state sent by the TX.

## 2. Main Design Philosophy

### 2.1 TX Owns Control

The TX:

- reads probes,
- optionally filters probe readings,
- decides desired pump state,
- sends raw probe mask,
- sends desired pump state,
- sends battery information,
- sends manual commands.

### 2.2 RX Executes

The RX:

- verifies RF framing and CRC,
- decrypts packet,
- verifies paired TX identity,
- applies sequence rules,
- sets relay from the packet,
- displays level/warnings,
- turns relay OFF on communication timeout.

The RX does not preserve automatic relay state across reset.

### 2.3 Fail-Safe Default

During uncertainty, pump must be OFF.

Uncertainty includes:

- RX boot before first valid packet,
- pairing mode,
- RF timeout,
- invalid packet,
- unpaired state,
- manual packets stopped,
- TX reset before it has decided pump ON again.

## 3. Hardware Pin Map

### 3.1 TX: ATtiny204

```text
PA4: RF data output to STX882
PA6: common probe drive, active HIGH
PA7: button input, active LOW, internal pull-up
PB0: status LED, active HIGH
PA1: probe 1, motor-start/low level
PA2: probe 2, middle/display level
PA3: probe 3, motor-stop/full level
PA5: probe 4, overflow warning level
```

### 3.2 RX: ATtiny402

```text
PA1: RF data input from RXB12/SRX882
PA0: pairing button input on UPDI pin, active LOW, internal pull-up
PA2: relay output
PA6: charlieplex LED line A
PA7: charlieplex LED line B
PA3: charlieplex LED line C
```

PA0 is used as RX pairing button input only. Since the MCU is programmed out of socket, using PA0 as GPIO does not block the user's programming flow.

## 4. Water Probe Meaning

The TX sends a raw 4-bit probe mask.

```text
bit 0: probe 1 active
bit 1: probe 2 active
bit 2: probe 3 active
bit 3: probe 4 active
```

Probe roles:

```text
probe 1: motor-start/low level
probe 2: middle/display level
probe 3: motor-stop/full level
probe 4: overflow warning level
```

No probe active means level 0 / very low / dry warning region.

Control uses priority rules, so high probes can stop the pump even if lower probes are damaged. Display does not fake lower probes; it shows actual received probe bits so damaged probes and float-switch behavior are visible.

The TX should not spend flash on complex dirty-probe reasoning unless memory allows. The primary correction is that TX decides pump state and sends that state explicitly.

## 5. Automatic Pump Decision In TX

TX keeps a RAM variable:

```text
desiredPump
```

On TX reset:

```text
desiredPump = OFF
```

This is intentional and fail-safe. If TX resets while water is at probe 2 only, the pump stays OFF until water drops to probe 1 or level 0.

Automatic decision rule:

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
  dry/level-0 warning = ON

else if only probe 2 active:
  keep previous TX desiredPump
```

For impossible or damaged-probe combinations, control priority is:

```text
probe 4 has highest priority and requests pump OFF
probe 3 requests pump OFF
probe 1 requests pump ON
no probe active requests pump ON
probe 2 alone holds previous TX desiredPump
```

This means probe 3 or probe 4 still stops the pump even if probe 1 or probe 2 is damaged.

This keeps probe 1 meaningful as the normal start threshold while also allowing deterministic recovery and degraded operation:

- RX reset does not matter because TX keeps sending desired pump state while pump is expected ON.
- TX reset at probe 2 only becomes pump OFF, which is safe.
- TX reset at probe 1 or level 0 becomes pump ON again.
- If probe 1 is damaged, level 0/no-probe can still start the pump.
- If probe 3 is damaged, probe 4 can still act as a backup stop/overflow condition if it is installed with enough margin.
- Probe 2 is not required for basic motor operation.

## 6. TX Transmission Schedule

TX wakes about every 8 seconds.

On each wake:

```text
read probes
update desiredPump
handle button
send packet if required
sleep again
```

TX sends an automatic packet when:

- raw probe mask changes,
- desired pump state changes,
- desired pump is ON.

When desired pump is ON, TX sends every 8-second wake. This lets RX keep the relay ON and recover after RX reset.

When desired pump is OFF and probe state is unchanged, TX does not send a normal idle heartbeat.

Startup pairing:

```text
TX sends startup pairing packet for 4 wake cycles.
Short TX button press cancels remaining startup pairing.
```

## 7. Manual Mode

TX button:

```text
active LOW
internal pull-up enabled
```

Button behavior:

```text
short press during startup pairing:
  cancel remaining startup pairing

hold about 3 seconds:
  toggle manual requested state
```

Manual ON:

```text
TX sends MSG_MANUAL_ON
TX keeps sending MSG_MANUAL_KEEP every 8-second wake
```

Manual OFF:

```text
TX sends MSG_MANUAL_OFF
TX returns to automatic operation
```

Important RX rule:

```text
MSG_MANUAL_ON   means relay ON
MSG_MANUAL_KEEP means relay ON
MSG_MANUAL_OFF  means relay OFF
```

`MSG_MANUAL_KEEP` must be meaningful by itself. RX must not need old RAM state to understand it. Therefore RX reset during manual ON is acceptable: the next valid `MSG_MANUAL_KEEP` turns relay ON again.

Manual mode must stop when packets stop:

```text
manual packets stop -> RX communication timeout -> relay OFF
```

TX should also stop manual keep-alive after about 10 minutes unless the user turns it off earlier.

## 8. RF Packet Format

Encrypted payload length remains 8 bytes.

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

Message types:

```text
0x01: automatic state report
0x02: manual motor ON
0x03: manual motor OFF
0x05: manual keep-alive, meaning pump ON
```

Flags byte:

```text
bit 0: desired pump state, 1 = relay ON requested
bit 6: dry/level-0 warning
bit 7: overflow warning
```

Reserved flag bits:

```text
bit 1: reserved
bit 2: reserved
bit 3: reserved
bit 4: reserved
bit 5: reserved
```

Compatibility rule:

```text
TX must transmit reserved bits as 0.
RX must ignore all reserved bits.
Breaking protocol changes must use a new message type or pairing/protocol marker, not reinterpret old reserved bits in a way that breaks old RX behavior.
```

Battery byte:

```text
raw compressed ADC code from TX
```

RX should compare this raw code against thresholds. It does not need decimal voltage conversion.

## 9. RF Physical Protocol

Current RF link assumptions:

```text
ASK/OOK RF modules: STX882 TX, RXB12/SRX882 RX
bit time: 1000 us
RX sample rate: 8000 Hz
packet repeats per message: 4
gap between repeats: about 35 ms
```

RF packet handling:

- repeated packets improve reception,
- CRC detects packet corruption,
- encryption/key/ID validates sender,
- sequence number rejects simple duplicate repeats.

## 10. Pairing

Pairing design:

```text
TX sends pairing packets on startup for limited attempts.
RX enters pairing if unpaired or pairing button is held.
Pairing uses master key.
Normal operation uses derived unique key.
RX binds to last three TX silicon ID bytes.
```

Pairing packet:

```text
byte 0: pairing magic byte 0
byte 1: pairing magic byte 1
byte 2-7: six TX silicon ID bytes
```

Pairing safety:

- RX relay is always OFF in pairing mode.
- RX accepts pairing only after seeing the same valid pairing payload twice.
- RX may use an EEPROM valid marker written last, so interrupted pairing writes do not create a half-paired state.

Pairing button:

```text
RX PA0 active LOW
hold about 5 seconds to clear pairing and enter pairing mode
```

## 11. EEPROM Policy

TX:

- no EEPROM needed for normal pump state,
- after reset, TX behaves fresh.

RX:

EEPROM may store:

- paired TX ID,
- derived unique key,
- pairing-valid marker,
- future configuration values if needed.

RX must not store across reset:

- previous relay state,
- previous manual state,
- previous automatic state,
- previous level,
- temporary communication fault,
- timeout state.

EEPROM wear is not a major concern for pairing because pairing is rare. Avoid unnecessary writes, but do not avoid EEPROM where it makes pairing robust.

## 12. RX Boot Behavior

On RX boot:

```text
relay OFF
manual state OFF
auto state empty
current level unknown
sequence history empty
load paired ID/key from EEPROM
if paired: wait for next valid packet
if unpaired: enter pairing mode
```

RX after reset is like a fresh receiver that already knows the paired TX identity.

## 13. RX Relay Behavior

Automatic packet:

```text
relay = desired pump state bit
```

Manual packet:

```text
MSG_MANUAL_ON   -> relay ON
MSG_MANUAL_KEEP -> relay ON
MSG_MANUAL_OFF  -> relay OFF
```

Timeout:

```text
if relay is ON and no valid packet arrives for about 24 seconds:
  relay OFF
```

Timeout is temporary. The next valid packet decides again.

Pairing:

```text
if RX is in pairing mode:
  relay OFF
```

Invalid packets:

```text
bad CRC -> ignore
bad decrypt/ID -> ignore
bad sequence -> ignore or wait for resync timeout
```

Ignored packets must not change relay state.

## 14. Sequence Handling

Sequence counter:

```text
8-bit counter
wraps from 255 to 0
```

RX sequence behavior:

```text
first valid packet after RX boot is accepted
exact duplicate sequence is rejected
normal forward sequence is accepted
after resync timeout, next valid paired packet can establish new baseline
manual packets may be accepted more freely for reset recovery
```

Debug timing:

```text
SEQUENCE_RESYNC_MS = 16000 ms
```

Production timing:

```text
SEQUENCE_RESYNC_MS = 120000 ms
```

Reason:

- 16 seconds makes debugging quick.
- 2 minutes makes simple replay less useful in the final product.

This is practical lightweight replay protection, not high-security anti-replay.

## 15. Communication Loss

If packets stop while relay is ON:

```text
after about 24 seconds:
  relay OFF
  show communication-loss indication
```

Suggested timing:

```text
TX expected ON packet interval: 8 seconds
RX lost-link timeout: 24 seconds
missed packets tolerated: 3
```

If relay is already OFF and TX is silent, silence is normal.

During normal pump-OFF silence, RX may keep showing the last valid received level/battery display. It should show the waiting indication only after boot/pairing before any valid normal packet has been received.

No permanent lockout is created by communication loss.

## 16. RX LED Indications

RX has six charlieplexed LEDs.

Pairing:

```text
LEDs chase in sequence
```

Paired but no valid normal packet yet:

```text
waiting indication
```

Normal level display:

```text
level 0 / no probe active:
  LED1 fast blink

probe 1 active:
  LED2 ON

probe 2 active:
  LED3 ON

probe 3 active:
  LED4 ON

probe 4 active / overflow:
  LED5 fast blink
```

The RX display shows actual probe bits, not inferred lower levels. Example: if probe 3 is active but probe 1 and probe 2 are not active, only the probe 3 display LED is shown. This makes damaged probes easier to diagnose and allows ordinary float switches to be used without special firmware.

Battery:

```text
LED6 steady ON when TX battery is OK
LED6 slow blink when TX battery is low
```

Communication loss:

```text
relay OFF
show a visible lost-link indication
exact LED pattern may be finalized later
```

## 17. Battery Handling

TX measures battery using internal reference and ADC.

TX sends raw compressed ADC code.

RX handles warning display using raw code threshold.

No decimal voltage calculation is required inside RX.

Low-battery policy:

- warn user using LED6 slow blink,
- do not necessarily force relay OFF only because battery is low,
- exact thresholds can be calibrated later.

## 18. Probe Filtering

Probe filtering is required unless final TX flash size proves it impossible.

Current filter scope:

```text
only used while desiredPump is OFF
only used before allowing a new pump-start decision
```

Current method:

```text
TX wakes every about 8 seconds
if reading requests pump start, count it as a start vote
use a 5-wake window
if at least 3 of those 5 wakes request pump start, desiredPump becomes ON
```

Pump-start readings:

```text
probe 1 active
or no probe active / level 0
```

Pump stop is not delayed by this slow filter:

```text
while desiredPump is ON:
  probe 3 active -> desiredPump OFF on that wake
  probe 4 active -> desiredPump OFF on that wake
```

This avoids false starts during idle, but keeps pump stop responsive while filling. Raw probe mask changes may still be transmitted for display/diagnosis before the pump-start filter has accepted a new ON decision.

If flash becomes too tight, this is one of the first features allowed to be reduced or removed.

Dirty/flickering probes are not expected to create dangerous behavior after TX owns the desired pump state. Worst case is extra transmissions and slightly reduced battery life.

## 19. Dynamic Pump-Off Timing

This section is reserved for later.

Current confirmed approach:

```text
do not add dynamic pump-off timing yet
do not add learned fill-time EEPROM writes yet
first stabilize fixed deterministic behavior
```

Future dynamic pump-off timing notes are kept in `special/SPECIAL_FIRMWARE_IDEAS.md`.

Exact dynamic timing values are not finalized yet.

## 20. Timing Summary

Current agreed timings:

```text
TX wake interval: about 8 seconds
TX packet repeats: 4
RF packet gap: about 35 ms
RF bit time: 1000 us
RX sample rate: 8000 Hz
TX manual hold: about 3 seconds
RX pairing hold: about 5 seconds
RX communication timeout while relay ON: about 24 seconds
manual maximum runtime target: about 10 minutes
debug sequence resync: 16 seconds
production sequence resync: 2 minutes
startup pairing attempts: 4 wake cycles
```

Future/not finalized:

```text
dynamic pump-off timing
exact low-battery threshold
exact communication-loss LED pattern
exact probe-filter threshold if field testing suggests changing 3-of-5
```

## 21. Test Checklist

### Reset And Recovery

- RX reset while pump should be ON.
- RX reset while pump should be OFF.
- TX reset at no-probe/level 0.
- TX reset at probe 1.
- TX reset at probe 2 only.
- TX reset at probe 3.
- TX reset at probe 4.
- Both modules reset together.
- Remove and restore TX battery.
- Remove and restore RX power.

### Automatic Control

- No probe active should request pump ON.
- Probe 1 should request pump ON.
- Probe 2 only should keep TX previous desired pump state.
- Probe 3 should request pump OFF.
- Probe 4 should request pump OFF and overflow warning.
- Pump ON state should transmit every 8 seconds.
- Pump OFF stable idle should stop transmitting.

### Manual Control

- Hold TX button for manual ON.
- Confirm RX turns relay ON from `MSG_MANUAL_ON`.
- Confirm RX turns relay ON from `MSG_MANUAL_KEEP` after RX reset.
- Confirm manual packets stop after manual timeout.
- Confirm RX turns relay OFF when manual packets stop.
- Confirm manual OFF turns relay OFF.

### Pairing

- Fresh RX enters pairing.
- RX pairing button hold enters pairing.
- RX relay stays OFF during pairing.
- Correct TX pairs successfully.
- Wrong TX does not control RX after pairing.
- Power loss during pairing does not create unsafe relay state.

### RF Robustness

- One corrupted packet does not change relay.
- Duplicate packet is rejected.
- Sequence wrap 255 to 0 works.
- TX reset sequence recovery works after configured resync timeout.
- RF loss while relay ON turns relay OFF after timeout.

### Display

- Pairing chase visible.
- Waiting-for-packet indication visible.
- Dry/level 0 LED1 fast blink.
- Probe 1 lights only the probe 1 display LED.
- Probe 2 lights only the probe 2 display LED.
- Probe 3 lights only the probe 3 display LED.
- Broken lower probe is visible when a higher probe is active alone.
- Overflow LED5 fast blink.
- Battery OK LED6 steady.
- Low battery LED6 slow blink.

## 22. Non-Goals

These are intentionally not part of the current firmware:

- RX reconstructing automatic pump state from probe history.
- Permanent fault lockout that requires user reset.
- EEPROM writes for every sequence counter.
- Full cryptographic anti-replay with persistent rolling counter.
- Dynamic pump-off timing before basic firmware is stable.
- Complex dirty-probe diagnosis in TX unless flash remains available.
