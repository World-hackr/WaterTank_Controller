# RF Water Level Logic Plan

This note records the current design discussion for the ATtiny402 RF water tank controller.

## Canonical Decisions

These rules override older experiments and summaries:

```text
No normal idle heartbeat.
TX wakes every 8 seconds to check level, but does not transmit if level is unchanged and nothing important is happening.
On ATtiny402, implement the 8-second wake tick with RTC/PIT sleep wake, not a WDT interrupt. The WDT is reset-oriented on this target and does not expose a normal `WDT_vect` interrupt in the installed device header.
RF lost is only a fault while RX is expecting packets during automatic filling or manual ON.
RX must not enter permanent FAULT_LOCKOUT just because no idle packet arrived.
Level 1 starts motor.
Levels 2 and 3 hold previous motor state.
Level 4 stops motor normally.
Level 5 is overflow warning and also stops motor.
Manual mode is separate from automatic mode.
Manual ON max runtime is 10 minutes on RX.
Current RF security is authentication, not encryption.
Do not add parity unless real testing proves CRC/repeats are insufficient.
```

Do not implement these rejected/obsolete ideas:

```text
Do not send a 72 second normal heartbeat.
Do not use a 45 second global RF-loss timeout while idle.
Do not make level 2 start the motor.
Do not make level 5 the normal full stop level.
Do not use LED 6 only as a fault LED; LED 6 is part of the 6-level bar.
Do not make PUMPING a permanent lockout state.
Do not claim the packet is encrypted unless encryption is actually added.
```

## Hardware Roles

TX board:

- Battery powered.
- Reads tank probes or float switch.
- Sends authenticated RF packets.
- Must sleep most of the time to save battery.

RX board:

- Mains powered through 5 V supply / regulator.
- Receives RF packets.
- Drives relay/buzzer output.
- Displays level/status on 6 charlieplexed LEDs.

## Water Levels

Use 6 logical levels and show all 6 on the RX LED bar:

```text
Level 0 = lowest probe / dry warning zone
Level 1 = normal motor-start level
Level 2 = low / hold previous motor state
Level 3 = middle
Level 4 = full enough / normal motor-stop level
Level 5 = overflow warning level
```

The highest active probe should decide the level. If a higher probe is wet, lower probe state should not matter.

The drive probe should go directly to the bottom/common rod. The remaining sense probes hang at different heights. Only the highest active/wet probe matters for the transmitted level.

The ADC ladder should be calibrated so higher water level gives higher ADC voltage. The highest probe should be near the highest ADC reading because it is closest to a short/highest ladder voltage.

## Motor Logic

Preferred basic motor behavior:

```text
Level 0: dry warning, motor should run if no fault exists
Level 1: normal motor ON/start signal
Level 2: keep previous motor state
Level 3: keep previous motor state
Level 4: normal motor OFF/stop signal
Level 5: overflow warning and motor OFF/stop signal
```

Reasoning:

- Level 1 is the normal low-water motor-start point.
- Level 0 is below the useful range and acts as a dry/critical warning.
- Level 2 and level 3 are hysteresis/hold levels so the relay does not chatter.
- Level 4 is the normal full point where the motor should stop.
- Level 5 is not the normal stop point; it is an overflow warning level. It should also force motor OFF.

When level 5 is detected, TX should send the motor-off/overflow packet aggressively for the next 10 sleep wakes. With an 8 second wake interval, this means about 80 seconds of repeated stop/overflow reporting.

When level 0 is detected, TX should send the dry/start packet for the next 5 sleep wakes. With an 8 second wake interval, this means about 40 seconds of repeated dry/start reporting.

After the first 5 dry packets, if the tank is still dry and unchanged, TX should send one dry retry packet every 20 minutes. This gives the system a recovery path after a motor/RX problem is fixed without spending much battery.

These must be named constants in code so they are easy to change later:

```text
WAKE_TICK_SEC = 8
DRY_REPORT_WAKE_COUNT = 5
DRY_RETRY_WAKE_INTERVAL = 150
OVERFLOW_REPORT_WAKE_COUNT = 10
RF_MISSED_FILL_PACKETS = 3
```

With an 8 second sleep wake:

```text
DRY_RETRY_WAKE_INTERVAL = 150 wakes = 20 minutes
```

Startup behavior:

```text
On TX boot, treat the first measured level as a new level.
If first level is 0, send the dry/start packet for 5 wakes.
After that, if still dry and unchanged, send one dry retry every 20 minutes.
```

## Fault Logic

TX should keep sending packets forever. TX should not stop sending after a fault.

Faults should stop the motor on RX, but packets should continue so RX can display the actual tank state.

Important fault cases:

- RF lost: RX turns motor OFF after timeout.
- Bad authentication/CRC: RX ignores packet.
- Pump damaged or no water supply: motor should not run forever.
- Low TX battery: warn user; final policy still open.

Current safest recovery method with existing hardware:

```text
Motor fault is cleared by power-cycling RX.
```

This avoids automatic restart after a damaged pump or dry source condition.

## TX Battery Strategy

TX must be conservative with power.

Use low-power timed sleep:

```text
Sleep most of the time.
Wake every ~8 seconds.
Read ADC briefly.
Update counters by one wake tick.
Decide whether RF transmit is needed.
Sleep again.
```

Do not keep the MCU awake just to count time. Long timers should be simple counters:

```text
wake_tick = 8 seconds
send_ticks += 1
fill_ticks += 1
no_rise_ticks += 1
stable_ticks += 1
```

Example tick conversions:

```text
10 minutes / 8 seconds = 75 ticks
30 minutes / 8 seconds = 225 ticks
60 minutes / 8 seconds = 450 ticks
80 seconds / 8 seconds = 10 ticks
24 seconds / 8 seconds = 3 ticks
```

## TX RF Send Schedule

RF transmission uses much more power than simple counters, so do not transmit every 8-second wake unless needed.

Suggested schedule:

```text
Wake/check ADC every 8 seconds
Idle unchanged level: do not transmit
Level changed: send immediately
Filling active: send every 8 seconds
Dry level 0: send every wake for 5 wakes after entering dry
Dry level 0 after first 5 wakes: retry once every 20 minutes while still dry
Overflow level 5: send every wake for 10 wakes
Other fault/warning behavior: still open
```

Important battery note: waking every 8 seconds is reasonable. RF transmitting every 8 seconds all year is probably not reasonable with an 800 mAh cell.

Approximate battery limit:

```text
800 mAh / 1 year = about 91 uA average current budget
```

The current robust RF packet uses repeated packets and can keep the RF transmitter active for roughly around 1 second per sent message. If this is sent every 8 seconds all the time, RF duty cycle is too high for a 1 year 800 mAh target. The TX should still wake every 8 seconds to sense the tank, but should only transmit while filling, on level changes, or during dry/overflow/error conditions.

Every sent message should still use repeated packets, because range testing showed this works well:

```text
Bit time: 1000 us
RX sample rate: 8000 Hz
Packet repeats: 4
Gap between repeats: about 35 ms
```

## TX Smart Logic

TX can be smarter than a passive sensor, but it cannot directly know RX relay state.

TX can infer filling behavior from water level changes:

```text
If level <= 1:
  filling is expected
  start or continue fill counters

If level rises:
  update last-progress counter

If level >= 4:
  filling complete
  clear fill warning/fault

If level >= 5:
  overflow warning
  force motor-off flag
  send overflow/stop packet for 10 sleep wakes

If level does not rise for too long:
  set no-progress warning/fault flag

If level does not reach full for too long:
  set fill-timeout fault flag
```

Filling must not keep TX transmitting every 8 seconds forever. If the tank stays at level 1/2/3 too long while filling, both TX and RX should stop treating it as active filling.

Recommended first implementation:

```text
TX starts fill timer when level 1 starts motor.
TX sends filling packets every 8 seconds while fill timer is valid.
RX starts its own fill timer when it turns relay ON from an automatic fill command.

If level reaches 4:
  TX clears filling state
  RX turns relay OFF
  fill was successful

If fill timeout expires before level 4:
  TX stops frequent 8 second filling packets
  TX sends a fill-timeout/fault packet a few times
  RX turns relay OFF
  RX clears active filling state
```

Timeout should be conservative. Do not use the exact learned average as the cutoff, because users may be using water while the tank is filling.

Suggested formula if learned fill time is used in RAM:

```text
DEFAULT_FILL_TIMEOUT = 30 minutes
MIN_FILL_TIMEOUT = 15 minutes
MAX_FILL_TIMEOUT = 60 minutes
LEARN_FACTOR = 2

If no learned fill time exists:
  allowed_fill_time = DEFAULT_FILL_TIMEOUT

If learned fill time exists:
  allowed_fill_time = learned_fill_time * LEARN_FACTOR

allowed_fill_time = max(15 minutes, learned_fill_time * 2)
allowed_fill_time = min(allowed_fill_time, 60 minutes)
```

Example:

```text
If normal learned fill time is 5 minutes:
  allowed timeout = 15 minutes

If learned fill time is 20 minutes:
  allowed timeout = 40 minutes
```

Meaning of the limits:

```text
DEFAULT_FILL_TIMEOUT:
  used before any successful fill has been learned

MIN_FILL_TIMEOUT:
  protects against the timeout becoming too strict after fast fills

MAX_FILL_TIMEOUT:
  hard upper safety limit so the motor cannot run too long
```

Example with a pump that normally fills the tank in 5 minutes:

```text
After many successful cycles, learned_fill_time is about 5 minutes.
Raw calculated timeout = 5 minutes * 2 = 10 minutes.
MIN_FILL_TIMEOUT raises this to 15 minutes.
Actual allowed timeout remains 15 minutes, even after 100 cycles.
```

Manual filling does not update learned fill time.

If timeout happens because many taps are open or pump/source is weak, the motor turns OFF and TX stops frequent sending. The system can recover later when the level changes or when level 1 is reached again and a new filling attempt starts.

Learned fill-time can be added later after real-world data, but the current direction is to avoid EEPROM. EEPROM adds code size and wear-management complexity. For now, use RAM-only learned values or fixed conservative limits.

## TX Manual Control

Manual motor control from the TX/tank side is useful for tank cleaning because the person is already on the roof/top floor. Without this, one person must stay near the tank and another person must operate the ground-floor motor switch.

Use the unused TX `PA2` mode-switch pin as a sealed manual-control button if hardware allows it.

Suggested hardware:

```text
PA2 input with internal pull-up
Sealed push button from PA2 to GND
Button can wake MCU from sleep if pin interrupt is enabled
```

Manual mode is a separate manual-only state, not a normal overlay on automatic mode.

Manual-only entry:

```text
From AUTO:
  button press wakes MCU by pin interrupt
  button must be held for 5 seconds
  if confirmed, TX enters MANUAL_SLEEP mode
  all automatic tank logic stops
  timed wake is disabled
  probe drive is OFF
  sense/ADC is OFF
  only button interrupt remains active
```

First-power activation:

```text
On fresh TX power-up:
  quickly configure the button interrupt
  keep probe drive OFF
  keep timed wake OFF
  enter deep sleep

Button held for 10 seconds:
  mark TX as activated for this power session
  enter normal AUTO startup
  read level and send the first startup packet
```

This prevents a newly powered module from immediately doing tank logic until the user intentionally activates it.

Manual-only operation:

```text
In MANUAL_SLEEP:
  MCU sleeps with no timed wake
  button press wakes MCU
  TX sends MANUAL_TOGGLE
  TX starts 8 second wake cycle

In MANUAL_ON:
  timed wake is enabled for 8 second wake cycle
  TX sends MANUAL_KEEPALIVE every 8 seconds while RX motor should remain ON
  probe drive/sense stay OFF
  automatic level checks stay disabled
  button press wakes/handles command
  next valid button press sends MANUAL_TOGGLE
  TX returns to MANUAL_SLEEP
```

Manual-only exit back to AUTO:

```text
Button held for 5 seconds again:
  exit manual-only mode
  re-enable normal 8-second sleep wake
  re-enable normal probe reading
  treat next measured level as fresh startup level
```

RX behavior for manual commands:

```text
MANUAL_TOGGLE:
  ignore normal water-level features
  if relay is OFF: relay ON and start RX-side 10 minute manual safety timer
  if relay is ON: relay OFF and clear manual ON state

MANUAL_KEEPALIVE:
  accepted only while manual relay is already ON
  refresh/confirm manual ON state

If manual ON timer reaches 10 minutes:
  relay OFF
  clear manual ON state
```

In manual mode, the only safety is RX-side 10 minute maximum runtime. TX does not read probes and does not enforce dry/overflow/fill logic. `MANUAL_SLEEP` has timed wake OFF. `MANUAL_ON` has timed wake ON so it can send the 8 second keepalive. If the user needs more runtime, they can press the button again to send another manual toggle and start another 10 minute window.

Button edge/hold rule:

```text
Button release after a 5 second hold must not also count as a toggle.
Toggle is generated only from a clean short press/release edge while already in manual mode.
5 second hold is reserved for entering/exiting manual-only mode.
10 second hold is reserved for first-power activation.
```

## RX Logic

RX should stay simple because ATtiny402 flash is limited.

Preferred RX behavior:

```text
If RF lost while filling: relay OFF and all LEDs OFF
If packet authentication/CRC fails: ignore packet
If TX fault flag is set: relay OFF
If level <= 1 and no fault: relay ON
If level >= 4: relay OFF
If level is 2 or 3: keep previous relay state
```

RX may also keep a local max motor runtime as a backup safety, but this increases firmware size.

There is no idle heartbeat. If motor is OFF and TX is silent, RX should treat silence as normal.

RF lost only matters while motor/filling is active:

```text
If motor is ON and 3 expected filling packets are missed:
  relay OFF
  all LEDs OFF
```

## Packet Contents

Suggested final packet fields:

```text
message_type
sequence
level
battery_code
flags
auth_tag
```

Possible flags:

```text
MOTOR_SHOULD_RUN
MOTOR_SHOULD_STOP
MANUAL_MODE
MANUAL_TOGGLE
MANUAL_KEEPALIVE
BOOT_FLAG
LEVEL_CHANGED
FILL_ACTIVE
NO_PROGRESS_WARNING
FILL_TIMEOUT_FAULT
DRY_WARNING
OVERFLOW_WARNING
LOW_BATTERY
CRITICAL_BATTERY
```

Authentication:

- Keep 64-bit shared key.
- Keep 64-bit tag if flash allows.
- CRC still needed for packet damage detection.
- Sequence checking helps reject stale/replayed packets during normal operation.
- Do not add simple parity unless testing proves a need. CRC already detects packet damage much better than parity. If stronger noise handling is needed later, use forward error correction or more packet repeats, not only parity.

## LED Display Plan

There are only 6 LEDs, so patterns must be simple and easy to notice.

Recommended meaning:

```text
LED 1-6 = water level bar
```

Normal display:

```text
Level 0: LED 1 slow blink as dry warning
Level 1: LED 1 ON
Level 2: LED 1-2 ON
Level 3: LED 1-3 ON
Level 4: LED 1-4 ON
Level 5: LED 1-6 ON as full bar, with LED 6 blinking as overflow warning
```

Recommended warning patterns:

```text
RF lost while filling: all LEDs OFF, relay OFF
Dry level 0: LED 1 blinking
Overflow level 5: LED 6 blinking opposite of the dry LED 1 pattern
Motor/fill fault: undecided
Low battery: all LEDs slow-blink as an overlay
Normal level: solid bar graph
```

Low battery overlay idea:

```text
Normal level/warning display still runs.
All LEDs also slow-blink around 1 Hz as a background layer.
Dry LED 1 and overflow LED 6 should blink faster or differently enough to remain visible over the low-battery overlay.
```

Priority if multiple conditions happen:

```text
1. Motor/fill fault
2. RF lost
3. Low battery
4. Normal water level
```

LED 6 is now part of the water level bar. Warning patterns must therefore use blink behavior, not a dedicated warning LED.

## Current Open Decisions

Need to decide later:

- Exact TX timed wake implementation details.
- Exact dry retry interval. Current idea: one retry every 20 minutes while still dry.
- Exact filling/error repeated send duration beyond dry/overflow.
- Exact RF lost timeout on RX while filling.
- Whether RX should keep its own motor max-run timer or trust TX fault flags.
- Exact automatic fill timeout formula. Current idea: `max(15 minutes, learned_fill_time * 2)`, capped at 60 minutes.
- Exact low-battery and critical-battery thresholds.
- Whether motor fault clears only by RX power-cycle or also by a specific level pattern.
- Whether learned fill-time should be added after field testing. Current direction: avoid EEPROM and use fixed conservative limits first.
- Final warning pattern for motor fault and low battery now that LED 6 is used as level 5.
- Dry recovery is handled by TX retry: after the first 5 dry packets, TX sends one dry retry about every 20 minutes while still dry.
- Exact TX manual button press edge cases and debounce timing.
- Manual motor runtime is fixed on RX side for now: 10 minutes maximum, then relay OFF.
- First-power activation: require 10 second button hold before AUTO startup begins.

## Current Recommendation

Next firmware direction:

1. Keep the proven RF protocol unchanged.
2. Move long fill/fault timers to TX using 8-second sleep-wake counters.
3. Reduce TX RF transmissions when the level is stable.
4. Keep RX relay logic small and simple.
5. Finalize LED warning patterns before adding more code.
6. Calibrate ADC thresholds using real probe/float readings before trusting motor control.
