# Special Firmware Ideas

This folder is for useful future ideas that are not part of the current product firmware. Some may be valuable later, but they should not be implemented until the basic deterministic firmware is stable and flash space is known.

## Dynamic Pump-Off Timing

Future idea:

```text
measure normal time from pump ON to probe 3/full
use that to choose a smarter pump timeout
turn pump OFF if filling takes too long
recover on the next valid low-level cycle
```

Possible parameters:

```text
default fill timeout
minimum allowed timeout
maximum allowed timeout
safety multiplier over learned fill time
number of successful fills before trusting learned value
whether learned value is RAM-only or stored in EEPROM
```

Main concern:

```text
water may be used while filling, so timeout must not be too aggressive
EEPROM writes add code and wear-management complexity
```

Current decision:

```text
do not implement yet
stabilize fixed deterministic firmware first
```

## Reset-Cause Diagnostics

Future idea:

```text
read MCU reset cause at startup
show a debug LED pattern or store temporary debug state
```

Possible reset causes:

```text
power-on reset
brown-out reset
watchdog reset
external reset
UPDI/programming reset
```

Why it may help:

```text
debug random field resets
separate brownout problems from firmware crashes
understand connector/power behavior during testing
```

Current decision:

```text
do not implement in product firmware now
flash memory is more important
use only in a special debug build if needed
```

## Stronger Protocol Versioning

Current approach:

```text
packet remains 8 encrypted bytes
reserved flag bits must be sent as 0 by TX
reserved flag bits must be ignored by RX
breaking changes should use a new message type or pairing/protocol marker
```

Future idea:

```text
add explicit protocol version only if packet format must change
```

Main concern:

```text
extra version byte does not fit cleanly in the current 8-byte packet
```

## Stronger Replay Protection

Future idea:

```text
use persistent rolling counter or challenge-response style freshness
```

Main concern:

```text
EEPROM writes per packet are not acceptable
extra protocol machinery costs flash
current lightweight sequence check is enough for this use case
```

## Advanced Probe Diagnostics

Future idea:

```text
detect impossible probe combinations
warn about damaged lower probes
distinguish float-switch mode from probe mode
```

Current decision:

```text
do not add special logic now
display raw probe bits so the user/technician can see damaged probes directly
control priority already lets the system keep working in many damaged-probe cases
```

