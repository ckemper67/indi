# Orion SteadyStar Driver Architecture

## Reference Material

This driver is based on Benoit Schillings' reverse engineering of the Orion
SteadyStar protocol and its Python implementation
(https://github.com/BenoitSchillings/orion_steadystar). The primary source
files are:

- `orion_ao.py` -- core AO control and USB protocol
- `move_ao.py` -- CLI utility for XY positioning
- `rotate.py` -- CLI utility for rotation control

USB captures used during analysis:

- `captures/reset_zero.txt` -- homing sequence
- `captures/move_up.txt`, `captures/move_right.txt` -- axis movement
- `captures/rotate+10.txt` -- rotation command
- `captures/up_mult_seq_of_2.txt` -- multi-step movement

---

## Overview

The driver is split into two classes:

- **OrionSteadyStarHardware** -- low-level USB layer (libusb-1.0). Owns the
  device handle, mutex, and all wire encoding. No INDI dependency.
- **OrionSteadyStarDriver** -- INDI interface layer. Owns all properties,
  timer loop, and derotation state. Delegates every hardware operation to
  an `OrionSteadyStarHardware` member.

This separation lets the hardware class be tested or reused without INDI, and
keeps the driver class free of USB details.

---

## USB Device

| Field       | Value                                |
|-------------|--------------------------------------|
| Vendor ID   | 0x03EB (Atmel Corp.)                 |
| Product ID  | 0x2013 (LUFA Generic HID default)    |
| Interface   | 0                                    |
| Write EP    | 0x02 (interrupt)                     |
| Read EP     | 0x81 (interrupt)                     |
| Packet size | 8 bytes                              |

The device is an Atmel AVR running the LUFA Generic HID firmware. Although it
enumerates as HID, it is accessed as a raw USB device via interrupt transfers.

### Platform notes

**Linux:** The udev rule `99-orion_steadystar.rules` sets `MODE="0666"` and
`TAG+="uaccess"` so the device is accessible without root. After installing,
run `sudo udevadm control --reload-rules && sudo udevadm trigger` and replug.
`libusb_set_auto_detach_kernel_driver` handles detaching `hid-generic` at
claim time.

**macOS:** `IOHIDFamily` claims the device on plug-in and hides it from
non-root libusb enumeration. Run `sudo indiserver -v indi_orion_steadystar`
if the driver fails to open the device.

### Connection retry

`Connect()` enumerates the USB bus with `libusb_get_device_list` (rather than
the silent `libusb_open_device_with_vid_pid`) so each failure logs the exact
`libusb_error_name`. On macOS, IOHIDFamily can briefly re-attach after another
process releases the interface. The driver retries up to 5 times with a 1 s
delay, reinitializing the libusb context between attempts to get a fresh bus
view.

---

## Wire Protocol

All commands are 8-byte arrays. Integers use little-endian byte order.
Motor deltas use sign-magnitude encoding (bit 7 = sign, bits 0-6 = magnitude),
implemented in `c2_byte()`.

### Position query

Requests current step positions of the four internal motors.

```
Write: 30 00 00 00 c7 dd 42 1a
Write: 39 16 42 1a c7 dd 42 1a
Write: 62 00 00 00 c7 dd 42 1a
Read:  result[0..3] = m1, m2, m3, m4 (two's complement signed bytes)
```

### Homing status

Queries limit switches.

```
Write:  34 00 00 00 c7 dd 42 1a
Read:   result[0] bitmask:
          0x10 = motor 1 at limit
          0x20 = motor 2 at limit
          0x40 = motor 3 at limit
          0x80 = motor 4 at limit
          0x00 = all clear
```

### Move individual motor

Moves one motor by a relative delta.

```
Write: 61 <delta> <motor_idx> 00 c7 dd 42 1a
         delta:     sign-magnitude encoded relative steps
         motor_idx: 0..3
```

### Rotate to angle

Angle is expressed in firmware steps: `iangle = int(angle_deg * 29) % 10440`.

```
Write: 67 27 74 03 8f b0 27 95      (init handshake)
Read:  8 bytes                       (status, ignored)
Write: 6b <lo> <hi> 0d <lo> <hi> 00 00   (lo/hi = iangle & 0xFF, iangle >> 8)
```

### Poll rotation status

```
Write: 68 75 f9 05 38 db 76 0b
Read:  8 bytes -- any non-zero byte means rotation still in progress
```

### Query absolute rotator position

```
Write: 70 00 00 00 00 00 00 20
Read:  result[0] | (result[1] << 8) = position in firmware steps
       angle_deg = steps / 29.0
```

---

## Motor Layout and Coordinate Mapping

The four motors are arranged at 2, 5, 7, and 10 o'clock on the mirror mount:

```
      m1 (2 o'clock)
  m4 (10)        m2 (5)
      m3 (7 o'clock)
```

### XY to motors (SetAO)

Given a target `(tx, ty)` in steps:

```
ddx = tx / 2    (integer division)
ddy = ty / 2

target_m1 = tx - ddx   (X axis, positive half)
target_m2 = ty - ddy   (Y axis, positive half)
target_m3 =    - ddx   (X axis, negative half)
target_m4 =    - ddy   (Y axis, negative half)
```

Each motor is clamped to [-80, 80] steps. The driver sends only the delta from
the last known position, not the absolute target.

### Motors to XY (inverse)

```
x = m1 - m3
y = m2 - m4
```

---

## Homing Sequence

On connection, `Home()` mirrors the Python reference implementation:

1. Read current motor positions; replay them as deltas (firmware counter sync).
2. Wait 200 ms.
3. Read positions and homing status.
4. Loop until all limit bits clear (or 200 iterations): move each motor by -7
   if its limit bit is set, or +2 if clear.
5. Read positions; replay as deltas. Wait 1 s.
6. Read final positions to confirm settled state.

---

## INDI Driver Layer

### Interfaces

- `INDI::DefaultDevice` -- base device lifecycle
- `INDI::GuiderInterface` -- GuideNorth/South/East/West (ms-based pulses)
- `INDI::RotatorInterface` -- optional; enabled/disabled at runtime via the
  "Rotator support" switch without requiring reconnect

### Guiding

`GuideNorth/South/East/West(ms)` convert milliseconds to steps using `ms / 10`
(matching the sxao reference driver) and call `AONorth/South/East/West()`.
The accumulated `(m_ax, m_ay)` position is displayed as the absolute AO offset.

### Software derotation

When the rotator interface is enabled, the driver supports continuous software
derotation:

- **Rate property** (`ROTATOR_DEROTATE`, arcsec/sec): sets the continuous drift
  rate. On rate change, the current theoretical position is computed using the
  old rate and stored as the new epoch start, so position is continuous.
- **Guide property** (`ROTATOR_GUIDE`, degrees): accumulates an offset added to
  the theoretical position. Reset to zero after each update.
- **TimerHit dispatch**: every 100 ms, the driver computes:
  ```
  theoretical = range360(start_angle + guide_offset + elapsed_sec * rate / 3600)
  ```
  Converts to firmware steps (29 steps/degree, modulo 10440). If the quantized
  step differs from the current hardware position, issues `StartRotation()`.

Rate changes do not cause a position jump because the epoch is recalculated
before the new rate takes effect.

### Timer loop (100 ms)

1. **Watchdog** (every 5 s): calls `GetHomingStatus`. On failure, attempts
   auto-reconnect; disconnects if reconnect fails.
2. **Rotator poll**: polls hardware position when a Goto is in progress, or
   every 2 s when idle (keeps KStars position display current).
3. **Derotation dispatch**: computes theoretical angle and issues
   `StartRotation()` if the quantized step differs from the current position.
