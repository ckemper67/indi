# Orion SteadyStar USB Protocol Reference

This document provides a summary of the low-level USB interface and command structures for the Orion SteadyStar Adaptive Optics (AO) unit, based on reverse-engineered Python implementations.

## Device Identification
*   **Vendor ID (VID):** `0x03EB` (Atmel Corp.)
*   **Product ID (PID):** `0x2013`
*   **Interface:** 0
*   **Endpoints:**
    *   **Write:** `0x02` (Interrupt)
    *   **Read:** `0x81` (Interrupt)
*   **Packet Size:** 8 bytes

## Hardware & Firmware Context
The Orion SteadyStar AO unit is built around an **Atmel AVR USB** microcontroller (likely **ATmega32U4** or **AT90USB** series). 
*   **Firmware Stack:** It utilizes the **LUFA (Lightweight USB Framework for AVRs)** stack. 
*   **USB Profile:** The PID `0x2013` is a standard default for the LUFA **Generic HID** demo application, which allows for raw 8-byte data transfer via HID reports. 
*   **Communication Mode:** While it identifies as an HID device, it is accessed as a raw USB device using interrupt transfers in this implementation.

---

## Command Structure

All commands are 8-byte arrays sent via `interruptWrite` to endpoint `0x02`. Responses are 8-byte arrays retrieved via `interruptRead` from endpoint `0x81`.

### 1. Position & Status Queries

#### Get Motor Positions
Requests the current step position of the four internal motors (`m1`, `m2`, `m3`, `m4`).
*   **Command Sequence:**
    1.  `30000000c7dd421a`
    2.  `3916421ac7dd421a`
    3.  `62000000c7dd421a`
*   **Response:** 8 bytes where `result[0...3]` correspond to `m1`, `m2`, `m3`, `m4` in 2's complement format.

#### Get Homing Status
Queries the limit switches to determine if the motors are at their home positions.
*   **Command:** `34000000c7dd421a`
*   **Response:** `result[0]` contains a bitmask of the limit switches:
    *   `0x10`: Motor 1 limit
    *   `0x20`: Motor 2 limit
    *   `0x40`: Motor 3 limit
    *   `0x80`: Motor 4 limit
    *   `0x00`: All limits clear (Homed)

---

### 2. Movement Commands

#### Move Individual Motor
Moves a specific motor by a relative delta.
*   **Structure:** `[0x61, delta, motor_idx, 0x00, 0xc7, 0xdd, 0x42, 0x1a]`
    *   `delta`: Relative steps (8-bit 2's complement).
    *   `motor_idx`: `0` to `3`.
*   **Note:** Positive delta typically moves towards the limit, negative away.

#### Set XY Position
Moves the AO unit to a specific (X, Y) coordinate.
*   **Structure:** `[0x32, x_byte, y_byte, 0x00, 0x81, 0x7a, 0x90, 0x90]`
    *   `x_byte`, `y_byte`: Coordinates in 2's complement format.

#### Rotate
Rotates the AO unit to a specified angle.
*   **Command Sequence:**
    1.  Write: `672474038fb02795`
    2.  Read: 8 bytes (Handshake/Status)
    3.  Write Rotation: `[0x6b, low_byte, high_byte, 0x0d, low_byte, high_byte, 0x00, 0x00]`
        *   `low_byte`, `high_byte`: 16-bit integer calculated as `int(angle * 29)`.
    4.  Poll Loop (approx. 1110 iterations):
        *   Write: `6875f90538db760b`
        *   Read: 8 bytes
        *   Sleep: `0.01s`

---

## Motor Configuration & Coordinate Mapping

### Hardware Layout
The four motors are arranged as follows:
*   **m1:** 2 o'clock
*   **m2:** 5 o'clock
*   **m3:** 7 o'clock
*   **m4:** 10 o'clock

### Coordinate Mapping (XY to Motor)
To move the unit to an `(tx, ty)` target:
1.  Calculate base offsets: `ddx = tx // 2`, `ddy = ty // 2`.
2.  Set targets:
    *   `m1 = tx - ddx`
    *   `m2 = ty - ddy`
    *   `m3 = 0 - ddx`
    *   `m4 = 0 - ddy`

### Motor to XY (Inverse)
*   `x = m1 - m3`
*   `y = m2 - m4`

## Initialization & Safety
1.  **Homing:** Always perform a homing sequence on connection by moving motors until `GetHomingStatus` returns `0x00`.
2.  **Clipping:** Software should limit motor positions to a safe range (e.g., `-80` to `80` steps) to prevent mechanical binding.
