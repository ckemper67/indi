# Orion SteadyStar INDI Driver

This directory contains the native C++ INDI driver for the Orion SteadyStar Adaptive Optics (AO) unit.

## Credits

This implementation is based on the reverse-engineered protocol and Python prototype work by **Benoit Schillings**, available at:
https://github.com/BenoitSchillings/orion_steadystar

## Features

* **AO Interface**: Precise tilt control in North/South and East/West axes.
* **Guider Interface**: Standard pulse guiding support.
* **Rotator Interface**: Optional absolute position rotation with support for:
    * **Sync**: Align software angle with physical position.
    * **Reverse**: Toggle rotation direction.
    * **Abort**: Stop ongoing rotation.
* **Configurable**: Rotator functionality can be enabled or disabled via the "Rotator support" switch in the Options tab, accommodating devices with or without the rotation hardware.
* **Homing**: Automatic homing sequence on connection to clear limit switches.
* **Simulation**: Robust built-in simulation for all interfaces. The rotator simulation includes shortest-path movement logic and 360-degree range wrapping.

## Files

* `orion_steadystar_driver.cpp/h`: INDI driver interface and logic.
* `orion_steadystar_hardware.cpp/h`: Low-level USB communication using libusb-1.0 and simulation engine.
* `protocol.md`: Technical reference for the reverse-engineered USB protocol.
