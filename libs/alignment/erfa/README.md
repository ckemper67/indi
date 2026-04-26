# Vendored ERFA subset for the SPK Math Plugin

## Why vendor instead of requiring liberfa?

The Wallace SPK kernel (`spk/vtel.c`, `spk/pmfit.c`) was written against the
SOFA/ERFA API.  Rather than adding `liberfa` as a new system dependency for
INDI, this directory vendors the small subset of ERFA functions that are
actually reachable at runtime when the plugin operates in its normal mode
(`tar.sys = APPT`, pressure = 0).

Only 19 functions are needed.  The rest of ERFA — in particular the full ICRS
precession/nutation pipeline (`eraApco13`, `eraAtciqz`, `eraAticq`) — is
either unreachable via `tar.sys = APPT` or replaced by thin wrappers in
`erfa_libnova.c` that call libnova, which INDI already depends on.

## File provenance

All `.c` and `.h` files except `erfa_libnova.c` are copied verbatim from the
ERFA 2.0.1 release:

    https://github.com/liberfa/erfa/releases/tag/v2.0.1

ERFA is a BSD-licensed fork of the IAU SOFA library with identical mathematics.
The files are Copyright (C) 2013–2023, NumFOCUS Foundation, and are reproduced
here under the terms of the three-clause BSD licence reproduced at the bottom of
each source file.

The subset included is:

| File | Purpose |
|------|---------|
| `anp.c`, `anpm.c` | Angle normalization to [0,2π) and (−π,π] |
| `s2c.c`, `c2s.c` | Spherical ↔ Cartesian unit-vector conversion |
| `cp.c`, `cr.c`, `tr.c` | Vector/matrix copy and transpose |
| `ir.c`, `rx.c`, `ry.c`, `rz.c` | Rotation matrices about X/Y/Z axes |
| `rxp.c`, `trxp.c` | Matrix–vector multiply and transpose–multiply |
| `pas.c` | Position-angle between two directions |
| `hd2ae.c`, `ae2hd.c` | Hour-angle/Dec ↔ Az/El conversion |
| `era00.c` | Earth Rotation Angle (IAU 2000 model) |
| `atioq.c` | CIRS → observed place (Az/El, HA/Dec) |
| `atoiq.c` | Observed place → CIRS (inverse of atioq) |
| `erfa.h`, `erfam.h` | Public API and internal constants |

## erfa_libnova.c

`erfa_libnova.c` is original INDI code (not derived from ERFA or SOFA).  It
provides the four `eraASTROM`-related functions that the linker requires but
that are not in the subset above:

- `eraAper`, `eraAper13` — update the Local Earth Rotation Angle field.
- `eraAtciqz`, `eraAticq` — ICRS ↔ CIRS transforms; provided as pass-through
  stubs because `SPKMathPlugin` always uses `tar.sys = APPT` (apparent
  coordinates), so these paths are never executed.

`SPKMathPlugin::UpdateAstrometry` populates `eraASTROM` directly using
`eraEra00` and libnova's `ln_get_apparent_sidereal_time`, so `eraApco13` is
not needed and is not included.

## Obtaining the complete ERFA library

The full ERFA source is available at:

    https://github.com/liberfa/erfa

On Debian/Ubuntu: `apt install liberfa-dev`
On Fedora/RHEL:   `dnf install erfa-devel`
On macOS:         `brew install erfa`
