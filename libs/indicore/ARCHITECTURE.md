# Architecture: Pluggable Coordinate Engine

## Overview

`libastro` (`libs/indicore/libastro.cpp`) is the single entry point for all coordinate math in INDI. It has been refactored from a direct libnova wrapper into a **factory dispatcher** that routes work to one of two independent engine hierarchies.

The stellar engine (frame rotation: precession, nutation, aberration) and the planetary engine (solar system ephemeris) are selected separately. Any combination is valid — a driver can use ERFA for stars and libnova for planets, or the full EPH stack, without either engine knowing about the other.

---

## Interface Layer

Defined in `libs/indicore/CoordinateEngine.h`.

### `ICoordinateEngine` — Stellar Frame Rotation

```cpp
class ICoordinateEngine {
    virtual void J2000toObserved(IEquatorialCoordinates *j2000, double jd,
                                 IEquatorialCoordinates *jnow) = 0;
    virtual void ObservedToJ2000(IEquatorialCoordinates *jnow, double jd,
                                 IEquatorialCoordinates *j2000) = 0;
    virtual void EquatorialToHorizontal(IEquatorialCoordinates *object,
                                        IGeographicCoordinates *observer,
                                        double jd,
                                        IHorizontalCoordinates *position) = 0;
};
```

### `IPlanetaryEngine` — Solar System Ephemeris

```cpp
class IPlanetaryEngine {
    virtual void GetPlanetObserved(int np, double jd,
                                   IEquatorialCoordinates *observed) = 0;
};
```

Planet codes: 1=Mercury, 2=Venus, 3=Moon, 4=Mars, 5=Jupiter, 6=Saturn, 7=Uranus, 8=Neptune, else=Sun.

---

## Engine Implementations

### Stellar Engines (`libs/indicore/ErfaEngine.cpp`, `LibnovaEngine.cpp`)

| Class | Backend | Nutation | Precision | Status |
|-------|---------|----------|-----------|--------|
| `LibnovaStellarEngine` | libnova | IAU 1980 | ~15" | Fallback |
| `ErfaEngine2000A` | ERFA `eraAtci13` / `eraApci13` | IAU 2000A (1,365 terms) | ~0.1 mas | Available |
| `ErfaEngine2000B` | ERFA `eraAtci00b` / `eraApci00b` | IAU 2000B (77 terms) | ~1.0 mas | Default |

**ERFA-2000B wrapper design**: `eraApci00b` mirrors `eraApci13` exactly, substituting two calls:
- `eraPnm06a` → `eraPnm00b` (77-term nutation matrix)
- `eraS06` → `eraS00` (CIO locator for IAU 2000)

All other steps (`eraBpn2xy`, `eraApci`, `eraEors`) are identical. This gives the `ObservedToJ2000` / `J2000toObserved` pair a single consistent ASTROM context with a correctly computed equation of origins.

`eraApco00b` extends `eraApci00b` with observer-local setup by calling `eraApio13` after the geocentric step. `eraApio13` is nutation-model-independent — it only computes ERA, polar motion, and the TIO locator s' (`eraSp00`), none of which depend on the 2000A/B choice. This keeps `EquatorialToHorizontal` on a pure 2000B code path.

**2000B model purity audit** — the following 2000A identifiers must not appear anywhere in the 2000B code path:

| Identifier | Role | Present in 2000B? |
|---|---|---|
| `eraPnm06a` / `eraNut06a` | 1365-term nutation matrix | No |
| `eraS06` | CIO locator for IAU 2006 | No |
| `eraApci13` | geocentric ASTROM (2000A) | No |
| `eraApco13` | observer ASTROM (2000A) | No |
| `eraAtci13` | ICRS→CIRS (2000A) | No |

All leaf functions called by the 2000B path (`eraEpv00`, `eraBpn2xy`, `eraApci`, `eraEors`, `eraAtciq`, `eraAticq`, `eraApio13`, `eraAtioq`) are model-agnostic — they operate on pre-computed BPN matrices or pre-built ASTROM contexts.

**Time input**: JD is treated as UTC and split into a two-part Julian Date per ERFA convention (`utc1 = floor(jd) + 0.5`, `utc2 = jd - utc1`).

**`EquatorialToHorizontal`**: uses `eraApco00b` + `eraAtioq`. The observer's longitude and latitude are used to compute local sidereal time and hour angle; no topocentric parallax is applied (geocentric CIRS input). Full topocentric support is planned for M5.

### Planetary Engines (`libs/indicore/EphEngine.cpp`, `LibnovaEngine.cpp`)

The EPH library is authored by Patrick Wallace (also the original author of SLALIB and co-author of ERFA/SOFA). It implements VSOP2010 planetary theory and the ELP/MPP02 lunar theory, distributed as pre-compiled binary `.ctx` data files. See https://tpointsw.uk/eph.htm.

The library is released under an ISC-style license: permission is granted to use, copy, modify, and distribute for any purpose with or without fee. Copyright (C) P.T.Wallace. All rights reserved.


| Class | Backend | Theory | Precision | Status |
|-------|---------|--------|-----------|--------|
| `LibnovaPlanetaryEngine` | libnova | VSOP87 | ~1000" | Fallback |
| `EphEngineFull` | EPH library | VSOP2010 (full) | ~0.73" geocentric | Available |
| `EphEngineINDI` | EPH library | VSOP2010 (truncated, 2.6 MB) | +0.003" vs EPH_FULL | Available |

**Time conversion**: UTC → TAI → TT via `eraUtctai` + `eraTaitt`. TT is used as TDB (TDB-TT < 2ms geocentric). This corrects the ~69s UT1/TDB discrepancy present in the original implementation.

**Context loading**: `EphEngineFull` holds the `.ctx` contexts as member variables. The Earth/EMB context and Moon context are loaded once on first use; the planet context is reloaded when `np` changes. Load failures are detected from `ephPlanc`/`ephMoonc` return values and cause an early return.

**`EphEngineINDI`**: loads `.ictx` packed files via `ephPlanci()`, falling back to `ephPlanc()` (`.ctx`) if `.ictx` files are absent. The packer (`tools/indi_eph_packer`) applies a time-weighted amplitude filter: threshold $10^{-9}$ AU with `max_tm=0.2` (±200 yr from J2000), producing a 2.6 MB dataset across 8 planets. Validated delta vs `EphEngineFull`: 0.003" for Mars at JD 2459019.833333. Both engines agree with JPL DE440 geocentric to ~0.25".

---

## Dispatch Layer

`libs/indicore/libastro.cpp` manages engine lifetime and exposes the public API.

```
INDI::setStellarEngine(StellarEngine::ERFA_2000B)   // independent selections
INDI::setPlanetaryEngine(PlanetaryEngine::EPH_FULL)
```

Engines are lazily constructed on first use and destroyed on engine change. A single mutex guards all reads and writes to the engine pointers, making the dispatch layer safe for concurrent driver use.

The public API (`J2000toObserved`, `ObservedToJ2000`, `EquatorialToHorizontal`, `GetPlanetObserved`, `HorizontalToEquatorial`) is unchanged from the original libnova-based `libastro` — callers require no modification.

---

## Validation

All accuracy claims are verified against external sources of truth. The test suite is never self-referential.

| Domain | Reference | Rationale |
|--------|-----------|-----------|
| Stars | IMCCE / SIMBAD-CDS | Standard for stellar catalogs |
| Planets | JPL Horizons (DE440) | Standard for solar system bodies |

The split provides cross-validation between two independent ephemeris models (INPOP21 vs DE440). At INDI's target accuracy, the two models agree to well under 0.01" for inner planets over modern epochs.

### Verified results

| Engine | Test | Error vs truth |
|--------|------|----------------|
| libnova stellar | Deneb apparent position | 15.7" |
| ERFA-2000A | Deneb apparent position | 0.050" |
| ERFA-2000B | Deneb apparent position | 0.050" |
| ERFA A vs B delta | — | 0.000264" |
| libnova planetary | Mars geocentric (2020) | 1008" |
| EPH-Full | Mars geocentric (2020) | 0.26" vs DE440 |
| EPH-Full | Moon geocentric (2020) | 0.25" vs DE440 |
| EPH-INDI vs EPH-Full | Mars geocentric (2020) | 0.003" |

Test binaries: `test/core/test_engine_comparison`, `test/core/test_eph_library`.

---

## File Map

| File | Role |
|------|------|
| `libs/indicore/libastro.h` | Public API: coordinate functions + engine selection enums |
| `libs/indicore/libastro.cpp` | Factory dispatcher, mutex-guarded engine lifecycle |
| `libs/indicore/CoordinateEngine.h` | `ICoordinateEngine`, `IPlanetaryEngine`, factory function declarations |
| `libs/indicore/ErfaEngine.cpp` | `ErfaEngine2000A`, `ErfaEngine2000B`, `eraApci00b`/`eraAtci00b` wrappers |
| `libs/indicore/LibnovaEngine.cpp` | `LibnovaStellarEngine`, `LibnovaPlanetaryEngine` |
| `libs/indicore/EphEngine.cpp` | `EphEngineFull`, `EphEngineINDI` |
| `libs/indicore/eph/` | EPH library source and `.ctx` data files (gitignored) |

---

## Planned Extensions

**M5 — Topocentric Promotion**: Introduce `INDI::ObservationContext` mapping to `eraASTROM`. Replace `eraApci*` with `eraApco*` context builders in `EquatorialToHorizontal`, enabling diurnal aberration, polar motion, and topocentric parallax when observer location is available.

**M5 — Topocentric Promotion**: Introduce `INDI::ObservationContext` mapping to `eraASTROM`. Replace `eraApci*` with `eraApco*` context builders in `EquatorialToHorizontal`, enabling diurnal aberration, polar motion, and topocentric parallax when observer location is available.
