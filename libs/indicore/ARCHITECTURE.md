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
    virtual void J2000toGeocentric(const J2000Coordinates *j2000, double jd,
                                   GeocentricApparent *out) = 0;
    virtual void J2000toGeocentricFull(const CatalogStar *star, double jd,
                                       GeocentricApparent *out) = 0;
    virtual void J2000toTopocentric(const J2000Coordinates *j2000,
                                    AstrometricContext &ctx, double jd,
                                    TopocentricApparent *out) = 0;
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

## Coordinate Type Hierarchy

`libastro.h` defines zero-cost subtypes of `IEquatorialCoordinates` that carry frame information at compile time:

| Type | Frame | Produced by |
|---|---|---|
| `J2000Coordinates` | ICRS / J2000.0 catalog | caller |
| `GeocentricApparent` | Geocentric CIRS apparent | `J2000toGeocentric`, `J2000toObserved` |
| `TopocentricApparent` | Topocentric CIRS apparent | `J2000toTopocentric` |

`CatalogStar` extends `J2000Coordinates` with proper motion, parallax, and radial velocity for use with `J2000toGeocentricFull`:

```cpp
struct CatalogStar : J2000Coordinates {
    double mu_ra_masyr;    // mu_alpha * cos(dec) in mas/yr
    double mu_dec_masyr;   // mu_delta in mas/yr
    double parallax_mas;   // annual parallax in mas
    double radial_vel_kms; // radial velocity in km/s (+ve receding)
};
```

`J2000toGeocentric` and `J2000toObserved` pass zero for all catalog parameters (pm=px=rv=0) and expect the caller to pre-propagate proper motion if needed. `J2000toGeocentricFull` passes all parameters to `eraAtciq` so that annual parallax and proper-motion propagation are handled by ERFA internally.

---

## Engine Implementations

### Stellar Engines (`libs/indicore/ErfaEngine.cpp`, `LibnovaEngine.cpp`)

| Class | Backend | Nutation | Precision | Status |
|-------|---------|----------|-----------|--------|
| `LibnovaStellarEngine` | libnova | IAU 1980 | ~15" | Fallback |
| `ErfaEngine2000A` | ERFA `eraAtci13` / `eraApci13` | IAU 2000A (1,365 terms) | ~0.02" | Available |
| `ErfaEngine2000B` | ERFA `eraAtci00b` / `eraApci00b` | IAU 2000B (77 terms) | ~0.02" | Default |

**Time input**: all public functions receive JD as UTC. The engines convert UTC → TAI → TT via `eraUtctai` + `eraTaitt` before passing to any ERFA function that requires TT (nutation, ephemeris, BPN matrix). Functions that take UTC directly (`eraApco13`, `eraApco00b`, `eraApio13`) receive the UTC split unchanged.

**ERFA-2000B wrapper design**: `eraApci00b` mirrors `eraApci13` exactly, substituting:
- `eraPnm06a` → `eraPnm00b` (77-term nutation matrix)
- `eraS06` → `eraS00` (CIO locator for IAU 2000)

`eraApco00b` mirrors `eraApco13`: it computes the Earth rotation angle (`eraEra00`), TIO locator (`eraSp00`), refraction constants (`eraRefco`), and then calls the bare `eraApco` combinator. `eraApco` uses `eraPvtob` to compute the observer's geocentric position and velocity and passes them to `eraApcs`, which sets `astrom->v` to the observer's full barycentric velocity (annual + diurnal component). This is required for `eraAtciq` to apply diurnal aberration correctly when computing topocentric coordinates.

**`EquatorialToHorizontal`**: uses `eraApco13`/`eraApco00b` + `eraAtioq`. Input is geocentric CIRS (from `J2000toObserved`); `eraAtioq` rotates to the local horizontal frame using the observer's ERA and latitude.

**`J2000toTopocentric`**: uses the cached `AstrometricContext` built by `ensureAstrom2000A`/`ensureAstrom2000B` (both call `eraApco13`/`eraApco00b` with the observer location), then `eraAtciq`. Because `eraApco*` sets `astrom->v` to the observer's velocity including diurnal rotation, `eraAtciq` applies both annual and diurnal aberration, giving topocentric CIRS output. The geo-topo delta for a zero-parallax source equals diurnal aberration only (~0.16" at mid-latitudes).

### Planetary Engines (`libs/indicore/EphEngine.cpp`, `LibnovaEngine.cpp`)

The EPH library is authored by Patrick Wallace (also the original author of SLALIB and co-author of ERFA/SOFA). It implements VSOP2013 planetary theory and the ELP/MPP02 lunar theory, distributed as pre-compiled binary `.ctx` data files. See https://tpointsw.uk/eph.htm.

The library is released under an ISC-style license: permission is granted to use, copy, modify, and distribute for any purpose with or without fee. Copyright (C) P.T.Wallace. All rights reserved.

| Class | Backend | Theory | Precision | Status |
|-------|---------|--------|-----------|--------|
| `LibnovaPlanetaryEngine` | libnova | VSOP87 | ~1000" | Fallback |
| `EphEngineFull` | EPH library | VSOP2013 (full) | ~0.73" geocentric | Available |
| `EphEngineINDI` | EPH library | VSOP2013 (truncated, 2.6 MB) | +0.003" vs EPH_FULL | Default |

**Time conversion**: UTC → TAI → TT via `eraUtctai` + `eraTaitt`. TT is used as TDB (TDB-TT < 2ms geocentric). This corrects the ~69s UT1/TDB discrepancy present in the original implementation.

**Context loading**: `EphEngineFull` holds the `.ctx` contexts as member variables. The Earth/EMB context and Moon context are loaded once on first use; the planet context is reloaded when `np` changes.

**`EphEngineINDI`**: loads `.ictx` packed files via `ephPlanci()`, falling back to `ephPlanc()` (`.ctx`) if `.ictx` files are absent. The packer (`tools/indi_eph_packer`) applies a time-weighted amplitude filter: threshold $10^{-9}$ AU with `max_tm=0.2` (±200 yr from J2000), producing a 2.6 MB dataset across 8 planets.

### Generating EPH data files

The `.ctx` and `.ictx` files in `libs/indicore/eph/` are derived data — they
are not in version control and must be generated before the first build.

**Step 1 — Build the tools**

```bash
cd build
cmake ..
make eph_plan_bin indi_eph_packer
```

**Step 2 — Generate `.ctx` binary context files**

`eph_plan_bin` reads the VSOP2013 ASCII data files (`VSOP2013p1.dat` …
`VSOP2013p8.dat`) from the current working directory and writes
`VSOP2013_1.ctx` … `VSOP2013_8.ctx`. Download the ASCII files from:

    https://ftp.imcce.fr/pub/ephem/planets/vsop2013/solution/

```bash
/path/to/build/libs/indicore/eph/eph_plan_bin
```

The Moon context files (`ELP_MPP02_JPL.ctx`, `ELP_MPP02_LLR.ctx`) are
generated separately using the `moon_bin` tool from the upstream EPH
distribution.

**Step 3 — Pack `.ictx` files (for `EphEngineINDI`)**

```bash
mkdir -p /tmp/ictx_out
./build/tools/indi_eph_packer \
    /path/to/indi/libs/indicore/eph \
    /tmp/ictx_out \
    1e-9 0.2
cp /tmp/ictx_out/VSOP2013_*.ictx /path/to/indi/libs/indicore/eph/
```

**Step 4 — Verify**

```bash
cd build
./test/core/test_eph_library
./test/core/test_engine_comparison
```

---

## Dispatch Layer

`libs/indicore/libastro.cpp` manages engine lifetime and exposes the public API.

```
INDI::setStellarEngine(StellarEngine::ERFA_2000B)
INDI::setPlanetaryEngine(PlanetaryEngine::EPH_INDI)
```

Engines are lazily constructed on first use and destroyed on engine change. A single mutex guards all reads and writes to the engine pointers.

---

## Validation

All accuracy claims are verified against external sources of truth. The test suite is never self-referential.

| Domain | Reference | Rationale |
|--------|-----------|-----------|
| Stars (IMCCE) | IMCCE Miriade, EOP=off, IAU 2006/2000A, geocentric | Standard for stellar apparent positions |
| Stars (SOFA) | ERFA `eraAtci13` zero-PM/parallax/RV point | Engine-internal calibration, no EOP by construction |
| Planets | JPL Horizons (DE440) | Standard for solar system bodies |

**IMCCE queries must use EOP=off.** With EOP=on, IMCCE applies measured IERS nutation corrections (typically 100–300 μas polar wobble) that `eraAtci13` — a pure IAU model — does not include.

### Verified results

| Test | Engine | Error vs truth |
|------|--------|----------------|
| Deneb apparent (frame-rotation only) | libnova | 0.094" |
| Deneb apparent (frame-rotation only) | ERFA-2000B | 0.019" |
| Deneb apparent (frame-rotation only) | ERFA-2000A | 0.020" |
| Sirius apparent (frame-rotation only) | ERFA-2000B | 0.268" (parallax-dominated) |
| Sirius apparent (full catalog: pm+px+rv) | ERFA-2000B | 0.024" |
| Vega apparent (full catalog) | ERFA-2000B | 0.035" |
| Arcturus apparent (full catalog) | ERFA-2000B | 0.001" |
| SOFA-QSO geocentric (zero pm/px/rv) | ERFA-2000A | 0.000" (< 0.2 mas) |
| SOFA-QSO geocentric (zero pm/px/rv) | ERFA-2000B | 0.000" (< 0.2 mas) |
| ERFA 2000A vs 2000B delta | — | < 0.001" |
| Geo-topo delta, zero-parallax source | ERFA-2000A | 0.157" (diurnal aberration) |
| Geo-topo delta, zero-parallax source | ERFA-2000B | 0.157" (diurnal aberration) |
| Mars geocentric | EPH-Full vs DE440 | 0.26" |
| Moon geocentric | EPH-Full vs DE440 | 0.25" |
| EPH-INDI vs EPH-Full | Mars geocentric | 0.003" |

**Residuals in full-catalog mode** (pm+px+rv from Hipparcos-2 passed to ERFA, compared against IMCCE truth) are dominated by catalog differences between Hipparcos-2 and the IMCCE reference catalog, not by model error. Betelgeuse's 57 mas residual is astrometrically limited — its parallax is uncertain at ~10 mas due to its resolved angular disc.

Test binaries: `test/core/test_engine_comparison`, `test/core/test_eph_library`.

---

## File Map

| File | Role |
|------|------|
| `libs/indicore/libastro.h` | Public API: coordinate types, functions, engine selection enums |
| `libs/indicore/libastro.cpp` | Factory dispatcher, mutex-guarded engine lifecycle |
| `libs/indicore/CoordinateEngine.h` | `ICoordinateEngine`, `IPlanetaryEngine`, factory function declarations |
| `libs/indicore/ErfaEngine.cpp` | `ErfaEngine2000A`, `ErfaEngine2000B`, `eraApci00b`/`eraAtci00b`/`eraApco00b` wrappers |
| `libs/indicore/LibnovaEngine.cpp` | `LibnovaStellarEngine`, `LibnovaPlanetaryEngine` |
| `libs/indicore/EphEngine.cpp` | `EphEngineFull`, `EphEngineINDI` |
| `libs/indicore/eph/` | EPH library source and `.ctx` data files (gitignored) |
| `test/data/star_golden.json` | Ground-truth apparent positions (IMCCE + SOFA) with Hipparcos-2 catalog data |
