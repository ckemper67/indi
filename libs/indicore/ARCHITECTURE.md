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

**`EquatorialToHorizontal` — geocentric CIRS input, observer-local output**: uses `eraApco00b` + `eraAtioq`. This involves two conceptually distinct coordinate frames that are worth distinguishing:

- The CIRS coordinates fed in (JNow RA/Dec) are **geocentric**: they were computed by `J2000toObserved` using `eraApci*`, which places the origin at the Earth's center with no observer offset.
- `eraAtioq` with the `eraApco00b` context converts those geocentric CIRS coordinates to the **observer's local horizontal frame**, using the observer's longitude and latitude to compute local sidereal time and hence the local hour angle. This step is correct and necessary.

There is no problematic mixing here — the observer location is required to rotate from CIRS to local horizontal, and `eraAtioq` does exactly that. The limitation is that no **topocentric parallax** is applied: the geocentric CIRS direction is not shifted by the geocenter-to-observer baseline (~6400 km). For stars and outer planets this shift is negligible. For the Moon (~57' parallax) and Sun (~9"), the horizontal position will be off by that amount. This is a known limitation to be addressed in M5 (topocentric promotion via `eraApco*` with full observer context through the entire pipeline).

### Planetary Engines (`libs/indicore/EphEngine.cpp`, `LibnovaEngine.cpp`)

The EPH library is authored by Patrick Wallace (also the original author of SLALIB and co-author of ERFA/SOFA). It implements VSOP2010 planetary theory and the ELP/MPP02 lunar theory, distributed as pre-compiled binary `.ctx` data files. See https://tpointsw.uk/eph.htm.

The library is released under an ISC-style license: permission is granted to use, copy, modify, and distribute for any purpose with or without fee. Copyright (C) P.T.Wallace. All rights reserved.


| Class | Backend | Theory | Enum | Precision | Status |
|-------|---------|--------|------|-----------|--------|
| `LibnovaPlanetaryEngine` | libnova | VSOP87 | `LIBNOVA` | ~1000" | Fallback |
| `EphEngineFull` | EPH library | VSOP2013 (full) | `VSOP2013` | ~0.27" geocentric | Available |
| `EphEngineINDI` | EPH library | VSOP2013 (truncated, 2.6 MB) | `VSOP2013_PACKED` | +0.003" vs VSOP2013 | Default |
| `EphEngineHybrid` | EPH + TOP2013 | VSOP2013 (inner) + TOP2013 (outer) | `VSOPTOP2013` | ~0.27" inner; < 2" vs VSOP2013 outer | Available |

**Time conversion**: UTC → TAI → TT via `eraUtctai` + `eraTaitt`. TT is used as TDB (TDB-TT < 2ms geocentric). This corrects the ~69s UT1/TDB discrepancy present in the original implementation.

**Context loading**: `EphEngineFull` holds the `.ctx` contexts as member variables. The Earth/EMB context and Moon context are loaded once on first use; the planet context is reloaded when `np` changes. Load failures are detected from `ephPlanc`/`ephMoonc` return values and cause an early return.

**`EphEngineINDI`**: loads `.ictx` packed files via `ephPlanci()`, falling back to `ephPlanc()` (`.ctx`) if `.ictx` files are absent. The packer (`tools/indi_eph_packer`) applies a time-weighted amplitude filter: threshold $10^{-9}$ AU with `max_tm=0.2` (±200 yr from J2000), producing a 2.6 MB dataset across 8 planets. Validated delta vs `EphEngineFull`: 0.003" for Mars at JD 2459019.833333. Both engines agree with JPL DE440 geocentric to ~0.25".

**`EphEngineHybrid`**: uses VSOP2013 for bodies 1–4 (Mercury through Moon/Earth) and TOP2013 for bodies 5–8 (Jupiter through Neptune). TOP2013 uses a simpler frequency parameterization (`arg = m × dmu × t` with a single global `dmu`) that achieves higher accuracy for the outer planets with comparable term counts. Outer planet contexts are loaded from `.tictx` packed files (0.74 MB total) via `ephTopci()`, falling back to `.tctx` binary contexts. Validated delta vs `EphEngineFull` at JD 2459019.833333: Jupiter 1.2", Saturn 0.6", Uranus 0.1", Neptune 0.2". Inner planet path is identical to `EphEngineINDI`.

### Generating EPH data files

The `.ctx` and `.ictx` files in `libs/indicore/eph/` are derived data — they
are not in version control and must be generated before the first build.

**Step 1 — Build the tools**

Both the ASCII-to-binary converter and the INDI packer are built as part of the
normal CMake build:

```bash
cd build
cmake ..
make eph_plan_bin indi_eph_packer
```

`eph_plan_bin` is defined in `libs/indicore/eph/CMakeLists.txt` and links
against the `eph` static library. `indi_eph_packer` is in `tools/CMakeLists.txt`
and links against the same library.

**Step 2 — Generate `.ctx` binary context files**

`eph_plan_bin` reads the VSOP2013 ASCII data files (`VSOP2013p1.dat` …
`VSOP2013p8.dat`) from the current working directory and writes
`VSOP2013_1.ctx` … `VSOP2013_8.ctx`. The ASCII data files are distributed
separately and are not vendored; download them from:

    https://ftp.imcce.fr/pub/ephem/planets/vsop2013/solution/

```bash
# Run from a directory containing the VSOP2013p*.dat files
/path/to/build/libs/indicore/eph/eph_plan_bin

# Keep the .ctx files in this working directory — they are gitignored
# and are only needed as input to indi_eph_packer in Step 3.
```

The Moon context files (`ELP_MPP02_JPL.ctx`, `ELP_MPP02_LLR.ctx`) are
generated separately using the `moon_bin` tool from the upstream EPH
distribution with the ELP/MPP02 ASCII series files.

**Step 3 — Pack `.ictx` files (for `EphEngineINDI`)**

The INDI packer applies a truncation filter to reduce the eight planet contexts
to ~2.6 MB total while keeping error below 0.003" vs the full theory.

```bash
mkdir -p /tmp/ictx_out
./build/tools/indi_eph_packer \
    /path/to/indi/libs/indicore/eph \
    /tmp/ictx_out \
    1e-9 0.2
# writes VSOP2013_1.ictx … VSOP2013_8.ictx

# .ictx files are committed to the repo (2.6 MB total)
cp /tmp/ictx_out/VSOP2013_*.ictx /path/to/indi/libs/indicore/eph/
```

The packer reads each `.ctx` in order (bodies 1–8), applies the amplitude
filter, and writes a compact variable-length `.ictx` with a 32-byte header
(magic `ICTX`, version 2, ibody, nterms, threshold, reserved). The `ephPlanci`
loader validates magic and version; a version mismatch is a hard error, so
regenerate `.ictx` files any time the packer or `ephPLANctx` struct changes.

**Step 4 — Verify**

```bash
./test/core/test_eph_library        # load + compute smoke tests
./test/core/test_engine_comparison  # validates against JPL DE440
```

### Generating TOP2013 data files (for `EphEngineHybrid`)

TOP2013 uses a single ASCII file (`TOP2013.dat`) for all outer planets, available from the same IMCCE URL above.

**Step 1 — Build the tool**

```bash
make eph_top_bin indi_eph_packer
```

**Step 2 — Generate `.tctx` binary context files**

`eph_top_bin` reads `TOP2013.dat` from the given directory and writes `TOP2013_5.tctx` … `TOP2013_8.tctx` to the current directory:

```bash
cd <dir-with-TOP2013.dat>
<build>/libs/indicore/eph/eph_top_bin
# or: eph_top_bin <path-to-TOP2013.dat-directory>
```

`.tctx` files are large (~2 MB each) and gitignored; they are only needed as input to the packer.

**Step 3 — Pack `.tictx` files**

```bash
mkdir -p /tmp/tictx_out
./tools/indi_eph_packer --top \
    <dir-with-tctx-files> \
    /tmp/tictx_out \
    1e-9 0.2
# writes TOP2013_5.tictx … TOP2013_8.tictx  (0.74 MB total)

cp /tmp/tictx_out/TOP2013_*.tictx libs/indicore/eph/
```

The `--top` flag selects TOP2013 mode; without it the packer produces VSOP2013 `.ictx` files. The `.tictx` files use a separate magic (`TICT`, version 1) from the VSOP2013 `.ictx` files (`ICTX`, version 2) and are not interchangeable.

---

## Dispatch Layer

`libs/indicore/libastro.cpp` manages engine lifetime and exposes the public API.

```
INDI::setStellarEngine(StellarEngine::ERFA_2000B)      // independent selections
INDI::setPlanetaryEngine(PlanetaryEngine::VSOP2013)
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
| libnova planetary | Mars geocentric (2020) | 1013" vs DE440 |
| VSOP2013_PACKED | Mars geocentric (2020) | 0.26" vs DE440 |
| VSOP2013_PACKED | Moon geocentric (2020) | 0.25" vs DE440 |
| VSOP2013 vs VSOP2013_PACKED | Mars geocentric (2020) | 0.003" |
| VSOP2013_PACKED | Jupiter geocentric (2000–2100, 11 epochs) | max 0.43" vs DE440 |
| VSOP2013_PACKED | Saturn geocentric (2000–2100, 11 epochs) | max 0.37" vs DE440 |
| VSOPTOP2013 | Jupiter geocentric (2000–2100, 11 epochs) | max 0.38" vs DE440 |
| VSOPTOP2013 | Saturn geocentric (2000–2100, 11 epochs) | max 0.37" vs DE440 |
| VSOP2013_PACKED vs VSOPTOP2013 | outer planets (2000–2100) | max delta 0.004" |

**TOP2013 vs VSOP2013 for outer planets**: within the modern epoch (2000–2100 AD), TOP2013 and VSOP2013 agree to within 0.004" and both achieve sub-0.4" accuracy vs JPL DE440 for Jupiter and Saturn. TOP2013's theoretical advantage over VSOP2013 only materialises at very long timescales (centuries to millennia from J2000); within the operational window for INDI it is negligible.

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
| `libs/indicore/EphEngine.cpp` | `EphEngineFull` (`VSOP2013`), `EphEngineINDI` (`VSOP2013_PACKED`), `EphEngineHybrid` (`VSOPTOP2013`) |
| `libs/indicore/eph/` | EPH library source; `.ictx`/`.tictx` data files committed; `.ctx`/`.tctx` gitignored |

---

## Running the Tests

### Build system

The project uses an **in-source CMake build** — `CMakeCache.txt` lives at the repo root and all build artifacts land in-tree. Always use `cmake --build` from the repo root to rebuild; using sub-directory `Makefile`s directly will not recompile when headers change:

```bash
# Rebuild a single test binary (picks up all header and library changes)
cmake --build /path/to/indi --target test_engine_comparison
cmake --build /path/to/indi --target test_eph_library

# Rebuild everything
cmake --build /path/to/indi
```

The test binaries land at:
```
test/core/test_engine_comparison
test/core/test_eph_library
```

### Runtime data dependencies

`EphEngine.cpp` locates data files via `INDI_DATA_DIR "/eph/"`, which CMake bakes in as `<repo>/libs/indicore/eph/` at compile time.

| File type | Location | Status | Required by |
|-----------|----------|--------|-------------|
| `VSOP2013_*.ictx` (8 files, ~2.6 MB) | `libs/indicore/eph/` | **committed** | `VSOP2013_PACKED`, `VSOPTOP2013` (inner) |
| `TOP2013_*.tictx` (4 files, ~0.74 MB) | `libs/indicore/eph/` | **committed** | `VSOPTOP2013` (outer) |
| `ELP_MPP02_JPL.ctx` | `libs/indicore/eph/` | gitignored, must generate | Moon (all EPH engines) |
| `VSOP2013_*.ctx` (8 files) | `libs/indicore/eph/` | gitignored, must generate | `VSOP2013` (full) only |
| `TOP2013_*.tctx` (4 files) | any | gitignored, intermediate | input to packer only |

**After a fresh clone, `VSOP2013_PACKED` and `VSOPTOP2013` work immediately** — their packed data files are committed. `VSOP2013` (full, untruncated) requires generating `.ctx` files; see "Generating EPH data files" above.

`TEST_DATA_DIR` for golden JSON files is baked in as `<repo>/test/data/` — the JSON files are committed and always available.

### Running

```bash
# All engine comparison tests (stars, planets, multi-epoch, hybrid)
./test/core/test_engine_comparison

# Run a specific test
./test/core/test_engine_comparison --gtest_filter="EngineComparison.MultiEpochDeviation"

# EPH library load and compute smoke tests
./test/core/test_eph_library
```

The `MultiEpochDeviation` test (~150 ms) is the most informative: it validates VSOP2013_PACKED and VSOPTOP2013 against JPL DE440 across 11 epochs from 2000 to 2100 for Mars, Jupiter, Saturn, and Moon.

---

## Planned Extensions

**M5 — Topocentric Promotion**: Introduce `INDI::ObservationContext` mapping to `eraASTROM`. Replace `eraApci*` with `eraApco*` context builders in `EquatorialToHorizontal`, enabling diurnal aberration, polar motion, and topocentric parallax when observer location is available.
