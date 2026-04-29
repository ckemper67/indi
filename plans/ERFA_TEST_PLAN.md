# Test Plan: INDI ERFA/EPH Migration

## 1. Overview

The migration replaces the aging `libnova` coordinate engine with a modern, high-precision modular core based on **ERFA** (stellar frame rotation) and **EPH** (VSOP2010 planetary theory). The two engine hierarchies are pluggable independently.

All validation uses **external sources of truth** — the test suite never compares the implementation against itself:
- **Stars**: IMCCE golden dataset (SIMBAD/CDS apparent positions)
- **Planets**: JPL Horizons geocentric apparent positions (DE440)

The split follows domain expertise — IMCCE/CDS is the standard for stellar catalogs, JPL Horizons for solar system bodies — and provides independent model cross-validation (INPOP21 vs DE440).

---

## 2. Test Levels

### Level 1: External Truth Validation (COMPLETED)
**Objective**: Prove each engine matches an independent arbiter.

**Stellar** — `test/core/test_engine_comparison.cpp` / `EngineComparison.StarDeviation`
- Reference: IMCCE apparent position for Deneb at a fixed epoch.
- Results:
  - libnova: **15.7"** error
  - ERFA-2000A: **0.050"** error
  - ERFA-2000B: **0.050"** error
  - A vs B delta: **0.000264"**

**Planetary** — `test/core/test_engine_comparison.cpp` / `EngineComparison.PlanetDeviation`
- Reference: JPL Horizons geocentric apparent position for Mars (2020).
- Results:
  - libnova: **1008"** error
  - EPH-Full: **0.73"** error (after correct UTC→TDB conversion)

**EPH library** — `test/core/test_eph_library.cpp`
- Validates `.ctx` file loading and `ephRdplan` computation for Earth, Mars, Moon.

### Level 2: Backend Parity & Reciprocity (COMPLETED)
**Objective**: Validate the pluggable dispatch layer.

`EngineComparison.Reciprocity` — `test/core/test_engine_comparison.cpp`
- J2000 → JNow → J2000 round-trip error < 0.001".
- Engine switching at runtime is state-safe and memory-safe (mutex-guarded lazy init).

### Level 3: Stellar Model Parity — A vs B (COMPLETED)
**Objective**: Prove the IAU 2000B "Lite" model is sufficient for INDI's default tier.

- ERFA-2000A vs ERFA-2000B delta: **0.000264"** (criterion: < 1 mas).
- Both use the same `eraApci*/eraAtci*` call structure; 2000B substitutes `eraPnm00b` and `eraS00`.

### Level 4: EPH Parity — Full vs Packed (Pending — M4)
**Objective**: Verify the `indi_eph_packer` truncation and dynamic loader.

- Compare `EphEngineFull` (complete VSOP2010) vs `EphEngineINDI` (packed, $10^{-11}$ filter).
- Success criterion: delta < 0.04" (matching truncation filter budget).
- Currently `EphEngineINDI` delegates to `EphEngineFull` with a log warning; this test is blocked on the packer implementation.

### Level 5: Topocentric Promotion (Pending — M5)
**Objective**: Verify that topocentric corrections produce accurate alt/az coordinates.

- Reference: JPL Horizons topocentric apparent positions for a known observer location.
- Success criterion: alt/az error < 0.1" vs JPL truth.
- Currently blocked: `EquatorialToHorizontal` is geocentric (observer parameter unused).

### Level 6: Concurrency & Multi-Site Stress Test (Pending — M5)
**Objective**: Prove that mutex-guarded dispatch and explicit context passing prevent cross-driver data pollution when multiple drivers run simultaneously with different engine selections.

### Level 7: Full "Null-Delta" Simulator Synchronization (Pending — M6)
**Objective**: Close the loop between Telescope and CCD Simulators end-to-end.

- Success criterion: < 0.1" discrepancy between simulated pointing and CCD centroid.

---

## 3. Milestone Status

| Milestone | Test Focus | Status |
|-----------|------------|--------|
| **M1** | Star (IMCCE) and Planet (JPL) external truth validation | **PASS** |
| **M2** | Pluggable dispatch, libnova baseline, runtime engine switching | **PASS** |
| **M3** | ERFA-2000A and 2000B vs IMCCE; A-vs-B parity < 1 mas | **PASS** |
| **M4** | EPH-Full vs JPL truth; Full vs Packed parity < 0.04" | Partial (Full done, Packed pending) |
| **M5** | Topocentric promotion; alt/az vs JPL truth | Pending |
| **M6** | Full simulator null-delta synchronization | Pending |

---

## 4. Test Infrastructure

| File | What it tests |
|------|---------------|
| `test/core/test_engine_comparison.cpp` | StarDeviation (IMCCE), Reciprocity, PlanetDeviation (JPL) |
| `test/core/test_eph_library.cpp` | EPH `.ctx` loading and `ephRdplan` computation |
| `test/core/test_libastro_regression.cpp` | Legacy libastro API regression (currently broken — references removed `INDI::setEngine` API) |
