# Report: libastro-Centric ERFA/EPH Migration Strategy

## Executive Summary

The INDI project is migrating to a **Modular Astronomical Core**. By decoupling stellar math (**ERFA**) from planetary math (**EPH**), the framework can provide independent precision and performance tiers tailored to different hardware and use cases.

The migration path is to retrofit **`libastro`** (`libs/indicore/libastro.cpp`) as a factory wrapper, upgrading the entire framework's coordinate brain while maintaining full backward compatibility with the existing public API.

---

## 1. Modular Engine Architecture

Two independent engine interfaces allow stellar and planetary precision to be selected separately:

### 1.1 Stellar Engine (Frame Rotation)
| Model | Theory | Terms | Precision | Target |
| :--- | :--- | :--- | :--- | :--- |
| **ERFA-2000B** | IAU 2000B | 77 | ~1.0 mas | **Default INDI High-Precision** |
| **ERFA-2000A** | IAU 2000A | 1,365 | ~0.1 mas | Research / Simulators |
| **Legacy** | libnova | - | ~15" | Safety Fallback |

### 1.2 Planetary Engine (Solar System)
| Model | Theory | Footprint | Precision | Target |
| :--- | :--- | :--- | :--- | :--- |
| **EPH-INDI** | VSOP2010 (Trunc) | ~12 MB | ~0.04" | **Standard INDI High-Precision** |
| **EPH-FULL** | VSOP2010 (Full) | ~140 MB | < 0.01" | Research / High-End Mounts |
| **Legacy** | VSOP87 | ~3 MB | ~1.0" | Minimal Footprint Fallback |

---

## 2. Migration Strategy

### Step 1: The Pluggable Core (COMPLETED)
`libastro.cpp` acts as a factory wrapper. Stellar and planetary engines are selected independently at runtime via `setStellarEngine()` and `setPlanetaryEngine()`. The existing public API (`J2000toObserved`, `ObservedToJ2000`, `EquatorialToHorizontal`, `GetPlanetObserved`) is fully preserved — callers are unaware of which engine is active.

### Step 2: Geocentric Foundation (COMPLETED)
Modern engines establish a geocentric baseline, delivering a 30x precision boost over libnova without breaking the position-independent API signatures. Topocentric promotion is deferred to Step 3.

### Step 3: Topocentric Promotion (Pending — M5)
`INDI::ObservationContext` will be introduced as shared environment state, allowing engines to promote themselves to full topocentric truth (diurnal aberration, parallax, rigorous refraction) when observer location is available.

---

## 3. Implementation Roadmap

### M1: Mathematical Truth (COMPLETED)
- Validated ERFA star positions against IMCCE golden dataset: Deneb error 0.05" geocentric.
- Validated EPH planetary positions against JPL Horizons: Mars error 0.73" geocentric (after correct UTC→TDB conversion via ERFA).
- Golden datasets: `test/core/test_engine_comparison.cpp`, `test/core/test_eph_library.cpp`.

### M2: Pluggable Backend Infrastructure (COMPLETED)
- Defined `ICoordinateEngine` and `IPlanetaryEngine` interfaces (`CoordinateEngine.h`).
- Implemented `LibnovaStellarEngine` and `LibnovaPlanetaryEngine` as the legacy control group.
- `libastro.cpp` dispatches to the active engine with mutex-guarded lazy initialization.
- Stellar and planetary engines are plugged independently — any combination is valid.

### M3: The ERFA Stellar Tier (COMPLETED)
- Implemented `ErfaEngine2000A` using the existing `eraAtci13`/`eraApci13`/`eraAtic13` wrappers.
- Implemented `ErfaEngine2000B` with new `eraAtci00b`/`eraApci00b` wrappers that mirror the `eraApci13` call chain, substituting `eraPnm00b` for `eraPnm06a` and `eraS00` for `eraS06`.
- Verified A-vs-B delta: **0.000264"** (< 1 mas criterion).
- Time input handled as UTC; split into two-part JD per ERFA convention.

### M4: libastro Solar System Tier (IN PROGRESS)
- `EphEngineFull` implemented: loads VSOP2010 `.ctx` files, calls `ephRdplan` for geocentric apparent coordinates.
- Time conversion corrected: UTC → TAI → TT via ERFA; TT used as TDB (TDB-TT < 2ms geocentric).
- `EphEngineINDI` is a placeholder that delegates to `EphEngineFull` with a log warning.
- **Remaining**: implement `indi_eph_packer` (truncation filter at $10^{-11}$) and the packed loader in `EphEngineINDI`.

### M5: Observation Context & Topocentric Promotion (Pending)
- **Goal**: Implement `ObservationContext` mapping to `eraASTROM`.
- **Task**: Add topocentric support to all modern engines via `eraApco*` context builders.
- **Note**: The `EquatorialToHorizontal` method currently uses a geocentric ASTROM; the observer parameter is accepted but unused pending this milestone.

### M6: Systematic Validation & Driver Sync (Pending)
- **Goal**: Synchronize all core simulators and verify framework-wide accuracy.

---

## 4. Key Design Decisions

1.  **Independent stellar and planetary selection**: The two engine hierarchies are fully decoupled. A driver can use ERFA-2000B for star tracking and libnova for planets, or any other combination, without either engine knowing about the other.
2.  **ERFA-2000B by Default**: The 77-term model offers the best speed/precision balance. Standard ERFA convenience wrappers and the SPK plugin default to IAU 2000A; INDI's 2000B default provides a unique performance tier suitable for Raspberry Pi hardware while maintaining sub-arcsecond accuracy.
3.  **External sources of truth for validation**: Star positions are validated against IMCCE (SIMBAD/CDS); planetary positions against JPL Horizons (DE440). Neither the implementation nor the test framework is self-referential. The split follows domain expertise — IMCCE/CDS is the standard for stellar catalogs, JPL Horizons for solar system bodies — and provides independent model cross-validation (INPOP21 vs DE440).
4.  **Packed EPH for Standard Rollout**: The $10^{-11}$ truncation filter yields ~12 MB vs 140 MB for the full theory, with < 0.04" additional error.
5.  **Tiered Fallback**: Automatically fall back to legacy libnova if high-precision datasets are missing.
6.  **No Global State for Precision**: Engine access is mutex-guarded. Explicit context passing (planned for M5) will prevent cross-driver data pollution.
