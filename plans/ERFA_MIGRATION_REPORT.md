# Report: libastro-Centric ERFA/EPH Migration Strategy

## Executive Summary

The INDI project is migrating to a **Modular Astronomical Core**. By decoupling stellar math (**ERFA**) from planetary math (**EPH**), the framework can provide independent precision and performance tiers tailored to different hardware and use cases.

The most efficient path is to migrate **`libastro`** (`libs/indicore/libastro.cpp`) to use pluggable backends, upgrading the entire framework's "brain" while maintaining full backward compatibility.

---

## 1. Modular Engine Architecture

We introduce two independent engine interfaces to manage accuracy and CPU/storage footprints:

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

## 2. Updated Migration Strategy

### Step 1: The Pluggable Core
We implement logic in `libastro.cpp` to act as a **Factory Wrapper**. It can load any combination of Stellar and Planetary engines at runtime based on configuration or environment.

### Step 2: Geocentric Foundation
The engines will first establish a MODERN geocentric baseline. This ensures a 30x precision boost for all celestial bodies without breaking existing position-independent API signatures.

### Step 3: Topocentric Promotion
The `INDI::ObservationContext` will be introduced as the shared environment state, allowing the engines to "promote" themselves to full topocentric truth (diurnal aberration, parallax, rigorous refraction) when location data is available.

---

## 3. Implementation Roadmap

### M1: Mathematical Truth (COMPLETED)
- Verified star and planetary corrections against JPL/IMCCE golden datasets.

### M2: Pluggable Backend Infrastructure
- **Goal**: Define `ICoordinateEngine` and `IPlanetaryEngine` interfaces.
- **Task**: Implement the `LibnovaEngine` (Legacy Control Group) for both.

### M3: The ERFA Stellar Tier
- **Goal**: Implement `ErfaEngineA` and `ErfaEngineB` geocentric backends.
- **Verification**: Mathematically prove the 0.001" delta between A and B models.

### M4: libastro Solar System Tier
- **Goal**: Integrate `eph` library and implement `EphEngineFull`.
- **Optimization**: Implement the `indi_eph_packer` and `EphEngineINDI` (Truncated).

### M5: Observation Context & Topocentric Promotion
- **Goal**: Implement `ObservationContext` mapping to `eraASTROM`.
- **Task**: Add topocentric support to all modern engines.

### M6: Systematic Validation & Driver Sync
- **Goal**: Synchronize all core simulators and verify framework-wide accuracy.

---

## 4. Key Design Decisions

1.  **ERFA-2000B by Default**: Use the 77-term model for the best balance of speed and precision.
    - *Note*: Standard ERFA convenience wrappers and the SPK plugin default to the **IAU 2000A** model. INDI's adoption of the 2000B model as the framework default provides a unique "Performance Tier" suitable for Raspberry Pi hardware while maintaining sub-arcsecond accuracy.
2.  **Packed EPH for Standard Rollout**: Adopt the $10^{-11}$ truncation filter (~12MB) for the default planetary engine.
3.  **Tiered Fallback**: Automatically fall back to legacy `libnova` math if high-precision datasets are missing.
4.  **No Global State for Precision**: Mandate explicit context passing for high-precision operations to prevent cross-driver data pollution.
