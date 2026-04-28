# Report: libastro-Centric ERFA/EPH Migration Strategy

## Executive Summary

The primary interface for coordinate conversions in the INDI framework is **`libastro`** (`libs/indicore/libastro.cpp`). While `libnova` symbols are used throughout the codebase, `libastro` provides the high-level logic for J2000/JNow conversions used by nearly all modern telescope drivers and simulators.

The most efficient path to high-precision coordinates is to **migrate `libastro` to ERFA and EPH**. This upgrades the entire framework's "brain" in one place, providing immediate benefits to any driver using the `INDI::` coordinate namespace.

---

## 1. libastro: The Strategic Leverage Point

Investigation shows that while `libnova` is a ubiquitous dependency, its usage for core tracking (precession, nutation, aberration) is consolidated into a few key functions in `libastro`:

- `INDI::J2000toObserved`: The standard path for calculating JNow coordinates.
- `INDI::ObservedToJ2000`: The standard path for converting mount feedback to catalog coordinates.
- `INDI::EquatorialToHorizontal`: The path for Alt/Az tracking and refraction.

By rewriting these three functions using **ERFA** and **EPH**, we achieve a framework-wide accuracy boost with minimal disruption to individual drivers.

---

## 2. Updated Migration Strategy

### Step 1: The ERFA Engine (`erfa_engine.cpp`)
To avoid a destructive rewrite, we will implement the new logic in a dedicated `erfa_engine.cpp`. `libastro.cpp` will act as a thin wrapper that delegates to either the legacy `libnova` math or the new `erfa_engine`.
- **Transparent Time Translation**: The engine will use ERFA's `eraDat` to derive the $\Delta T$ (TT - UTC) offset from the Julian Date, ensuring consistent time-scale handling.
- **Geocentric Foundation**: Deliver ~0.3" accuracy (a 30x improvement) as a pure drop-in replacement.

### Step 2: Explicit Observation Context
To avoid data pollution in multi-driver processes, the framework will prioritize **Explicit Context Passing**. While a snooped global context exists for legacy support, high-precision drivers will be updated to pass their own `ObservationContext` instances.

### Step 3: Topocentric Promotion & Refraction Guards
The `erfa_engine` will utilize the `ObservationContext` to account for diurnal aberration and parallax. It will implement a **Refraction Guard** for targets below 1° altitude to prevent mathematical discontinuities that could cause hardware "jerks."

### Step 4: The libnova Compatibility Layer (ABI-Safe)
For drivers that call `libnova` directly, we provide a "Namespace-Safe" shim using internal symbol redirection (`INDI_LN_*`).

---

## 3. The INDI Observation Context

To achieve maximum precision without complicating the API, we introduce `INDI::ObservationContext`. This structure encapsulates the physical environment:

- **Observer Location**: Longitude, Latitude, and Elevation.
- **Time Scales**: Explicit handling of ΔT (TT - UTC).
- **Atmospheric Model**: Pressure, Temperature, and Humidity.
- **Wavelength**: Support for IR or UV observation offsets.

**Implementation Note**: `INDI::ObservationContext` maps directly to ERFA's internal **`eraASTROM`** structure.

---

## 4. Implementation Roadmap

## 4. Implementation Roadmap

### M1: Mathematical Truth (COMPLETED)
- CCD Simulator verified with ERFA stars.

### M2: The `erfa_engine` Core (Geocentric Stars)
- **Goal**: Implement the `erfa_engine.cpp` backend using ERFA's geocentric path (`eraAtci13`).
- **Tasks**:
    - Implement time-scale translation using `eraDat`.
    - Implement the J2000/JNow wrapper logic in `libastro.cpp`.

### M3: libastro Solar System (Geocentric Planets)
- **Goal**: Integrate `eph` library into `libastro`.
- **Tasks**:
    - **Library Integration**: Establish linking against the EPH static library.
    - **Geocentric EPH Path**: Wrap `ephRdplan` to provide geocentric apparent places.
- **Benefit**: Sub-arcsecond planetary accuracy for all drivers.

### M4: Observation Context & Time (Topocentric Upgrade)
- **Goal**: Implement the `ObservationContext` infrastructure.
- **Tasks**:
    - Add **Explicit Context** overloads to the `libastro` API.
    - Implement the **Refraction Guard** logic for low-altitude safety.

### M5: EPH Packed Optimization
- **Goal**: Implement the `indi_eph_packer` utility.
- **Tasks**:
    - **Packed EPH Format**: Implement the dynamic term loader (~1.5 MB/planet).
    - **Tools**: Commit `indi_eph_packer` to `tools/` for theory reproducibility.

### M6: ABI-Safe libnova Shim
- **Goal**: Implement the `libnova` compatibility layer without symbol collisions.

### M7: Systematic Validation & Driver Sync
- **Goal**: Synchronize simulators and verify "Null-Delta" (< 0.1").

### M8: Dataset Deployment & Packaging
- **Goal**: Define the distribution of EPH context files.

## 5. Planetary Accuracy Tiers

| Tier | Theory | Footprint | Accuracy | Target |
| :--- | :--- | :--- | :--- | :--- |
| **Ultra** | VSOP2010 (Full) | 140 MB | < 0.01" | Research / Simulators |
| **Standard**| **VSOP2010 (INDI)**| **~25 MB** | **~0.04"** | **Default INDI Rollout**|
| **Legacy** | VSOP87 (libnova)| ~3.5 MB | ~1.0" | Minimal Fallback |

---

## 6. Verification Metrics

All migration steps must be validated against the **Discrepancy Baseline** established in the CCD simulator:
1.  **Null-Delta Goal**: Discrepancy < 0.1 arcsec.
2.  **A/B Parity**: Ensure the system can toggle between engines for field verification.

---

## 7. Key Design Decisions

1.  **IAU 2000B by Default**: Use the "Lite" model (~1 mas accuracy, 0.7 μs overhead).
2.  **No Global State for Precision**: Mandate explicit context passing for high-precision operations.
3.  **Refraction Safety**: Prioritize hardware stability (smooth curves) over mathematical rigor below 1° altitude.

---

## 8. Open Questions

1.  **ΔT Source**: Should INDI eventually support a network-updated ΔT from IERS, or is the Morrison & Stephenson estimator sufficient for all use cases?
2.  **EPH Packaging**: Should the binary datasets be a separate `indi-eph-data` package?
3.  **Shim Depth**: Should we shim the entire `libnova` API surface or strictly focus on coordinates?
