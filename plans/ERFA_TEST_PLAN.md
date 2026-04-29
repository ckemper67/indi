# Test Plan: INDI ERFA/EPH Migration

## 1. Overview
The goal of this migration is to replace the aging `libnova` coordinate engine with a modern, high-precision core based on **ERFA** (stars/rotation) and **EPH** (solar system). This test plan ensures mathematical correctness, framework integrity, and backward compatibility.

## 2. Testing Levels

### Level 0: Baseline Regression (Current libastro 1.0)
**Objective**: Capture the exact output of the current `libnova`-based implementation.
- **Method**: Use `test/core/test_libastro_regression.cpp`.
- **Targets**: Polaris, Canopus, Siding Spring, Quito.
- **Success Criteria**: 100% pass against current `libnova` behavior.

### Level 0.5: Wrapper Parity (Legacy Mode)
**Objective**: Prove that the `libastro.cpp` wrapper correctly delegates to the legacy engine with **zero deviation**.
- **Method**: Run Level 0 tests through the new `libastro.cpp` wrapper with the `ERFA_ENGINE` toggle **DISABLED**.
- **Success Criteria**: **Bit-for-bit mathematical identity** (0.0" delta) compared to Level 0. This proves the wrapper is a "no-op" for current users.
### Level 0.75: A/B Engine Comparison
**Objective**: Validate the engine-swap architecture by running both engines in parallel within the same process.
- **Method**: Instantiate both `libnova_engine` and `erfa_engine` (geocentric) and calculate the same coordinates for the same JD.
- **Success Criteria**: 
    - The delta must be predictable (matching the geocentric-vs-topocentric shift).
    - Swapping the 'Active Engine' via the wrapper must not induce memory leaks or state pollution.

### Level 1: Controlled Deviation (The "Accuracy Proof")
...

**Objective**: Prove that toggling the wrapper to **ERFA Mode** successfully upgrades the math.
- **Method**: Run Level 0 tests through the `libastro.cpp` wrapper with the `ERFA_ENGINE` toggle **ENABLED**.
- **Success Criteria**: 
    - **Star Shift**: Delta must match predicted "libnova error" (3–12 arcsec).
    - **ΔT Correction**: Verify the Morrison & Stephenson estimator logic.

### Level 2: Mathematical Unit Tests (Truth Validation)
**Objective**: Verify implementation matches independent arbiters with high precision.
- **Method**: Compare engine results against the **Golden Datasets**.
- **Truth Providers**: 
    - `tools/generate_planet_golden.py`: Queries JPL Horizons for planetary truth.
    - `tools/generate_star_golden.py`: Queries IMCCE Miriade for stellar truth.
- **Reference Data**: 
    - `test/data/planet_golden.json`: NASA-verified planetary coordinates.
    - `test/data/star_golden.json`: IMCCE-verified star coordinates.
- **Success Criteria**: 
    - Stars: Delta < 0.001 arcsec (vs. SOFA examples).
    - Planets: Delta < 2.0 arcsec (vs. JPL Golden baseline in geocentric mode).
    - Planets (Final): Delta < 0.05 arcsec (after Milestone 4 topocentric upgrade).

### Level 3: libastro 2.0 Regression & Sweeps
**Objective**: Validate abstractions and physical edge cases.
- **Targets**:
    - **Centennial Sweep**: Stability check every 10 years (1950–2050).
    - **Refraction Profile**: Sweep altitude (90° to 0°).
- **Success Criteria**: Smooth curves, no discontinuities, and **Refraction Guard** verification (no `NaN` or jumps at the horizon).

### Level 4: Solar System Parity & Reproducibility
**Objective**: Verify the **`indi_eph_packer`** and dynamic loader.
- **Method**: Compare coordinates generated from raw ASCII coefficients against coordinates from our packed binary format.
- **Success Criteria**: Bit-for-bit mathematical identity.

### Level 5: The "Null-Delta" System Test (Simulator Sync)
**Objective**: Close the loop between Telescope and CCD Simulators.
- **Success Criteria**: Discrepancy < 0.1 arcsec.

### Level 6: Distribution & Fallback
**Objective**: Ensure robustness to missing EPH datasets.
- **Success Criteria**: Clear warning log and automatic fallback to VSOP87.

### Level 7: Concurrency & Multi-Site Stress Test
**Objective**: Prove that explicit context passing prevents data pollution.
- **Method**: Spawn concurrent threads with **conflicting** geographic coordinates in their respective `ObservationContext` objects.
- **Success Criteria**: Each thread receives results corresponding *only* to its own context; zero cross-pollination.

## 3. Test Environments
- **CI/CD**: Automated unit tests in GitHub Actions.
- **Integration Harness**: Headless `indiserver` instance for simulator sync.

## 4. Milestone Schedule

| Milestone | Test Focus | Status |
|-----------|------------|--------|
| **M0** | Baseline Regression (Capture libastro 1.0 output) | **PASS** |
| **M1** | Star rendering vs. Telescope JNow (CCD experiment) | **PASS** |
| **M1.5** | Wrapper A/B Parity & Comparison | Pending |
| **M2** | `erfa_engine` Core & ΔT Estimator | Pending |
| **M3** | EPH Unit Tests & Parity Check (Geocentric Planets) | Pending |
| **M4** | Observation Context & Refraction Guard | Pending |
| **M5** | EPH Packed Optimization (Packer Tool) | Pending |
| **M6** | libnova-shim integration & ABI check | Pending |
| **M7** | Full "Null-Delta" Simulator Synchronization | Pending |
| **M8** | Centennial & Refraction Profile Sweeps | Pending |
| **M9** | Concurrency & Multi-Site Stress Validation | Pending |
| **M10** | Dataset Deployment & Packaging | Pending |


## 5. Risk Mitigation

| Risk | Impact | Mitigation Strategy |
|------|--------|---------------------|
| **Observer Context** | ~0.3" error | Explicit context passing (M4). |
| **Time Scales** | ~1.1" error | Internal JD(UTC) to TT/TDB translation via Morrison & Stephenson ΔT estimator (M2). |
| **Multi-Site Pollution**| Data corruption | Prioritize explicit context over global snooping (M4/M8). |
| **Horizon Discontinuity**| Hardware jerks | Implement Refraction Guard for < 1° altitude (M4/M7). |
| **Symbol Collisions**| Crashes | Internal symbol redirection (`INDI_LN_*`) (M5). |
| **Data Silos** | Unverifiable math | Commit `indi_eph_packer` to tools (M3). |
| **Thread Safety** | Intermittent Crashes | Concurrency stress testing (M8). |

---

## 6. Open Questions
- **JPL Horizons Automation**: Feasibility of building a CI helper for live truth validation.
- **Platform Parity**: Bit-for-bit parity between x86_64 and ARM64.
