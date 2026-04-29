# Test Plan: INDI ERFA/EPH Migration

## 1. Overview
The goal of this migration is to replace the aging `libnova` coordinate engine with a modern, high-precision modular core based on **ERFA** and **EPH**. 

## 2. Testing Levels

### Level 1: Mathematical Truth (COMPLETED)
**Objective**: Verify implementation matches independent arbiters.
- **Method**: Compare engine results against the **Golden Datasets**.
- **Reference Data**: `test/data/planet_golden.json` (JPL) and `test/data/star_golden.json` (IMCCE).
- **Outcome**: Verified ERFA Reduces Deneb error to 0.05" and EPH reduces Mars error to 1.3" (Geocentric).

### Level 2: Backend Parity & Reciprocity
**Objective**: Validate the pluggable backend logic.
- **Method**: Run `test/core/test_engine_comparison.cpp`.
- **Criteria**: 
    - Reversing J2000 -> JNow -> J2000 results in < 0.001 arcsec error.
    - Swapping between engines at runtime is state-safe and memory-safe.

### Level 3: Stellar Model Parity (A vs B)
**Objective**: Mathematically prove the "Lite" model is sufficient.
- **Method**: Compare `ErfaEngineA` vs `ErfaEngineB`.
- **Success Criteria**: Delta < 0.001 arcsec.

### Level 4: EPH Parity & Reproducibility
**Objective**: Verify the `indi_eph_packer` and dynamic loader.
- **Method**: Compare `EphEngineFull` (ASCII Theory) vs `EphEngineINDI` (Packed).
- **Success Criteria**: Delta < 0.04 arcsec (matching our truncation filter).

### Level 5: The "Null-Delta" System Test (Simulator Sync)
**Objective**: Close the loop between Telescope and CCD Simulators.
- **Success Criteria**: Discrepancy < 0.1 arcsec.

### Level 6: Concurrency & Multi-Site Stress Test
**Objective**: Prove that explicit context passing prevents data pollution.

## 3. Milestone Schedule

| Milestone | Test Focus | Status |
|-----------|------------|--------|
| **M1** | Star and Planet truth vs JPL/IMCCE Golden Files | **PASS** |
| **M2** | Pluggable Backend Infrastructure & libnova baseline | Pending |
| **M3** | ERFA-A vs ERFA-B Parity Verification | Pending |
| **M4** | EPH Library Integration & Full vs. Packed Parity | Pending |
| **M5** | Observation Context & Topocentric Promotion | Pending |
| **M6** | Full "Null-Delta" Simulator Synchronization | Pending |
