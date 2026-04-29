# Plan: Packed EPH Dataset for EphEngineINDI

## Background

`EphEngineFull` loads VSOP2010 planetary theory from `.ctx` files produced by `plan_bin.c` — a raw `fwrite` of the entire `ephPLANctx` struct. Because `MAXTERM = 351000` is a compile-time array dimension, every `.ctx` file is exactly 17.6 MB regardless of how many terms are actually populated. At `~0.73"` geocentric accuracy, VSOP2010 far exceeds INDI's needs; a large fraction of its terms contribute sub-nanoradian corrections.

The goal is to produce truncated `.ictx` files (INDI packed context) that:
- Fit in ~12 MB total (8 planets + moon)
- Deliver < 0.04" additional error vs the full theory
- Load directly into the existing `ephPLANctx` struct so `ephPlanet`/`ephRdplan` require no changes

The moon context (`ephMOONctx`, ~1.7 MB) is already compact and will not be truncated.

**Status: COMPLETE.** All steps implemented and validated. See results below.

---

## Truncation Principle

Each VSOP2010 term has a sine amplitude `ss[n]` and cosine amplitude `cc[n]` in AU (for position variables). Its contribution to the ephemeris is bounded by:

```
amplitude = hypot(ss[n], cc[n])
```

Terms with `amplitude < threshold` are dropped. The threshold $10^{-10}$ AU is used (not $10^{-11}$ as originally estimated — see threshold analysis results below). The surviving terms are compacted in-place; the `limit[it][iv]` counts are updated to reflect the new term counts.

---

## Step 1: Threshold Analysis — COMPLETE

**Tool**: `tools/eph_threshold_analysis.c`

Results (total across 8 planets):

| threshold | terms_out | ratio | est_MB |
|-----------|-----------|-------|--------|
| 1e-8  | 26,605  |  1.1% |  1.34 |
| 1e-9  | 72,013  |  3.0% |  3.61 |
| 1e-10 | 185,938 |  7.6% |  9.30 |
| 1e-11 | 459,303 | 18.9% | 22.97 |
| 1e-12 | 1,016,490 | 41.8% | 50.83 |

**Selected threshold: $10^{-10}$** (9.30 MB total, well under 12 MB target). The original estimate of $10^{-11}$ was wrong — it produces 23 MB.

---

## Step 2: Compact Binary Format (.ictx) — COMPLETE

The existing `.ctx` format cannot be made smaller without changing the struct layout, because it is a fixed-size memory dump. The `.ictx` format stores only surviving terms in a variable-length record layout, then deserializes into `ephPLANctx` on load.

**File layout**:

```
[Header — 48 bytes]
  magic       : uint32  0x49435458 ("ICTX")
  version     : uint32  1
  ibody       : int32
  nterms      : int32   total surviving terms
  threshold   : double
  reserved    : double

[Metadata — same as ephPLANctx fixed fields]
  receq[3][3] : 9 doubles
  rgm         : double
  ci0[17]     : 17 doubles
  ci1[17]     : 17 doubles
  freqpla[8]  : 8 doubles

[limit table]
  limit[21][6]: 126 int16

[Term records — nterms entries]
  iphi[17]    : 17 int16
  ss          : double
  cc          : double
  (50 bytes per term)
```

This format is self-describing (magic + version), body-identified, and contains only the terms that survived the filter. A planet with 20,000 surviving terms produces a 1.05 MB file. Actual packed sizes at $10^{-10}$: Mercury 0.18 MB, Venus 0.26 MB, EMB 0.39 MB, Mars 0.81 MB, Jupiter 1.00 MB, Saturn 2.17 MB, Uranus 2.88 MB, Neptune 1.62 MB.

---

## Step 3: indi_eph_packer Tool — COMPLETE

**File**: `tools/indi_eph_packer.c`

Standalone C program. Reads existing binary `.ctx` files via `ephPlanc`.

```
Usage: indi_eph_packer <ctx_dir> <output_dir> [threshold]
  threshold defaults to 1e-11
```

**Algorithm** for each planet 1–8:

1. Call `ephPlanc(ibody, ctx_dir, &ctx)` to load the full binary context
2. Two-pass filter: count surviving terms (amplitude >= threshold), then compact in planet.c iteration order (iv outer, it inner)
3. Write `.ictx`: header, metadata, updated limit table, surviving term records
4. Report: terms retained, file size, compression ratio

It is a one-time offline step; its output `.ictx` files are what gets distributed with INDI.

---

## Step 4: Packed Context Loader — COMPLETE

**File**: `libs/indicore/eph/planc_indi.c`

New function `ephPlanci(int ibody, char* path, ephPLANctx* c)` — same signature as `ephPlanc` but reads `.ictx` instead of `.ctx`:

1. Open `VSOP2010_#.ictx`
2. Read and validate magic + version
3. Read metadata into `c->receq`, `c->rgm`, `c->ci0`, etc.
4. Zero the `c->limit` array and `c->ss`/`c->cc`/`c->iphi` arrays
5. Read the limit table; accumulate term offsets to compute the start index per `(it, iv)` slot
6. Read term records sequentially into `c->iphi[n]`, `c->ss[n]`, `c->cc[n]`

After loading, `c` is a valid `ephPLANctx` and `ephPlanet`/`ephRdplan` work unchanged.

Declare `ephPlanci` in `eph.h` alongside `ephPlanc`.

---

## Step 5: Wire EphEngineINDI — COMPLETE

Replace the placeholder in `EphEngine.cpp`:

- `EphEngineINDI` calls `ephPlanci()` instead of `ephPlanc()`
- Data path resolves `.ictx` files from `INDI_DATA_DIR "/eph/"`
- Remove the `IDLog` fallback warning once implemented
- The moon context (`ephMoonc` / `.ctx`) is unchanged

`EphEngineFull` retains `ephPlanc` and the original `.ctx` files unchanged.

---

## Step 6: Validation — COMPLETE

**Test**: `test/core/test_engine_comparison.cpp`, `PlanetDeviation` test case.

**Results (Mars, JD 2459019.833333)**:

| Engine | Error vs JPL | FULL vs INDI delta |
|--------|-------------|-------------------|
| EPH_FULL | 0.732" | — |
| EPH_INDI | 0.732" | 0.000638" |

**Criteria met**:
- `EphEngineINDI` vs `EphEngineFull` delta: **0.000638"** < 0.04" budget
- Total `.ictx` dataset (8 planets): **9.30 MB** < 12 MB target
- `EphEngineINDI` falls back to `.ctx` gracefully when `.ictx` files are absent

---

## File Map

| File | Role |
|------|------|
| `tools/eph_threshold_analysis.c` | Offline analysis: count surviving terms per threshold |
| `tools/indi_eph_packer.c` | Offline packer: ASCII VSOP2010 → `.ictx` |
| `libs/indicore/eph/planc_indi.c` | Runtime loader: `.ictx` → `ephPLANctx` |
| `libs/indicore/eph/VSOP2010_#.ictx` | Packed planet contexts (8 files, gitignored) |
| `libs/indicore/EphEngine.cpp` | Wire `EphEngineINDI` to `ephPlanci` |

---

## Dependencies and Constraints

- The offline tools (`eph_threshold_analysis`, `indi_eph_packer`) require the ASCII VSOP2010 `.dat` files from the VSOP2010 distribution, which are not part of the INDI repo.
- The runtime `.ictx` files are the distributable artifact; they ship with INDI in the same way `.ctx` files do (i.e., as optional data, not in the repo).
- `ephPlanci` must stay within the same `ephPLANctx` struct so no changes are needed to `ephPlanet`, `ephRdplan`, or `EphEngineFull`.
- The moon (ELP/MPP02) is not truncated; `EphEngineINDI` uses the same `ELP_MPP02_JPL.ctx` as `EphEngineFull`.
