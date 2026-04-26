# Alignment Math Subsystem Architecture

This document defines the mathematical and conceptual architecture of INDI's telescope pointing model subsystem. Because telescope tracking depends on translating idealized sky coordinates to physical mechanical hardware, INDI provides different math plugins to model these discrepancies depending on user needs and hardware setups.

This guide details the shared conventions across the subsystem and outlines the two primary approaches used by the plugins: **Matrix-Based local mapping** and **Physics-Based unified modeling**.

---

## 1. Shared Core Concepts

Regardless of the solver algorithm, all alignment plugins must negotiate standard coordinate systems and sidereal time.

### Coordinate Systems

| System | Origin | Direction | Notes |
| :--- | :--- | :--- | :--- |
| **INDI/SOFA** | North (0°) | East positive (CCW from above) | Used by INDI's `IHorizontalCoordinates`, `iauHd2ae`, `iauAe2hd`. |
| **SPK internal (altaz)** | — | roll = π − Az | Left-handed encoding; see roll convention below. |
| **SPK internal (equatorial)** | — | roll = −HA | Right-hand rule; HA increases westward. |

INDI and SOFA use the same azimuth convention (North=0°, East=90°), so no conversion is required between them. The only non-trivial mapping is between INDI/SOFA azimuth and SPK's internal roll coordinate.

### SPK Roll Convention

SPK's internal solver (`spkVtel`) encodes mount axes as a (roll, pitch) pair whose semantics depend on mount type (see `spk/vtel.c` Note 7):

- **Altazimuth**: `roll = π − Az_NE`, where Az_NE is the north-through-east azimuth.
  Equivalently: `Az_NE = π − roll`.
- **Equatorial**: `roll = −HA`, where HA is the local hour angle in radians.

This encoding is internal to the SPK library. `SPKMathPlugin` translates between it and INDI's `TelescopeDirectionVector` (TDV) representation at every boundary:

| Function | Direction | Operation |
| :--- | :--- | :--- |
| `RollPitchToDirectionVector` | SPK → TDV | `Az = π − roll` |
| `DirectionVectorToRollPitch` | TDV → SPK | `roll = π − Az` |
| `ExtractObsRow` / `BuildObservationData` | TDV → Pmfit | `rdem = π − roll = Az` |

> [!IMPORTANT]
> The helpers `RollPitchToDirectionVector` and `DirectionVectorToRollPitch` are mutually
> inverse and both use `π − x`, so round-trip tests (C→T→C) pass even if the constant is
> wrong. The correctness of the constant must be verified against an external reference such
> as `iauHd2ae` or by comparing the decoded TDV azimuth directly against a ground-truth
> encoder position. See `test/alignment/test_alignment_plugins.cpp`:
> `SPK_AltAz_Goto_Convention_Regression` for the test that verifies this convention.

### Time and Sidereal Synchronization

All plugins rely on **libnova** to determine Local Sidereal Time (LST) to maintain tight integration and backward compatibility with the INDI driver ecosystem.

The SPK plugin also needs the ERFA `eraASTROM` astrometry context for its coordinate transforms.  `UpdateAstrometry` populates this struct directly from libnova and the vendored `eraEra00` function, keeping the entire pipeline consistent with a single LST source:

$$ \text{ERA} = \texttt{eraEra00}(JD) $$
$$ \text{eral} = \text{ERA} + \lambda_{rad} \quad \text{(Local Earth Rotation Angle)} $$
$$ \text{EO} = \text{ERA} - \text{GAST}_{\text{libnova}} \quad \text{(Equation of Origins, for APPT} \leftrightarrow \text{CIRS)} $$

---

## 2. Telescope Simulator: Wallace Error Injection and EQUATORIAL_PE

The telescope simulator (`drivers/telescope/telescope_simulator.cpp`) implements a six-term Wallace pointing model via `scopesim_helper.cpp`. This lets you inject known errors into the simulator and verify that alignment plugins recover them.

### Mount Model Parameters

Six error terms are exposed as the `MOUNT_MODEL` property (UI label "Mount Model", Simulator tab). Values are stored and displayed in **arcminutes**; the driver divides by 60 before passing degrees to `setCorrections()`.

| Property element | Wallace term | Physical meaning |
| :--- | :--- | :--- |
| `MM_IH` | IH | HA zero / roll index error |
| `MM_ID` | ID | Dec zero / pitch index error |
| `MM_CH` | CH | Collimation (non-perpendicularity of optical and mechanical axes) |
| `MM_NP` | NP | HA/Dec non-perpendicularity |
| `MM_MA` | MA | Polar axis azimuth error |
| `MM_ME` | ME | Polar axis elevation error |

Settings are saved across sessions and applied immediately on connection via `ISGetProperties`.

### Wallace Model Implementation

`scopesim_helper.cpp` provides two directions of the Wallace transform:

- **`instrumentToObserved`** (encoder → true sky): `observedHA = instrumentHA + IH`, plus cone, non-perpendicularity, and polar-axis corrections. Called from `mountToApparentRaDec` to compute where the scope physically points.
- **`observedToInstrument`** (true sky → encoder): the inverse, used by `apparentHaDecToMount` during Goto to convert a commanded sky position to encoder targets.


### EQUATORIAL_PE Publishing

Every `ReadScopeStatus` tick the telescope simulator computes the true pointing position (with all Wallace errors applied) and publishes it as the `EQUATORIAL_PE` property (`IP_RO`, state `Ok`):

```
EQUATORIAL_PE / RA_PE   — true pointing RA  (JNow hours)
EQUATORIAL_PE / DEC_PE  — true pointing Dec (JNow degrees)
```

`EQUATORIAL_EOD_COORD` is published separately as the raw encoder position (no Wallace correction), which is what the INDI alignment subsystem and goto logic use.

### CCD/Guide Simulator Snoop

Both `indi_simulator_ccd` and `indi_simulator_guide` are built with `-DUSE_EQUATORIAL_PE`. They register a snoop for `EQUATORIAL_PE` from the active telescope device during `initProperties`.

`ISSnoopDevice` handles the snoop:
1. Parses `RA_PE` / `DEC_PE` from the incoming XML element.
2. Ignores the initial `defNumberVector` (state `Idle`, values 0/0) — only activates on state `Ok`.
3. Converts JNow → J2000 via `INDI::ObservedToJ2000`.
4. Stores `raPE` / `decPE` (J2000 hours / degrees) and sets `usePE = true`.
5. Logs a one-time `INFO` message on activation.

`DrawCcdFrame` then renders the star field centred on `raPE`/`decPE` instead of the raw `EQUATORIAL_EOD_COORD` position, making the Wallace errors visible to the plate solver.

---

## 4. Mathematical Philosophies

The Math Subsystem operates two distinct classes of plugin solvers, each excelling in different deployment scenarios.

### A. Matrix-Based Pointing (SVD & BuiltIn)

These matrix plugins treat pointing as a geometric mapping problem, without requiring explicit parameterization of telescope hardware constraints (like bent tubes or drooping vertical axes). They solve a transformation matrix $T$ that takes a true Celestial Vector ($A$) and maps it to a mechanical Mount Vector ($B$).

Because a single transformation matrix cannot account for physical flexure across the full sky, **these plugins rely on a highly effective piecewise local mesh:**
1. The `BasicMathPlugin` engine connects all alignment points into a Delaunay triangle mesh.
2. When slewing to a target, the subsystem finds the 3 nearest alignment points surrounding the target.
3. The plugin generates a highly localized matrix just for that small patch of sky, which organically absorbs local hardware flexure without needing to mathematically model the physics.

**The Convex Hull Consideration:** This local mesh strategy works best when the target is inside the "boundary" or Convex Hull of all alignment points. Slewing significantly outside the boundary requires the subsystem to extrapolate, which can reduce relative accuracy.

#### BuiltIn Plugin (Algebraic Inversion)
Given 3 local alignment points, it does raw inversion: $T = B \cdot A^{-1}$
> [!NOTE]
> Because BuiltIn optimizes for an exact fit to the three points, $T$ may mathematically skew or scale the local coordinate space to force the triangle to match. This can occasionally affect precision if the mechanical axes are heavily misaligned.

#### SVD Plugin (Markley's Method / Kabsch)
Given the same 3 local alignment points, SVD applies rigid-body constraints. It uses Singular Value Decomposition to calculate the "best-fit" matrix $T$ while demanding that $T$ remains a **pure rigid 3D rotation** (an orthogonal matrix with determinant = $+1$). This protects the geometry of the coordinate sphere while solving both Index Errors and Polar Misalignment simultaneously.

---

### B. Physics-Based Pointing (SPK)

The SPK plugin is built on Patrick Wallace's SPK pointing kernel library. Instead of mapping localized sky patches, it models the entire sky using a unified physical parameterization, estimating the mechanical traits of the mount directly.

It fits 6 terms whose names differ by mount type:

| # | Equatorial term | Altazimuth term | Physical meaning |
| :- | :--- | :--- | :--- |
| 0 | `IH` | `IA` | Roll (Az/HA) index error |
| 1 | `ID` | `IE` | Pitch (Dec/El) index error |
| 2 | `ME` | `AN` | Polar/azimuth-axis north tilt |
| 3 | `MA` | `AW` | Polar/azimuth-axis east tilt |
| 4 | `CH` | `CA` | Collimation / non-perpendicularity |
| 5 | `TF` | `TF` | Tube flexure (vertical droop) |

#### Progression Constraints

Because it uses a unified physics model, SPK requires sufficient data points to differentiate between similar mechanical errors. The plugin enforces the following progression to maintain mathematical stability:

| Sync points | Terms fitted | Terms |
| :--- | :--- | :--- |
| 1–2 | 2 | IH/IA, ID/IE |
| 3–4 | 4 | + ME/AN, MA/AW |
| 5 | 5 | + CH/CA |
| 6+ | 6 | + TF (full model) |

---

## 5. Implementation Specifics (SPK/Physics)

The SPK model involves a few specific implementation abstractions to interface with the rest of INDI:

### SPK Transformation Pipeline

Transformations occur via a "Virtual Telescope" model.

1. `TransformCelestialToTelescope(RA, Dec)` → `Vector`
   * Calls `UpdateAstrometry` to refresh the time-dependent astrometric state (`m_Ast`) for the current Julian Date before each transform.  `UpdateAstrometry` populates the `eraASTROM` struct directly from libnova's GAST and the vendored `eraEra00` — it does not call `spkAstr` or `eraApco13`.
   * Takes the target coordinate and calls `spkVtel` in AXES mode twice (two-pass loop).
   * Because tube flexure depends on the current tracking elevation, but the plugin operates statelessly, the first pass calculates a hypothetical mount position (using $0,0,0$). The second pass feeds that result back in to calculate a refined, self-consistent tube droop correction.
   * Output `soln[0]` is the SPK roll (altaz: `π − Az`; equatorial: `−HA`). `RollPitchToDirectionVector` converts this to a TDV.

2. `TransformTelescopeToCelestial(Vector)` → `(RA, Dec)`
   * Also calls `UpdateAstrometry` to refresh `m_Ast` before solving.
   * `DirectionVectorToRollPitch` converts the TDV to SPK roll/pitch.
   * Calls `spkVtel` in TARG mode with those axis values to recover the celestial direction.

3. Pmfit (pointing model fit, called from `Initialise`)
   * `ExtractObsRow` recovers the encoder roll/pitch from each sync point's stored TDV, then converts roll back to SOFA Az (`rdem = π − roll`) so `Bfun('A')` sees the correct coordinate.
   * `BuildObservationData` applies the same conversion for the batch-fit path.

### Hot Path / Cold Path

`Initialise` uses a two-path design to keep per-sync-point cost low once a full model is established.

**Cold path** — used on the first fit, after a mount-type change, or as a fallback: calls `Pmfit` directly over all sync points. This is O(n) in the number of points and rebuilds the model from scratch.

**Hot path** — active once a 6-term model exists: each new sync point calls `AccumulateObsRow` to update a set of pre-built normal equation accumulators (`m_NE_A`, `m_NE_v`), then `SolveNormalEquations` to extract updated coefficients. The point history is not re-traversed, keeping `Initialise` O(1) per added point regardless of how many sync points have accumulated.

If the hot-path accumulation or solve fails for any reason (degenerate geometry, numerical issue), the plugin falls through to the cold path automatically.

### ERFA / SOFA Compatibility Layer

The SPK C sources (`vtel.c`, `pmfit.c`) were natively designed for the SOFA (`iauXxx`) API. Since SOFA has a non-standard restrictive license, INDI uses the open-source **ERFA** fork (which is mathematically identical).

A C-header shim (`libs/alignment/spk/sofa.h`) maps the API at compile time without modifying Wallace's core source:
```c
typedef eraASTROM iauASTROM;
#define iauXxx eraXxx
```

Rather than adding `liberfa` as a new system dependency, INDI vendors a minimal subset of ERFA source files in `libs/alignment/erfa/`.  Only 19 functions are needed at runtime (angle normalization, rotation matrices, Az/El ↔ HA/Dec conversions, ERA, and the CIRS ↔ observed transforms `eraAtioq`/`eraAtoiq`).  The rest of ERFA — particularly the full ICRS precession/nutation pipeline (`eraApco13`, `eraAtciqz`, `eraAticq`) — is either unreachable via `tar.sys = APPT` or replaced by thin wrappers in `erfa_libnova.c` that delegate to libnova, which INDI already depends on.  See `libs/alignment/erfa/README.md` for the full file list and provenance.

#### ERFA Rotation Matrix Convention Note

The internal matrix generator (`spkVtel`) uses `iauRx`, `iauRy`, and `iauRz`. It is important to note that ERFA does **not** follow the standard "active" rotation matrix format.
1. **Passive**: They are "alias" rotations (they rotate the coordinate frame, not the vector). Each matrix is the *transpose* of standard active matrices.
2. **Left-multiplication**: They stack left-to-right rather than right-to-left.

Substituting standard $M^T_{active}$ math engines here will result in polar offset (`AN, AW`) coefficients being applied inversely, which would cause systematic pointing errors.
