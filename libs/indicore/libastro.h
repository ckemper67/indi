/*
    libastro

    functions used for coordinate conversions, based on libnova

    Copyright (C) 2020 Chris Rowland
    Copyright (C) 2021 Jasem Mutlaq

    This library is free software; you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published
    by the Free Software Foundation; either version 2.1 of the License, or
    (at your option) any later version.

    This library is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
    License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this library; if not, write to the Free Software Foundation,
    Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA

*/

// file libsatro.h

// functions used for coordinate conversions, based on libnova

#pragma once

#include <libnova/utility.h>
#ifdef HAVE_ERFA
#include <erfa.h>
#endif

namespace INDI
{

#define RAD_TO_DEG(rad) ((rad) * 180.0/M_PI)
#define DEG_TO_RAD(deg) ((deg) * M_PI/180.0)

/**
 * \defgroup Position Structures

     Structure for Celestial Equatorial, horizontal, and geographic positions.
 */
/*@{*/

/** \typedef IEquatorialCoordinates
    \brief Celestial Equatorial Coordinates */
typedef struct
{
    double rightascension;  /*!< Right Ascension in Hours (0 to 24)*/
    double declination;     /*!< Delination in degrees (-90 to +90) */
} IEquatorialCoordinates;

/** \typedef IHorizontalCoordinates
    \brief Topocentric Horizontal Coordinates */
typedef struct
{
    double azimuth;  /*!< Azimuth in degrees (0 to 360 eastward. 0 North, 90 East, 180 South, 270 West)*/
    double altitude; /*!< Altitude in degrees (-90 to +90) */
} IHorizontalCoordinates;

/** \typedef IGeographicCoordinates
    \brief Geographic Coordinates */
typedef struct
{
    double longitude; /*!< Longitude in degrees (0 to 360 eastward.)*/
    double latitude;  /*!< Latitude in degrees (-90 to +90) */
    double elevation; /*!< Elevation from Mean Sea Level in meters */
} IGeographicCoordinates;

/*@}*/

/**
 * @brief Frame-tagged subtypes of IEquatorialCoordinates.
 *
 * Zero-cost C++ subtypes — layout-identical to IEquatorialCoordinates.
 * Use these in new code so the compiler rejects mismatched frames at the
 * call site. Existing code using IEquatorialCoordinates* is unaffected.
 */
struct J2000Coordinates    : IEquatorialCoordinates {};  ///< ICRS / J2000.0 catalog position
struct GeocentricApparent  : IEquatorialCoordinates {};  ///< Geocentric CIRS apparent (output of J2000toObserved)
struct TopocentricApparent : IEquatorialCoordinates {};  ///< Topocentric CIRS apparent (parallax-corrected)

/**
 * @brief Observer location and precomputed astrometric context.
 *
 * Set the input fields (observer, dut1, …) once, then pass the same context
 * to multiple J2000toTopocentric / GetPlanetTopocentric calls at the same JD.
 * The engine populates the cached ASTROM on first use and reuses it for every
 * subsequent call at the same JD, avoiding redundant nutation computation.
 *
 * The cached_* and astrom fields are managed by the engine — do not set them
 * directly.
 */
struct AstrometricContext {
    IGeographicCoordinates observer{};  ///< lon (0–360 E deg), lat (deg), elev (m)
    double dut1          = 0.0;         ///< UT1–UTC (seconds)
    double xp            = 0.0;         ///< polar motion x (arcsec)
    double yp            = 0.0;         ///< polar motion y (arcsec)
    double pressure_hPa  = 0.0;         ///< atmospheric pressure (0 = no refraction)
    double temp_C        = 15.0;        ///< temperature (Celsius)
    double humidity      = 0.5;         ///< relative humidity (0–1)
    double wavelength_um = 0.55;        ///< observation wavelength (micron)

    // Tolerance for JD cache reuse (days). The ASTROM components (BPN matrix,
    // aberration vector, observer position) drift slowly enough that 1 minute
    // (1.0/1440.0) keeps all errors below 0.001". Zero means exact match.
    double jd_tolerance = 0.0;

    // Engine-managed cache — do not modify directly; call invalidate() if
    // observer parameters are changed after the first use.
    double                 cached_jd       = 0.0;
    IGeographicCoordinates cached_observer = {};
    bool                   cache_valid     = false;
#ifdef HAVE_ERFA
    eraASTROM astrom{};
    double    eo = 0.0;
#endif

    void invalidate() { cache_valid = false; }
};

/*
* \brief This provides astrometric helper functions
* based on the libnova library
*/

/**
* \brief ObservedToJ2000 converts an observed position to a J2000 catalogue position
*  removes aberration, nutation and precession
* \param observed position
* \param jd Julian day epoch of observed position
* \param J2000pos returns catalogue position
*/
void ObservedToJ2000(IEquatorialCoordinates *observed, double jd, IEquatorialCoordinates *J2000pos);

/**
 * @brief J2000toGeocentric converts a J2000 catalog position to geocentric apparent coordinates.
 * @param j2000 J2000 catalog position (typed)
 * @param jd Julian Date (UTC)
 * @param out Geocentric apparent (CIRS) coordinates
 */
void J2000toGeocentric(J2000Coordinates *j2000, double jd, GeocentricApparent *out);

/**
 * @brief GeocentricToJ2000 converts geocentric apparent coordinates back to J2000.
 * @param apparent Geocentric apparent (CIRS) coordinates
 * @param jd Julian Date (UTC)
 * @param out J2000 catalog position
 */
void GeocentricToJ2000(GeocentricApparent *apparent, double jd, J2000Coordinates *out);

/**
 * @brief J2000toTopocentric converts a J2000 catalog position to topocentric apparent
 * coordinates, applying geocentric parallax and diurnal aberration for the given observer.
 * @param j2000 J2000 catalog position
 * @param ctx Observer context (set observer fields; cache is managed automatically)
 * @param jd Julian Date (UTC)
 * @param out Topocentric apparent (CIRS) coordinates
 */
void J2000toTopocentric(J2000Coordinates *j2000, AstrometricContext &ctx, double jd, TopocentricApparent *out);

/**
 * @brief TopocentricToJ2000 converts topocentric apparent coordinates back to J2000.
 * @param apparent Topocentric apparent (CIRS) coordinates
 * @param ctx Observer context (same context used for the forward transform)
 * @param jd Julian Date (UTC)
 * @param out J2000 catalog position
 */
void TopocentricToJ2000(TopocentricApparent *apparent, AstrometricContext &ctx, double jd, J2000Coordinates *out);

/**
* \brief J2000toObserved converts a J2000 catalogue position to an observed position for the epoch jd
*    applies precession, nutation and aberration
* \param J2000pos J2000 catalogue position
* \param jd Julian day epoch of observed position
* \param observed returns observed position for the JD epoch
*/
void J2000toObserved(IEquatorialCoordinates *J2000pos, double jd, IEquatorialCoordinates * observed);

/**
 * @brief EquatorialToHorizontal Calculate horizontal coordinates from equatorial coordinates.
 * @param object JNow/CIRS apparent coordinates (RA hours, Dec degrees) — output of J2000toObserved.
 *               Do NOT pass J2000 catalogue coordinates directly.
 * @param observer Observer location (longitude 0–360 eastward, latitude, elevation in meters).
 * @param JD Julian Date (UTC).
 * @param position Calculated horizontal coordinates (azimuth 0=N/90=E, altitude degrees).
 */
void EquatorialToHorizontal(IEquatorialCoordinates *object, IGeographicCoordinates *observer, double JD,
                            IHorizontalCoordinates *position);

/**
 * @brief GetPlanetObserved Calculate planetary equatorial coordinates.
 * @param np Body identifier: 1=Mercury, 2=Venus, 3=Moon, 4=Mars, 5=Jupiter, 6=Saturn, 7=Uranus, 8=Neptune, else=Sun.
 * @param jd Julian Date.
 * @param observed Calculated observed coordinates (RA hours, DE degrees).
 */
void GetPlanetObserved(int np, double jd, IEquatorialCoordinates *observed);

/**
 * @brief GetPlanetTopocentric calculates topocentric planetary/lunar equatorial coordinates.
 * @param np Body identifier: 1=Mercury, 2=Venus, 3=Moon, 4=Mars, 5=Jupiter, 6=Saturn, 7=Uranus, 8=Neptune, else=Sun.
 * @param jd Julian Date (UTC).
 * @param ctx Observer context (observer location required; cache managed automatically).
 * @param out Topocentric apparent coordinates (RA hours, Dec degrees).
 */
void GetPlanetTopocentric(int np, double jd, AstrometricContext &ctx, TopocentricApparent *out);

/**
 * @brief HorizontalToEquatorial Calculate JNow/CIRS apparent equatorial coordinates from horizontal coordinates.
 * @param object Horizontal coordinates (azimuth 0=N/90=E, altitude degrees).
 * @param observer Observer location (longitude 0–360 eastward, latitude, elevation in meters).
 * @param JD Julian Date (UTC).
 * @param position Calculated JNow/CIRS apparent coordinates (RA hours, Dec degrees).
 *                 Pass through ObservedToJ2000 to obtain J2000 catalogue coordinates.
 */
void HorizontalToEquatorial(IHorizontalCoordinates *object, IGeographicCoordinates *observer, double JD,
                            IEquatorialCoordinates *position);

/**
 * @brief Typed overloads of EquatorialToHorizontal.
 *
 * Accept geocentric or topocentric CIRS coordinates explicitly, preventing
 * accidental use of raw J2000 coordinates. Both delegate to the same
 * underlying frame rotation.
 */
void EquatorialToHorizontal(const GeocentricApparent *object, IGeographicCoordinates *observer, double JD,
                            IHorizontalCoordinates *position);
void EquatorialToHorizontal(const TopocentricApparent *object, IGeographicCoordinates *observer, double JD,
                            IHorizontalCoordinates *position);

/**
* \brief ln_get_equ_nut applies or removes nutation in place for the epoch JD
* \param posn position, nutation is applied or removed in place
* \param jd
* \param reverse  set to true to remove nutation
* TODO: this exposes ln_equ_posn (a libnova type) in the public API — revisit
* when libnova is fully decoupled from the coordinate engine layer.
*/
void ln_get_equ_nut(ln_equ_posn *posn, double jd, bool reverse = false);

/**
 * @brief Select the coordinate engine to use (legacy Libnova or modern ERFA).
 */
enum class StellarEngine {
    LIBNOVA,
    ERFA_2000A,  // Full IAU 2000A (1,365 terms)
    ERFA_2000B   // Lite IAU 2000B (77 terms) - Default high-precision
};

enum class PlanetaryEngine {
    LIBNOVA,     // VSOP87
    EPH_FULL,    // VSOP2010 Full (140MB)
    EPH_INDI     // VSOP2010 Truncated (~12MB)
};

void setStellarEngine(StellarEngine engine);
void setPlanetaryEngine(PlanetaryEngine engine);

}
