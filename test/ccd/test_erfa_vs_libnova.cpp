/*******************************************************************************
 * test_erfa_vs_libnova.cpp
 *
 * Measures the difference between ERFA and libnova for computing equinox-based
 * apparent RA/Dec from an ICRS J2000 catalog position.
 *
 * ERFA uses the IAU 2006/2000A precession-nutation model.  libnova uses an
 * older, simplified model.  The delta reported by these tests is the headline
 * number for the case to migrate INDI's coordinate layer from libnova to ERFA.
 *******************************************************************************/

#include <gtest/gtest.h>
#include <erfa.h>
#include <libastro.h>
#include <cmath>

static constexpr double DEG2RAD = M_PI / 180.0;
static constexpr double RAD2DEG = 180.0 / M_PI;

/**
 * Compute equinox-based apparent RA/Dec (degrees) from ICRS J2000 coords.
 * Mirrors the erfaApparentPlace() helper in ccd_simulator.cpp exactly.
 */
static void erfa_apparent(double ra_deg, double dec_deg, double jd_utc,
                           double lon_deg, double lat_deg, double alt_m,
                           double *ra_app_deg, double *dec_app_deg)
{
    double utc1 = std::floor(jd_utc) + 0.5;
    double utc2 = jd_utc - utc1;
    double aob, zob, hob, dob, rob, eo;
    eraAtco13(ra_deg * DEG2RAD, dec_deg * DEG2RAD,
              0.0, 0.0, 0.0, 0.0,      // pm, parallax, rv
              utc1, utc2, 0.0,          // UTC split, DUT1=0
              lon_deg * DEG2RAD, lat_deg * DEG2RAD, alt_m,
              0.0, 0.0,                 // polar motion
              0.0, 15.0, 0.0, 0.55,    // phpa=0 (no refraction)
              &aob, &zob, &hob, &dob, &rob, &eo);
    *ra_app_deg  = eraAnp(rob - eo) * RAD2DEG;
    *dec_app_deg = dob * RAD2DEG;
}

/**
 * Compute equinox-based apparent RA/Dec (degrees) via the libnova path used
 * by INDI (INDI::J2000toObserved from libastro).
 */
static void libnova_apparent(double ra_deg, double dec_deg, double jd_utc,
                              double *ra_app_deg, double *dec_app_deg)
{
    INDI::IEquatorialCoordinates j2000 { ra_deg / 15.0, dec_deg };
    INDI::IEquatorialCoordinates app   { 0.0, 0.0 };
    INDI::J2000toObserved(&j2000, jd_utc, &app);
    *ra_app_deg  = app.rightascension * 15.0;
    *dec_app_deg = app.declination;
}

// Fixed test epoch: 2026-03-14 00:00 UTC, observer at Greenwich sea level.
static constexpr double TEST_JD  = 2461112.5;
static constexpr double TEST_LON =   -0.1278;  // degrees east
static constexpr double TEST_LAT =   51.5074;  // degrees north
static constexpr double TEST_ALT =   10.0;     // metres

// Helper: compute and log ERFA vs libnova delta, return total angular error
static double measureDelta(const char *label,
                            double ra_icrs, double dec_icrs)
{
    double ra_erfa,  dec_erfa;
    double ra_ln,    dec_ln;

    erfa_apparent(ra_icrs, dec_icrs, TEST_JD,
                  TEST_LON, TEST_LAT, TEST_ALT,
                  &ra_erfa, &dec_erfa);
    libnova_apparent(ra_icrs, dec_icrs, TEST_JD,
                     &ra_ln, &dec_ln);

    // RA delta in arcsec — account for cos(dec) compression
    double cos_dec    = std::cos(dec_erfa * DEG2RAD);
    double delta_ra   = (ra_erfa  - ra_ln)  * 3600.0 * cos_dec;
    double delta_dec  = (dec_erfa - dec_ln) * 3600.0;
    double total      = std::hypot(delta_ra, delta_dec);

    GTEST_LOG_(INFO) << label
                     << "  dRA=" << delta_ra  << " arcsec"
                     << "  dDec=" << delta_dec << " arcsec"
                     << "  |total|=" << total  << " arcsec";
    return total;
}

// ---------------------------------------------------------------------------
// Deneb (alpha Cygni): northern sky, Dec ~+45, negligible proper motion
// Hipparcos HIP 102098: RA=310.35797523 Dec=+45.28033881 (J2000 degrees)
// ---------------------------------------------------------------------------
TEST(ErfaVsLibnova, Deneb)
{
    double total = measureDelta("Deneb", 310.35797523, 45.28033881);
    // ERFA and libnova must differ — confirms both are doing real work
    EXPECT_GT(total, 0.1);
    // Sanity: difference should be sub-arcminute (models agree to ~arcsec level)
    EXPECT_LT(total, 60.0);
}

// ---------------------------------------------------------------------------
// Fomalhaut (alpha PsA): southern sky, Dec ~-30, negligible proper motion
// Hipparcos HIP 113368: RA=344.41269169 Dec=-29.62223703 (J2000 degrees)
// ---------------------------------------------------------------------------
TEST(ErfaVsLibnova, Fomalhaut)
{
    double total = measureDelta("Fomalhaut", 344.41269169, -29.62223703);
    EXPECT_GT(total, 0.1);
    EXPECT_LT(total, 60.0);
}

// ---------------------------------------------------------------------------
// Betelgeuse (alpha Ori): equatorial, Dec ~+7, small proper motion
// Hipparcos HIP 27989: RA=88.79293899 Dec=+7.40706318 (J2000 degrees)
// ---------------------------------------------------------------------------
TEST(ErfaVsLibnova, Betelgeuse)
{
    double total = measureDelta("Betelgeuse", 88.79293899, 7.40706318);
    EXPECT_GT(total, 0.1);
    EXPECT_LT(total, 60.0);
}

// ---------------------------------------------------------------------------
// Canopus (alpha Car): southern sky, Dec ~-52, negligible proper motion
// Hipparcos HIP 30438: RA=95.98796 Dec=-52.69566 (J2000 degrees)
// ---------------------------------------------------------------------------
TEST(ErfaVsLibnova, Canopus)
{
    double total = measureDelta("Canopus", 95.98796, -52.69566);
    EXPECT_GT(total, 0.1);
    EXPECT_LT(total, 60.0);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
