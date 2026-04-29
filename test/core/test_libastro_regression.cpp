/*******************************************************************************
 * test_libastro_regression.cpp
 *
 * Regression baseline for libastro with ERFA_2000B engine.
 * All expected values were established from the ERFA_2000B implementation.
 *******************************************************************************/

#include <gtest/gtest.h>
#include <libastro.h>
#include <indicom.h>

#define EXPECT_RA_NEAR(val1, val2, abs_error) \
    EXPECT_NEAR(range24(val1), range24(val2), abs_error)

#define EXPECT_DEC_NEAR(val1, val2, abs_error) \
    EXPECT_NEAR(val1, val2, abs_error)

// ---------------------------------------------------------------------------
// Test Case 1: J2000 to JNow conversion (ERFA_2000B baseline)
// ---------------------------------------------------------------------------
TEST(Libastro1Regression, J2000toObserved)
{
    INDI::setStellarEngine(INDI::StellarEngine::ERFA_2000B);
    double jd = 2461112.5; // 2026-03-14 00:00 UTC
    INDI::IEquatorialCoordinates jnow;

    // Deneb
    INDI::IEquatorialCoordinates deneb_j2000 = { 20.69053168, 45.28033881 };
    INDI::J2000toObserved(&deneb_j2000, jd, &jnow);
    EXPECT_RA_NEAR(jnow.rightascension, 20.705011606, 0.000001);
    EXPECT_DEC_NEAR(jnow.declination,   45.369578407, 0.000001);

    // Polaris (High Declination)
    INDI::IEquatorialCoordinates polaris_j2000 = { 2.530303, 89.264111 };
    INDI::J2000toObserved(&polaris_j2000, jd, &jnow);
    EXPECT_RA_NEAR(jnow.rightascension, 3.077078490, 0.000001);
    EXPECT_DEC_NEAR(jnow.declination,   89.378965182, 0.000001);

    // Canopus (Southern Hemisphere)
    INDI::IEquatorialCoordinates canopus_j2000 = { 6.399197, -52.695667 };
    INDI::J2000toObserved(&canopus_j2000, jd, &jnow);
    EXPECT_RA_NEAR(jnow.rightascension, 6.409060800, 0.000001);
    EXPECT_DEC_NEAR(jnow.declination,  -52.714078774, 0.000001);
}

// ---------------------------------------------------------------------------
// Test Case 2: JNow to J2000 roundtrip (reciprocity)
// ---------------------------------------------------------------------------
TEST(Libastro1Regression, ObservedToJ2000)
{
    INDI::setStellarEngine(INDI::StellarEngine::ERFA_2000B);
    double jd = 2461112.5;
    INDI::IEquatorialCoordinates j2000;

    // JNow values are ERFA_2000B output for Deneb at this epoch
    INDI::IEquatorialCoordinates deneb_jnow = { 20.705011606073739, 45.369578406737539 };
    INDI::ObservedToJ2000(&deneb_jnow, jd, &j2000);
    EXPECT_RA_NEAR(j2000.rightascension, 20.69053168, 0.000001);
    EXPECT_DEC_NEAR(j2000.declination,   45.28033881, 0.000001);
}

// ---------------------------------------------------------------------------
// Test Case 3: CIRS to Horizontal (observer-specific, no refraction)
// ---------------------------------------------------------------------------
TEST(Libastro1Regression, EquatorialToHorizontal)
{
    INDI::setStellarEngine(INDI::StellarEngine::ERFA_2000B);
    double jd = 2461112.5;
    INDI::IEquatorialCoordinates deneb_jnow = { 20.705011606073739, 45.369578406737539 };
    INDI::IHorizontalCoordinates hrz;

    // Greenwich
    INDI::IGeographicCoordinates greenwich = { -0.1278, 51.5074, 10.0 };
    INDI::EquatorialToHorizontal(&deneb_jnow, &greenwich, jd, &hrz);
    EXPECT_NEAR(hrz.azimuth,  27.550842, 0.000001);
    EXPECT_NEAR(hrz.altitude, 12.811659, 0.000001);

    // Siding Spring, Australia (Southern Hemisphere, East Longitude)
    INDI::IGeographicCoordinates siding_spring = { 149.0661, -31.2770, 1165.0 };
    INDI::EquatorialToHorizontal(&deneb_jnow, &siding_spring, jd, &hrz);
    EXPECT_NEAR(hrz.azimuth,  353.430006, 0.000001);
    EXPECT_NEAR(hrz.altitude,  12.905442, 0.000001);

    // Quito, Ecuador (Equatorial)
    INDI::IGeographicCoordinates quito = { -78.4678, -0.1807, 2850.0 };
    INDI::EquatorialToHorizontal(&deneb_jnow, &quito, jd, &hrz);
    EXPECT_NEAR(hrz.azimuth,  328.421831, 0.000001);
    EXPECT_NEAR(hrz.altitude, -33.560363, 0.000001);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
