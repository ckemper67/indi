/*******************************************************************************
 * test_libastro_regression.cpp
 *
 * This test captures the behavior of libastro 1.0 (libnova-based).
 *
 * During the migration to libastro 2.0 (ERFA-based), these tests will ensure
 * that the new implementation remains "consistent" with the old one, while
 * explicitly quantifying the accuracy improvement.
 *******************************************************************************/

#include <gtest/gtest.h>
#include <libastro.h>
#include <indicom.h>
#include <cmath>

// Helper to check RA/Dec similarity within a tolerance
#define EXPECT_RA_NEAR(val1, val2, abs_error) \
    EXPECT_NEAR(range24(val1), range24(val2), abs_error)

#define EXPECT_DEC_NEAR(val1, val2, abs_error) \
    EXPECT_NEAR(val1, val2, abs_error)

// ---------------------------------------------------------------------------
// Test Case 1: Standard J2000 to JNow conversion
// ---------------------------------------------------------------------------
TEST(Libastro1Regression, J2000toObserved)
{
    INDI::setEngine(true); // ENABLE ERFA ENGINE
    double jd = 2461112.5; // 2026-03-14 00:00 UTC
    INDI::IEquatorialCoordinates jnow;

    // Deneb
    INDI::IEquatorialCoordinates deneb_j2000 = { 20.69053168, 45.28033881 };
    INDI::J2000toObserved(&deneb_j2000, jd, &jnow);
    EXPECT_RA_NEAR(jnow.rightascension, 20.7050106, 0.000001);
    EXPECT_DEC_NEAR(jnow.declination,    45.3679173, 0.000001);

    // Polaris (High Declination)
    INDI::IEquatorialCoordinates polaris_j2000 = { 2.530303, 89.264111 };
    INDI::J2000toObserved(&polaris_j2000, jd, &jnow);
    EXPECT_RA_NEAR(jnow.rightascension, 3.065834477, 0.000001); 
    EXPECT_DEC_NEAR(jnow.declination,    89.37721178, 0.000001);

    // Canopus (Southern Hemisphere)
    INDI::IEquatorialCoordinates canopus_j2000 = { 6.399197, -52.695667 };
    INDI::J2000toObserved(&canopus_j2000, jd, &jnow);
    EXPECT_RA_NEAR(jnow.rightascension, 6.409158196, 0.000001);
    EXPECT_DEC_NEAR(jnow.declination,    -52.7137917, 0.000001);
}

// ---------------------------------------------------------------------------
// Test Case 2: Standard JNow to J2000 conversion (Reciprocity)
// ---------------------------------------------------------------------------
TEST(Libastro1Regression, ObservedToJ2000)
{
    INDI::setEngine(true);
    double jd = 2461112.5;
    INDI::IEquatorialCoordinates j2000;

    // Deneb JNow baseline
    INDI::IEquatorialCoordinates deneb_jnow  = { 20.7050106, 45.3679173 };
    INDI::ObservedToJ2000(&deneb_jnow, jd, &j2000);
    EXPECT_RA_NEAR(j2000.rightascension, 20.69053168, 0.000001);
    EXPECT_DEC_NEAR(j2000.declination,    45.28033881, 0.000001);
}

// ---------------------------------------------------------------------------
// Test Case 3: Horizontal Coordinates (Multi-Location)
// ---------------------------------------------------------------------------
TEST(Libastro1Regression, EquatorialToHorizontal)
{
    INDI::setEngine(true);
    double jd = 2461112.5;
    INDI::IEquatorialCoordinates deneb_jnow = { 20.7050106, 45.3679173 };
    INDI::IHorizontalCoordinates hrz;

    // Greenwich
    INDI::IGeographicCoordinates greenwich = { -0.1278, 51.5074, 10.0 };
    INDI::EquatorialToHorizontal(&deneb_jnow, &greenwich, jd, &hrz);
    EXPECT_NEAR(hrz.azimuth,  27.550380, 0.000001);
    EXPECT_NEAR(hrz.altitude, 12.809663, 0.000001);

    // Siding Spring, Australia (Southern Hemisphere, East Longitude)
    INDI::IGeographicCoordinates siding_spring = { 149.0661, -31.2770, 1165.0 };
    INDI::EquatorialToHorizontal(&deneb_jnow, &siding_spring, jd, &hrz);
    EXPECT_NEAR(hrz.azimuth,  353.4308967, 0.000001);
    EXPECT_NEAR(hrz.altitude,  12.9072505, 0.000001);

    // Quito, Ecuador (Equatorial)
    INDI::IGeographicCoordinates quito = { -78.4678, -0.1807, 2850.0 };
    INDI::EquatorialToHorizontal(&deneb_jnow, &quito, jd, &hrz);
    EXPECT_NEAR(hrz.azimuth,  328.4193055, 0.000001);
    EXPECT_NEAR(hrz.altitude, -33.5605605, 0.000001);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
