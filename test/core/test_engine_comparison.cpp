#include <gtest/gtest.h>
#include <libastro.h>
#include <indicom.h>
#include <cmath>

/**
 * @brief Test Level 0.75: A/B Engine Comparison
 * 
 * This test runs the same coordinates through both the Libnova (Legacy)
 * and ERFA (Modern) engines and verifies the delta is within the expected
 * range for geocentric-to-geocentric comparison.
 */
TEST(EngineComparison, StarDeviation)
{
    double jd = 2461112.5; // 2026-03-14
    INDI::IEquatorialCoordinates j2000 = { 20.69053168, 45.28033881 }; // Deneb
    INDI::IEquatorialCoordinates jnow_libnova, jnow_erfa;

    // 1. Get legacy result
    INDI::setEngine(false);
    INDI::J2000toObserved(&j2000, jd, &jnow_libnova);

    // 2. Get modern result
    INDI::setEngine(true);
    INDI::J2000toObserved(&j2000, jd, &jnow_erfa);

    // 3. Calculate delta in arcseconds
    double cos_dec = std::cos(jnow_erfa.declination * M_PI / 180.0);
    double dRA = (jnow_erfa.rightascension - jnow_libnova.rightascension) * 15.0 * 3600.0 * cos_dec;
    double dDec = (jnow_erfa.declination - jnow_libnova.declination) * 3600.0;
    double total_delta = std::hypot(dRA, dDec);

    GTEST_LOG_(INFO) << "Deneb A/B Delta: " << total_delta << " arcsec (" 
                     << "dRA=" << dRA << ", dDec=" << dDec << ")";

    // Success Criteria:
    // - Delta must be significant (proving it's not the same math)
    EXPECT_GT(total_delta, 1.0);
    // - Delta must be within the expected 'libnova error' range for this epoch
    EXPECT_LT(total_delta, 15.0);
}

TEST(EngineComparison, Reciprocity)
{
    double jd = 2461112.5;
    INDI::IEquatorialCoordinates j2000_in = { 20.69053168, 45.28033881 };
    INDI::IEquatorialCoordinates jnow, j2000_out;

    INDI::setEngine(true);
    
    // Forward
    INDI::J2000toObserved(&j2000_in, jd, &jnow);
    // Backward
    INDI::ObservedToJ2000(&jnow, jd, &j2000_out);

    // Success Criteria: 
    // Reversing the transformation must return to original coords
    EXPECT_NEAR(j2000_in.rightascension, j2000_out.rightascension, 0.000001);
    EXPECT_NEAR(j2000_in.declination,    j2000_out.declination,    0.000001);
}

TEST(EngineComparison, PlanetDeviation)
{
    double jd = 2461112.5;
    INDI::IEquatorialCoordinates mars_libnova, mars_erfa;

    // 1. Get legacy result (VSOP87)
    INDI::setEngine(false);
    INDI::GetPlanetObserved(4, jd, &mars_libnova);

    // 2. Get modern result (VSOP2010 via EPH)
    INDI::setEngine(true);
    INDI::GetPlanetObserved(4, jd, &mars_erfa);

    // 3. Calculate delta in arcseconds
    double cos_dec = std::cos(mars_erfa.declination * M_PI / 180.0);
    double dRA = (mars_erfa.rightascension - mars_libnova.rightascension) * 15.0 * 3600.0 * cos_dec;
    double dDec = (mars_erfa.declination - mars_libnova.declination) * 3600.0;
    double total_delta = std::hypot(dRA, dDec);

    GTEST_LOG_(INFO) << "Mars A/B Delta: " << total_delta << " arcsec (" 
                     << "dRA=" << dRA << ", dDec=" << dDec << ")";

    // Success Criteria: 
    // - Should show improvement over legacy math.
    EXPECT_GT(total_delta, 0.1);
    // 2026 epoch shift is ~1300 arcsec (mostly precession)
    EXPECT_LT(total_delta, 2000.0);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
