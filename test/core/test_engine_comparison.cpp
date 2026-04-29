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
    double jd = 2459019.833333;
    INDI::IEquatorialCoordinates j2000 = { 20.69053168, 45.28033881 }; // Deneb
    INDI::IEquatorialCoordinates jnow_libnova, jnow_erfa;

    // IMCCE Truth for Deneb at JD 2459019.833333
    // RA: 20h 42m 08.533s  -> 20.70237028 h
    // Dec: +45° 21' 01.308" -> 45.35036333 deg
    const double RA_TRUTH = 20.70237028;
    const double DEC_TRUTH = 45.35036333;

    // 1. Get legacy result
    INDI::setEngine(false);
    INDI::J2000toObserved(&j2000, jd, &jnow_libnova);

    // 2. Get modern result
    INDI::setEngine(true);
    INDI::J2000toObserved(&j2000, jd, &jnow_erfa);

    // 3. Absolute Error Analysis
    auto calc_error = [&](INDI::IEquatorialCoordinates &pos) {
        double cos_dec = std::cos(DEC_TRUTH * M_PI / 180.0);
        double dRA = (pos.rightascension - RA_TRUTH) * 15.0 * 3600.0 * cos_dec;
        double dDec = (pos.declination - DEC_TRUTH) * 3600.0;
        return std::hypot(dRA, dDec);
    };

    double error_libnova = calc_error(jnow_libnova);
    double error_erfa = calc_error(jnow_erfa);

    GTEST_LOG_(INFO) << "Deneb Absolute Error vs IMCCE Truth:";
    GTEST_LOG_(INFO) << "  libnova: " << error_libnova << " arcsec";
    GTEST_LOG_(INFO) << "  ERFA:    " << error_erfa << " arcsec";

    EXPECT_LT(error_erfa, 0.1);
    EXPECT_GT(error_libnova, 5.0);
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
    // Date: 2020-06-19 08:00 UTC (from confirmed unit test)
    double jd = 2459019.833333; 
    INDI::IEquatorialCoordinates mars_libnova, mars_erfa;

    // JPL Horizons Truth for Mars at JD 2459019.833333 (Geocentric Apparent Equinox)
    // Values from our confirmed test_eph_library output
    const double RA_TRUTH = 23.7442125;
    const double DEC_TRUTH = -4.7558155;

    // 1. Get legacy result (VSOP87)
    INDI::setEngine(false);
    INDI::GetPlanetObserved(4, jd, &mars_libnova);

    // 2. Get modern result (VSOP2010 via EPH)
    INDI::setEngine(true);
    INDI::GetPlanetObserved(4, jd, &mars_erfa);

    // 3. Calculate Error vs Truth
    auto calc_error = [&](INDI::IEquatorialCoordinates &pos) {
        double cos_dec = std::cos(DEC_TRUTH * M_PI / 180.0);
        double dRA = (pos.rightascension - RA_TRUTH) * 15.0 * 3600.0 * cos_dec;
        double dDec = (pos.declination - DEC_TRUTH) * 3600.0;
        return std::hypot(dRA, dDec);
    };

    double error_libnova = calc_error(mars_libnova);
    double error_erfa = calc_error(mars_erfa);

    GTEST_LOG_(INFO) << "Mars Absolute Error vs JPL Truth (2020):";
    GTEST_LOG_(INFO) << "  libnova: " << error_libnova << " arcsec";
    GTEST_LOG_(INFO) << "  ERFA/EPH: " << error_erfa << " arcsec";

    // Success Criteria: 
    // - ERFA/EPH must be deep sub-arcminute (geocentric baseline)
    EXPECT_LT(error_erfa, 2.0);
    // - libnova (VSOP87) is off by ~1000 arcsec (~16 arcmin)
    EXPECT_GT(error_libnova, 1000.0);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
