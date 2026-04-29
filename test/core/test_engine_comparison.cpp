#include <gtest/gtest.h>
#include <libastro.h>
#include <indicom.h>
#include <cmath>

/**
 * @brief Test Level 1: Mathematical Truth Validation
 */
TEST(EngineComparison, StarDeviation)
{
    // 2020-06-19 08:00 UTC
    double jd = 2459019.833333;
    INDI::IEquatorialCoordinates j2000 = { 20.69053168, 45.28033881 }; // Deneb
    INDI::IEquatorialCoordinates jnow_libnova, jnow_erfa_a, jnow_erfa_b;

    // IMCCE Truth for Deneb at this epoch
    const double RA_TRUTH = 20.70237028;
    const double DEC_TRUTH = 45.35036333;

    auto calc_error = [&](INDI::IEquatorialCoordinates &pos) {
        double cos_dec = std::cos(DEC_TRUTH * M_PI / 180.0);
        double dRA = (pos.rightascension - RA_TRUTH) * 15.0 * 3600.0 * cos_dec;
        double dDec = (pos.declination - DEC_TRUTH) * 3600.0;
        return std::hypot(dRA, dDec);
    };

    // 1. Get legacy result
    INDI::setStellarEngine(INDI::StellarEngine::LIBNOVA);
    INDI::J2000toObserved(&j2000, jd, &jnow_libnova);

    // 2. Get modern results
    INDI::setStellarEngine(INDI::StellarEngine::ERFA_2000A);
    INDI::J2000toObserved(&j2000, jd, &jnow_erfa_a);

    INDI::setStellarEngine(INDI::StellarEngine::ERFA_2000B);
    INDI::J2000toObserved(&j2000, jd, &jnow_erfa_b);

    double err_ln = calc_error(jnow_libnova);
    double err_a  = calc_error(jnow_erfa_a);
    double err_b  = calc_error(jnow_erfa_b);

    // 3. Parity Analysis (A vs B)
    double delta_ab = std::hypot(
        (jnow_erfa_a.rightascension - jnow_erfa_b.rightascension) * 15.0 * 3600.0 * std::cos(DEC_TRUTH * M_PI / 180.0),
        (jnow_erfa_a.declination - jnow_erfa_b.declination) * 3600.0
    );

    GTEST_LOG_(INFO) << "Deneb Absolute Errors:";
    GTEST_LOG_(INFO) << "  libnova:  " << err_ln << " arcsec";
    GTEST_LOG_(INFO) << "  ERFA-A:   " << err_a  << " arcsec";
    GTEST_LOG_(INFO) << "  ERFA-B:   " << err_b  << " arcsec";
    GTEST_LOG_(INFO) << "  A vs B:   " << delta_ab << " arcsec";

    // Success Criteria:
    EXPECT_GT(err_ln, 15.0);
    EXPECT_LT(err_a, 0.1);
    EXPECT_LT(err_b, 0.1);
    EXPECT_LT(delta_ab, 0.001); 
}

TEST(EngineComparison, Reciprocity)
{
    double jd = 2461112.5;
    INDI::IEquatorialCoordinates j2000_in = { 20.69053168, 45.28033881 };
    INDI::IEquatorialCoordinates jnow, j2000_out;

    INDI::setStellarEngine(INDI::StellarEngine::ERFA_2000B);
    
    INDI::J2000toObserved(&j2000_in, jd, &jnow);
    INDI::ObservedToJ2000(&jnow, jd, &j2000_out);

    EXPECT_NEAR(j2000_in.rightascension, j2000_out.rightascension, 0.000001);
    EXPECT_NEAR(j2000_in.declination,    j2000_out.declination,    0.000001);
}

TEST(EngineComparison, PlanetDeviation)
{
    double jd = 2459019.833333;
    INDI::IEquatorialCoordinates mars_libnova, mars_full, mars_indi;

    // JPL Horizons DE440 geocentric apparent (from test/data/planet_golden.json)
    const double RA_TRUTH  = 356.16466 / 15.0;  // hours
    const double DEC_TRUTH = -4.75544;

    auto calc_error = [&](INDI::IEquatorialCoordinates &pos) {
        double cos_dec = std::cos(DEC_TRUTH * M_PI / 180.0);
        double dRA = (pos.rightascension - RA_TRUTH) * 15.0 * 3600.0 * cos_dec;
        double dDec = (pos.declination - DEC_TRUTH) * 3600.0;
        return std::hypot(dRA, dDec);
    };

    // 1. Legacy result (VSOP87)
    INDI::setPlanetaryEngine(INDI::PlanetaryEngine::LIBNOVA);
    INDI::GetPlanetObserved(4, jd, &mars_libnova);

    // 2. Full VSOP2010 (EPH_FULL)
    INDI::setPlanetaryEngine(INDI::PlanetaryEngine::EPH_FULL);
    INDI::GetPlanetObserved(4, jd, &mars_full);

    // 3. Truncated VSOP2010 (EPH_INDI, packed .ictx or full fallback)
    INDI::setPlanetaryEngine(INDI::PlanetaryEngine::EPH_INDI);
    INDI::GetPlanetObserved(4, jd, &mars_indi);

    double error_libnova = calc_error(mars_libnova);
    double error_full    = calc_error(mars_full);
    double error_indi    = calc_error(mars_indi);

    // Delta between EPH_FULL and EPH_INDI (truncation budget)
    double delta_full_indi = std::hypot(
        (mars_full.rightascension - mars_indi.rightascension) * 15.0 * 3600.0 * std::cos(DEC_TRUTH * M_PI / 180.0),
        (mars_full.declination    - mars_indi.declination)    * 3600.0
    );

    GTEST_LOG_(INFO) << "Mars Absolute Error vs JPL Truth (2020):";
    GTEST_LOG_(INFO) << "  libnova:   " << error_libnova << " arcsec";
    GTEST_LOG_(INFO) << "  EPH_FULL:  " << error_full    << " arcsec";
    GTEST_LOG_(INFO) << "  EPH_INDI:  " << error_indi    << " arcsec";
    GTEST_LOG_(INFO) << "  FULL vs INDI delta: " << delta_full_indi << " arcsec";

    EXPECT_GT(error_libnova, 1000.0);
    EXPECT_LT(error_full, 1.0);   // EPH vs DE440 geocentric; ~0.25" observed
    EXPECT_LT(error_indi, 0.04 + error_full);   // truncation adds at most 0.04"
    EXPECT_LT(delta_full_indi, 0.04);
}

TEST(EngineComparison, MoonDeviation)
{
    double jd = 2459019.833333;
    INDI::IEquatorialCoordinates moon_full, moon_indi;

    // JPL Horizons Truth (DE440) for Moon at JD 2459019.833333
    // RA in degrees: 64.16991, Dec: 19.17745
    const double RA_TRUTH  = 64.16991 / 15.0;  // hours
    const double DEC_TRUTH = 19.17745;

    auto calc_error = [&](INDI::IEquatorialCoordinates &pos) {
        double cos_dec = std::cos(DEC_TRUTH * M_PI / 180.0);
        double dRA = (pos.rightascension - RA_TRUTH) * 15.0 * 3600.0 * cos_dec;
        double dDec = (pos.declination - DEC_TRUTH) * 3600.0;
        return std::hypot(dRA, dDec);
    };

    INDI::setPlanetaryEngine(INDI::PlanetaryEngine::EPH_FULL);
    INDI::GetPlanetObserved(3, jd, &moon_full);

    INDI::setPlanetaryEngine(INDI::PlanetaryEngine::EPH_INDI);
    INDI::GetPlanetObserved(3, jd, &moon_indi);

    double error_full = calc_error(moon_full);
    double error_indi = calc_error(moon_indi);

    GTEST_LOG_(INFO) << "Moon Absolute Error vs JPL Truth (2020):";
    GTEST_LOG_(INFO) << "  EPH_FULL:  " << error_full << " arcsec";
    GTEST_LOG_(INFO) << "  EPH_INDI:  " << error_indi << " arcsec";

    // Moon uses ELP/MPP02 (same .ctx in both engines) — results must be identical
    EXPECT_LT(error_full, 20.0);   // ELP/MPP02 geocentric accuracy vs DE440
    EXPECT_NEAR(error_full, error_indi, 0.001);  // same loader, must match
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
