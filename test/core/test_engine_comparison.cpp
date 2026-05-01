#include <gtest/gtest.h>
#include <libastro.h>
#include <indicom.h>
#include <nlohmann/json.hpp>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

/**
 * @brief Test Level 1: Mathematical Truth Validation
 *
 * Data-driven: iterates over test/data/star_golden.json.
 * For each star reports error (arcsec) vs IMCCE apparent truth, then prints
 * a summary table: rows = stars, columns = engines.
 */
TEST(EngineComparison, StarDeviation)
{
    std::ifstream f(TEST_DATA_DIR "/star_golden.json");
    ASSERT_TRUE(f.is_open()) << "Could not open star_golden.json";
    nlohmann::json golden = nlohmann::json::parse(f);

    struct Row { std::string name; double err_ln, err_a, err_b, delta_ab; };
    std::vector<Row> rows;

    for (auto &entry : golden)
    {
        std::string name   = entry["name"];
        double jd          = entry["jd"];
        // ICRS J2000.0 in degrees -> RA in hours for IEquatorialCoordinates
        INDI::IEquatorialCoordinates j2000 = {
            static_cast<double>(entry["ra_j2000"]) / 15.0,
            entry["dec_j2000"]
        };
        double ra_truth  = static_cast<double>(entry["ra"])  / 15.0; // hours
        double dec_truth = entry["dec"];

        auto calc_error = [&](INDI::IEquatorialCoordinates &pos) {
            double cos_dec = std::cos(dec_truth * M_PI / 180.0);
            double dRA  = (pos.rightascension - ra_truth) * 15.0 * 3600.0 * cos_dec;
            double dDec = (pos.declination    - dec_truth) * 3600.0;
            return std::hypot(dRA, dDec);
        };

        INDI::IEquatorialCoordinates jnow_ln, jnow_a, jnow_b;

        INDI::setStellarEngine(INDI::StellarEngine::LIBNOVA);
        INDI::J2000toObserved(&j2000, jd, &jnow_ln);

        INDI::setStellarEngine(INDI::StellarEngine::ERFA_2000A);
        INDI::J2000toObserved(&j2000, jd, &jnow_a);

        INDI::setStellarEngine(INDI::StellarEngine::ERFA_2000B);
        INDI::J2000toObserved(&j2000, jd, &jnow_b);

        double err_ln = calc_error(jnow_ln);
        double err_a  = calc_error(jnow_a);
        double err_b  = calc_error(jnow_b);
        double delta_ab = std::hypot(
            (jnow_a.rightascension - jnow_b.rightascension) * 15.0 * 3600.0 * std::cos(dec_truth * M_PI / 180.0),
            (jnow_a.declination    - jnow_b.declination) * 3600.0
        );

        // J2000toObserved does not apply proper motion.  IMCCE propagates PM
        // from the Hipparcos epoch (J1991.25) through the observation date, so
        // the residual vs IMCCE is dominated by PM × 20yr (J2000→J2020).
        // per-star tolerance = PM contribution + 0.2" margin for frame rotation.
        double mu_a = entry["mu_alpha_star_masyr"];
        double mu_d = entry["mu_delta_masyr"];
        constexpr double DT_YR = 20.0; // J2000 -> observation epoch
        double pm_arcsec = std::hypot(mu_a * DT_YR, mu_d * DT_YR) / 1000.0;
        double tol_erfa    = pm_arcsec + 0.2;
        double tol_libnova = pm_arcsec + 2.0;

        // libnova < tol: nutation fix in local_ln_get_equ_nut (DEG_TO_RAD arg parenthesization)
        // made libnova accurate; a large libnova error would indicate that regression.
        EXPECT_LT(err_ln, tol_libnova) << name << " libnova";
        EXPECT_LT(err_a,  tol_erfa)    << name << " ERFA-2000A";
        EXPECT_LT(err_b,  tol_erfa)    << name << " ERFA-2000B";
        EXPECT_LT(delta_ab, 0.001)     << name << " A vs B delta";

        rows.push_back({name, err_ln, err_a, err_b, delta_ab});
    }

    // Print summary table
    GTEST_LOG_(INFO) << "";
    GTEST_LOG_(INFO) << "Star accuracy vs IMCCE apparent truth (arcsec):";
    GTEST_LOG_(INFO) << "------------------------------------------------------------";
    GTEST_LOG_(INFO) << std::left
                     << std::setw(14) << "Star"
                     << std::setw(12) << "libnova"
                     << std::setw(12) << "ERFA-2000A"
                     << std::setw(12) << "ERFA-2000B"
                     << "A vs B";
    GTEST_LOG_(INFO) << "------------------------------------------------------------";
    for (auto &r : rows)
    {
        std::ostringstream line;
        line << std::left  << std::setw(14) << r.name
             << std::right << std::fixed << std::setprecision(4)
             << std::setw(10) << r.err_ln << "  "
             << std::setw(10) << r.err_a  << "  "
             << std::setw(10) << r.err_b  << "  "
             << std::setw(10) << r.delta_ab;
        GTEST_LOG_(INFO) << line.str();
    }
    GTEST_LOG_(INFO) << "------------------------------------------------------------";
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

// Validates EPH_FULL and EPH_INDI against JPL DE440 geocentric truth across
// 11 epochs from 2000 to 2100 (every ~10 years) for Mars, Jupiter, Saturn, Moon.
// Guards against time-dependent failures introduced by the time-weighted packing filter.
TEST(EngineComparison, MultiEpochDeviation)
{
    std::ifstream f(TEST_DATA_DIR "/multi_epoch_golden.json");
    ASSERT_TRUE(f.is_open()) << "Could not open multi_epoch_golden.json";
    nlohmann::json golden = nlohmann::json::parse(f);

    double max_err_full = 0, max_err_indi = 0, max_delta = 0;
    int n = 0;

    for (auto& entry : golden) {
        std::string planet = entry["planet"];
        int np      = entry["np"];
        double jd   = entry["jd"];
        double ra_truth  = static_cast<double>(entry["ra_deg"]) / 15.0;  // hours
        double dec_truth = entry["dec_deg"];

        INDI::IEquatorialCoordinates pos_full, pos_indi;
        INDI::setPlanetaryEngine(INDI::PlanetaryEngine::EPH_FULL);
        INDI::GetPlanetObserved(np, jd, &pos_full);
        INDI::setPlanetaryEngine(INDI::PlanetaryEngine::EPH_INDI);
        INDI::GetPlanetObserved(np, jd, &pos_indi);

        double cos_dec = std::cos(dec_truth * M_PI / 180.0);
        auto err = [&](INDI::IEquatorialCoordinates& pos) {
            double dRA  = (pos.rightascension - ra_truth) * 15.0 * 3600.0 * cos_dec;
            double dDec = (pos.declination    - dec_truth) * 3600.0;
            return std::hypot(dRA, dDec);
        };
        double delta = std::hypot(
            (pos_full.rightascension - pos_indi.rightascension) * 15.0 * 3600.0 * cos_dec,
            (pos_full.declination    - pos_indi.declination)    * 3600.0);

        double ef = err(pos_full), ei = err(pos_indi);
        max_err_full = std::max(max_err_full, ef);
        max_err_indi = std::max(max_err_indi, ei);
        max_delta    = std::max(max_delta, delta);

        EXPECT_LT(ef, 2.0) << planet << " EPH_FULL at JD " << jd;
        EXPECT_LT(ei, 2.0) << planet << " EPH_INDI at JD " << jd;
        EXPECT_LT(delta, 0.1) << planet << " FULL vs INDI delta at JD " << jd;
        n++;
    }

    GTEST_LOG_(INFO) << "Multi-epoch (" << n << " points): "
                     << "max EPH_FULL=" << max_err_full << "\"  "
                     << "max EPH_INDI=" << max_err_indi << "\"  "
                     << "max delta=" << max_delta << "\"";
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
