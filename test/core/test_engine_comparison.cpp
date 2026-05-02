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
 * Input is the PM-propagated ICRS position at the observation epoch (ra_j2obs/dec_j2obs),
 * so PM is cancelled on both sides — residual is pure frame rotation error.
 * Truth is IMCCE apparent (includes PM from J1991.25).
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
        std::string name = entry["name"];
        double jd        = entry["jd"];

        // PM-propagated ICRS input: J2000.0 + PM × (obs_epoch - J2000.0).
        // Feeding this cancels PM against the IMCCE truth so only frame rotation error remains.
        INDI::IEquatorialCoordinates input = {
            static_cast<double>(entry["ra_j2obs"])  / 15.0,  // hours
            entry["dec_j2obs"]
        };
        double ra_truth  = static_cast<double>(entry["ra"])  / 15.0;  // hours
        double dec_truth = entry["dec"];

        auto calc_error = [&](INDI::IEquatorialCoordinates &pos) {
            double cos_dec = std::cos(dec_truth * M_PI / 180.0);
            double dRA  = (pos.rightascension - ra_truth) * 15.0 * 3600.0 * cos_dec;
            double dDec = (pos.declination    - dec_truth) * 3600.0;
            return std::hypot(dRA, dDec);
        };

        INDI::IEquatorialCoordinates jnow_ln, jnow_a, jnow_b;

        INDI::setStellarEngine(INDI::StellarEngine::LIBNOVA);
        INDI::J2000toObserved(&input, jd, &jnow_ln);

        INDI::setStellarEngine(INDI::StellarEngine::ERFA_2000A);
        INDI::J2000toObserved(&input, jd, &jnow_a);

        INDI::setStellarEngine(INDI::StellarEngine::ERFA_2000B);
        INDI::J2000toObserved(&input, jd, &jnow_b);

        double err_ln   = calc_error(jnow_ln);
        double err_a    = calc_error(jnow_a);
        double err_b    = calc_error(jnow_b);
        double delta_ab = std::hypot(
            (jnow_a.rightascension - jnow_b.rightascension) * 15.0 * 3600.0 * std::cos(dec_truth * M_PI / 180.0),
            (jnow_a.declination    - jnow_b.declination)    * 3600.0
        );

        EXPECT_LT(err_a,    0.5)  << name << " ERFA-2000A";
        EXPECT_LT(err_b,    0.5)  << name << " ERFA-2000B";
        EXPECT_LT(err_ln,   2.0)  << name << " libnova";
        EXPECT_LT(delta_ab, 0.01) << name << " A vs B delta";

        rows.push_back({name, err_ln, err_a, err_b, delta_ab});
    }

    const std::string SEP = "------------------------------------------------------------";
    GTEST_LOG_(INFO) << "";
    GTEST_LOG_(INFO) << "Star accuracy vs IMCCE (PM-corrected input, arcsec):";
    GTEST_LOG_(INFO) << SEP;
    {
        std::ostringstream hdr;
        hdr << std::left  << std::setw(14) << "Star"
            << std::right << std::setw(10) << "libnova"
                          << std::setw(12) << "ERFA-2000B"
                          << std::setw(12) << "ERFA-2000A"
                          << std::setw(14) << "B vs libnova"
                          << std::setw(10) << "B vs A";
        GTEST_LOG_(INFO) << hdr.str();
    }
    GTEST_LOG_(INFO) << SEP;
    for (auto &r : rows)
    {
        std::ostringstream line;
        line << std::left  << std::setw(14) << r.name
             << std::right << std::fixed << std::setprecision(4)
             << std::setw(10) << r.err_ln
             << std::setw(12) << r.err_b
             << std::setw(12) << r.err_a
             << std::setw(14) << std::abs(r.err_b - r.err_ln)
             << std::setw(10) << std::abs(r.err_b - r.err_a);
        GTEST_LOG_(INFO) << line.str();
    }
    GTEST_LOG_(INFO) << SEP;
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
    INDI::IEquatorialCoordinates mars_libnova, mars_vsop2013, mars_packed;

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

    // 2. Full VSOP2013
    INDI::setPlanetaryEngine(INDI::PlanetaryEngine::VSOP2013);
    INDI::GetPlanetObserved(4, jd, &mars_vsop2013);

    // 3. VSOP2013 packed (.ictx, fallback to full)
    INDI::setPlanetaryEngine(INDI::PlanetaryEngine::VSOP2013_PACKED);
    INDI::GetPlanetObserved(4, jd, &mars_packed);

    double error_libnova  = calc_error(mars_libnova);
    double error_vsop2013 = calc_error(mars_vsop2013);
    double error_packed   = calc_error(mars_packed);

    double delta = std::hypot(
        (mars_vsop2013.rightascension - mars_packed.rightascension) * 15.0 * 3600.0 * std::cos(DEC_TRUTH * M_PI / 180.0),
        (mars_vsop2013.declination    - mars_packed.declination)    * 3600.0
    );

    GTEST_LOG_(INFO) << "Mars Absolute Error vs JPL Truth (2020):";
    GTEST_LOG_(INFO) << "  libnova:        " << error_libnova  << " arcsec";
    GTEST_LOG_(INFO) << "  VSOP2013:       " << error_vsop2013 << " arcsec";
    GTEST_LOG_(INFO) << "  VSOP2013_PACKED:" << error_packed   << " arcsec";
    GTEST_LOG_(INFO) << "  VSOP2013 vs PACKED delta: " << delta << " arcsec";

    EXPECT_GT(error_libnova, 1000.0);
    EXPECT_LT(error_vsop2013, 1.0);
    EXPECT_LT(error_packed, 0.04 + error_vsop2013);
    EXPECT_LT(delta, 0.04);
}

TEST(EngineComparison, MoonDeviation)
{
    double jd = 2459019.833333;
    INDI::IEquatorialCoordinates moon_vsop2013, moon_packed;

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

    INDI::setPlanetaryEngine(INDI::PlanetaryEngine::VSOP2013);
    INDI::GetPlanetObserved(3, jd, &moon_vsop2013);

    INDI::setPlanetaryEngine(INDI::PlanetaryEngine::VSOP2013_PACKED);
    INDI::GetPlanetObserved(3, jd, &moon_packed);

    double error_vsop2013 = calc_error(moon_vsop2013);
    double error_packed   = calc_error(moon_packed);

    GTEST_LOG_(INFO) << "Moon Absolute Error vs JPL Truth (2020):";
    GTEST_LOG_(INFO) << "  VSOP2013:        " << error_vsop2013 << " arcsec";
    GTEST_LOG_(INFO) << "  VSOP2013_PACKED: " << error_packed   << " arcsec";

    // Moon uses ELP/MPP02 (same .ctx in both engines) — results must be identical
    EXPECT_LT(error_vsop2013, 20.0);
    EXPECT_NEAR(error_vsop2013, error_packed, 0.001);
}

// Validates VSOP2013 and VSOP2013_PACKED against JPL DE440 geocentric truth across
// 11 epochs from 2000 to 2100 (every ~10 years) for Mars, Jupiter, Saturn, Moon.
// Guards against time-dependent failures introduced by the time-weighted packing filter.
TEST(EngineComparison, MultiEpochDeviation)
{
    std::ifstream f(TEST_DATA_DIR "/multi_epoch_golden.json");
    ASSERT_TRUE(f.is_open()) << "Could not open multi_epoch_golden.json";
    nlohmann::json golden = nlohmann::json::parse(f);

    double max_err_vsop = 0, max_err_packed = 0, max_delta = 0;
    int n = 0;

    for (auto& entry : golden) {
        std::string planet = entry["planet"];
        int np      = entry["np"];
        double jd   = entry["jd"];
        double ra_truth  = static_cast<double>(entry["ra_deg"]) / 15.0;  // hours
        double dec_truth = entry["dec_deg"];

        INDI::IEquatorialCoordinates pos_vsop, pos_packed;
        INDI::setPlanetaryEngine(INDI::PlanetaryEngine::VSOP2013);
        INDI::GetPlanetObserved(np, jd, &pos_vsop);
        INDI::setPlanetaryEngine(INDI::PlanetaryEngine::VSOP2013_PACKED);
        INDI::GetPlanetObserved(np, jd, &pos_packed);

        double cos_dec = std::cos(dec_truth * M_PI / 180.0);
        auto err = [&](INDI::IEquatorialCoordinates& pos) {
            double dRA  = (pos.rightascension - ra_truth) * 15.0 * 3600.0 * cos_dec;
            double dDec = (pos.declination    - dec_truth) * 3600.0;
            return std::hypot(dRA, dDec);
        };
        double delta = std::hypot(
            (pos_vsop.rightascension - pos_packed.rightascension) * 15.0 * 3600.0 * cos_dec,
            (pos_vsop.declination    - pos_packed.declination)    * 3600.0);

        double ef = err(pos_vsop), ei = err(pos_packed);
        max_err_vsop   = std::max(max_err_vsop,   ef);
        max_err_packed = std::max(max_err_packed,  ei);
        max_delta      = std::max(max_delta, delta);

        EXPECT_LT(ef, 2.0) << planet << " VSOP2013 at JD " << jd;
        EXPECT_LT(ei, 2.0) << planet << " VSOP2013_PACKED at JD " << jd;
        EXPECT_LT(delta, 0.1) << planet << " VSOP2013 vs PACKED delta at JD " << jd;
        n++;
    }

    GTEST_LOG_(INFO) << "Multi-epoch (" << n << " points): "
                     << "max VSOP2013=" << max_err_vsop << "\"  "
                     << "max VSOP2013_PACKED=" << max_err_packed << "\"  "
                     << "max delta=" << max_delta << "\"";
}

// Validates EphEngineHybrid (TOP2013 for outer planets).
// Compares VSOPTOP2013 against VSOP2013 for outer planets — they should
// agree to within a few hundred arcsec since both implement the same theory era.
// Also verifies inner planets route through the same VSOP2013 path as VSOP2013_PACKED.
TEST(EngineComparison, HybridOuterPlanets)
{
    double jd = 2459019.833333;
    int outer[] = { 5, 6, 7, 8 };
    const char* names[] = { "Jupiter", "Saturn", "Uranus", "Neptune" };

    GTEST_LOG_(INFO) << "VSOPTOP2013 vs VSOP2013 for outer planets (JD 2459019.833333):";
    GTEST_LOG_(INFO) << "  (VSOPTOP2013=TOP2013, VSOP2013=full; should agree to ~100\")";

    for (int i = 0; i < 4; i++) {
        int np = outer[i];

        INDI::IEquatorialCoordinates pos_top, pos_vsop;
        INDI::setPlanetaryEngine(INDI::PlanetaryEngine::VSOPTOP2013);
        INDI::GetPlanetObserved(np, jd, &pos_top);
        INDI::setPlanetaryEngine(INDI::PlanetaryEngine::VSOP2013);
        INDI::GetPlanetObserved(np, jd, &pos_vsop);

        double cos_dec = std::cos(pos_vsop.declination * M_PI / 180.0);
        double delta = std::hypot(
            (pos_top.rightascension - pos_vsop.rightascension) * 15.0 * 3600.0 * cos_dec,
            (pos_top.declination    - pos_vsop.declination)    * 3600.0);

        GTEST_LOG_(INFO) << "  " << names[i]
            << "  VSOPTOP2013 RA=" << std::fixed << std::setprecision(4) << pos_top.rightascension * 15.0
            << " Dec=" << pos_top.declination
            << "  VSOP2013 RA=" << pos_vsop.rightascension * 15.0
            << " Dec=" << pos_vsop.declination
            << "  delta=" << delta << "\"";

        EXPECT_LT(delta, 3600.0) << names[i] << " VSOPTOP2013 vs VSOP2013 exceeds 1 degree";
    }

    // Inner planet (Mars, np=4) must match VSOP2013_PACKED exactly
    INDI::IEquatorialCoordinates mars_top, mars_packed;
    INDI::setPlanetaryEngine(INDI::PlanetaryEngine::VSOPTOP2013);
    INDI::GetPlanetObserved(4, jd, &mars_top);
    INDI::setPlanetaryEngine(INDI::PlanetaryEngine::VSOP2013_PACKED);
    INDI::GetPlanetObserved(4, jd, &mars_packed);

    double mars_delta = std::hypot(
        (mars_top.rightascension - mars_packed.rightascension) * 15.0 * 3600.0,
        (mars_top.declination    - mars_packed.declination)    * 3600.0);
    GTEST_LOG_(INFO) << "  Mars VSOPTOP2013 vs VSOP2013_PACKED delta: " << mars_delta << " arcsec";
    EXPECT_LT(mars_delta, 0.04) << "Mars inner-planet path mismatch";
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
