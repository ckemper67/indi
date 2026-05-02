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

// Compares geocentric vs topocentric for stars, Moon, and Mars against
// geocentric truth (IMCCE / JPL DE440) and topocentric truth (JPL DE440,
// observer at Greenwich: 0.0°E, 51.4769°N, 45 m).
//
// Stars (px=0): topocentric differs from geocentric by diurnal aberration
// (~0-0.3") — both are measured against geocentric IMCCE truth, so
// topocentric will appear slightly worse against that benchmark.  That is
// correct: the star golden data has no observer location, so geocentric is
// the right comparison there.
//
// Planets/Moon: topocentric truth is observer-specific.  The table shows
// that geocentric EPH carries the full parallax error vs a surface observer
// (~38' for Moon, ~9" for Mars) while topocentric EPH recovers sub-arcsecond
// accuracy against the topocentric JPL benchmark.
TEST(EngineComparison, TopocentricComparison)
{
    // Observer: Greenwich (lon=0°E, lat=51.4769°N, elev=45 m)
    INDI::AstrometricContext ctx;
    ctx.observer = { 0.0, 51.4769, 45.0 };

    double jd = 2459019.833333;

    INDI::setStellarEngine(INDI::StellarEngine::ERFA_2000B);
    INDI::setPlanetaryEngine(INDI::PlanetaryEngine::EPH_FULL);

    // -----------------------------------------------------------------------
    // Stars: geocentric vs topocentric vs IMCCE geocentric truth
    // -----------------------------------------------------------------------
    {
        std::ifstream f(TEST_DATA_DIR "/star_golden.json");
        ASSERT_TRUE(f.is_open()) << "Could not open star_golden.json";
        nlohmann::json golden = nlohmann::json::parse(f);

        struct StarRow {
            std::string name;
            double err_geo, err_topo, delta_gt;
        };
        std::vector<StarRow> rows;

        for (auto &entry : golden)
        {
            std::string name = entry["name"];
            double jd_star   = entry["jd"];

            INDI::J2000Coordinates input;
            input.rightascension = static_cast<double>(entry["ra_j2obs"]) / 15.0;
            input.declination    = entry["dec_j2obs"];

            double ra_truth  = static_cast<double>(entry["ra"])  / 15.0;
            double dec_truth = entry["dec"];
            double cos_dec   = std::cos(dec_truth * M_PI / 180.0);

            auto calc_error = [&](INDI::IEquatorialCoordinates &pos) {
                double dRA  = (pos.rightascension - ra_truth) * 15.0 * 3600.0 * cos_dec;
                double dDec = (pos.declination    - dec_truth) * 3600.0;
                return std::hypot(dRA, dDec);
            };

            INDI::GeocentricApparent  geo;
            INDI::TopocentricApparent topo;

            ctx.invalidate();
            INDI::J2000toGeocentric(&input, jd_star, &geo);
            INDI::J2000toTopocentric(&input, ctx, jd_star, &topo);

            double err_geo  = calc_error(geo);
            double err_topo = calc_error(topo);
            double delta_gt = std::hypot(
                (geo.rightascension - topo.rightascension) * 15.0 * 3600.0 * cos_dec,
                (geo.declination    - topo.declination)    * 3600.0);

            rows.push_back({name, err_geo, err_topo, delta_gt});
        }

        const std::string SEP = "-----------------------------------------------------------------------";
        GTEST_LOG_(INFO) << "";
        GTEST_LOG_(INFO) << "Star accuracy vs IMCCE geocentric truth (arcsec) — observer: Greenwich";
        GTEST_LOG_(INFO) << "  geo-topo delta = diurnal aberration (~0–0.3\"); truth is geocentric";
        GTEST_LOG_(INFO) << SEP;
        {
            std::ostringstream hdr;
            hdr << std::left  << std::setw(14) << "Star"
                << std::right << std::setw(14) << "geo vs truth"
                              << std::setw(16) << "topo vs truth"
                              << std::setw(16) << "geo-topo delta";
            GTEST_LOG_(INFO) << hdr.str();
        }
        GTEST_LOG_(INFO) << SEP;
        for (auto &r : rows)
        {
            std::ostringstream line;
            line << std::left  << std::setw(14) << r.name
                 << std::right << std::fixed << std::setprecision(4)
                 << std::setw(14) << r.err_geo
                 << std::setw(16) << r.err_topo
                 << std::setw(16) << r.delta_gt;
            GTEST_LOG_(INFO) << line.str();
        }
        GTEST_LOG_(INFO) << SEP;
    }

    // -----------------------------------------------------------------------
    // Moon and Mars: geocentric vs topocentric vs both truth benchmarks
    //
    // Geocentric truth (JPL DE440 geocentric apparent):
    //   Moon: RA=64.16991°  Dec=+19.17745°
    //   Mars: RA=356.16466°  Dec=-4.75544°
    //
    // Topocentric truth (JPL DE440, observer=Greenwich):
    //   Moon: RA=64.53351°  Dec=+18.64394°
    //   Mars: RA=356.16380°  Dec=-4.75770°
    // -----------------------------------------------------------------------
    struct PlanetCase {
        const char *name;
        int   np;
        double geo_ra_truth,  geo_dec_truth;   // JPL geocentric apparent (deg)
        double topo_ra_truth, topo_dec_truth;  // JPL topocentric apparent, Greenwich (deg)
    };
    const PlanetCase cases[] = {
        { "Moon", 3,  64.16991,  19.17745,  64.53351,  18.64394 },
        { "Mars", 4, 356.16466,  -4.75544, 356.16380,  -4.75770 },
    };

    auto sep2 = std::string(86, '-');
    GTEST_LOG_(INFO) << "";
    GTEST_LOG_(INFO) << "Planet accuracy (arcsec) — observer: Greenwich  JD=" << jd;
    GTEST_LOG_(INFO) << sep2;
    {
        std::ostringstream hdr;
        hdr << std::left  << std::setw(6) << "Body"
            << std::right << std::setw(20) << "geo vs geo-truth"
                          << std::setw(22) << "geo vs topo-truth"
                          << std::setw(22) << "topo vs geo-truth"
                          << std::setw(18) << "topo vs topo-truth";
        GTEST_LOG_(INFO) << hdr.str();
    }
    GTEST_LOG_(INFO) << sep2;

    for (auto &c : cases)
    {
        INDI::IEquatorialCoordinates pos_geo;
        INDI::TopocentricApparent    pos_topo;

        ctx.invalidate();
        INDI::GetPlanetObserved(c.np, jd, &pos_geo);
        INDI::GetPlanetTopocentric(c.np, jd, ctx, &pos_topo);

        auto err = [](double ra_deg, double dec_deg,
                      double ra_truth, double dec_truth) {
            double cos_dec = std::cos(dec_truth * M_PI / 180.0);
            double dRA  = (ra_deg  - ra_truth) * 3600.0 * cos_dec;
            double dDec = (dec_deg - dec_truth) * 3600.0;
            return std::hypot(dRA, dDec);
        };

        double ra_geo_deg  = pos_geo.rightascension  * 15.0;
        double ra_topo_deg = pos_topo.rightascension * 15.0;

        double geo_vs_geo   = err(ra_geo_deg,  pos_geo.declination,  c.geo_ra_truth,  c.geo_dec_truth);
        double geo_vs_topo  = err(ra_geo_deg,  pos_geo.declination,  c.topo_ra_truth, c.topo_dec_truth);
        double topo_vs_geo  = err(ra_topo_deg, pos_topo.declination, c.geo_ra_truth,  c.geo_dec_truth);
        double topo_vs_topo = err(ra_topo_deg, pos_topo.declination, c.topo_ra_truth, c.topo_dec_truth);

        std::ostringstream line;
        line << std::left  << std::setw(6) << c.name
             << std::right << std::fixed << std::setprecision(3)
             << std::setw(20) << geo_vs_geo
             << std::setw(22) << geo_vs_topo
             << std::setw(22) << topo_vs_geo
             << std::setw(18) << topo_vs_topo;
        GTEST_LOG_(INFO) << line.str();

        // Geocentric EPH should match JPL geocentric truth
        EXPECT_LT(geo_vs_geo,   1.0) << c.name << " geocentric vs geocentric truth";
        // Topocentric EPH should match JPL topocentric truth
        EXPECT_LT(topo_vs_topo, 1.0) << c.name << " topocentric vs topocentric truth";
    }
    GTEST_LOG_(INFO) << sep2;
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
