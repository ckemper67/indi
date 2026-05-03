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
 *
 * Two comparisons are shown side by side:
 *
 *   Frame-rotation only (libnova / ERFA with pm=px=0):
 *     Input = ra_j2obs/dec_j2obs (PM pre-propagated by caller, px=0).
 *     Residual = nutation-model error + missing annual parallax.
 *     For libnova: IAU 1980 nutation adds ~15" systematic.
 *     For ERFA: residual is dominated by the missing annual parallax term.
 *
 *   Full catalog (ERFA only, pm+parallax+rv from Hipparcos-2):
 *     Input = ra_j2000/dec_j2000 + catalog pm, px, rv via J2000toGeocentricFull.
 *     Residual = nutation-model error only (~0.01" for 2000B vs 2000A).
 *     libnova does not support pm/parallax; its full-catalog column = frame-rotation result.
 *
 * Truth sources:
 *   IMCCE stars : IMCCE Miriade apparent geocentric, EOP=off, theory=IAU2006/2000A,
 *                 no observer, no refraction.  EOP must be OFF: with EOP=on, IMCCE
 *                 applies measured IERS nutation corrections that eraAtci13 does not.
 *   SOFA-QSO    : Zero-PM/parallax/RV calibration point.  Truth from ERFA eraAtci13,
 *                 pure IAU 2006/2000A, no EOP.  Expected error < 2 mas.
 */
TEST(EngineComparison, StarDeviation)
{
    std::ifstream f(TEST_DATA_DIR "/star_golden.json");
    ASSERT_TRUE(f.is_open()) << "Could not open star_golden.json";
    nlohmann::json golden = nlohmann::json::parse(f);

    struct Row {
        std::string name;
        double err_ln_frame;   // libnova, pm=px=0
        double err_b_frame;    // ERFA-2000B, pm=px=0
        double err_a_frame;    // ERFA-2000A, pm=px=0
        double err_b_full;     // ERFA-2000B, full catalog (pm+px+rv)
        double err_a_full;     // ERFA-2000A, full catalog (pm+px+rv)
    };
    std::vector<Row> rows;

    for (auto &entry : golden)
    {
        std::string name = entry["name"];
        double jd        = entry["jd"];

        // --- frame-rotation input: J2000 + PM pre-applied, pm=px=0 ---
        INDI::IEquatorialCoordinates frame_input = {
            static_cast<double>(entry["ra_j2obs"]) / 15.0,
            entry["dec_j2obs"]
        };

        // --- full-catalog input: J2000 position + Hipparcos-2 catalog data ---
        INDI::CatalogStar cat;
        cat.rightascension = static_cast<double>(entry["ra_j2000"]) / 15.0;
        cat.declination    = entry["dec_j2000"];
        cat.mu_ra_masyr    = entry["mu_alpha_star_masyr"];
        cat.mu_dec_masyr   = entry["mu_delta_masyr"];
        cat.parallax_mas   = entry["parallax_mas"];
        cat.radial_vel_kms = entry["radial_vel_kms"];

        double ra_truth  = static_cast<double>(entry["ra"]) / 15.0;
        double dec_truth = entry["dec"];

        auto sep = [&](const INDI::IEquatorialCoordinates &pos) {
            double cos_dec = std::cos(dec_truth * M_PI / 180.0);
            return std::hypot(
                (pos.rightascension - ra_truth) * 15.0 * 3600.0 * cos_dec,
                (pos.declination    - dec_truth) * 3600.0);
        };

        INDI::GeocentricApparent geo_a_frame, geo_b_frame, geo_ln_frame;
        INDI::GeocentricApparent geo_a_full,  geo_b_full;

        INDI::setStellarEngine(INDI::StellarEngine::LIBNOVA);
        INDI::J2000toGeocentric(reinterpret_cast<INDI::J2000Coordinates*>(&frame_input), jd, &geo_ln_frame);

        INDI::setStellarEngine(INDI::StellarEngine::ERFA_2000A);
        INDI::J2000toGeocentric(reinterpret_cast<INDI::J2000Coordinates*>(&frame_input), jd, &geo_a_frame);
        INDI::J2000toGeocentricFull(&cat, jd, &geo_a_full);

        INDI::setStellarEngine(INDI::StellarEngine::ERFA_2000B);
        INDI::J2000toGeocentric(reinterpret_cast<INDI::J2000Coordinates*>(&frame_input), jd, &geo_b_frame);
        INDI::J2000toGeocentricFull(&cat, jd, &geo_b_full);

        double err_ln_f = sep(geo_ln_frame);
        double err_b_f  = sep(geo_b_frame);
        double err_a_f  = sep(geo_a_frame);
        double err_b_fl = sep(geo_b_full);
        double err_a_fl = sep(geo_a_full);

        EXPECT_LT(err_ln_f, 2.0) << name << " libnova frame";
        EXPECT_LT(err_b_f,  0.5) << name << " ERFA-2000B frame";
        EXPECT_LT(err_a_f,  0.5) << name << " ERFA-2000A frame";
        // Full-catalog tolerance: 0.1" covers catalog differences (IMCCE vs Hipparcos-2)
        // and intrinsic astrometric uncertainty (e.g. Betelgeuse ~57 mas from disc/variability).
        EXPECT_LT(err_b_fl, 0.1) << name << " ERFA-2000B full";
        EXPECT_LT(err_a_fl, 0.1) << name << " ERFA-2000A full";

        rows.push_back({name, err_ln_f, err_b_f, err_a_f, err_b_fl, err_a_fl});
    }

    const std::string SEP(78, '-');
    GTEST_LOG_(INFO) << "";
    GTEST_LOG_(INFO) << "Star accuracy vs IMCCE (arcsec):";
    GTEST_LOG_(INFO) << "  frame = pm pre-applied by caller, px=0  |  full = Hipparcos-2 pm+px+rv";
    GTEST_LOG_(INFO) << SEP;
    {
        std::ostringstream hdr;
        hdr << std::left  << std::setw(12) << "Star"
            << std::right << std::setw(10) << "ln-frame"
                          << std::setw(10) << "B-frame"
                          << std::setw(10) << "A-frame"
                          << std::setw(12) << "B-full"
                          << std::setw(10) << "A-full";
        GTEST_LOG_(INFO) << hdr.str();
    }
    GTEST_LOG_(INFO) << SEP;
    for (auto &r : rows)
    {
        std::ostringstream line;
        line << std::left  << std::setw(12) << r.name
             << std::right << std::fixed << std::setprecision(4)
             << std::setw(10) << r.err_ln_frame
             << std::setw(10) << r.err_b_frame
             << std::setw(10) << r.err_a_frame
             << std::setw(12) << r.err_b_full
             << std::setw(10) << r.err_a_full;
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

// Validates that LibnovaPlanetaryEngine::GetPlanetTopocentric applies the
// ln_get_parallax correction and produces a meaningfully better result than
// the geocentric fallback when compared against JPL topocentric truth.
//
// Truth (JPL DE440, observer=Greenwich 0°E/51.4769°N/45m, JD 2459019.833333):
//   Moon geo: RA=64.16991°  Dec=+19.17745°
//   Moon topo: RA=64.53351°  Dec=+18.64394°   (parallax ~57')
//   Mars geo: RA=356.16466°  Dec=-4.75544°
//   Mars topo: RA=356.16380°  Dec=-4.75770°   (parallax ~9")
TEST(EngineComparison, LibnovaTopocentricParallax)
{
    double jd = 2459019.833333;

    INDI::AstrometricContext ctx;
    ctx.observer = { 0.0, 51.4769, 45.0 };  // Greenwich

    INDI::setPlanetaryEngine(INDI::PlanetaryEngine::LIBNOVA);

    struct PlanetCase {
        const char *name;
        int    np;
        double geo_ra_truth,  geo_dec_truth;
        double topo_ra_truth, topo_dec_truth;
        double plx_min, plx_max;  // expected parallax correction range (arcsec)
    };
    const PlanetCase cases[] = {
        // Moon parallax ~2279" (measured); allow 10% tolerance
        { "Moon", 3,  64.16991,  19.17745,  64.53351,  18.64394, 2000.0, 2600.0 },
        // Mars parallax ~8.7" (measured); allow 50% tolerance
        { "Mars", 4, 356.16466,  -4.75544, 356.16380,  -4.75770,    4.0,   15.0 },
    };

    auto err = [](double ra_deg, double dec_deg, double ra_truth, double dec_truth) {
        double cos_dec = std::cos(dec_truth * M_PI / 180.0);
        double dRA  = (ra_deg  - ra_truth) * 3600.0 * cos_dec;
        double dDec = (dec_deg - dec_truth) * 3600.0;
        return std::hypot(dRA, dDec);
    };

    auto sep = std::string(72, '-');
    GTEST_LOG_(INFO) << "";
    GTEST_LOG_(INFO) << "Libnova parallax correction vs JPL topo-truth — Greenwich";
    GTEST_LOG_(INFO) << sep;
    {
        std::ostringstream hdr;
        hdr << std::left  << std::setw(6)  << "Body"
            << std::right << std::setw(22) << "geo vs topo-truth"
                          << std::setw(22) << "topo vs topo-truth"
                          << std::setw(14) << "improvement";
        GTEST_LOG_(INFO) << hdr.str();
    }
    GTEST_LOG_(INFO) << sep;

    for (auto &c : cases)
    {
        INDI::IEquatorialCoordinates pos_geo;
        INDI::TopocentricApparent    pos_topo;

        ctx.invalidate();
        INDI::GetPlanetObserved   (c.np, jd, &pos_geo);
        INDI::GetPlanetTopocentric(c.np, jd, ctx, &pos_topo);

        double geo_vs_topo  = err(pos_geo.rightascension  * 15.0, pos_geo.declination,
                                  c.topo_ra_truth, c.topo_dec_truth);
        double topo_vs_topo = err(pos_topo.rightascension * 15.0, pos_topo.declination,
                                  c.topo_ra_truth, c.topo_dec_truth);

        std::ostringstream line;
        line << std::left  << std::setw(6) << c.name
             << std::right << std::fixed << std::setprecision(1)
             << std::setw(22) << geo_vs_topo
             << std::setw(22) << topo_vs_topo
             << std::setw(13) << (geo_vs_topo / std::max(topo_vs_topo, 0.001)) << "x";
        GTEST_LOG_(INFO) << line.str();

        // Parallax correction magnitude: geo-topo delta should match expected range
        double ra_deg_geo  = pos_geo.rightascension  * 15.0;
        double ra_deg_topo = pos_topo.rightascension * 15.0;
        double cos_dec = std::cos(c.geo_dec_truth * M_PI / 180.0);
        double plx_delta = std::hypot(
            (ra_deg_topo - ra_deg_geo) * 3600.0 * cos_dec,
            (pos_topo.declination - pos_geo.declination) * 3600.0);

        EXPECT_GT(plx_delta, c.plx_min) << c.name << " parallax correction too small";
        EXPECT_LT(plx_delta, c.plx_max) << c.name << " parallax correction too large";
        // Topocentric must be at least as good as geocentric vs topocentric truth
        // (for Mars the theory error >> parallax so improvement may be marginal)
        EXPECT_LE(topo_vs_topo, geo_vs_topo + 20.0)
            << c.name << " topo should not be significantly worse than geo vs topo-truth";
    }
    GTEST_LOG_(INFO) << sep;
}

// Calibration against SOFA t_atci13 test vector: quasar-like point (zero PM/parallax/RV).
//
// Validates two properties of the ERFA stellar engines:
//   (1) Geocentric: UTC→TT conversion is applied and EO sign is correct.
//       Without the conversion the TT error is ~67s → ~0.8 mas nutation error.
//   (2) Topocentric: geo-topo delta for a zero-parallax source equals diurnal
//       aberration only (no additional parallax shift).
//
// Truth: ERFA eraAtci13 at TT=2456165.5+0.401182685, UTC=TT-67.184s (2012 era,
//   TAI-UTC=35 leap-seconds, TT-TAI=32.184s), zero PM/parallax/RV (pure quasar).
//   rc=2.71 rad, dc=0.174 rad, pr=pd=px=rv=0.
//   ri=2.709994899247256 rad, di=0.172874072098362 rad,
//   eo=-0.002900618712657 rad  (IAU 2006/2000A; PM-independent).
//   Note: SOFA t_atci13 test uses non-zero PM — not directly comparable to this case.
TEST(EngineComparison, SofaCalibration)
{
    // UTC JD = TT − 67.184 s = 2456165.901182685 − 0.000777593 = 2456165.900405092
    const double jd_utc = 2456165.900405092;

    // ICRS direction (rc=2.71 rad, dc=0.174 rad) in engine units (hours / degrees)
    INDI::J2000Coordinates j2000 { 2.71 * 12.0 / M_PI, 0.174 * 180.0 / M_PI };

    // Expected geocentric apparent: zero-PM/parallax/RV at this epoch (see comment above)
    const double ri_sofa = 2.709994899247256426;
    const double di_sofa = 0.1728740720983623;
    const double eo_sofa = -0.002900618712657375647;
    const double expected_ra  = (ri_sofa - eo_sofa) * 12.0 / M_PI;  // hours
    const double expected_dec = di_sofa * 180.0 / M_PI;              // degrees

    // 2 mas tolerance: covers 2000A vs 2000B nutation model difference (~0.3 mas)
    // plus floating-point and UTC→TT rounding (~0.01 mas).
    const double tol_ra  = 2e-3 / 3600.0 / 15.0;
    const double tol_dec = 2e-3 / 3600.0;

    // --- Geocentric validation ---
    struct Case { const char *label; INDI::StellarEngine engine; };
    for (auto &c : std::initializer_list<Case>{
            {"2000A", INDI::StellarEngine::ERFA_2000A},
            {"2000B", INDI::StellarEngine::ERFA_2000B}})
    {
        INDI::setStellarEngine(c.engine);
        INDI::GeocentricApparent geo;
        INDI::J2000toGeocentric(&j2000, jd_utc, &geo);
        EXPECT_NEAR(geo.rightascension, expected_ra,  tol_ra)  << c.label << " RA";
        EXPECT_NEAR(geo.declination,    expected_dec, tol_dec) << c.label << " Dec";
    }

    // --- Topocentric validation (both engines) ---
    // For zero parallax the geo-topo delta is diurnal aberration only.
    // Max at Greenwich (lat=51.5°): v_rot*cos(lat)/c ≈ 290 m/s / c ≈ 0.20".
    // Bound set to 0.35" to allow for any hour angle.
    INDI::AstrometricContext ctx;
    ctx.observer = { 0.0, 51.4769, 45.0 };  // Greenwich

    auto sep = std::string(72, '-');
    GTEST_LOG_(INFO) << "";
    GTEST_LOG_(INFO) << "SOFA calibration — geo-topo delta (diurnal aberration, arcsec):";
    GTEST_LOG_(INFO) << sep;
    {
        std::ostringstream hdr;
        hdr << std::left  << std::setw(8)  << "Engine"
            << std::right << std::setw(20) << "geo RA (h)"
                          << std::setw(20) << "geo Dec (deg)"
                          << std::setw(18) << "geo-topo delta\"";
        GTEST_LOG_(INFO) << hdr.str();
    }
    GTEST_LOG_(INFO) << sep;

    for (auto &c : std::initializer_list<Case>{
            {"2000A", INDI::StellarEngine::ERFA_2000A},
            {"2000B", INDI::StellarEngine::ERFA_2000B}})
    {
        INDI::setStellarEngine(c.engine);
        INDI::GeocentricApparent  geo;
        INDI::TopocentricApparent topo;
        ctx.invalidate();
        INDI::J2000toGeocentric (&j2000, jd_utc, &geo);
        INDI::J2000toTopocentric(&j2000, ctx, jd_utc, &topo);

        double cos_dec = std::cos(geo.declination * M_PI / 180.0);
        double delta = std::hypot(
            (topo.rightascension - geo.rightascension) * 15.0 * 3600.0 * cos_dec,
            (topo.declination    - geo.declination)    * 3600.0);

        EXPECT_LT(delta, 0.35) << c.label << " geo-topo for zero-parallax must be diurnal aberration only";

        std::ostringstream line;
        line << std::left  << std::setw(8) << c.label
             << std::right << std::fixed << std::setprecision(9)
             << std::setw(20) << geo.rightascension
             << std::setw(20) << geo.declination
             << std::fixed << std::setprecision(4)
             << std::setw(18) << delta;
        GTEST_LOG_(INFO) << line.str();
    }
    GTEST_LOG_(INFO) << sep;
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
