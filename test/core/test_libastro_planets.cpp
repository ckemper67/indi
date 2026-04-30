/*******************************************************************************
 * Tests for libnova planetary position accuracy against JPL Horizons DE440.
 *
 * libnova uses VSOP87 for planets and ELP2000-82B for the Moon. These tests
 * document the accuracy of those theories at two epochs and catch gross
 * regressions (wrong body, wrong epoch, unit errors) which produce errors of
 * many arcminutes.
 *
 * Golden data: test/data/jpl_golden.json
 *   20 cases: all planets + Moon at JD 2459019.833333 (2020-06-19 08:00 UTC)
 *             and at JD 2451545.0 (J2000.0).
 *   Source: JPL Horizons DE440, geocentric astrometric J2000 (QUANTITIES=1).
 *   Regenerate with: python3 tools/generate_golden_files.py
 *******************************************************************************/

#include <gtest/gtest.h>
#include <indijson.hpp>
#include <libnova/mercury.h>
#include <libnova/venus.h>
#include <libnova/solar.h>
#include <libnova/mars.h>
#include <libnova/jupiter.h>
#include <libnova/saturn.h>
#include <libnova/uranus.h>
#include <libnova/neptune.h>
#include <libnova/pluto.h>
#include <libnova/lunar.h>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

struct PlanetCase {
    std::string name;
    double jd;
    double ra_deg_truth;
    double dec_deg_truth;
};

static std::vector<PlanetCase> load_golden()
{
    std::ifstream f(TEST_DATA_DIR "/jpl_golden.json");
    if (!f.is_open()) return {};
    nlohmann::json j = nlohmann::json::parse(f);
    std::vector<PlanetCase> cases;
    for (auto &e : j) {
        PlanetCase c;
        c.name          = e["name"];
        c.jd            = e["jd"];
        c.ra_deg_truth  = e["ra_deg"];
        c.dec_deg_truth = e["dec_deg"];
        cases.push_back(c);
    }
    return cases;
}

static double position_error_arcsec(double ra_deg, double dec_deg,
                                     double ra_truth, double dec_truth)
{
    double cos_dec = std::cos(dec_truth * M_PI / 180.0);
    double dRA  = (ra_deg  - ra_truth)  * 3600.0 * cos_dec;
    double dDec = (dec_deg - dec_truth) * 3600.0;
    return std::hypot(dRA, dDec);
}

// ---------------------------------------------------------------------------
// Accuracy vs JPL DE440 geocentric astrometric J2000
//
// libnova accuracy vs DE440 (VSOP87 for planets, ELP2000-82B for Moon):
//              2020      J2000
//   Mercury:   0.8"      4.2"
//   Venus:     0.8"      3.2"
//   Sun:      22.4"     16.7"
//   Mars:      1.8"      2.1"
//   Jupiter:   0.6"      0.1"
//   Saturn:    0.4"      0.1"
//   Uranus:    1.0"      0.1"
//   Neptune:   1.2"      0.5"
//   Pluto:     0.2"      0.1"
//   Moon:     54.9"     42.6"
//
// The assertion catches gross regressions (wrong body, unit error, epoch
// mismatch) which produce errors of many arcminutes. Per-body errors are logged.
// ---------------------------------------------------------------------------
TEST(LibastroPlanets, AccuracyVsJPL)
{
    static constexpr double TOLERANCE_ARCSEC = 120.0;

    auto cases = load_golden();
    ASSERT_GT(cases.size(), 0u) << "Could not load jpl_golden.json";

    for (auto &c : cases) {
        struct ln_equ_posn posn;
        double ra_deg = 0, dec_deg = 0;

        if (c.name == "Mercury") {
            ln_get_mercury_equ_coords(c.jd, &posn);
        } else if (c.name == "Venus") {
            ln_get_venus_equ_coords(c.jd, &posn);
        } else if (c.name == "Sun") {
            ln_get_solar_equ_coords(c.jd, &posn);
        } else if (c.name == "Mars") {
            ln_get_mars_equ_coords(c.jd, &posn);
        } else if (c.name == "Jupiter") {
            ln_get_jupiter_equ_coords(c.jd, &posn);
        } else if (c.name == "Saturn") {
            ln_get_saturn_equ_coords(c.jd, &posn);
        } else if (c.name == "Uranus") {
            ln_get_uranus_equ_coords(c.jd, &posn);
        } else if (c.name == "Neptune") {
            ln_get_neptune_equ_coords(c.jd, &posn);
        } else if (c.name == "Pluto") {
            ln_get_pluto_equ_coords(c.jd, &posn);
        } else if (c.name == "Moon") {
            ln_get_lunar_equ_coords(c.jd, &posn);
        } else {
            FAIL() << "Unknown body: " << c.name;
        }
        ra_deg  = posn.ra;
        dec_deg = posn.dec;

        double err = position_error_arcsec(ra_deg, dec_deg,
                                            c.ra_deg_truth, c.dec_deg_truth);
        GTEST_LOG_(INFO) << c.name << " @JD" << c.jd
                         << "  err=" << err << "\""
                         << "  (truth RA=" << c.ra_deg_truth/15.0 << "h"
                         << " Dec=" << c.dec_deg_truth << "d"
                         << "  got RA=" << ra_deg/15.0 << "h"
                         << " Dec=" << dec_deg << "d)";

        EXPECT_LT(err, TOLERANCE_ARCSEC) << c.name << " position error vs JPL DE440";
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
