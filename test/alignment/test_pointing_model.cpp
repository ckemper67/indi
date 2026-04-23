/*******************************************************************************
 Copyright(c) 2026 Christian Kemper. All rights reserved.

 Integration test for the SPK pointing model: inject a known error (any of the
 6 Wallace terms) into a synthetic mount, generate FITS images with real star
 positions, plate-solve them with astrometry.net, feed the solutions as
 alignment sync points into the SPK math plugin, and verify that the corrected
 positions recover the injected error to within 3 arcminutes. Covers both
 equatorial (EQ_GEM) and AltAz mount types. Tests are skipped if `gsc` or
 `solve-field` are not found in PATH.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.
 .
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.
 .
 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.
*******************************************************************************/

#include <gtest/gtest.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <utility>

#include <indilogger.h>

#include <alignment/SPKMathPlugin.h>
#include <alignment/InMemoryDatabase.h>
#include <alignment/TelescopeDirectionVectorSupportFunctions.h>
#include "../../drivers/telescope/scopesim_helper.h"
#include "fits_generator.h"

#include <libnova/sidereal_time.h>
#include <libnova/julian_day.h>
#include <libnova/transform.h>

using namespace INDI::AlignmentSubsystem;

// ---------------------------------------------------------------------------
// Test configuration
// ---------------------------------------------------------------------------
static constexpr double PIXEL_SCALE_ARCSEC = 25.0;  // arcsec/pixel — "short focal" telescope (~8.9° × 7.1° field)
static constexpr int    IMG_WIDTH          = 1280;
static constexpr int    IMG_HEIGHT         = 1024;
static constexpr double TEST_LAT           = 51.5;    // London, degrees N
static constexpr double TEST_LON           = -0.1;    // degrees E
static constexpr double TEST_JD            = 2461113.81667; // 2026-03-15 07:36 UTC
static constexpr double RECOVERY_TOL_DEG   = 0.05;    // ~3 arcminutes

// Sky positions that constrain all 6 pointing-model terms:
//   - span HA roughly -4h to +5h  → constrains MA and ME (sin/cos HA terms)
//   - span Dec -40° to +70°       → constrains CH (sec Dec) and NP (tan Dec)
static const std::vector<std::pair<double, double>> SYNC_POINTINGS =
{
    {  60.0,  30.0 },   // HA ~ -4h,  Dec +30
    { 150.0,  10.0 },   // HA ~ -1h,  Dec +10  (low Dec, constrains CH)
    { 210.0,  50.0 },   // HA ~ +2h,  Dec +50  (high Dec, constrains NP)
    { 300.0, -20.0 },   // HA ~ +5h,  Dec -20
    { 180.0, -40.0 },   // meridian,  Dec -40  (southern sky)
    { 120.0,  70.0 },   // HA ~ -2h,  Dec +70  (very high Dec, isolates NP/MA)
};

// Test point NOT used for fitting (blind verification)
static const std::pair<double, double> VERIFY_POINT = { 90.0, 20.0 };

// ---------------------------------------------------------------------------
// AltAz mount test configuration
//
// Sync positions are specified as {azimuth_deg, altitude_deg} using INDI
// convention (Az=0 is North, 90=East).  Chosen to span all four quadrants
// and two elevation bands so all 5 AltAz model terms are well constrained:
//   IA  — azimuth index error  (uniform Az offset)
//   IE  — elevation index error (uniform El offset)
//   CA  — collimation error     (grows as sec(El); needs low-El positions)
//   AN  — N-S tilt of Az axis   (sin(Az)*tan(El) pattern; needs N and S)
//   AW  — E-W tilt of Az axis   (cos(Az)*tan(El) pattern; needs E and W)
// ---------------------------------------------------------------------------
static const std::vector<std::pair<double, double>> ALTAZ_SYNC_POINTINGS =
{
    {   0.0, 30.0 },  // Due N, low   — constrains AN
    {   0.0, 60.0 },  // Due N, high  — constrains AN at different elevation
    {  90.0, 40.0 },  // Due E        — constrains AW, IE
    { 180.0, 25.0 },  // Due S, low   — constrains CA (large sec(El) contribution)
    { 270.0, 50.0 },  // Due W        — constrains AW, IE
    {  45.0, 55.0 },  // NE diagonal  — cross-term constraint for AN/AW
};

// Blind verification point for AltAz tests (SE direction, moderate elevation)
static const std::pair<double, double> ALTAZ_VERIFY_POINT = { 135.0, 45.0 };

// ---------------------------------------------------------------------------
// Utility: check whether an external binary is available in PATH
// ---------------------------------------------------------------------------
static bool binaryExists(const char *name)
{
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "which %s > /dev/null 2>&1", name);
    return system(cmd) == 0;
}

// ---------------------------------------------------------------------------
// Run solve-field on a FITS file.
//
// Passes approximate centre coordinates and pixel-scale hints.  Parses the
// "Field center: (RA,Dec) = (%f,%f)" line from solve-field stdout.
//
// Returns true on success.
// ---------------------------------------------------------------------------
static bool platesolve(const char *fits_path,
                       double hint_ra_deg, double hint_dec_deg,
                       double pixel_scale_arcsec,
                       double *solved_ra_deg, double *solved_dec_deg)
{
    char output_path[512];
    snprintf(output_path, sizeof(output_path), "%s.solve-output", fits_path);

    // scale hints: half to double the nominal scale
    const double scale_lo = pixel_scale_arcsec * 0.5;
    const double scale_hi = pixel_scale_arcsec * 2.0;

    char cmd[1024];
    // --temp-dir is required: solve-field defaults to /tmp which may be
    // outside the sandbox write allowlist; use $TMPDIR or a known path.
    const char *tmpdir = getenv("TMPDIR");
    if (!tmpdir || tmpdir[0] == '\0') tmpdir = "/tmp";

    snprintf(cmd, sizeof(cmd),
             "mkdir -p '%s' && "
             "solve-field --no-plots --no-verify --overwrite "
             "--ra %.6f --dec %.6f --radius 5 "
             "--scale-low %.3f --scale-high %.3f "
             "--scale-units arcsecperpix "
             "--new-fits none "
             "--temp-dir '%s' "
             "%s > %s 2>&1",
             tmpdir,
             hint_ra_deg, hint_dec_deg,
             scale_lo, scale_hi,
             tmpdir,
             fits_path, output_path);

    int ret = system(cmd);

    // Parse the output regardless of return code — solve-field exits 1 if it
    // did not find a solution.
    FILE *fp = fopen(output_path, "r");
    if (fp == nullptr)
        return false;

    bool found = false;
    char line[256];
    while (fgets(line, sizeof(line), fp) != nullptr)
    {
        float ra_f, dec_f;
        if (sscanf(line, "Field center: (RA,Dec) = (%f,%f)", &ra_f, &dec_f) == 2)
        {
            *solved_ra_deg  = static_cast<double>(ra_f);
            *solved_dec_deg = static_cast<double>(dec_f);
            found = true;
            break;
        }
    }
    fclose(fp);
    return found && ret == 0;
}

// ---------------------------------------------------------------------------
// Core test runner used by every TEST() below.
//
// @param IH_deg  HA index error in degrees
// @param ID_deg  Dec index error in degrees
// @param CH_deg  Collimation error in degrees
// @param NP_deg  Non-perpendicularity in degrees
// @param MA_deg  Polar azimuth error in degrees
// @param ME_deg  Polar elevation error in degrees
// @param label   Short description for diagnostic output
// ---------------------------------------------------------------------------
static void runPointingModelTest(double IH_deg, double ID_deg,
                                  double CH_deg, double NP_deg,
                                  double MA_deg, double ME_deg,
                                  const char *label)
{
    // Skip if external tools are absent
    if (!binaryExists("query-starkd") || !binaryExists("solve-field"))
        GTEST_SKIP() << "query-starkd or solve-field not found in PATH — skipping";

    INDI::Logger::getInstance().configure("", INDI::Logger::file_off,
                                          INDI::Logger::DBG_DEBUG,
                                          INDI::Logger::DBG_DEBUG);

    // Setup mount error generator
    ::Alignment generator;
    generator.mountType = ::Alignment::EQ_GEM;
    generator.latitude  = Angle(TEST_LAT);
    generator.longitude = Angle(TEST_LON);
    generator.setCorrections(IH_deg, ID_deg, CH_deg, NP_deg, MA_deg, ME_deg);

    // Compute Local Sidereal Time at the test epoch
    double gmst_hrs = ln_get_apparent_sidereal_time(TEST_JD);
    double lst_hrs  = std::fmod(gmst_hrs + TEST_LON / 15.0, 24.0);
    if (lst_hrs < 0.0) lst_hrs += 24.0;

    // Setup alignment database
    InMemoryDatabase db;
    db.SetDatabaseReferencePosition(TEST_LAT, TEST_LON);

    SPKMathPlugin plugin;
    plugin.SetApproximateMountAlignment(NORTH_CELESTIAL_POLE);
    auto *pSupport = dynamic_cast<TelescopeDirectionVectorSupportFunctions *>(&plugin);
    ASSERT_NE(pSupport, nullptr);

    // Collect sync points
    for (size_t i = 0; i < SYNC_POINTINGS.size(); ++i)
    {
        const double true_ra_deg  = SYNC_POINTINGS[i].first;
        const double true_dec_deg = SYNC_POINTINGS[i].second;

        // Compute true HA from LST and catalog RA
        double ha_val = lst_hrs - true_ra_deg / 15.0;
        if (ha_val < -12.0) ha_val += 24.0;
        if (ha_val >  12.0) ha_val -= 24.0;

        Angle ha_obs(ha_val, Angle::HOURS);
        Angle dec_obs(true_dec_deg);

        // Apply mount error: true sky → mount encoder position
        Angle ha_inst, dec_inst;
        generator.observedToInstrument(ha_obs, dec_obs, &ha_inst, &dec_inst);

        // Mount-reported RA (hours) = LST - HA_instrument
        double mount_ra_hrs = lst_hrs - ha_inst.Degrees() / 15.0;
        if (mount_ra_hrs < 0.0)  mount_ra_hrs += 24.0;
        if (mount_ra_hrs >= 24.0) mount_ra_hrs -= 24.0;

        // Generate a FITS image centred on the TRUE sky position
        const char *tmpdir_env = getenv("TMPDIR");
        if (!tmpdir_env || tmpdir_env[0] == '\0') tmpdir_env = "/tmp";
        char fits_path[256];
        snprintf(fits_path, sizeof(fits_path),
                 "%s/test_pointing_%s_%zu.fits", tmpdir_env, label, i);

        ASSERT_TRUE(generateFITS(true_ra_deg, true_dec_deg,
                                  PIXEL_SCALE_ARCSEC,
                                  IMG_WIDTH, IMG_HEIGHT,
                                  fits_path))
            << "generateFITS failed for pointing " << i
            << " RA=" << true_ra_deg << " Dec=" << true_dec_deg;

        // Plate-solve the image to recover the true sky position
        double solved_ra_deg, solved_dec_deg;
        ASSERT_TRUE(platesolve(fits_path,
                                true_ra_deg, true_dec_deg,
                                PIXEL_SCALE_ARCSEC,
                                &solved_ra_deg, &solved_dec_deg))
            << "plate solve failed for pointing " << i;

        GTEST_LOG_(INFO) << label
                         << "  point " << i
                         << "  true=(" << true_ra_deg / 15.0 << "h, " << true_dec_deg << "°)"
                         << "  solved=(" << solved_ra_deg / 15.0 << "h, " << solved_dec_deg << "°)"
                         << "  mount encoder=(" << mount_ra_hrs << "h, " << dec_inst.Degrees() << "°)";

        // Add sync point: catalog = plate-solved position, telescope = encoder
        AlignmentDatabaseEntry entry;
        entry.ObservationJulianDate = TEST_JD;
        entry.RightAscension        = solved_ra_deg / 15.0;   // hours
        entry.Declination           = solved_dec_deg;          // degrees

        // Encode instrument RA/Dec as RA-based TDV (ANTI_CLOCKWISE),
        // matching the convention used by all alignment plugins.
        INDI::IEquatorialCoordinates raCoords = { mount_ra_hrs, dec_inst.Degrees() };
        entry.TelescopeDirection =
            pSupport->TelescopeDirectionVectorFromEquatorialCoordinates(raCoords);

        db.GetAlignmentDatabase().push_back(entry);
    }

    // Fit the pointing model
    ASSERT_TRUE(plugin.Initialise(&db))
        << "SPKMathPlugin::Initialise failed for " << label;

    // Verify on a position NOT used for fitting
    const double test_ra_hrs = VERIFY_POINT.first / 15.0;
    const double test_dec_deg = VERIFY_POINT.second;

    TelescopeDirectionVector tdv;
    ASSERT_TRUE(plugin.TransformCelestialToTelescope(test_ra_hrs, test_dec_deg, 0.0, tdv))
        << "TransformCelestialToTelescope failed";

    double out_ra_hrs, out_dec_deg;
    plugin.TransformTelescopeToCelestial(tdv, out_ra_hrs, out_dec_deg);

    GTEST_LOG_(INFO) << label
                     << "  verify: expected=(" << test_ra_hrs << "h, " << test_dec_deg << "°)"
                     << "  recovered=(" << out_ra_hrs << "h, " << out_dec_deg << "°)"
                     << "  delta_ra=" << (out_ra_hrs - test_ra_hrs) * 3600.0 << "\""
                     << "  delta_dec=" << (out_dec_deg - test_dec_deg) * 3600.0 << "\"";

    EXPECT_NEAR(test_ra_hrs,  out_ra_hrs,  RECOVERY_TOL_DEG)
        << label << ": RA not recovered within " << RECOVERY_TOL_DEG * 60.0 << " arcmin";
    EXPECT_NEAR(test_dec_deg, out_dec_deg, RECOVERY_TOL_DEG)
        << label << ": Dec not recovered within " << RECOVERY_TOL_DEG * 60.0 << " arcmin";
}

// ---------------------------------------------------------------------------
// AltAz pointing-model test runner.
//
// Parameters use the Wallace AltAz notation.  The scopesim_helper applies
// them via the same correction() function used for equatorial mounts, with
// azimuth playing the role of HA and altitude playing the role of Dec:
//   IA (az index)      — passed as ih
//   IE (el index)      — passed as id
//   CA (collimation)   — passed as ch
//   NP (non-perp)      — passed as np  (not fitted by SPK AltAz model)
//   AN (N-S tilt)      — passed as ma
//   AW (E-W tilt)      — passed as me
//
// Sync positions are defined as (Az, El) in degrees (INDI convention,
// Az=0 North); they are converted to (RA, Dec) via libnova for FITS
// generation and plate-solving.
// ---------------------------------------------------------------------------
static void runAltAzPointingModelTest(double IA_deg, double IE_deg,
                                       double CA_deg, double NP_deg,
                                       double AN_deg, double AW_deg,
                                       const char *label)
{
    if (!binaryExists("query-starkd") || !binaryExists("solve-field"))
        GTEST_SKIP() << "query-starkd or solve-field not found in PATH -- skipping";

    INDI::Logger::getInstance().configure("", INDI::Logger::file_off,
                                          INDI::Logger::DBG_DEBUG,
                                          INDI::Logger::DBG_DEBUG);

    ::Alignment generator;
    generator.mountType = ::Alignment::ALTAZ;
    generator.latitude  = Angle(TEST_LAT);
    generator.longitude = Angle(TEST_LON);
    // For AltAz: IA=ih, IE=id, CA=ch, NP=np, AN=ma, AW=me
    generator.setCorrections(IA_deg, IE_deg, CA_deg, NP_deg, AN_deg, AW_deg);

    double gmst_hrs = ln_get_apparent_sidereal_time(TEST_JD);
    double lst_hrs  = std::fmod(gmst_hrs + TEST_LON / 15.0, 24.0);
    if (lst_hrs < 0.0) lst_hrs += 24.0;

    InMemoryDatabase db;
    db.SetDatabaseReferencePosition(TEST_LAT, TEST_LON);

    SPKMathPlugin plugin;
    plugin.SetApproximateMountAlignment(ZENITH);
    auto *pSupport = dynamic_cast<TelescopeDirectionVectorSupportFunctions *>(&plugin);
    ASSERT_NE(pSupport, nullptr);

    for (size_t i = 0; i < ALTAZ_SYNC_POINTINGS.size(); ++i)
    {
        const double true_az_deg  = ALTAZ_SYNC_POINTINGS[i].first;
        const double true_el_deg  = ALTAZ_SYNC_POINTINGS[i].second;

        // Convert Az/El to RA/Dec at TEST_JD for FITS generation
        ln_hrz_posn hrz  = { true_az_deg, true_el_deg };
        ln_lnlat_posn obs = { TEST_LON, TEST_LAT };
        ln_equ_posn equ;
        ln_get_equ_from_hrz(&hrz, &obs, TEST_JD, &equ);
        const double true_ra_deg  = equ.ra;
        const double true_dec_deg = equ.dec;

        // Compute HA from LST and RA
        double ha_val = lst_hrs - true_ra_deg / 15.0;
        if (ha_val < -12.0) ha_val += 24.0;
        if (ha_val >  12.0) ha_val -= 24.0;

        Angle ha_obs(ha_val, Angle::HOURS);
        Angle dec_obs(true_dec_deg);

        // Apply mount error: true HA/Dec -> instrument Az/Alt
        Angle primary, secondary;
        generator.apparentHaDecToMount(ha_obs, dec_obs, &primary, &secondary);
        // primary = instrument Az (INDI convention, 0=N), secondary = instrument Alt

        // Generate FITS centred on the true sky position
        const char *tmpdir_env = getenv("TMPDIR");
        if (!tmpdir_env || tmpdir_env[0] == '\0') tmpdir_env = "/tmp";
        char fits_path[256];
        snprintf(fits_path, sizeof(fits_path),
                 "%s/test_pointing_altaz_%s_%zu.fits", tmpdir_env, label, i);

        ASSERT_TRUE(generateFITS(true_ra_deg, true_dec_deg,
                                  PIXEL_SCALE_ARCSEC,
                                  IMG_WIDTH, IMG_HEIGHT,
                                  fits_path))
            << "generateFITS failed for pointing " << i
            << " Az=" << true_az_deg << " El=" << true_el_deg;

        double solved_ra_deg, solved_dec_deg;
        ASSERT_TRUE(platesolve(fits_path,
                                true_ra_deg, true_dec_deg,
                                PIXEL_SCALE_ARCSEC,
                                &solved_ra_deg, &solved_dec_deg))
            << "plate solve failed for pointing " << i;

        GTEST_LOG_(INFO) << label
                         << "  point " << i
                         << "  az=" << true_az_deg << " el=" << true_el_deg
                         << "  true=(" << true_ra_deg / 15.0 << "h, " << true_dec_deg << "deg)"
                         << "  solved=(" << solved_ra_deg / 15.0 << "h, " << solved_dec_deg << "deg)"
                         << "  encoder az=" << primary.Degrees() << "deg alt=" << secondary.Degrees() << "deg";

        AlignmentDatabaseEntry entry;
        entry.ObservationJulianDate = TEST_JD;
        entry.RightAscension        = solved_ra_deg / 15.0;
        entry.Declination           = solved_dec_deg;

        INDI::IHorizontalCoordinates instrCoords = { primary.Degrees(), secondary.Degrees() };
        entry.TelescopeDirection = pSupport->TelescopeDirectionVectorFromAltitudeAzimuth(instrCoords);
        entry.PrivateDataSize = 0;

        db.GetAlignmentDatabase().push_back(entry);
    }

    ASSERT_TRUE(plugin.Initialise(&db))
        << "SPKMathPlugin::Initialise failed for " << label;

    // Convert blind verify point (Az/El) to RA/Dec
    ln_hrz_posn hrz_v  = { ALTAZ_VERIFY_POINT.first, ALTAZ_VERIFY_POINT.second };
    ln_lnlat_posn obs_v = { TEST_LON, TEST_LAT };
    ln_equ_posn equ_v;
    ln_get_equ_from_hrz(&hrz_v, &obs_v, TEST_JD, &equ_v);
    const double test_ra_hrs  = equ_v.ra / 15.0;
    const double test_dec_deg = equ_v.dec;

    TelescopeDirectionVector tdv;
    ASSERT_TRUE(plugin.TransformCelestialToTelescope(test_ra_hrs, test_dec_deg, 0.0, tdv))
        << "TransformCelestialToTelescope failed";

    double out_ra_hrs, out_dec_deg;
    plugin.TransformTelescopeToCelestial(tdv, out_ra_hrs, out_dec_deg);

    GTEST_LOG_(INFO) << label
                     << "  verify: expected=(" << test_ra_hrs << "h, " << test_dec_deg << "deg)"
                     << "  recovered=(" << out_ra_hrs << "h, " << out_dec_deg << "deg)"
                     << "  delta_ra=" << (out_ra_hrs - test_ra_hrs) * 3600.0 << "\""
                     << "  delta_dec=" << (out_dec_deg - test_dec_deg) * 3600.0 << "\"";

    EXPECT_NEAR(test_ra_hrs,  out_ra_hrs,  RECOVERY_TOL_DEG)
        << label << ": RA not recovered within " << RECOVERY_TOL_DEG * 60.0 << " arcmin";
    EXPECT_NEAR(test_dec_deg, out_dec_deg, RECOVERY_TOL_DEG)
        << label << ": Dec not recovered within " << RECOVERY_TOL_DEG * 60.0 << " arcmin";
}

// ---------------------------------------------------------------------------
// Test cases — one per error term, plus a combined test
// ---------------------------------------------------------------------------

TEST(PointingModel, PolarAzimuth_5arcmin)
{
    runPointingModelTest(0, 0, 0, 0, 5.0 / 60.0, 0, "MA5");
}

TEST(PointingModel, PolarElevation_5arcmin)
{
    runPointingModelTest(0, 0, 0, 0, 0, 5.0 / 60.0, "ME5");
}

TEST(PointingModel, PolarCombined_5_3_arcmin)
{
    runPointingModelTest(0, 0, 0, 0, 5.0 / 60.0, 3.0 / 60.0, "MA5ME3");
}

TEST(PointingModel, HAIndexError_5arcmin)
{
    runPointingModelTest(5.0 / 60.0, 0, 0, 0, 0, 0, "IH5");
}

TEST(PointingModel, DecIndexError_5arcmin)
{
    runPointingModelTest(0, 5.0 / 60.0, 0, 0, 0, 0, "ID5");
}

TEST(PointingModel, CollimationError_5arcmin)
{
    runPointingModelTest(0, 0, 5.0 / 60.0, 0, 0, 0, "CH5");
}

TEST(PointingModel, NonPerpendicularity_3arcmin)
{
    runPointingModelTest(0, 0, 0, 3.0 / 60.0, 0, 0, "NP3");
}

TEST(PointingModel, AllTermsCombined)
{
    runPointingModelTest(3.0 / 60.0, 2.0 / 60.0, 4.0 / 60.0,
                          2.0 / 60.0, 5.0 / 60.0, 3.0 / 60.0, "all6");
}

// ---------------------------------------------------------------------------
// AltAz mount test cases — one per model term, plus a combined test.
//
// Parameters: IA (az index), IE (el index), CA (collimation),
//             NP (non-perp, not fitted), AN (N-S tilt), AW (E-W tilt)
// ---------------------------------------------------------------------------

TEST(PointingModel, AltAz_AzimuthIndexError_5arcmin)
{
    runAltAzPointingModelTest(5.0 / 60.0, 0, 0, 0, 0, 0, "IA5");
}

TEST(PointingModel, AltAz_ElevationIndexError_5arcmin)
{
    runAltAzPointingModelTest(0, 5.0 / 60.0, 0, 0, 0, 0, "IE5");
}

TEST(PointingModel, AltAz_CollimationError_5arcmin)
{
    runAltAzPointingModelTest(0, 0, 5.0 / 60.0, 0, 0, 0, "CA5");
}

TEST(PointingModel, AltAz_NorthMisalignment_5arcmin)
{
    runAltAzPointingModelTest(0, 0, 0, 0, 5.0 / 60.0, 0, "AN5");
}

TEST(PointingModel, AltAz_WestMisalignment_5arcmin)
{
    runAltAzPointingModelTest(0, 0, 0, 0, 0, 5.0 / 60.0, "AW5");
}

TEST(PointingModel, AltAz_AllTermsCombined)
{
    runAltAzPointingModelTest(3.0 / 60.0, 2.0 / 60.0, 4.0 / 60.0,
                               2.0 / 60.0, 5.0 / 60.0, 3.0 / 60.0, "altaz_all6");
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
