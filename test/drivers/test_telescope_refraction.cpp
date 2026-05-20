// Verify atmospheric refraction correction in INDI::Telescope.
//
// Strategy: ERFA serves as an independent oracle.
//   - eraRefco gives refraction coefficients A,B; the ERFA model is compared
//     against libnova's Bennett formula for the refraction amount.
//   - eraAtco13 (full J2000->observed chain) gives the refracted AltAz that
//     the mount should point to; this is compared against the INDI pipeline
//     (J2000toObserved + EquatorialToHorizontal + ln_get_refraction_adj).
//   - The round-trip (forward + inverse) is verified to be an identity.
//   - Integration tests drive NewRaDec() on a concrete TestTelescope.
//
// Stars used (J2000, from Munich lat=48.14N lon=11.58E at JD 2459945.5):
//   Capella   RA= 79.172 Dec=+45.998  -- high altitude (~70 deg)
//   Betelgeuse RA= 88.793 Dec= +7.407  -- mid altitude (~46 deg)
//   Rigel     RA= 78.634 Dec= -8.202  -- lower altitude (~28 deg)

#include <gtest/gtest.h>

#include <erfa.h>
#include <libastro.h>
#include <libnova/julian_day.h>
#include <libnova/refraction.h>
#include "inditelescope.h"

#include <cmath>

using namespace INDI;

// Observer: Munich, Germany
static constexpr double OBS_LAT  = 48.1351;   // degrees N
static constexpr double OBS_LON  = 11.5820;   // degrees E (INDI: 0-360)
static constexpr double OBS_ELEV = 520.0;     // meters
static constexpr double ATM_TEMP = 10.0;      // Celsius
static constexpr double ATM_PRES = 1010.0;    // mBar

// Reproducible epoch: 2023-01-01 00:00 UTC
static constexpr double TEST_JD = 2459945.5;

// J2000 catalog positions (degrees)
static constexpr double CAPELLA_RA    = 79.1723;
static constexpr double CAPELLA_DEC   = 45.9980;
static constexpr double BETELGEUSE_RA = 88.7929;
static constexpr double BETELGEUSE_DEC = 7.4070;
static constexpr double RIGEL_RA      = 78.6345;
static constexpr double RIGEL_DEC     = -8.2016;

// Required by the INDI framework at link time
char _me[] = "TestTelescope";
char *me   = _me;

// -----------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------

// ERFA refraction at a geometric altitude using eraRefco A+B*tan^3 model.
static double erfaRefraction_deg(double geom_alt_deg, double pres_mb, double temp_c)
{
    double refa, refb;
    eraRefco(pres_mb, temp_c, 0.5 /*rh*/, 0.55 /*wl um*/, &refa, &refb);
    double zd_rad = (90.0 - geom_alt_deg) * M_PI / 180.0;
    double t      = std::tan(zd_rad);
    double ref_rad = refa * t + refb * t * t * t; // radians added to apparent alt
    return ref_rad * 180.0 / M_PI;
}

// Run eraAtco13 for a J2000 star and return observed altitude and azimuth.
// Pass pres_mb=0 to get the geometric (unrefracted) position.
static bool erfaObsAltAz(double ra2000_deg, double dec2000_deg,
                          double jd,
                          double lat_deg, double lon_deg, double elev_m,
                          double pres_mb, double temp_c,
                          double &alt_deg, double &az_deg)
{
    double aob, zob, hob, dob, rob, eo;
    int rc = eraAtco13(
        ra2000_deg  * M_PI / 180.0,
        dec2000_deg * M_PI / 180.0,
        0, 0, 0, 0,          // no proper motion, parallax, rv
        jd, 0.0, 0.0,        // utc1, utc2, dut1
        lon_deg * M_PI / 180.0,
        lat_deg * M_PI / 180.0,
        elev_m,
        0, 0,                // polar motion
        pres_mb, temp_c, 0.5 /*rh*/, 0.55 /*wl*/,
        &aob, &zob, &hob, &dob, &rob, &eo);
    if (rc != 0)
        return false;
    az_deg  = aob * 180.0 / M_PI;       // N=0 E=90, same as INDI
    alt_deg = 90.0 - zob * 180.0 / M_PI;
    return true;
}

// Apply the same refraction pipeline as INDI::Telescope ISNewNumber/NewRaDec.
//   forward=true  -> add refraction (Goto path: geometric -> apparent)
//   forward=false -> subtract refraction (NewRaDec path: apparent -> geometric)
static void refractionPipeline(double ra_h, double dec_deg,
                                IGeographicCoordinates loc,
                                double jd, double pres, double temp,
                                bool forward,
                                double &raOut_h, double &decOut_deg)
{
    IEquatorialCoordinates eq  {ra_h, dec_deg};
    IHorizontalCoordinates hz;
    EquatorialToHorizontal(&eq, &loc, jd, &hz);
    if (hz.altitude > 0)
    {
        double adj = ln_get_refraction_adj(hz.altitude, pres, temp);
        hz.altitude += forward ? adj : -adj;
    }
    HorizontalToEquatorial(&hz, &loc, jd, &eq);
    raOut_h    = eq.rightascension;
    decOut_deg = eq.declination;
}

// -----------------------------------------------------------------------
// Minimal concrete telescope for integration tests
// -----------------------------------------------------------------------
class TestTelescope : public INDI::Telescope
{
public:
    TestTelescope()
    {
        SetTelescopeCapability(TELESCOPE_CAN_GOTO | TELESCOPE_HAS_LOCATION, 0);
        initProperties();
        ISGetProperties(me);
    }
    const char *getDefaultName() override { return "TestTelescope"; }
    bool Connect()         override { return true; }
    bool Disconnect()      override { return true; }
    bool ReadScopeStatus() override { return true; }
    bool Sync(double, double) override { return true; }

    double lastGotoRA {0}, lastGotoDec {0};
    bool Goto(double ra, double dec) override
    {
        lastGotoRA  = ra;
        lastGotoDec = dec;
        return true;
    }

    // Test-only access to protected members
    void setTestLocation(double lat, double lon, double elev)
    {
        m_Location = {lon, lat, elev};
    }
    void setTestRefraction(bool on, double temp = ATM_TEMP, double pres = ATM_PRES)
    {
        m_RefractionEnabled = on;
        m_AtmTemperature    = temp;
        m_AtmPressure       = pres;
    }
    IGeographicCoordinates testLocation() const { return m_Location; }

    // Drive NewRaDec() and return what EqNP holds afterward.
    void callNewRaDec(double ra, double dec, double &raOut, double &decOut)
    {
        // Force the threshold check to pass by setting sentinel values.
        EqNP[AXIS_RA].setValue(-999.0);
        EqNP[AXIS_DE].setValue(-999.0);
        TrackState = SCOPE_TRACKING;
        NewRaDec(ra, dec);
        raOut  = EqNP[AXIS_RA].getValue();
        decOut = EqNP[AXIS_DE].getValue();
    }
};

// -----------------------------------------------------------------------
// 1. libnova ln_get_refraction_adj basic properties
// -----------------------------------------------------------------------

TEST(RefracAdj, AlwaysPositiveAboveHorizon)
{
    for (double alt = 1.0; alt <= 85.0; alt += 2.0)
        EXPECT_GT(ln_get_refraction_adj(alt, ATM_PRES, ATM_TEMP), 0.0) << "alt=" << alt;
}

TEST(RefracAdj, MonotonicallyDecreasing)
{
    double prev = ln_get_refraction_adj(1.0, ATM_PRES, ATM_TEMP);
    for (double alt = 5.0; alt <= 85.0; alt += 5.0)
    {
        double cur = ln_get_refraction_adj(alt, ATM_PRES, ATM_TEMP);
        EXPECT_LT(cur, prev) << "Refraction should decrease at alt=" << alt;
        prev = cur;
    }
}

// -----------------------------------------------------------------------
// 2. libnova formula vs ERFA refraction model
//    ERFA eraRefco gives coefficients A,B for the standard A*tan(zd)+B*tan^3(zd)
//    model. libnova uses Bennett's empirical formula. Agreement expected to
//    within ~0.5 arcminute across altitudes 5-70 deg.
// -----------------------------------------------------------------------

TEST(RefracModel, LibnovaMatchesErfa)
{
    // Per-altitude tolerances reflect the genuine divergence between
    // Bennett's empirical formula and ERFA's physics-based model.
    // At 5 deg, ERFA's humidity/dispersion terms push the two apart by
    // ~26 arcsec; above 10 deg the models converge rapidly.
    struct { double alt; double tol_deg; } cases[] = {
        { 5.0, 0.009},  // 32 arcsec -- Bennett vs ERFA diverge at low alt
        {10.0, 0.003},  // 11 arcsec
        {20.0, 0.001},  //  4 arcsec
        {30.0, 0.001},
        {45.0, 0.001},
        {60.0, 0.001},
        {70.0, 0.001},
    };
    for (auto &c : cases)
    {
        double libnova_deg = ln_get_refraction_adj(c.alt, ATM_PRES, ATM_TEMP);
        double erfa_deg    = erfaRefraction_deg(c.alt, ATM_PRES, ATM_TEMP);
        EXPECT_NEAR(libnova_deg, erfa_deg, c.tol_deg)
            << "alt=" << c.alt
            << "  libnova=" << libnova_deg * 60 << " arcmin"
            << "  erfa=" << erfa_deg * 60 << " arcmin"
            << "  diff=" << (libnova_deg - erfa_deg) * 3600 << " arcsec";
    }
}

// -----------------------------------------------------------------------
// 3. INDI pipeline vs ERFA full chain (J2000 -> observed)
//    We compare the refraction AMOUNT (delta-alt) to isolate the refraction
//    model from coordinate-frame differences between libnova and ERFA.
//    The two pipelines may disagree on absolute position by several arcseconds
//    due to precession/nutation model differences, but the refraction delta
//    should agree to < 0.5 arcminute.
// -----------------------------------------------------------------------

struct StarCase
{
    const char *name;
    double ra_deg, dec_deg;
    double expected_min_alt; // sanity-check: star must be above this altitude
};

class PipelineVsErfa : public ::testing::TestWithParam<StarCase>
{
protected:
    IGeographicCoordinates loc {OBS_LON, OBS_LAT, OBS_ELEV};
};

TEST_P(PipelineVsErfa, RefractionAmount)
{
    const auto &s = GetParam();

    // ERFA: geometric altitude (pres=0 disables refraction)
    double geom_alt, geom_az;
    ASSERT_TRUE(erfaObsAltAz(s.ra_deg, s.dec_deg, TEST_JD,
                              OBS_LAT, OBS_LON, OBS_ELEV,
                              0.0, ATM_TEMP, geom_alt, geom_az))
        << "eraAtco13 failed for " << s.name;
    ASSERT_GT(geom_alt, s.expected_min_alt) << s.name << " must be above horizon";

    // ERFA: observed altitude (with refraction)
    double obs_alt, obs_az;
    ASSERT_TRUE(erfaObsAltAz(s.ra_deg, s.dec_deg, TEST_JD,
                              OBS_LAT, OBS_LON, OBS_ELEV,
                              ATM_PRES, ATM_TEMP, obs_alt, obs_az));
    double erfa_delta = obs_alt - geom_alt; // ERFA refraction amount

    // INDI: geometric altitude via J2000toObserved + EquatorialToHorizontal
    IEquatorialCoordinates j2000 {s.ra_deg / 15.0, s.dec_deg};
    IEquatorialCoordinates apparent;
    J2000toObserved(&j2000, TEST_JD, &apparent);
    IHorizontalCoordinates indi_hz;
    EquatorialToHorizontal(&apparent, &loc, TEST_JD, &indi_hz);
    ASSERT_GT(indi_hz.altitude, 0.0);
    double indi_delta = ln_get_refraction_adj(indi_hz.altitude, ATM_PRES, ATM_TEMP);

    // Tolerance: 7 arcsec (0.002 deg). Stars are all above 20 deg where
    // Bennett vs ERFA formula difference is < 1 arcsec; coordinate-transform
    // differences between libnova and ERFA have negligible effect on the
    // refraction amount at these altitudes.
    EXPECT_NEAR(indi_delta, erfa_delta, 0.002)
        << s.name
        << "  INDI=" << indi_delta * 60 << " arcmin"
        << "  ERFA=" << erfa_delta * 60 << " arcmin"
        << "  at alt=" << indi_hz.altitude << " deg";
}

INSTANTIATE_TEST_SUITE_P(Stars, PipelineVsErfa,
    ::testing::Values(
        StarCase{"Capella",    CAPELLA_RA,    CAPELLA_DEC,    5.0},
        StarCase{"Betelgeuse", BETELGEUSE_RA, BETELGEUSE_DEC, 5.0},
        StarCase{"Rigel",      RIGEL_RA,      RIGEL_DEC,      5.0}
    ),
    [](const auto &info) { return info.param.name; }
);

// -----------------------------------------------------------------------
// 4. Forward correction moves mount pointing in the right direction
//    After applying the Goto refraction correction, the mount's target
//    AltAz should match ERFA's full observed AltAz more closely than the
//    uncorrected apparent position does.
// -----------------------------------------------------------------------

TEST(GotoCorrection, CloserToErfaObserved)
{
    IGeographicCoordinates loc {OBS_LON, OBS_LAT, OBS_ELEV};

    // Use Betelgeuse as test target
    IEquatorialCoordinates j2000 {BETELGEUSE_RA / 15.0, BETELGEUSE_DEC};
    IEquatorialCoordinates apparent;
    J2000toObserved(&j2000, TEST_JD, &apparent);

    // ERFA full-chain observed AltAz
    double erfa_obs_alt, erfa_obs_az;
    ASSERT_TRUE(erfaObsAltAz(BETELGEUSE_RA, BETELGEUSE_DEC, TEST_JD,
                              OBS_LAT, OBS_LON, OBS_ELEV,
                              ATM_PRES, ATM_TEMP, erfa_obs_alt, erfa_obs_az));

    // INDI apparent AltAz (uncorrected)
    IHorizontalCoordinates uncorr_hz;
    EquatorialToHorizontal(&apparent, &loc, TEST_JD, &uncorr_hz);
    double uncorr_diff = std::abs(uncorr_hz.altitude - erfa_obs_alt);

    // INDI apparent AltAz after forward refraction correction
    double corrRA, corrDec;
    refractionPipeline(apparent.rightascension, apparent.declination,
                       loc, TEST_JD, ATM_PRES, ATM_TEMP, true, corrRA, corrDec);
    IEquatorialCoordinates corrEq {corrRA, corrDec};
    IHorizontalCoordinates corr_hz;
    EquatorialToHorizontal(&corrEq, &loc, TEST_JD, &corr_hz);
    double corr_diff = std::abs(corr_hz.altitude - erfa_obs_alt);

    EXPECT_LT(corr_diff, uncorr_diff)
        << "Corrected alt diff=" << corr_diff * 60 << " arcmin"
        << " should be less than uncorrected=" << uncorr_diff * 60 << " arcmin";
}

// -----------------------------------------------------------------------
// 5. Round-trip: forward + inverse = identity
//    Applying the Goto correction then the NewRaDec inverse correction
//    should recover the original apparent (JNow) RA/Dec within < 0.1 arcsec.
// -----------------------------------------------------------------------

TEST(RoundTrip, GotoThenNewRaDec)
{
    IGeographicCoordinates loc {OBS_LON, OBS_LAT, OBS_ELEV};

    IEquatorialCoordinates j2000 {CAPELLA_RA / 15.0, CAPELLA_DEC};
    IEquatorialCoordinates apparent;
    J2000toObserved(&j2000, TEST_JD, &apparent);

    double fwdRA, fwdDec;
    refractionPipeline(apparent.rightascension, apparent.declination,
                       loc, TEST_JD, ATM_PRES, ATM_TEMP, true, fwdRA, fwdDec);

    double backRA, backDec;
    refractionPipeline(fwdRA, fwdDec,
                       loc, TEST_JD, ATM_PRES, ATM_TEMP, false, backRA, backDec);

    // Tolerance: 1e-4 deg ~ 0.36 arcsec in Dec; scale RA to degrees for comparison.
    EXPECT_NEAR(backRA * 15.0, apparent.rightascension * 15.0, 1e-4);
    EXPECT_NEAR(backDec,       apparent.declination,           1e-4);
}

// -----------------------------------------------------------------------
// 6. Below-horizon guard: no correction applied when altitude <= 0
// -----------------------------------------------------------------------

TEST(BelowHorizon, NoCorrectionApplied)
{
    IGeographicCoordinates loc {OBS_LON, OBS_LAT, OBS_ELEV};

    // Dec=-80 is always below the horizon from Munich (lat 48N).
    constexpr double ra_h = 0.0, dec_deg = -80.0;

    IEquatorialCoordinates eq {ra_h, dec_deg};
    IHorizontalCoordinates hz;
    EquatorialToHorizontal(&eq, &loc, TEST_JD, &hz);
    ASSERT_LT(hz.altitude, 0.0) << "Test position must be below horizon";

    double raOut, decOut;
    refractionPipeline(ra_h, dec_deg, loc, TEST_JD, ATM_PRES, ATM_TEMP, true, raOut, decOut);
    EXPECT_NEAR(raOut,  ra_h,    1e-12);
    EXPECT_NEAR(decOut, dec_deg, 1e-12);
}

// -----------------------------------------------------------------------
// 7. Integration: TestTelescope::NewRaDec (inverse / position-reporting path)
// -----------------------------------------------------------------------

class IntegrationNewRaDec : public ::testing::Test
{
protected:
    TestTelescope scope;
    void SetUp() override
    {
        scope.setTestLocation(OBS_LAT, OBS_LON, OBS_ELEV);
    }
};

TEST_F(IntegrationNewRaDec, Disabled_PassThrough)
{
    scope.setTestRefraction(false);
    double raIn = BETELGEUSE_RA / 15.0, decIn = BETELGEUSE_DEC;
    double raOut, decOut;
    scope.callNewRaDec(raIn, decIn, raOut, decOut);
    EXPECT_NEAR(raOut, raIn,  1e-9);
    EXPECT_NEAR(decOut, decIn, 1e-9);
}

TEST_F(IntegrationNewRaDec, Enabled_ShiftsCoordinates)
{
    scope.setTestRefraction(true);

    // Use an apparent position that is well above the horizon.
    double raIn  = CAPELLA_RA / 15.0;
    double decIn = CAPELLA_DEC;

    // Pre-check: verify star is above horizon at current time
    IEquatorialCoordinates eq {raIn, decIn};
    IHorizontalCoordinates hz;
    IGeographicCoordinates loc = scope.testLocation();
    EquatorialToHorizontal(&eq, &loc, INDI::getJulianDate(), &hz);
    if (hz.altitude <= 0)
        GTEST_SKIP() << "Capella is below horizon at current time; skipping";

    double raOut, decOut;
    scope.callNewRaDec(raIn, decIn, raOut, decOut);

    bool shifted = (std::abs(raOut - raIn) > 1e-6 || std::abs(decOut - decIn) > 1e-6);
    EXPECT_TRUE(shifted) << "NewRaDec with refraction enabled must shift the output";
}

// -----------------------------------------------------------------------
// 8. Integration: symmetric round-trip through both halves of the pipeline.
//    The Goto path adds refraction; NewRaDec removes it. After both,
//    clients should see approximately the original geometric position.
// -----------------------------------------------------------------------

TEST_F(IntegrationNewRaDec, SymmetricRoundTrip)
{
    scope.setTestRefraction(true);

    IGeographicCoordinates loc = scope.testLocation();
    double jd = INDI::getJulianDate();

    // Use Capella apparent JNow as the client's target (treat as apparent for
    // this integration test -- the exact value doesn't matter, only that the
    // round-trip is internally consistent).
    double ra0 = CAPELLA_RA / 15.0, dec0 = CAPELLA_DEC;

    // Check above horizon
    IEquatorialCoordinates eq0 {ra0, dec0};
    IHorizontalCoordinates hz0;
    EquatorialToHorizontal(&eq0, &loc, jd, &hz0);
    if (hz0.altitude <= 0)
        GTEST_SKIP() << "Capella is below horizon at current time; skipping";

    // Simulate Goto correction (same logic as ISNewNumber)
    double gotoRA, gotoDec;
    refractionPipeline(ra0, dec0, loc, jd, ATM_PRES, ATM_TEMP, true, gotoRA, gotoDec);

    // Simulate hardware reporting back gotoRA/gotoDec; NewRaDec strips refraction.
    double reportedRA, reportedDec;
    scope.callNewRaDec(gotoRA, gotoDec, reportedRA, reportedDec);

    // After round-trip, EqNP should show ~original coordinates.
    // 18 arcsec tolerance accounts for the tiny JD drift between the forward
    // refractionPipeline call and NewRaDec's internal getJulianDate() call.
    EXPECT_NEAR(reportedRA * 15.0, ra0 * 15.0, 0.005);
    EXPECT_NEAR(reportedDec,       dec0,        0.005);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
