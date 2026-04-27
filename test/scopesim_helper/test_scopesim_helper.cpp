#include <gtest/gtest.h>
#include <gmock/gmock.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <cstdlib>
#include <cstring>

#include <indilogger.h>
#include <indidevapi.h>
#include <defaultdevice.h>
#include <scopesim_helper.h>
#include <telescope_simulator.h>

#include <libnova/julian_day.h>
#include <libnova/lunar.h>
#include <libnova/solar.h>

using ::testing::_;
using ::testing::StrEq;

// Angle tests
TEST(AngleTest, CreateAngle)
{

    Angle a = Angle(60, Angle::DEGREES);
    EXPECT_EQ(a.Degrees(), 60);
    EXPECT_EQ(a.Hours(), 4);
    EXPECT_EQ(a.radians(), 60. * M_PI / 180.);

    a = Angle(1.0, Angle::RADIANS);
    EXPECT_EQ(a.radians(), 1.0);

    a = Angle(2.0, Angle::HOURS);
    EXPECT_EQ(a.Hours(), 2.0);
    EXPECT_EQ(a.Degrees(), 30.0);

    EXPECT_EQ(Angle(180).Degrees(), 180);
    EXPECT_EQ(Angle(-180).Degrees(), 180);
    EXPECT_EQ(Angle(180).Hours(), 12);
    EXPECT_EQ(Angle(-180).HoursHa(), 12);
    EXPECT_EQ(Angle(360).Degrees360(), 0);
    EXPECT_EQ(Angle(-360).Degrees360(), 0);
    EXPECT_EQ(Angle(720).Hours(), 0);
    EXPECT_EQ(Angle(360).HoursHa(), 0);
    EXPECT_EQ(Angle(-345).Hours(), 1);
    EXPECT_EQ(Angle(-345).HoursHa(), 1);
    EXPECT_EQ(Angle(345).Hours(), 23);
    EXPECT_EQ(Angle(345).HoursHa(), -1);
}

TEST(AngleTest, Logic)
{
    Angle a = -60;
    EXPECT_EQ(a.HoursHa(), -4);

    EXPECT_TRUE(a == Angle(-60));
    EXPECT_FALSE(a == Angle(60));
    EXPECT_TRUE(a != Angle(-61));
    EXPECT_FALSE(a != Angle(-60));
    EXPECT_TRUE(a < Angle(119));
    EXPECT_TRUE(a > Angle(121));
    EXPECT_TRUE(Angle(350) < Angle(10));
    EXPECT_FALSE(Angle(351) > Angle(11));
}

TEST(AngleTest, Arithmetic)
{
    Angle a = -60;
    Angle b = 60;
    EXPECT_EQ((a + b).Degrees(), 0);
    EXPECT_EQ((a - b).Degrees(), -120);
    EXPECT_EQ((b - a).Degrees(), 120);

    a += 10;
    EXPECT_EQ(a.Degrees(), -50);
    a += Angle(10);
    EXPECT_EQ(a.Degrees(), -40);

    EXPECT_EQ((b * 0.5).Degrees(), 30);
    EXPECT_EQ(-a.Degrees(), 40);

}

// Vector tests

TEST(VectorTest, Constructors)
{
    Vector v = Vector();
    EXPECT_EQ(v.l(), 0);
    EXPECT_EQ(v.m(), 0);
    EXPECT_EQ(v.n(), 0);

    v = Vector(2, 3, 6);
    EXPECT_EQ(v.l(), 2. / 7);
    EXPECT_EQ(v.m(), 3. / 7);
    EXPECT_EQ(v.n(), 6. / 7);

    v = Vector(Angle(90), Angle(45));
    EXPECT_NEAR(v.l(), 0, 0.00001);
    EXPECT_NEAR(v.m(), 0.707107, 0.00001);
    EXPECT_NEAR(v.n(), 0.707107, 0.00001);
}

TEST(VectorTest, PriSec)
{
    Vector v = Vector(Angle(90), Angle(45));
    EXPECT_NEAR(v.primary().Degrees(), 90, 0.00001);
    EXPECT_NEAR(v.secondary().Degrees(), 45, 0.00001);
    EXPECT_EQ(v.lengthSquared(), 1.0);
}

TEST(VectorTest, length)
{
    Vector v(1, 4, 8);
    EXPECT_DOUBLE_EQ(v.lengthSquared(), 1);
    EXPECT_DOUBLE_EQ(v.length(), 1);
    v.normalise();
    EXPECT_DOUBLE_EQ(v.length(), 1);
    EXPECT_DOUBLE_EQ(v.l(), 1. / 9.);
    EXPECT_DOUBLE_EQ(v.m(), 4. / 9.);
    EXPECT_DOUBLE_EQ(v.n(), 8. / 9.);
}

TEST(VectorTest, rotateX)
{
    Vector v = Vector(Angle(90), Angle(45));
    Vector vr = v.rotateX(Angle(45));
    EXPECT_NEAR(vr.primary().Degrees(), 90, 0.00001);
    EXPECT_NEAR(vr.secondary().Degrees(), 0, 0.00001);
    v = vr.rotateX(Angle(-45));
    EXPECT_NEAR(v.primary().Degrees(), 90, 0.00001);
    EXPECT_NEAR(v.secondary().Degrees(), 45, 0.00001);
}

TEST(VectorTest, rotateY)
{
    Vector v = Vector(Angle(90), Angle(45));
    Vector vr = v.rotateY(Angle(45));
    EXPECT_NEAR(vr.primary().Degrees(), 125.26439, 0.00001);
    EXPECT_NEAR(vr.secondary().Degrees(), 30, 0.00001);
    v = vr.rotateY(Angle(-45));
    EXPECT_NEAR(v.primary().Degrees(), 90, 0.00001);
    EXPECT_NEAR(v.secondary().Degrees(), 45, 0.00001);
}

TEST(VectorTest, rotateZ)
{
    Vector v = Vector(Angle(90), Angle(45));
    Vector vr = v.rotateZ(Angle(45));
    EXPECT_NEAR(vr.primary().Degrees(), 45, 0.00001);
    EXPECT_NEAR(vr.secondary().Degrees(), 45, 0.00001);
    v = vr.rotateZ(Angle(-45));
    EXPECT_NEAR(v.primary().Degrees(), 90, 0.00001);
    EXPECT_NEAR(v.secondary().Degrees(), 45, 0.00001);
}

// ParabolicWindow tests
TEST(ParabolicWindowTest, SimpleParabola)
{
    // Test with x(t) = t^2, y(t) = 2*t + 5
    // dt in seconds. JD step = 10s = 10/86400 days.
    double step = 10.0 / 86400.0;
    double JD0 = 0.0;
    auto fn = [JD0](double JD) -> std::pair<double, double>
    {
        double t = (JD - JD0) * 86400.0;
        return { t * t, 2 * t + 5 };
    };

    ParabolicWindow win(fn, step);
    win.prime(JD0);

    // Midpoint: t=0 relative to JD0.  Value should be exact.
    auto val = win.valueAt(JD0);
    EXPECT_NEAR(val.first,  0.0, 1e-7);
    EXPECT_NEAR(val.second, 5.0, 1e-7);

    // Quarter step: t = 2.5s -> x = 6.25, y = 10
    auto val2 = win.valueAt(JD0 + 2.5 / 86400.0);
    EXPECT_NEAR(val2.first,  6.25, 1e-7);
    EXPECT_NEAR(val2.second, 10.0, 1e-7);

    // Rate: dx/dt = 2*t, dy/dt = 2
    // At t=5.0s, dx/dt = 10.0, dy/dt = 2.0
    auto rate = win.rateAt(JD0 + 5.0 / 86400.0);
    EXPECT_NEAR(rate.first,  10.0, 1e-7);
    EXPECT_NEAR(rate.second, 2.0, 1e-7);
}

TEST(ParabolicWindowTest, AngularWrapping)
{
    // Test linear drift crossing 180 degrees.
    // x(t) = 175 + 2*t (deg)
    double step = 10.0 / 86400.0;
    double JD0 = 0.0;
    auto fn = [JD0](double JD) -> std::pair<double, double>
    {
        double t = (JD - JD0) * 86400.0;
        return { Angle(175.0 + 2 * t).Degrees(), 0 };
    };

    ParabolicWindow win(fn, step);
    win.prime(JD0);

    // At t=0, x=175.
    EXPECT_NEAR(win.valueAt(JD0).first, 175.0, 1e-7);

    // At t=5s, x=185 -> -175.
    EXPECT_NEAR(win.valueAt(JD0 + 5.0 / 86400.0).first, -175.0, 1e-7);

    // Rate should be constant 2.0 deg/s even across the wrap.
    EXPECT_NEAR(win.rateAt(JD0 + 2.5 / 86400.0).first, 2.0, 1e-7);
    EXPECT_NEAR(win.rateAt(JD0 + 7.5 / 86400.0).first, 2.0, 1e-7);
}

TEST(ParabolicWindowTest, LunarTracking)
{
    // Source of truth: libnova lunar ephemeris.
    // The driver uses a 30s step for ephemeris interpolation.
    double step = 30.0 / 86400.0;
    auto lunarFn = [](double JD) -> std::pair<double, double>
    {
        ln_equ_posn epochPos;
        ln_get_lunar_equ_coords(JD, &epochPos);

        // Mirror driver logic: convert JNow to J2000 hours/deg
        INDI::IEquatorialCoordinates epochEq = { epochPos.ra / 15.0, epochPos.dec };
        INDI::IEquatorialCoordinates J2000Eq;
        INDI::ObservedToJ2000(&epochEq, JD, &J2000Eq);
        return { J2000Eq.rightascension, J2000Eq.declination };
    };

    ParabolicWindow win(lunarFn, step);
    double JD_start = 2461148.5; // Example JD
    win.prime(JD_start);

    // Track for 10 minutes (600 seconds) at 1-second intervals.
    // The ParabolicWindow should advance its internal 30s samples automatically.
    for (int s = 0; s <= 600; ++s)
    {
        double JD = JD_start + s / 86400.0;
        auto interpolated = win.valueAt(JD);
        auto truth        = lunarFn(JD);

        double raDiff  = Angle(interpolated.first - truth.first, Angle::HOURS).Degrees() * 3600.0;
        double decDiff = (interpolated.second - truth.second) * 3600.0;
        double residual = std::sqrt(raDiff * raDiff + decDiff * decDiff);

        // A 30s parabolic fit should be extremely accurate (well under 0.1 arcsec).
        EXPECT_LT(residual, 0.1) << "Lunar tracking residual exceeded 0.1\" at second " << s
                                 << " (residual=" << residual << "\")";
    }
}

TEST(ParabolicWindowTest, FeatureTracking)
{
    // Test that we can track a specific "feature" (crater) at an offset from the Moon.
    // The driver applies the Moon's *center delta* to the current target RA/Dec.
    double step = 30.0 / 86400.0;
    auto lunarFn = [](double JD) -> std::pair<double, double>
    {
        ln_equ_posn epochPos;
        ln_get_lunar_equ_coords(JD, &epochPos);
        return { epochPos.ra / 15.0, epochPos.dec }; // JNow hours/deg
    };

    ParabolicWindow win(lunarFn, step);
    double JD_start = 2461148.5;
    win.prime(JD_start);

    // Feature is 10 arcmin North and 10 arcmin West of the initial Moon center.
    auto moonStart = lunarFn(JD_start);
    double initialRA  = moonStart.first  - (10.0 / 60.0 / 15.0 / std::cos(DEG_TO_RAD(moonStart.second)));
    double initialDec = moonStart.second + (10.0 / 60.0);

    double trackedRA  = initialRA;
    double trackedDec = initialDec;
    double lastEphemRA = moonStart.first;
    double lastEphemDec = moonStart.second;

    // Track for 10 minutes, applying the driver's delta update logic each second.
    for (int s = 1; s <= 600; ++s)
    {
        double JD = JD_start + s / 86400.0;

        // Current ephemeris center
        auto center = win.valueAt(JD);

        // Apply positional delta of the center to our tracked feature
        double deltaRA  = Angle(center.first - lastEphemRA, Angle::HOURS).HoursHa();
        double deltaDec = center.second - lastEphemDec;

        trackedRA  = range24(trackedRA  + deltaRA);
        trackedDec = rangeDec(trackedDec + deltaDec);

        // Update state for next tick
        lastEphemRA = center.first;
        lastEphemDec = center.second;

        // Verify tracked feature still has the same offset relative to the new Moon center
        auto moonNow = lunarFn(JD);

        double raOffsetNow  = Angle(trackedRA - moonNow.first, Angle::HOURS).Degrees() * 60.0 * std::cos(DEG_TO_RAD(moonNow.second));
        double decOffsetNow = (trackedDec - moonNow.second) * 60.0;

        // The delta-based approach is mathematically robust, but because RA lines converge
        // towards the poles, a constant RA offset physically shrinks as Declination changes.
        // Over 10 minutes of lunar tracking, this geometric effect causes a ~0.002 arcmin
        // (0.12 arcsec) drift in the physical RA offset. We use a 0.01 arcmin (0.6 arcsec) tolerance.
        EXPECT_NEAR(raOffsetNow,  -10.0, 0.01) << "RA Offset drifted at second " << s;
        EXPECT_NEAR(decOffsetNow,  10.0, 0.01) << "Dec Offset drifted at second " << s;
    }
}

// Alignment tests
// the tuple contains:
// Test Ha, Ra, primary, azimuth angle
// Test Dec, altitude, secondary angle
// Expected Ha, Ra, primary, azimuth angle
// Expected Dec, altitude, secondary angle

class AlignmentTest : public ::testing::Test
{
    protected:
        Alignment alignment;
        AlignmentTest()
        {
            alignment.latitude = Angle(51.6);
            alignment.longitude = Angle(-0.73);
        }
};

TEST_F(AlignmentTest, Create)
{
    EXPECT_NEAR(alignment.latitude.Degrees(), 51.6, 0.00001);
    EXPECT_NEAR(alignment.longitude.Degrees(), -0.73, 0.00001);

    EXPECT_EQ(alignment.mountType, Alignment::MOUNT_TYPE::EQ_FORK);
}

TEST_F(AlignmentTest, Errors)
{
    EXPECT_EQ(alignment.ih(), 0);
    EXPECT_EQ(alignment.id(), 0);
    EXPECT_EQ(alignment.np(), 0);
    EXPECT_EQ(alignment.ch(), 0);
    EXPECT_EQ(alignment.ma(), 0);
    EXPECT_EQ(alignment.me(), 0);
    alignment.setCorrections(1, 2, 3, 4, 5, 6);
    EXPECT_EQ(alignment.ih(), 1);
    EXPECT_EQ(alignment.id(), 2);
    EXPECT_EQ(alignment.np(), 4);
    EXPECT_EQ(alignment.ch(), 3);
    EXPECT_EQ(alignment.ma(), 5);
    EXPECT_EQ(alignment.me(), 6);
}

TEST_F(AlignmentTest, instrumentToObservedME1)
{
    Angle oHa, oDec;

    alignment.setCorrections(0, 0, 0, 0, 0, 1); // ME 1

    // looking NS
    alignment.instrumentToObserved(Angle(0), Angle(0), &oHa, &oDec);
    EXPECT_EQ(oHa.Degrees(), 0);
    EXPECT_EQ(oDec.Degrees(), 1);

    // looking EW
    alignment.instrumentToObserved(Angle(90), Angle(0), &oHa, &oDec);
    EXPECT_EQ(oHa.Degrees(), 90);
    EXPECT_NEAR(oDec.Degrees(), 0, 1e10);

    // on meridian, dec 80
    alignment.instrumentToObserved(Angle(0), Angle(80), &oHa, &oDec);
    EXPECT_EQ(oHa.Degrees(), 0);
    EXPECT_FLOAT_EQ(oDec.Degrees(), 81);

    // looking at pole
    alignment.instrumentToObserved(Angle(0), Angle(90), &oHa, &oDec);
    EXPECT_FLOAT_EQ(oHa.HoursHa(), 12);
    EXPECT_FLOAT_EQ(oDec.Degrees(), 89);

    // 89 dec, expect move to pole
    alignment.instrumentToObserved(Angle(0), Angle(89), &oHa, &oDec);
    //EXPECT_FLOAT_EQ(oHa.HoursHa(), 12); // 0 or 12, similar case as observedToInstrumentME1
    EXPECT_FLOAT_EQ(oDec.Degrees(), 90);

    // 1 deg W of pole
    alignment.instrumentToObserved(Angle(90), Angle(89), &oHa, &oDec);
    EXPECT_NEAR(oHa.HoursHa(), 8.9997, 0.0001);
    EXPECT_FLOAT_EQ(oDec.Degrees(), 88.5858);
}

TEST_F(AlignmentTest, observedToInstrumentME1)
{
    Angle oHa, oDec;

    alignment.setCorrections(0, 0, 0, 0, 0, 1); // ME 1

    // looking NS
    alignment.observedToInstrument(Angle(0), Angle(0), &oHa, &oDec);
    EXPECT_EQ(oHa.Degrees(), 0);
    EXPECT_EQ(oDec.Degrees(), -1);

    // looking EW
    alignment.instrumentToObserved(Angle(-90), Angle(0), &oHa, &oDec);
    EXPECT_EQ(oHa.Degrees(), -90);
    EXPECT_NEAR(oDec.Degrees(), 0, 1e10);

    // on meridian, dec 80
    alignment.observedToInstrument(Angle(0), Angle(80), &oHa, &oDec);
    EXPECT_EQ(oHa.Degrees(), 0);
    EXPECT_FLOAT_EQ(oDec.Degrees(), 79);

    // looking at pole
    alignment.observedToInstrument(Angle(90), Angle(90), &oHa, &oDec);
    EXPECT_NEAR(oHa.HoursHa(), 0, 0.0001);
    EXPECT_FLOAT_EQ(oDec.Degrees(), 89);

    // 89 dec, expect move to pole
    alignment.observedToInstrument(Angle(180), Angle(89), &oHa, &oDec);
    //EXPECT_FLOAT_EQ(oHa.HoursHa(), 0);
    EXPECT_FLOAT_EQ(oDec.Degrees(), 90);

    // 1 deg E of pole
    alignment.observedToInstrument(Angle(-90), Angle(89), &oHa, &oDec);
    EXPECT_NEAR(oHa.HoursHa(), -3.0003, 0.00001);
    EXPECT_FLOAT_EQ(oDec.Degrees(), 88.5858);
}


TEST_F(AlignmentTest, observedToInstrumentMEn1)
{
    Angle oHa, oDec;

    alignment.setCorrections(0, 0, 0, 0, 0, -1); // ME -1

    // looking NS
    alignment.observedToInstrument(Angle(0), Angle(0), &oHa, &oDec);
    EXPECT_EQ(oHa.Degrees(), 0);
    EXPECT_EQ(oDec.Degrees(), 1);

    // looking EW
    alignment.instrumentToObserved(Angle(-90), Angle(0), &oHa, &oDec);
    EXPECT_EQ(oHa.Degrees(), -90);
    EXPECT_NEAR(oDec.Degrees(), 0, 1e10);

    // on meridian, dec 80
    alignment.observedToInstrument(Angle(0), Angle(80), &oHa, &oDec);
    EXPECT_EQ(oHa.Degrees(), 0);
    EXPECT_FLOAT_EQ(oDec.Degrees(), 81);

    // looking at pole
    alignment.observedToInstrument(Angle(90), Angle(90), &oHa, &oDec);
    EXPECT_NEAR(oHa.HoursHa(), 12, 0.0001);
    EXPECT_FLOAT_EQ(oDec.Degrees(), 89);

    // 89 dec, expect move to pole
    alignment.observedToInstrument(Angle(0), Angle(89), &oHa, &oDec);
    //EXPECT_FLOAT_EQ(oHa.HoursHa(), 0);
    EXPECT_FLOAT_EQ(oDec.Degrees(), 90);

    // 1 deg E of pole
    alignment.observedToInstrument(Angle(-90), Angle(89), &oHa, &oDec);
    EXPECT_NEAR(oHa.HoursHa(), -8.9997, 0.00001);
    EXPECT_FLOAT_EQ(oDec.Degrees(), 88.5858);
}

TEST_F(AlignmentTest, instrumentToObservedMA1)
{
    Angle oHa, oDec;

    alignment.setCorrections(0, 0, 0, 0, 1, 0); // MA 1

    // looking NS
    alignment.instrumentToObserved(Angle(0), Angle(0), &oHa, &oDec);
    EXPECT_EQ(oHa.Degrees(), 0);
    EXPECT_EQ(oDec.Degrees(), 0);

    // looking WE
    alignment.instrumentToObserved(Angle(-90), Angle(0), &oHa, &oDec);
    EXPECT_EQ(oHa.Degrees(), -90);
    EXPECT_EQ(oDec.Degrees(), 1);

    // W, dec 80
    alignment.instrumentToObserved(Angle(90), Angle(80), &oHa, &oDec);
    EXPECT_EQ(oHa.Degrees(), 90);
    EXPECT_FLOAT_EQ(oDec.Degrees(), 79);

    // looking at pole
    alignment.instrumentToObserved(Angle(0), Angle(90), &oHa, &oDec);
    EXPECT_FLOAT_EQ(oHa.HoursHa(), 6);
    EXPECT_FLOAT_EQ(oDec.Degrees(), 89);

    // 89 dec, expect move to pole
    alignment.instrumentToObserved(Angle(-90), Angle(89), &oHa, &oDec);
    //EXPECT_FLOAT_EQ(oHa.HoursHa(), -6);
    EXPECT_FLOAT_EQ(oDec.Degrees(), 90);

    // 1 deg N of pole
    alignment.instrumentToObserved(Angle(180), Angle(89), &oHa, &oDec);
    EXPECT_NEAR(oHa.HoursHa(), 9.0003, 0.0001);
    EXPECT_FLOAT_EQ(oDec.Degrees(), 88.5858);
}


TEST_F(AlignmentTest, instrumentToObservedMAm1)
{
    Angle oHa, oDec;

    alignment.setCorrections(0, 0, 0, 0, -1, 0); // MA -1

    // looking NS
    alignment.instrumentToObserved(Angle(0), Angle(0), &oHa, &oDec);
    EXPECT_EQ(oHa.Degrees(), 0);
    EXPECT_EQ(oDec.Degrees(), 0);

    // looking WE
    alignment.instrumentToObserved(Angle(-90), Angle(0), &oHa, &oDec);
    EXPECT_EQ(oHa.Degrees(), -90);
    EXPECT_EQ(oDec.Degrees(), -1);

    // W, dec 80
    alignment.instrumentToObserved(Angle(90), Angle(80), &oHa, &oDec);
    EXPECT_EQ(oHa.Degrees(), 90);
    EXPECT_FLOAT_EQ(oDec.Degrees(), 81);

    // looking at pole
    alignment.instrumentToObserved(Angle(0), Angle(90), &oHa, &oDec);
    EXPECT_FLOAT_EQ(oHa.HoursHa(), -6);
    EXPECT_FLOAT_EQ(oDec.Degrees(), 89);

    // 89 dec, expect move to pole
    alignment.instrumentToObserved(Angle(90), Angle(89), &oHa, &oDec);
    //EXPECT_FLOAT_EQ(oHa.HoursHa(), -6);
    EXPECT_FLOAT_EQ(oDec.Degrees(), 90);

    // 1 deg S of pole
    alignment.instrumentToObserved(Angle(0), Angle(89), &oHa, &oDec);
    EXPECT_NEAR(oHa.HoursHa(), -2.9997, 0.0001);
    EXPECT_FLOAT_EQ(oDec.Degrees(), 88.5858);
}

TEST_F(AlignmentTest, observedToInstrumentMA1)
{
    Angle iHa, iDec;

    alignment.setCorrections(0, 0, 0, 0, 1, 0); // MA 1

    // looking NS
    alignment.observedToInstrument(Angle(0), Angle(0), &iHa, &iDec);
    EXPECT_EQ(iHa.Degrees(), 0);
    EXPECT_EQ(iDec.Degrees(), 0);

    // looking EW
    alignment.observedToInstrument(Angle(90), Angle(0), &iHa, &iDec);
    EXPECT_EQ(iHa.Degrees(), 90);
    EXPECT_EQ(iDec.Degrees(), 1);

    // E, dec 80
    alignment.observedToInstrument(Angle(-90), Angle(80), &iHa, &iDec);
    EXPECT_EQ(iHa.Degrees(), -90);
    EXPECT_FLOAT_EQ(iDec.Degrees(), 79);

    // looking at pole
    alignment.observedToInstrument(Angle(90), Angle(90), &iHa, &iDec);
    EXPECT_NEAR(iHa.HoursHa(), -6, 0.0001);
    EXPECT_FLOAT_EQ(iDec.Degrees(), 89);

    // 1 deg S of pole
    alignment.observedToInstrument(Angle(0), Angle(89), &iHa, &iDec);
    EXPECT_NEAR(iHa.HoursHa(), -2.9997, 0.00001);
    EXPECT_FLOAT_EQ(iDec.Degrees(), 88.5858);
}


TEST_F(AlignmentTest, instrumentToObservedCH1)
{
    Angle oHa, oDec;

    alignment.setCorrections(0, 0, 1, 0, 0, 0);      // CH 1

    // looking NS
    alignment.instrumentToObserved(Angle(0), Angle(0), &oHa, &oDec);
    EXPECT_EQ(oHa.Degrees(), 1);
    EXPECT_EQ(oDec.Degrees(), 0);

    // looking WE
    alignment.instrumentToObserved(Angle(-90), Angle(0), &oHa, &oDec);
    EXPECT_EQ(oHa.Degrees(), -89);
    EXPECT_EQ(oDec.Degrees(), 0);

    // W, dec 60
    alignment.instrumentToObserved(Angle(90), Angle(60), &oHa, &oDec);
    EXPECT_EQ(oHa.Degrees(), 92);
    EXPECT_FLOAT_EQ(oDec.Degrees(), 60);

    // looking at pole
    alignment.instrumentToObserved(Angle(0), Angle(90), &oHa, &oDec);
    //EXPECT_FLOAT_EQ(oHa.HoursHa(), 6);
    EXPECT_FLOAT_EQ(oDec.Degrees(), 90);

    // 89 dec
    alignment.instrumentToObserved(Angle(-90), Angle(89), &oHa, &oDec);
    EXPECT_NEAR(oHa.HoursHa(), -2.180087, 0.0001);
    EXPECT_FLOAT_EQ(oDec.Degrees(), 89);

    // 1 deg N of pole
    alignment.instrumentToObserved(Angle(180), Angle(89), &oHa, &oDec);
    EXPECT_NEAR(oHa.HoursHa(), -8.180087, 0.0001);
    EXPECT_FLOAT_EQ(oDec.Degrees(), 89);
}

TEST_F(AlignmentTest, observedToInstrumentCH1)
{
    Angle iHa, iDec;

    alignment.setCorrections(0, 0, 1, 0, 0, 0);      // CH 1

    // looking NS
    alignment.observedToInstrument(Angle(0), Angle(0), &iHa, &iDec);
    EXPECT_EQ(iHa.Degrees(), -1);
    EXPECT_EQ(iDec.Degrees(), 0);

    // looking EW
    alignment.observedToInstrument(Angle(90), Angle(0), &iHa, &iDec);
    EXPECT_EQ(iHa.Degrees(), 89);
    EXPECT_EQ(iDec.Degrees(), 0);

    // E, dec 60
    alignment.observedToInstrument(Angle(-90), Angle(60), &iHa, &iDec);
    EXPECT_EQ(iHa.Degrees(), -92);
    EXPECT_FLOAT_EQ(iDec.Degrees(), 60);

    // looking at pole
    alignment.observedToInstrument(Angle(90), Angle(90), &iHa, &iDec);
    //EXPECT_NEAR(iHa.HoursHa(), -6, 0.0001);
    EXPECT_FLOAT_EQ(iDec.Degrees(), 90);

    // 1 deg S of pole
    alignment.observedToInstrument(Angle(0), Angle(89), &iHa, &iDec);
    EXPECT_NEAR(iHa.HoursHa(), -3.81991, 0.00001);
    EXPECT_FLOAT_EQ(iDec.Degrees(), 89);
}

TEST_F(AlignmentTest, roundTripMAME1)
{
    Angle oHa, oDec;
    Angle iHa, iDec;

    alignment.setCorrections(0, 0, 0, 0, 1, 1); // MA 1, ME 1

    // looking NS
    alignment.instrumentToObserved(Angle(0), Angle(0), &oHa, &oDec);
    alignment.observedToInstrument(oHa, oDec, &iHa, &iDec);
    EXPECT_NEAR(iHa.Degrees(), 0, 0.00001);
    EXPECT_NEAR(iDec.Degrees(), 0, 0.00001);

    // looking EW
    alignment.instrumentToObserved(Angle(90), Angle(0), &oHa, &oDec);
    alignment.observedToInstrument(oHa, oDec, &iHa, &iDec);
    EXPECT_NEAR(iHa.Degrees(), 90, 0.00001);
    EXPECT_NEAR(iDec.Degrees(), 0, 0.00001);

    // on meridian, dec 80
    alignment.instrumentToObserved(Angle(0), Angle(80), &oHa, &oDec);
    alignment.observedToInstrument(oHa, oDec, &iHa, &iDec);
    EXPECT_NEAR(iHa.Degrees(), 0, 0.00001);
    EXPECT_NEAR(iDec.Degrees(), 80, 0.00001);

    // E, dec 80
    alignment.instrumentToObserved(Angle(-90), Angle(80), &oHa, &oDec);
    alignment.observedToInstrument(oHa, oDec, &iHa, &iDec);
    EXPECT_NEAR(iHa.Degrees(), -90, 0.00001);
    EXPECT_NEAR(iDec.Degrees(), 80, 0.00001);

    // looking at pole
    alignment.instrumentToObserved(Angle(0), Angle(90), &oHa, &oDec);
    alignment.observedToInstrument(oHa, oDec, &iHa, &iDec);
    //EXPECT_NEAR(iHa.Degrees(), 0, 0.0001);
    EXPECT_NEAR(iDec.Degrees(), 90, 0.00001);

    // 89 dec, expect move to pole
    alignment.instrumentToObserved(Angle(0), Angle(89), &oHa, &oDec);
    alignment.observedToInstrument(oHa, oDec, &iHa, &iDec);
    EXPECT_NEAR(iHa.Degrees(), 0, 0.00001);
    EXPECT_NEAR(iDec.Degrees(), 89, 0.00001);

    // 1 deg W of pole
    alignment.instrumentToObserved(Angle(90), Angle(89), &oHa, &oDec);
    alignment.observedToInstrument(oHa, oDec, &iHa, &iDec);
    EXPECT_NEAR(iHa.Degrees(), 90, 0.00001);
    EXPECT_NEAR(iDec.Degrees(), 89, 0.00001);

    // 1 deg S of pole
    alignment.instrumentToObserved(Angle(0), Angle(89), &oHa, &oDec);
    alignment.observedToInstrument(oHa, oDec, &iHa, &iDec);
    EXPECT_NEAR(iHa.Degrees(), 0, 0.00001);
    EXPECT_NEAR(iDec.Degrees(), 89, 0.00001);
}

/*
class AlignmentParametersTest :public ::testing::TestWithParam<std::tuple<double, double, double, double>>
{
protected:
    Alignment alignment;
    AlignmentParametersTest()
    {
        alignment.latitude = Angle(51.6);
        alignment.longitude = Angle(-0.73);
    }
};

INSTANTIATE_TEST_SUITE_P
(
    ZeroErrors,
    AlignmentParametersTest,
    ::testing::Values
    (
            // test Ha, Dec, expected Ha, Dec - all in degrees
            std::make_tuple(0, 0, 0, 0),
            std::make_tuple(0, 30, 0, 30),
            std::make_tuple(-90, 0, -90, 0),
            std::make_tuple(90, 30, 90, 30),
            std::make_tuple(-179, 60, -179, 60),
            std::make_tuple(-180, 90, 180, 90)
    )
);

TEST_P(AlignmentParametersTest, observed2Instrument)
{
    Angle iHa, iDec;

    double testHa = std::get<0>(GetParam());
    double testDec = std::get<1>(GetParam());
    double expectedHa = std::get<2>(GetParam());
    double expectedDec = std::get<3>(GetParam());

    alignment.observedToInstrument(Angle(testHa), Angle(testDec), &iHa, &iDec);
    EXPECT_EQ(iHa.Degrees(), expectedHa);
    EXPECT_EQ(iDec.Degrees(), expectedDec);
}

TEST_P(AlignmentParametersTest, InstrumentToObserved)
{
    Angle iHa, iDec;

    double testHa = std::get<0>(GetParam());
    double testDec = std::get<1>(GetParam());
    double expectedHa = std::get<2>(GetParam());
    double expectedDec = std::get<3>(GetParam());

    alignment.instrumentToObserved(Angle(testHa), Angle(testDec), &iHa, &iDec);
    EXPECT_FLOAT_EQ(iHa.Degrees(), expectedHa);
    EXPECT_FLOAT_EQ(iDec.Degrees(), expectedDec);
}

INSTANTIATE_TEST_SUITE_P
(
    TestME1,
    AlignmentParametersTest,
    ::testing::Values
    (
            // test Ha, Dec, expected Ha, Dec - all in degrees
            std::make_tuple(0, 0, 0, 0),
            std::make_tuple(0, 30, 0, 30),
            std::make_tuple(-90, 0, -90, 0),
            std::make_tuple(90, 30, 90, 30),
            std::make_tuple(-179, 60, -179, 60),
            std::make_tuple(-180, 90, 180, 90)
    )
);
*/

int main(int argc, char **argv)
{
    INDI::Logger::getInstance().configure("", INDI::Logger::file_off,
                                          INDI::Logger::DBG_ERROR, INDI::Logger::DBG_ERROR);

    ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}


