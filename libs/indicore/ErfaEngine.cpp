#include "CoordinateEngine.h"
#include "indidevapi.h"
#include "indicom.h"
#ifdef HAVE_ERFA
#include <erfa.h>
#include "eph/eph.h"
#endif
#include <cmath>
#include <memory>

#ifdef HAVE_ERFA

static void eraApci00b(double date1, double date2, eraASTROM *astrom, double *eo)
{
    double ehpv[2][3], ebpv[2][3], r[3][3], x, y, s;
    eraEpv00(date1, date2, ehpv, ebpv);
    eraPnm00b(date1, date2, r);
    eraBpn2xy(r, &x, &y);
    s = eraS00(date1, date2, x, y);
    eraApci(date1, date2, ebpv, ehpv[0], x, y, s, astrom);
    *eo = eraEors(r, s);
}

static void eraAtci00b(double rc, double dc, double pr, double pd, double px, double rv,
                       double date1, double date2, double *ri, double *di, double *eo)
{
    eraASTROM astrom;
    eraApci00b(date1, date2, &astrom, eo);
    eraAtciq(rc, dc, pr, pd, px, rv, &astrom, ri, di);
}

static void eraApco00b(double utc1, double utc2, double dut1,
                       double elong, double phi, double hm,
                       double xp, double yp, double phpa, double tc, double rh, double wl,
                       eraASTROM *astrom, double *eo)
{
    // Mirrors eraApco13 but substitutes 2000B nutation (eraPnm00b) for 2000A.
    // eraApco (the bare combinator) computes the observer's geocentric position/velocity
    // via eraPvtob and passes it to eraApcs, so astrom->v includes the diurnal component.
    // This is required for eraAtciq to apply diurnal aberration correctly.
    double tai1, tai2, tt1, tt2;
    eraUtctai(utc1, utc2, &tai1, &tai2);
    eraTaitt(tai1, tai2, &tt1, &tt2);

    double ut11, ut12;
    eraUtcut1(utc1, utc2, dut1, &ut11, &ut12);
    double theta = eraEra00(ut11, ut12);
    double sp    = eraSp00(tt1, tt2);

    double ehpv[2][3], ebpv[2][3], r[3][3], x, y, s;
    eraEpv00(tt1, tt2, ehpv, ebpv);
    eraPnm00b(tt1, tt2, r);
    eraBpn2xy(r, &x, &y);
    s = eraS00(tt1, tt2, x, y);

    double refa, refb;
    eraRefco(phpa, tc, rh, wl, &refa, &refb);

    eraApco(tt1, tt2, ebpv, ehpv[0], x, y, s, theta,
            elong, phi, hm, xp, yp, sp, refa, refb, astrom);
    *eo = eraEors(r, s);
}

static bool observerChanged(const INDI::AstrometricContext &ctx)
{
    return ctx.observer.longitude != ctx.cached_observer.longitude ||
           ctx.observer.latitude  != ctx.cached_observer.latitude  ||
           ctx.observer.elevation != ctx.cached_observer.elevation;
}

static void ensureAstrom2000B(INDI::AstrometricContext &ctx, double jd)
{
    if (ctx.cache_valid &&
        std::abs(jd - ctx.cached_jd) <= ctx.jd_tolerance &&
        !observerChanged(ctx))
        return;
    double utc1 = std::floor(jd) + 0.5, utc2 = jd - utc1;
    eraApco00b(utc1, utc2, ctx.dut1,
               DEG_TO_RAD(ctx.observer.longitude), DEG_TO_RAD(ctx.observer.latitude),
               ctx.observer.elevation,
               DEG_TO_RAD(ctx.xp / 3600.0), DEG_TO_RAD(ctx.yp / 3600.0),
               ctx.pressure_hPa, ctx.temp_C, ctx.humidity, ctx.wavelength_um,
               &ctx.astrom, &ctx.eo);
    ctx.cached_jd       = jd;
    ctx.cached_observer = ctx.observer;
    ctx.cache_valid     = true;
}

static void ensureAstrom2000A(INDI::AstrometricContext &ctx, double jd)
{
    if (ctx.cache_valid &&
        std::abs(jd - ctx.cached_jd) <= ctx.jd_tolerance &&
        !observerChanged(ctx))
        return;
    double utc1 = std::floor(jd) + 0.5, utc2 = jd - utc1;
    eraApco13(utc1, utc2, ctx.dut1,
              DEG_TO_RAD(ctx.observer.longitude), DEG_TO_RAD(ctx.observer.latitude),
              ctx.observer.elevation,
              DEG_TO_RAD(ctx.xp / 3600.0), DEG_TO_RAD(ctx.yp / 3600.0),
              ctx.pressure_hPa, ctx.temp_C, ctx.humidity, ctx.wavelength_um,
              &ctx.astrom, &ctx.eo);
    ctx.cached_jd       = jd;
    ctx.cached_observer = ctx.observer;
    ctx.cache_valid     = true;
}

#endif

class ErfaEngine2000A : public ICoordinateEngine {
public:
    void J2000toObserved(INDI::IEquatorialCoordinates *j2000, double jd, INDI::IEquatorialCoordinates *jnow) override {
#ifdef HAVE_ERFA
        double utc1 = std::floor(jd) + 0.5, utc2 = jd - utc1;
        double tai1, tai2, tt1, tt2;
        eraUtctai(utc1, utc2, &tai1, &tai2);
        eraTaitt(tai1, tai2, &tt1, &tt2);
        double ri = HOURS_TO_RAD(j2000->rightascension);
        double di = DEG_TO_RAD(j2000->declination);
        double rc, dc, eo;
        eraAtci13(ri, di, 0, 0, 0, 0, tt1, tt2, &rc, &dc, &eo);
        jnow->rightascension = RAD_TO_HOURS(eraAnp(rc - eo));
        jnow->declination = RAD_TO_DEG(dc);
#else
        INDI_UNUSED(j2000); INDI_UNUSED(jd); INDI_UNUSED(jnow);
#endif
    }

    void ObservedToJ2000(INDI::IEquatorialCoordinates *jnow, double jd, INDI::IEquatorialCoordinates *j2000) override {
#ifdef HAVE_ERFA
        double utc1 = std::floor(jd) + 0.5, utc2 = jd - utc1;
        double tai1, tai2, tt1, tt2;
        eraUtctai(utc1, utc2, &tai1, &tai2);
        eraTaitt(tai1, tai2, &tt1, &tt2);
        double ra_jnow_rad = HOURS_TO_RAD(jnow->rightascension);
        double dec_jnow_rad = DEG_TO_RAD(jnow->declination);
        eraASTROM astrom;
        double eo;
        eraApci13(tt1, tt2, &astrom, &eo);
        double ri, di;
        eraAticq(eraAnp(ra_jnow_rad + eo), dec_jnow_rad, &astrom, &ri, &di);
        j2000->rightascension = RAD_TO_HOURS(eraAnp(ri));
        j2000->declination = RAD_TO_DEG(di);
#else
        INDI_UNUSED(jnow); INDI_UNUSED(jd); INDI_UNUSED(j2000);
#endif
    }

    void EquatorialToHorizontal(INDI::IEquatorialCoordinates *object, INDI::IGeographicCoordinates *observer, double JD, INDI::IHorizontalCoordinates *position) override {
#ifdef HAVE_ERFA
        double utc1 = std::floor(JD) + 0.5, utc2 = JD - utc1;
        eraASTROM astrom;
        double eo;
        eraApco13(utc1, utc2, 0.0,
                  DEG_TO_RAD(observer->longitude), DEG_TO_RAD(observer->latitude), observer->elevation,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.55, &astrom, &eo);
        double ra_cirs = eraAnp(HOURS_TO_RAD(object->rightascension) + eo);
        double aob, zob, hob, dob, rob;
        eraAtioq(ra_cirs, DEG_TO_RAD(object->declination), &astrom, &aob, &zob, &hob, &dob, &rob);
        position->azimuth  = RAD_TO_DEG(aob);
        position->altitude = 90.0 - RAD_TO_DEG(zob);
#else
        INDI_UNUSED(object); INDI_UNUSED(observer); INDI_UNUSED(JD); INDI_UNUSED(position);
#endif
    }

    void HorizontalToEquatorial(INDI::IHorizontalCoordinates *object, INDI::IGeographicCoordinates *observer, double JD, INDI::IEquatorialCoordinates *position) override {
#ifdef HAVE_ERFA
        double utc1 = std::floor(JD) + 0.5, utc2 = JD - utc1;
        eraASTROM astrom;
        double eo;
        eraApco13(utc1, utc2, 0.0,
                  DEG_TO_RAD(observer->longitude), DEG_TO_RAD(observer->latitude), observer->elevation,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.55, &astrom, &eo);
        double ri, di;
        eraAtoiq("A", DEG_TO_RAD(object->azimuth), DEG_TO_RAD(90.0 - object->altitude), &astrom, &ri, &di);
        position->rightascension = RAD_TO_HOURS(eraAnp(ri - eo));
        position->declination    = RAD_TO_DEG(di);
#else
        INDI_UNUSED(object); INDI_UNUSED(observer); INDI_UNUSED(JD); INDI_UNUSED(position);
#endif
    }

    void J2000toGeocentric(const INDI::J2000Coordinates *j2000, double jd, INDI::GeocentricApparent *out) override {
#ifdef HAVE_ERFA
        double utc1 = std::floor(jd) + 0.5, utc2 = jd - utc1;
        double tai1, tai2, tt1, tt2;
        eraUtctai(utc1, utc2, &tai1, &tai2);
        eraTaitt(tai1, tai2, &tt1, &tt2);
        double rc, dc, eo;
        eraAtci13(HOURS_TO_RAD(j2000->rightascension), DEG_TO_RAD(j2000->declination),
                  0, 0, 0, 0, tt1, tt2, &rc, &dc, &eo);
        out->rightascension = RAD_TO_HOURS(eraAnp(rc - eo));
        out->declination    = RAD_TO_DEG(dc);
#else
        INDI_UNUSED(j2000); INDI_UNUSED(jd); INDI_UNUSED(out);
#endif
    }

    void GeocentricToJ2000(const INDI::GeocentricApparent *apparent, double jd, INDI::J2000Coordinates *out) override {
#ifdef HAVE_ERFA
        double utc1 = std::floor(jd) + 0.5, utc2 = jd - utc1;
        double tai1, tai2, tt1, tt2;
        eraUtctai(utc1, utc2, &tai1, &tai2);
        eraTaitt(tai1, tai2, &tt1, &tt2);
        eraASTROM astrom;
        double eo;
        eraApci13(tt1, tt2, &astrom, &eo);
        double ri, di;
        eraAticq(eraAnp(HOURS_TO_RAD(apparent->rightascension) + eo), DEG_TO_RAD(apparent->declination),
                 &astrom, &ri, &di);
        out->rightascension = RAD_TO_HOURS(eraAnp(ri));
        out->declination    = RAD_TO_DEG(di);
#else
        INDI_UNUSED(apparent); INDI_UNUSED(jd); INDI_UNUSED(out);
#endif
    }

    void J2000toTopocentric(const INDI::J2000Coordinates *j2000, INDI::AstrometricContext &ctx, double jd,
                            INDI::TopocentricApparent *out) override {
#ifdef HAVE_ERFA
        ensureAstrom2000A(ctx, jd);
        double ri, di;
        eraAtciq(HOURS_TO_RAD(j2000->rightascension), DEG_TO_RAD(j2000->declination),
                 0, 0, 0, 0, &ctx.astrom, &ri, &di);
        out->rightascension = RAD_TO_HOURS(eraAnp(ri - ctx.eo));
        out->declination    = RAD_TO_DEG(di);
#else
        INDI_UNUSED(j2000); INDI_UNUSED(ctx); INDI_UNUSED(jd); INDI_UNUSED(out);
#endif
    }

    void TopocentricToJ2000(const INDI::TopocentricApparent *apparent, INDI::AstrometricContext &ctx, double jd,
                            INDI::J2000Coordinates *out) override {
#ifdef HAVE_ERFA
        ensureAstrom2000A(ctx, jd);
        double ri, di;
        eraAticq(eraAnp(HOURS_TO_RAD(apparent->rightascension) + ctx.eo), DEG_TO_RAD(apparent->declination),
                 &ctx.astrom, &ri, &di);
        out->rightascension = RAD_TO_HOURS(eraAnp(ri));
        out->declination    = RAD_TO_DEG(di);
#else
        INDI_UNUSED(apparent); INDI_UNUSED(ctx); INDI_UNUSED(jd); INDI_UNUSED(out);
#endif
    }
};

class ErfaEngine2000B : public ICoordinateEngine {
public:
    void J2000toObserved(INDI::IEquatorialCoordinates *j2000, double jd, INDI::IEquatorialCoordinates *jnow) override {
#ifdef HAVE_ERFA
        double utc1 = std::floor(jd) + 0.5, utc2 = jd - utc1;
        double tai1, tai2, tt1, tt2;
        eraUtctai(utc1, utc2, &tai1, &tai2);
        eraTaitt(tai1, tai2, &tt1, &tt2);
        double ri = HOURS_TO_RAD(j2000->rightascension);
        double di = DEG_TO_RAD(j2000->declination);
        double rc, dc, eo;
        eraAtci00b(ri, di, 0, 0, 0, 0, tt1, tt2, &rc, &dc, &eo);
        jnow->rightascension = RAD_TO_HOURS(eraAnp(rc - eo));
        jnow->declination = RAD_TO_DEG(dc);
#else
        INDI_UNUSED(j2000); INDI_UNUSED(jd); INDI_UNUSED(jnow);
#endif
    }

    void ObservedToJ2000(INDI::IEquatorialCoordinates *jnow, double jd, INDI::IEquatorialCoordinates *j2000) override {
#ifdef HAVE_ERFA
        double utc1 = std::floor(jd) + 0.5, utc2 = jd - utc1;
        double tai1, tai2, tt1, tt2;
        eraUtctai(utc1, utc2, &tai1, &tai2);
        eraTaitt(tai1, tai2, &tt1, &tt2);
        double ra_jnow_rad = HOURS_TO_RAD(jnow->rightascension);
        double dec_jnow_rad = DEG_TO_RAD(jnow->declination);
        eraASTROM astrom;
        double eo;
        eraApci00b(tt1, tt2, &astrom, &eo);
        double ri, di;
        eraAticq(eraAnp(ra_jnow_rad + eo), dec_jnow_rad, &astrom, &ri, &di);
        j2000->rightascension = RAD_TO_HOURS(eraAnp(ri));
        j2000->declination = RAD_TO_DEG(di);
#else
        INDI_UNUSED(jnow); INDI_UNUSED(jd); INDI_UNUSED(j2000);
#endif
    }

    void EquatorialToHorizontal(INDI::IEquatorialCoordinates *object, INDI::IGeographicCoordinates *observer, double JD, INDI::IHorizontalCoordinates *position) override {
#ifdef HAVE_ERFA
        double utc1 = std::floor(JD) + 0.5, utc2 = JD - utc1;
        eraASTROM astrom;
        double eo;
        eraApco00b(utc1, utc2, 0.0,
                   DEG_TO_RAD(observer->longitude), DEG_TO_RAD(observer->latitude), observer->elevation,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.55, &astrom, &eo);
        double ra_cirs = eraAnp(HOURS_TO_RAD(object->rightascension) + eo);
        double aob, zob, hob, dob, rob;
        eraAtioq(ra_cirs, DEG_TO_RAD(object->declination), &astrom, &aob, &zob, &hob, &dob, &rob);
        position->azimuth  = RAD_TO_DEG(aob);
        position->altitude = 90.0 - RAD_TO_DEG(zob);
#else
        INDI_UNUSED(object); INDI_UNUSED(observer); INDI_UNUSED(JD); INDI_UNUSED(position);
#endif
    }

    void HorizontalToEquatorial(INDI::IHorizontalCoordinates *object, INDI::IGeographicCoordinates *observer, double JD, INDI::IEquatorialCoordinates *position) override {
#ifdef HAVE_ERFA
        double utc1 = std::floor(JD) + 0.5, utc2 = JD - utc1;
        eraASTROM astrom;
        double eo;
        eraApco00b(utc1, utc2, 0.0,
                   DEG_TO_RAD(observer->longitude), DEG_TO_RAD(observer->latitude), observer->elevation,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.55, &astrom, &eo);
        double ri, di;
        eraAtoiq("A", DEG_TO_RAD(object->azimuth), DEG_TO_RAD(90.0 - object->altitude), &astrom, &ri, &di);
        position->rightascension = RAD_TO_HOURS(eraAnp(ri - eo));
        position->declination    = RAD_TO_DEG(di);
#else
        INDI_UNUSED(object); INDI_UNUSED(observer); INDI_UNUSED(JD); INDI_UNUSED(position);
#endif
    }

    void J2000toGeocentric(const INDI::J2000Coordinates *j2000, double jd, INDI::GeocentricApparent *out) override {
#ifdef HAVE_ERFA
        double utc1 = std::floor(jd) + 0.5, utc2 = jd - utc1;
        double tai1, tai2, tt1, tt2;
        eraUtctai(utc1, utc2, &tai1, &tai2);
        eraTaitt(tai1, tai2, &tt1, &tt2);
        double rc, dc, eo;
        eraAtci00b(HOURS_TO_RAD(j2000->rightascension), DEG_TO_RAD(j2000->declination),
                   0, 0, 0, 0, tt1, tt2, &rc, &dc, &eo);
        out->rightascension = RAD_TO_HOURS(eraAnp(rc - eo));
        out->declination    = RAD_TO_DEG(dc);
#else
        INDI_UNUSED(j2000); INDI_UNUSED(jd); INDI_UNUSED(out);
#endif
    }

    void GeocentricToJ2000(const INDI::GeocentricApparent *apparent, double jd, INDI::J2000Coordinates *out) override {
#ifdef HAVE_ERFA
        double utc1 = std::floor(jd) + 0.5, utc2 = jd - utc1;
        double tai1, tai2, tt1, tt2;
        eraUtctai(utc1, utc2, &tai1, &tai2);
        eraTaitt(tai1, tai2, &tt1, &tt2);
        eraASTROM astrom;
        double eo;
        eraApci00b(tt1, tt2, &astrom, &eo);
        double ri, di;
        eraAticq(eraAnp(HOURS_TO_RAD(apparent->rightascension) + eo), DEG_TO_RAD(apparent->declination),
                 &astrom, &ri, &di);
        out->rightascension = RAD_TO_HOURS(eraAnp(ri));
        out->declination    = RAD_TO_DEG(di);
#else
        INDI_UNUSED(apparent); INDI_UNUSED(jd); INDI_UNUSED(out);
#endif
    }

    void J2000toTopocentric(const INDI::J2000Coordinates *j2000, INDI::AstrometricContext &ctx, double jd,
                            INDI::TopocentricApparent *out) override {
#ifdef HAVE_ERFA
        ensureAstrom2000B(ctx, jd);
        double ri, di;
        eraAtciq(HOURS_TO_RAD(j2000->rightascension), DEG_TO_RAD(j2000->declination),
                 0, 0, 0, 0, &ctx.astrom, &ri, &di);
        out->rightascension = RAD_TO_HOURS(eraAnp(ri - ctx.eo));
        out->declination    = RAD_TO_DEG(di);
#else
        INDI_UNUSED(j2000); INDI_UNUSED(ctx); INDI_UNUSED(jd); INDI_UNUSED(out);
#endif
    }

    void TopocentricToJ2000(const INDI::TopocentricApparent *apparent, INDI::AstrometricContext &ctx, double jd,
                            INDI::J2000Coordinates *out) override {
#ifdef HAVE_ERFA
        ensureAstrom2000B(ctx, jd);
        double ri, di;
        eraAticq(eraAnp(HOURS_TO_RAD(apparent->rightascension) + ctx.eo), DEG_TO_RAD(apparent->declination),
                 &ctx.astrom, &ri, &di);
        out->rightascension = RAD_TO_HOURS(eraAnp(ri));
        out->declination    = RAD_TO_DEG(di);
#else
        INDI_UNUSED(apparent); INDI_UNUSED(ctx); INDI_UNUSED(jd); INDI_UNUSED(out);
#endif
    }
};

std::unique_ptr<ICoordinateEngine> createErfaEngine2000A() {
    return std::make_unique<ErfaEngine2000A>();
}

std::unique_ptr<ICoordinateEngine> createErfaEngine2000B() {
    return std::make_unique<ErfaEngine2000B>();
}
