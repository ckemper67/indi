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
    // Geocentric part: 77-term nutation (2000B) via eraPnm00b + eraS00, same as eraApci00b.
    // Observer part: eraApio13 is nutation-model-independent — it only uses ERA, polar motion,
    // and the TIO locator s' (eraSp00), none of which depend on the 2000A/B choice.
    // eraApco13 is "2000A" only because it calls eraPnm06a before eraApio13; eraApio13 itself is neutral.
    double ehpv[2][3], ebpv[2][3], r[3][3], x, y, s;
    eraEpv00(utc1, utc2, ehpv, ebpv);
    eraPnm00b(utc1, utc2, r);
    eraBpn2xy(r, &x, &y);
    s = eraS00(utc1, utc2, x, y);
    eraApci(utc1, utc2, ebpv, ehpv[0], x, y, s, astrom);
    eraApio13(utc1, utc2, dut1, elong, phi, hm, xp, yp, phpa, tc, rh, wl, astrom);
    *eo = eraEors(r, s);
}

#endif

class ErfaEngine2000A : public ICoordinateEngine {
public:
    void J2000toObserved(INDI::IEquatorialCoordinates *j2000, double jd, INDI::IEquatorialCoordinates *jnow) override {
#ifdef HAVE_ERFA
        double utc1 = std::floor(jd) + 0.5, utc2 = jd - utc1;
        double ri = HOURS_TO_RAD(j2000->rightascension);
        double di = DEG_TO_RAD(j2000->declination);
        double rc, dc, eo;
        eraAtci13(ri, di, 0, 0, 0, 0, utc1, utc2, &rc, &dc, &eo);
        jnow->rightascension = RAD_TO_HOURS(eraAnp(rc - eo));
        jnow->declination = RAD_TO_DEG(dc);
#else
        INDI_UNUSED(j2000); INDI_UNUSED(jd); INDI_UNUSED(jnow);
#endif
    }

    void ObservedToJ2000(INDI::IEquatorialCoordinates *jnow, double jd, INDI::IEquatorialCoordinates *j2000) override {
#ifdef HAVE_ERFA
        double utc1 = std::floor(jd) + 0.5, utc2 = jd - utc1;
        double ra_jnow_rad = HOURS_TO_RAD(jnow->rightascension);
        double dec_jnow_rad = DEG_TO_RAD(jnow->declination);
        eraASTROM astrom;
        double eo;
        eraApci13(utc1, utc2, &astrom, &eo);
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
};

class ErfaEngine2000B : public ICoordinateEngine {
public:
    void J2000toObserved(INDI::IEquatorialCoordinates *j2000, double jd, INDI::IEquatorialCoordinates *jnow) override {
#ifdef HAVE_ERFA
        double utc1 = std::floor(jd) + 0.5, utc2 = jd - utc1;
        double ri = HOURS_TO_RAD(j2000->rightascension);
        double di = DEG_TO_RAD(j2000->declination);
        double rc, dc, eo;
        eraAtci00b(ri, di, 0, 0, 0, 0, utc1, utc2, &rc, &dc, &eo);
        jnow->rightascension = RAD_TO_HOURS(eraAnp(rc - eo));
        jnow->declination = RAD_TO_DEG(dc);
#else
        INDI_UNUSED(j2000); INDI_UNUSED(jd); INDI_UNUSED(jnow);
#endif
    }

    void ObservedToJ2000(INDI::IEquatorialCoordinates *jnow, double jd, INDI::IEquatorialCoordinates *j2000) override {
#ifdef HAVE_ERFA
        double utc1 = std::floor(jd) + 0.5, utc2 = jd - utc1;
        double ra_jnow_rad = HOURS_TO_RAD(jnow->rightascension);
        double dec_jnow_rad = DEG_TO_RAD(jnow->declination);
        eraASTROM astrom;
        double eo;
        eraApci00b(utc1, utc2, &astrom, &eo);
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
};

std::unique_ptr<ICoordinateEngine> createErfaEngine2000A() {
    return std::make_unique<ErfaEngine2000A>();
}

std::unique_ptr<ICoordinateEngine> createErfaEngine2000B() {
    return std::make_unique<ErfaEngine2000B>();
}
