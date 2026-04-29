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
/**
 * Internal helper to build an eraASTROM context using IAU 2000B.
 */
static void populate_astrom_2000b(double date1, double date2, eraASTROM *astrom, double *eo) {
    double rbpn[3][3], pvh[2][3], pvb[2][3];
    
    // 1. IAU 2000B BPN and Equation of Origins
    eraPnm00b(date1, date2, rbpn);
    *eo = eraEe00b(date1, date2);

    // 2. Convert Equinox-based BPN to CIO-based (CIRS) BPN
    // R_cio = R_z(-eo) * R_equinox
    double rz[3][3], rbpn_cio[3][3];
    eraIr(rz);
    eraRz(-(*eo), rz);
    eraRxr(rz, rbpn, rbpn_cio);
    
    // 3. Populate context from Earth ephemeris
    eraEpv00(date1, date2, pvh, pvb);
    eraApci(date1, date2, pvb, pvh[0], 0, 0, 0, astrom);
    
    // 4. Force context to use the 2000B matrix
    for (int i=0; i<3; i++) for (int j=0; j<3; j++) astrom->bpn[i][j] = rbpn_cio[i][j];
}

static void atci00b(double rc, double dc, double date1, double date2,
                    double *ri, double *di, double *eo) {
    eraASTROM astrom;
    populate_astrom_2000b(date1, date2, &astrom, eo);
    eraAtciqz(rc, dc, &astrom, ri, di);
}

static void atic00b(double ri, double di, double date1, double date2,
                    double *rc, double *dc) {
    eraASTROM astrom;
    double eo;
    populate_astrom_2000b(date1, date2, &astrom, &eo);
    // Convert Equinox RA to CIO RA (RI = RA_equinox + eo)
    eraAticq(eraAnp(ri + eo), di, &astrom, rc, dc);
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
        eraAtic13(eraAnp(ra_jnow_rad + eo), dec_jnow_rad, utc1, utc2, &ri, &di, &eo);
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
        eraApci13(utc1, utc2, &astrom, &eo);
        double ra_rad = HOURS_TO_RAD(object->rightascension);
        double dec_rad = DEG_TO_RAD(object->declination);
        double ra_cirs = eraAnp(ra_rad + eo);
        double aob, zob, hob, dob, rob;
        eraAtioq(ra_cirs, dec_rad, &astrom, &aob, &zob, &hob, &dob, &rob);
        position->azimuth = RAD_TO_DEG(aob);
        position->altitude = RAD_TO_DEG(rob);
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
        atci00b(ri, di, utc1, utc2, &rc, &dc, &eo);
        jnow->rightascension = RAD_TO_HOURS(eraAnp(rc - eo));
        jnow->declination = RAD_TO_DEG(dc);
#else
        INDI_UNUSED(j2000); INDI_UNUSED(jd); INDI_UNUSED(jnow);
#endif
    }

    void ObservedToJ2000(INDI::IEquatorialCoordinates *jnow, double jd, INDI::IEquatorialCoordinates *j2000) override {
#ifdef HAVE_ERFA
        double utc1 = std::floor(jd) + 0.5, utc2 = jd - utc1;
        double ri = HOURS_TO_RAD(jnow->rightascension);
        double di = DEG_TO_RAD(jnow->declination);
        double rc, dc;
        atic00b(ri, di, utc1, utc2, &rc, &dc);
        j2000->rightascension = RAD_TO_HOURS(eraAnp(rc));
        j2000->declination = RAD_TO_DEG(dc);
#else
        INDI_UNUSED(jnow); INDI_UNUSED(jd); INDI_UNUSED(j2000);
#endif
    }

    void EquatorialToHorizontal(INDI::IEquatorialCoordinates *object, INDI::IGeographicCoordinates *observer, double JD, INDI::IHorizontalCoordinates *position) override {
#ifdef HAVE_ERFA
        double utc1 = std::floor(JD) + 0.5, utc2 = JD - utc1;
        eraASTROM astrom;
        double eo;
        populate_astrom_2000b(utc1, utc2, &astrom, &eo);
        double ra_rad = HOURS_TO_RAD(object->rightascension);
        double dec_rad = DEG_TO_RAD(object->declination);
        double ra_cirs = eraAnp(ra_rad + eo);
        double aob, zob, hob, dob, rob;
        eraAtioq(ra_cirs, dec_rad, &astrom, &aob, &zob, &hob, &dob, &rob);
        position->azimuth = RAD_TO_DEG(aob);
        position->altitude = RAD_TO_DEG(rob);
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
