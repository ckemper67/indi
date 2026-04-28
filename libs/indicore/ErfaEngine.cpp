#include "CoordinateEngine.h"
#include "indidevapi.h"
#include "indicom.h"
#ifdef HAVE_ERFA
#include <erfa.h>
#endif
#include <cmath>
#include <memory>

class ErfaEngine : public ICoordinateEngine {
public:
    void J2000toObserved(INDI::IEquatorialCoordinates *j2000, double jd, INDI::IEquatorialCoordinates *jnow) override {
#ifdef HAVE_ERFA
        double utc1 = std::floor(jd) + 0.5;
        double utc2 = jd - utc1;

        // Convert ICRS/J2000 to CIRS (Geocentric Apparent)
        double ri = HOURS_TO_RAD(j2000->rightascension);
        double di = DEG_TO_RAD(j2000->declination);
        double rc, dc, eo;
        eraAtci13(ri, di, 0, 0, 0, 0, utc1, utc2, &rc, &dc, &eo);

        // Convert CIRS RA to equinox-based apparent RA (JNow)
        jnow->rightascension = RAD_TO_HOURS(eraAnp(rc - eo));
        jnow->declination = RAD_TO_DEG(dc);
#else
        INDI_UNUSED(j2000);
        INDI_UNUSED(jd);
        INDI_UNUSED(jnow);
#endif
    }

    void ObservedToJ2000(INDI::IEquatorialCoordinates *jnow, double jd, INDI::IEquatorialCoordinates *j2000) override {
#ifdef HAVE_ERFA
        double utc1 = std::floor(jd) + 0.5;
        double utc2 = jd - utc1;

        // Equinox-based apparent RA to CIRS RA
        double ra_jnow_rad = HOURS_TO_RAD(jnow->rightascension);
        double dec_jnow_rad = DEG_TO_RAD(jnow->declination);
        
        // We need 'eo' to get from Equinox to CIRS. 
        eraASTROM astrom;
        double eo;
        eraApci13(utc1, utc2, &astrom, &eo);
        
        double ri, di;
        // eraAtic13 goes from CIRS to ICRS
        eraAtic13(eraAnp(ra_jnow_rad + eo), dec_jnow_rad, utc1, utc2, &ri, &di, &eo);

        j2000->rightascension = RAD_TO_HOURS(eraAnp(ri));
        j2000->declination = RAD_TO_DEG(di);
#else
        INDI_UNUSED(jnow);
        INDI_UNUSED(jd);
        INDI_UNUSED(j2000);
#endif
    }

    void EquatorialToHorizontal(INDI::IEquatorialCoordinates *object, INDI::IGeographicCoordinates *observer, double JD, INDI::IHorizontalCoordinates *position) override {
#ifdef HAVE_ERFA
        double utc1 = std::floor(JD) + 0.5;
        double utc2 = JD - utc1;

        // Populate context for geocentric transformation
        eraASTROM astrom;
        double eo;
        eraApci13(utc1, utc2, &astrom, &eo);

        // Convert Equinox-based JNow to CIRS
        double ra_rad = HOURS_TO_RAD(object->rightascension);
        double dec_rad = DEG_TO_RAD(object->declination);
        double ra_cirs = eraAnp(ra_rad + eo);

        // Geocentric transformation to observed Az/El (ignoring location for now)
        double aob, zob, hob, dob, rob;
        eraAtioq(ra_cirs, dec_rad, &astrom, &aob, &zob, &hob, &dob, &rob);

        // ERFA returns Az in radians [0, 2pi], El in radians
        // INDI expects Az in degrees [0, 360], 0=North
        position->azimuth = RAD_TO_DEG(aob);
        position->altitude = RAD_TO_DEG(rob);
#else
        INDI_UNUSED(object);
        INDI_UNUSED(observer);
        INDI_UNUSED(JD);
        INDI_UNUSED(position);
#endif
    }
};

std::unique_ptr<ICoordinateEngine> createErfaEngine() {
    return std::make_unique<ErfaEngine>();
}
