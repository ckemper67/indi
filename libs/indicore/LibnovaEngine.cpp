#include "CoordinateEngine.h"
#include "indicom.h"
#include <libnova/julian_day.h>
#include <libnova/transform.h>
#include <libnova/precession.h>
#include <libnova/nutation.h>
#include <libnova/aberration.h>
#include <math.h>
#include <memory>

/**
 * @brief Internal helper from original libastro.cpp
 */
static void local_ln_get_equ_nut(ln_equ_posn *posn, double jd, bool reverse)
{
    struct ln_nutation nut;
    ln_get_nutation (jd, &nut);

    double mean_ra, mean_dec, delta_ra, delta_dec;

    mean_ra = DEG_TO_RAD(posn->ra);
    mean_dec = DEG_TO_RAD(posn->dec);

    double nut_ecliptic = DEG_TO_RAD(nut.ecliptic + nut.obliquity);
    double sin_ecliptic = sin(nut_ecliptic);
    double sin_ra = sin(mean_ra);
    double cos_ra = cos(mean_ra);
    double tan_dec = tan(mean_dec);

    delta_ra = (cos (nut_ecliptic) + sin_ecliptic * sin_ra * tan_dec) * nut.longitude - cos_ra * tan_dec * nut.obliquity;
    delta_dec = (sin_ecliptic * cos_ra) * nut.longitude + sin_ra * nut.obliquity;

    if (reverse) {
        delta_ra = -delta_ra;
        delta_dec = -delta_dec;
    }
    posn->ra += delta_ra;
    posn->dec += delta_dec;
}

class LibnovaEngine : public ICoordinateEngine {
public:
    void J2000toObserved(INDI::IEquatorialCoordinates *j2000, double jd, INDI::IEquatorialCoordinates *jnow) override {
        ln_equ_posn tempPosn;
        struct ln_equ_posn libnova_J2000Pos = {j2000->rightascension * 15.0, j2000->declination };

        ln_get_equ_prec2(&libnova_J2000Pos, JD2000, jd, &tempPosn);
        local_ln_get_equ_nut(&tempPosn, jd, false);

        struct ln_equ_posn libnova_observed;
        ln_get_equ_aber(&tempPosn, jd, &libnova_observed);

        jnow->rightascension = libnova_observed.ra / 15.0;
        jnow->declination = libnova_observed.dec;
    }

    void ObservedToJ2000(INDI::IEquatorialCoordinates *observed, double jd, INDI::IEquatorialCoordinates *j2000) override {
        ln_equ_posn tempPos;
        struct ln_equ_posn libnova_observed = {observed->rightascension * 15.0, observed->declination};
        
        ln_get_equ_aber(&libnova_observed, jd, &tempPos);
        tempPos.ra = libnova_observed.ra - (tempPos.ra - libnova_observed.ra);
        tempPos.dec = libnova_observed.dec * 2 - tempPos.dec;

        local_ln_get_equ_nut(&tempPos, jd, true);

        struct ln_equ_posn libnova_J2000Pos;
        ln_get_equ_prec2(&tempPos, jd, JD2000, &libnova_J2000Pos);

        j2000->rightascension = libnova_J2000Pos.ra / 15.0;
        j2000->declination = libnova_J2000Pos.dec;
    }

    void EquatorialToHorizontal(INDI::IEquatorialCoordinates *object, INDI::IGeographicCoordinates *observer, double JD, INDI::IHorizontalCoordinates *position) override {
        struct ln_lnlat_posn libnova_location = {observer->longitude > 180 ? observer->longitude - 360 : observer->longitude, observer->latitude};
        struct ln_equ_posn libnova_object = {object->rightascension * 15.0, object->declination};
        struct ln_hrz_posn horizontalPos;
        ln_get_hrz_from_equ(&libnova_object, &libnova_location, JD, &horizontalPos);
        position->azimuth = range360(180 + horizontalPos.az);
        position->altitude = horizontalPos.alt;
    }
};

std::unique_ptr<ICoordinateEngine> createLibnovaEngine() {
    return std::make_unique<LibnovaEngine>();
}
