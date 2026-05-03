#include "CoordinateEngine.h"
#include "indidevapi.h"
#include "indicom.h"
#include <libnova/mercury.h>
#include <libnova/venus.h>
#include <libnova/mars.h>
#include <libnova/jupiter.h>
#include <libnova/saturn.h>
#include <libnova/uranus.h>
#include <libnova/neptune.h>
#include <libnova/solar.h>
#include <libnova/lunar.h>
#include <libnova/transform.h>
#include <libnova/precession.h>
#include <libnova/nutation.h>
#include <libnova/aberration.h>
#include <libnova/parallax.h>
#include <cmath>
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

class LibnovaStellarEngine : public ICoordinateEngine {
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

    void HorizontalToEquatorial(INDI::IHorizontalCoordinates *object, INDI::IGeographicCoordinates *observer, double JD, INDI::IEquatorialCoordinates *position) override {
        ln_lnlat_posn libnova_location = {observer->longitude > 180 ? observer->longitude - 360 : observer->longitude, observer->latitude};
        ln_hrz_posn libnova_object = {range360(object->azimuth + 180), object->altitude};
        ln_equ_posn equatorialPos;
        get_equ_from_hrz(&libnova_object, &libnova_location, JD, &equatorialPos);
        position->rightascension = equatorialPos.ra / 15.0;
        position->declination = equatorialPos.dec;
    }

    // libnova has no topocentric support; all typed methods fall back to geocentric.
    void J2000toGeocentric(const INDI::J2000Coordinates *j2000, double jd, INDI::GeocentricApparent *out) override {
        J2000toObserved(const_cast<INDI::J2000Coordinates*>(j2000), jd, out);
    }

    // libnova has no parallax or proper-motion support; ignore extra catalog fields.
    void J2000toGeocentricFull(const INDI::CatalogStar *star, double jd, INDI::GeocentricApparent *out) override {
        J2000toObserved(const_cast<INDI::CatalogStar*>(star), jd, out);
    }

    void GeocentricToJ2000(const INDI::GeocentricApparent *apparent, double jd, INDI::J2000Coordinates *out) override {
        ObservedToJ2000(const_cast<INDI::GeocentricApparent*>(apparent), jd, out);
    }

    void J2000toTopocentric(const INDI::J2000Coordinates *j2000, INDI::AstrometricContext &ctx, double jd,
                            INDI::TopocentricApparent *out) override {
        INDI_UNUSED(ctx);
        J2000toObserved(const_cast<INDI::J2000Coordinates*>(j2000), jd, out);
    }

    void TopocentricToJ2000(const INDI::TopocentricApparent *apparent, INDI::AstrometricContext &ctx, double jd,
                            INDI::J2000Coordinates *out) override {
        INDI_UNUSED(ctx);
        ObservedToJ2000(const_cast<INDI::TopocentricApparent*>(apparent), jd, out);
    }
};

class LibnovaPlanetaryEngine : public IPlanetaryEngine {
public:
    void GetPlanetTopocentric(int np, double jd, INDI::AstrometricContext &ctx,
                              INDI::TopocentricApparent *out) override {
        // Get geocentric apparent position
        struct ln_equ_posn geo;
        switch (np) {
            case 1: ln_get_mercury_equ_coords(jd, &geo); break;
            case 2: ln_get_venus_equ_coords  (jd, &geo); break;
            case 3: ln_get_lunar_equ_coords  (jd, &geo); break;
            case 4: ln_get_mars_equ_coords   (jd, &geo); break;
            case 5: ln_get_jupiter_equ_coords(jd, &geo); break;
            case 6: ln_get_saturn_equ_coords (jd, &geo); break;
            case 7: ln_get_uranus_equ_coords (jd, &geo); break;
            case 8: ln_get_neptune_equ_coords(jd, &geo); break;
            default: ln_get_solar_equ_coords (jd, &geo); break;
        }

        // Earth-body distance in AU (Moon function returns km)
        static constexpr double KM_PER_AU = 149597870.7;
        double dist_au;
        switch (np) {
            case 1: dist_au = ln_get_mercury_earth_dist(jd); break;
            case 2: dist_au = ln_get_venus_earth_dist  (jd); break;
            case 3: dist_au = ln_get_lunar_earth_dist  (jd) / KM_PER_AU; break;
            case 4: dist_au = ln_get_mars_earth_dist   (jd); break;
            case 5: dist_au = ln_get_jupiter_earth_dist(jd); break;
            case 6: dist_au = ln_get_saturn_earth_dist (jd); break;
            case 7: dist_au = ln_get_uranus_earth_dist (jd); break;
            case 8: dist_au = ln_get_neptune_earth_dist(jd); break;
            default: {
                struct ln_rect_posn sr;
                ln_get_solar_geo_coords(jd, &sr);
                dist_au = std::sqrt(sr.X*sr.X + sr.Y*sr.Y + sr.Z*sr.Z);
                break;
            }
        }

        // libnova expects longitude in [-180, +180] E
        double lon_ln = ctx.observer.longitude > 180.0
                        ? ctx.observer.longitude - 360.0
                        : ctx.observer.longitude;
        struct ln_lnlat_posn observer = { lon_ln, ctx.observer.latitude };

        struct ln_equ_posn parallax;
        ln_get_parallax(&geo, dist_au, &observer, ctx.observer.elevation, jd, &parallax);

        out->rightascension = (geo.ra + parallax.ra) / 15.0;
        out->declination    =  geo.dec + parallax.dec;
    }

    void GetPlanetObserved(int np, double jd, INDI::IEquatorialCoordinates *observed) override {
        struct ln_equ_posn equatorialPos;
        switch(np) {
            case 1: ln_get_mercury_equ_coords(jd, &equatorialPos); break;
            case 2: ln_get_venus_equ_coords(jd, &equatorialPos); break;
            case 3: ln_get_lunar_equ_coords(jd, &equatorialPos); break;
            case 4: ln_get_mars_equ_coords(jd, &equatorialPos); break;
            case 5: ln_get_jupiter_equ_coords(jd, &equatorialPos); break;
            case 6: ln_get_saturn_equ_coords(jd, &equatorialPos); break;
            case 7: ln_get_uranus_equ_coords(jd, &equatorialPos); break;
            case 8: ln_get_neptune_equ_coords(jd, &equatorialPos); break;
            default: ln_get_solar_equ_coords(jd, &equatorialPos); break;
        }
        observed->rightascension = equatorialPos.ra / 15.0;
        observed->declination = equatorialPos.dec;
    }
};

std::unique_ptr<ICoordinateEngine> createLibnovaStellarEngine() {
    return std::make_unique<LibnovaStellarEngine>();
}

std::unique_ptr<IPlanetaryEngine> createLibnovaPlanetaryEngine() {
    return std::make_unique<LibnovaPlanetaryEngine>();
}
