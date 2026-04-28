#include "CoordinateEngine.h"
#include "indidevapi.h"
#include "indicom.h"
#ifdef HAVE_ERFA
#include <erfa.h>
#include "eph/eph.h"
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

    void GetPlanetObserved(int np, double jd, INDI::IEquatorialCoordinates *observed) override {
#ifdef HAVE_ERFA
        // Use heap-allocated contexts to avoid stack overflow (each context is ~17.5 MB)
        static std::unique_ptr<ephPLANctx> cemb = nullptr;
        static std::unique_ptr<ephPLANctx> cplan = nullptr;
        static std::unique_ptr<ephMOONctx> cmoon = nullptr;
        static int current_planet = -1;

        // Use the absolute path defined in CMake
        const char* data_path = INDI_DATA_DIR "/eph/";

        if (!cemb) {
            cemb = std::make_unique<ephPLANctx>();
            int status = ephPlanc(3, const_cast<char*>(data_path), cemb.get());
            if (status != 0) {
                fprintf(stderr, "ERFA Engine: Failed to load Earth context from %s (status %d)\n", data_path, status);
                fflush(stderr);
            }
        }
        
        if (!cmoon) {
            cmoon = std::make_unique<ephMOONctx>();
            int status = ephMoonc(const_cast<char*>(data_path), 2, cmoon.get());
            if (status != 0) {
                fprintf(stderr, "ERFA Engine: Failed to load Moon context (status %d)\n", status);
                fflush(stderr);
            }
        }

        if (np != 3 && (!cplan || current_planet != np)) {
            cplan = std::make_unique<ephPLANctx>();
            int status = ephPlanc(np, const_cast<char*>(data_path), cplan.get());
            if (status != 0) {
                fprintf(stderr, "ERFA Engine: Failed to load Planet %d context (status %d)\n", np, status);
                fflush(stderr);
            }
            current_planet = np;
        }

        // We use UTC for UT1 and TDB for this geocentric baseline (neglecting dT for M3)
        double ut1 = jd - 2400000.5;
        double tdb = ut1;

        double rast, dast, rapp, dapp, eo, diam;
        // elong=0, phi=0, hm=0 gives geocentric apparent place
        ephPLANctx* pctx = (np == 3) ? cemb.get() : cplan.get();
        if (pctx && cmoon && cemb) {
            ephRdplan(cmoon.get(), cemb.get(), pctx, 
                      ut1, tdb, np, 0, 0, 0, 
                      &rast, &dast, &rapp, &dapp, &eo, &diam);

            observed->rightascension = RAD_TO_HOURS(eraAnp(rapp));
            observed->declination = RAD_TO_DEG(dapp);
        } else {
            observed->rightascension = 0;
            observed->declination = 0;
        }
#else
        INDI_UNUSED(np);
        INDI_UNUSED(jd);
        INDI_UNUSED(observed);
#endif
    }
};

std::unique_ptr<ICoordinateEngine> createErfaEngine() {
    return std::make_unique<ErfaEngine>();
}
