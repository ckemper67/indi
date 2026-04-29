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
#ifdef HAVE_ERFA
#include <erfa.h>
#include "eph/eph.h"
#endif
#include <cmath>
#include <memory>

class EphEngineFull : public IPlanetaryEngine {
public:
    void GetPlanetObserved(int np, double jd, INDI::IEquatorialCoordinates *observed) override {
#ifdef HAVE_ERFA
        static std::unique_ptr<ephPLANctx> cemb = nullptr;
        static std::unique_ptr<ephPLANctx> cplan = nullptr;
        static std::unique_ptr<ephMOONctx> cmoon = nullptr;
        static int current_planet = -1;

        const char* data_path = INDI_DATA_DIR "/eph/";

        if (!cemb) {
            cemb = std::make_unique<ephPLANctx>();
            ephPlanc(3, const_cast<char*>(data_path), cemb.get());
        }
        
        if (!cmoon) {
            cmoon = std::make_unique<ephMOONctx>();
            ephMoonc(const_cast<char*>(data_path), 2, cmoon.get());
        }

        if (np != 3 && (!cplan || current_planet != np)) {
            cplan = std::make_unique<ephPLANctx>();
            ephPlanc(np, const_cast<char*>(data_path), cplan.get());
            current_planet = np;
        }

        double ut1 = jd - 2400000.5, tdb = ut1;
        double rast, dast, rapp, dapp, eo, diam;
        ephPLANctx* pctx = (np == 3) ? cemb.get() : cplan.get();
        if (pctx && cmoon && cemb) {
            ephRdplan(cmoon.get(), cemb.get(), pctx, ut1, tdb, np, 0, 0, 0, &rast, &dast, &rapp, &dapp, &eo, &diam);
            observed->rightascension = RAD_TO_HOURS(eraAnp(rapp));
            observed->declination = RAD_TO_DEG(dapp);
        }
#else
        INDI_UNUSED(np); INDI_UNUSED(jd); INDI_UNUSED(observed);
#endif
    }
};

class EphEngineINDI : public IPlanetaryEngine {
public:
    void GetPlanetObserved(int np, double jd, INDI::IEquatorialCoordinates *observed) override {
#ifdef HAVE_ERFA
        // Placeholder for Milestone 5: Packed/Truncated loader
        // For now, falls back to Full implementation
        static EphEngineFull full;
        full.GetPlanetObserved(np, jd, observed);
#else
        INDI_UNUSED(np); INDI_UNUSED(jd); INDI_UNUSED(observed);
#endif
    }
};

std::unique_ptr<IPlanetaryEngine> createEphEngineFull() {
    return std::make_unique<EphEngineFull>();
}

std::unique_ptr<IPlanetaryEngine> createEphEngineINDI() {
    return std::make_unique<EphEngineINDI>();
}
