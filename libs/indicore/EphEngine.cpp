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
        const char* data_path = INDI_DATA_DIR "/eph/";

        if (!m_cemb) {
            m_cemb = std::make_unique<ephPLANctx>();
            if (ephPlanc(3, const_cast<char*>(data_path), m_cemb.get()) != 0) {
                m_cemb.reset();
                return;
            }
        }

        if (!m_cmoon) {
            m_cmoon = std::make_unique<ephMOONctx>();
            if (ephMoonc(const_cast<char*>(data_path), EPH_MOON_DE405, m_cmoon.get()) != 0) {
                m_cmoon.reset();
                return;
            }
        }

        if (np != 3 && (!m_cplan || m_current_planet != np)) {
            m_cplan = std::make_unique<ephPLANctx>();
            if (ephPlanc(np, const_cast<char*>(data_path), m_cplan.get()) != 0) {
                m_cplan.reset();
                return;
            }
            m_current_planet = np;
        }

        // Convert JD (UTC) to TDB via ERFA: UTC -> TAI -> TT.
        // TDB-TT < 2ms geocentric, so TT is used as TDB.
        double utc1 = std::floor(jd) + 0.5, utc2 = jd - utc1;
        double tai1, tai2, tt1, tt2;
        eraUtctai(utc1, utc2, &tai1, &tai2);
        eraTaitt(tai1, tai2, &tt1, &tt2);
        double ut1_mjd = jd - 2400000.5;
        double tdb_mjd = (tt1 + tt2) - 2400000.5;

        double rast, dast, rapp, dapp, eo, diam;
        ephPLANctx* pctx = (np == 3) ? m_cemb.get() : m_cplan.get();
        ephRdplan(m_cmoon.get(), m_cemb.get(), pctx, ut1_mjd, tdb_mjd, np, 0, 0, 0,
                  &rast, &dast, &rapp, &dapp, &eo, &diam);
        observed->rightascension = RAD_TO_HOURS(eraAnp(rapp));
        observed->declination = RAD_TO_DEG(dapp);
#else
        INDI_UNUSED(np); INDI_UNUSED(jd); INDI_UNUSED(observed);
#endif
    }

private:
#ifdef HAVE_ERFA
    // icor=1: LLR corrections, icor=2: DE405 corrections
    static constexpr int EPH_MOON_DE405 = 2;
    std::unique_ptr<ephPLANctx> m_cemb;
    std::unique_ptr<ephPLANctx> m_cplan;
    std::unique_ptr<ephMOONctx> m_cmoon;
    int m_current_planet = -1;
#endif
};

class EphEngineINDI : public IPlanetaryEngine {
public:
    void GetPlanetObserved(int np, double jd, INDI::IEquatorialCoordinates *observed) override {
#ifdef HAVE_ERFA
        // TODO Milestone 5: replace with packed/truncated VSOP2010 loader.
        // Currently falls back to the full engine — callers requesting EPH_INDI
        // get EPH_FULL precision and footprint until the truncated loader is implemented.
        IDLog("EphEngineINDI: truncated loader not yet implemented, using full EPH engine\n");
        m_full.GetPlanetObserved(np, jd, observed);
#else
        INDI_UNUSED(np); INDI_UNUSED(jd); INDI_UNUSED(observed);
#endif
    }

private:
    EphEngineFull m_full;
};

std::unique_ptr<IPlanetaryEngine> createEphEngineFull() {
    return std::make_unique<EphEngineFull>();
}

std::unique_ptr<IPlanetaryEngine> createEphEngineINDI() {
    return std::make_unique<EphEngineINDI>();
}
