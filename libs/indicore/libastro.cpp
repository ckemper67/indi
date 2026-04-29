#include "libastro.h"
#include "indicom.h"
#include "CoordinateEngine.h"
#include <math.h>
#include <memory>
#include <mutex>

namespace INDI
{

static StellarEngine currentStellarType = StellarEngine::LIBNOVA;
static PlanetaryEngine currentPlanetaryType = PlanetaryEngine::LIBNOVA;

static std::unique_ptr<ICoordinateEngine> stellarEngine = nullptr;
static std::unique_ptr<IPlanetaryEngine> planetaryEngine = nullptr;

void setStellarEngine(StellarEngine engine) {
    currentStellarType = engine;
    stellarEngine.reset(); // Force re-init
}

void setPlanetaryEngine(PlanetaryEngine engine) {
    currentPlanetaryType = engine;
    planetaryEngine.reset(); // Force re-init
}

ICoordinateEngine& getStellarEngine() {
    if (!stellarEngine) {
        switch(currentStellarType) {
            case StellarEngine::ERFA_2000A: stellarEngine = createErfaEngine2000A(); break;
            case StellarEngine::ERFA_2000B: stellarEngine = createErfaEngine2000B(); break;
            default:                        stellarEngine = createLibnovaStellarEngine(); break;
        }
    }
    return *stellarEngine;
}

IPlanetaryEngine& getPlanetaryEngine() {
    if (!planetaryEngine) {
        switch(currentPlanetaryType) {
            case PlanetaryEngine::EPH_FULL: planetaryEngine = createEphEngineFull(); break;
            case PlanetaryEngine::EPH_INDI: planetaryEngine = createEphEngineINDI(); break;
            default:                        planetaryEngine = createLibnovaPlanetaryEngine(); break;
        }
    }
    return *planetaryEngine;
}

void ObservedToJ2000(IEquatorialCoordinates * observed, double jd, IEquatorialCoordinates * J2000pos)
{
    getStellarEngine().ObservedToJ2000(observed, jd, J2000pos);
}

void J2000toObserved(IEquatorialCoordinates *J2000pos, double jd, IEquatorialCoordinates *observed)
{
    getStellarEngine().J2000toObserved(J2000pos, jd, observed);
}

void EquatorialToHorizontal(IEquatorialCoordinates *object, IGeographicCoordinates *observer, double JD,
                            IHorizontalCoordinates *position)
{
    getStellarEngine().EquatorialToHorizontal(object, observer, JD, position);
}

void GetPlanetObserved(int np, double jd, IEquatorialCoordinates *observed)
{
    getPlanetaryEngine().GetPlanetObserved(np, jd, observed);
}

void HorizontalToEquatorial(IHorizontalCoordinates *object, IGeographicCoordinates *observer, double JD,
                            IEquatorialCoordinates *position)
{
    // Legacy fallback: This will be moved to the engine interface in Milestone 5
    ln_lnlat_posn libnova_location = {observer->longitude > 180 ? observer->longitude - 360 : observer->longitude, observer->latitude};
    ln_hrz_posn libnova_object = {range360(object->azimuth + 180), object->altitude};
    ln_equ_posn equatorialPos;
    get_equ_from_hrz(&libnova_object, &libnova_location, JD, &equatorialPos);
    position->rightascension = equatorialPos.ra / 15.0;
    position->declination = equatorialPos.dec;
}

}
