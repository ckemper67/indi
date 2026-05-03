#include "libastro.h"
#include "indicom.h"
#include "CoordinateEngine.h"
#include <math.h>
#include <memory>
#include <mutex>

namespace INDI
{

static std::mutex s_engineMutex;
static StellarEngine currentStellarType = StellarEngine::LIBNOVA;
static PlanetaryEngine currentPlanetaryType = PlanetaryEngine::LIBNOVA;

static std::unique_ptr<ICoordinateEngine> stellarEngine = nullptr;
static std::unique_ptr<IPlanetaryEngine> planetaryEngine = nullptr;

void setStellarEngine(StellarEngine engine) {
    std::lock_guard<std::mutex> lock(s_engineMutex);
    currentStellarType = engine;
    stellarEngine.reset();
}

void setPlanetaryEngine(PlanetaryEngine engine) {
    std::lock_guard<std::mutex> lock(s_engineMutex);
    currentPlanetaryType = engine;
    planetaryEngine.reset();
}

ICoordinateEngine& getStellarEngine() {
    std::lock_guard<std::mutex> lock(s_engineMutex);
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
    std::lock_guard<std::mutex> lock(s_engineMutex);
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

void J2000toGeocentric(J2000Coordinates *j2000, double jd, GeocentricApparent *out)
{
    getStellarEngine().J2000toGeocentric(j2000, jd, out);
}

void J2000toGeocentricFull(const CatalogStar *star, double jd, GeocentricApparent *out)
{
    getStellarEngine().J2000toGeocentricFull(star, jd, out);
}

void GeocentricToJ2000(GeocentricApparent *apparent, double jd, J2000Coordinates *out)
{
    getStellarEngine().GeocentricToJ2000(apparent, jd, out);
}

void J2000toTopocentric(J2000Coordinates *j2000, AstrometricContext &ctx, double jd, TopocentricApparent *out)
{
    getStellarEngine().J2000toTopocentric(j2000, ctx, jd, out);
}

void TopocentricToJ2000(TopocentricApparent *apparent, AstrometricContext &ctx, double jd, J2000Coordinates *out)
{
    getStellarEngine().TopocentricToJ2000(apparent, ctx, jd, out);
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

void GetPlanetTopocentric(int np, double jd, AstrometricContext &ctx, TopocentricApparent *out)
{
    getPlanetaryEngine().GetPlanetTopocentric(np, jd, ctx, out);
}

void HorizontalToEquatorial(IHorizontalCoordinates *object, IGeographicCoordinates *observer, double JD,
                            IEquatorialCoordinates *position)
{
    getStellarEngine().HorizontalToEquatorial(object, observer, JD, position);
}

void EquatorialToHorizontal(const GeocentricApparent *object, IGeographicCoordinates *observer, double JD,
                            IHorizontalCoordinates *position)
{
    getStellarEngine().EquatorialToHorizontal(const_cast<GeocentricApparent*>(object), observer, JD, position);
}

void EquatorialToHorizontal(const TopocentricApparent *object, IGeographicCoordinates *observer, double JD,
                            IHorizontalCoordinates *position)
{
    getStellarEngine().EquatorialToHorizontal(const_cast<TopocentricApparent*>(object), observer, JD, position);
}

}
