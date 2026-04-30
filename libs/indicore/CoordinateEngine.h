#ifndef COORDINATE_ENGINE_H
#define COORDINATE_ENGINE_H

#include <libastro.h>
#include <memory>

/**
 * @brief Interface for planetary/lunar ephemeris engines.
 */
class IPlanetaryEngine {
public:
    virtual ~IPlanetaryEngine() = default;
    virtual void GetPlanetObserved(int np, double jd, INDI::IEquatorialCoordinates *observed) = 0;
};

/**
 * @brief Abstract interface for stellar coordinate engines.
 */
class ICoordinateEngine {
public:
    virtual ~ICoordinateEngine() = default;

    virtual void J2000toObserved(INDI::IEquatorialCoordinates *j2000, 
                                 double jd, 
                                 INDI::IEquatorialCoordinates *jnow) = 0;

    virtual void ObservedToJ2000(INDI::IEquatorialCoordinates *jnow, 
                                 double jd, 
                                 INDI::IEquatorialCoordinates *j2000) = 0;

    virtual void EquatorialToHorizontal(INDI::IEquatorialCoordinates *object,
                                        INDI::IGeographicCoordinates *observer,
                                        double jd,
                                        INDI::IHorizontalCoordinates *position) = 0;

    virtual void HorizontalToEquatorial(INDI::IHorizontalCoordinates *object,
                                        INDI::IGeographicCoordinates *observer,
                                        double jd,
                                        INDI::IEquatorialCoordinates *position) = 0;
};

// Factory functions for Stellar Engines
std::unique_ptr<ICoordinateEngine> createLibnovaStellarEngine();
std::unique_ptr<ICoordinateEngine> createErfaEngine2000A();
std::unique_ptr<ICoordinateEngine> createErfaEngine2000B();

// Factory functions for Planetary Engines
std::unique_ptr<IPlanetaryEngine> createLibnovaPlanetaryEngine();
std::unique_ptr<IPlanetaryEngine> createEphEngineFull();
std::unique_ptr<IPlanetaryEngine> createEphEngineINDI();

#endif // COORDINATE_ENGINE_H
