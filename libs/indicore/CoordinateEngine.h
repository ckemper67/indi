#ifndef COORDINATE_ENGINE_H
#define COORDINATE_ENGINE_H

#include <memory>
#include <libastro.h>

/**
 * @brief Abstract interface for astronomical coordinate engines.
 */
class ICoordinateEngine {
public:
    virtual ~ICoordinateEngine() = default;
    virtual void J2000toObserved(INDI::IEquatorialCoordinates *j2000, double jd, INDI::IEquatorialCoordinates *jnow) = 0;
    virtual void ObservedToJ2000(INDI::IEquatorialCoordinates *jnow, double jd, INDI::IEquatorialCoordinates *j2000) = 0;
    virtual void EquatorialToHorizontal(INDI::IEquatorialCoordinates *object, INDI::IGeographicCoordinates *observer, double JD, INDI::IHorizontalCoordinates *position) = 0;

    virtual void GetPlanetObserved(int np, double jd, INDI::IEquatorialCoordinates *observed) = 0;
};

std::unique_ptr<ICoordinateEngine> createLibnovaEngine();
std::unique_ptr<ICoordinateEngine> createErfaEngine();

#endif // COORDINATE_ENGINE_H

