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

    virtual void GetPlanetTopocentric(int np, double jd,
                                      INDI::AstrometricContext &ctx,
                                      INDI::TopocentricApparent *out) = 0;
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

    // Typed frame-safe variants (M5 topocentric promotion)

    virtual void J2000toGeocentric(const INDI::J2000Coordinates *j2000,
                                   double jd,
                                   INDI::GeocentricApparent *out) = 0;

    virtual void J2000toGeocentricFull(const INDI::CatalogStar *star,
                                       double jd,
                                       INDI::GeocentricApparent *out) = 0;

    virtual void GeocentricToJ2000(const INDI::GeocentricApparent *apparent,
                                   double jd,
                                   INDI::J2000Coordinates *out) = 0;

    virtual void J2000toTopocentric(const INDI::J2000Coordinates *j2000,
                                    INDI::AstrometricContext &ctx,
                                    double jd,
                                    INDI::TopocentricApparent *out) = 0;

    virtual void TopocentricToJ2000(const INDI::TopocentricApparent *apparent,
                                    INDI::AstrometricContext &ctx,
                                    double jd,
                                    INDI::J2000Coordinates *out) = 0;
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
