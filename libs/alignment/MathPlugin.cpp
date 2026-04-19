/*!
 * \file MathPlugin.cpp
 *
 * \author Roger James
 * \date 13th November 2013
 *
 */

#include "MathPlugin.h"

#include <indicom.h>
#include <libnova/sidereal_time.h>

#include <cmath>

namespace INDI
{
namespace AlignmentSubsystem
{
bool MathPlugin::Initialise(InMemoryDatabase *pInMemoryDatabase)
{
    MathPlugin::pInMemoryDatabase = pInMemoryDatabase;
    SanitizePolarEntries(pInMemoryDatabase, ApproximateMountAlignment);
    return true;
}

void MathPlugin::SanitizePolarEntries(InMemoryDatabase *pDb, MountAlignment_t mountAlignment)
{
    if (!pDb) return;

    // Threshold in degrees for celestial Dec (EQ) or encoder El (altaz).
    // For EQ, 88 deg catches the truly degenerate zone where cos(Dec)->0
    // while leaving multi-point models that include near-pole coverage intact.
    static constexpr double kPolarThresholdDeg = 88.0;
    // Shift degenerate entries so that celestial Dec (EQ) or encoder El
    // (altaz) ends up at this value, well within the conditioned region.
    static constexpr double kTargetPitchDeg = 80.0;

    auto &db = pDb->GetAlignmentDatabase();
    IGeographicCoordinates location;
    bool haveLoc = pDb->GetDatabaseReferencePosition(location);

    TelescopeDirectionVectorSupportFunctions sf;

    for (auto &entry : db)
    {
        if (mountAlignment == ZENITH)
        {
            // Altaz: degeneracy at the zenith -- check encoder El via TDV.z
            double sinThresh = std::sin(DEG_TO_RAD(kPolarThresholdDeg));
            if (std::abs(entry.TelescopeDirection.z) < sinThresh)
                continue;

            if (!haveLoc) continue;

            double currentPitchDeg = RAD_TO_DEG(std::asin(entry.TelescopeDirection.z));
            double targetPitch     = (currentPitchDeg >= 0) ? kTargetPitchDeg : -kTargetPitchDeg;
            double deltaDeg        = currentPitchDeg - targetPitch;

            INDI::IEquatorialCoordinates celEq { entry.RightAscension, entry.Declination };
            INDI::IHorizontalCoordinates hor;
            EquatorialToHorizontal(&celEq, &location,
                                   entry.ObservationJulianDate, &hor);

            double newEl = hor.altitude - deltaDeg;
            if (newEl < 20.0) newEl = 20.0;
            if (newEl > kPolarThresholdDeg) newEl = kPolarThresholdDeg;

            INDI::IHorizontalCoordinates newHor { hor.azimuth, newEl };
            entry.TelescopeDirection = sf.TelescopeDirectionVectorFromAltitudeAzimuth(newHor);
            entry.Declination -= (hor.altitude - newEl);
        }
        else
        {
            // EQ: degeneracy at the celestial pole -- cos(Dec)->0 makes the
            // roll-index (IH) term unobservable.  Check celestial Dec.
            if (std::abs(entry.Declination) < kPolarThresholdDeg)
                continue;

            double gmst = ln_get_apparent_sidereal_time(entry.ObservationJulianDate);
            double lst  = haveLoc ? range24(gmst + location.longitude / 15.0) : gmst;

            double encoderPitchDeg = RAD_TO_DEG(std::asin(entry.TelescopeDirection.z));
            double celDec          = entry.Declination;
            double targetCelDec    = (celDec >= 0) ? kTargetPitchDeg : -kTargetPitchDeg;
            double deltaDeg        = celDec - targetCelDec;

            // Shift encoder pitch by the same delta to preserve the correction
            double newEncoderPitch = encoderPitchDeg - deltaDeg;

            // Rebuild TDV using celestial HA as encoder HA (assumes zero
            // roll-index error -- the only safe assumption when unobservable)
            double celHA = get_local_hour_angle(lst, entry.RightAscension);  // hours
            INDI::IEquatorialCoordinates haCoords { celHA, newEncoderPitch };
            entry.TelescopeDirection = sf.TelescopeDirectionVectorFromLocalHourAngleDeclination(haCoords);
            entry.Declination = targetCelDec;
        }
    }
}

} // namespace AlignmentSubsystem
} // namespace INDI
