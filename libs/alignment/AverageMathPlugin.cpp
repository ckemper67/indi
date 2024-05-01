/*******************************************************************************
 Copyright(c) 2024 Christian Kemper. All rights reserved.

 AverageMathPlugin

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.
 .
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.
 .
 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.
*******************************************************************************/

#include "AverageMathPlugin.h"

#include <libnova/julian_day.h>

namespace INDI
{
namespace AlignmentSubsystem
{
// Standard functions required for all plugins
extern "C" {

    //////////////////////////////////////////////////////////////////////////////////////
    ///
    //////////////////////////////////////////////////////////////////////////////////////
    AverageMathPlugin *Create()
    {
        return new AverageMathPlugin;
    }

    //////////////////////////////////////////////////////////////////////////////////////
    ///
    //////////////////////////////////////////////////////////////////////////////////////
    void Destroy(AverageMathPlugin *pPlugin)
    {
        delete pPlugin;
    }

    //////////////////////////////////////////////////////////////////////////////////////
    ///
    //////////////////////////////////////////////////////////////////////////////////////
    const char *GetDisplayName()
    {
        return "Average Math Plugin";
    }
}

//////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////
AverageMathPlugin::AverageMathPlugin()
{

}

//////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////
AverageMathPlugin::~AverageMathPlugin()
{

}

//////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////
bool AverageMathPlugin::Initialise(InMemoryDatabase *pInMemoryDatabase)
{
    // Call the base class to initialise to in in memory database pointer
    MathPlugin::Initialise(pInMemoryDatabase);
    const auto &SyncPoints = pInMemoryDatabase->GetAlignmentDatabase();
    // Clear all extended alignment points so we can re-create them.
    ExtendedAlignmentPoints.clear();

    IGeographicCoordinates Position;
    if (!pInMemoryDatabase->GetDatabaseReferencePosition(Position))
        return false;

    // JM: We iterate over all the sync point and compute the celestial and telescope horizontal coordinates
    // Since these are used to sort the nearest alignment points to the current target. The offsets of the
    // nearest point celestial coordinates are then applied to the current target to correct for its position.
    // No complex transformations used.
    for (auto &oneSyncPoint : SyncPoints)
    {
        ExtendedAlignmentDatabaseEntry oneEntry;
        oneEntry.RightAscension = oneSyncPoint.RightAscension;
        oneEntry.Declination = oneSyncPoint.Declination;
        oneEntry.ObservationJulianDate = oneSyncPoint.ObservationJulianDate;
        oneEntry.TelescopeDirection = oneSyncPoint.TelescopeDirection;

        INDI::IEquatorialCoordinates CelestialRADE {oneEntry.RightAscension, oneEntry.Declination};
        INDI::IHorizontalCoordinates CelestialAltAz;
        EquatorialToHorizontal(&CelestialRADE, &Position, oneEntry.ObservationJulianDate, &CelestialAltAz);

        oneEntry.CelestialAzimuth = CelestialAltAz.azimuth;
        oneEntry.CelestialAltitude = CelestialAltAz.altitude;

        INDI::IHorizontalCoordinates TelescopeAltAz;
        // Alt-Az Mounts?
        if (ApproximateMountAlignment == ZENITH)
        {
            AltitudeAzimuthFromTelescopeDirectionVector(oneEntry.TelescopeDirection, TelescopeAltAz);
        }
        // Equatorial?
        else
        {
            INDI::IEquatorialCoordinates TelescopeRADE;
            EquatorialCoordinatesFromTelescopeDirectionVector(oneEntry.TelescopeDirection, TelescopeRADE);
            EquatorialToHorizontal(&TelescopeRADE, &Position, oneEntry.ObservationJulianDate, &TelescopeAltAz);
        }

        oneEntry.TelescopeAzimuth = TelescopeAltAz.azimuth;
        oneEntry.TelescopeAltitude = TelescopeAltAz.altitude;

        ExtendedAlignmentPoints.push_back(oneEntry);
    }

    return true;
}

//////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////
bool AverageMathPlugin::TransformCelestialToTelescope(const double RightAscension, const double Declination,
        double JulianOffset, TelescopeDirectionVector &ApparentTelescopeDirectionVector)
{
    // Get Position
    IGeographicCoordinates Position;
    if (!pInMemoryDatabase || !pInMemoryDatabase->GetDatabaseReferencePosition(Position))
        return false;

    // Get Julian date from system and apply Julian Offset if any.
    double JDD = ln_get_julian_from_sys() + JulianOffset;

    // Compute CURRENT horizontal coords.
    INDI::IEquatorialCoordinates CelestialRADE {RightAscension, Declination};
    INDI::IHorizontalCoordinates CelestialAltAz;
    EquatorialToHorizontal(&CelestialRADE, &Position, JDD, &CelestialAltAz);

    // Return Telescope Direction Vector directly from Celestial coordinates if we
    // do not have any sync points.
    if (ExtendedAlignmentPoints.empty())
    {
        if (ApproximateMountAlignment == ZENITH)
        {
            // Return Alt-Az Telescope Direction Vector For Alt-Az mounts.
            ApparentTelescopeDirectionVector = TelescopeDirectionVectorFromAltitudeAzimuth(CelestialAltAz);
        }
        // Equatorial?
        else
        {
            // Return RA-DE Telescope Direction Vector for Equatorial mounts.
            ApparentTelescopeDirectionVector = TelescopeDirectionVectorFromEquatorialCoordinates(CelestialRADE);
        }

        return true;
    }

    // If we have sync points, then calculate the average offset
    double totalRAOffset = 0, totalDecOffset = 0;
    for (const auto &point : ExtendedAlignmentPoints)
    {
	INDI::IEquatorialCoordinates TelescopeRADE;
	
	// Alt-Az? Transform the telescope direction vector to telescope Alt-Az and then to telescope RA/DE
	if (ApproximateMountAlignment == ZENITH)
	{
	    INDI::IHorizontalCoordinates TelescopeAltAz;
	    AltitudeAzimuthFromTelescopeDirectionVector(point.TelescopeDirection, TelescopeAltAz);
	    HorizontalToEquatorial(&TelescopeAltAz, &Position, point.ObservationJulianDate, &TelescopeRADE);
	}
	// Equatorial? Transform directly to telescope RA/DE
	else
	{
	    EquatorialCoordinatesFromTelescopeDirectionVector(point.TelescopeDirection, TelescopeRADE);
	}

	totalRAOffset += point.RightAscension - TelescopeRADE.rightascension;
	totalDecOffset += point.Declination - TelescopeRADE.declination;
      }

    const double numPoints = ExtendedAlignmentPoints.size();
    const double avgRAOffset = totalRAOffset / numPoints;
    const double avgDecOffset = totalDecOffset / numPoints;

    // Adjust the Celestial coordinates to account for the average offset
    INDI::IEquatorialCoordinates TransformedTelescopeRADE = CelestialRADE;
    TransformedTelescopeRADE.rightascension -= avgRAOffset;
    TransformedTelescopeRADE.declination -= avgDecOffset;

    // Final step is to convert transformed telescope coordinates to a direction vector
    if (ApproximateMountAlignment == ZENITH)
    {
	INDI::IHorizontalCoordinates TransformedTelescopeAltAz;
	EquatorialToHorizontal(&TransformedTelescopeRADE, &Position, JDD, &TransformedTelescopeAltAz);
	ApparentTelescopeDirectionVector = TelescopeDirectionVectorFromAltitudeAzimuth(TransformedTelescopeAltAz);
    }
    // Equatorial?
    else
    {
	ApparentTelescopeDirectionVector = TelescopeDirectionVectorFromEquatorialCoordinates(TransformedTelescopeRADE);
    }

    return true;
}

//////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////
bool AverageMathPlugin::TransformTelescopeToCelestial(const TelescopeDirectionVector &ApparentTelescopeDirectionVector,
        double &RightAscension, double &Declination)
{
    IGeographicCoordinates Position;
    if (!pInMemoryDatabase || !pInMemoryDatabase->GetDatabaseReferencePosition(Position))
        return false;

    double JDD = ln_get_julian_from_sys();

    // Telescope Equatorial Coordinates
    INDI::IEquatorialCoordinates TelescopeRADE;

    // Do nothing if we don't have sync points.
    if (ExtendedAlignmentPoints.empty())
    {
        // Alt/Az Mount?
        if (ApproximateMountAlignment == ZENITH)
        {
            INDI::IHorizontalCoordinates TelescopeAltAz;
            AltitudeAzimuthFromTelescopeDirectionVector(ApparentTelescopeDirectionVector, TelescopeAltAz);
            HorizontalToEquatorial(&TelescopeAltAz, &Position, JDD, &TelescopeRADE);
        }
        // Equatorial?
        else
        {
            EquatorialCoordinatesFromTelescopeDirectionVector(ApparentTelescopeDirectionVector, TelescopeRADE);
        }

        RightAscension = TelescopeRADE.rightascension;
        Declination = TelescopeRADE.declination;
        return true;
    }
    else
    {
        // Need to get CURRENT Telescope horizontal coords
        INDI::IHorizontalCoordinates TelescopeAltAz;
        // Alt/Az Mount?
        if (ApproximateMountAlignment == ZENITH)
        {
            AltitudeAzimuthFromTelescopeDirectionVector(ApparentTelescopeDirectionVector, TelescopeAltAz);
            HorizontalToEquatorial(&TelescopeAltAz, &Position, JDD, &TelescopeRADE);
        }
        // Equatorial?
        else
        {
            EquatorialCoordinatesFromTelescopeDirectionVector(ApparentTelescopeDirectionVector, TelescopeRADE);
            EquatorialToHorizontal(&TelescopeRADE, &Position, JDD, &TelescopeAltAz);
        }

        // Calculate the average offset
        double totalRAOffset = 0, totalDecOffset = 0;
        for (const auto &point : ExtendedAlignmentPoints)
        {
            INDI::IEquatorialCoordinates TransformedTelescopeRADE;
            if (ApproximateMountAlignment == ZENITH)
            {
                INDI::IHorizontalCoordinates TransformedTelescopeAltAz {point.TelescopeAzimuth, point.TelescopeAltitude};
                HorizontalToEquatorial(&TransformedTelescopeAltAz, &Position, point.ObservationJulianDate, &TransformedTelescopeRADE);
            }
            else
            {
                EquatorialCoordinatesFromTelescopeDirectionVector(point.TelescopeDirection, TransformedTelescopeRADE);
            }

            totalRAOffset += point.RightAscension - TransformedTelescopeRADE.rightascension;
            totalDecOffset += point.Declination - TransformedTelescopeRADE.declination;
        }

        const double numPoints = ExtendedAlignmentPoints.size();
        const double avgRAOffset = totalRAOffset / numPoints;
        const double avgDecOffset = totalDecOffset / numPoints;

        // Adjust the Telescope coordinates to account for the average offset
        INDI::IEquatorialCoordinates TransformedCelestialRADE = TelescopeRADE;
        TransformedCelestialRADE.rightascension += avgRAOffset;
        TransformedCelestialRADE.declination += avgDecOffset;

        RightAscension = TransformedCelestialRADE.rightascension;
        Declination = TransformedCelestialRADE.declination;
    }

    return true;
}


} // namespace AlignmentSubsystem
} // namespace INDI
