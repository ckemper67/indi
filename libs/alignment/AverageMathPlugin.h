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

#pragma once

#include "AlignmentSubsystemForMathPlugins.h"
#include "ConvexHull.h"

namespace INDI
{
namespace AlignmentSubsystem
{

/*!
 * \struct AlignmentDatabaseEntry
 * \brief Entry in the in memory alignment database
 *
 */
struct ExtendedAlignmentDatabaseEntry : public AlignmentDatabaseEntry
{
    /// \brief Default constructor
    ExtendedAlignmentDatabaseEntry()
        : AlignmentDatabaseEntry(),
          CelestialAzimuth(0),
          CelestialAltitude(0),
          TelescopeAzimuth(0),
          TelescopeAltitude(0) {}

    /// \brief Copy constructor
    ExtendedAlignmentDatabaseEntry(const ExtendedAlignmentDatabaseEntry &Source)
        : AlignmentDatabaseEntry(Source),
          CelestialAzimuth(Source.CelestialAzimuth),
          CelestialAltitude(Source.CelestialAltitude),
          TelescopeAzimuth(Source.TelescopeAzimuth),
          TelescopeAltitude(Source.TelescopeAltitude)
    {
    }

    /// Override the assignment operator to provide a const version
    inline const ExtendedAlignmentDatabaseEntry &operator=(const ExtendedAlignmentDatabaseEntry &RHS)
    {
        ObservationJulianDate = RHS.ObservationJulianDate;
        RightAscension        = RHS.RightAscension;
        Declination           = RHS.Declination;
        TelescopeDirection    = RHS.TelescopeDirection;
        PrivateDataSize       = RHS.PrivateDataSize;
        CelestialAzimuth      = RHS.CelestialAzimuth;
        CelestialAltitude     = RHS.CelestialAltitude;
        TelescopeAzimuth      = RHS.TelescopeAzimuth;
        TelescopeAltitude     = RHS.TelescopeAltitude;
        if (0 != PrivateDataSize)
        {
            PrivateData.reset(new unsigned char[PrivateDataSize]);
            memcpy(PrivateData.get(), RHS.PrivateData.get(), PrivateDataSize);
        }

        return *this;
    }

    /**
     * @brief Celestial Azimuth of Sync Point at the time it was added to the list.
     */
    double CelestialAzimuth;
    /**
     * @brief Celestial Altitude of the Sync point at the time it was added to the list.
     */
    double CelestialAltitude;
    /**
     * @brief Telescope Azimuth of Sync Point at the time it was added to the list.
     */
    double TelescopeAzimuth;
    /**
     * @brief Telescope Altitude of the Sync point at the time it was added to the list.
     */
    double TelescopeAltitude;


};

class AverageMathPlugin : public AlignmentSubsystemForMathPlugins
{
    public:
        AverageMathPlugin();
        virtual ~AverageMathPlugin();

        virtual bool Initialise(InMemoryDatabase *pInMemoryDatabase);

        virtual bool TransformCelestialToTelescope(const double RightAscension, const double Declination,
                double JulianOffset,
                TelescopeDirectionVector &ApparentTelescopeDirectionVector);

        virtual bool TransformTelescopeToCelestial(const TelescopeDirectionVector &ApparentTelescopeDirectionVector,
                double &RightAscension, double &Declination);

    private:

        std::vector<ExtendedAlignmentDatabaseEntry> ExtendedAlignmentPoints;
};

} // namespace AlignmentSubsystem
} // namespace INDI
