/*
 * scopesim_helper.h
 *
 * Copyright 2020 Chris Rowland <chris.rowland@cherryfield.me.uk>
 *
 * helper classes for the telescope simulator
 *
 * The Angle structure defines an angle class that manages the wrap round
 * 0 to 360 and handles arithmetic and logic across this boundary.
 *
 * The Axis class manages a simulated mount axis and handles moving, tracking, and guiding.
 *
 * The Alignment class handles the alignment, converting between the observed and instrument
 * places, and allowing for the axis positions needed for a GEM mount.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 *
 */

#pragma once

#include <stdint.h>
#include <sys/time.h>
#include <cmath>

#include <indicom.h>

static char device_str[64] = "Telescope Simulator";

///
/// \brief The Angle class
/// This class implements an angle type.
/// This holds an angle that is always in the range -648000 to +648000 arcseconds
/// Relational and arithmetic operators work over the -648000 - +648000 discontinuity
///
class Angle
{
private:
    double angle; // position in arcseconds, range -648000 to 0 to 648000

    ///
    /// \brief range
    /// \param arcsecs
    /// \return returns an angle in arcseconds folded to the range -647999... to 0 to 648000
    ///
    static double range(double arcsecs)
    {
        while (arcsecs > 648000.0) arcsecs -= 1296000.0;
        while (arcsecs <= -648000.0) arcsecs += 1296000.0;
        return arcsecs;
    }

    static double degToArcsec(double deg)
    {
        return range(deg * 3600.0);
    }

    static double hrsToArcsec(double hrs)
    {
        return range(hrs * 15.0 * 3600.0);
    }

public:
  enum ANGLE_UNITS {DEGREES, HOURS, RADIANS, ARCSECS};

    Angle()
    {
        angle = 0;
    }

    Angle(double value, ANGLE_UNITS type)
    {
        switch (type)
        {
        case ARCSECS:
            angle = value;
            break;
        case DEGREES:
            angle = degToArcsec(value);
            break;
        case HOURS:
            angle = hrsToArcsec(value);
            break;
        case RADIANS:
            angle = degToArcsec(value * 180.0 / M_PI);
            break;
        }
    }

    Angle(double degrees)
    {
        angle = degToArcsec(degrees);
    }

    //virtual ~Angle() = default;

    ///
    /// \brief Degrees
    /// \return angle in degrees, range -180 to 0 to +180
    ///
    double Degrees()
    {
        return angle / 3600.0;
    }

    ///
    /// \brief Degrees360
    /// \return angle in degrees, range 0 to 360
    ///
    double Degrees360()
    {
        double deg = angle / 3600.0;
        return (deg >= 0) ? deg : 360.0 + deg;
    }

    ///
    /// \brief Hours
    /// \return angle in hours, range 0 to 24
    ///
    double Hours()
    {
        double h = angle / (15.0 * 3600.0);
        if (h < 0.0)
            h = 24 + h;
        return h;
    }

    ///
    /// \brief HoursHa
    /// \return angle in hours, range -12 to +12
    ///
    double HoursHa()
    {
        return angle / (15.0 * 3600.0);
    }

    ///
    /// \brief radians
    /// \return angle in radians, range -Pi to 0 to +PI
    ///
    double radians()
    {
        return angle * M_PI / (180.0 * 3600.0);
    }

    ///
    /// \brief arcsecs
    /// \return angle in arcseconds, range range -648000 to 0 to 648000
    ///
    double arcsecs()
    {
        return angle;
    }

    ///
    /// \brief setDegrees
    /// set the angle in degrees
    /// \param deg angle in degrees
    ///
    void setArcSeconds(double arcsecs)
    {
        angle = arcsecs;
    }

    ///
    /// \brief setDegrees
    /// set the angle in degrees
    /// \param deg angle in degrees
    ///
    void setDegrees(double deg)
    {
        angle = degToArcsec(deg);
    }

    ///
    /// \brief setHours set the angle
    /// \param hrs angle in hours
    ///
    void setHours(double hrs)
    {
        angle = hrsToArcsec(hrs);
    }

    Angle add(Angle a)
    {
        double total = a.angle + this->angle;
        return Angle(total, ARCSECS);
    }

    Angle subtract(Angle a)
    {
      return Angle((this->angle - a.angle),ARCSECS);
    }

    double difference(Angle a)
    {
        return range(this->angle - a.angle) / 3600.0;
    }

    Angle operator - ()
    {
      return Angle(-this->angle, ARCSECS);
    }

    Angle &operator += (const Angle &a)
    {
        angle = range(angle + a.angle);
        return *this;
    }

    Angle &operator += (const double d)
    {
        angle = range(angle + degToArcsec(d));
        return *this;
    }

    Angle &operator-= (const Angle &a)
    {
        angle = range(angle - a.angle);
        return *this;
    }

    Angle &operator-= (const double d)
    {
        angle = range(angle - degToArcsec(d));
        return *this;
    }

    Angle operator+ (const Angle &a)
    {
      return Angle((this->angle + a.angle), ARCSECS);
    }

    Angle operator+ (const double &d)
    {
      return Angle((this->angle + degToArcsec(d)), ARCSECS);
    }

    Angle operator- (const Angle &rhs)
    {
      return Angle((this->angle - rhs.angle), ARCSECS);
    }

    Angle operator- (const double &rhs)
    {
      return Angle((this->angle - degToArcsec(rhs)), ARCSECS);
    }

    ///
    /// \brief operator *
    /// multiplies the angle by a double,
    /// used to manage tracking and slewing
    /// \param duration as a double
    /// \return Angle
    ///
    Angle operator * (const double duration)
    {
      return Angle(this->angle * duration, ARCSECS);
    }

    bool operator== (const Angle &a)
    {
        return this->angle == a.angle;
    }

    bool operator!= (const Angle &a)
    {
        return this->angle != a.angle;
    }

    bool operator > (const Angle &a)
    {
        return difference(a) > 0;
    }

    bool operator < (const Angle &a)
    {
        return difference(a) < 0;
    }
    bool operator >= (const Angle &a)
    {
        return difference(a) >= 0;
    }
    bool operator <= (const Angle &a)
    {
        return difference(a) <= 0;
    }
};


/////////////////////////////////////////////////////////////////////////////////////////////////

///
/// \brief The Axis class
/// Implements a generic Axis which can be used for equatorial or AltAz mounts for both axes.
///
/// For an equatorial mount use the TrackRate to set the standard tracking rates. for the primary axis only.
/// For an AltAz mount the TrackingRateDegSec rate must be set for both axes.
///
class Axis
{
    public:
        ///
        /// \brief The AXIS_TRACK_RATE enum defines the common track rates
        ///
        enum AXIS_TRACK_RATE { OFF, SIDEREAL, SOLAR, LUNAR };

        Axis(const char * name)
        {
            axisName = name;
        }

        const char * axisName;

        // sets position and target so does not cause a slew.
        void setDegrees(double degrees);
        void setHours(double hours);

        Angle position;         // current axis position

        void StartSlew(Angle angle);

        void Abort()
        {
            target = position;
            mcRate = 0;
            guideDuration = 0;
        }

        bool isSlewing;

        bool isTracking()
        {
            return tracking;
        }

        void Tracking(bool enabled);

        ///
        /// \brief TrackRate set the track rate to one of the standard rates
        /// \param rate as AXIS_TRACK_RATE
        ///
        /// use this for the primary axis on an equatorial mount
        ///
        void TrackRate(AXIS_TRACK_RATE rate);

        ///
        /// \brief TrackRate returns the AXIS_TRACK_RATE property
        /// \return
        ///
        AXIS_TRACK_RATE TrackRate();

        ///
        /// \brief TrackingRateDegSec
        ///
        Angle TrackingRateDegSec;

        ///
        /// \brief StartGuide   start guiding
        /// \param rate as a fraction of the sidereal rate, signed
        /// \param durationMs
        ///
        void StartGuide(double rate, uint32_t durationMs);

        bool IsGuiding()
        {
            return guideDuration > 0;
        }

        int mcRate;             // int -4 to 4 sets move rate, zero is stopped

        void update();         // called about once a second to update the position and mode

        // needed for debug MACROS
        const char *getDeviceName()
        {
            return device_str;
        }

    private:
        Angle target;           // target axis position

        struct timeval lastTime
        {
            0, 0
        };

        bool tracking;      // this allows the tracking state and rate to be set independently

        AXIS_TRACK_RATE trackingRate { AXIS_TRACK_RATE::OFF };

        Angle rotateCentre { 90.0 };

        double guideDuration;
        Angle guideRateDegSec;

        // rates are angles in degrees per second derived from the values in indicom.h
        // which are in arcsec per second.

        const Angle solarRate { TRACKRATE_SOLAR / 3600.0 };
        const Angle siderealRate { TRACKRATE_SIDEREAL / 3600.0 };
        const Angle lunarRate { TRACKRATE_LUNAR / 3600.0 };

        Angle mcRates[5]
        {
            0,
            siderealRate,   // guide rate
            0.5,            // fine rate
            2.5,            // center rate
            6.0,            // goto rate
        };
};

///
/// \brief The Alignment class
/// This converts between the mount axis angles and the sky hour angle and declination angles.
/// Initially for equatorial fork and GEM mounts.
/// To start with there is a unity mount model.
/// The axis zeros correspond to the declination and hour angle zeroes and the directions match in the Northern henisphere
/// For the GEM the normal pointing state is defined as positive hour angles, ie. with the mount on the East, looking West
/// Both axis directions are mirrored in the South
///
/// This uses a simple mount model based on Patrick Wallace's paper.
/// this is at http://www.tpointsw.uk/pointing.htm
///
/// Terminology is as defined in figure 1:
///
///  Apparent Ra and Dec - what is (incorrectly) called JNow. positions are apparentRa and apparentDec
///     apply local sidereal time
///  Apparent Ha and Dec, positions are apparentHa and apparentDec
///     ignore diurnal effects
///     ignore refraction (for now)
///  Observed Place  These are the mount coordinates for a perfect mount, positions are observedHa and observedDec
///     apply telescope pointing corrections
///  Instrument Place these are the mount coordinates for the mount with corrections, values are instrumentHa and instrumentDec
///     for a  GEM convert to axis coordinates ( this isn't in the paper).
///  Mount Place these give primary (ha) and secondary (dec) positions
///
/// At present AltAz mounts are not implemented
///
class Alignment
{
    public:
        Alignment() {}

        enum MOUNT_TYPE { ALTAZ, EQ_FORK, EQ_GEM };

        Angle latitude = 0;
        Angle longitude = 0;

        MOUNT_TYPE mountType = EQ_FORK;

        ///
        /// \brief mountToApparentRaDec: convert mount position to apparent Ra, Dec
        /// \param primary
        /// \param secondary
        /// \param apparentRa
        /// \param apparentDec
        ///
        void mountToApparentRaDec(Angle primary, Angle secondary, Angle * apparentRa, Angle* apparentDec);


        void apparentRaDecToMount(Angle apparentRa, Angle apparentDec, Angle *primary, Angle *secondary);


        ///
        /// \brief setCorrections set the values of the six corrections
        /// \param ih
        /// \param id
        /// \param ch
        /// \param np
        /// \param ma
        /// \param me
        ///
        void setCorrections(double ih, double id, double ch, double np, double ma, double me);

        void setFlipHourAngle(double deg)
        {
            flipHourAngle = deg;
        }

        // needed for debug MACROS
        const char *getDeviceName()
        {
            return device_str;
        }

        double ih()
        {
            return IH;
        }
        double id()
        {
            return ID;
        }
        double np()
        {
            return NP;
        }
        double ch()
        {
            return CH;
        }
        double ma()
        {
            return MA;
        }
        double me()
        {
            return ME;
        }

        void instrumentToObserved(Angle instrumentHa, Angle instrumentDec, Angle *observedHa, Angle *observedDec);

        void observedToInstrument(Angle observedHa, Angle observedDec, Angle * instrumentHa, Angle * instrumentDec);


        ///
        /// \brief mountToApparentHaDec: convert mount position to apparent Ha, Dec
        /// \param primary
        /// \param secondary
        /// \param apparentHa
        /// \param apparentDec
        ///
        void mountToApparentHaDec(Angle primary, Angle secondary, Angle *apparentHa, Angle *apparentDec);

        ///
        /// \brief apparentHaDecToMount
        /// \param ha
        /// \param apparentDec
        /// \param primary
        /// \param secondary
        ///
        void apparentHaDecToMount(Angle apparentHa, Angle apparentDec, Angle *primary, Angle *secondary);

        Angle lst();            // returns the current LST as an angle
    private:




        Angle flipHourAngle = 0;

        ///
        /// \brief correction: determines the correction to the instrument position to get the observed
        /// Based on Patrick Wallace's paper, see Table 1.
        ///
        /// correction parameters are:
        /// IH: The hour angle axis index error
        /// ID: The dec axis index error
        /// CH: the telescope collimation error, popularly known as cone
        /// NP: the amount that the mount dec axis is not perpendicular to the hour angle axis
        /// MA: the polar axis azimuth error
        /// ME: the polar axis elevation error
        ///
        /// \param instrumentHa
        /// \param instrumentDec
        /// \param correctionHa
        /// \param correctionDec
        ///
        void correction(Angle instrumentHa, Angle instrumentDec, Angle *correctionHa, Angle *correctionDec);

        // mount model, these angles are in arcseconds
        // the angles are small so use double to avoid
        // loads of conversions
        ///
        /// \brief IH hour angle/azimuth offset
        ///
        double IH = 0;
        ///
        /// \brief ID declination/elevation offset
        ///
        double ID = 0;
        ///
        /// \brief CH OTA - dec axis non orthogonality
        ///
        double CH = 0;
        ///
        /// \brief NP hour angle - dec axis non orthogonality
        ///
        double NP = 0;
        ///
        /// \brief MA polar align error in Azimuth
        ///
        double MA = 0;
        ///
        /// \brief ME polar align error in Elevation
        ///
        double ME = 0;


        // corrections done using direction cosines and rotations after Taki
        //apparentHaDecToMount(Vector HaDec, Vector * mount);
};

///
/// \brief The Vector class
///  This implements the Directional Cosine used by Taki in his Matrix method.
///  using Vector because it's a 3D vector, hope this doesn't clash too badly
///  with std::vector
///
///  The parameter names are selected to match what Taki uses.
///
class Vector
{
    public:
        ///
        /// \brief Vector
        /// creates an empty vector
        ///
        Vector()
        {
            L = M = N = 0;
        }

        ///
        /// \brief Vector creates vector from the three vector values
        /// \param l
        /// \param m
        /// \param n
        ///
        Vector(double l, double m, double n);

        ///
        /// \brief Vector creates a vector from two angles (Ra, Dec), (Ha, Dec), (Azimuth, Altitude)
        /// \param primary
        /// \param secondary
        ///
        Vector(Angle primary, Angle secondary);

        double lengthSquared()
        {
            return L * L + M * M + N * N;
        }

        double length()
        {
            return std::sqrt(lengthSquared());
        }

        void normalise();

        ///
        /// \brief primary returns the primary angle (Ra, Ha, Azimuth) from this vector
        /// \return
        ///
        Angle primary();

        ///
        /// \brief secondary returns the secondary angle (dec, altitude) from this vector
        /// \return
        ///
        Angle secondary();

        ///
        /// \brief rotateX rotates this vector through angle about the X axis
        /// \param angle
        ///
        Vector rotateX(Angle angle);

        ///
        /// \brief rotateY rotates this vector through angle about the Y axis
        /// \param angle
        ///
        Vector rotateY(Angle angle);

        ///
        /// \brief rotateZ rotates this vector through angle about the Z axis
        /// \param rotX
        ///
        Vector rotateZ(Angle angle);

        double l()
        {
            return L;
        }
        double m()
        {
            return M;
        }
        double n()
        {
            return N;
        }

    protected:
        double L;   // in the Ha 0 direction, pointing at Ha 0, Dec 0, X direction
        double M;   // in the Ha 6 direction, pointing at Ha 6h, Dec 0, Y direction
        double N;   // toward the pole, Dec 0, Z direction
};
