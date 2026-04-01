#include "SPKMathPlugin.h"
#include "DriverCommon.h"
#include <vector>
#include <cmath>
#include <cstring>
#include "indicom.h"
#include "libastro.h"
#include "spk/sofa.h"
#include <libnova/julian_day.h>
#include <libnova/sidereal_time.h>

namespace INDI
{
namespace AlignmentSubsystem
{

#ifndef NO_PLUGIN_HOOKS
// Standard functions required for all plugins
extern "C" {
    SPKMathPlugin *Create()
    {
        return new SPKMathPlugin;
    }

    void Destroy(SPKMathPlugin *pPlugin)
    {
        delete pPlugin;
    }

    const char *GetDisplayName()
    {
        return "SPK Math Plugin";
    }
}
#endif

SPKMathPlugin::SPKMathPlugin()
{
    // Initialize structures with defaults
    memset(&m_Obs, 0, sizeof(m_Obs));
    memset(&m_PM, 0, sizeof(m_PM));
    memset(&m_Ast, 0, sizeof(m_Ast));
    memset(&m_Opt, 0, sizeof(m_Opt));
    memset(&m_PO, 0, sizeof(m_PO));
    
    m_Opt.fl = 1000.0; // Default fl
    m_Opt.wl = 0.55;   // Default wavelength
}

SPKMathPlugin::~SPKMathPlugin()
{
}

bool SPKMathPlugin::Initialise(InMemoryDatabase *pInMemoryDatabase)
{
    if (!pInMemoryDatabase) return false;
    this->pInMemoryDatabase = pInMemoryDatabase;

    UpdateObsConfig();

    // Collect sync points from database
    InMemoryDatabase::AlignmentDatabaseType &syncPoints = pInMemoryDatabase->GetAlignmentDatabase();

    if (syncPoints.size() < 1)
    {
        return false;
    }

    // Build observation data and determine the number of terms
    int nt = 0;
    std::vector<double> obsData = BuildObservationData(syncPoints, nt);

    double pmv[6], pms[6], skysig;
    char mountChar = (m_Obs.mount == ALTAZ) ? 'A' : 'E';
    int js = Pmfit(m_Obs.slat, mountChar, syncPoints.size(), obsData.data(), nt, pmv, pms, &skysig);

    if (js == 0)
    {
        ParsePmfitCoefficients(pmv, nt);
        return true;
    }

    return false;
}

bool SPKMathPlugin::TransformCelestialToTelescope(const double RightAscension, const double Declination,
        double JulianOffset,
        TelescopeDirectionVector &ApparentTelescopeDirectionVector)
{
    UpdateAstrometry(ln_get_julian_from_sys() + JulianOffset);

    spkTAR tar;
    tar.sys = APPT;
    tar.a = HOURS_TO_RAD(RightAscension);
    tar.b = DEG_TO_RAD(Declination);

    double tara, tare, tarr, tarp, soln[5];

    // Two-pass: first AXES call with ax3={0,0,0} gives an initial encoder
    // demand; second pass re-evaluates the VD correction at the correct
    // elevation.  One refinement suffices; residual is O(pvd^2).
    spkAX3 ax3 = {0, 0, 0};
    int status = spkVtel(AXES, &m_Obs, &m_Opt, &m_PM, &m_Ast, &ax3, &tar, &m_PO,
                         &tara, &tare, &tarr, &tarp, soln);

    if (status >= 0)
    {
        ax3.a = soln[0];
        ax3.b = soln[1];
        status = spkVtel(AXES, &m_Obs, &m_Opt, &m_PM, &m_Ast, &ax3, &tar, &m_PO,
                         &tara, &tare, &tarr, &tarp, soln);
    }

    if (status >= 0)
    {
        double roll = soln[0];
        double pitch = soln[1];
        ApparentTelescopeDirectionVector = RollPitchToDirectionVector(roll, pitch);
        return true;
    }
    return false;
}

bool SPKMathPlugin::TransformTelescopeToCelestial(const TelescopeDirectionVector &ApparentTelescopeDirectionVector,
        double &RightAscension, double &Declination, double JulianOffset)
{
    UpdateAstrometry(ln_get_julian_from_sys() + JulianOffset);

    spkAX3 ax3;
    double roll, pitch;
    DirectionVectorToRollPitch(ApparentTelescopeDirectionVector, roll, pitch);
    
    ax3.a = roll;
    ax3.b = pitch;
    ax3.r = 0.0;

    spkTAR tar;
    tar.sys = APPT;

    double tara, tare, tarr, tarp, soln[5];
    int status = spkVtel(TARG, &m_Obs, &m_Opt, &m_PM, &m_Ast, &ax3, &tar, &m_PO,
                         &tara, &tare, &tarr, &tarp, soln);

    if (status >= 0)
    {
        // soln[0] is RA in radians
        RightAscension = RAD_TO_HOURS(iauAnp(soln[0]));
        Declination = RAD_TO_DEG(soln[1]);
        return true;
    }

    return false;
}

void SPKMathPlugin::UpdateObsConfig()
{
    INDI::IGeographicCoordinates pos;
    if (pInMemoryDatabase->GetDatabaseReferencePosition(pos))
    {
        m_Obs.slat = DEG_TO_RAD(pos.latitude);
        m_Obs.slon = DEG_TO_RAD(pos.longitude);
        m_Obs.sh = pos.elevation;
    }
    
    // Set mount type based on driver's approximate alignment
    m_Obs.mount = (ApproximateMountAlignment == ZENITH) ? ALTAZ : EQUAT;
}

void SPKMathPlugin::UpdateAstrometry(double JD)
{
    // Use libnova for JD to calendar conversion
    ln_date date;
    ln_get_date(JD, &date);

    spkUTC utc;
    utc.iy = date.years;
    utc.mo = date.months;
    utc.id = date.days;
    utc.ih = date.hours;
    utc.mi = date.minutes;
    utc.sec = date.seconds;

    spkEOP eop = {0, 0, 0};
    // Set pressure to 0 to disable refraction, matching simulator
    spkAIR air = {0.0, 10.0, 0.5};
    
    // Use official spkAstr
    spkAstr(1, utc, 0.0, &eop, &m_Obs, &air, &m_Opt, &m_Ast);

    // Synchronize SOFA internal "clock" (ERA) with libnova LST
    // This ensures consistency with the simulator's view of HA/Az.
    double gmst_hrs = ln_get_apparent_sidereal_time(JD);
    double lst_rad  = HOURS_TO_RAD(range24(gmst_hrs + RAD_TO_HOURS(m_Obs.slon)));
    // eral = ERA + Longitude = LST + EO
    m_Ast.astrom.eral = iauAnp(lst_rad + m_Ast.eo);
}


std::vector<double> SPKMathPlugin::BuildObservationData(const InMemoryDatabase::AlignmentDatabaseType &syncPoints, int &outTermCount)
{
    // Pmfit term order: IH, ID, ME, MA, CH, TF  (equatorial)
    //                   IA, IE, AN, AW, CA, TF  (altazimuth)
    // Polar/axis-tilt terms come before collimation so that a 4-term fit on
    // 3 points (6 measurements, 4 unknowns) is well-conditioned.
    outTermCount = 6;
    if      (syncPoints.size() < 3) outTermCount = 2; // 1-2 pts: IH, ID
    else if (syncPoints.size() < 5) outTermCount = 4; // 3-4 pts: IH, ID, ME, MA
    else if (syncPoints.size() < 6) outTermCount = 5; // 5 pts:   IH, ID, ME, MA, CH
    // 6+ pts: full model

    std::vector<double> obsData;
    for (const auto &point : syncPoints)
    {
        UpdateAstrometry(point.ObservationJulianDate);
        
        double gmst_hrs = ln_get_apparent_sidereal_time(point.ObservationJulianDate);
        double lst_hrs  = range24(gmst_hrs + RAD_TO_HOURS(m_Obs.slon));

        double obslon_ha = HOURS_TO_RAD(get_local_hour_angle(lst_hrs, point.RightAscension));
        double obslat    = DEG_TO_RAD(point.Declination);

        double roll, pitch;
        DirectionVectorToRollPitch(point.TelescopeDirection, roll, pitch);

        if (m_Obs.mount == ALTAZ)
        {
            double az_cel, el_cel;
            iauHd2ae(obslon_ha, obslat, m_Obs.slat, &az_cel, &el_cel);
            obsData.push_back(az_cel);
            obsData.push_back(el_cel);
            obsData.push_back(iauAnp(-roll)); // roll = -Az
        }
        else
        {
            obsData.push_back(obslon_ha);
            obsData.push_back(obslat);
            obsData.push_back(-roll); // roll = -HA
        }
        obsData.push_back(pitch);
    }
    return obsData;
}

void SPKMathPlugin::ParsePmfitCoefficients(const double pmv[6], int terms)
{
    memset(&m_PM, 0, sizeof(m_PM));

    if (m_Obs.mount == ALTAZ)
    {
        // Pmfit order: IA, IE, AN, AW, CA, TF
        // Wallace (2002) Note 4 (altaz): IA=bf[0], IB=bf[1], AN=bf[2], AW=bf[3], CA=bf[4], VD=bf[5]
        if (terms >= 1) m_PM.pia = pmv[0];
        if (terms >= 2) m_PM.pib = pmv[1];
        if (terms >= 3) m_PM.pan = pmv[2];
        if (terms >= 4) m_PM.paw = pmv[3];
        if (terms >= 5) m_PM.pca = pmv[4];
        if (terms >= 6) m_PM.pvd = pmv[5];
    }
    else // EQUATORIAL
    {
        // Pmfit order: IH, ID, ME, MA, CH, TF
        // Wallace (2002) Note 4 (equat): IA=-bf[0], IB=bf[1], AW=-bf[2], AN=bf[3], CA=-bf[4], VD=bf[5]
        if (terms >= 1) m_PM.pia = -pmv[0];
        if (terms >= 2) m_PM.pib = pmv[1];
        if (terms >= 3) m_PM.paw = -pmv[2];
        if (terms >= 4) m_PM.pan = pmv[3];
        if (terms >= 5) m_PM.pca = -pmv[4];
        if (terms >= 6) m_PM.pvd = pmv[5];
    }
}

TelescopeDirectionVector SPKMathPlugin::RollPitchToDirectionVector(double roll, double pitch)
{
    if (m_Obs.mount == ALTAZ)
    {
        INDI::IHorizontalCoordinates hor;
        hor.azimuth = RAD_TO_DEG(iauAnp(-roll));
        hor.altitude = RAD_TO_DEG(pitch);
        return TelescopeDirectionVectorFromAltitudeAzimuth(hor);
    }
    else
    {
        INDI::IEquatorialCoordinates eq;
        eq.rightascension = RAD_TO_HOURS(iauAnp(-roll));
        eq.declination = RAD_TO_DEG(pitch);
        return TelescopeDirectionVectorFromLocalHourAngleDeclination(eq);
    }
}

void SPKMathPlugin::DirectionVectorToRollPitch(const TelescopeDirectionVector &v, double &roll, double &pitch)
{
    if (m_Obs.mount == ALTAZ)
    {
        INDI::IHorizontalCoordinates hor;
        AltitudeAzimuthFromTelescopeDirectionVector(v, hor);
        roll  = iauAnp(-DEG_TO_RAD(hor.azimuth));
        pitch = DEG_TO_RAD(hor.altitude);
    }
    else
    {
        INDI::IEquatorialCoordinates eq;
        LocalHourAngleDeclinationFromTelescopeDirectionVector(v, eq);
        roll  = iauAnpm(-HOURS_TO_RAD(eq.rightascension));
        pitch = DEG_TO_RAD(eq.declination);
    }
}

} // namespace AlignmentSubsystem
} // namespace INDI
