#pragma once

#include "MathPlugin.h"
#include "spk/spk.h"
#include "TelescopeDirectionVectorSupportFunctions.h"
#include "libastro.h"

namespace INDI
{
namespace AlignmentSubsystem
{

class SPKMathPlugin : public MathPlugin, public TelescopeDirectionVectorSupportFunctions
{
    public:
        SPKMathPlugin();
        virtual ~SPKMathPlugin();

        virtual bool Initialise(InMemoryDatabase *pInMemoryDatabase) override;

        virtual bool TransformCelestialToTelescope(const double RightAscension, const double Declination,
                double JulianOffset,
                TelescopeDirectionVector &ApparentTelescopeDirectionVector) override;

        virtual bool TransformTelescopeToCelestial(const TelescopeDirectionVector &ApparentTelescopeDirectionVector,
                double &RightAscension, double &Declination, double JulianOffset = 0) override;

    private:
        spkOBS m_Obs;
        spkPM m_PM;
        spkAST m_Ast;
        spkOPT m_Opt;
        spkPO m_PO;

        void UpdateObsConfig();
        void UpdateAstrometry(double JD);

        std::vector<double> BuildObservationData(const InMemoryDatabase::AlignmentDatabaseType &syncPoints, int &outTermCount);
        void ParsePmfitCoefficients(const double pmv[6], int terms);
        TelescopeDirectionVector RollPitchToDirectionVector(double roll, double pitch);
        void DirectionVectorToRollPitch(const TelescopeDirectionVector &v, double &roll, double &pitch);
};

} // namespace AlignmentSubsystem
} // namespace INDI
