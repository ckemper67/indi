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

        // Incremental normal-equation accumulation (hot path, active when nt=6)
        bool ExtractObsRow(const AlignmentDatabaseEntry &point,
                           double &oblon, double &oblat,
                           double &rdem,  double &pdem);
        bool AccumulateObsRow(double oblon, double oblat, double rdem, double pdem);
        bool SolveNormalEquations(double pmv[6]);

        double m_NE_A[36];      // AᵀA accumulator, row-major 6×6
        double m_NE_v[6];       // Aᵀb accumulator
        int    m_NE_n;          // number of sync points accumulated
        char   m_NE_mountChar;  // 'A' or 'E' at time of last build
        bool   m_NE_valid;      // true when NE state matches m_PM and m_NE_n
};

} // namespace AlignmentSubsystem
} // namespace INDI
