#include <gtest/gtest.h>
#include "eph/eph.h"
#include <cmath>
#include <cstdio>

/**
 * @brief Unit tests for the vendored EPH library.
 * 
 * Verifies that the library can load its binary context files and compute
 * planetary positions without crashing.
 */
TEST(EphLibrary, LoadEarth)
{
    auto cemb = std::make_unique<ephPLANctx>();
    const char* data_path = INDI_DATA_DIR "/eph/";
    
    int status = ephPlanc(3, const_cast<char*>(data_path), cemb.get());
    ASSERT_EQ(status, 0) << "Failed to load Earth context from " << data_path;
    EXPECT_EQ(cemb->ibody, 3);
}

TEST(EphLibrary, LoadMars)
{
    auto cplan = std::make_unique<ephPLANctx>();
    const char* data_path = INDI_DATA_DIR "/eph/";
    
    int status = ephPlanc(4, const_cast<char*>(data_path), cplan.get());
    ASSERT_EQ(status, 0) << "Failed to load Mars context from " << data_path;
    EXPECT_EQ(cplan->ibody, 4);
}

TEST(EphLibrary, LoadMoon)
{
    auto cmoon = std::make_unique<ephMOONctx>();
    const char* data_path = INDI_DATA_DIR "/eph/";
    
    int status = ephMoonc(const_cast<char*>(data_path), 2, cmoon.get());
    ASSERT_EQ(status, 0) << "Failed to load Moon context from " << data_path;
}

TEST(EphLibrary, ComputeMars)
{
    auto cemb = std::make_unique<ephPLANctx>();
    auto cplan = std::make_unique<ephPLANctx>();
    auto cmoon = std::make_unique<ephMOONctx>();
    const char* data_path = INDI_DATA_DIR "/eph/";
    
    // Setup contexts
    ASSERT_EQ(ephPlanc(3, const_cast<char*>(data_path), cemb.get()), 0);
    ASSERT_EQ(ephPlanc(4, const_cast<char*>(data_path), cplan.get()), 0);
    ASSERT_EQ(ephMoonc(const_cast<char*>(data_path), 2, cmoon.get()), 0);

    // Date: 2020-06-19 08:00 UTC
    // MJD = JD - 2400000.5
    // 2020-06-19 is JD 2459019.833333
    double ut1 = 2459019.833333 - 2400000.5;
    double tdb = ut1;

    double rast, dast, rapp, dapp, eo, diam;
    int status = ephRdplan(cmoon.get(), cemb.get(), cplan.get(), ut1, tdb, 4, 0, 0, 0, 
                           &rast, &dast, &rapp, &dapp, &eo, &diam);

    EXPECT_EQ(status, 0);
    
    // Approximate Mars position for this date from the 'planets' output:
    // RA 23h 44m, Dec -04d 45m
    double ra_hours = rapp * (12.0 / M_PI);
    double dec_deg = dapp * (180.0 / M_PI);
    
    GTEST_LOG_(INFO) << "Computed Mars: RA=" << ra_hours << " Dec=" << dec_deg;
    
    EXPECT_NEAR(ra_hours, 23.74, 0.1);
    EXPECT_NEAR(dec_deg, -4.75, 0.1);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
