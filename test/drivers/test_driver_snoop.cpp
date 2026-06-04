// testbus.h must come before INDI driver headers -- it owns the gtest/gmock
// include to prevent dsp.h's Log() macro from conflicting with gmock.
#include "testbus.h"
#include <gmock/gmock.h>

#include "indilogger.h"
#include "inditelescope.h"
#include "telescope_simulator.h"
#include "ccd_simulator.h"

#include <cstdlib>
#include <cstring>

// Required: libindidriver declares `char *me` as a global; tests must
// provide a strong definition (same pattern as test_ccd_simulator.cpp).
char _me_snoop[] = "TestSnoopDriver";
char *me = _me_snoop;

// ---------------------------------------------------------------------------
// Test driver subclasses that expose protected members for test inspection
// ---------------------------------------------------------------------------

class TestScopeSim : public ScopeSim
{
public:
    explicit TestScopeSim(const char *name = "TestScope")
    {
        initProperties();
        ISGetProperties(name);
    }

    // Publisher mock: silence incoming snoop callbacks so this device does not
    // try to process deliveries intended for consumer mocks.
    bool ISSnoopDevice(XMLEle *) override { return false; }

    using INDI::Telescope::EqNP;
    using INDI::Telescope::PierSideSP;
};

class TestCCDSim : public CCDSim
{
public:
    TestCCDSim()
    {
        initProperties();
        ISGetProperties(me);
    }

    using INDI::CCD::RA;
    using INDI::CCD::Dec;
    using INDI::CCD::pierSide;
    using INDI::CCD::ActiveDeviceTP;
    using INDI::CCD::ACTIVE_TELESCOPE;
    using INDI::CCD::EqNP;
    using INDI::CCD::J2000EqNP;

    // Wire this CCD to snoop the named telescope.  Must be called before
    // the first deliver() so EqNP.device matches the XML device attribute.
    void setSnoopedTelescope(const char *name)
    {
        ActiveDeviceTP[ACTIVE_TELESCOPE].setText(name);
        EqNP.setDeviceName(name);
        J2000EqNP.setDeviceName(name);
    }
};

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------

class SnoopTest : public ::testing::Test
{
protected:
    TestBus bus;
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

// TELESCOPE_PIER_SIDE -> CCD::pierSide
// Raw XML path: indiccd.cpp:758-770
TEST_F(SnoopTest, pier_side_east_reaches_ccd)
{
    TestScopeSim scope("TestScope");
    TestCCDSim   ccd;
    bus.snoop(ccd, scope, [&]{ ccd.setSnoopedTelescope(scope.getDeviceName()); });

    scope.PierSideSP[INDI::Telescope::PIER_EAST].s = ISS_ON;
    scope.PierSideSP[INDI::Telescope::PIER_WEST].s = ISS_OFF;
    scope.PierSideSP.setState(IPS_OK);

    auto t = bus.deliver(scope.PierSideSP);
    ASSERT_TRUE(t) << t.last_xml;
    EXPECT_EQ(ccd.pierSide, 1);  // East = 1 (indiccd.cpp:768)
}

TEST_F(SnoopTest, pier_side_west_reaches_ccd)
{
    TestScopeSim scope("TestScope");
    TestCCDSim   ccd;
    bus.snoop(ccd, scope, [&]{ ccd.setSnoopedTelescope(scope.getDeviceName()); });

    scope.PierSideSP[INDI::Telescope::PIER_WEST].s = ISS_ON;
    scope.PierSideSP[INDI::Telescope::PIER_EAST].s = ISS_OFF;
    scope.PierSideSP.setState(IPS_OK);

    auto t = bus.deliver(scope.PierSideSP);
    ASSERT_TRUE(t) << t.last_xml;
    EXPECT_EQ(ccd.pierSide, 0);  // West = 0 (indiccd.cpp:770)
}

// EQUATORIAL_EOD_COORD -> CCD::RA / CCD::Dec
// Modern snoop path: EqNP.snoop(root) at indiccd.cpp:733
TEST_F(SnoopTest, equatorial_coord_reaches_ccd)
{
    TestScopeSim scope("TestScope");
    TestCCDSim   ccd;
    bus.snoop(ccd, scope, [&]{ ccd.setSnoopedTelescope(scope.getDeviceName()); });

    scope.EqNP[AXIS_RA].setValue(6.75);
    scope.EqNP[AXIS_DE].setValue(-45.0);
    scope.EqNP.setState(IPS_OK);

    auto t = bus.deliver(scope.EqNP);
    ASSERT_TRUE(t) << t.last_xml;
    EXPECT_DOUBLE_EQ(ccd.RA,  6.75);
    EXPECT_DOUBLE_EQ(ccd.Dec, -45.0);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char **argv)
{
    // Isolate from user config: redirect HOME to a throwaway temp dir so
    // initProperties() finds no saved config and keeps compiled-in defaults.
    // HOME must be set BEFORE ::ISGetProperties(nullptr) below.
    const char *tmpbase = getenv("TMPDIR");
    char tmpdir[512];
    snprintf(tmpdir, sizeof(tmpdir), "%s/indi_snoop_XXXXXX",
             (tmpbase && tmpbase[0]) ? tmpbase : "/tmp");
    char *td = mkdtemp(tmpdir);
    EXPECT_NE(td, nullptr);
    if (td)
        setenv("HOME", td, 1);

    // Unset vars that can override config and resource paths regardless of HOME.
    unsetenv("INDICONFIG");   // would redirect config file path entirely
    unsetenv("INDIPREFIX");   // would redirect data/resource directories

    INDI::Logger::getInstance().configure("", INDI::Logger::file_off,
                                          INDI::Logger::DBG_ERROR,
                                          INDI::Logger::DBG_ERROR);
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    int result = RUN_ALL_TESTS();

    if (td)
    {
        std::string cmd = std::string("rm -rf ") + td;
        system(cmd.c_str());
    }

    return result;
}
