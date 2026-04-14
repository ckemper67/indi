#include "indicom.h"
#include "indilogger.h"
#include "libastro.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "guide_simulator.h"

#include <cmath>
#include <libnova/julian_day.h>

char _me[] = "MockGuideSimDriver";
char *me = _me;

// Capella (alpha Aurigae): J2000 RA 5h 16m 41.4s, Dec +45 59' 56" -- mag 0.08.
// Matches the brightest entry in bright_stars_catalog.h at RA=79.1721 deg.
static constexpr double kCapellaJ2000RAHours = 79.1721 / 15.0;  // ~5.2781 h
static constexpr double kCapellaJ2000DecDeg  = 45.9990;

// DrawCcdFrame takes telescope RA/Dec as JNow and converts to J2000 internally.
// Invert that conversion so the rendered position lines up with the J2000
// catalog entry for Capella.
static void capellaJNow(double &raHours, double &decDeg)
{
    INDI::IEquatorialCoordinates j2000 { kCapellaJ2000RAHours, kCapellaJ2000DecDeg };
    INDI::IEquatorialCoordinates observed { 0, 0 };
    INDI::J2000toObserved(&j2000, ln_get_julian_from_sys(), &observed);
    raHours = observed.rightascension;
    decDeg  = observed.declination;
}

class MockGuideSimDriver: public GuideSim
{
    public:
        MockGuideSimDriver(): GuideSim()
        {
            initProperties();
            ISGetProperties(me);
        }

        // Populate sensor + scope-info, run SetupParms so the buffer is
        // allocated and the bright-star supplement has valid plate constants.
        void prepareForRendering()
        {
            // Reasonable guide-scope geometry. Defaults from initProperties already
            // give pixel=2.4um and resolution 1280x1024; we keep those and add
            // explicit aperture+focal length so DrawCcdFrame does not need to
            // snoop a telescope to compute the diffraction floor.
            ScopeInfoNP[FOCAL_LENGTH].setValue(400.0);
            ScopeInfoNP[APERTURE].setValue(60.0);

            // SIM_SKYGLOW is a magnitude (higher = fainter), and SIM_NOISE drives
            // the per-pixel read noise loop. Drive both down so a star contribution
            // is unambiguous against the background.
            auto p = getNumber("SIMULATOR_SETTINGS");
            p.findWidgetByName("SIM_NOISE")->setValue(0.0);
            p.findWidgetByName("SIM_SKYGLOW")->setValue(30.0);
            p.findWidgetByName("SIM_BIAS")->setValue(0.0);

            SetupParms();
            memset(PrimaryCCD.getFrameBuffer(), 0, PrimaryCCD.getFrameBufferSize());
        }

        // Peak ADU across the frame. A real star renders thousands of ADU at its
        // core; sky-glow dither contributes <2 ADU per pixel at SIM_SKYGLOW=30.
        int peakPixel()
        {
            uint16_t *fb = reinterpret_cast<uint16_t *>(PrimaryCCD.getFrameBuffer());
            int const n  = PrimaryCCD.getXRes() * PrimaryCCD.getYRes();
            int peak = 0;
            for (int i = 0; i < n; i++)
                if (fb[i] > peak) peak = fb[i];
            return peak;
        }

        // Direct exercise of DrawImageStar in isolation -- mirrors
        // test_ccd_simulator::testDrawStar. Confirms the shared rendering path
        // works when called with valid scale/seeing values.
        void testDrawStarBasic()
        {
            int const xres   = 65;
            int const yres   = 65;
            ScopeInfoNP[FOCAL_LENGTH].setValue(400.0);
            ScopeInfoNP[APERTURE].setValue(60.0);
            auto p = getNumber("SIMULATOR_SETTINGS");
            p.findWidgetByName("SIM_XRES")->setValue(xres);
            p.findWidgetByName("SIM_YRES")->setValue(yres);
            p.findWidgetByName("SIM_SATURATION")->setValue(0.0);
            p.findWidgetByName("SIM_LIMITINGMAG")->setValue(30.0);
            p.findWidgetByName("SIM_NOISE")->setValue(0.0);
            p.findWidgetByName("SIM_SKYGLOW")->setValue(0.0);
            p.findWidgetByName("SIM_BIAS")->setValue(0.0);
            SetupParms();
            memset(PrimaryCCD.getFrameBuffer(), 0, PrimaryCCD.getFrameBufferSize());

            // Mirror DrawCcdFrame's setup of the rendering scale.
            m_Seeing      = 1.0f;
            m_ImageScaleX = 1.0f;
            m_ImageScaleY = 1.0f;

            int const cx = xres / 2 + 1;
            int const cy = yres / 2 + 1;
            DrawImageStar(&PrimaryCCD, 0.0f, cx, cy, 1.0f);

            uint16_t *fb = reinterpret_cast<uint16_t *>(PrimaryCCD.getFrameBuffer());
            EXPECT_GT(fb[cx + cy * xres], 0u)
                << "DrawImageStar must render at least the center pixel";
        }

        // Point the simulated mount at Capella and render. The bright-star
        // supplement must put non-zero ADUs in the frame.
        // Goes through StartExposure so m_ExposureRequest is set before
        // DrawCcdFrame consumes it (calling DrawCcdFrame directly with the
        // default m_ExposureRequest=0 would force totalFlux=0 and render nothing).
        void testBrightStarSupplementRenders()
        {
            prepareForRendering();

            double raHours, decDeg;
            capellaJNow(raHours, decDeg);
            RA  = raHours;
            Dec = decDeg;
            pierSide = 0;

            ASSERT_TRUE(StartExposure(1.0f));

            EXPECT_GT(peakPixel(), 1000)
                << "Pointing at Capella (mag 0.08) with a 60mm aperture should "
                << "saturate at least one pixel; bright-star supplement appears "
                << "to be culling Capella";
        }

        // Reproduce plan Option A: a stored config with SIM_SEEING=0 leaves
        // fwhm2=0; with no aperture either, alpha2 collapses to 0 and the
        // Moffat math produces NaN. DrawImageStar then writes no pixels for
        // any star, so the whole frame goes black.
        void testZeroSeeingStillRenders()
        {
            prepareForRendering();

            // Simulate the failing-config: zero seeing, no aperture override.
            auto p = getNumber("SIMULATOR_SETTINGS");
            p.findWidgetByName("SIM_SEEING")->setValue(0.0);
            ScopeInfoNP[APERTURE].setValue(0.0);  // no override -> falls back to snoopedAperture (NaN)
            SetupParms();
            memset(PrimaryCCD.getFrameBuffer(), 0, PrimaryCCD.getFrameBufferSize());

            double raHours, decDeg;
            capellaJNow(raHours, decDeg);
            RA  = raHours;
            Dec = decDeg;
            pierSide = 0;

            ASSERT_TRUE(StartExposure(1.0f));

            EXPECT_GT(peakPixel(), 1000)
                << "Capella must still render with seeing=0 and unknown aperture; "
                << "this is the runtime configuration that produces a black frame";
        }

        // Same code path but pointed at RA=0 Dec=0 -- no bright-star catalog entries
        // fall inside a ~25 arcmin guide-scope FOV, so the frame must stay dark.
        // This anchors the "off-frame culling actually works" half of the contract.
        void testOffFrameCulling()
        {
            prepareForRendering();

            RA  = 0.0;
            Dec = 0.0;
            pierSide = 0;

            ASSERT_TRUE(StartExposure(1.0f));

            EXPECT_LT(peakPixel(), 100)
                << "No bright-star catalog entry is within a guide FOV of (0,0); "
                << "any star-bright pixel would indicate the culling guard is broken";
        }
};

TEST(GuideSimulatorDriverTest, draw_image_star_basic)
{
    MockGuideSimDriver().testDrawStarBasic();
}

TEST(GuideSimulatorDriverTest, bright_star_supplement_renders)
{
    MockGuideSimDriver().testBrightStarSupplementRenders();
}

TEST(GuideSimulatorDriverTest, zero_seeing_still_renders)
{
    MockGuideSimDriver().testZeroSeeingStillRenders();
}

TEST(GuideSimulatorDriverTest, off_frame_stars_culled)
{
    MockGuideSimDriver().testOffFrameCulling();
}

int main(int argc, char **argv)
{
    INDI::Logger::getInstance().configure("", INDI::Logger::file_off,
                                          INDI::Logger::DBG_ERROR, INDI::Logger::DBG_ERROR);

    ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
