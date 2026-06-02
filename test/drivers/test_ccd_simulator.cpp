#include "indicom.h"
#include "indilogger.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

using ::testing::_;
using ::testing::StrEq;

#include "ccd_simulator.h"

char _me[] = "MockCCDSimDriver";
char *me = _me;
class MockCCDSimDriver: public CCDSim
{
    public:
        MockCCDSimDriver(): CCDSim()
        {
            initProperties();
            ISGetProperties(me);
        }

        // Set up a minimal sensor for rendering tests.
        // Returns a pointer to the zeroed 16-bit frame buffer.
        uint16_t *setupSensor(int xres, int yres, int maxval,
                              float satmag, float limmag,
                              float seeing = 1.0f,
                              float scaleX = 1.0f, float scaleY = 1.0f)
        {
            auto p = getNumber("SIMULATOR_SETTINGS");
            p.findWidgetByName("SIM_XRES")->setValue(xres);
            p.findWidgetByName("SIM_YRES")->setValue(yres);
            p.findWidgetByName("SIM_MAXVAL")->setValue(maxval);
            p.findWidgetByName("SIM_XSIZE")->setValue(4.6);
            p.findWidgetByName("SIM_YSIZE")->setValue(4.6);
            p.findWidgetByName("SIM_SATURATION")->setValue(satmag);
            p.findWidgetByName("SIM_LIMITINGMAG")->setValue(limmag);
            p.findWidgetByName("SIM_SKYGLOW")->setValue(0.0);
            p.findWidgetByName("SIM_NOISE")->setValue(0.0);
            m_Seeing      = seeing;
            m_ImageScaleX = scaleX;
            m_ImageScaleY = scaleY;
            setupParameters();
            memset(PrimaryCCD.getFrameBuffer(), 0, PrimaryCCD.getFrameBufferSize());
            return reinterpret_cast<uint16_t *>(PrimaryCCD.getFrameBuffer());
        }

        void testProperties()
        {
            auto p = getNumber("SIMULATOR_SETTINGS");
            ASSERT_NE(p, nullptr);
            ASSERT_NE(p.findWidgetByName("SIM_XRES"), nullptr);
            ASSERT_NE(p.findWidgetByName("SIM_YRES"), nullptr);
            ASSERT_NE(p.findWidgetByName("SIM_XSIZE"), nullptr);
            ASSERT_NE(p.findWidgetByName("SIM_YSIZE"), nullptr);
            ASSERT_NE(p.findWidgetByName("SIM_MAXVAL"), nullptr);
            ASSERT_NE(p.findWidgetByName("SIM_SATURATION"), nullptr);
            ASSERT_NE(p.findWidgetByName("SIM_LIMITINGMAG"), nullptr);
            ASSERT_NE(p.findWidgetByName("SIM_NOISE"), nullptr);
            ASSERT_NE(p.findWidgetByName("SIM_SKYGLOW"), nullptr);
            ASSERT_NE(p.findWidgetByName("SIM_OAGOFFSET"), nullptr);
            ASSERT_NE(p.findWidgetByName("SIM_POLAR"), nullptr);
            ASSERT_NE(p.findWidgetByName("SIM_POLARDRIFT"), nullptr);
        }

        void testGuideAPI()
        {
            EXPECT_TRUE(isnan(currentRA)) << "Field 'currentRA' is undefined when initializing CCDSim.";
            EXPECT_TRUE(isnan(currentDE)) << "Field 'currentDEC' is undefined when initializing CCDSim.";

            EXPECT_EQ(GuideRate, 7 /* arcsec/s */);
            EXPECT_EQ(guideNSOffset, 0);
            EXPECT_EQ(guideWEOffset, 0);

            double const arcsec = 1.0 / 3600.0;

            EXPECT_EQ(GuideNorth(1000.0), IPS_OK);
            EXPECT_NEAR(guideNSOffset, +7 * arcsec, 1 * arcsec);
            EXPECT_EQ(GuideSouth(1000.0), IPS_OK);
            EXPECT_NEAR(guideNSOffset, +0 * arcsec, 1 * arcsec);
            EXPECT_EQ(GuideSouth(1000.0), IPS_OK);
            EXPECT_NEAR(guideNSOffset, -7 * arcsec, 1 * arcsec);
            EXPECT_EQ(GuideNorth(1000.0), IPS_OK);
            EXPECT_NEAR(guideNSOffset, +0 * arcsec, 1 * arcsec);

            currentDE = 0;

            EXPECT_EQ(GuideWest(1000.0), IPS_OK);
            EXPECT_NEAR(guideWEOffset, +7 * arcsec, 15 * arcsec);
            EXPECT_EQ(GuideEast(1000.0), IPS_OK);
            EXPECT_NEAR(guideWEOffset, +0 * arcsec, 15 * arcsec);
            EXPECT_EQ(GuideEast(1000.0), IPS_OK);
            EXPECT_NEAR(guideWEOffset, -7 * arcsec, 15 * arcsec);
            EXPECT_EQ(GuideWest(1000.0), IPS_OK);
            EXPECT_NEAR(guideWEOffset, +0 * arcsec, 15 * arcsec);
        }

        void testDrawStar()
        {
            int const xres   = 65;
            int const yres   = 65;
            int const maxval = 10000;

            uint16_t *fb = setupSensor(xres, yres, maxval, 0.0f, 30.0f);

            ASSERT_EQ(PrimaryCCD.getBPP(), 16);
            ASSERT_EQ(PrimaryCCD.getXRes(), xres);
            ASSERT_EQ(PrimaryCCD.getYRes(), yres);
            ASSERT_NE(PrimaryCCD.getFrameBuffer(), nullptr);

            EXPECT_EQ(m_Seeing,      1.0f);
            EXPECT_EQ(m_ImageScaleX, 1.0f);
            EXPECT_EQ(m_ImageScaleY, 1.0f);

            EXPECT_NEAR(flux(m_SaturationMag), maxval, 1.0);
            EXPECT_NEAR(flux(m_LimitingMag),   1.0,    0.001);

            int const cx = xres / 2 + 1;
            int const cy = yres / 2 + 1;
            DrawImageStar(&PrimaryCCD, 0.0f, cx, cy, 1.0f);

            // Moffat profile: center pixel must be the brightest and non-zero.
            int const center = cx + cy * xres;
            EXPECT_GT(fb[center], 0u);

            // Profile falls off monotonically: 1 px out < center.
            EXPECT_LT(fb[center + 1],    fb[center]);
            EXPECT_LT(fb[center - 1],    fb[center]);
            EXPECT_LT(fb[center + xres], fb[center]);
            EXPECT_LT(fb[center - xres], fb[center]);

            // Moffat has broader wings than Gaussian -- pixels several sigma out are non-zero.
            EXPECT_GT(fb[center + 3], 0u);
            EXPECT_GT(fb[center - 3], 0u);

            // Benchmark
            auto const before = std::chrono::steady_clock::now();
            int const loops = 200000;
            for (int i = 0; i < loops; i++)
            {
                float const m = (15.0f * rand()) / RAND_MAX;
                float const x = static_cast<float>(xres * rand()) / RAND_MAX;
                float const y = static_cast<float>(yres * rand()) / RAND_MAX;
                float const e = (100.0f * rand()) / RAND_MAX;
                DrawImageStar(&PrimaryCCD, m, x, y, e);
            }
            auto const after    = std::chrono::steady_clock::now();
            auto const duration = std::chrono::duration_cast<std::chrono::nanoseconds>(after - before).count() / loops;
            std::cout << "[          ] DrawImageStar benchmark: " << duration << " ns/call\n";
        }

        // Flux must scale as (D/D_ref)^2 between two apertures.
        // Test uses total frame flux rather than peak pixel so that PSF shape
        // changes with the diffraction floor do not confound the measurement.
        void testApertureScaling()
        {
            int const xres   = 33;
            int const yres   = 33;
            int const maxval = 60000;
            int const cx     = xres / 2;
            int const cy     = yres / 2;

            auto sumFrame = [&](uint16_t *fb) -> long {
                long s = 0;
                for (int i = 0; i < xres * yres; i++) s += fb[i];
                return s;
            };

            // Reference aperture (100 mm = m_RefApertureMM default).
            uint16_t *fb = setupSensor(xres, yres, maxval, 10.0f, 20.0f);
            ScopeInfoNP[APERTURE].setValue(100.0);
            DrawImageStar(&PrimaryCCD, 12.0f, cx, cy, 1.0f);
            long const sum100 = sumFrame(fb);

            // Double the aperture -> 4x the flux.
            setupSensor(xres, yres, maxval, 10.0f, 20.0f);
            ScopeInfoNP[APERTURE].setValue(200.0);
            DrawImageStar(&PrimaryCCD, 12.0f, cx, cy, 1.0f);
            long const sum200 = sumFrame(fb);

            ASSERT_GT(sum100, 0L) << "star must register at 100mm aperture";
            EXPECT_NEAR(static_cast<double>(sum200) / sum100, 4.0, 0.4)
                << "total flux should scale as (D/D_ref)^2";
        }

        // BleedColumn must propagate overflow both upward and downward.
        void testBidirectionalBleed()
        {
            int const xres   = 11;
            int const yres   = 11;
            int const maxval = 1000;
            int const cx     = xres / 2;
            int const cy     = yres / 2;

            uint16_t *fb = setupSensor(xres, yres, maxval, 0.0f, 30.0f);

            // Manually saturate the center pixel well above m_MaxVal.
            int const center = cx + cy * xres;
            fb[center] = static_cast<uint16_t>(maxval + 400);

            BleedColumn(&PrimaryCCD, cx, cy);

            // Center must be clamped to m_MaxVal.
            EXPECT_EQ(fb[center], static_cast<uint16_t>(maxval));

            // Pixel above and below must have received overflow.
            EXPECT_GT(fb[center - xres], 0u) << "upward bleed missing";
            EXPECT_GT(fb[center + xres], 0u) << "downward bleed missing";

            // Overflow was split: neither direction gets all of it.
            EXPECT_LT(fb[center - xres], 400u);
            EXPECT_LT(fb[center + xres], 400u);
        }

        // Diffraction spikes must follow m_CameraTheta.
        // Each test draws a single frame and compares an on-axis pixel against an
        // off-axis pixel at comparable distance. The halo is isotropic so it
        // contributes equally to both; only the spike is directional.
        //
        // At theta=0, d=10: spike lands at (cx+10, cy). Off-axis reference: (cx+7, cy+7)
        // at r~10 px, same halo/PSF but no spike.
        // At theta=pi/4, d=10: round(10*cos(45))=7, so spike lands at (cx+7, cy+7).
        // The horizontal pixel (cx+10, cy) is then off-axis.
        void testDiffractionSpikeOrientation()
        {
            int const xres   = 65;
            int const yres   = 65;
            int const maxval = 65000;
            int const cx     = xres / 2;
            int const cy     = yres / 2;
            // totalFlux ~ 65000 * 10 * 4 = 2.6M -> spikeCoeff = 520.
            // At d=10: spike ADU = int(520/100) = 5; halo at r=10 = int(2600/100) = 26.
            float const exposure = 100.0f;

            // Theta = 0: horizontal arm is lit by spikes.
            // No explicit aperture: apertureScale=1, totalFlux=65000*100=6.5M,
            // spikeCoeff=1300, spike at d=10 = int(1300/100) = 13 ADU, halo at r=10 = 65 ADU.
            uint16_t *fb = setupSensor(xres, yres, maxval, 0.0f, 20.0f);
            m_DiffractionSpikes = true;
            m_CameraTheta       = 0.0;
            DrawImageStar(&PrimaryCCD, 0.0f, cx, cy, exposure);
            uint16_t const horiz_0 = fb[cy * xres + cx + 10];       // on spike axis
            uint16_t const diag_0  = fb[(cy + 7) * xres + cx + 7];  // off spike axis

            // Theta = 45: diagonal arm is lit by spikes.
            fb = setupSensor(xres, yres, maxval, 0.0f, 20.0f);
            m_DiffractionSpikes = true;
            m_CameraTheta       = M_PI / 4.0;
            DrawImageStar(&PrimaryCCD, 0.0f, cx, cy, exposure);
            uint16_t const horiz_45 = fb[cy * xres + cx + 10];       // off spike axis
            uint16_t const diag_45  = fb[(cy + 7) * xres + cx + 7];  // on spike axis

            // Theta = 90: vertical arm is lit by spikes.
            // At d=10: arm 1 hits (cx, cy+10); arm 2 hits (cx-10,cy) and (cx+10,cy).
            // Off-axis reference (cx+7,cy+7) lies on no arm at theta=90.
            fb = setupSensor(xres, yres, maxval, 0.0f, 20.0f);
            m_DiffractionSpikes = true;
            m_CameraTheta       = M_PI / 2.0;
            DrawImageStar(&PrimaryCCD, 0.0f, cx, cy, exposure);
            uint16_t const vert_90 = fb[(cy + 10) * xres + cx];      // on spike axis (arm 1)
            uint16_t const diag_90 = fb[(cy + 7) * xres + cx + 7];   // off all spike axes

            EXPECT_GT(horiz_0,  diag_0)   << "theta=0: horizontal arm should be brighter than diagonal";
            EXPECT_GT(diag_45,  horiz_45) << "theta=45: diagonal arm should be brighter than horizontal";
            EXPECT_GT(vert_90,  diag_90)  << "theta=90: vertical arm should be brighter than diagonal";
        }

        // Disabling m_DiffractionSpikes must remove the on-axis brightness boost.
        void testDiffractionSpikesDisabled()
        {
            int const xres   = 65;
            int const yres   = 65;
            int const maxval = 65000;
            int const cx     = xres / 2;
            int const cy     = yres / 2;
            float const exposure = 100.0f;

            uint16_t *fb = setupSensor(xres, yres, maxval, 0.0f, 20.0f);
            m_DiffractionSpikes = true;
            m_CameraTheta       = 0.0;
            DrawImageStar(&PrimaryCCD, 0.0f, cx, cy, exposure);
            uint16_t const with_spikes = fb[cy * xres + cx + 10];

            fb = setupSensor(xres, yres, maxval, 0.0f, 20.0f);
            m_DiffractionSpikes = false;
            m_CameraTheta       = 0.0;
            DrawImageStar(&PrimaryCCD, 0.0f, cx, cy, exposure);
            uint16_t const without_spikes = fb[cy * xres + cx + 10];

            EXPECT_GT(with_spikes, without_spikes)
                << "enabling spikes must add brightness on the spike axis";
        }

        // 1/r^2 halo must reach well past the PSF core and be isotropic.
        // Also verifies the halo is suppressed below the flux threshold.
        void testHaloIsotropic()
        {
            int const xres = 65;
            int const yres = 65;
            int const cx   = xres / 2;
            int const cy   = yres / 2;

            // Bright star: haloCoeff = 1e-3 * flux(0) * 100 = 6500.
            // At r=20: adu = int(6500/400) = 16. PSF contributes 0 ADU at r=20.
            uint16_t *fb = setupSensor(xres, yres, 65000, 0.0f, 20.0f);
            m_DiffractionSpikes = false;
            DrawImageStar(&PrimaryCCD, 0.0f, cx, cy, 100.0f);

            uint16_t const right = fb[cy * xres + cx + 20];
            uint16_t const left  = fb[cy * xres + cx - 20];
            uint16_t const down  = fb[(cy + 20) * xres + cx];
            uint16_t const up    = fb[(cy - 20) * xres + cx];

            EXPECT_GT(right, 0u) << "halo must illuminate pixels at r=20";
            EXPECT_EQ(right, left) << "halo must be isotropic (right vs left)";
            EXPECT_EQ(right, down) << "halo must be isotropic (right vs down)";
            EXPECT_EQ(right, up)   << "halo must be isotropic (right vs up)";

            // Sub-threshold: totalFlux = flux(0) * 0.01 = 650 < 1/haloNorm = 1000.
            // Halo loop is skipped entirely; no contribution at r=20.
            fb = setupSensor(xres, yres, 65000, 0.0f, 20.0f);
            m_DiffractionSpikes = false;
            DrawImageStar(&PrimaryCCD, 0.0f, cx, cy, 0.01f);
            EXPECT_EQ(fb[cy * xres + cx + 20], 0u)
                << "halo must not appear below flux threshold";
        }

        // flux() must be calibrated at both boundary magnitudes and be monotone.
        // The degenerate case (sat == lim) must return 1.0 without div-by-zero.
        void testFluxCalibration()
        {
            setupSensor(33, 33, 65000, 2.0f, 20.0f);
            EXPECT_NEAR(flux(m_SaturationMag), m_MaxVal, 1.0)
                << "flux at saturation mag must equal m_MaxVal";
            EXPECT_NEAR(flux(m_LimitingMag), 1.0, 0.001)
                << "flux at limiting mag must equal 1.0";
            EXPECT_GT(flux(0.0), flux(5.0)) << "brighter stars must yield more flux";

            setupSensor(33, 33, 65000, 10.0f, 10.0f);
            EXPECT_EQ(flux(10.0), 1.0) << "degenerate mag range must return 1.0";
        }

        // Spike loop must be suppressed when totalFlux <= 500.
        // Contrast: sub-threshold star shows no on-axis elevation vs an above-threshold star.
        void testDimStarSpikesSuppressed()
        {
            int const xres = 65;
            int const yres = 65;
            int const cx   = xres / 2;
            int const cy   = yres / 2;

            // Sub-threshold: totalFlux = flux(0) * 0.005 = 325 <= 500 -> spike loop skipped.
            uint16_t *fb = setupSensor(xres, yres, 65000, 0.0f, 20.0f);
            m_DiffractionSpikes = true;
            m_CameraTheta       = 0.0;
            DrawImageStar(&PrimaryCCD, 0.0f, cx, cy, 0.005f);
            uint16_t const on_axis_dim = fb[cy * xres + cx + 10];

            // Above threshold: totalFlux = 6.5M >> 500 -> spikes rendered, on-axis elevated.
            fb = setupSensor(xres, yres, 65000, 0.0f, 20.0f);
            m_DiffractionSpikes = true;
            m_CameraTheta       = 0.0;
            DrawImageStar(&PrimaryCCD, 0.0f, cx, cy, 100.0f);
            uint16_t const on_axis_bright = fb[cy * xres + cx + 10];

            EXPECT_GT(on_axis_bright, on_axis_dim)
                << "sub-threshold star must show no spike elevation on the spike axis";
        }

        // AddToPixel must clamp at uint16_t max (65535), not at m_MaxVal.
        // A pixel clamped to 65535 must then trigger BleedColumn redistribution.
        void testAddToPixelClamp()
        {
            int const xres   = 5;
            int const yres   = 5;
            int const maxval = 1000;
            int const cx     = xres / 2;
            int const cy     = yres / 2;

            uint16_t *fb = setupSensor(xres, yres, maxval, 0.0f, 30.0f);

            // Pixel starts at 1; adding 65535 must clamp to 65535, not wrap to 0.
            fb[cy * xres + cx] = 1;
            AddToPixel(&PrimaryCCD, cx, cy, 65535);
            EXPECT_EQ(fb[cy * xres + cx], static_cast<uint16_t>(65535))
                << "AddToPixel must clamp at 65535, not wrap";

            // BleedColumn must drain from 65535 down to m_MaxVal and distribute excess.
            BleedColumn(&PrimaryCCD, cx, cy);
            EXPECT_EQ(fb[cy * xres + cx], static_cast<uint16_t>(maxval))
                << "BleedColumn must drain center pixel to m_MaxVal";
            EXPECT_GT(fb[(cy - 1) * xres + cx], 0u) << "excess must bleed upward";
            EXPECT_GT(fb[(cy + 1) * xres + cx], 0u) << "excess must bleed downward";
        }

        // BleedColumn must cascade overflow through multiple saturated rows.
        void testBleedColumnCascade()
        {
            int const xres   = 5;
            int const yres   = 9;
            int const maxval = 500;
            int const cx     = xres / 2;
            int const cy     = yres / 2;  // row 4

            uint16_t *fb = setupSensor(xres, yres, maxval, 0.0f, 30.0f);

            // 5x overflow: upward and downward carry each saturate two rows before stopping.
            // carry_up = ceil(2000/2) = 1000 -> row cy-1 saturates (carry 500) -> row cy-2 = 500 (= maxval, carry = 0)
            // carry_dn = floor(2000/2) = 1000 -> row cy+1 saturates (carry 500) -> row cy+2 = 500 (= maxval, carry = 0)
            fb[cy * xres + cx] = static_cast<uint16_t>(5 * maxval);
            BleedColumn(&PrimaryCCD, cx, cy);

            EXPECT_EQ(fb[ cy      * xres + cx], static_cast<uint16_t>(maxval)) << "center must clamp";
            EXPECT_EQ(fb[(cy - 1) * xres + cx], static_cast<uint16_t>(maxval)) << "cascade: row above center";
            EXPECT_EQ(fb[(cy - 2) * xres + cx], static_cast<uint16_t>(maxval)) << "cascade: two rows above center";
            EXPECT_EQ(fb[(cy + 1) * xres + cx], static_cast<uint16_t>(maxval)) << "cascade: row below center";
            EXPECT_EQ(fb[(cy + 2) * xres + cx], static_cast<uint16_t>(maxval)) << "cascade: two rows below center";
        }

        // BleedColumn must stop at the top and bottom frame boundaries without writing
        // out-of-bounds or losing overflow silently.
        void testBleedColumnBoundary()
        {
            int const xres   = 5;
            int const yres   = 5;
            int const maxval = 1000;
            int const cx     = xres / 2;

            // Top row: upward loop must not execute (row -1 does not exist).
            uint16_t *fb = setupSensor(xres, yres, maxval, 0.0f, 30.0f);
            fb[0 * xres + cx] = static_cast<uint16_t>(maxval + 400);
            BleedColumn(&PrimaryCCD, cx, 0);
            EXPECT_EQ(fb[0 * xres + cx], static_cast<uint16_t>(maxval))
                << "top-row pixel must be clamped to m_MaxVal";
            EXPECT_GT(fb[1 * xres + cx], 0u) << "overflow must drain downward from top row";

            // Bottom row: downward loop must not execute (row yres does not exist).
            fb = setupSensor(xres, yres, maxval, 0.0f, 30.0f);
            fb[(yres - 1) * xres + cx] = static_cast<uint16_t>(maxval + 400);
            BleedColumn(&PrimaryCCD, cx, yres - 1);
            EXPECT_EQ(fb[(yres - 1) * xres + cx], static_cast<uint16_t>(maxval))
                << "bottom-row pixel must be clamped to m_MaxVal";
            EXPECT_GT(fb[(yres - 2) * xres + cx], 0u) << "overflow must drain upward from bottom row";
        }

        // Aperture below kMinApertureMM (5mm) must not apply the diffraction floor.
        // Aperture above it must broaden the PSF visibly at the same exposure.
        void testDiffractionFloor()
        {
            int const xres = 33;
            int const yres = 33;
            int const cx   = xres / 2;
            int const cy   = yres / 2;

            // D=4mm (< kMinApertureMM=5mm): no diffraction floor -> narrow PSF (seeing-limited, FWHM~1").
            // At r=8 px (scaleX=1"/px): PSF contribution is 0 ADU with totalFlux~39k.
            uint16_t *fb = setupSensor(xres, yres, 60000, 10.0f, 20.0f);
            ScopeInfoNP[APERTURE].setValue(4.0);
            DrawImageStar(&PrimaryCCD, 15.0f, cx, cy, 100000.0f);
            uint16_t const wing_4mm = fb[cy * xres + cx + 8];

            // D=6mm (> kMinApertureMM=5mm): diffraction floor = 23 arcsec -> FWHM~23".
            // At r=8 px the broad PSF puts ~70 ADU into this pixel.
            fb = setupSensor(xres, yres, 60000, 10.0f, 20.0f);
            ScopeInfoNP[APERTURE].setValue(6.0);
            DrawImageStar(&PrimaryCCD, 15.0f, cx, cy, 100000.0f);
            uint16_t const wing_6mm = fb[cy * xres + cx + 8];

            EXPECT_EQ(wing_4mm, 0u)
                << "aperture below kMinApertureMM=5mm must not broaden PSF";
            EXPECT_GT(wing_6mm, 0u)
                << "aperture above kMinApertureMM=5mm must apply diffraction floor";

            // Wider separation: D=10mm (floor=13.9", broad) vs D=200mm (floor=0.69", narrow).
            // Equal-exposure comparison: D=10mm has far more diffraction spread at r=8.
            fb = setupSensor(xres, yres, 60000, 10.0f, 20.0f);
            ScopeInfoNP[APERTURE].setValue(10.0);
            DrawImageStar(&PrimaryCCD, 15.0f, cx, cy, 100000.0f);
            uint16_t const wing_10mm = fb[cy * xres + cx + 8];

            fb = setupSensor(xres, yres, 60000, 10.0f, 20.0f);
            ScopeInfoNP[APERTURE].setValue(200.0);
            DrawImageStar(&PrimaryCCD, 15.0f, cx, cy, 1.0f);
            uint16_t const wing_200mm = fb[cy * xres + cx + 8];

            EXPECT_GT(wing_10mm, wing_200mm)
                << "small aperture (large diffraction floor) must spread more flux to wing pixels";
        }

        // Stars within kMaxStarInfluencePx (100px) of the frame edge must illuminate
        // border pixels via their halo. Stars beyond 100px must be culled entirely.
        void testOffFrameStarHalo()
        {
            int const xres       = 65;
            int const yres       = 65;
            int const cy         = yres / 2;
            int const frame_edge = xres - 1;  // column 64

            // Star 50px beyond right edge: halo at r=51 gives int(6500/2601)=2 ADU at x=64.
            uint16_t *fb = setupSensor(xres, yres, 65000, 0.0f, 20.0f);
            m_DiffractionSpikes = false;
            DrawImageStar(&PrimaryCCD, 0.0f, static_cast<float>(xres + 50),
                          static_cast<float>(cy), 100.0f);
            EXPECT_GT(fb[cy * xres + frame_edge], 0u)
                << "halo from star 50px off-frame must illuminate the frame edge";

            // Star 101px beyond right edge: guard condition (x > subW + 100) culls it entirely.
            fb = setupSensor(xres, yres, 65000, 0.0f, 20.0f);
            m_DiffractionSpikes = false;
            DrawImageStar(&PrimaryCCD, 0.0f, static_cast<float>(xres + 101),
                          static_cast<float>(cy), 100.0f);
            bool any_lit = false;
            for (int i = 0; i < xres * yres; i++) any_lit |= (fb[i] > 0);
            EXPECT_FALSE(any_lit)
                << "star beyond kMaxStarInfluencePx=100 must write no pixels";
        }
};

TEST(CCDSimulatorDriverTest, test_properties)
{
    MockCCDSimDriver().testProperties();
}

TEST(CCDSimulatorDriverTest, test_guide_api)
{
    MockCCDSimDriver().testGuideAPI();
}

TEST(CCDSimulatorDriverTest, test_draw_star)
{
    MockCCDSimDriver().testDrawStar();
}

TEST(CCDSimulatorDriverTest, test_aperture_scaling)
{
    MockCCDSimDriver().testApertureScaling();
}

TEST(CCDSimulatorDriverTest, test_bidirectional_bleed)
{
    MockCCDSimDriver().testBidirectionalBleed();
}

TEST(CCDSimulatorDriverTest, test_diffraction_spike_orientation)
{
    MockCCDSimDriver().testDiffractionSpikeOrientation();
}

TEST(CCDSimulatorDriverTest, test_diffraction_spikes_disabled)
{
    MockCCDSimDriver().testDiffractionSpikesDisabled();
}

TEST(CCDSimulatorDriverTest, test_halo_isotropic)
{
    MockCCDSimDriver().testHaloIsotropic();
}

TEST(CCDSimulatorDriverTest, test_flux_calibration)
{
    MockCCDSimDriver().testFluxCalibration();
}

TEST(CCDSimulatorDriverTest, test_dim_star_spikes_suppressed)
{
    MockCCDSimDriver().testDimStarSpikesSuppressed();
}

TEST(CCDSimulatorDriverTest, test_add_to_pixel_clamp)
{
    MockCCDSimDriver().testAddToPixelClamp();
}

TEST(CCDSimulatorDriverTest, test_bleed_column_cascade)
{
    MockCCDSimDriver().testBleedColumnCascade();
}

TEST(CCDSimulatorDriverTest, test_bleed_column_boundary)
{
    MockCCDSimDriver().testBleedColumnBoundary();
}

TEST(CCDSimulatorDriverTest, test_diffraction_floor)
{
    MockCCDSimDriver().testDiffractionFloor();
}

TEST(CCDSimulatorDriverTest, test_off_frame_star_halo)
{
    MockCCDSimDriver().testOffFrameStarHalo();
}

int main(int argc, char **argv)
{
    INDI::Logger::getInstance().configure("", INDI::Logger::file_off,
                                          INDI::Logger::DBG_ERROR, INDI::Logger::DBG_ERROR);

    ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
