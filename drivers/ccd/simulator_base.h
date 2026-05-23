#pragma once
#include <indiccd.h>
#include <sys/time.h>

// SimulatorBase: shared rendering engine for CCD and guide camera simulators.
// Owns the pixel-level rendering primitives that both drivers use identically.
// CCDSim and GuideSim add their own INDI properties and DrawCcdFrame logic on top.
class SimulatorBase : public INDI::CCD
{
    public:
        SimulatorBase() = default;

    protected:
        // Pixel range tracking updated by AddToPixel each frame.
        int m_MaxPix {0};
        int m_MinPix {65000};

        // ADU ceiling and magnitude calibration.
        // m_SaturationMag saturates in 1 s; m_LimitingMag gives 1 ADU in 1 s.
        int   m_MaxVal        {65000};
        float m_LimitingMag   {11.5};
        float m_SaturationMag {2.0};

        // Imaging geometry - updated each frame from telescope/focuser snooping.
        float m_Seeing      {3.5};  // arcsec FWHM
        float m_ImageScaleX {1.0};  // arcsec/pixel
        float m_ImageScaleY {1.0};  // arcsec/pixel

        // Convert magnitude to ADU per second at the calibration aperture.
        double flux(double mag) const;

        // Draw a Gaussian star PSF centered at pixel (x, y).
        int DrawImageStar(INDI::CCDChip *targetChip, float mag, float x, float y, float exposure_time);

        // Add val ADU to pixel (x, y); clamps to m_MaxVal, updates m_MaxPix/m_MinPix.
        int AddToPixel(INDI::CCDChip *targetChip, int x, int y, int val);

        // Seconds remaining of an exposure started at 'start' with duration 'req'.
        float CalcTimeLeft(timeval start, float req);
};
