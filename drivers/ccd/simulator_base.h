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

        // Reference aperture for flux calibration (mm).
        // CCD sim: 100 mm; guide sim: 30 mm (typical guide scope).
        // Flux scales as (D / m_RefApertureMM)^2.
        double m_RefApertureMM {100.0};

        // Camera rotation in radians; updated each frame from rotation offset + snooped rotator angle.
        double m_CameraTheta {0.0};

        // When true, render 4-pointed diffraction spikes for bright stars.
        // Only appropriate for scopes with secondary mirror spider vanes (reflectors).
        bool m_DiffractionSpikes {false};

        // Convert magnitude to ADU per second at the reference aperture.
        double flux(double mag) const;

        // Draw a Moffat PSF star centered at pixel (x, y), aperture-scaled.
        int DrawImageStar(INDI::CCDChip *targetChip, float mag, float x, float y, float exposure_time);

        // Render the lunar disk at pixel (cx, cy) using libnova phase/position data.
        void DrawMoon(INDI::CCDChip *targetChip, double cx, double cy, float exposure_time);

        // Propagate saturated-pixel overflow up/down the column.
        void BleedColumn(INDI::CCDChip *targetChip, int cx, int cy);

        // Add val ADU to pixel (x, y); clamps to uint16_t max (65535) so BleedColumn
        // can see and redistribute the excess above m_MaxVal. Updates m_MaxPix/m_MinPix.
        int AddToPixel(INDI::CCDChip *targetChip, int x, int y, int val);

        // Seconds remaining of an exposure started at 'start' with duration 'req'.
        float CalcTimeLeft(timeval start, float req);
};
