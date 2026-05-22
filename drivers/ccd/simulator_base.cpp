#include "simulator_base.h"
#include <cmath>
#include <sys/time.h>
#include <libnova/julian_day.h>
#include <libnova/lunar.h>

// Central wavelength used for diffraction-floor FWHM calculation (mm).
static constexpr float kCenterWavelengthMM = 550e-6f;

// Minimum aperture accepted for diffraction calculations (mm).
// Values below this produce nonphysical PSF widths and are treated as "unknown".
static constexpr float kMinApertureMM = 5.0f;

double SimulatorBase::flux(double mag) const
{
    if (m_LimitingMag == m_SaturationMag)
        return 1.0;
    double const z = m_LimitingMag;
    double const k = 2.5 * log10(m_MaxVal) / (m_LimitingMag - m_SaturationMag);
    return pow(10, (z - mag) * k / 2.5);
}

int SimulatorBase::DrawImageStar(INDI::CCDChip *targetChip, float mag, float x, float y, float exposure_time)
{
    // #20: guard against uninitialized image scale (causes division by zero below).
    if (m_ImageScaleX <= 0.0f || m_ImageScaleY <= 0.0f ||
        !std::isfinite(m_ImageScaleX) || !std::isfinite(m_ImageScaleY))
        return 0;

    int drew = 0;

    int const subX = targetChip->getSubX();
    int const subY = targetChip->getSubY();
    int const subW = targetChip->getSubW() + subX;
    int const subH = targetChip->getSubH() + subY;

    // #18: allow stars whose centers are slightly outside the frame -- their halo
    // and diffraction spikes can still illuminate the sensor. AddToPixel clips every
    // write to the subframe, so over-shooting is safe.
    static constexpr float kMaxStarInfluencePx = 100.0f;
    if (x < subX - kMaxStarInfluencePx || x > subW + kMaxStarInfluencePx ||
        y < subY - kMaxStarInfluencePx || y > subH + kMaxStarInfluencePx)
        return 0;

    // Aperture: prefer the manual ScopeInfoNP entry; fall back to the aperture
    // snooped from the connected telescope via INDI::CCD::ISSnoopDevice.
    // Flux scales as (D / m_RefApertureMM)^2 so larger scopes collect more light.
    double const apertureMM = ScopeInfoNP[APERTURE].getValue() > 0
                                  ? ScopeInfoNP[APERTURE].getValue()
                                  : snoopedAperture;
    float const apertureScale = (!std::isnan(apertureMM) && apertureMM > 0.0)
                                    ? static_cast<float>((apertureMM / m_RefApertureMM) * (apertureMM / m_RefApertureMM))
                                    : 1.0f;

    float const totalFlux = static_cast<float>(flux(mag) * exposure_time * apertureScale);

    float const pixel_area = m_ImageScaleX * m_ImageScaleY;

    if (!std::isfinite(pixel_area) || pixel_area <= 0.0f)
        return 0;

    // Moffat profile: f(r) = (beta-1)/(pi*alpha^2) * (1 + r^2/alpha^2)^(-beta)
    // FWHM_total^2 = FWHM_seeing^2 + (1.22*lambda/D * 206265)^2  (diffraction floor)
    float const beta     = 2.5f;
    float const betaTerm = 4.0f * (std::pow(2.0f, 1.0f / beta) - 1.0f);

    float fwhm2 = m_Seeing * m_Seeing;
    // #4: clamp aperture to kMinApertureMM before computing diffraction floor to
    // prevent 1/aperture blowing up for unphysically small values.
    if (!std::isnan(apertureMM) && apertureMM >= kMinApertureMM)
    {
        float const fwhm_diff = 1.22f * kCenterWavelengthMM / static_cast<float>(apertureMM) * 206265.0f;
        fwhm2 += fwhm_diff * fwhm_diff;
    }

    float const alpha2     = fwhm2 / betaTerm;
    float const moffatNorm = (beta - 1.0f) / (float(M_PI) * alpha2);

    // Compute rendering radius analytically: solve moffat(r)*totalFlux*pixel_area = 0.5 ADU.
    float const threshold = std::max(moffatNorm * totalFlux * pixel_area * 2.0f, 1.0f);
    float const r2_max    = alpha2 * (std::pow(threshold, 1.0f / beta) - 1.0f);
    int   const moffatBox = (r2_max > 0.0f)
                                ? static_cast<int>(std::sqrt(r2_max) / std::min(m_ImageScaleX, m_ImageScaleY)) + 1
                                : static_cast<int>(3.0f * m_Seeing / std::min(m_ImageScaleX, m_ImageScaleY)) + 1;
    // Hard cap: 40 pixels captures >99.9% of Moffat energy at beta=2.5.
    static constexpr int maxRenderBox = 40;
    int const boxsizey = std::min(moffatBox, maxRenderBox);

    float const fracX = x - std::floor(x);
    float const fracY = y - std::floor(y);
    int   const ix    = static_cast<int>(std::floor(x));
    int   const iy    = static_cast<int>(std::floor(y));

    int const box2 = boxsizey * boxsizey;
    for (int sy = -boxsizey; sy <= boxsizey; sy++)
    {
        for (int sx = -boxsizey; sx <= boxsizey; sx++)
        {
            if (sx * sx + sy * sy > box2)
                continue;
            float const dx  = (sx - fracX) * m_ImageScaleX;
            float const dy  = (sy - fracY) * m_ImageScaleY;
            float const dc2 = dx * dx + dy * dy;
            float const moffat = moffatNorm * std::pow(1.0f + dc2 / alpha2, -beta);
            int   const fp     = static_cast<int>(moffat * totalFlux * pixel_area);
            if (fp > 0)
            {
                if (AddToPixel(targetChip, ix + sx, iy + sy, fp) != 0)
                    drew = 1;
            }
        }
    }

    // Isotropic 1/r^2 scattered-light halo for bright stars.
    static constexpr float haloNorm = 1e-3f;
    float const haloCoeff = haloNorm * totalFlux;
    if (haloCoeff >= 1.0f)
    {
        int const maxHaloR = static_cast<int>(std::sqrt(haloCoeff)) + 1;
        for (int sy = -maxHaloR; sy <= maxHaloR; sy++)
        {
            for (int sx = -maxHaloR; sx <= maxHaloR; sx++)
            {
                float const r2 = static_cast<float>(sx * sx + sy * sy);
                if (r2 < 1.0f)
                    continue;
                int const adu = static_cast<int>(haloCoeff / r2);
                if (adu < 1)
                    continue;
                AddToPixel(targetChip, ix + sx, iy + sy, adu);
            }
        }
    }

    // Saturation blooming: bleed columns inside the bright PSF core (~3x FWHM radius).
    int const bleedBox = static_cast<int>(3.0f * m_Seeing / std::min(m_ImageScaleX, m_ImageScaleY)) + 1;
    for (int sx = -bleedBox; sx <= bleedBox; sx++)
        BleedColumn(targetChip, ix + sx, iy);

    // Diffraction spikes: only for scopes with secondary mirror spider vanes (reflectors).
    // Models a four-vane Newtonian-style spider; vane angle follows m_CameraTheta.
    if (m_DiffractionSpikes && totalFlux > 500.0f)
    {
        float const spikeCoeff = 0.0002f * totalFlux;
        int   const maxDist    = std::max(targetChip->getSubW(), targetChip->getSubH());
        float const cosT = std::cos(static_cast<float>(m_CameraTheta));
        float const sinT = std::sin(static_cast<float>(m_CameraTheta));
        for (int d = 1; d <= maxDist; d++)
        {
            int const adu = static_cast<int>(spikeCoeff / (d * d));
            if (adu < 1)
                break;
            float const fd = static_cast<float>(d);
            int const dx1 = static_cast<int>(std::round(fd * cosT));
            int const dy1 = static_cast<int>(std::round(fd * sinT));
            AddToPixel(targetChip, ix + dx1, iy + dy1, adu);
            AddToPixel(targetChip, ix - dx1, iy - dy1, adu);
            int const dx2 = static_cast<int>(std::round(-fd * sinT));
            int const dy2 = static_cast<int>(std::round( fd * cosT));
            AddToPixel(targetChip, ix + dx2, iy + dy2, adu);
            AddToPixel(targetChip, ix - dx2, iy - dy2, adu);
        }
    }

    return drew;
}

// #8: BleedColumn propagates overflow above m_MaxVal bidirectionally.
// AddToPixel allows pixel values up to uint16_t max (65535) so that this
// function can see and redistribute the excess above the saturation floor.
void SimulatorBase::BleedColumn(INDI::CCDChip *targetChip, int cx, int cy)
{
    int const nwidth  = targetChip->getSubW();
    int const nheight = targetChip->getSubH();
    int const ix      = cx - targetChip->getSubX();
    int const iy      = cy - targetChip->getSubY();

    if (ix < 0 || ix >= nwidth || iy < 0 || iy >= nheight)
        return;

    uint16_t *buf    = reinterpret_cast<uint16_t *>(targetChip->getFrameBuffer());
    uint16_t *center = buf + iy * nwidth + ix;

    if (static_cast<int>(*center) <= m_MaxVal)
        return;

    // Split overflow equally: odd ADU goes upward.
    int const total = static_cast<int>(*center) - m_MaxVal;
    *center = static_cast<uint16_t>(m_MaxVal);

    // Drain upward (toward row 0), carrying ceil(total/2).
    int carry = (total + 1) / 2;
    for (int py = iy - 1; py >= 0 && carry > 0; py--)
    {
        uint16_t *p = buf + py * nwidth + ix;
        int newval  = static_cast<int>(*p) + carry;
        if (newval > m_MaxVal)
        {
            carry  = newval - m_MaxVal;
            newval = m_MaxVal;
        }
        else
        {
            carry = 0;
        }
        if (newval > m_MaxPix) m_MaxPix = newval;
        *p = static_cast<uint16_t>(newval);
    }

    // Drain downward (toward row nheight-1), carrying floor(total/2).
    carry = total / 2;
    for (int py = iy + 1; py < nheight && carry > 0; py++)
    {
        uint16_t *p = buf + py * nwidth + ix;
        int newval  = static_cast<int>(*p) + carry;
        if (newval > m_MaxVal)
        {
            carry  = newval - m_MaxVal;
            newval = m_MaxVal;
        }
        else
        {
            carry = 0;
        }
        if (newval > m_MaxPix) m_MaxPix = newval;
        *p = static_cast<uint16_t>(newval);
    }
}

// #8: Clip at uint16_t max (65535), not at m_MaxVal, so BleedColumn can see
// and redistribute the excess above the saturation floor.
int SimulatorBase::AddToPixel(INDI::CCDChip *targetChip, int x, int y, int val)
{
    int const nwidth  = targetChip->getSubW();
    int const nheight = targetChip->getSubH();

    x -= targetChip->getSubX();
    y -= targetChip->getSubY();

    if (x < 0 || x >= nwidth || y < 0 || y >= nheight)
        return 0;

    auto *pt = reinterpret_cast<uint16_t *>(targetChip->getFrameBuffer());
    pt += y * nwidth + x;

    int newval = static_cast<int>(pt[0]) + val;
    if (newval > 65535)
        newval = 65535;
    if (newval > m_MaxPix)
        m_MaxPix = newval;
    if (newval < m_MinPix)
        m_MinPix = newval;
    pt[0] = static_cast<uint16_t>(newval);

    return 1;
}

float SimulatorBase::CalcTimeLeft(timeval start, float req)
{
    struct timeval now {0, 0};
    gettimeofday(&now, nullptr);
    double const timesince = (now.tv_sec  * 1000.0 + now.tv_usec  / 1000.0)
                           - (start.tv_sec * 1000.0 + start.tv_usec / 1000.0);
    return static_cast<float>(req - timesince / 1000.0);
}

void SimulatorBase::DrawMoon(INDI::CCDChip *targetChip, double cx, double cy, float exposure_time)
{
    if (m_ImageScaleX <= 0.0f || m_ImageScaleY <= 0.0f)
        return;

    double jd = ln_get_julian_from_sys();

    // #11: compute separate pixel radii per axis so the disk stays circular on
    // sky even when pixels are not square.
    double const R_arcsec = ln_get_lunar_sdiam(jd);
    double const Rx       = R_arcsec / m_ImageScaleX;
    double const Ry       = R_arcsec / m_ImageScaleY;
    if (Rx < 0.5 || Ry < 0.5)
        return;

    double const phase_rad      = ln_get_lunar_phase(jd) * M_PI / 180.0;
    double const bright_limb_PA = ln_get_lunar_bright_limb(jd) * M_PI / 180.0;

    // Bright-limb unit vector in sky arcsec coordinates (N up, E left after flip).
    double const arg   = m_CameraTheta + bright_limb_PA;
    double const bl_dx = -sin(arg);
    double const bl_dy =  cos(arg);

    // #13: hoist loop-invariant terminator factor.
    double const cos_phase = cos(phase_rad);

    // Full-moon surface brightness ~3.4 mag/arcsec^2; scale by pixel area.
    double const pixel_area = m_ImageScaleX * m_ImageScaleY;
    double pixel_flux = flux(3.4 - 2.5 * log10(pixel_area)) * exposure_time;

    // #14: apply aperture scaling -- larger scopes collect more lunar surface flux.
    double const apertureMM = ScopeInfoNP[APERTURE].getValue() > 0
                                  ? ScopeInfoNP[APERTURE].getValue()
                                  : snoopedAperture;
    if (!std::isnan(apertureMM) && apertureMM > 0.0)
        pixel_flux *= (apertureMM / m_RefApertureMM) * (apertureMM / m_RefApertureMM);

    int const subX = targetChip->getSubX();
    int const subY = targetChip->getSubY();
    int const subW = subX + targetChip->getSubW();
    int const subH = subY + targetChip->getSubH();
    int const xlo  = std::max(subX, (int)(cx - Rx) - 1);
    int const xhi  = std::min(subW, (int)(cx + Rx) + 2);
    int const ylo  = std::max(subY, (int)(cy - Ry) - 1);
    int const yhi  = std::min(subH, (int)(cy + Ry) + 2);

    for (int iy = ylo; iy < yhi; iy++)
    {
        for (int ix = xlo; ix < xhi; ix++)
        {
            // #11: disk test in pixel-ellipse space (== sky-circle space).
            double const u_pix = (ix - cx) / Rx;
            double const v_pix = (iy - cy) / Ry;
            if (u_pix * u_pix + v_pix * v_pix > 1.0)
                continue;

            // Illumination terminator: work in sky arcsec coords (normalized by R_arcsec)
            // so the terminator shape is correct for non-square pixels.
            double const u = (ix - cx) * m_ImageScaleX / R_arcsec;
            double const v = (iy - cy) * m_ImageScaleY / R_arcsec;

            double const lu  =  u * bl_dx + v * bl_dy;
            double const lv  = -u * bl_dy + v * bl_dx;
            double       lv2 = lv * lv;
            if (lv2 > 1.0) lv2 = 1.0;
            if (lu < cos_phase * sqrt(1.0 - lv2))
                continue;

            AddToPixel(targetChip, ix, iy, static_cast<int>(pixel_flux));
        }
    }
}
