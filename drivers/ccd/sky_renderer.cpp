#include "sky_renderer.h"
#include "bright_stars_catalog.h"
#include "indicom.h"
#include "locale_compat.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#ifdef HAVE_ERFA
#include <erfa.h>

// Apply a pre-built eraASTROM context to one ICRS catalog star.
// eraAtciqz handles zero proper-motion/parallax/RV (the common catalog case).
// eraAtioq adds diurnal aberration and refraction.
// Result is equinox-based apparent RA/Dec (topocentric, refracted).
static inline void applyAstrom(double ra_icrs, double dec_icrs,
                               eraASTROM *astrom, double eo,
                               double *ra_app, double *dec_app)
{
    double ri, di;
    eraAtciqz(ra_icrs, dec_icrs, astrom, &ri, &di);

    double aob, zob, hob, dob, rob;
    eraAtioq(ri, di, astrom, &aob, &zob, &hob, &dob, &rob);

    *ra_app  = eraAnp(rob - eo);
    *dec_app = dob;
}
#endif // HAVE_ERFA

static constexpr float kCenterWavelengthMM = 550e-6f;
static constexpr float kMinApertureMM = 5.0f;

double SkyRenderer::flux(double mag) const
{
    if (m_Cfg.limitingMag == m_Cfg.saturationMag)
        return 1.0;
    double const z = m_Cfg.limitingMag;
    double const k = 2.5 * log10(m_Cfg.maxVal) / (m_Cfg.limitingMag - m_Cfg.saturationMag);
    return pow(10.0, (z - mag) * k / 2.5);
}

int SkyRenderer::addToPixel(INDI::CCDChip *chip, int x, int y, int val)
{
    int const nwidth  = chip->getSubW();
    int const nheight = chip->getSubH();

    x -= chip->getSubX();
    y -= chip->getSubY();

    if (x < 0 || x >= nwidth || y < 0 || y >= nheight)
        return 0;

    auto *pt = reinterpret_cast<uint16_t *>(chip->getFrameBuffer());
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

void SkyRenderer::bleedColumn(INDI::CCDChip *chip, int cx, int cy)
{
    int const nwidth  = chip->getSubW();
    int const nheight = chip->getSubH();
    int const ix      = cx - chip->getSubX();
    int const iy      = cy - chip->getSubY();

    if (ix < 0 || ix >= nwidth || iy < 0 || iy >= nheight)
        return;

    uint16_t *buf    = reinterpret_cast<uint16_t *>(chip->getFrameBuffer());
    uint16_t *center = buf + iy * nwidth + ix;

    if (static_cast<int>(*center) <= m_Cfg.maxVal)
        return;

    int const total = static_cast<int>(*center) - m_Cfg.maxVal;
    *center = static_cast<uint16_t>(m_Cfg.maxVal);

    int carry = (total + 1) / 2;
    for (int py = iy - 1; py >= 0 && carry > 0; py--)
    {
        uint16_t *p = buf + py * nwidth + ix;
        int newval  = static_cast<int>(*p) + carry;
        if (newval > m_Cfg.maxVal)
        {
            carry  = newval - m_Cfg.maxVal;
            newval = m_Cfg.maxVal;
        }
        else
        {
            carry = 0;
        }
        if (newval > m_MaxPix) m_MaxPix = newval;
        *p = static_cast<uint16_t>(newval);
    }

    carry = total / 2;
    for (int py = iy + 1; py < nheight && carry > 0; py++)
    {
        uint16_t *p = buf + py * nwidth + ix;
        int newval  = static_cast<int>(*p) + carry;
        if (newval > m_Cfg.maxVal)
        {
            carry  = newval - m_Cfg.maxVal;
            newval = m_Cfg.maxVal;
        }
        else
        {
            carry = 0;
        }
        if (newval > m_MaxPix) m_MaxPix = newval;
        *p = static_cast<uint16_t>(newval);
    }
}

int SkyRenderer::drawImageStar(INDI::CCDChip *chip, float mag, float x, float y, float exp_s)
{
    if (m_ImageScaleX <= 0.0f || m_ImageScaleY <= 0.0f ||
            !std::isfinite(m_ImageScaleX) || !std::isfinite(m_ImageScaleY))
        return 0;

    int drew = 0;

    int const subX = chip->getSubX();
    int const subY = chip->getSubY();
    int const subW = chip->getSubW() + subX;
    int const subH = chip->getSubH() + subY;

    static constexpr float kMaxStarInfluencePx = 100.0f;
    if (x < subX - kMaxStarInfluencePx || x > subW + kMaxStarInfluencePx ||
            y < subY - kMaxStarInfluencePx || y > subH + kMaxStarInfluencePx)
        return 0;

    float const apertureMM = m_Cfg.apertureMM;
    float const apertureScale = (!std::isnan(apertureMM) && apertureMM > 0.0f)
                                ? (apertureMM / m_Cfg.refApertureMM) * (apertureMM / m_Cfg.refApertureMM)
                                : 1.0f;

    float const totalFlux = static_cast<float>(flux(mag) * exp_s * apertureScale);

    float const pixel_area = m_ImageScaleX * m_ImageScaleY;
    if (!std::isfinite(pixel_area) || pixel_area <= 0.0f)
        return 0;

    float const beta     = 2.5f;
    float const betaTerm = 4.0f * (std::pow(2.0f, 1.0f / beta) - 1.0f);

    float fwhm2 = m_Cfg.seeing * m_Cfg.seeing;
    if (!std::isnan(apertureMM) && apertureMM >= kMinApertureMM)
    {
        float const fwhm_diff = 1.22f * kCenterWavelengthMM / apertureMM * 206265.0f;
        fwhm2 += fwhm_diff * fwhm_diff;
    }

    float const minFwhm  = std::min(m_ImageScaleX, m_ImageScaleY);
    float const minFwhm2 = minFwhm * minFwhm;
    if (!std::isfinite(fwhm2) || fwhm2 < minFwhm2)
        fwhm2 = minFwhm2;

    float const alpha2     = fwhm2 / betaTerm;
    float const moffatNorm = (beta - 1.0f) / (float(M_PI) * alpha2);

    float const threshold = std::max(moffatNorm * totalFlux * pixel_area * 2.0f, 1.0f);
    float const r2_max    = alpha2 * (std::pow(threshold, 1.0f / beta) - 1.0f);
    int   const moffatBox = (r2_max > 0.0f)
                            ? static_cast<int>(std::sqrt(r2_max) / std::min(m_ImageScaleX, m_ImageScaleY)) + 1
                            : static_cast<int>(3.0f * m_Cfg.seeing / std::min(m_ImageScaleX, m_ImageScaleY)) + 1;
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
                if (addToPixel(chip, ix + sx, iy + sy, fp) != 0)
                    drew = 1;
            }
        }
    }

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
                addToPixel(chip, ix + sx, iy + sy, adu);
            }
        }
    }

    int const bleedBox = static_cast<int>(3.0f * m_Cfg.seeing / std::min(m_ImageScaleX, m_ImageScaleY)) + 1;
    for (int sx = -bleedBox; sx <= bleedBox; sx++)
        bleedColumn(chip, ix + sx, iy);

    if (m_Cfg.diffractionSpikes && totalFlux > 500.0f)
    {
        float const spikeCoeff = 0.0002f * totalFlux;
        int   const maxDist    = std::max(chip->getSubW(), chip->getSubH());
        float const cosT = std::cos(m_Cfg.cameraTheta);
        float const sinT = std::sin(m_Cfg.cameraTheta);
        for (int d = 1; d <= maxDist; d++)
        {
            int const adu = static_cast<int>(spikeCoeff / (d * d));
            if (adu < 1)
                break;
            float const fd = static_cast<float>(d);
            int const dx1 = static_cast<int>(std::round(fd * cosT));
            int const dy1 = static_cast<int>(std::round(fd * sinT));
            addToPixel(chip, ix + dx1, iy + dy1, adu);
            addToPixel(chip, ix - dx1, iy - dy1, adu);
            int const dx2 = static_cast<int>(std::round(-fd * sinT));
            int const dy2 = static_cast<int>(std::round( fd * cosT));
            addToPixel(chip, ix + dx2, iy + dy2, adu);
            addToPixel(chip, ix - dx2, iy - dy2, adu);
        }
    }

    return drew;
}

void SkyRenderer::applyReadoutNoise(INDI::CCDChip *chip, int bias, int maxNoise)
{
    if (maxNoise <= 0)
        return;

    int const nx = chip->getSubW();
    int const ny = chip->getSubH();
    auto *buf = reinterpret_cast<uint16_t *>(chip->getFrameBuffer());

    for (int y = 0; y < ny; y++)
        for (int x = 0; x < nx; x++)
        {
            int newval = static_cast<int>(buf[y * nx + x]) + bias + (random() % maxNoise);
            if (newval > m_Cfg.maxVal)
                newval = m_Cfg.maxVal;
            buf[y * nx + x] = static_cast<uint16_t>(newval);
        }
}

void SkyRenderer::drawSkyGlow(INDI::CCDChip *chip, float exp_s)
{
    float const skyflux = static_cast<float>(flux(m_Cfg.skyGlow)) * exp_s;

    int const nwidth  = chip->getSubW();
    int const nheight = chip->getSubH();

    auto *pt = reinterpret_cast<uint16_t *>(chip->getFrameBuffer());

    float const vig = std::min(nwidth, nheight) * m_ImageScaleX;
    float const invVig2 = 1.0f / (vig * vig);

    for (int y = 0; y < nheight; y++)
    {
        float const sy = nheight / 2.0f - y;

        for (int x = 0; x < nwidth; x++)
        {
            float const sx = nwidth / 2.0f - x;

            float const dc2 = sx * sx * m_ImageScaleX * m_ImageScaleX
                              + sy * sy * m_ImageScaleY * m_ImageScaleY;

            float const fa = std::exp(-2.0f * 0.7f * dc2 * invVig2);

            float fp = (pt[0] + skyflux) * fa;

            if (fp > m_Cfg.maxVal) fp = static_cast<float>(m_Cfg.maxVal);
            if (fp < pt[0]) fp = pt[0];
            if (fp > m_MaxPix) m_MaxPix = static_cast<int>(fp);
            if (fp < m_MinPix) m_MinPix = static_cast<int>(fp);

            pt[0] = static_cast<uint16_t>(fp);
            pt++;
        }
    }
}

int SkyRenderer::renderFrame(
    INDI::CCDChip *chip,
    double ra_j2000_deg,
    double dec_j2000_deg,
    double focal_length_mm,
    double rotation_deg,
    float  exposure_s,
    bool   renderStars,
    double minSearchRadiusArcmin,
    const ObserverContext *obs)
{
    m_MaxPix = 0;
    m_MinPix = 65000;

    m_ImageScaleX = static_cast<float>((chip->getPixelSizeX() / focal_length_mm) * 206.3);
    m_ImageScaleY = static_cast<float>((chip->getPixelSizeY() / focal_length_mm) * 206.3);

    double const theta = rotation_deg * (M_PI / 180.0);
    double const pprx  = focal_length_mm / chip->getPixelSizeX() * 1000.0;
    double const ppry  = focal_length_mm / chip->getPixelSizeY() * 1000.0;

    double const pa =  pprx * std::cos(theta);
    double const pb =  ppry * std::sin(theta);
    double const pd = -pprx * std::sin(theta);
    double const pe =  ppry * std::cos(theta);
    double const pc =  chip->getXRes() / 2.0;
    double const pf =  chip->getYRes() / 2.0;
    double const ccdW = chip->getXRes();

    // J2000 center -- used only for the GSC catalog query
    double const rar  = ra_j2000_deg  * (M_PI / 180.0);
    double const decr = dec_j2000_deg * (M_PI / 180.0);

#ifdef HAVE_ERFA
    bool useErfa = (obs != nullptr && obs->jd_utc > 0.0);
    eraASTROM astrom {};
    double    frameEo = 0.0;
    if (useErfa)
    {
        double utc1 = std::floor(obs->jd_utc) + 0.5;
        double utc2 = obs->jd_utc - utc1;
        int const j = eraApco13(utc1, utc2, 0.0,
                                obs->lon_rad, obs->lat_rad, obs->alt_m,
                                0.0, 0.0,
                                obs->phpa, obs->tc, obs->rh, obs->wl,
                                &astrom, &frameEo);
        if (j == -1)
            useErfa = false; // unacceptable date -- fall back to J2000
    }
#else
    bool const useErfa = false;
    (void)obs;
#endif
    // Gnomonic projection center: apparent place when ERFA is active, J2000 otherwise
    double const rar_proj  = useErfa ? obs->rar_proj  : rar;
    double const decr_proj = useErfa ? obs->decr_proj : decr;

    double radius = std::sqrt(
                        m_ImageScaleX * m_ImageScaleX * chip->getXRes() / 2.0 * chip->getXRes() / 2.0 +
                        m_ImageScaleY * m_ImageScaleY * chip->getYRes() / 2.0 * chip->getYRes() / 2.0) / 60.0;
    if (minSearchRadiusArcmin > 0 && radius < minSearchRadiusArcmin)
        radius = minSearchRadiusArcmin;

    double lookuplimit = m_Cfg.limitingMag;
    if (radius > 60)
        lookuplimit = 11;

    memset(chip->getFrameBuffer(), 0, chip->getFrameBufferSize());

    int drawn = 0;

    if (renderStars)
    {
        AutoCNumeric locale;
        char gsccmd[250];

        snprintf(gsccmd, sizeof(gsccmd),
                 "gsc -c %8.6f %+8.6f -r %4.1f -m 0 %4.2f -n 3000",
                 range360(ra_j2000_deg),
                 rangeDec(dec_j2000_deg),
                 radius,
                 lookuplimit);

        FILE *pp = popen(gsccmd, "r");
        if (pp != nullptr)
        {
            char line[256];

            while (fgets(line, sizeof(line), pp) != nullptr)
            {
                char  id[20], plate[6], ob[6];
                float ra, dec, pose, mag, mage, dist;
                int   band, c, dir;

                int rc = sscanf(line, "%10s %f %f %f %f %f %d %d %4s %2s %f %d",
                                id, &ra, &dec, &pose, &mag, &mage,
                                &band, &c, plate, ob, &dist, &dir);
                if (rc != 12)
                    continue;

                double srar, sdecr;
#ifdef HAVE_ERFA
                if (useErfa)
                    applyAstrom(ra * (M_PI / 180.0), dec * (M_PI / 180.0),
                                &astrom, frameEo, &srar, &sdecr);
                else
#endif
                {
                    srar  = ra  * (M_PI / 180.0);
                    sdecr = dec * (M_PI / 180.0);
                }

                double const denom = cos(decr_proj) * cos(sdecr) * cos(srar - rar_proj)
                                     + sin(decr_proj) * sin(sdecr);
                if (denom <= 0)
                    continue;

                double const sx = cos(sdecr) * sin(srar - rar_proj) / denom;
                double const sy = (sin(decr_proj) * cos(sdecr) * cos(srar - rar_proj)
                                   - cos(decr_proj) * sin(sdecr)) / denom;

                double const ccdx = ccdW - (pa * sx + pb * sy + pc);
                double const ccdy =          pd * sx + pe * sy + pf;

                drawn += drawImageStar(chip, mag,
                                       static_cast<float>(ccdx),
                                       static_cast<float>(ccdy),
                                       exposure_s);
            }
            pclose(pp);
        }
        else
        {
            drawn = -1;
        }

        // Bright star supplement: GSC omits stars brighter than ~mag 6.5.
        if (drawn >= 0)
        {
            for (int bsi = 0; bsi < s_BrightStarsCount; ++bsi)
            {
                float const bsRaDeg  = s_BrightStars[bsi].ra;
                float const bsDecDeg = s_BrightStars[bsi].dec;
                float const bsMag    = s_BrightStars[bsi].mag;

                double srar, sdecr;
#ifdef HAVE_ERFA
                if (useErfa)
                    applyAstrom(bsRaDeg * (M_PI / 180.0), bsDecDeg * (M_PI / 180.0),
                                &astrom, frameEo, &srar, &sdecr);
                else
#endif
                {
                    srar  = bsRaDeg  * (M_PI / 180.0);
                    sdecr = bsDecDeg * (M_PI / 180.0);
                }

                double const denom = cos(decr_proj) * cos(sdecr) * cos(srar - rar_proj)
                                     + sin(decr_proj) * sin(sdecr);
                if (denom <= 0)
                    continue;

                double const sx = cos(sdecr) * sin(srar - rar_proj) / denom;
                double const sy = (sin(decr_proj) * cos(sdecr) * cos(srar - rar_proj)
                                   - cos(decr_proj) * sin(sdecr)) / denom;

                double const ccdx = ccdW - (pa * sx + pb * sy + pc);
                double const ccdy =          pd * sx + pe * sy + pf;

                drawn += drawImageStar(chip, bsMag,
                                       static_cast<float>(ccdx),
                                       static_cast<float>(ccdy),
                                       exposure_s);
            }
        }
    }

    drawSkyGlow(chip, exposure_s);

    return drawn;
}
