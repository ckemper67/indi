#include "simulator_base.h"
#include <cmath>
#include <sys/time.h>

double SimulatorBase::flux(double mag) const
{
    double const z = m_LimitingMag;
    double const k = 2.5 * log10(m_MaxVal) / (m_LimitingMag - m_SaturationMag);
    return pow(10, (z - mag) * k / 2.5);
}

int SimulatorBase::DrawImageStar(INDI::CCDChip *targetChip, float mag, float x, float y, float exposure_time)
{
    int drew = 0;

    int const subX = targetChip->getSubX();
    int const subY = targetChip->getSubY();
    int const subW = targetChip->getSubW() + subX;
    int const subH = targetChip->getSubH() + subY;

    if ((x < subX) || (x > subW) || (y < subY) || (y > subH))
        return 0;

    float const totalFlux = static_cast<float>(flux(mag) * exposure_time);

    int const boxsizey = static_cast<int>(3.0f * m_Seeing / m_ImageScaleY) + 1;

    // Gaussian PSF: f(r) = 1/(sigma*sqrt(2*pi)) * exp(-r^2 / (2*sigma^2))
    // sigma relates to FWHM by: FWHM = 2*sqrt(2*ln2)*sigma
    float const sigma = m_Seeing / (2.0f * std::sqrt(2.0f * std::log(2.0f)));

    for (int sy = -boxsizey; sy <= boxsizey; sy++)
    {
        for (int sx = -boxsizey; sx <= boxsizey; sx++)
        {
            float const dc2 = sx * sx * m_ImageScaleX * m_ImageScaleX
                            + sy * sy * m_ImageScaleY * m_ImageScaleY;
            float const fa  = (1.0f / (sigma * std::sqrt(2.0f * float(M_PI))))
                            * std::exp(-dc2 / (2.0f * sigma * sigma));
            int const fp = static_cast<int>(fa * totalFlux);
            if (fp > 0)
            {
                if (AddToPixel(targetChip, static_cast<int>(x) + sx,
                               static_cast<int>(y) + sy, fp) != 0)
                    drew = 1;
            }
        }
    }
    return drew;
}

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
    if (newval > m_MaxVal)
        newval = m_MaxVal;
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
