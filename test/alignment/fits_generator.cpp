/*******************************************************************************
 * fits_generator.cpp
 *
 * Uses query-starkd (from astrometry.net) to get star positions from the
 * installed index files, then renders them into a synthetic FITS image.
 * This replaces the GSC dependency used by ccd_simulator.cpp.
 ******************************************************************************/

#include "fits_generator.h"

#include <fitsio.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

static constexpr double DEG2RAD = M_PI / 180.0;

// Search these paths for astrometry.net index files, in order.
static const char *INDEX_SEARCH_PATHS[] = {
    "/opt/homebrew/share/astrometry",
    "/usr/local/share/astrometry",
    "/usr/share/astrometry",
    nullptr
};

// Preferred index files for a ~5° field, from finest to coarsest.
static const char *INDEX_NAMES[] = {
    "index-4110.fits",
    "index-4111.fits",
    nullptr
};

// Find the first readable index file from the lists above.
static bool findIndexFile(char *out, size_t out_size)
{
    for (const char **dir = INDEX_SEARCH_PATHS; *dir; ++dir)
    {
        for (const char **name = INDEX_NAMES; *name; ++name)
        {
            snprintf(out, out_size, "%s/%s", *dir, *name);
            if (FILE *f = fopen(out, "r"))
            {
                fclose(f);
                return true;
            }
        }
    }
    return false;
}

bool generateFITS(double ra_deg, double dec_deg,
                  double pixel_scale_arcsec,
                  int width, int height,
                  const char *output_path)
{
    char index_path[512];
    if (!findIndexFile(index_path, sizeof(index_path)))
        return false;

    // Pixels per radian
    const double ppr = (180.0 / M_PI) * 3600.0 / pixel_scale_arcsec;

    // Field half-diagonal in degrees (radius for query-starkd)
    const double radius_deg =
        std::hypot(width * pixel_scale_arcsec, height * pixel_scale_arcsec) / 2.0 / 3600.0;

    // Boresight in radians
    const double rar  = ra_deg  * DEG2RAD;
    const double decr = dec_deg * DEG2RAD;

    // Allocate pixel buffer (16-bit unsigned, initialised to 300 ADU sky background)
    std::vector<uint16_t> buf(width * height, 300);

    // Query star positions from the astrometry.net index file.
    // Output format: "RA, Dec, MAG_BT, MAG_VT, MAG_HP, MAG" (lines starting
    // with '#' are comments).
    char cmd[1024];
    snprintf(cmd, sizeof(cmd),
             "query-starkd -r %.6f -d %.6f -R %.3f -T %s",
             ra_deg, dec_deg, radius_deg, index_path);

    FILE *pp = popen(cmd, "r");
    if (pp == nullptr)
        return false;

    int drawn = 0;
    char line[256];
    while (fgets(line, sizeof(line), pp) != nullptr)
    {
        if (line[0] == '#' || line[0] == '\n')
            continue;

        double star_ra, star_dec;
        float  mag_bt, mag_vt, mag_hp, mag;
        // "RA, Dec, MAG_BT, MAG_VT, MAG_HP, MAG"
        if (sscanf(line, "%lf, %lf, %f, %f, %f, %f",
                   &star_ra, &star_dec,
                   &mag_bt, &mag_vt, &mag_hp, &mag) != 6)
            continue;

        // Gnomonic projection — equations 9.1 / 9.2 from
        // Handbook of Astronomical Image Processing (Berry & Burnell).
        // Same formulas as ccd_simulator.cpp lines 789-792.
        const double srar  = star_ra  * DEG2RAD;
        const double sdecr = star_dec * DEG2RAD;
        const double denom = cos(decr) * cos(sdecr) * cos(srar - rar) + sin(decr) * sin(sdecr);
        if (denom <= 0.0)
            continue;

        const double sx = cos(sdecr) * sin(srar - rar) / denom;
        const double sy = (sin(decr) * cos(sdecr) * cos(srar - rar) - cos(decr) * sin(sdecr)) / denom;

        // Convert to pixel coordinates with horizontal flip (East to the left),
        // matching the orientation produced by ccd_simulator.cpp line 799.
        const double ccdx = width  - (ppr * sx + width  / 2.0);
        const double ccdy =           ppr * sy + height / 2.0;

        // Gaussian PSF: sigma = 1.5 pixels.
        // Amplitude uses a 10x-compressed magnitude scale so that even the
        // brightest Tycho-2 star in the field (mag ~2.86 in Pleiades) stays
        // well below the 16-bit cap.  This ensures simplexy sees a proper
        // brightness ordering: 3000 ADU (mag 11) → ~20 000 ADU (mag 3).
        // Uncompressed photometry (30000 * 10^-((mag-8)/2.5)) saturates bright
        // stars which produces identical FLUX values in the AXY catalogue,
        // breaking the solver's source-ranking and spawning false detections.
        const double sigma  = 1.5;
        const double amp    = 3000.0 * std::pow(10.0, (11.0 - mag) / 10.0);
        const int    radius = static_cast<int>(4.0 * sigma + 1.0);

        const int cx = static_cast<int>(ccdx + 0.5);
        const int cy = static_cast<int>(ccdy + 0.5);

        for (int dy = -radius; dy <= radius; ++dy)
        {
            for (int dx = -radius; dx <= radius; ++dx)
            {
                const int px = cx + dx;
                const int py = cy + dy;
                if (px < 0 || px >= width || py < 0 || py >= height)
                    continue;

                const double r2  = dx * dx + dy * dy;
                const double val = amp * std::exp(-r2 / (2.0 * sigma * sigma));
                const uint32_t pix = static_cast<uint32_t>(buf[py * width + px])
                                   + static_cast<uint32_t>(val);
                buf[py * width + px] = static_cast<uint16_t>(pix > 65535u ? 65535u : pix);
            }
        }
        ++drawn;
    }
    pclose(pp);

    if (drawn == 0)
        return false;

    // Write FITS file via cfitsio
    fitsfile *fptr = nullptr;
    int status = 0;

    // Prefix with '!' so cfitsio overwrites any existing file.
    char fits_path[512];
    snprintf(fits_path, sizeof(fits_path), "!%s", output_path);

    fits_create_file(&fptr, fits_path, &status);
    if (status != 0)
        return false;

    long naxes[2] = { width, height };
    fits_create_img(fptr, USHORT_IMG, 2, naxes, &status);
    fits_write_img(fptr, TUSHORT, 1, (long)(width * height), buf.data(), &status);
    fits_close_file(fptr, &status);
    return status == 0;
}
