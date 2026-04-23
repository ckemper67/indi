/*******************************************************************************
 * fits_generator.h
 *
 * Generates a synthetic FITS image populated with real GSC stars, suitable
 * for plate-solving with astrometry.net.  Used by the pointing-model
 * integration test to produce images centred on a known sky position without
 * requiring a running INDI server.
 ******************************************************************************/

#pragma once

/**
 * Generate a synthetic FITS image centred on (ra_deg, dec_deg).
 *
 * Stars are queried from the GSC via the system `gsc` binary (same command
 * format as CCDSim::DrawCcdFrame).  Each star is rendered as a 2-D Gaussian
 * PSF with σ = 1.5 pixels.  The image is written as a 16-bit grayscale FITS
 * file using cfitsio.
 *
 * @param ra_deg            Field centre RA  in decimal degrees (J2000)
 * @param dec_deg           Field centre Dec in decimal degrees (J2000)
 * @param pixel_scale_arcsec  Image scale in arcsec/pixel
 * @param width             Image width  in pixels
 * @param height            Image height in pixels
 * @param output_path       File path for the output FITS file
 * @return true on success (at least one star was drawn); false otherwise.
 */
bool generateFITS(double ra_deg, double dec_deg,
                  double pixel_scale_arcsec,
                  int width, int height,
                  const char *output_path);
