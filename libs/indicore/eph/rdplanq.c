#include "eph.h"
#include <sofa.h>

int ephRdplanq ( double pvgm[2][3], double pvsb[2][3], ephPLANctx* cplan,
                 double ut1, double tdb, int np,
                 double elong, double phi, double hm,
                 double* rast, double* dast,
                 double* rapp, double* dapp, double* eo, double* diam )
/*
**  - - - - - - - - - - -
**   e p h R d p l a n q
**  - - - - - - - - - - -
**
**  Topocentric apparent RA,Dec of a planet (also Sun & Moon), and its
**  angular diameter.  This "quick" version starts with precomputed
**  position+velocity vectors for geocenter-to-Moon and Sun-to-EMB
**  respectively, so that computationally expensive ephemeris
**  calculations involve the target body alone.
**
**  Given:
**     pvgm       double[2][3] Earth-to-Moon PV (ICRS, au, au/s, Note 2)
**     pvsb       double[2][3] Sun-to-EMB PV (ICRS, au, au/s, Note 2)
**     cplan      ephPLANctx*  context for planet ephemeris (Notes 3-5)
**     ut1        double       UT1 (MJD, Note 6)
**     tdb        double       TDB (MJD, Note 6)
**     np         int          body: 1 = Mercury
**                                   2 = Venus
**                                   3 = Moon
**                                   4 = Mars
**                                   5 = Jupiter
**                                   6 = Saturn
**                                   7 = Uranus
**                                   8 = Neptune
**                                else = Sun
**     elong      double       observer east longitude (radians)
**     phi        double       observer geodetic latitude (radians)
**     hm         double       observer height above sea level (meters)
**
**  Returned:
**     rast,dast  double*      RA, Dec (ICRS astrometric, radians)
**     rapp,dapp  double*      RA, Dec (topocentric apparent, radians)
**     eo         double*      equation of the origins (radians, Note 8)
**     diam       double*      angular diameter (equatorial, radians)
**
**  Returned (function value):
**                int          status: +1 = warning: date outside 1-4000
**                                      0 = OK
**                                     -1 = uninitialized context
**
**  Defined in eph.h:
**     DAYSEC        seconds per day
**     AUKM          au in km
**     TAU           light time for unit distance (sec)
**     RMME          mass ratio Moon/Earth
**     ephPLANctx    planetary ephemeris context
**
**  Notes:
**
**  1  The ephemerides are calculated using the ELP/MPP02+VSOP2010
**     theories.  For the 21st century the results agree with JPL DE405
**     to the following geocentric accuracies:
**
**                       RMS          worst
**
**        Sun           0.243         0.559
**        Mercury       0.265         0.858
**        Venus         0.374         3.352
**        Moon          4.998         9.651
**        Mars          2.428        16.957
**        Jupiter       1.355         2.789
**        Saturn        1.435         3.294
**        Uranus        1.426         3.543
**        Neptune       1.433         2.916
**
**                       mas           mas
**
**     Comparisons with JPL DE430 are as follows:
**
**        Sun           1.329         1.841
**        Mercury       2.810         6.883
**        Venus         1.648         4.553
**        Moon         11.048        16.561
**        Mars          3.882        19.072
**        Jupiter      82.013       177.075
**        Saturn       92.820       164.302
**        Uranus      337.295       555.496
**        Neptune    1065.73       1883.56
**
**  2  The geocenter-to-Moon and Sun-to-EMB PV vectors pvgm and pvsb
**     can be computed by calling the functions ephMoon and ephPlanet
**     respectively.
**
**  3  The context argument cplan points to tables of constants that are
**     used by the planetary ephemeris function ephPlanet.  Before the
**     present function is called, this context must be populated, by
**     calling the function ephPlanc.
**
**  4  It is the caller's responsibility to populate the cplan context
**     for the same planet that is identified by the argument np.
**
**  5  For np = 3 (Moon) or np = else (Sun), cplan is not used and can
**     be left uninitialized.  For np = 1 or 2 or 4-9, cplan must be
**     populated.
**
**  6  The date of observation must be supplied in two forms, namely
**     UT1 and TDB, both in the form of Modified Julian Date
**     (JD-2400000.5).  Depending on the application, liberties can be
**     taken:  usually TT will do instead of TDB and UTC instead of UT1,
**     and in some very low-precision applications UTC might be adequate
**     for both.  The particular need for Earth-rotation time as opposed
**     to what used to be called "ephemeris time" is for the accurate
**     computation of lunar geocentric parallax.
**
**  7  The observer location coordinates (elongm, phi, hm) allow
**     correction for geocentric parallax.  This is a major effect for
**     the Moon, but its effect on planetary positions is small
**     (for some applications negligible, especially for the outer
**     planets).  Geocentric positions can be generated by specifying
**     hm below -1000.0, for example "-1e6".
**
**  8  The "current" RA/Decs returned are topocentric apparent.  For
**     topocentric intermediate add the equation of the origins, eo.
**
**  9  See also the function ephRdplan, which does not need precomputed
**     Moon and EMB vectors but consumes more computing power.
**
**  Called:  ephPlanet, iauCp, iauSxp, iauPmp, iauZp, iauApco13,
**           iauEra00, iauPvtob, iauSxpv, iauZpv, iauTrxp, iauPm,
**           iauPpp, iauC2s, iauAnp, iauAtciqz
**
**  Last revision:   2022 October 30
**
**  Author P.T.Wallace - see license notice at end.
*/
{

/* Vectors (pos and pos+vel), all au/au/s and ICRS */
   double pvsg[2][3],   /* Sun to geocenter */
          pgt[3],       /* geocenter to target */
          pvst[2][3],   /* Sun to target */
          pvgo[2][3],   /* geocenter to observer */
          pot[3];       /* observer to target */

/* ICRS to CIRS parameters */
   iauASTROM astrom;

/* Radii (km, mostly equatorial, various sources) */
   static double rau[] = { 695660.0,           /* Sun     */
                             2440.53,          /* Mercury */
                             6051.8,           /* Venus   */
                             1737.4,           /* Moon    */
                             3396.19,          /* Mars    */
                            71492.0,           /* Jupiter */
                            60268.0,           /* Saturn  */
                            25559.0,           /* Uranus  */
                            24764.0 };         /* Neptune */

/* Mass of Moon divided by mass of Earth+Moon */
   const double RMMEPM = RMME/(1.0+RMME);

   int ip, j;
   double p[3], era, tl, ra, dec;


/* Preset results to error values. */
   *rast = *dast = *rapp = *dapp = *eo = *diam = 0.0;

/* Sun to geocenter. */
   iauSxpv ( RMMEPM, pvgm, pvsg );
   iauPvmpv ( pvsb, pvsg, pvsg );

/* Classify np. */
   ip = ( np >= 1 && np <= 8 ) ? np : 0;

/* Identify target. */
   if ( ip == 3 ) {

   /* Moon:  geocenter to target. */
      iauCp ( pvgm[0], pgt );

   } else if ( ip == 0 ) {

   /* Sun: geocenter to target. */
      iauZp ( p );
      iauPmp ( p, pvsg[0], pgt );

   } else {

   /* No: Sun to target. */
      j = ephPlanet ( ip, cplan, tdb, pvst );
      if ( j ) return j;

   /* Geocenter to target. */
      iauPmp ( pvst[0], pvsg[0], pgt );
   }

/* ICRS to topocentric parameters (neglecting polar motion). */
   iauApco13 ( DJM0, ut1, 0.0, elong, phi, hm,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, &astrom, eo );

/* Geocenter to observer (equator of date, CIO). */
   if ( hm > -1000.0 ) {
      era = iauEra00(DJM0,ut1);
      iauPvtob ( elong, phi, hm, 0.0, 0.0, 0.0, era, pvgo );
      iauSxpv ( 1e-3/AUKM, pvgo, pvgo );
   } else {
      iauZpv ( pvgo );
   }

/* Rotate geocenter-to-observer position from intermediate to ICRS. */
   iauTrxp ( astrom.bpn, pvgo[0], pvgo[0] );

/* Observer to target body (topocentric ICRS). */
   iauPmp ( pgt, pvgo[0], pot );

/* If not Sun we need to allow for target motion during light time. */
   if ( ip ) {

   /* Light time (sec). */
      tl = TAU * iauPm(pot);

      /* Moon or planet? */
      if ( ip == 3 ) {

      /* Moon:  apply the change in Sun-to-Moon coordinates. */
         iauPpp ( pvsg[1], pvgm[1], p );
         iauSxp ( tl, p, p );
         iauPmp ( pot, p, pot );

      } else {

      /* No:  for planet use a fresh prediction. */
         j = ephPlanet ( ip, cplan, tdb-tl/DAYSEC, pvst );
         if ( j ) return j;

      /* Apply the change in Sun-to-planet coordinates. */
         iauPmp ( pvst[0], pvsg[0], p );
         iauPmp ( p, pvgo[0], pot );
      }
   }

/* ICRS astrometric place. */
   iauC2s ( pot, &ra, &dec );
   *rast = iauAnp(ra);
   *dast = dec;

/* Topocentric apparent place. */
   iauAtciqz ( ra, dec, &astrom, &ra, &dec );
   *rapp = iauAnp(ra-*eo);
   *dapp = dec;

/* Angular diameter (radians). */
   *diam = 2.0*asin(rau[ip]/(iauPm(pot)*AUKM));

/* Success. */
   return 0;
}

/*----------------------------------------------------------------------
**
**  Copyright (C) 2022 by P.T.Wallace
**
**  Permission to use, copy, modify, and/or distribute this software for
**  any purpose with or without fee is hereby granted.
**
**  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
**  WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
**  WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
**  AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
**  CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS
**  OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
**  NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
**  CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
**
**--------------------------------------------------------------------*/
