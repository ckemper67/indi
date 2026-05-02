#include "eph.h"
#include <sofa.h>

int ephRdtop ( double pvgm[2][3], double pvsb[2][3], ephTOPctx* ctop,
               double ut1, double tdb, int np,
               double elong, double phi, double hm,
               double* rast, double* dast,
               double* rapp, double* dapp, double* eo, double* diam )
/*
**  - - - - - - - - -
**   e p h R d t o p
**  - - - - - - - - -
**
**  Topocentric apparent RA,Dec of an outer planet using TOP2013.
**
**  This function mirrors ephRdplanq but uses ephTopPlanet (TOP2013)
**  instead of ephPlanet (VSOP2013) for the target body.
**
**  Given:
**     pvgm       double[2][3]  Earth-to-Moon PV (ICRS, au, au/s)
**     pvsb       double[2][3]  Sun-to-EMB PV (ICRS, au, au/s)
**     ctop       ephTOPctx*    TOP2013 context for the target body
**     ut1        double        UT1 (MJD)
**     tdb        double        TDB (MJD)
**     np         int           body: 5=Jupiter .. 8=Neptune
**     elong      double        observer east longitude (radians)
**     phi        double        observer geodetic latitude (radians)
**     hm         double        observer height above sea level (meters)
**
**  Returned:
**     rast,dast  double*       RA, Dec (ICRS astrometric, radians)
**     rapp,dapp  double*       RA, Dec (topocentric apparent, radians)
**     eo         double*       equation of the origins (radians)
**     diam       double*       angular diameter (equatorial, radians)
**
**  Returned (function value):
**             int              status: +1 = date outside 1-4000 CE
**                                       0 = OK
**                                      <0 = uninitialized / wrong body
**
**  Reference:
**     J.-L. Simon et al., A&A 557, A49 (2013).
*/
{
   double pvsg[2][3],    /* Sun to geocenter */
          pgt[3],        /* geocenter to target */
          pvst[2][3],    /* Sun to target */
          pvgo[2][3],    /* geocenter to observer */
          pot[3];        /* observer to target */

   iauASTROM astrom;

/* Radii (km, equatorial) for bodies 5-8 */
   static double rau_top[] = {
       71492.0,   /* Jupiter (np=5) */
       60268.0,   /* Saturn  (np=6) */
       25559.0,   /* Uranus  (np=7) */
       24764.0    /* Neptune (np=8) */
   };

   const double RMMEPM = RMME / (1.0 + RMME);

   int j;
   double p[3], era, tl, ra, dec;

/* Preset results. */
   *rast = *dast = *rapp = *dapp = *eo = *diam = 0.0;

   if ( np < 5 || np > 8 ) return -2;

/* Sun to geocenter (from Moon and EMB vectors). */
   iauSxpv ( RMMEPM, pvgm, pvsg );
   iauPvmpv ( pvsb, pvsg, pvsg );

/* Sun to target (TOP2013 heliocentric position). */
   j = ephTopPlanet ( np, ctop, tdb, pvst );
   if ( j < 0 ) return j;

/* Geocenter to target. */
   iauPmp ( pvst[0], pvsg[0], pgt );

/* ICRS to topocentric parameters (neglecting polar motion). */
   iauApco13 ( DJM0, ut1, 0.0, elong, phi, hm,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, &astrom, eo );

/* Geocenter to observer (equator of date, CIO). */
   if ( hm > -1000.0 ) {
      era = iauEra00(DJM0, ut1);
      iauPvtob ( elong, phi, hm, 0.0, 0.0, 0.0, era, pvgo );
      iauSxpv ( 1e-3/AUKM, pvgo, pvgo );
   } else {
      iauZpv ( pvgo );
   }

/* Rotate geocenter-to-observer from intermediate to ICRS. */
   iauTrxp ( astrom.bpn, pvgo[0], pvgo[0] );

/* Observer to target (topocentric ICRS). */
   iauPmp ( pgt, pvgo[0], pot );

/* Light-time correction: fresh TOP2013 prediction. */
   tl = TAU * iauPm(pot);
   j = ephTopPlanet ( np, ctop, tdb - tl/DAYSEC, pvst );
   if ( j < 0 ) return j;

   iauPmp ( pvst[0], pvsg[0], p );
   iauPmp ( p, pvgo[0], pot );

/* ICRS astrometric place. */
   iauC2s ( pot, &ra, &dec );
   *rast = iauAnp(ra);
   *dast = dec;

/* Topocentric apparent place. */
   iauAtciqz ( ra, dec, &astrom, &ra, &dec );
   *rapp = iauAnp(ra - *eo);
   *dapp = dec;

/* Angular diameter. */
   *diam = 2.0 * asin(rau_top[np-5] / (iauPm(pot) * AUKM));

   return j;   /* 0 = OK, +1 = date warning */
}

/*----------------------------------------------------------------------
**
**  Copyright (C) 2025 INDI Contributors
**
**  Permission to use, copy, modify, and/or distribute this software for
**  any purpose with or without fee is hereby granted.
**
**--------------------------------------------------------------------*/
