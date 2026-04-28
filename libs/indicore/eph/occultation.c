#include "eph.h"
#include <sofa.h>
#include <stdio.h>
#include <stdlib.h>

/*
**  Mars occultation, 2020 February 18.
**
**  Any command-line argument is used as the path for the (binary)
**  ephemeris files.
**
**  Latest revision:   2020 October 9
**
**  Author P.T.Wallace - see license notice at end.
*/

int main ( int nargs, char* argv[] )
{

/* Lunar and planetary ephemeris contexts */
   ephMOONctx* cmoon = malloc(sizeof(ephMOONctx));
   ephPLANctx* cemb = malloc(sizeof(ephPLANctx));
   ephPLANctx* cplanet = malloc(sizeof(ephPLANctx));

   char s;
   int j, is, iy, im, id, i4[4];
   double elong, phi, hm, w, utc0, fdutc, utc, tai, tt, pv[2][3], u, v,
          dtr, tdb, rast, dast, eo, ramoon, dcmoon, dmoon, rmoon,
          ramars, dcmars, dmars, rmars, sep;


   (void) nargs;

/* Site (Glen D. Riley Observatory, near Chicago). */
   j = iauAf2a ( '-', 88, 9, 34.0, &elong );
   if ( j ) {
      printf ( "iauAf2a error %d\n", j );
      return 1;
   }
   j = iauAf2a ( '+', 41, 42, 2.0, &phi );
   if ( j ) {
      printf ( "iauAf2a error %d\n", j );
      return 1;
   }
   hm = 197.0;

/* Start UTC (2020 February 18, 12:04:00) to Modified Julian Date. */
   j = iauCal2jd ( 2020, 2, 18, &w, &utc0 );
	j = iauTf2d ( '+', 12,  4, 0.0, &fdutc );
   utc0 += fdutc;

/* Populate the Moon, EMB and Mars contexts. */
   j = ephMoonc ( argv[1], 2, cmoon );
   if ( j ) {
      printf ( "ephMoonc error %d\n", j );
      return 1;
   }
   j = ephPlanc ( 3, argv[1], cemb );
   if ( j ) {
      printf ( "ephPlanc error %d\n", j );
      return 1;
   }
   j = ephPlanc ( 4, argv[1], cplanet );
   if ( j ) {
      printf ( "ephPlanc error %d\n", j );
      return 1;
   }

/* Every half second for half a minute. */
   for ( is = 0; is <= 60; is++ ) {

   /* UTC. */
      utc = utc0 + ((double) is)/86400.0/2.0;

   /* TAI. */
      j = iauUtctai ( DJM0, utc, &w, &tai );
      if ( j ) {
         printf ( "iauUtctai error %d\n", j );
         return 1;
      }

   /* TT. */
      j = iauTaitt ( w, tai, &w, &tt );
      if ( j ) {
         printf ( "iauTaitt error %d\n", j );
         return 1;
      }

   /* Observer geocentric coordinates (m, m/s). */
      iauPvtob ( elong, phi, hm, 0.0, 0.0, 0.0, 0.0, pv );

   /* Distances from axis and equator (km). */
      u = sqrt(pv[0][0]*pv[0][0]+pv[0][1]*pv[0][1]) / 1e3;
      v = pv[0][2] / 1e3;

   /* TDB-TT (sec). */
      dtr = iauDtdb ( w, tt, fmod(utc,1.0), elong, u, v );

   /* TDB. */
      j = iauTttdb ( w, tt, dtr, &w, &tdb );
      if ( j ) {
         printf ( "iauTttdb error %d\n", j );
         return 1;
      }

   /* Calculate the apparent RA,Dec and size of the Moon. */
      j = ephRdplan ( cmoon, cemb, cplanet, utc, tdb, 3, elong, phi, hm,
                      &rast, &dast, &ramoon, &dcmoon, &eo, &dmoon );
      if ( j ) {
         printf ( "ephRdplan error %d\n", j );
         return 1;
      }
      rmoon = dmoon/2.0;

   /* Calculate the apparent RA,Dec and size of Mars. */
      j = ephRdplan ( cmoon, cemb, cplanet, utc, tdb, 4, elong, phi, hm,
                      &rast, &dast, &ramars, &dcmars, &eo, &dmars );
      if ( j ) {
         printf ( "ephRdplan error %d\n", j );
         return 1;
      }
      rmars = dmars/2.0;

   /* Separation between Moon centre and Mars centre. */
      sep = iauSeps ( ramoon, dcmoon, ramars, dcmars );

   /* Report. */
      j = iauJd2cal ( DJM0, utc+0.05/86400.0, &iy, &im, &id, &fdutc );
      iauD2tf ( 3, fdutc-0.05/86400.0, &s, i4 );
      printf ( " %4i/%2.2i/%2.2i  %2.2d:%2.2d:%2.2d.%3.3d UTC  ",
               iy, im, id, i4[0], i4[1], i4[2], i4[3] );
      if ( sep > rmoon+rmars ) {
         printf ( "clear of Moon\n" );
      } else if ( sep < rmoon-rmars ) {
         printf ( "completely hidden\n" );
      } else if ( sep > rmoon ) {
         printf ( "mostly visible\n" );
      } else {
         printf ( "mostly hidden\n" );
      }

   /* Next time. */
   }

/* Success. */
   return 0;
}

/*----------------------------------------------------------------------
**
**  Copyright (C) 2021 by P.T.Wallace
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
