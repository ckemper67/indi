#include "eph.h"
#include <sofa.h>

int ephEarth ( double date, ephMOONctx* cmoon, ephPLANctx* cemb,
               double pv[2][3] )
/*
**  - - - - - - - - -
**   e p h E a r t h
**  - - - - - - - - -
**
**  Heliocentric ICRS position and velocity of the Earth.
**
**  Given:
**     date   double         date, TDB Modified Julian Date (Note 1)
**     cmoon  ephMOONctx*    context for Moon (Note 2)
**     cemb   ephPLANctx*    context for EMB (Note 2)
**
**  Returned:
**     pv     double[2][3]   Earth position & velocity (Note 3)
**
**  Returned (function value):
**            int            status: 0 = OK
**                                else = ephemeris problem
**
**  Defined in eph.h:
**     ephMOONctx    lunar ephemeris context
**     ephPLANctx    planetary ephemeris context
**     EP2000        J2000.0 MJD
**     RMME          mass ratio Moon/Earth
**
**  Notes:
**
**  1  The date is TDB as an MJD (=JD-2400000.5).  TT can be used
**     instead of TDB in most applications.
**
**  2  The context arguments point to tables of constants that are used
**     by the lunar and planetary ephemeris functions ephMoon and
**     ephPlanet.  Before the present function is called, these two
**     context tables must be populated, by calling the function
**     ephMoonc in the case of cmoon (the Moon) and ephPlanc in the case
**     of cemb (the Earth-Moon barycenter).
**
**  3  The result pv is the heliocentric position and velocity of the
**     geocenter in au and au/s with respect to ICRS axes.
**
**  4  Compared with DE405 over the 21st century, the position errors
**     are 1.0 km RMS (1.5 km worst case), and the speed errors are
**     0.2 mm/s RMS (0.3 mm/s worst case).
**
**  Called:  ephMoon, ephPlanet, iauZpv, iauSxpv, iauPvmpv
**
**  Last revision:   2022 November 1
**
**  Author P.T.Wallace - see license notice at end.
*/
{
   int j;
   double elun[2][3], solemb[2][3];


/* Preset the result to zeroes in case of error. */
   iauZpv ( pv );

/* Earth to Moon. */
   j = ephMoon ( cmoon, date, elun );
   if ( j ) return j;

/* Sun to EMB. */
   j = ephPlanet ( 3, cemb, date, solemb );
   if ( j ) return j;

/* Sun to Earth. */
   iauSxpv ( -RMME/(1.0+RMME), elun, pv );
   iauPvppv ( pv, solemb, pv );

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
