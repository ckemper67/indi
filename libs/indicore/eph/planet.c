#include "eph.h"

int ephPlanet ( int ibody, ephPLANctx* c, double date, double pv[2][3] )
/*
**  - - - - - - - - - -
**   e p h P l a n e t
**  - - - - - - - - - -
**
**  Heliocentric ICRS position and velocity of a planet.
**
**  Given:
**     ibody   int           body index:
**                             1: Mercury
**                             2: Venus
**                             3: EMB
**                             4: Mars
**                             5: Jupiter
**                             6: Saturn
**                             7: Uranus
**                             8: Neptune
**     c       ephPLANctx    context for chosen body (Notes 1,2)
**     date    double        TDB as an MJD (Note 3)
**
**  Returned:
**     pv      double[2][3]  x,y,z,xdot,ydot,zdot, ICRS (au,au/s)
**
*   Returned (function value):
**             int           status:
**                             +1 = warning: date outside 1-4000 CE
**                              0 = OK
**                             -1 = uninitialized context
**                             -2 = context is for wrong body
**
**  Defined in eph.h:
**     ephPLANctx    planetary ephemeris context
**     MAXTIME       VSOP2010 parameter
**     EP2000        J2000.0 MJD
**     DJMLA         days per Julian millennium
**     MAXARG        VSOP2010 parameter
**     C2PI          2pi
**     DAYSEC        seconds per day
**
**  Notes:
**
**  1  Before calling the present function (as many times as necessary),
**     a (single) call to ephPlanc (or alternatively ephPlani) must be
**     made to populate the context c for the body of interest.  If the
**     application involves more than one body at once, in order to
**     avoid costly re-reading of the ephemeris files, each body must
**     have its own context variable.
**
**  2  The choice of body was made when the context c was populated
**     (Note 1).  The ibody argument serves only to allow a check that
**     the context is for the correct body.
**
**  3  The date is a Modified Julian Date, JD-2400000.5.  The time scale
**     is TDB, but for most applications Terrestrial Time TT can be used
**     without significant loss of accuracy (but not of course UTC).
**
**  4  The reference frame is heliocentric ICRS, roughly J2000.0 mean
**     equinox and equator.
**
**  5  Applications requiring apparent place must allow for planetary
**     aberration (by supplying a date that is one light-time before the
**     present) and apply bias-precession-nutation to the results.
**
**  6  The context data structure c, of type ephPLANctx, contains:
**
**       . initialization status (0 = not yet initialized)
**       . VSOP2010 constants
**       . VSOP2010 series
**
**     In an application, ephemerides for successive bodies can be
**     computed using a single context structure that is reinitialized
**     repeatedly.  Should the application need to compute ephemerides
**     for more than one body at once, multiple context structures will
**     need to be declared.
**
**  7  For the given time (in the TDB time scale) the function
**     calculates the following elliptic variables:
**
**        .  semi-major axis a (au)
**        .  mean longitude lambda (radians)
**        .  k = e*cos(pomega) (radians)
**        .  h = e*sin(pomega) (radians)
**        .  q = sin(i/2)*cos(Omega) (radians)
**        .  p = sin(i/2)*sin(Omega) (radians)
**
**     where e is the eccentricity, pi is the longitude of perihelion,
**     i is the inclination and omega is the longitude of the ascending
**     node.  These six quantities (returned as array eels) are then
**     used to predict ecliptic position and velocity vectors, which are
**     then rotated onto ICRS axes (returned as array pv).
**
**  8  Comparisons with DE405 over the interval 2000-2100 gave the
**     following results:
**
**                       position              velocity
**                    RMS        worst       RMS     worst
**
**       Mercury      0.057      0.178       0.043   0.157
**       Venus        0.096      0.310       0.030   0.094
**       Earth        0.177      0.408       0.035   0.078
**       Mars         2.181      5.245       0.234   0.617
**       Jupiter      5.069      8.364       0.085   0.132
**       Saturn       9.854     20.354       0.067   0.133
**       Uranus      20.996     49.481       0.033   0.077
**       Neptune     33.799     66.281       0.042   0.079
**
**                    km         km          mm/s    mm/s
**
**     The comparisons with DE430 were as follows:
**
**       Mercury      2.025      3.259       1.767   3.619
**       Venus        0.791      1.184       0.256   0.379
**       Earth        0.964      1.322       0.192   0.267
**       Mars         3.405      5.028       0.365   0.609
**       Jupiter    306.242    515.699       5.150   8.951
**       Saturn     661.195    989.748       3.793   5.423
**       Uranus    4778.612   7684.369      11.047  17.113
**       Neptune  24117.350  40137.632      26.750  42.459
**
**  References:
**
**     Moisson, X. & Bretagnon, P., Celestial Mechanics and Dynamical
**     Astronomy (2001) 80, 205.
**
**     J.-L. Simon, G. Francou, A. Fienga & H. Manche, "New analytical
**     planetary theories VSOP2010 and TOP2013", Astronomy &
**     Astrophysics 557, A49 (2013).
**
**  Last revision:   2025 February 26
**
**  Author P.T.Wallace - see license notice at end.
*/
{

/* Safe date range (MJD) */
#define TMIN (-678577.0)         /* start of 1 CE */
#define TMAX (782395.0)          /* start of 4001 CE */

   int i, j, it, nn, iv, n, k;
   double tm, t[MAXTIME+1], aa, bb, arg, xl, eels[6], xa, xk, xh, xq,
          xp, xfi, xki, u, ex2, ex, ex3, gl, gm , e, ce, se, wk, dl,
          rsa, xcw, xsw, xm, xr, pvecl[2][3], xms, xmc, xn;


/* Preset the result to zeroes. */
   for ( i = 0; i < 2; i++ ) {
      for ( j = 0; j < 3; j++ ) {
         pv[i][j] = 0.0;
      }
   }

/* Check context has been initialized and is for the right body. */
   if ( ! c->init ) return -1;
   if ( ibody != (int) c->ibody ) return -2;

/* Julian millennia since J2000.0. */
   tm = ( date - EP2000 ) / DJMLA;

/* Powers of time. */
   t[0] = 1.0;
   t[1] = tm;
   for ( it = 2; it <= MAXTIME; it++ ) {
      t[it] = t[1]*t[it-1];
   }

/* Evaluate the series to give elliptic variables. */
   nn = 0;
   for ( iv = 0; iv < 6; iv++ ) {
      eels[iv] = 0.0;
      for ( it = 0; it <= MAXTIME; it++ ) {
         if ( c->limit[it][iv] == 0 ) continue;
         for ( n = 1; n <= c->limit[it][iv]; n++ ) {
            aa = 0.0;
            bb = 0.0;
            for ( j = 0; j < MAXARG; j++ ) {
               aa += ( (double) c->iphi[nn][j] ) * c->ci0[j];
               bb += ( (double) c->iphi[nn][j] ) * c->ci1[j];
            }
            arg = aa + bb*tm;
            eels[iv] += t[it] *( c->ss[nn]*sin(arg)
                              +  c->cc[nn]*cos(arg) );
            nn++;
         }
      }
   }

   xl = fmod ( eels[1] + c->freqpla[c->ibody-1]*tm, C2PI );
   if ( xl < 0.0 ) xl += C2PI;
   eels[1] = xl;

/* Elliptic variables to ecliptic positions & velocities. */
   xa = eels[0];
   xk = eels[2];
   xh = eels[3];
   xq = eels[4];
   xp = eels[5];
   xfi = sqrt(1.0 - xk*xk - xh*xh);
   xki = sqrt(1.0 - xq*xq - xp*xp);
   u = 1.0 / (1.0 + xfi);
   ex2 = xk*xk + xh*xh;
   ex = sqrt(ex2);
   ex3 = ex2*ex;

   gl = fmod ( xl, C2PI );
   gm = gl - atan2(xh,xk);
   e = gl + (ex-0.125*ex3)*sin(gm)
                 + 0.5*ex2*sin(gm*2.0)
               + 0.375*ex3*sin(gm*3.0);
   do {
      ce = cos(e);
      se = sin(e);
      wk = xk*se - xh*ce;
      dl = gl - e + wk;
      rsa = 1.0 - xk*ce - xh*se;
      e += dl/rsa; }
   while ( fabs(dl) > 1e-15 );

   xcw = (-xk+ce+u*xh*wk)/rsa;
   xsw = (-xh+se-u*xk*wk)/rsa;
   xm = xp*xcw-xq*xsw;
   xr = xa*rsa;

   pvecl[0][0] = xr*(xcw-2.0*xp*xm);
   pvecl[0][1] = xr*(xsw+2.0*xq*xm);
   pvecl[0][2] = -2.0*xr*xki*xm;

   xms = xa*(xh+xsw)/xfi;
   xmc = xa*(xk+xcw)/xfi;
   xn = c->rgm/pow(xa,1.5);

   pvecl[1][0] = xn*((2.0*xp*xp-1.0)*xms+2.0*xp*xq*xmc);
   pvecl[1][1] = xn*((1.0-2.0*xq*xq)*xmc-2.0*xp*xq*xms);
   pvecl[1][2] = 2.0*xn*xki*(xp*xms+xq*xmc);

/* Rotate from ecliptic coordinates to ICRS. */
   for ( k = 0; k < 2; k++ ) {
      for ( j = 0; j < 3; j++ ) {
         pv[k][j] = 0.0;
         for ( i = 0; i < 3; i++ ) {
            pv[k][j] += c->receq[i][j]*pvecl[k][i];
         }
      }
   }

/* Change velocity to per second. */
   for ( i = 0; i < 3; i++ ) {
      pv[1][i] /= DAYSEC;
   }

/* Success (perhaps with range warning). */
   return date >= TMIN && date <= TMAX ? 0 : 1;
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
