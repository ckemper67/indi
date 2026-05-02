#include "eph.h"
#include <math.h>

int ephTopPlanet ( int ibody, ephTOPctx* c, double date, double pv[2][3] )
/*
**  - - - - - - - - - - - -
**   e p h T o p P l a n e t
**  - - - - - - - - - - - -
**
**  Heliocentric ICRS position and velocity of an outer planet using
**  the TOP2013 theory.
**
**  Given:
**     ibody   int           body index (5=Jupiter .. 8=Neptune)
**     c       ephTOPctx*    context for chosen body
**     date    double        TDB as an MJD
**
**  Returned:
**     pv      double[2][3]  x,y,z,xdot,ydot,zdot in ICRS (au, au/s)
**
**  Returned (function value):
**             int           status:
**                             +1 = warning: date outside 1-4000 CE
**                              0 = OK
**                             -1 = uninitialized context
**                             -2 = context is for wrong body
**
**  Notes:
**
**  1  The evaluator mirrors the TOP2013.f reference Fortran exactly:
**       arg = m(k) * dmu * time   (time in Julian millennia from J2000)
**       el(iv) += time^it * (c(k)*cos(arg) + s(k)*sin(arg))
**     After summation, the secular mean longitude rate is added:
**       lambda += freq * time;  lambda = fmod(lambda, 2pi)
**
**  2  The elliptic-to-Cartesian conversion (ELLXYZ in the Fortran) uses
**     the same INPOP10A mass parameters embedded in the context rgm field.
**
**  Reference:
**     J.-L. Simon et al., A&A 557, A49 (2013).
*/
{
#define TMIN_TOP (-678577.0)   /* start of 1 CE (MJD) */
#define TMAX_TOP  (782395.0)   /* start of 4001 CE (MJD) */

   int i, j, it, nn, iv, n, k;
   double tm, t[MAXTIME_TOP+1], arg, xl, eels[6];
   double xa, xk, xh, xq, xp, xfi, xki, u, ex2, ex, ex3;
   double gl, gm, e, ce, se, wk, dl, rsa, xcw, xsw, xm, xr;
   double pvecl[2][3], xms, xmc, xn;

/* Preset result. */
   for ( i = 0; i < 2; i++ )
      for ( j = 0; j < 3; j++ )
         pv[i][j] = 0.0;

   if ( !c->init ) return -1;
   if ( ibody != (int) c->ibody ) return -2;

/* Julian millennia since J2000.0. */
   tm = ( date - EP2000 ) / DJMLA;

/* Powers of time. */
   t[0] = 1.0;
   for ( it = 1; it <= MAXTIME_TOP; it++ ) t[it] = t[it-1] * tm;

/* Evaluate the TOP2013 series. */
   nn = 0;
   for ( iv = 0; iv < 6; iv++ ) {
      eels[iv] = 0.0;
      for ( it = 0; it <= MAXTIME_TOP; it++ ) {
         if ( c->limit[it][iv] == 0 ) continue;
         for ( n = 1; n <= c->limit[it][iv]; n++ ) {
            arg = c->m[nn] * c->dmu * tm;
            eels[iv] += t[it] * ( c->c[nn]*cos(arg) + c->s[nn]*sin(arg) );
            nn++;
         }
      }
   }

/* Add secular mean longitude rate, wrap to [0, 2pi). */
   xl = fmod( eels[1] + c->freq * tm, C2PI );
   if ( xl < 0.0 ) xl += C2PI;
   eels[1] = xl;

/* Elliptic variables to ecliptic positions and velocities (ELLXYZ). */
   xa  = eels[0];
   xk  = eels[2];
   xh  = eels[3];
   xq  = eels[4];
   xp  = eels[5];
   xfi = sqrt(1.0 - xk*xk - xh*xh);
   xki = sqrt(1.0 - xq*xq - xp*xp);
   u   = 1.0 / (1.0 + xfi);
   ex2 = xk*xk + xh*xh;
   ex  = sqrt(ex2);
   ex3 = ex2 * ex;

   gl = fmod( xl, C2PI );
   gm = gl - atan2(xh, xk);
   e  = gl + (ex - 0.125*ex3)*sin(gm)
            + 0.5*ex2*sin(2.0*gm)
            + 0.375*ex3*sin(3.0*gm);
   do {
      ce = cos(e);
      se = sin(e);
      wk = xk*se - xh*ce;
      dl = gl - e + wk;
      rsa = 1.0 - xk*ce - xh*se;
      e += dl / rsa;
   } while ( fabs(dl) > 1e-15 );

   xcw = (-xk + ce + u*xh*wk) / rsa;
   xsw = (-xh + se - u*xk*wk) / rsa;
   xm  = xp*xcw - xq*xsw;
   xr  = xa * rsa;

   pvecl[0][0] = xr * (xcw - 2.0*xp*xm);
   pvecl[0][1] = xr * (xsw + 2.0*xq*xm);
   pvecl[0][2] = -2.0 * xr * xki * xm;

   xms = xa * (xh + xsw) / xfi;
   xmc = xa * (xk + xcw) / xfi;
   xn  = c->rgm / pow(xa, 1.5);

   pvecl[1][0] = xn * ((2.0*xp*xp - 1.0)*xms + 2.0*xp*xq*xmc);
   pvecl[1][1] = xn * ((1.0 - 2.0*xq*xq)*xmc - 2.0*xp*xq*xms);
   pvecl[1][2] = 2.0 * xn * xki * (xp*xms + xq*xmc);

/* Rotate from ecliptic to ICRS. */
   for ( k = 0; k < 2; k++ )
      for ( j = 0; j < 3; j++ ) {
         pv[k][j] = 0.0;
         for ( i = 0; i < 3; i++ )
            pv[k][j] += c->receq[i][j] * pvecl[k][i];
      }

/* Velocity to per second. */
   for ( i = 0; i < 3; i++ ) pv[1][i] /= DAYSEC;

   return date >= TMIN_TOP && date <= TMAX_TOP ? 0 : 1;
}

/*----------------------------------------------------------------------
**
**  Copyright (C) 2025 INDI Contributors
**
**  Permission to use, copy, modify, and/or distribute this software for
**  any purpose with or without fee is hereby granted.
**
**--------------------------------------------------------------------*/
