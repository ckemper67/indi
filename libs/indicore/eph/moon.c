#include "eph.h"

int ephMoon ( ephMOONctx* const c, double date, double pv[2][3] )
/*
**  - - - - - - - -
**   e p h M o o n
**  - - - - - - - -
**
**  Geocentric ICRS position and velocity of the Moon.
**
**  Given:
**     c      ephMOONctx*  ephemeris context structure
**     date   double       TDB as a Modified Julian Date (JD-2400000.5)
**
**  Returned:
**     pv     double[2][3] Moon x,y,z,xdot,ydot,zdot, (GCRS, au, au/s)
**
**  Returned (function value):
**            int          status:
**                           0 : OK
**                          -1 : context not initialized
**
**  Defined in eph.h:
**     ephMOONctx    lunar ephemeris context
**
**  Notes:
**
**  1  Before the present function can be used, the ephMoonc or
**     ephMooni function must be executed.
**
**  2  TT can be used instead of TDB in most applications, but not UTC.
**
**  3  Generating the ephemeris involves first reading files to obtain
**     arrays of coefficients etc., followed by computation of the
**     coordinates for a nominated time.  The first phase, initializing
**     a context data structure, is carried out by calling one of the
**     functions ephMooni or ephMoonc.   Once this context is available
**     the present function can be called to compute the ephemeris.
**
**  4  The context is a data structure of type ephMOONctx supplied by
**     the caller.  It contains:
**
**       . initialization status:
**           0 : not yet initialized
**           1 : icor value (1 for LLR or 2 for DE405)
**       . ELP/MPP02 constants
**       . ELP/MPP02 series
**
**     Because the caller supplies the context structure, it is possible
**     to have more than one in operation at once, though this will not
**     as a rule be useful.  The principal benefit of separating the
**     context from the logic is to make reentrancy possible.
**
**  Reference:
**     LUNAR SOLUTION ELP version ELP/MPP02 - Jean CHAPRONT and
**     Gerard FRANCOU Observatoire de Paris - SYRTE department -
**     UMR 8630/CNRS - October 2002
**
**  Last revision:   2025 February 23
**
**  Author P.T.Wallace - see license notice at end.
*/
{
/* Lunar orbit a0 values (km) */
#define A405 (384747.9613701725)   /* DE405 */
#define AELP (384747.980674318)    /* ELP */

   int it, iv, k, n, j, i;
   double tjc, t[5], v[2][3], cbeta, clamb, cw, ppw, ppw2, ppwqpw,
          ppwra, pw, pw2, pwqw, pwra, pvecl[2][3], qpw, qpw2, qpwra, qw,
          qw2, qwra, ra, rap, sbeta, slamb, sw, x, x1, x2, x3, xp, xp1,
          xp2, xp3, y, yp;


/* Check context initialized. */
   if ( ! c->jftf ) return -1;

/* Time since J2000.0 in Julian centuries. */
   tjc = ( date - EP2000 ) / DJC;

/* Initialize time powers.*/
   t[0] = 1.0;
   t[1] = tjc;
   t[2] = t[1]*t[1];
   t[3] = t[2]*t[1];
   t[4] = t[2]*t[2];

/* Evaluate the series for the specified time. */
/*    iv=1 : longitude                         */
/*    iv=2 : latitude                          */
/*    iv=3 : distance                          */

   for ( iv = 0; iv < 3; iv++ ) {

       v[0][iv] = 0.0;
       v[1][iv] = 0.0;

   /* Main Problem series. */
      for ( n = c->nmpb[1][iv]-1; n <= c->nmpb[2][iv]-1; n++ ) {
          x = c->cmpb[n];
          y = c->fmpb[n][0];
          yp = 0.0;
          for ( k = 1; k <= 4; k++ ) {
             y += c->fmpb[n][k]*t[k];
             yp += ((double) k)*c->fmpb[n][k]*t[k-1];
          }
          v[0][iv] += x*sin(y);
          v[1][iv] += x*yp*cos(y);
      }

   /* Perturbations series. */
      for ( it = 0; it <= 3; it++ ) {
         for ( n = c->nper[1][it][iv]-1;
               n <= c->nper[2][it][iv]-1; n++ ) {
            x = c->cper[n];
            y = c->fper[n][0];
            xp = 0.0;
            yp = 0.0;
            if ( it != 0 ) xp = it*x*t[it-1];
            for ( k = 1; k <= 4; k++ ) {
               y += c->fper[n][k]*t[k];
               yp += ((double) k)*c->fper[n][k]*t[k-1];
            }
            v[0][iv] += x*t[it]*sin(y);
            v[1][iv] += xp*sin(y) + x*t[it]*yp*cos(y);
         }
      }
   }

/* Compute the J2000.0 rectangular ecliptic coordinates. */
   v[0][0] = v[0][0]*AS2R + c->w[0][0]
                          + c->w[1][0]*t[1]
                          + c->w[2][0]*t[2]
                          + c->w[3][0]*t[3]
                          + c->w[4][0]*t[4];
   v[0][1] *= AS2R;
   v[0][2] *= A405/AELP;
   v[1][0] = v[1][0]*AS2R + c->w[1][0]
                      + 2.0*c->w[2][0]*t[1]
                      + 3.0*c->w[3][0]*t[2]
                      + 4.0*c->w[4][0]*t[3];
   v[1][1] *= AS2R;

   clamb = cos(v[0][0]);
   slamb = sin(v[0][0]);
   cbeta = cos(v[0][1]);
   sbeta = sin(v[0][1]);
   cw = v[0][2]*cbeta;
   sw = v[0][2]*sbeta;

   x1 = cw*clamb;
   x2 = cw*slamb;
   x3 = sw;
   xp1 = (v[1][2]*cbeta-v[1][1]*sw)*clamb - v[1][0]*x2;
   xp2 = (v[1][2]*cbeta-v[1][1]*sw)*slamb + v[1][0]*x1;
   xp3 = v[1][2]*sbeta + v[1][1]*cw;

   pw = ( c->p1
        + c->p2*t[1]
        + c->p3*t[2]
        + c->p4*t[3]
        + c->p5*t[4])*t[1];
   qw = ( c->q1
        + c->q2*t[1]
        + c->q3*t[2]
        + c->q4*t[3]
        + c->q5*t[4])*t[1];
   ra = 2.0*sqrt(1.0-pw*pw-qw*qw);
   pwqw = 2.0*pw*qw;
   pw2 = 1.0-2.0*pw*pw;
   qw2 = 1.0-2.0*qw*qw;
   pwra = pw*ra;
   qwra = qw*ra;

   pvecl[0][0] = pw2*x1 + pwqw*x2 + pwra*x3;
   pvecl[0][1] = pwqw*x1 + qw2*x2 - qwra*x3;
   pvecl[0][2] = - pwra*x1 + qwra*x2 + (pw2+qw2-1.0)*x3;

   ppw = c->p1 + ( 2.0*c->p2 + 3.0*c->p3*t[1]
                 + 4.0*c->p4*t[2]
                 + 5.0*c->p5*t[3] ) * t[1];
   qpw = c->q1 + ( 2.0*c->q2 + 3.0*c->q3*t[1]
                 + 4.0*c->q4*t[2]
                 + 5.0*c->q5*t[3] ) * t[1];
   ppw2 = -4.0*pw*ppw;
   qpw2 = -4.0*qw*qpw;
   ppwqpw = 2.0*(ppw*qw+pw*qpw);
   rap = ( ppw2 + qpw2 ) / ra;
   ppwra = ppw*ra + pw*rap;
   qpwra = qpw*ra + qw*rap;

   pvecl[1][0] = ( pw2*xp1 + pwqw*xp2 + pwra*xp3
                + ppw2*x1 + ppwqpw*x2 + ppwra*x3 ) / DJC;
   pvecl[1][1] = ( pwqw*xp1 + qw2*xp2 - qwra*xp3
                + ppwqpw*x1 + qpw2*x2 - qpwra*x3 ) / DJC;
   pvecl[1][2] = ( - pwra*xp1 + qwra*xp2 + (pw2+qw2-1.0)*xp3
                   - ppwra*x1 + qpwra*x2 + (ppw2+qpw2)*x3 ) / DJC;

/* Rotate into GCRS. */
   for ( k = 0; k < 2; k++ ) {
      for ( j = 0; j < 3; j++ ) {
         pv[k][j] = 0.0;
         for ( i = 0; i < 3; i++ ) {
            pv[k][j] += c->receq[i][j]*pvecl[k][i];
         }
      }
   }

/* Change units from km and km/day to au and au/sec. */
   for ( i = 0; i < 3; i++ ) {
      pv[0][i] /= AUKM;
      pv[1][i] /= AUKM * DAYSEC;
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
