#include "eph.h"
#include <string.h>
#include <stdio.h>

int ephMooni ( char* path, int icor, ephMOONctx* c )
/*
**   - - - - - - - - -
**    e p h M o o n i
**   - - - - - - - - -
**
**  Generate the context needed for computation of lunar ephemerides
**  reading from an ELP/MPP02 theory ASCII file.
**
**  Given:
**     path    char*        path for filenames
**     icor    int          choice of corrections to the constants:
**                            icor = 1 : LLR
**                            icor = 2 : DE405
**
**  Returned:
**     c       ephMOONctx*  context structure
**
**  Returned (function value):
**             int          status:
**                            0 : OK
**                           -1 : illegal icor
**                           -2 : file path+name too big
**                         else : file-related errors (Note 5)
**
**  Defined in eph.h:
**     CPI           pi
**     C2PI          2pi
**     D90           pi/2
**     AS2R          arcsec to radians
**     DMS           degrees, arcmin, arcsec to radians
**     ephMOONctx    lunar ephemeris context
**
**  Notes:
**
**  1  Generating lunar ephemerides involves first reading files to
**     obtain arrays of coefficients etc., followed by computation of
**     the coordinates for a nominated time.  The present function
**     performs the first phase, initializing a context data structure
**     by reading and interpreting an ASCII file.  Once this context is
**     available a different function, ephMoon, can  be called to
**     compute the ephemeris.
**
**  2  The context data structure, of type ephMOONctx, contains:
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
**  3  The present code explicitly declares the names of the six files
**     that contain the ELP/MPP02 series (array file), and these can be
**     edited if necessary, for example to include the directory name.
**     The latter is in any case supplied through the argument path,
**     which can be the null string if the code has been edited to
**     include the path or the files are in the current directory.
**
**  4  The theoretical values of some constants have to be corrected.
**     There are two sets of corrections, and the choice is indicated by
**     the argument icor:
**
**       icor=1, the constants are fitted to LLR observations provided
**               from 1970 to 2001; it is the default value;
**
**       icor=2, the constants are fitted to the DE405 ephemeris over the
**               interval 1950-2060); the lunar angles w1, w2, w3 receive
**               also additive corrections to the secular coefficients.
**
**  5  Several status values are file-related.  For the three main-
**     problem files, returned status -3 is an open error, -4 and -6 are
**     read errors, -5 and -7 indicate wrong content, and -8 is a close
**     error.  For the three perturbations files, -9 is an open error,
**     -10 and -12 are read errors, -11 and -13 indicate wrong content,
**     and -14 is a close error.  For more detail see the code itself.
**
**  Notations:
**
**     Moon elements (4th order polynomial coefficients):
**       w(1,0:4) : mean longitude of the Moon                      (W1)
**       w(2,0:4) : mean longitude of the lunar perigee             (W2)
**       w(3,0:4) : mean longitude of the lunar ascending node      (W3)
**       zeta(0:4): mean longitude of the Moon + precession      (W1+pt)
**                  where p is the precession rate and t is the time
**
**     Earth-Moon (EMB) elements (4th order polynomial coefficients):
**       eart(0:4): mean longitude of EMB                           (Te)
**       peri(0:4): mean longitude of the EMB perihelion           (Pip)
**
**     Delaunay arguments (4th order polynomial coefficients):
**       del(1,0:4): D  =  W1 - Te + 180 degrees                     (D)
**       del(2,0:4): F  =  W1 - W3                                   (F)
**       del(3,0:4): l  =  W1 - W2   mean anomaly of the Moon        (l)
**       del(4,0:4): l' =  Te - Pip  mean anomaly of EMB            (l')
**
**     Planetary arguments (mean longitudes at J2000 and mean motions):
**       p(1,0:1): mean longitude of Mercury
**       p(2,0:1): mean longitude of Venus
**       p(3,0:1): mean longitude of EMB (eart(0:1))
**       p(4,0:1): mean longitude of Mars
**       p(5,0:1): mean longitude of Jupiter
**       p(6,0:1): mean longitude of Saturn
**       p(7,0:1): mean longitude of Uranus
**       p(8,0:1): mean longitude of Neptune
**
**     Moon constants:
**       nu:    mean motion of the Moon (W1(1,1))                   (Nu)
**       g:     half coefficient of sin(F) in latitude           (Gamma)
**       e:     half coefficient of sin(l) in longitude              (E)
**       np:    mean motion of EMB (eart(1))                        (n')
**       ep:    eccentricity of EMB                                 (e')
**       alpha: ratio of the semi-major axis (Moon/EMB)
**       am:    ratio of the mean motions (EMB/Moon)
**       dtasm: (2*alpha)/(3*am)
**
**     Corrections to the constants Nu, Gamma, E, n', e':
**       delnu: to the mean motion of the Moon
**       delg:  to the half coefficient of sin(F) in latitude
**       dele:  to the half coefficient of sin(l) in longitude
**       delnp: to the mean motion of the EMB
**       delep: to the eccentricity of the EMB
**
**     Precession of the longitude of the ascending node of the mean
**     ecliptic of date on the fixed ecliptic of J2000.0:
**       pi(i=1,5): sine coefficients
**       qi(i=1,5): cosine coefficients
**
**  Reference:
**
**     LUNAR SOLUTION ELP version ELP/MPP02 - Jean CHAPRONT and
**     Gerard FRANCOU Observatoire de Paris - SYRTE department -
**     UMR 8630/CNRS - October 2002
**
**  Latest revision:   2022 July 25
**
**  Author P.T.Wallace - see license notice at end.
*/
{

/* Scratch string */
#define LS 1000
   char str[LS];

   FILE *fp;
   char* dd;
   int lpath, i, j, ir, iv, k, n, nf, ilu[4], it, ipt, icount, ifi[16];
   double eps, phi, ceps, seps, cphi, sphi, dprec, alpha, xa, dw1_0,
          dw2_0, dw3_0, deart_0, dperi, dw1_1, dgam, de, deart_1, dep,
          dw2_1, dw3_1, dw1_2, x2, x3, y2, y3, d21, d22, d23, d24, d25,
          d31, d32, d33, d34, d35, cw2_1, cw3_1, a, b[5], tgv, si, co,
          pha;

/* Additional corrections (paragraph 4.3.2) */
   static double bp[2][5] = { { +0.311079095,    -0.4482398e-2,
                        -0.110248500e-2, +0.1056062e-2, +0.50928e-4 },
                              { -0.103837907,    +0.6682870e-3,
                        -0.129807200e-2, -0.1780280e-3, -0.37342e-4 } };

/* Names of the ELPMPP02 files (n.b. edit if necessary) */
#define LFILE 12
   static char file[][LFILE] = { "elp_main.s1",
                                 "elp_main.s2",
                                 "elp_main.s3",
                                 "elp_pert.s1",
                                 "elp_pert.s2",
                                 "elp_pert.s3" };


/* Reset the context initialized flag. */
   c->jftf = 0;

/* Validate icor. */
   if ( icor != 1 && icor != 2 ) return -1;

/* Check path+filename length. */
   lpath = strlen(path);
   if ( lpath+LFILE > LS ) return -2;

/* Rotation matrix, ecliptic to ICRS (Chapront & Francou Oct 2002. */
   eps = DMS(23, 26, 21.41100);
   phi = -0.05542*AS2R;
   ceps = cos(eps);
   seps = sin(eps);
   cphi = cos(phi);
   sphi = sin(phi);

   c->receq[0][0] =  cphi;
   c->receq[1][0] = -sphi*ceps;
   c->receq[2][0] =  sphi*seps;
   c->receq[0][1] =  sphi;
   c->receq[1][1] =  cphi*ceps;
   c->receq[2][1] = -cphi*seps;
   c->receq[0][2] =  0.0;
   c->receq[1][2] =  seps;
   c->receq[2][2] =  ceps;

/* Correction to the IAU 1976 precession constant (arcsec/JC). */
   dprec = -0.29965;

/* Constants for the evaluation of the partial derivatives. */
   c->am = 0.074801329;
   alpha = 0.002571881;
   c->dtasm = (2.0*alpha)/(3.0*c->am);
   xa = alpha/1.5;

/* Corrections to the constants. */
   if ( icor == 1 ) {

   /* Values fitted to LLR:  fit 13-05-02 (two iterations) except Phi */
   /* and eps w2_1 et w3_1. */
      dw1_0 = -0.10525;
      dw2_0 =  0.16826;
      dw3_0 = -0.10760;
      deart_0 = -0.04012;
      dperi = -0.04854;
      dw1_1 = -0.32311;
      dgam =  0.00069;
      de = 0.00005;
      deart_1 =  0.01442;
      dep =  0.00226;
      dw2_1 =  0.08017;
      dw3_1 = -0.04317;
      dw1_2 = -0.03794;

   } else {

   /* Values fitted to DE405 over the time interval (1950-2060). */
      dw1_0 = -0.07008;
      dw2_0 =  0.20794;
      dw3_0 = -0.07215;
      deart_0 = -0.00033;
      dperi = -0.00749;
      dw1_1 = -0.35106;
      dgam =  0.00085;
      de = -0.00006;
      deart_1 = 0.00732;
      dep = 0.00224;
      dw2_1 =  0.08017;
      dw3_1 = -0.04317;
      dw1_2 = -0.03743;

  }

/* Fundamental arguments (Moon and EMB). */
   c->w[0][0] = DMS(218,18,59.955710+dw1_0);                   /* ELP */
   c->w[1][0] =   (1732559343.73604+dw1_1)*AS2R;               /* ELP */
   c->w[2][0] =   (        -6.8084+dw1_2)*AS2R;              /* DE405 */
   c->w[3][0] =             0.66040e-2*AS2R;                   /* ELP */
   c->w[4][0] =            -0.31690e-4*AS2R;                   /* ELP */

   c->w[0][1] = DMS( 83,21,11.67475+dw2_0);                    /* ELP */
   c->w[1][1] = (    14643420.3171+dw2_1)*AS2R;              /* DE405 */
   c->w[2][1] = (         -38.2631)*AS2R;                    /* DE405 */
   c->w[3][1] =            -0.45047e-1*AS2R;                   /* ELP */
   c->w[4][1] =             0.21301e-3*AS2R;                   /* ELP */

   c->w[0][2] = DMS(125, 2,40.39816+dw3_0);                    /* ELP */
   c->w[1][2] =   (  -6967919.5383+dw3_1)*AS2R;              /* DE405 */
   c->w[2][2] = (           6.3590)*AS2R;                    /* DE405 */
   c->w[3][2] =             0.76250e-2*AS2R;                   /* ELP */
   c->w[4][2] =            -0.35860e-4*AS2R;                   /* ELP */

   c->eart[0] = DMS(100,27,59.13885+deart_0);             /* VSOP2000 */
   c->eart[1] =    (129597742.29300+deart_1)*AS2R;        /* VSOP2000 */
   c->eart[2] =            -0.020200*AS2R;                     /* ELP */
   c->eart[3] =             0.90000e-5*AS2R;                   /* ELP */
   c->eart[4] =             0.15000e-6*AS2R;                   /* ELP */

   c->peri[0] = DMS(102,56,14.45766+dperi);               /* VSOP2000 */
   c->peri[1] =          1161.24342*AS2R;                 /* VSOP2000 */
   c->peri[2] =             0.529265*AS2R;                /* VSOP2000 */
   c->peri[3] =            -0.11814e-3*AS2R;              /* VSOP2000 */
   c->peri[4] =             0.11379e-4*AS2R;              /* VSOP2000 */

/* Corrections to the secular terms of the Moon angles. */
   if ( icor == 2 ) {
      c->w[3][0] += - 0.00018865*AS2R;
      c->w[4][0] += - 0.00001024*AS2R;
      c->w[2][1] += + 0.00470602*AS2R;
      c->w[3][1] += - 0.00025213*AS2R;
      c->w[2][2] += - 0.00261070*AS2R;
      c->w[3][2] += - 0.00010712*AS2R;
   }

/* Corrections to the mean motions of Moon angles W2 and W3: inferred */
/* from the modifications of the constants. */
   x2 = c->w[1][1]/c->w[1][0];
   x3 = c->w[1][2]/c->w[1][0];
   y2 = c->am*bp[0][0] + xa*bp[0][4];
   y3 = c->am*bp[1][0] + xa*bp[1][4];

   d21 = x2-y2;
   d22 = c->w[1][0]*bp[0][1];
   d23 = c->w[1][0]*bp[0][2];
   d24 = c->w[1][0]*bp[0][3];
   d25 = y2/c->am;

   d31 = x3-y3;
   d32 = c->w[1][0]*bp[1][1];
   d33 = c->w[1][0]*bp[1][2];
   d34 = c->w[1][0]*bp[1][3];
   d35 = y3/c->am;

   cw2_1 = d21*dw1_1 + d25*deart_1 + d22*dgam + d23*de + d24*dep;
   cw3_1 = d31*dw1_1 + d35*deart_1 + d32*dgam + d33*de + d34*dep;

   c->w[1][1] += cw2_1*AS2R;
   c->w[1][2] += cw3_1*AS2R;

/* Delaunay arguments. */
   for ( i = 0; i <= 4; i++ ) {
      c->del[i][0] = c->w[i][0] - c->eart[i];                   /* D  */
      c->del[i][1] = c->w[i][0] - c->w[i][2];                   /* F  */
      c->del[i][2] = c->w[i][0] - c->w[i][1];                   /* l  */
      c->del[i][3] = c->eart[i] - c->peri[i];                   /* l' */
   }
   c->del[0][0] += CPI;

/* Planetary arguments (mean longitudes and mean motions). */
   c->p[0][0] = DMS( 252, 15,  3.216919 );                /* VSOP2000 */
   c->p[0][1] = DMS( 181, 58, 44.758419 );                /* VSOP2000 */
   c->p[0][2] = DMS( 100, 27, 59.138850 );                /* VSOP2000 */
   c->p[0][3] = DMS( 355, 26,  3.642778 );                /* VSOP2000 */
   c->p[0][4] = DMS(  34, 21,  5.379392 );                /* VSOP2000 */
   c->p[0][5] = DMS(  50,  4, 38.902495 );                /* VSOP2000 */
   c->p[0][6] = DMS( 314,  3,  4.354234 );                /* VSOP2000 */
   c->p[0][7] = DMS( 304, 20, 56.808371 );                /* VSOP2000 */

   c->p[1][0] = 538101628.66888*AS2R;                     /* VSOP2000 */
   c->p[1][1] = 210664136.45777*AS2R;                     /* VSOP2000 */
   c->p[1][2] = 129597742.29300*AS2R;                     /* VSOP2000 */
   c->p[1][3] =  68905077.65936*AS2R;                     /* VSOP2000 */
   c->p[1][4] =  10925660.57335*AS2R;                     /* VSOP2000 */
   c->p[1][5] =   4399609.33632*AS2R;                     /* VSOP2000 */
   c->p[1][6] =   1542482.57845*AS2R;                     /* VSOP2000 */
   c->p[1][7] =    786547.89700*AS2R;                     /* VSOP2000 */

   for ( i = 0; i < 8; i++ ) {
      for ( j = 2; j <= 4; j++ ) {
         c->p[j][i] = 0.0;
      }
   }

/* Zeta:  mean longitude W1 + precession rate. */
   c->zeta[0] = c->w[0][0];
   c->zeta[1] = c->w[1][0] + ( 5029.0966 + dprec ) * AS2R;
   c->zeta[2] = c->w[2][0];
   c->zeta[3] = c->w[3][0];
   c->zeta[4] = c->w[4][0];

/* Corrections to the parameters:  Nu, E, Gamma, n' and e'. */
   c->delnu = (0.55604 + dw1_1)*AS2R / c->w[1][0];             /* ELP */
   c->dele = (0.01789 + de)*AS2R;                              /* ELP */
   c->delg = (-0.08066 + dgam)*AS2R;                           /* ELP */
   c->delnp = (-0.06424 + deart_1)*AS2R / c->w[1][0];          /* ELP */
   c->delep = (-0.12879 + dep)*AS2R;                           /* ELP */

/* Precession coefficients for P and Q (Laskar, 1986). */
   c->p1 =  0.10180391e-4;
   c->p2 =  0.47020439e-6;
   c->p3 = -0.5417367e-9;
   c->p4 = -0.2507948e-11;
   c->p5 =  0.463486e-14;

   c->q1 = -0.113469002e-3;
   c->q2 =  0.12372674e-6;
   c->q3 =  0.1265417e-8;
   c->q4 = -0.1371808e-11;
   c->q5 = -0.320334e-14;

/* Read main problem series. */
   ir = 0;
   for ( iv = 0; iv < 3; iv++ ) {
      for ( k = 0; k < 3; k++ ) {
         c->nmpb[k][iv] = 0;
      }
   }

   for ( iv = 0; iv < 3; iv++ ) {
      strncpy ( str, path, LS );
      strncat ( str, file[iv], LS-lpath );
      fp = fopen ( str, "r" );
      if ( ! fp ) return -3;
      if ( ! fgets ( str, LS, fp ) ) return -4;
      nf = sscanf ( str+25, "%d", &c->nmpb[0][iv] );
      c->nmpb[1][iv] = ir + 1;
      if ( nf != 1 ) return -5;
      c->nmpb[2][iv] = c->nmpb[0][iv] + c->nmpb[1][iv] - 1;
      for ( n = 1; n <= c->nmpb[0][iv]; n++ ) {
         if ( ! fgets ( str, LS, fp ) ) return -6;
         while ( ( dd = strpbrk ( str, "Dd" ) ) ) *dd = (char) 'e';
         nf = sscanf ( str, "%d %d %d %d %lf %lf %lf %lf %lf %lf",
                  ilu, ilu+1, ilu+2, ilu+3, &a, b, b+1, b+2, b+3, b+4 );
         if ( nf != 10 ) return -7;
         ir++;
         tgv = b[0] + c->dtasm*b[4];
         if ( iv == 2 ) a -= 2.0*a*c->delnu/3.0;
         c->cmpb[ir-1] = a + tgv*(c->delnp-c->am*c->delnu)
                     + b[1]*c->delg + b[2]*c->dele + b[3]*c->delep;
         for ( k = 0; k < 5; k++ ) {
            c->fmpb[ir-1][k] = 0.0;
            for ( i = 0; i < 4; i++ ) {
               c->fmpb[ir-1][k] += ilu[i]*c->del[k][i];
            }
         }
         if ( iv == 2 ) c->fmpb[ir-1][0] += D90;
      }
      if ( fclose ( fp ) ) return -8;
   }

/* Read perturbations series. */
   ir = 0;
   for ( iv = 0; iv < 3; iv++ ) {
      for ( it = 0; it < 4; it++ ) {
         for ( k = 0; k < 3; k++ ) {
            c->nper[k][it][iv] = 0;
         }
      }
   }

   for ( iv = 0; iv < 3; iv++ ) {
      strncpy ( str, path, LS );
      strncat ( str, file[iv+3], LS-lpath );
      fp = fopen ( str, "r" );
      if ( ! fp ) return -9;
      for ( it = 0; it < 4; it++ ) {
         if ( ! fgets ( str, LS, fp ) ) return -10;
         nf = sscanf ( str+25, "%d %d", &c->nper[0][it][iv], &ipt );
         if ( nf != 2 ) return -11;
         c->nper[1][it][iv] = ir + 1;
         c->nper[2][it][iv] = c->nper[0][it][iv]
                            + c->nper[1][it][iv] - 1;
         if ( c->nper[0][it][iv] == 0 ) continue;
         for ( n = 1; n <= c->nper[0][it][iv]; n++ ) {
            if ( ! fgets ( str, LS, fp ) ) return -12;
            while ( ( dd = strpbrk ( str, "Dd" ) ) ) *dd = (char) 'e';
            nf = sscanf ( str,
           "%d %lf %lf %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
                               &icount, &si, &co,
                               ifi, ifi+1, ifi+2, ifi+3,
                               ifi+4, ifi+5, ifi+6, ifi+7,
                               ifi+8, ifi+9, ifi+10, ifi+11,
                               ifi+12, ifi+13, ifi+14, ifi+15 );
            if ( nf != 19 ) return -13;
            ir++;
            c->cper[ir-1] = sqrt ( co*co + si*si );
            pha = atan2 ( co, si );
            if ( pha < 0.0 ) pha += C2PI;
            for ( k = 0; k < 5; k++ ) {
               c->fper[ir-1][k] = 0.0;
               if ( k == 0 ) c->fper[ir-1][k] = pha;
               for ( i = 0; i < 4; i++ ) {
                  c->fper[ir-1][k] += ((double) ifi[i])*c->del[k][i];
               }
               for ( i = 4; i < 12; i++ ) {
                  c->fper[ir-1][k] += ((double) ifi[i])*c->p[k][i-4];
               }
               c->fper[ir-1][k] += ((double) ifi[12])*c->zeta[k];
            }
         }
      }
      if ( fclose ( fp ) ) return -14;
   }

/* Set the context initialized flag. */
   c->jftf = 1;

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
