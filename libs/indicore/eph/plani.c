#include "eph.h"
#include <string.h>
#include <stdio.h>

int ephPlani ( char* path, int ibody, ephPLANctx* c )
/*
**  - - - - - - - - -
**   e p h P l a n i
**  - - - - - - - - -
**
**  Generate the context needed for computation of planetary ephemerides
**  reading from a VSOP2010 theory ASCII file.
**
**  Given:
**     path    char*         path for filenames
**     ibody   int           body index:
**                             1: Mercury
**                             2: Venus
**                             3: EMB
**                             4: Mars
**                             5: Jupiter
**                             6: Saturn
**                             7: Uranus
**                             8: Neptune
**
**  Returned:
**     c       ephPLANctx*   context structure
**
**  Returned (function value):
**             int         status:
**                             0  no error
**                            -1  illegal ibody
**                            -2  file path+name too big
**                            -3  file open error
**                            -4  file content error
**                          else  file read error
**
**  Defined in eph.h:
**     AS2R          arcsec to radians
**     DMS           degrees, arcmin, arcsec to radians
**     MAXTIME       VSOP2010 parameter
**     ephPLANctx    planetary ephemeris context
**
**  Notes:
**
**  1  Generating planetary ephemerides involves first reading files
**     (one per body) to obtain arrays of coefficients etc., followed by
**     computation of the coordinates for a nominated time.  The present
**     function performs the first phase, initializing a context data
**     structure for the chosen body by reading and interpreting an
**     ASCII file.  Once this context is available a different function,
**     ephPlanpv, can be called to compute the ephemeris data.
**
**  2  The context data structure, of type ephPLANctx, contains:
**
**       . initialization status (false = not yet initialized)
**       . VSOP2010 constants
**       . VSOP2010 series
**
**     In an application, ephemerides for successive bodies can be
**     computed using a single context structure that is reinitialized
**     repeatedly.  Should the application need to compute ephemerides
**     for more than one body at once, multiple context structures must
**     be declared.
**
**  3  The present code explicitly declares the names of the nine files
**     that contain the VSOP2010 series (array file), and these can be
**     edited if necessary, for example to include the directory name.
**     The latter can in any case be supplied through the argument path,
**     which can be set to an empty string if the code has been edited
**     to include the path or because the files are in the current
**     directory.
**
**  Reference:
**
**     J.-L. Simon, G. Francou, A. Fienga & H. Manche, "New analytical
**     planetary theories VSOP2010 and TOP2013", Astronomy &
**     Astrophysics 557, A49 (2013).
**
**  Last revision:   2021 October 9
**
**  Author P.T.Wallace - see license notice at end.
*/
{

/* Scratch string */
#define LS 200
   char str[LS];

/* Names of the VSOP2010 files (n.b. edit if necessary) */
#define LFILEN 15
   static char file[][LFILEN] = { "VSOP2010p1.dat",
                                  "VSOP2010p2.dat",
                                  "VSOP2010p3.dat",
                                  "VSOP2010p4.dat",
                                  "VSOP2010p5.dat",
                                  "VSOP2010p6.dat",
                                  "VSOP2010p7.dat",
                                  "VSOP2010p8.dat" };

/* Masses system (DE405) */
   const double gmsol =    0.2959122082855911e-03;
   const double gmp[9] = { 0.4912547451450812e-10,    /* Mercury */
                           0.7243452486162703e-09,    /* Venus   */
                           0.8997011346712499e-09,    /* EMB     */
                           0.9549535105779258e-10,    /* Mars    */
                           0.2825345909524226e-06,    /* Jupiter */
                           0.8459715185680659e-07,    /* Saturn  */
                           0.1292024916781969e-07,    /* Uranus  */
                           0.1524358900784276e-07 };  /* Neptune */

   FILE* fp;
   int lpath, it, iv, nn, nf, ibo, ibf, nt, n;
   double eps, phi, ceps, seps, cphi, sphi;


/* Validate and save ibody. */
   if ( ibody < 1 || ibody > 8 ) return -1;
   c->ibody = (short) ibody;
   ibo = ibody-1;

/* Prepare file name. */
   lpath = strlen(path);
   if ( lpath+LFILEN > LS ) return -2;

/* Rotation matrix, ecliptic to ICRS (Francou & Simon December 2015). */
   eps = DMS(23, 26, 21.40960);
   phi = -0.05028*AS2R;
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

/* Mass parameter. */
   c->rgm = sqrt(gmp[ibo]+gmsol);

/* Mean longitudes at J2000.0 etc. (radian). */
   c->ci0[ 0] = 4.402608634958;            /* Mercury */
   c->ci0[ 1] = 3.176134454599;            /* Venus */
   c->ci0[ 2] = 1.753470407365;            /* EMB */
   c->ci0[ 3] = 6.203499866531;            /* Mars */
   c->ci0[ 4] = 4.091362210690;            /* Vesta */
   c->ci0[ 5] = 1.713743790353;            /* Iris */
   c->ci0[ 6] = 5.598651923117;            /* Bamberga */
   c->ci0[ 7] = 2.805135511956;            /* Ceres */
   c->ci0[ 8] = 2.326992146758;            /* Pallas */
   c->ci0[ 9] = 0.599546097920;            /* Jupiter */
   c->ci0[10] = 0.874018344970;            /* Saturn */
   c->ci0[11] = 5.481224786038;            /* Uranus */
   c->ci0[12] = 5.311894573453;            /* Neptune */
   c->ci0[13] = 0.0;                       /* not used (Pluto) */
   c->ci0[14] = 5.19846640063;             /* Moon (D) */
   c->ci0[15] = 1.62790513602;             /* Moon (F) */
   c->ci0[16] = 2.35555563875;             /* Moon (l) */

/* Mean motions in longitude (radian/cy). */
   c->ci1[ 0] = 0.2608790314074786e5;      /* Mercury */
   c->ci1[ 1] = 0.1021328554727840e5;      /* Venus */
   c->ci1[ 2] = 0.6283075850238015e4;      /* EMB */
   c->ci1[ 3] = 0.3340612433480507e4;      /* Mars */
   c->ci1[ 4] = 0.1731170540074402e4;      /* Vesta */
   c->ci1[ 5] = 0.1704450784022772e4;      /* Iris */
   c->ci1[ 6] = 0.1428949097282629e4;      /* Bamberga */
   c->ci1[ 7] = 0.1364756486739947e4;      /* Ceres */
   c->ci1[ 8] = 0.1361923496417814e4;      /* Pallas */
   c->ci1[ 9] = 0.5296909681760810e3;      /* Jupiter */
   c->ci1[10] = 0.2132990860917330e3;      /* Saturn */
   c->ci1[11] = 0.7478165380027799e2;      /* Uranus */
   c->ci1[12] = 0.3813292737322700e2;      /* Neptune */
   c->ci1[13] = 0.3595362366859080;        /* not used (Pluto) */
   c->ci1[14] = 0.777137714481804e5;       /* Moon (D) */
   c->ci1[15] = 0.843346615717837e5;       /* Moon (F) */
   c->ci1[16] = 0.832869142477147e5;       /* Moon (l) */

/* Planetary frequency in longitude. */
   c->freqpla[0] = 0.2608790314074786e5;   /* Mercury */
   c->freqpla[1] = 0.1021328554727840e5;   /* Venus */
   c->freqpla[2] = 0.6283075850238015e4;   /* EMB */
   c->freqpla[3] = 0.3340612433480507e4;   /* Mars */
   c->freqpla[4] = 0.5296909681760810e3;   /* Jupiter */
   c->freqpla[5] = 0.2132990860917330e3;   /* Saturn */
   c->freqpla[6] = 0.7478165380027799e2;   /* Uranus */
   c->freqpla[7] = 0.3813292737322700e2;   /* Neptune */

/* Open the VSOP2010 file for this body. */
   strncpy ( str, path, LS );
   strncat ( str, file[ibo], LS-lpath );
   fp = fopen ( str, "r" );
   if ( ! fp ) return -3;

/* Read the file to populate various arrays. */
   for ( it = 0; it <= MAXTIME; it++ ) {
      for ( iv = 0; iv < 6; iv++ ) {
         c->limit[it][iv] = 0;
      }
   }
   nn = 0;
   while ( 1 ) {
      if ( ! fgets ( str, LS, fp ) ) break;
      nf = sscanf ( str+10, "%d %d %d %d", &ibf, &iv, &it, &nt );
      if ( nf != 4 || ibf != ibody ) return -4;
      iv--;
      c->limit[it][iv] = (short) nt;
      for ( n = 1; n <= nt; n++ ) {
         if ( ! fgets ( str, LS, fp ) ) return -5;
         str[88] = (char) 'e';
         str[112] = (char) 'e';
         nf = sscanf ( str+6, "%hd %hd %hd %hd %hd %hd %hd %hd %hd "
                              "%hd %hd %hd %hd %hd %hd %hd %hd %lf %lf",
                 &c->iphi[nn][0],  &c->iphi[nn][1],  &c->iphi[nn][2],
                 &c->iphi[nn][3],  &c->iphi[nn][4],  &c->iphi[nn][5],
                 &c->iphi[nn][6],  &c->iphi[nn][7],  &c->iphi[nn][8],
                 &c->iphi[nn][9],  &c->iphi[nn][10], &c->iphi[nn][11],
                 &c->iphi[nn][12], &c->iphi[nn][13], &c->iphi[nn][14],
                 &c->iphi[nn][15], &c->iphi[nn][16],
                 &c->ss[nn], &c->cc[nn] );
         if ( nf != 19 ) return -4;
         if ( iv == 1 && it == 1 && n == 1 ) {
            c->limit[it][iv] = (short) nt-1;
         } else {
            nn++;
         }
      }
   }
   if ( fclose ( fp ) ) return -6;

/* Set the context initialized flag. */
   c->init = 1;

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
