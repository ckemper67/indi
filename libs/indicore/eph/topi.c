#include "eph.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int ephTopi ( char* path, int ibody, ephTOPctx* c )
/*
**  - - - - - - - - -
**   e p h T o p i
**  - - - - - - - - -
**
**  Generate the context needed for TOP2013 outer-planet ephemerides by
**  reading from the TOP2013.dat ASCII file.
**
**  Given:
**     path    char*         directory path (trailing slash optional)
**     ibody   int           body index:
**                             5: Jupiter
**                             6: Saturn
**                             7: Uranus
**                             8: Neptune
**
**  Returned:
**     c       ephTOPctx*    context structure
**
**  Returned (function value):
**             int         status:
**                             0  no error
**                            -1  illegal ibody
**                            -2  file path+name too big
**                            -3  file open error
**                            -4  file content error
**                            -5  too many terms (exceeds MAXTERM_TOP)
**
**  Reference:
**     J.-L. Simon, G. Francou, A. Fienga & H. Manche, "New analytical
**     planetary theories VSOP2013 and TOP2013", A&A 557, A49 (2013).
**
**  Last revision:   2025
*/
{
#define LS 200
   char str[LS];

/* Planetary mean longitude rates (rad/millennium), from TOP2013.f */
   static const double top_freq[5] = {
      0.5296909622785881e3,   /* Jupiter (ibody=5) */
      0.2132990811942489e3,   /* Saturn  (ibody=6) */
      0.7478166163181234e2,   /* Uranus  (ibody=7) */
      0.3813297236217556e2,   /* Neptune (ibody=8) */
      0.2533566020437000e2,   /* Pluto   (ibody=9) */
   };

/* Rotation matrix parameters, ecliptic to ICRS (Simon et al. 2013) */
   const double eps = DMS(23, 26, 21.41136);
   const double phi = -0.05188*AS2R;
   double ceps, seps, cphi, sphi;

/* Masses (INPOP10A) */
   const double gmsol = 0.2959122083684144e-03;
   static const double gmp[9] = {
      0.4912547451450812e-10,   /* Mercury */
      0.7243452486162703e-09,   /* Venus   */
      0.8997011603631609e-09,   /* EMB     */
      0.9549535105779258e-10,   /* Mars    */
      0.2825345842083778e-06,   /* Jupiter */
      0.8459715185680659e-07,   /* Saturn  */
      0.1292024916781969e-07,   /* Uranus  */
      0.1524358900784276e-07,   /* Neptune */
      0.0                       /* Pluto (not used via ELLXYZ for INDI) */
   };

   FILE *fp;
   int  lpath, ip, iv, it, nt, mk, n, nn;
   char cstr[27], sstr[27];


/* Validate ibody (TOP2013 covers outer planets 5-8; Pluto=9 not exposed). */
   if ( ibody < 5 || ibody > 8 ) return -1;
   c->ibody = (short) ibody;

/* Fundamental frequency step (rad/millennium). */
   c->dmu  = (top_freq[0] - top_freq[1]) / 880.0;
   c->freq = top_freq[ibody - 5];

/* Rotation matrix, ecliptic to ICRS. */
   ceps = cos(eps); seps = sin(eps);
   cphi = cos(phi); sphi = sin(phi);
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
   c->rgm = sqrt(gmp[ibody-1] + gmsol);

/* Open TOP2013.dat. */
   lpath = strlen(path);
   if ( lpath + 12 > LS ) return -2;
   strncpy(str, path, LS-1);
   if ( lpath > 0 && str[lpath-1] != '/' ) {
      str[lpath] = '/'; str[lpath+1] = '\0';
   }
   strncat(str, "TOP2013.dat", LS-1);
   fp = fopen(str, "r");
   if ( !fp ) return -3;

/* Initialise limit table. */
   for ( it = 0; it <= MAXTIME_TOP; it++ )
      for ( iv = 0; iv < 6; iv++ )
         c->limit[it][iv] = 0;

/* Scan file: parse all sections for this planet. */
   nn = 0;
   while ( fgets(str, LS, fp) ) {

      /* Section header: "TOP2013ELL    PLANET  5    VARIABLE  1    T**00    5921" */
      /* Header lines start with a leading space: " TOP2013ELL..." */
      if ( strncmp(str+1, "TOP2013ELL", 10) != 0 ) continue;

      /* Fortran format (21x,i2,12x,i2,7x,i2,2x,i6):
         positions (0-based): planet@21, variable@35, timepower@44, nt@51 */
      if ( sscanf(str+21, "%d", &ip) != 1 ) { fclose(fp); return -4; }
      if ( ip != ibody ) {
         /* Skip all term lines for a different planet. */
         if ( sscanf(str+35, "%*s %*s %d %*s %*s %*s %d", &it, &nt) == 2 ) {
            for ( n = 0; n < nt; n++ )
               if ( !fgets(str, LS, fp) ) { fclose(fp); return -4; }
         }
         continue;
      }

      /* Our planet: extract variable (iv), time power (it), term count (nt). */
      if ( sscanf(str+35, "%d", &iv) != 1 ) { fclose(fp); return -4; }
      if ( sscanf(str+44, "%d", &it) != 1 ) { fclose(fp); return -4; }
      if ( sscanf(str+50, "%d", &nt) != 1 ) { fclose(fp); return -4; }
      iv--;  /* convert to 0-based */
      if ( iv < 0 || iv > 5 || it < 0 || it > MAXTIME_TOP ) {
         fclose(fp); return -4;
      }

      c->limit[it][iv] = (short) nt;

      for ( n = 1; n <= nt; n++ ) {
         if ( !fgets(str, LS, fp) ) { fclose(fp); return -4; }

         /* Fortran format (1x,i8,2a26).
            positions (0-based): m@1, c-string@9, s-string@35 */
         if ( sscanf(str+1, "%d", &mk) != 1 ) { fclose(fp); return -4; }

         strncpy(cstr, str+9,  26); cstr[26] = '\0';
         strncpy(sstr, str+35, 26); sstr[26] = '\0';
         /* Patch the space before the exponent to 'e' (char index 22, 0-based). */
         if ( cstr[22] == ' ' || cstr[22] == 'D' || cstr[22] == 'd' ) cstr[22] = 'e';
         if ( sstr[22] == ' ' || sstr[22] == 'D' || sstr[22] == 'd' ) sstr[22] = 'e';

         /* Skip secular mean-longitude term (iv=1, it=1, m=0). */
         if ( iv == 1 && it == 1 && mk == 0 ) {
            c->limit[it][iv] = (short)(nt - 1);
            continue;
         }

         if ( nn >= MAXTERM_TOP ) { fclose(fp); return -5; }
         c->m[nn] = mk;
         c->c[nn] = strtod(cstr, NULL);
         c->s[nn] = strtod(sstr, NULL);
         nn++;
      }
   }

   if ( fclose(fp) ) return -6;
   c->init = 1;
   return 0;
}

/*----------------------------------------------------------------------
**
**  Copyright (C) 2025 INDI Contributors
**
**  Permission to use, copy, modify, and/or distribute this software for
**  any purpose with or without fee is hereby granted.
**
**--------------------------------------------------------------------*/
