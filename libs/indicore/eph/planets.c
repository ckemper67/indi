#include "eph.h"
#include <sofa.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <ctype.h>

int Dafin ( const char*, int*, double* );
int Dfltin ( const char*, int*, double* );
int Int2in ( const char*, int*, int* );
int Intin ( const char*, int*, long* );

#define dnint(A) (fabs(A)<0.5?0.0\
                                :((A)<0.0?ceil((A)-0.5):floor((A)+0.5)))
#define dint(A) (((A)<0.0)?ceil(A):floor(A))

/*
**  - - - - - - - -
**   P L A N E T S
**  - - - - - - - -
**
**  For a given date, time and geographical location, report topocentric
**  apparent places and angular diameters of solar-system bodies.
**
**  Any command-line argument is used as the path for the (binary)
**  ephemeris files.
**
**  Latest revision:   2022 November 2
**
**  Author P.T.Wallace - see license notice at end.
*/

int main ( int nargs, char *argv[] )
{
/* Default site longitude (dms), latitude (dms) and altitude (meters) */
   #define DLON "2 13 53.0"
   #define DLAT "48 48 18.0"
   #define DALT 162.0

/* Lunar ephemeris context */
   ephMOONctx* cmoon = malloc(sizeof(ephMOONctx));

/* Planetary ephemeris contexts for 0=Mercury thru 7=Neptune, 2=EMB */
   ephPLANctx* cplanet[8];

/* System time and UTC. */
   time_t rawtime;
   struct tm *gmt;

/* CIRS-to-observed parameters */
   iauASTROM astrom;

/* Body names (ephRdplan) */
   char bodies[9][8] = { "Sun",
                         "Mercury",
                         "Venus",
                         "Moon",
                         "Mars",
                         "Jupiter",
                         "Saturn",
                         "Uranus",
                         "Neptune" };

#define LB 100
   char b[LB+1];

   char s;
   int jft, i, np, iy, j, im, id, i4[4];
   double fd, utc, tai, tt, pv[2][3], u, v, tdb, elong, phi, hm, w,
          pvgm[2][3], pvsb[2][3], rast, dast, rapp, dapp, eo, diam, az,
          zd, hob, dob, rob;

   (void) nargs;


/* Set the first time flag. */
   jft = 1;

/* Get site location (defaults are hardwired). */
   printf ( "Longitude? (D,M,S, east +ve): " );
   if ( ! fgets ( b, LB, stdin ) ) {
      printf ( "Input error!\n" );
      return 1;
   }
   i = 1;
   j = Dafin ( b, &i, &elong );
   if ( j == 1 ) {
      strncpy ( b, DLON, LB );
      i = 1;
      j = Dafin ( b, &i, &elong );
   }
   printf ( "Latitude? (D,M,S, north +ve): " );
   if ( ! fgets ( b, LB, stdin ) ) {
      printf ( "Input error!\n" );
      return 1;
   }
   i = 1;
   j = Dafin ( b, &i, &phi );
   if ( j == 1 ) {
      strncpy ( b, DLAT, LB );
      i = 1;
      j = Dafin ( b, &i, &phi );
   }
   printf ( "Height above sea level? (meters, g=geocentric): " );
   if ( ! fgets ( b, LB, stdin ) ) {
      printf ( "Input error!\n" );
      return 1;
   }
   for ( i = 0; (b[i] = (char) toupper(b[i])); i++ );
   if ( strncmp ( b, "G\n", 3 ) ) {
      i = 1;
      j = Dfltin ( b, &i, &hm );
      if ( j == 1 ) hm = DALT;
   } else {
      hm = -1e6;
   }

/* Get date, using system time as the default. */
   printf ( "Date? (UTC Y,M,D, Gregorian): " );
   if ( ! fgets ( b, LB, stdin ) ) {
      printf ( "Input error!\n" );
      return 1;
   }
   (void) time ( &rawtime );
   gmt = gmtime ( &rawtime );
   iy = gmt->tm_year + 1900;
   im = gmt->tm_mon + 1;
   id = gmt->tm_mday;
   i = 1;
   j = Int2in ( b, &i, &iy );
   j = Int2in ( b, &i, &im );
   j = Int2in ( b, &i, &id );
   if ( abs(iy) < 100 ) printf ( "n.b. 1st century CE!!\n" );

/* Loop until 'end' or "q" typed. */
   b[0] = (char) '\0';
   while ( 1 ) {

   /* Get time, using system time as the default. */
      printf ( "Time? (UTC H,M,S, q=quit): " );
      if ( ! fgets ( b, LB, stdin ) ) {
         printf ( "Input error!\n" );
         return 1;
      }
      for ( i = 0; (b[i] = (char) toupper(b[i])); i++ );
      if ( ! strncmp ( b, "END\n", 5 ) ||
           ! strncmp ( b, "Q\n", 3 ) ) break;
      i = 1;
      j = Dafin ( b, &i, &fd );
      if ( j != 1 ) {
         fd *= 2.3873241463784300365;   /* factor is 15/2pi */
      } else {
         (void) time ( &rawtime );
         gmt = gmtime ( &rawtime );
         iy = gmt->tm_year + 1900;
         im = gmt->tm_mon + 1;
         id = gmt->tm_mday;
         j = iauTf2d ( '+', gmt->tm_hour,
                            gmt->tm_min,
                           (double) gmt->tm_sec, &fd );
      }
      j = iauCal2jd ( iy, im, id, &w, &utc );
      utc += fd;
      j = iauUtctai ( w, utc, &w, &tai );
      j = iauTaitt ( w, tai, &w, &tt );

   /* Observer geocentric coordinates (m, m/s). */
      iauPvtob ( elong, phi, hm, 0.0, 0.0, 0.0, 0.0, pv );

   /* Distances from axis and equator (km). */
      u = sqrt(pv[0][0]*pv[0][0]+pv[0][1]*pv[0][1])/1e3;
      v = pv[0][2]/1e3;

   /* TDB. */
      tdb = tt + iauDtdb(DJM0,tt,fmod(utc,1.0),elong,u,v)/86400.0;

   /* Report site and date/time. */
      printf ( "\n" );
      if ( hm > -1e3 ) {
         iauA2af ( 1, fabs(elong), &s, i4 );
         printf ( "%c%d %2.2d %2.2d.%d",
                  s,
                  i4[0], i4[1], i4[2], i4[3] );
         iauA2af ( 1, phi, &s, i4 );
         printf ( " %c%d %2.2d %2.2d.%1d, alt %.1fm\n",
                  s,
                  i4[0], i4[1], i4[2], i4[3], hm );
      } else {
         printf ( " geocentric\n" );
      }
      for ( i = 0; i < 2; i++ ) {
         w = i ? tdb : utc;
         j = iauJd2cal ( DJM0, w, &iy, &im, &id, &w );
         iauD2tf ( 1, w, &s, i4 );
         printf ( "%4i/%2.2i/%2.2i  %2.2d:%2.2d:%2.2d.%d ",
                  iy, im, id, i4[0], i4[1], i4[2], i4[3] );
         printf ( "%s\n", i ? "TDB" : "UTC" );
      }
      printf ( "\n" );

   /* If first time through, populate the contexts. */
      if ( jft ) {
         jft = 0;
         j = ephMoonc ( argv[1], 2, cmoon );
         if ( j ) {
            printf ( "ephMoonc error %d\n", j );
            return 1;
         }
         for ( np = 1; np <= 8; np++ ) {
            cplanet[np-1] = malloc(sizeof(ephPLANctx));
            j = ephPlanc ( np, argv[1], cplanet[np-1] );
            if ( j ) {
               printf ( "ephPlanc error %d\n", j );
               return 1;
            }
         }
      }

   /* CIRS-to-observed parameters. */
      iauApco13 ( DJM0, utc, 0.0, elong, phi, hm, 0.0, 0.0,
                  280.0, 1013.25, 0.5, 0.55, &astrom, &eo );

   /* Geocenter to Moon p+v vector (ICRS, au, au/s). */
      j = ephMoon ( cmoon, tdb, pvgm );
      if ( j ) {
         printf ( "ephMoon error %d\n", j );
         return 1;
      }

   /* Sun to Earth-Moon barycenter. */
      j = ephPlanet ( 3, cplanet[2], tdb, pvsb );
      if ( j ) {
         printf ( "ephPlanet error %d\n", j );
         return 1;
      }

   /* Loop body by body (Sun, Mercury, Venus, Moon, Mars,...Neptune). */
      for ( np = 0; np <= 8; np++ ) {

      /* Context index (0, 0, 1, 2,...7). */
         i = ( np > 0 ) ? np-1 : 0;

      /* Get RA,Decs and diameter (neglecting EOPs). */
         j = ephRdplanq ( pvgm, pvsb, cplanet[i],
                          utc, tdb, np, elong, phi, hm,
                          &rast, &dast, &rapp, &dapp, &eo, &diam );
         if ( j ) {
            printf ( "ephRdplanq error %d\n", j );
            return 1;
         }

      /* Calculate observed azimuth and zenith distance. */
         iauAtioq ( rapp+eo, dapp, &astrom,
                    &az, &zd, &hob, &dob, &rob );

      /* One line of report. */
         iauA2tf ( 2, rapp, &s, i4 );
         printf ( "%-8s %2.2d %2.2d %2.2d.%2.2d",
                  bodies[np], i4[0], i4[1], i4[2], i4[3] );
         iauA2af ( 1, dapp, &s, i4 );
         printf ( " %c%2.2d %2.2d %2.2d.%1d",
                  s, i4[0], i4[1], i4[2], i4[3] );
         printf ( " %7.1f  %6.2f %6.2f\n",
                  diam*R2AS, iauAnp(az)*R2D, 90.0-zd*R2D );

      /* Next body. */
      }
      printf ( "\n" );

   /* Next case. */
   }

/* Finished. */
   return 0;
}

/*--------------------------------------------------------------------*/

int Dafin ( const char *string, int *iptr, double *a )
/*
**  - - - - - -
**   D a f i n
**  - - - - - -
**
**  Sexagesimal character string to angle.
**
**  Given:
**     string  char*   string containing deg, arcmin, arcsec fields
**     iptr    int     where to start decode (1st = 1)
**
**  Returned:
**     iptr    int     advanced past the decoded angle
**     a       double  angle in radians
**
**  Returned (function value):
**             int     status:  0 = OK
**                             +1 = default, argument a unchanged
**                             -1 = bad degrees      )
**                             -2 = bad arcminutes   )  (Note 3)
**                             -3 = bad arcseconds   )
**
**  Example:
**
**    argument    before                           after
**
**    string      '-57 17 44.806  12 34 56.7'      unchanged
**    iptr        1                                16 (points to 12...)
**
**    a           ?                                -1.00000
**    status      ?                                0
**
**    A further call to Dafin, without adjustment of iptr, will decode
**    the second angle, 12deg 34min 56.7sec.
**
**  Notes:
**
**  1   The first three "fields" in string are degrees, arcminutes,
**      arcseconds, separated by spaces or commas.  The degrees field
**      may be signed, but not the others.  The decoding is carried out
**      by the Dfltin function and is free-format.
**
**  2   Successive fields may be absent, defaulting to zero.  For zero
**      status, the only combinations allowed are degrees alone, degrees
**      and arcminutes, and all three fields present.  If all three
**      fields are omitted, a status of +1 is returned and a is
**      unchanged.  In all other cases a is changed.
**
**  3   Range checking:
**        The degrees field is not range checked.  However, it is
**        expected to be integral unless the other two fields are
**        absent.  The arcminutes field is expected to be 0-59, and
**        integral if the arcseconds field is present.  If the
**        arcseconds field is absent, the arcminutes is expected to
**        be 0-59.9999...  The arcseconds field is expected to be
**        0-59.9999...
**
**  4   Decoding continues even when a check has failed.  Under these
**      circumstances the field takes the supplied value, defaulting to
**      zero, and the result a is computed and returned.
**
**  5   Further fields after the three expected ones are not treated as
**      an error.  The pointer iptr is left in the correct state for
**      further decoding with the present function or with Dfltin etc.
**      See the example, above.
**
**  6   If string contains hours, minutes, seconds instead of degrees
**      etc., or if the required units are turns (or days) instead of
**      radians, the result a should be multiplied as follows:
**
**        for        to obtain    multiply
**        string     a in         a by
**        d ' "      radians      1       =  1.0
**        d ' "      turns        1/2pi   =  0.1591549430918953358
**        h m s      radians      15      =  15.0
**        h m s      days         15/2pi  =  2.3873241463784300365
**
**  Called:  Dfltin
**
**  Last revision:   2022 March 17
*/
{
#define DAS2R (4.848136811095359935899141e-6)   /* arcsec to radians */

   int jd, jf, jm, js;
   double arcsec, arcmin, deg;


/* Preset the status to OK. */
   jf = 0;

/* Defaults */
   deg = 0.0;
   arcmin = 0.0;
   arcsec = 0.0;

/* Decode degrees, arcminutes, arcseconds. */
   jd = Dfltin ( string, iptr, &deg );
   if ( jd > 1 ) {
      jf = -1;
   } else {
      jm = Dfltin ( string, iptr, &arcmin );
      if ( jm < 0 || jm > 1 ) {
         jf = -2;
      } else {
         js = Dfltin ( string, iptr, &arcsec );
         if ( js < 0 || js > 1 ) {
            jf = -3;

      /* See if the combination of fields is credible */
         } else if ( jd > 0 ) {

         /* No degrees:  arcmin, arcsec ought also to be absent */
            if ( jm == 0 ) {

            /* Suspect arcmin */
               jf = -2;
            } else if ( js == 0 ) {

            /* Suspect arcsec */
               jf = -3;
            } else {

            /* All three fields absent */
               jf = 1;
            }

      /* Degrees present:  if arcsec present so ought arcmin to be. */
         } else if ( jm != 0 && js == 0 ) {
            jf = -3;

      /* Tests for range and integrality. */

      /* Degrees. */
         } else if ( jm == 0 && dint ( deg ) != deg ) {
            jf = -1;

      /* Arcminutes. */
         } else if ( ( js == 0 && dint ( arcmin ) != arcmin )
                     || arcmin >= 60.0 ) {
            jf = -2;

      /* Arcseconds. */
         } else if ( arcsec >= 60.0 ) {
            jf = -3;
         }
      }
   }

/* Unless all three fields absent, compute angle value. */
   if ( jf <= 0 ) {
      *a = ( ( fabs(deg) * 60.0 + arcmin ) * 60.0 + arcsec ) * DAS2R;
      if (jd < 0) {
           *a = -(*a);
      }
   }

/* Return the status. */
   return jf;
}

/*--------------------------------------------------------------------*/

#include <string.h>

static int idchf ( int, const char*, int*, int*, double* );

int Dfltin ( const char *string, int *nstrt, double *dreslt )
/*
**  - - - - - - -
**   D f l t i n
**  - - - - - - -
**
**  Convert free-format input into double precision floating point.
**
**  Given:
**     *string     char       string containing field to be decoded
**     *nstrt      int        where to start decode (1st = 1)
**
**  Returned:
**     *nstrt      int        advanced to next field
**     *dreslt     double     result
**
**  Returned (function value):
**                 int        -1 = -OK
**                             0 = +OK
**                            +1 = null field
**                            +2 = error
**
**  Notes:
**
**  1   A tab character is interpreted as a space, and lower case d,e
**      are interpreted as upper case.
**
**  2   The basic format is #^.^@#^ where # means + or -, ^ means a
**      decimal subfield and @ means D or E.
**
**  3   Spaces:
**
**        Leading spaces are ignored.
**        Embedded spaces are allowed only after # and D or E,
**           and after . where the first ^ is absent.
**        Trailing spaces are ignored;  the first signifies
**           end of decoding and subsequent ones are skipped.
**
**  4   Field separators:
**
**        Any character other than +,-,0-9,.,D,E or space may be
**        used to end a field.  Comma is recognized by Dfltin as a
**        special case; it is skipped, leaving the pointer on the
**        next character.  See 12, below.
**
**  5   Both signs are optional.  The default is +.
**
**  6   The mantissa defaults to 1.
**
**  7   The exponent defaults to e0.
**
**  8   The decimal subfields may be of any length.
**
**  9   The decimal point is optional for whole numbers.
**
**  10  A null field is one that does not begin with +,-,0-9,.,D or E,
**      or consists entirely of spaces.  If the field is null, the
**      returned status is 1 and dreslt is left untouched.
**
**  11  nstrt = 1 for the first character in the string.
**
**  12  On return from Dfltin, nstrt is set ready for the next decode -
**      following trailing blanks and (if used) the comma separator.  If
**      a separator other than comma is being used, nstrt must be
**      incremented before the next call to Dfltin.
**
**  13  Errors (status = 2) occur when:
**
**        a)  A +, -, D or E is left unsatisfied.
**        b)  The decimal point is present without at least
**            one decimal subfield.
**        c)  An exponent more than 100 has been presented.
**
**  14  When an error has been detected, nstrt is left pointing to the
**      character following the last one used before the error came to
**      light.  This may be after the point at which a more elaborate
**      algorithm could have detected the error.  For example, Dfltin
**      does not detect that '1e999' is unacceptable until the whole
**      field has been read.
**
**  15  Certain highly unlikely combinations of mantissa and exponent
**      can cause arithmetic faults during the decode, in some cases
**      despite the fact that they together could be construed as a
**      valid number.
**
**  16  Decoding is left to right, one pass.
**
**  17  End of field may occur in either of two ways: (a) as dictated by
**      the string length , (b) detected during the decode;  b overrides
**      a.
**
**  18  See also Intin and Int2in.
**
**  This revision:   2022 March 5
*/

/* Definitions shared between Dfltin and idchf */
#define d_NUMBER 0
#define d_SPACE  1
#define d_EXPSYM 2
#define d_PERIOD 3
#define d_PLUS   4
#define d_MINUS  5
#define d_COMMA  6
#define d_OTHER  7
#define d_END    8

{
   int l_string, nptr, ndigit=0;
   double digit=0.0;

/* Current state of the decode and the values it can take */

   int state;

#define d_seek_sign                       100
#define d_neg_mant                        200
#define d_seek_1st_leading_digit          300
#define d_accept_leading_digit            400
#define d_seek_digit_when_none_before_pt  500
#define d_seek_trailing_digit             600
#define d_accept_trailing_digit           700
#define d_accept_uns_exp_no_mant          800
#define d_seek_sign_exp                   900
#define d_neg_exp                        1000
#define d_seek_1st_exp_digit             1100
#define d_accept_exp_digit               1200
#define d_end_of_field                   1300
#define d_build_result                   1310
#define d_seeking_end_of_field           1620
#define d_next_field_OK                  1720
#define d_next_field_default             9100
#define d_null_field                     9110
#define d_next_field_error               9200
#define d_error                          9210
#define d_done                           9900


   int msign, nexp, ndp, isignx, j=0;
   double dmant;


/* Find string length. */
   l_string = (int) strlen ( string );

/* Current character index. */
   nptr = *nstrt - 1;

/* Set defaults: mantissa+sign, exponent+sign, DP count. */
   dmant = 0.0;
   msign = 1;
   nexp = 0;
   isignx = 1;
   ndp = 0;

/* Initialize state to "looking for sign". */
   state = d_seek_sign;

/* Loop until decode is complete. */
   while ( state != d_done ) {
      switch ( state ) {

      case d_seek_sign:

      /* Look for sign. */
         switch ( idchf ( l_string, string, &nptr, &ndigit, &digit ) ) {
         case d_NUMBER:
            state = d_accept_leading_digit;
            break;
         case d_SPACE:
            state = d_seek_sign;
            break;
         case d_EXPSYM:
            state = d_accept_uns_exp_no_mant;
            break;
         case d_PERIOD:
            state = d_seek_digit_when_none_before_pt;
            break;
         case d_PLUS:
            state = d_seek_1st_leading_digit;
            break;
         case d_MINUS:
            state = d_neg_mant;
            break;
         case d_OTHER:
            state = d_next_field_default;
            break;
         case d_COMMA:
         case d_END:
            state = d_null_field;
            break;
         default:
            state = d_error;
         }
         break;

      case d_neg_mant:

      /* Negative mantissa. */
         msign = -1;

      /* Falls through. */

      case d_seek_1st_leading_digit:

      /* Look for first leading decimal. */
         switch ( idchf ( l_string, string, &nptr, &ndigit, &digit ) ) {
         case d_NUMBER:
            state = d_accept_leading_digit;
            break;
         case d_SPACE:
            state = d_seek_1st_leading_digit;
            break;
         case d_EXPSYM:
            state = d_accept_uns_exp_no_mant;
            break;
         case d_PERIOD:
            state = d_seek_digit_when_none_before_pt;
            break;
         case d_PLUS:
         case d_MINUS:
         case d_COMMA:
         case d_OTHER:
            state = d_next_field_error;
            break;
         case d_END:
         default:
            state = d_error;
         }
         break;

      case d_accept_leading_digit:

      /* Accept leading decimals. */
         dmant = dmant * 1e1 + digit;
         switch ( idchf ( l_string, string, &nptr, &ndigit, &digit ) ) {
         case d_NUMBER:
            state = d_accept_leading_digit;
            break;
         case d_SPACE:
            state = d_build_result;
            break;
         case d_EXPSYM:
            state = d_seek_sign_exp;
            break;
         case d_PERIOD:
            state = d_seek_trailing_digit;
            break;
         case d_PLUS:
         case d_MINUS:
         case d_COMMA:
         case d_OTHER:
            state = d_end_of_field;
            break;
         case d_END:
            state = d_build_result;
            break;
         default:
            state = d_error;
         }
         break;

      case d_seek_digit_when_none_before_pt:

      /* Look for decimal when none preceded the point. */
         switch ( idchf ( l_string, string, &nptr, &ndigit, &digit ) ) {
         case d_NUMBER:
            state = d_accept_trailing_digit;
            break;
         case d_SPACE:
            state = d_seek_digit_when_none_before_pt;
            break;
         case d_EXPSYM:
         case d_PERIOD:
         case d_PLUS:
         case d_MINUS:
         case d_COMMA:
         case d_OTHER:
            state = d_next_field_error;
            break;
         case d_END:
         default:
            state = d_error;
         }
         break;

      case d_seek_trailing_digit:

      /* Look for trailing decimals. */
         switch ( idchf ( l_string, string, &nptr, &ndigit, &digit ) ) {
         case d_NUMBER:
            state = d_accept_trailing_digit;
            break;
         case d_EXPSYM:
            state = d_seek_sign_exp;
            break;
         case d_PERIOD:
         case d_PLUS:
         case d_MINUS:
         case d_COMMA:
         case d_OTHER:
            state = d_end_of_field;
            break;
         case d_SPACE:
         case d_END:
            state = d_build_result;
            break;
         default:
            state = d_error;
         }
         break;

      case d_accept_trailing_digit:

      /* Accept trailing decimals. */
         ndp++;
         dmant = dmant*10.0 + digit;
         state = d_seek_trailing_digit;
         break;

      case d_accept_uns_exp_no_mant:

      /* Exponent symbol first in field: default mantissa to 1. */
         dmant = 1.0;

      /* Falls through. */

      case d_seek_sign_exp:

      /* Look for sign of exponent. */
         switch ( idchf ( l_string, string, &nptr, &ndigit, &digit ) ) {
         case d_NUMBER:
            state = d_accept_exp_digit;
            break;
         case d_SPACE:
            state = d_seek_sign_exp;
            break;
         case d_PLUS:
            state = d_seek_1st_exp_digit;
            break;
         case d_MINUS:
            state = d_neg_exp;
            break;
         case d_EXPSYM:
         case d_PERIOD:
         case d_COMMA:
         case d_OTHER:
            state = d_next_field_error;
            break;
         case d_END:
         default:
            state = d_error;
         }
         break;

      case d_neg_exp:

      /* Exponent negative. */
         isignx = -1;

      /* Falls through. */

      case d_seek_1st_exp_digit:

      /* Look for first digit of exponent. */
         switch ( idchf ( l_string, string, &nptr, &ndigit, &digit ) ) {
         case d_NUMBER:
            state = d_accept_exp_digit;
            break;
         case d_SPACE:
            state = d_seek_1st_exp_digit;
            break;
         case d_EXPSYM:
         case d_PERIOD:
         case d_PLUS:
         case d_MINUS:
         case d_COMMA:
         case d_OTHER:
            state = d_next_field_error;
            break;
         case d_END:
         default:
            state = d_error;
         }
         break;

      case d_accept_exp_digit:

      /* Use exponent digit. */
         nexp = nexp*10 + ndigit;
         if ( nexp > 100 ) {
            state = d_next_field_error;
         } else {

         /* Look for subsequent digits of exponent. */
            switch ( idchf ( l_string, string, &nptr, &ndigit, &digit ) ) {
            case d_NUMBER:
               state = d_accept_exp_digit;
               break;
            case d_SPACE:
               state = d_build_result;
               break;
            case d_EXPSYM:
            case d_PERIOD:
            case d_PLUS:
            case d_MINUS:
            case d_COMMA:
            case d_OTHER:
               state = d_end_of_field;
               break;
            case d_END:
               state = d_build_result;
               break;
            default:
               state = d_error;
            }
         }
         break;

      case d_end_of_field:

      /* Off the end of the field: move pointer back. */
         nptr--;

      /* Falls through. */

      case d_build_result:

      /* Combine exponent and decimal place count. */
         nexp = nexp*isignx - ndp;

      /* Sign of exponent? */
         if ( nexp >= 0 ) {

         /* Positive exponent: scale up. */
            while ( nexp >= 10 ) {
               dmant *= 1e10;
               nexp -= 10;
            }
            while ( nexp >= 1 ) {
               dmant *= 1e1;
               nexp--;
            }
         } else {

         /* Negative exponent: scale down. */
            while ( nexp <= -10 ) {
               dmant /= 1e10;
               nexp += 10;
            }
            while ( nexp <= -1 ) {
               dmant /= 1e1;
               nexp++;
            }
         }

      /* Get result & status. */
         if ( msign == 1 ) {
             *dreslt = dmant;
             j = 0;
         } else {
             *dreslt = -dmant;
             j = -1;
         }

      /* Falls through. */

      case d_seeking_end_of_field:

      /* Skip to end of field. */
         switch ( idchf ( l_string, string, &nptr, &ndigit, &digit ) ) {
         case d_SPACE:
            state = d_seeking_end_of_field;
            break;
         case d_NUMBER:
         case d_EXPSYM:
         case d_PERIOD:
         case d_PLUS:
         case d_MINUS:
         case d_OTHER:
            state = d_next_field_OK;
            break;
         case d_COMMA:
         case d_END:
            state = d_done;
            break;
         default:
            state = d_error;
         }
         break;

      case d_next_field_OK:

      /* Next field terminates successful decode. */
         nptr--;
         state = d_done;
         break;

      case d_next_field_default:

      /* Next field terminates null decode. */
         nptr--;

      /* Falls through. */

      case d_null_field:

      /* Null decode. */
         j = 1;
         state = d_done;
         break;

      case d_next_field_error:

      /* Next field detected prematurely. */
         nptr--;

      /* Falls through. */

      case d_error:

      /* Decode has failed: set bad status. */
         j = 2;
         state = d_done;
         break;

      default:
         state = d_error;
      }
   }

/* Finished: return updated pointer and the status. */
   *nstrt = nptr + 1;
   return j;
}

static int idchf ( int l_string, const char *string,
                   int *nptr, int *ndigit, double *digit )
/*
**  - - - - - -
**   i d c h f
**  - - - - - -
**
**  Internal function used by Dfltin:
**
**  identify next character in string.
**
**  Given:
**     l_string    int         length of string
**     string      char*       string
**     nptr        int*        character to be identified (1st = 0)
**
**  Returned:
**     nptr        int*        incremented unless end of field
**     ndigit      int*        0-9 if character was a numeral
**     digit       double*     (double) ndigit
**
**  Returned (function value):
**     idchf       int         vector for identified character:
**
**                                  value   meaning
**
**                                d_NUMBER  0-9
**                                d_SPACE   space or tab
**                                d_EXPSYM  D, d, E or e
**                                d_PERIOD  .
**                                d_PLUS    +
**                                d_MINUS   -
**                                d_COMMA   ,
**                                d_OTHER   else
**                                d_END     outside field
**
**  This revision:   2022 January 30
*/
{
   int ivec, ictab;
   char c;

/* Character/vector tables */

#define NCRECD (20)
   static char kctab[NCRECD] = { '0','1','2','3','4','5',
                                 '6','7','8','9',
                                 ' ','\t',
                                 'D','d','E','e',
                                 '.',
                                 '+',
                                 '-',
                                 ',' };

   static int kvtab[NCRECD] = { d_NUMBER, d_NUMBER, d_NUMBER, d_NUMBER,
                                d_NUMBER, d_NUMBER, d_NUMBER, d_NUMBER,
                                d_NUMBER, d_NUMBER,
                                d_SPACE, d_SPACE,
                                d_EXPSYM, d_EXPSYM, d_EXPSYM, d_EXPSYM,
                                d_PERIOD,
                                d_PLUS,
                                d_MINUS,
                                d_COMMA };


/* Initialize returned value. */
   ivec = d_OTHER;

/* Pointer outside field? */
   if ( *nptr < 0 || *nptr >= l_string ) {

   /* Yes: prepare returned value. */
      ivec = d_END;

   } else {

   /* Not end of field: identify character. */
      c = string [ *nptr ];
      for ( ictab = 0; ictab < NCRECD; ictab++ ) {
         if ( kctab [ ictab ] == c ) {

         /* Recognized. */
            ivec = kvtab [ ictab ];

         /* Allow for numerals. */
            *ndigit = ictab;
            *digit = (double) *ndigit;

         /* Quit the loop. */
            break;
         }
      }

   /* Increment pointer. */
      ( *nptr )++;
   }

/* Return the value identifying the character. */
   return ivec;
}

/*--------------------------------------------------------------------*/

#include <limits.h>

int Int2in ( const char *string, int *nstrt, int *ireslt )
/*
**  - - - - - - -
**   I n t 2 i n
**  - - - - - - -
**
**  Convert free-format input into an integer.
**
**  Given:
**     string    char*    string containing number to be decoded
**     nstrt     int*     where to start decode (1st = 1)
**     ireslt    long*    current value of result
**
**  Returned:
**     nstrt     int*     advanced to next number
**     ireslt    int*     result
**
**  Returned (function value):
**               int*     status: -1 = -OK
**                                 0 = +OK
**                                +1 = null
**                                +2 = error
**
**  Notes:
**
**  1   The reason Int2in has separate OK status values for + and - is
**      to enable minus zero to be detected.   This is of crucial
**      importance when decoding mixed-radix numbers.  For example, an
**      angle expressed as deg, arcmin, arcsec may have a leading minus
**      sign but a zero degrees field.
**
**  2   A TAB is interpreted as a space.
**
**  3   The basic format is the sequence of fields #^, where # is a sign
**      character + or -, and ^ means a string of decimal digits.
**
**  4   Spaces:
**
**        .  Leading spaces are ignored.
**
**        .  Spaces between the sign and the number are allowed.
**
**        .  Trailing spaces are ignored;  the first signifies end
**           of decoding and subsequent ones are skipped.
**
**  5   Delimiters:
**
**        .  Any character other than +,-,0-9 or space may be used
**           to signal the end of the number and terminate
**           decoding.
**
**        .  Comma is recognized by Int2in as a special case;  it
**           is skipped, leaving the pointer on the next character.
**           See 9, below.
**
**  6   The sign is optional.  The default is +.
**
**  7   A "null result" occurs when the string of characters being
**      decoded does not begin with +,- or 0-9, or consists entirely of
**      spaces.  When this condition is detected, the returned status is
**      1 and ireslt is left untouched.
**
**  8   nstrt = 1 for the first character in the string.
**
**  9   On return from Int2in, nstrt is set ready for the next decode -
**      following trailing blanks and any comma.  If a delimiter other
**      than comma is being used, nstrt must be incremented before the
**      next call to Int2in, otherwise all subsequent calls will return
**      a null result.
**
**  10  Errors (status = 2) occur when:
**
**        .  there is a + or - but no number;  or
**
**        .  the number is larger than INT_MAX.
**
**  11  When an error has been detected, nstrt is left pointing to the
**      character following the last one used before the error came to
**      light.
**
**  12  See also Intin and Dfltin.
**
**  Called:  Intin
**
**  This revision:   2022 March 5
*/

{
   int j;
   long lreslt;


/* Decode a long integer. */
   lreslt = (long) *ireslt;
   j = Intin ( string, nstrt, &lreslt );

/* If OK, validate length. */
   if ( j < 2 ) {
      if ( lreslt >= (long) INT_MIN &&
           lreslt <= (long) INT_MAX ) {

      /* OK: cast the result to int. */
         *ireslt = (int) lreslt;

      } else {

      /* Number outside int range: error status. */
         j = 2;
      }
   }

/* Return the status. */
   return j;
}

/*--------------------------------------------------------------------*/

#include <string.h>
#include <limits.h>

static int idchi ( int, const char*, int*, double* );

int Intin ( const char *string, int *nstrt, long *ireslt )
/*
**  - - - - - -
**   I n t i n
**  - - - - - -
**
**  Convert free-format input into a long integer.
**
**  Given:
**     string    char*    string containing number to be decoded
**     nstrt     int*     where to start decode (1st = 1)
**     ireslt    long*    current value of result
**
**  Returned:
**     nstrt     int*     advanced to next number
**     ireslt    long*    result
**
**  Returned (function value):
**               int*     status: -1 = -OK
**                                 0 = +OK
**                                +1 = null
**                                +2 = error
**
**  Called:  idchi
**
**  Notes:
**
**  1   The reason Intin has separate OK status values for + and - is to
**      enable minus zero to be detected.   This is of crucial
**      importance when decoding mixed-radix numbers.  For example, an
**      angle expressed as deg, arcmin, arcsec may have a leading minus
**      sign but a zero degrees field.
**
**  2   A TAB is interpreted as a space.
**
**  3   The basic format is the sequence of fields #^, where # is a sign
**      character + or -, and ^ means a string of decimal digits.
**
**  4   Spaces:
**
**        .  Leading spaces are ignored.
**
**        .  Spaces between the sign and the number are allowed.
**
**        .  Trailing spaces are ignored;  the first signifies
**           end of decoding and subsequent ones are skipped.
**
**  5   Delimiters:
**
**        .  Any character other than +,-,0-9 or space may be
**           used to signal the end of the number and terminate
**           decoding.
**
**        .  Comma is recognized by Intin as a special case;  it
**           is skipped, leaving the pointer on the next character.
**           See 9, below.
**
**  6   The sign is optional.  The default is +.
**
**  7   A "null result" occurs when the string of characters being
**      decoded does not begin with +,- or 0-9, or consists entirely of
**      spaces.  When this condition is detected, the returned status is
**      1 and ireslt is left untouched.
**
**  8   nstrt = 1 for the first character in the string.
**
**  9   On return from Intin, nstrt is set ready for the next decode -
**      following trailing blanks and any comma.  If a delimiter other
**      than comma is being used, nstrt must be incremented before the
**      next call to Intin, otherwise all subsequent calls will return a
**      null result.
**
**  10  Errors (status = 2) occur when:
**
**        .  there is a + or - but no number;  or
**
**        .  the number is larger than LONG_MAX.
**
**  11  When an error has been detected, nstrt is left pointing to the
**      character following the last one used before the error came to
**      light.
**
**  12  See also Int2in and Dfltin.
**
**  This revision:   2022 March 5
*/

/* Definitions shared between Intin and idchi */
#define i_NUMBER 0
#define i_SPACE  1
#define i_PLUS   2
#define i_MINUS  3
#define i_COMMA  4
#define i_OTHER  5
#define i_END    6

{
   int l_string, nptr;
   double digit=0.0;

/* Current state of the decode and the values it can take */

   int state;

#define i_seek_sign                       100
#define i_neg                             200
#define i_seek_1st_digit                  300
#define i_accept_digit                    400
#define i_seek_digit                      410
#define i_end_of_field                   1600
#define i_build_result                   1610
#define i_seeking_end_of_field           1630
#define i_next_field_OK                  1720
#define i_next_field_default             9100
#define i_null_field                     9110
#define i_next_field_error               9200
#define i_error                          9210
#define i_done                           9900


   int j;
   double dres;


/* Find string length. */
   l_string = (int) strlen ( string );

/* Current character index (1st = 0). */
   nptr = *nstrt - 1;

/* Set defaults: result & sign. */
   dres = 0.0;
   j = 0;

/* Initialize state to "looking for sign". */
   state = i_seek_sign;

/* Loop until decode is complete. */
   while ( state != i_done ) {
      switch ( state ) {

      case i_seek_sign:

      /* Look for sign */
         switch ( idchi ( l_string, string, &nptr, &digit ) ) {
         case i_NUMBER:
            state = i_accept_digit;
            break;
         case i_SPACE:
            state = i_seek_sign;
            break;
         case i_PLUS:
            state = i_seek_1st_digit;
            break;
         case i_MINUS:
            state = i_neg;
            break;
         case i_OTHER:
            state = i_next_field_default;
            break;
         case i_COMMA:
         case i_END:
            state = i_null_field;
            break;
         default:
            state = i_error;
         }
         break;

      case i_neg:

      /* Negative result. */
         j = -1;

      /* Falls through. */

      case i_seek_1st_digit:

      /* Look for first leading decimal. */
         switch ( idchi ( l_string, string, &nptr, &digit ) ) {
         case i_NUMBER:
            state = i_accept_digit;
            break;
         case i_SPACE:
            state = i_seek_1st_digit;
            break;
         case i_PLUS:
         case i_MINUS:
         case i_COMMA:
         case i_OTHER:
            state = i_next_field_error;
            break;
         case i_END:
         default:
            state = i_error;
         }
         break;

      case i_accept_digit:

      /* Accept decimals. */
         dres = dres*10.0 + digit;
         state = ( dres >= (double) LONG_MIN &&
                   dres <= (double) LONG_MAX ) ? i_seek_digit :
                                                 i_next_field_error;
         break;

      case i_seek_digit:

      /* Look for next decimal. */
         switch ( idchi ( l_string, string, &nptr, &digit ) ) {
         case i_NUMBER:
            state = i_accept_digit;
            break;
         case i_SPACE:
            state = i_build_result;
            break;
         case i_PLUS:
         case i_MINUS:
         case i_COMMA:
         case i_OTHER:
            state = i_end_of_field;
            break;
         case i_END:
            state = i_build_result;
            break;
         default:
            state = i_error;
         }
         break;

      case i_end_of_field:

      /* Off the end of the field: move pointer back. */
         nptr--;

      /* Falls through. */

      case i_build_result:

      /* Make the result. */
         if ( j ) dres = - dres;
         *ireslt = (long) round(dres);

      /* Falls through. */

      case i_seeking_end_of_field:

      /* Skip to end of field */
         switch ( idchi ( l_string, string, &nptr, &digit ) ) {
         case i_SPACE:
            state = i_seeking_end_of_field;
            break;
         case i_NUMBER:
         case i_PLUS:
         case i_MINUS:
         case i_OTHER:
            state = i_next_field_OK;
            break;
         case i_COMMA:
         case i_END:
            state = i_done;
            break;
         default:
            state = i_error;
         }
         break;

      case i_next_field_OK:

      /* Next field terminates successful decode. */
         nptr--;
         state = i_done;
         break;

      case i_next_field_default:

      /* Next field terminates null decode. */
         nptr--;

      /* Falls through. */

      case i_null_field:

      /* Null decode. */
         j = 1;
         state = i_done;
         break;

      case i_next_field_error:

      /* Next field detected prematurely. */
         nptr--;

      /* Falls through. */

      case i_error:

      /* Decode has failed: set bad status. */
         j = 2;
         state = i_done;
         break;

      default:
         state = i_error;
      }
   }

/* Finished: return updated pointer and the status */
   *nstrt = nptr + 1;
   return j;
}

static int idchi ( int l_string, const char *string, int *nptr,
                   double *digit )
/*
**  - - - - - -
**   i d c h i
**  - - - - - -
**
**  Internal function used by Intin:
**
**  identify next character in string.
**
**  Given:
**     l_string    int         length of string
**     string      char*       string
**     nptr        int*        character to be identified (1st = 0)
**
**  Returned:
**     nptr        int*        incremented unless end of field
**     digit       double*     0.0 - 9.0 if character was a numeral
**
**  Returned (function value):
**                 int         vector for identified character:
**
**                                  value   meaning
**
**                                i_NUMBER  0-9
**                                i_SPACE   space or tab
**                                i_PLUS    +
**                                i_MINUS   -
**                                i_COMMA   ,
**                                i_OTHER   else
**                                i_END     outside field
**
**  This revision:   2022 January 31
*/
{
   int ivec, ictab;
   char c;

/* Character/vector tables */

#define NCRECI (15)
   static char kctab[NCRECI] = { '0','1','2','3','4','5',
                                 '6','7','8','9',
                                 ' ','\t',
                                 '+',
                                 '-',
                                 ',' };

   static int kvtab[NCRECI] = { i_NUMBER, i_NUMBER, i_NUMBER, i_NUMBER,
                                i_NUMBER, i_NUMBER, i_NUMBER, i_NUMBER,
                                i_NUMBER, i_NUMBER,
                                i_SPACE, i_SPACE,
                                i_PLUS,
                                i_MINUS,
                                i_COMMA };


/* Initialize returned value. */
   ivec = i_OTHER;

/* Pointer outside field? */
   if ( *nptr < 0 || *nptr >= l_string ) {

   /* Yes: prepare returned value. */
      ivec = i_END;

   } else {

   /* Not end of field: identify character. */
      c = string [ *nptr ];
      for ( ictab = 0; ictab < NCRECI; ictab++ ) {
         if ( kctab [ ictab ] == c ) {

         /* Recognized. */
            ivec = kvtab [ ictab ];

         /* Allow for numerals. */
            *digit = (double) ictab;

         /* Quit the loop. */
            break;
         }
      }

   /* Increment pointer. */
      ( *nptr )++;
   }

/* Return the value identifying the character. */
   return ivec;
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
