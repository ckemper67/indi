#ifndef EPHEM
#define EPHEM

/*
**  - - - - - -
**   s s e . h
**  - - - - - -
**
**  Header file for functions associated with SOFA-based solar system
**  ephemeris tools.
**
**  This revision:   2021 September 27
**
**  Copyright P.T.Wallace.  All rights reserved.
*/

#include <math.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Constants (IAU 2009/2012 as relevant) */
#define CPI (3.141592653589793238462643)   /* pi */
#define C2PI (6.283185307179586476925287)   /* 2pi */
#define D90 (1.570796326794896619231)   /* pi/2 */
#define R2D (57.29577951308232)   /* radians to degrees */
#define R2AS (206264.80624709635515647)   /* radians to arcsec */
#define AS2R (4.848136811095359935899141e-6)   /* arcsec to radians */
#define DAYSEC (86400.0)   /* seconds per day */
#define DJC (36525.0)   /* days per Julian century */
#define DJMLA (365250.0)   /* days per Julian millennium */
#define DJM0 (2400000.5)   /* JD for zero MJD */
#define DJ2000 (2451545.0)   /* J2000.0 JD */
#define EP2000 (51544.5)   /* J2000.0 MJD */
#define AUKM (149597870.700)   /* AU in km */
#define TAU (499.0047838061)   /* light time for unit distance (sec) */
#define RMME (1.23000371e-2)   /* mass ratio Moon/Earth */

/* Degrees, arcmin, arcsec to radians */
#define DMS(D,M,S) ((60.0*((double)(60*D+M))+S)*AS2R)

/* Context for planet computations */
#define MAXTERM 351000
#define MAXARG 17
#define MAXTIME 20
typedef struct {
   short init, ibody;
   double receq[3][3], rgm, ci0[MAXARG], ci1[MAXARG], freqpla[8];
   short limit[MAXTIME+1][6], iphi[MAXTERM][MAXARG];
   double ss[MAXTERM], cc[MAXTERM];
} ephPLANctx;

/*  Context for lunar computations */
#define MAX1 2645
#define MAX2 33256
typedef struct {
   short jftf;
   double receq[3][3], w[5][3], eart[5], peri[5], zeta[5], del[5][4],
          p[5][8], delnu, dele, delg, delnp, delep, dtasm, am,
          p1, p2, p3, p4, p5, q1, q2, q3, q4, q5,
          cmpb[MAX1], cper[MAX2], fmpb[MAX1][5], fper[MAX2][5];
   int nmpb[3][3], nper[3][4][3];
} ephMOONctx;

/* Function prototypes */
FILE* ephBopr ( char*, char*, int* );
int ephEarth ( double, ephMOONctx*, ephPLANctx*, double[2][3] );
int ephMoon ( ephMOONctx*, double, double[2][3] );
int ephMoonc ( char*, int, ephMOONctx* );
int ephMooni ( char*, int, ephMOONctx* );
int ephPlanc  ( int, char*, ephPLANctx* );
int ephPlanci ( int, char*, ephPLANctx* );
int ephPlanet ( int, ephPLANctx*, double, double[2][3] );
int ephPlani ( char*, int, ephPLANctx* );
int ephRdplan ( ephMOONctx*, ephPLANctx*, ephPLANctx*,
                double, double, int, double, double, double,
                double*, double*,
                double*, double*, double*, double* );
int ephRdplanq ( double[2][3], double[2][3], ephPLANctx*,
                 double, double, int, double, double, double,
                 double*, double*,
                 double*, double*, double*, double* );

#ifdef __cplusplus
}
#endif

#endif
