/*
** erfa_libnova.c - ERFA astrometry context shim for the INDI SPK Math Plugin
**
** Provides the eraASTROM context functions that vtel.c needs but that we do
** not build from the vendored ERFA sources.  The SPK plugin uses APPT
** coordinates throughout, so the ICRS transform chain (eraAtciqz, eraAticq)
** is never exercised; those are provided as no-op stubs for the linker.
**
** eraAper and eraAper13 are included here for completeness (they are called
** from astr.c) but astr.c is not compiled; SPKMathPlugin.cpp populates the
** eraASTROM fields directly from libnova LST and site coordinates instead.
**
** Copyright notice: this file is original INDI code, not derived from ERFA
** or SOFA.
*/

#include "erfa.h"

/* -----------------------------------------------------------------------
** eraAper - update the "local Earth rotation angle" field of an eraASTROM.
**
** theta  double      Earth rotation angle (radians, UT1-based)
** astrom eraASTROM*  astrometry parameters; eral is updated
*/
void eraAper(double theta, eraASTROM *astrom)
{
    astrom->eral = theta + astrom->along;
}

/* -----------------------------------------------------------------------
** eraAper13 - update eral for a given UT1 (two-part Julian date).
**
** ut11, ut12  double   UT1 as two-part Julian date
** astrom      eraASTROM*
*/
void eraAper13(double ut11, double ut12, eraASTROM *astrom)
{
    eraAper(eraEra00(ut11, ut12), astrom);
}

/* -----------------------------------------------------------------------
** eraAtciqz - ICRS astrometric to CIRS.  Stub: never called because the
** SPK plugin uses APPT targets, not ICRS.
*/
void eraAtciqz(double rc, double dc, eraASTROM *astrom,
               double *ri, double *di)
{
    (void)astrom;
    *ri = rc;
    *di = dc;
}

/* -----------------------------------------------------------------------
** eraAticq - CIRS to ICRS astrometric.  Stub: never called because the
** SPK plugin uses APPT targets, not ICRS.
*/
void eraAticq(double ri, double di, eraASTROM *astrom,
              double *rc, double *dc)
{
    (void)astrom;
    *rc = ri;
    *dc = di;
}
