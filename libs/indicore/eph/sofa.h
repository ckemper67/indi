/*
** sofa.h - Comprehensive SOFA-over-ERFA compatibility shim for INDI
**
** Maps the SOFA iauXxx API onto the ERFA eraXxx API so that the EPH and SPK
** libraries can be compiled unchanged against the system liberfa library.
*/

#ifndef SOFA_H
#define SOFA_H

#include <erfa.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Type aliases */
typedef eraASTROM iauASTROM;
typedef eraLDBODY iauLDBODY;

/* Function name mappings: iau* -> era* */
#define iauA2af    eraA2af
#define iauA2tf    eraA2tf
#define iauAe2hd   eraAe2hd
#define iauAf2a    eraAf2a
#define iauAnp     eraAnp
#define iauAnpm    eraAnpm
#define iauApco13  eraApco13
#define iauAper    eraAper
#define iauAper13  eraAper13
#define iauAtciqz  eraAtciqz
#define iauAticq   eraAticq
#define iauAtioq   eraAtioq
#define iauAtoiq   eraAtoiq
#define iauC2s     eraC2s
#define iauCal2jd  eraCal2jd
#define iauCp      eraCp
#define iauD2tf    eraD2tf
#define iauDtdb    eraDtdb
#define iauDtf2d   eraDtf2d
#define iauEra00   eraEra00
#define iauHd2ae   eraHd2ae
#define iauIr      eraIr
#define iauJd2cal  eraJd2cal
#define iauPas     eraPas
#define iauPm      eraPm
#define iauPmp     eraPmp
#define iauPpp     eraPpp
#define iauPvmpv   eraPvmpv
#define iauPvppv   eraPvppv
#define iauPvtob   eraPvtob
#define iauRx      eraRx
#define iauRxp     eraRxp
#define iauRy      eraRy
#define iauRz      eraRz
#define iauS2c     eraS2c
#define iauSeps    eraSeps
#define iauSxp     eraSxp
#define iauSxpv    eraSxpv
#define iauTaitt   eraTaitt
#define iauTaiutc  eraTaiutc
#define iauTf2d    eraTf2d
#define iauTrxp    eraTrxp
#define iauTttdb   eraTttdb
#define iauUtctai  eraUtctai
#define iauUtcut1  eraUtcut1
#define iauZp      eraZp
#define iauZpv     eraZpv

#ifdef __cplusplus
}
#endif

#endif /* SOFA_H */
