#include "eph.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/*
**  - - - - - - - - - - -
**   t o p c _ i n d i
**  - - - - - - - - - - -
**
**  Load a packed INDI TOP2013 ephemeris context (.tictx) into ephTOPctx.
**
**  Given:
**     ibody   int          body index (5=Jupiter .. 8=Neptune)
**     path    char*        directory path for .tictx files
**
**  Returned:
**     c       ephTOPctx*   context structure (compatible with ephTopPlanet)
**
**  Returned (function value):
**             int          status:
**                              0   OK
**                             -1   illegal ibody
**                             -2   file path too long
**                             -3   file open error
**                             -4   bad magic / wrong version
**                             -5   ibody mismatch
**                             -6   nterms exceeds MAXTERM_TOP
**                           else   file read error
**
**  The .tictx format is produced by indi_eph_packer --top and contains
**  only the TOP2013 terms that survive the amplitude filter.
**
**  Copyright (C) 2025 INDI Contributors.
**  Released under LGPL 2.1 or later.
*/

#define TICTX_MAGIC   0x54494354u   /* "TICT" */
#define TICTX_VERSION 1u

#define LFILE_TOP 18
static const char* tictx_names[] = {
    "TOP2013_5.tictx",   /* Jupiter */
    "TOP2013_6.tictx",   /* Saturn  */
    "TOP2013_7.tictx",   /* Uranus  */
    "TOP2013_8.tictx",   /* Neptune */
};

int ephTopci ( int ibody, char* path, ephTOPctx* c )
{
#define LS_TOP 256
    char fname[LS_TOP];
    int lpath, it, iv, nn, nterms, file_ibody;
    uint32_t magic, version;
    double threshold, reserved;
    FILE* fp;
    int16_t limit_buf[MAXTIME_TOP+1][6];

    if ( ibody < 5 || ibody > 8 ) return -1;

    lpath = (int) strlen(path);
    if ( lpath + LFILE_TOP >= LS_TOP ) return -2;
    strncpy ( fname, path, LS_TOP-1 );
    strncat ( fname, tictx_names[ibody-5], LS_TOP-1-lpath );

    fp = fopen ( fname, "rb" );
    if ( !fp ) return -3;

    /* Read and validate header */
    if ( fread(&magic,      sizeof(uint32_t), 1, fp) != 1 ||
         fread(&version,    sizeof(uint32_t), 1, fp) != 1 ||
         fread(&file_ibody, sizeof(int32_t),  1, fp) != 1 ||
         fread(&nterms,     sizeof(int32_t),  1, fp) != 1 ||
         fread(&threshold,  sizeof(double),   1, fp) != 1 ||
         fread(&reserved,   sizeof(double),   1, fp) != 1 ) {
        fclose(fp); return -7;
    }
    if ( magic != TICTX_MAGIC || version != TICTX_VERSION ) { fclose(fp); return -4; }
    if ( file_ibody != ibody )                               { fclose(fp); return -5; }
    if ( nterms > MAXTERM_TOP )                              { fclose(fp); return -6; }

    /* Read scalar metadata */
    if ( fread(c->receq, sizeof(double),  9, fp) != 9 ||
         fread(&c->rgm,  sizeof(double),  1, fp) != 1 ||
         fread(&c->dmu,  sizeof(double),  1, fp) != 1 ||
         fread(&c->freq, sizeof(double),  1, fp) != 1 ) {
        fclose(fp); return -7;
    }

    /* Read limit table */
    if ( fread(limit_buf, sizeof(int16_t), (MAXTIME_TOP+1)*6, fp) != (size_t)((MAXTIME_TOP+1)*6) ) {
        fclose(fp); return -7;
    }
    for ( it = 0; it <= MAXTIME_TOP; it++ )
        for ( iv = 0; iv < 6; iv++ )
            c->limit[it][iv] = limit_buf[it][iv];

    /* Read term records: integer multiplier + cos + sin */
    for ( nn = 0; nn < nterms; nn++ ) {
        int32_t mk;
        double cv, sv;
        if ( fread(&mk, sizeof(int32_t), 1, fp) != 1 ||
             fread(&cv, sizeof(double),  1, fp) != 1 ||
             fread(&sv, sizeof(double),  1, fp) != 1 ) {
            fclose(fp); return -7;
        }
        c->m[nn] = (int) mk;
        c->c[nn] = cv;
        c->s[nn] = sv;
    }

    fclose(fp);
    c->ibody = (short) ibody;
    c->init  = 1;
    return 0;
}

/*----------------------------------------------------------------------
**
**  Copyright (C) 2025 INDI Contributors
**
**  This library is free software; you can redistribute it and/or
**  modify it under the terms of the GNU Lesser General Public License
**  as published by the Free Software Foundation; either version 2.1
**  of the License, or (at your option) any later version.
**
**--------------------------------------------------------------------*/
