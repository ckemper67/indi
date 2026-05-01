#include "eph.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/*
**  - - - - - - - - - - -
**   p l a n c _ i n d i
**  - - - - - - - - - - -
**
**  Load a packed INDI planetary ephemeris context (.ictx) into ephPLANctx.
**
**  Given:
**     ibody   int          body index (1=Mercury .. 8=Neptune)
**     path    char*        directory path for .ictx files
**
**  Returned:
**     c       ephPLANctx*  context structure (compatible with ephPlanet)
**
**  Returned (function value):
**             int          status:
**                              0   OK
**                             -1   illegal ibody
**                             -2   file path too long
**                             -3   file open error
**                             -4   bad magic / wrong version
**                             -5   ibody mismatch
**                             -6   nterms exceeds MAXTERM
**                           else   file read error
**
**  The .ictx format is a compact variable-length binary produced by
**  indi_eph_packer. It contains only the VSOP2010 terms that survive
**  a truncation filter and loads directly into the standard ephPLANctx
**  struct so ephPlanet / ephRdplan require no changes.
**
**  Copyright (C) 2025 INDI Contributors.
**  Released under LGPL 2.1 or later.
*/

#define ICTX_MAGIC   0x49435458u   /* "ICTX" */
#define ICTX_VERSION 2u

#define LFILE 20
static const char* ictx_names[] = {
    "VSOP2013_1.ictx",   /* Mercury */
    "VSOP2013_2.ictx",   /* Venus   */
    "VSOP2013_3.ictx",   /* EMB     */
    "VSOP2013_4.ictx",   /* Mars    */
    "VSOP2013_5.ictx",   /* Jupiter */
    "VSOP2013_6.ictx",   /* Saturn  */
    "VSOP2013_7.ictx",   /* Uranus  */
    "VSOP2013_8.ictx",   /* Neptune */
};

int ephPlanci(int ibody, char* path, ephPLANctx* c)
{
#define LS 256
    char fname[LS];
    int lpath, it, iv, nn, j, nterms, file_ibody;
    uint32_t magic, version;
    double threshold, reserved;
    FILE* fp;
    int16_t iphi_buf[MAXARG];
    double ss_buf, cc_buf;
    int16_t limit_buf[MAXTIME+1][6];

    if (ibody < 1 || ibody > 8) return -1;

    lpath = (int)strlen(path);
    if (lpath + LFILE >= LS) return -2;
    strncpy(fname, path, LS-1);
    strncat(fname, ictx_names[ibody-1], LS-1-lpath);

    fp = fopen(fname, "rb");
    if (!fp) return -3;

    /* Read and validate header */
    if (fread(&magic,     sizeof(uint32_t), 1, fp) != 1 ||
        fread(&version,   sizeof(uint32_t), 1, fp) != 1 ||
        fread(&file_ibody,sizeof(int32_t),  1, fp) != 1 ||
        fread(&nterms,    sizeof(int32_t),  1, fp) != 1 ||
        fread(&threshold, sizeof(double),   1, fp) != 1 ||
        fread(&reserved,  sizeof(double),   1, fp) != 1) {
        fclose(fp); return -7;
    }
    if (magic != ICTX_MAGIC || version != ICTX_VERSION) { fclose(fp); return -4; }
    if (file_ibody != ibody)                             { fclose(fp); return -5; }
    if (nterms > MAXTERM)                                { fclose(fp); return -6; }

    /* Read metadata */
    if (fread(c->receq,   sizeof(double), 9,       fp) != 9       ||
        fread(&c->rgm,    sizeof(double), 1,       fp) != 1       ||
        fread(c->ci0,     sizeof(double), MAXARG,  fp) != MAXARG  ||
        fread(c->ci1,     sizeof(double), MAXARG,  fp) != MAXARG  ||
        fread(c->freqpla, sizeof(double), 8,       fp) != 8) {
        fclose(fp); return -7;
    }

    /* Read limit table */
    if (fread(limit_buf, sizeof(int16_t), (MAXTIME+1)*6, fp) != (size_t)((MAXTIME+1)*6)) {
        fclose(fp); return -7;
    }
    for (it = 0; it <= MAXTIME; it++)
        for (iv = 0; iv < 6; iv++)
            c->limit[it][iv] = limit_buf[it][iv];

    /* Read term records sequentially */
    for (nn = 0; nn < nterms; nn++) {
        if (fread(iphi_buf, sizeof(int16_t), MAXARG, fp) != MAXARG ||
            fread(&ss_buf,  sizeof(double),  1,      fp) != 1      ||
            fread(&cc_buf,  sizeof(double),  1,      fp) != 1) {
            fclose(fp); return -7;
        }
        for (j = 0; j < MAXARG; j++)
            c->iphi[nn][j] = iphi_buf[j];
        c->ss[nn] = ss_buf;
        c->cc[nn] = cc_buf;
    }

    fclose(fp);
    c->ibody = (short)ibody;
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
