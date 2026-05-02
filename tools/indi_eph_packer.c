/*
**  i n d i _ e p h _ p a c k e r
**
**  Produce compact INDI packed ephemeris (.ictx) files from the full
**  VSOP2013 binary .ctx files.
**
**  Usage:
**     indi_eph_packer <ctx_dir> <output_dir> [threshold [max_tm]]
**
**  Arguments:
**     ctx_dir     directory containing VSOP2013_#.ctx files
**     output_dir  directory for output VSOP2013_#.ictx files
**     threshold   amplitude filter (AU); default 1e-9
**     max_tm      target epoch limit in Julian millennia from J2000;
**                 default 0.0 (no time weighting).
**                 When non-zero, a term at time power 'it' is kept only
**                 if hypot(ss,cc) * max_tm^it >= threshold.
**                 Example: 0.2 covers +-200 years from J2000 (1800-2200).
**
**  The .ictx format stores only VSOP2013 terms whose effective amplitude
**  survives the filter, in a compact variable-length binary.
**  The loader ephPlanci() reads .ictx directly into ephPLANctx so
**  ephPlanet / ephRdplan require no changes.
**
**  Copyright (C) 2025 INDI Contributors.
**  Released under LGPL 2.1 or later.
*/

#include "../libs/indicore/eph/eph.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#define ICTX_MAGIC   0x49435458u   /* "ICTX" */
#define ICTX_VERSION 2u

/* Survival test: keep term if its contribution at max_tm is >= threshold.
   When max_tm == 0.0, only raw amplitude is checked (no time weighting). */
static int survives(double ss, double cc, int it, double threshold, double max_tm)
{
    double amp = hypot(ss, cc);
    if (max_tm == 0.0 || it == 0)
        return amp >= threshold;
    double tpow = pow(max_tm, (double)it);
    return amp * tpow >= threshold;
}

static int pack_planet(int ibody, const char* ctx_dir, const char* out_dir,
                       double threshold, double max_tm, ephPLANctx* c)
{
    char ctx_path[512], out_path[512];
    snprintf(ctx_path, sizeof(ctx_path), "%s/", ctx_dir);
    snprintf(out_path, sizeof(out_path), "%s/VSOP2013_%d.ictx", out_dir, ibody);

    /* Load the full binary context */
    int status = ephPlanc(ibody, ctx_path, c);
    if (status != 0) {
        fprintf(stderr, "  ephPlanc(%d) failed: %d\n", ibody, status);
        return status;
    }

    /* Count and collect surviving terms.
       planet.c iterates: for iv in [0,6), for it in [0,MAXTIME]:
         for n in [1, limit[it][iv]]: use iphi[nn], ss[nn], cc[nn]; nn++
       We must maintain this exact ordering when compacting. */

    /* First pass: count survivors per slot and globally */
    int new_limit[MAXTIME+1][6];
    int it, iv, nn, n;
    int total_in = 0, total_out = 0;

    for (iv = 0; iv < 6; iv++)
        for (it = 0; it <= MAXTIME; it++)
            new_limit[it][iv] = 0;

    nn = 0;
    for (iv = 0; iv < 6; iv++) {
        for (it = 0; it <= MAXTIME; it++) {
            int cnt = c->limit[it][iv];
            for (n = 0; n < cnt; n++) {
                if (survives(c->ss[nn], c->cc[nn], it, threshold, max_tm)) {
                    new_limit[it][iv]++;
                    total_out++;
                }
                total_in++;
                nn++;
            }
        }
    }

    /* Second pass: compact surviving terms into temporary arrays */
    short*  iphi_out = malloc((size_t)total_out * MAXARG * sizeof(short));
    double* ss_out   = malloc((size_t)total_out * sizeof(double));
    double* cc_out   = malloc((size_t)total_out * sizeof(double));
    if (!iphi_out || !ss_out || !cc_out) {
        fprintf(stderr, "  malloc failed\n");
        free(iphi_out); free(ss_out); free(cc_out);
        return -99;
    }

    nn = 0;
    int ww = 0; /* write index */
    for (iv = 0; iv < 6; iv++) {
        for (it = 0; it <= MAXTIME; it++) {
            int cnt = c->limit[it][iv];
            for (n = 0; n < cnt; n++) {
                if (survives(c->ss[nn], c->cc[nn], it, threshold, max_tm)) {
                    memcpy(&iphi_out[ww * MAXARG], c->iphi[nn], MAXARG * sizeof(short));
                    ss_out[ww] = c->ss[nn];
                    cc_out[ww] = c->cc[nn];
                    ww++;
                }
                nn++;
            }
        }
    }

    /* Write .ictx file */
    FILE* fp = fopen(out_path, "wb");
    if (!fp) {
        fprintf(stderr, "  cannot open output: %s\n", out_path);
        free(iphi_out); free(ss_out); free(cc_out);
        return -1;
    }

    /* Header */
    uint32_t magic   = ICTX_MAGIC;
    uint32_t version = ICTX_VERSION;
    int32_t  body32  = (int32_t)ibody;
    int32_t  nterms  = (int32_t)total_out;
    double   reserved = 0.0;
    fwrite(&magic,     sizeof(uint32_t), 1, fp);
    fwrite(&version,   sizeof(uint32_t), 1, fp);
    fwrite(&body32,    sizeof(int32_t),  1, fp);
    fwrite(&nterms,    sizeof(int32_t),  1, fp);
    fwrite(&threshold, sizeof(double),   1, fp);
    fwrite(&reserved,  sizeof(double),   1, fp);

    /* Metadata */
    fwrite(c->receq,   sizeof(double), 9,      fp);
    fwrite(&c->rgm,    sizeof(double), 1,      fp);
    fwrite(c->ci0,     sizeof(double), MAXARG, fp);
    fwrite(c->ci1,     sizeof(double), MAXARG, fp);
    fwrite(c->freqpla, sizeof(double), 8,      fp);

    /* Limit table as int16 */
    for (it = 0; it <= MAXTIME; it++) {
        for (iv = 0; iv < 6; iv++) {
            int16_t lim = (int16_t)new_limit[it][iv];
            fwrite(&lim, sizeof(int16_t), 1, fp);
        }
    }

    /* Term records */
    for (ww = 0; ww < total_out; ww++) {
        int16_t iphi_buf[MAXARG];
        int j;
        for (j = 0; j < MAXARG; j++)
            iphi_buf[j] = (int16_t)iphi_out[ww * MAXARG + j];
        fwrite(iphi_buf,    sizeof(int16_t), MAXARG, fp);
        fwrite(&ss_out[ww], sizeof(double),  1,      fp);
        fwrite(&cc_out[ww], sizeof(double),  1,      fp);
    }

    long file_size = ftell(fp);
    fclose(fp);
    free(iphi_out); free(ss_out); free(cc_out);

    printf("  body=%d  terms: %d -> %d (%.1f%%)  file: %.2f MB\n",
           ibody, total_in, total_out,
           100.0 * total_out / (total_in ? total_in : 1),
           file_size / 1e6);

    return 0;
}

/* -----------------------------------------------------------------------
**  TOP2013 packer — produces .tictx files from .tctx binary contexts.
**  Magic: "TICT" (0x54494354), version 1.
**  Term record: int32 m, double c, double s.
** --------------------------------------------------------------------- */

#define TICTX_MAGIC   0x54494354u   /* "TICT" */
#define TICTX_VERSION 1u

static int pack_top_planet(int ibody, const char* tctx_dir, const char* out_dir,
                           double threshold, double max_tm, ephTOPctx* c)
{
    char tctx_path[512], out_path[512];
    snprintf(tctx_path, sizeof(tctx_path), "%s/", tctx_dir);
    snprintf(out_path, sizeof(out_path), "%s/TOP2013_%d.tictx", out_dir, ibody);

    int status = ephTopc(ibody, tctx_path, c);
    if (status != 0) {
        fprintf(stderr, "  ephTopc(%d) failed: %d\n", ibody, status);
        return status;
    }

    /* Count surviving terms, preserving the iv/it/n iteration order used
       by ephTopPlanet. */
    int it, iv, nn, n;
    int new_limit[MAXTIME_TOP+1][6];
    int total_in = 0, total_out = 0;

    for (iv = 0; iv < 6; iv++)
        for (it = 0; it <= MAXTIME_TOP; it++)
            new_limit[it][iv] = 0;

    nn = 0;
    for (iv = 0; iv < 6; iv++) {
        for (it = 0; it <= MAXTIME_TOP; it++) {
            int cnt = c->limit[it][iv];
            for (n = 0; n < cnt; n++) {
                if (survives(c->s[nn], c->c[nn], it, threshold, max_tm)) {
                    new_limit[it][iv]++;
                    total_out++;
                }
                total_in++;
                nn++;
            }
        }
    }

    FILE* fp = fopen(out_path, "wb");
    if (!fp) {
        fprintf(stderr, "  cannot open output: %s\n", out_path);
        return -1;
    }

    /* Header */
    uint32_t magic   = TICTX_MAGIC;
    uint32_t version = TICTX_VERSION;
    int32_t  body32  = (int32_t)ibody;
    int32_t  nterms  = (int32_t)total_out;
    double   reserved = 0.0;
    fwrite(&magic,     sizeof(uint32_t), 1, fp);
    fwrite(&version,   sizeof(uint32_t), 1, fp);
    fwrite(&body32,    sizeof(int32_t),  1, fp);
    fwrite(&nterms,    sizeof(int32_t),  1, fp);
    fwrite(&threshold, sizeof(double),   1, fp);
    fwrite(&reserved,  sizeof(double),   1, fp);

    /* Scalar metadata */
    fwrite(c->receq, sizeof(double), 9, fp);
    fwrite(&c->rgm,  sizeof(double), 1, fp);
    fwrite(&c->dmu,  sizeof(double), 1, fp);
    fwrite(&c->freq, sizeof(double), 1, fp);

    /* Limit table as int16 */
    for (it = 0; it <= MAXTIME_TOP; it++) {
        for (iv = 0; iv < 6; iv++) {
            int16_t lim = (int16_t)new_limit[it][iv];
            fwrite(&lim, sizeof(int16_t), 1, fp);
        }
    }

    /* Term records: keep only survivors, in the same iv/it/n order */
    nn = 0;
    for (iv = 0; iv < 6; iv++) {
        for (it = 0; it <= MAXTIME_TOP; it++) {
            int cnt = c->limit[it][iv];
            for (n = 0; n < cnt; n++) {
                if (survives(c->s[nn], c->c[nn], it, threshold, max_tm)) {
                    int32_t mk = (int32_t)c->m[nn];
                    fwrite(&mk,      sizeof(int32_t), 1, fp);
                    fwrite(&c->c[nn], sizeof(double), 1, fp);
                    fwrite(&c->s[nn], sizeof(double), 1, fp);
                }
                nn++;
            }
        }
    }

    long file_size = ftell(fp);
    fclose(fp);

    printf("  body=%d  terms: %d -> %d (%.1f%%)  file: %.2f MB\n",
           ibody, total_in, total_out,
           100.0 * total_out / (total_in ? total_in : 1),
           file_size / 1e6);

    return 0;
}

int main(int argc, char* argv[])
{
    int do_top = 0;
    int argoff = 1;

    /* Optional --top flag selects TOP2013 mode */
    if (argc >= 2 && strcmp(argv[1], "--top") == 0) {
        do_top = 1;
        argoff = 2;
    }

    if (argc < argoff + 2) {
        fprintf(stderr, "Usage: %s [--top] <ctx_dir> <output_dir> [threshold [max_tm]]\n", argv[0]);
        fprintf(stderr, "  --top      pack TOP2013 .tctx -> .tictx (default: VSOP2013 .ctx -> .ictx)\n");
        fprintf(stderr, "  threshold  amplitude filter in AU (default 1e-9)\n");
        fprintf(stderr, "  max_tm     epoch limit in Julian millennia from J2000 (default 0 = no time weighting)\n");
        fprintf(stderr, "             e.g. 0.2 covers +-200 yr from J2000\n");
        return 1;
    }

    const char* ctx_dir   = argv[argoff];
    const char* out_dir   = argv[argoff+1];
    double threshold = (argc >= argoff+3) ? atof(argv[argoff+2]) : 1e-9;
    double max_tm    = (argc >= argoff+4) ? atof(argv[argoff+3]) : 0.0;

    printf("indi_eph_packer%s: threshold=%.2e  max_tm=%.4g%s\n",
           do_top ? " --top" : "",
           threshold, max_tm, max_tm == 0.0 ? " (no time weighting)" : "");
    printf("  src: %s\n  dst: %s\n\n", ctx_dir, out_dir);

    long total_bytes = 0;
    int failed = 0;

    if (do_top) {
        ephTOPctx* c = malloc(sizeof(ephTOPctx));
        if (!c) { fprintf(stderr, "malloc failed\n"); return 1; }

        int ibody;
        for (ibody = 5; ibody <= 8; ibody++) {
            int st = pack_top_planet(ibody, ctx_dir, out_dir, threshold, max_tm, c);
            if (st != 0) { failed++; continue; }

            char path[512];
            snprintf(path, sizeof(path), "%s/TOP2013_%d.tictx", out_dir, ibody);
            FILE* f = fopen(path, "rb");
            if (f) { fseek(f, 0, SEEK_END); total_bytes += ftell(f); fclose(f); }
        }
        free(c);
    } else {
        ephPLANctx* c = malloc(sizeof(ephPLANctx));
        if (!c) { fprintf(stderr, "malloc failed\n"); return 1; }

        int ibody;
        for (ibody = 1; ibody <= 8; ibody++) {
            int st = pack_planet(ibody, ctx_dir, out_dir, threshold, max_tm, c);
            if (st != 0) { failed++; continue; }

            char path[512];
            snprintf(path, sizeof(path), "%s/VSOP2013_%d.ictx", out_dir, ibody);
            FILE* f = fopen(path, "rb");
            if (f) { fseek(f, 0, SEEK_END); total_bytes += ftell(f); fclose(f); }
        }
        free(c);
    }

    printf("\nTotal packed size: %.2f MB\n", total_bytes / 1e6);
    if (failed) printf("WARNING: %d bodies failed\n", failed);
    return failed ? 1 : 0;
}
