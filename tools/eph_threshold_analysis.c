/*
**  e p h _ t h r e s h o l d _ a n a l y s i s
**
**  Count surviving VSOP2010 terms per planet at candidate amplitude
**  thresholds and estimate resulting .ictx file sizes.
**
**  Usage:
**     eph_threshold_analysis <ctx_dir>
**
**  Requires VSOP2010_#.ctx files (produced by plan_bin or distributed
**  with the EPH library). Prints a table of:
**    threshold | body | terms_in | terms_out | ratio | est_MB
**
**  Copyright (C) 2025 INDI Contributors.
**  Released under LGPL 2.1 or later.
*/

#include "../libs/indicore/eph/eph.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define N_THRESHOLDS 6
static const double THRESHOLDS[N_THRESHOLDS] = {
    1e-8, 1e-9, 1e-10, 1e-11, 1e-12, 0.0  /* 0.0 = keep all */
};

/* Bytes per term in .ictx: 17 int16 + 2 double = 34 + 16 = 50 */
#define BYTES_PER_TERM 50
/* Fixed overhead per file: header(48) + metadata(~512) + limit table(252) */
#define FIXED_OVERHEAD 812

static const char* body_names[] = {
    "", "Mercury", "Venus", "EMB", "Mars",
    "Jupiter", "Saturn", "Uranus", "Neptune"
};

int main(int argc, char* argv[])
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <ctx_dir>\n", argv[0]);
        return 1;
    }

    const char* ctx_dir = argv[1];

    ephPLANctx* c = malloc(sizeof(ephPLANctx));
    if (!c) { fprintf(stderr, "malloc failed\n"); return 1; }

    printf("%-12s  %-8s  %8s  %8s  %6s  %7s\n",
           "threshold", "body", "terms_in", "terms_out", "ratio%", "est_MB");
    printf("%-12s  %-8s  %8s  %8s  %6s  %7s\n",
           "------------", "--------", "--------", "--------", "------", "-------");

    int ti;
    for (ti = 0; ti < N_THRESHOLDS; ti++) {
        double threshold = THRESHOLDS[ti];
        long total_in = 0, total_out = 0, total_bytes = 0;
        int ibody;

        for (ibody = 1; ibody <= 8; ibody++) {
            char path[512];
            snprintf(path, sizeof(path), "%s/", ctx_dir);
            int st = ephPlanc(ibody, path, c);
            if (st != 0) {
                fprintf(stderr, "ephPlanc(%d) failed: %d\n", ibody, st);
                continue;
            }

            int it, iv, nn = 0, n;
            long in_body = 0, out_body = 0;

            for (iv = 0; iv < 6; iv++) {
                for (it = 0; it <= MAXTIME; it++) {
                    int cnt = c->limit[it][iv];
                    for (n = 0; n < cnt; n++) {
                        double amp = hypot(c->ss[nn], c->cc[nn]);
                        if (threshold == 0.0 || amp >= threshold)
                            out_body++;
                        in_body++;
                        nn++;
                    }
                }
            }

            long est_bytes = FIXED_OVERHEAD + out_body * BYTES_PER_TERM;
            total_in    += in_body;
            total_out   += out_body;
            total_bytes += est_bytes;

            printf("%-12.2e  %-8s  %8ld  %8ld  %5.1f%%  %7.2f\n",
                   threshold == 0.0 ? 1e-99 : threshold,
                   body_names[ibody],
                   in_body, out_body,
                   100.0 * out_body / (in_body ? in_body : 1),
                   est_bytes / 1e6);
        }

        printf("%-12.2e  %-8s  %8ld  %8ld  %5.1f%%  %7.2f  <-- total\n\n",
               threshold == 0.0 ? 1e-99 : threshold,
               "ALL",
               total_in, total_out,
               100.0 * total_out / (total_in ? total_in : 1),
               total_bytes / 1e6);
    }

    free(c);
    return 0;
}
