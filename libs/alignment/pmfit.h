#pragma once

typedef enum {
    PMFIT_NONE = 0,    /* sentinel / unset */
    PMFIT_IA   = 1,    /* index error, primary axis    (IH / IA) */
    PMFIT_IB   = 2,    /* index error, secondary axis  (ID / IE) */
    PMFIT_AN   = 3,    /* N-S polar/axis tilt          (ME / AN) */
    PMFIT_AW   = 4,    /* E-W polar/axis tilt          (MA / AW) */
    PMFIT_CA   = 5,    /* collimation error            (CH / CA) */
    PMFIT_TF   = 6,    /* tube flexure                 (TF / TF) */
} PmfitTerm;

/* Default full term list, sentinel-terminated. */
extern const PmfitTerm PMFIT_TERMS[7];

int Pmfit(double phi, char mount, int n, double *obs,
          const PmfitTerm *terms, int nt,
          double *pm, double *sigmas, double *skysig);
int Bfun(const PmfitTerm *terms, int nt, double phi, char mount,
         double rdem, double pdem, double *bf);
