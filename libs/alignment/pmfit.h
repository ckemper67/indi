#pragma once

int Pmfit(double phi, char mount, int n, double *obs, int nt,
          double *pm, double *sigmas, double *skysig);
int Bfun(int nt, double phi, char mount, double rdem, double pdem, double *bf);
