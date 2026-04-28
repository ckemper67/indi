#include "eph.h"
#include <string.h>
#include <stdio.h>

int ephPlanc ( int ibody, char* path, ephPLANctx* c )
/*
**  - - - - - - - - -
**   e p h P l a n c
**  - - - - - - - - -
**
**  Read from a binary file the context needed for computation of a
**  planet's ephemerides.
**
**  Given:
**     ibody   int          body index:
**                            1: Mercury
**                            2: Venus
**                            3: EMB
**                            4: Mars
**                            5: Jupiter
**                            6: Saturn
**                            7: Uranus
**                            8: Neptune
**     path    char*        path for filenames
**
**  Returned:
**     c       ephPLANctx*  context structure
**
**  Returned (function value):
**             int         status:
**                             0  no error
**                            -1  illegal ibody
**                            -2  file path+name too big
**                          else  file-related errors
**
**  Defined in eph.h:
**     ephPLANctx    planetary ephemeris context
**
**  Notes:
**
**  1  Generating planetary ephemerides involves first reading files
**     (one per body) to obtain arrays of coefficients etc., followed by
**     computation of the coordinates for a nominated time.  The present
**     function performs the first phase, initializing a context data
**     structure for the chosen body by reading it directly from a
**     binary file.  Once this context is available a different
**     function, ephPlanpv, can be called to compute the ephemeris data.
**
**  2  The context data structure, of type ephPLANctx, contains:
**
**       . initialization status (0 = not yet initialized)
**       . VSOP2010 constants
**       . VSOP2010 series
**
**     In an application, ephemerides for successive bodies can be
**     computed using a single context structure that is reinitialized
**     repeatedly.  Should the application need to compute ephemerides
**     for more than one body at once, multiple context structures must
**     be declared.
**
**  3  The present code explicitly declares the names of the nine files
**     that contain the contexts for each body, and these can be edited
**     if necessary, for example to include the directory name.  The
**     latter can in any case be supplied through the argument path,
**     which can be set to an empty string if the code has been edited
**     to include the path or because the files are in the current
**     directory.
**
**  Reference:
**
**     J.-L. Simon, G. Francou, A. Fienga & H. Manche, "New analytical
**     planetary theories VSOP2013 and TOP2013", Astronomy &
**     Astrophysics 557, A49 (2013).
**
**  Called:  ephBopr
**
**  Last revision:   2021 October 9
**
**  Author P.T.Wallace - see license notice at end.
*/
{
/* Name of binary file (# = body number) */
#define LF (15)
   char namef[LF] = "VSOP2010_#.ctx";
   char fname[LF];

   FILE* fp;
   int j;


/* Validate ibody. */
   if ( ibody < 1 || ibody > 8 ) return -1;

/* Generate name of binary file. */
   strncpy ( fname, namef, LF );
   *strchr ( fname, '#' ) = (char) ( '0' + ibody );

/* Open the binary file. */
   fp = ephBopr ( path, fname, &j );
   if ( j ) {
      printf ( "Failed to open file %s, error %d\n", fname, j );
      return j-1;
   }

/* Read the context. */
   j = fread ( c, sizeof(ephPLANctx), 1, fp );
   if ( j != 1 ) {
      printf ( "Error reading file\n" );
      return -4;
   }

/* Close the file. */
   j = fclose ( fp );
   if ( j  ) {
      printf ( "Error closing file %s\n", fname );
      return -5;
   }

/* Success. */
   return 0;
}

/*----------------------------------------------------------------------
**
**  Copyright (C) 2021 by P.T.Wallace
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
