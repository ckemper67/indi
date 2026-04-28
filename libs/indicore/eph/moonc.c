#include "eph.h"
#include <string.h>
#include <stdio.h>

int ephMoonc ( char* path, int icor, ephMOONctx* c )
/*
**   - - - - - - - - -
**    e p h M o o n c
**   - - - - - - - - -
**
**  Read from a binary file the context needed for computation of Moon
**  ephemerides.
**
**  Given:
**     path    char*       path for filenames
**     icor    int         choice of corrections to the constants:
**                           icor = 1 : LLR
**                           icor = 2 : DE405
**
**  Returned:
**     c       ephMOONctx* context structure
**
**  Returned (function value):
**             int         status:
**                             0  OK
**                            -1  illegal icor
**                            -2  file path+name too big
**                          else  file related errors
**
**  Defined in eph.h:
**     ephMOONctx    lunar ephemeris context
**
**  Notes:
**
**  1  Generating lunar ephemerides involves first reading a file to
**     obtain arrays of coefficients etc., followed by computation of
**     the coordinates for a nominated time.  The present function
**     performs the first phase, initializing the context data
**     structure by reading it directly from a binary file.  Once this
**     context is available a different function, ephMoon, can be called
**     to compute the ephemeris data.
**
**  2  The context data structure, of type ephMOONctx, contains:
**
**       . initialization status:
**           0 : not yet initialized
**           1 : icor value (1 for LLR or 2 for DE405)
**       . ELP/MPP02 constants
**       . ELP/MPP02 series
**
**     Because the caller supplies the context structure, it is possible
**     to have more than one in operation at once, though this will not
**     as a rule be useful.  The principal benefit of separating the
**     context from the logic is to make reentrancy possible.
**
**  3  The present code explicitly declares the names of the files that
**     contain the ELP/MPP02 contexts (for the LLR and DE405 options
**     respectively), and these can be edited if necessary, for example
**     to include the directory name.  The latter is in any case
**     supplied through the argument path, which can be the null string
**     if the code has been edited to include the path or the files are
**     in the current directory.
**
**  4  The theoretical values of some constants have to be corrected.
**     There are two sets of corrections, and the choice is indicated by
**     the argument icor:
**
**       icor=1, the constants are fitted to LLR observations provided
**               from 1970 to 2001; it is the default value;
**
**       icor=2, the constants are fitted to the DE405 ephemeris over the
**               interval 1950-2060; the lunar angles w1, w2, w3 receive
**               also additive corrections to the secular coefficients.
**
**     Binary files for these two options exist and the appropriate one
**     read when the present function is called.
**
**  Reference:
**
**     LUNAR SOLUTION ELP version ELP/MPP02 - Jean CHAPRONT and
**     Gerard FRANCOU Observatoire de Paris - SYRTE department -
**     UMR 8630/CNRS - October 2002
**
**  Called:  ephBopr
**
**  Latest revision:   2022 July 25
**
**  Copyright P.T.Wallace.  All rights reserved.
*/
{
/* Name of binary file (### = either LLR or JPL) */
#define LF (18)
   char namef[LF] = "ELP_MPP02_###.ctx";
   char fname[LF];

   FILE* fp;
   int j;


/* Validate icor. */
   if ( icor != 1 && icor != 2 ) return -1;

/* Generate name of binary file. */
   strncpy ( fname, namef, LF );
   strncpy ( strstr ( fname, "###" ), icor == 1 ? "LLR" : "JPL", 3 );

/* Open the binary file. */
   fp = ephBopr ( path, fname, &j );
   if ( ! fp ) {
      printf ( "Failed to open file %s, error %d\n", fname, j );
      return j-1;
   }

/* Read the context. */
   j = fread ( c, sizeof(ephMOONctx), 1, fp );
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
**  Copyright (C) 2022 by P.T.Wallace
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
