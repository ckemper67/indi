#include "eph.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/*
**  - - - - - - - - -
**   P L A N _ B I N
**  - - - - - - - - -
**
**  Read VSOP2010 ASCII files and write binary versions.
**
**  Latest revision:   2021 October 9
**
**  Author P.T.Wallace - see license notice at end.
*/

int main ( )
{
/* Planetary ephemeris context */
   ephPLANctx* c = malloc(sizeof(ephPLANctx));

/* Name of binary file */
#define LF (15)
   char namef[LF] = "VSOP2010_#.ctx";
   char fname[LF];

   FILE* fp;
   int ibody, j;


/* Body by body. */
   for ( ibody = 1; ibody <= 8; ibody++ ) {

   /* Generate name of binary file. */
      strncpy ( fname, namef, LF );
      *strchr ( fname, '#' ) = (char) ( '0' + ibody );

   /* Open the binary file. */
      fp = fopen ( fname, "wb" );
      if ( ! fp ) {
         printf ( "Failed to open file %s\n", fname );
         return 1;
      }

   /* Initialize the context. */
      j = ephPlani ( "", ibody, c );
      if ( j ) {
         printf ( "ephPlani error %d\n", j );
         return 1;
      }

   /* Write the binary file. */
      j = fwrite ( c, sizeof(ephPLANctx), 1, fp );
      if ( j != 1 ) {
         printf ( "Error writing to file %s\n", fname );
         return 1;
      }

   /* Close it. */
      j = fclose ( fp );
      if ( j  ) {
         printf ( "Error closing file %s\n", fname );
         return 1;
      }

   /* Next body. */
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
