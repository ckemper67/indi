#include "eph.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/*
**  - - - - - - - - -
**   M O O N _ B I N
**  - - - - - - - - -
**
**  Read ELP/MPP02 ASCII files and write binary versions.
**
**  Latest revision:   October 9
**
**  Author P.T.Wallace - see license notice at end.
*/

int main ( )
{
/* Lunar ephemeris context */
   ephMOONctx* c = malloc(sizeof(ephMOONctx));

/* Name of binary file */
#define LFN (18)
   char namef[LFN] = "ELP_MPP02_###.ctx";
   char fname[LFN];

   FILE* fp;
   int icor, j;


/* Do both LLR and DE405 variants. */
   for ( icor = 1; icor <= 2; icor++ ) {

   /* Generate name of binary file. */
      strncpy ( fname, namef, LFN );
      strncpy ( strstr ( fname, "###" ), icor == 1 ? "LLR" : "JPL", 3 );

   /* Open the binary file. */
      fp = fopen ( fname, "wb" );
      if ( ! fp ) {
         printf ( "Failed to open file %s\n", fname );
         return 1;
      }

   /* Initialize the context. */
      j = ephMooni ( "", icor, c );
      if ( j ) {
         printf ( "ephMooni error %d\n", j );
         return 1;
      }

   /* Write the binary file. */
      j = fwrite ( c, sizeof(ephMOONctx), 1, fp );
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

/* Next variant. */
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
