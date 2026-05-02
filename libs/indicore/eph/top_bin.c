#include "eph.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/*
**  - - - - - - - - -
**   T O P _ B I N
**  - - - - - - - - -
**
**  Read TOP2013.dat ASCII file and write binary .tctx context files
**  for bodies 5-8 (Jupiter through Neptune).
**
**  Usage:
**     eph_top_bin [path-to-TOP2013.dat-directory]
**
**  The .tctx files are written to the current directory.
**  If no path argument is given, TOP2013.dat is read from the current directory.
**
**  Copyright (C) 2025 INDI Contributors.
**  Permission to use, copy, modify, and/or distribute this software for
**  any purpose with or without fee is hereby granted.
*/

int main ( int argc, char* argv[] )
{
   const char* dat_path = (argc >= 2) ? argv[1] : "";

/* Outer-planet TOP2013 context (heap-allocated: ~2 MB per body) */
   ephTOPctx* c = malloc ( sizeof(ephTOPctx) );
   if ( !c ) {
      fprintf ( stderr, "malloc failed\n" );
      return 1;
   }

/* Name of binary output file */
#define LF_TOP (17)
   char namef[LF_TOP] = "TOP2013_#.tctx";
   char fname[LF_TOP];

   FILE* fp;
   int ibody, j;

/* Body by body: Jupiter=5, Saturn=6, Uranus=7, Neptune=8. */
   for ( ibody = 5; ibody <= 8; ibody++ ) {

   /* Parse the ASCII TOP2013.dat for this body. */
      j = ephTopi ( (char*) dat_path, ibody, c );
      if ( j ) {
         fprintf ( stderr, "ephTopi(%d) failed: %d\n", ibody, j );
         free ( c );
         return 1;
      }

   /* Generate output filename. */
      strncpy ( fname, namef, LF_TOP );
      *strchr ( fname, '#' ) = (char) ( '0' + ibody );

   /* Open and write. */
      fp = fopen ( fname, "wb" );
      if ( !fp ) {
         fprintf ( stderr, "Failed to open %s for writing\n", fname );
         free ( c );
         return 1;
      }

      j = fwrite ( c, sizeof(ephTOPctx), 1, fp );
      if ( j != 1 ) {
         fprintf ( stderr, "Error writing %s\n", fname );
         fclose ( fp );
         free ( c );
         return 1;
      }

      fclose ( fp );
      printf ( "Wrote %s (%.2f MB)\n", fname, (double) sizeof(ephTOPctx) / 1e6 );
   }

   free ( c );
   return 0;
}
