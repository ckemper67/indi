#include "eph.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

FILE* ephBopr ( char* path, char* file, int* jstat )
/*
**  - - - - - - - -
**   e p h B o p r
**  - - - - - - - -
**
**  Open a binary ephemeris file for reading.
**
**  Given:
**     path   const char*    path for filenames
**     file   const char*    filename of the binary ephemeris file
**
**  Returned:
**     jstat   int*          status:
**                              0  no error
**                             -1  path + filename too long
**                             -2  file not found
**
**  Returned (function value):
**             FILE*         file pointer for opened file
**
**  Notes:
**
**  1  Generating planetary ephemerides involves first reading files to
**     populate arrays of coefficients etc. that can later be used for
**     computation of the coordinates for a nominated time.  The present
**     function assists in this first phase, by opening one of the
**     ephemeris files (in its binary form).
**
**  2  The given path, which typically comes from the command line, may
**     or may not be empty.  If it is null, but the named file turns out
**     not to be in the current directory, a second attempt is made
**     using a standard path appropriate for the given platform.
**
**  3  The path argument is used exactly as given, with no handling of
**     leading or trailing spaces, and must contain any required
**     directory separators such as slash or backslash.
**
**  4  If path is the null pointer the behavior is as for an empty
**     string.
**
**  Last revision:   2021 October 9
**
**  Author P.T.Wallace - see license notice at end.
*/
{

/* Scratch string */
#define LSTR 200
   char str[LSTR+1];

   FILE* fp;
   int lfile, more, lpath;


/* Handle case of path pointer null. */
   if ( path == NULL ) path = "";

/* Length of  filename. */
   lfile = strlen ( file );

/* Make up to two attempts to open the file. */
   do {

   /* Assume this is the final attempt. */
      more = 0;

   /* Length of path. */
      lpath = strlen ( path );

   /* Abort if excessive path+filename length. */
      if ( lpath+lfile >= LSTR ) {
         *jstat = -1;
         return NULL;
      }

   /* Assemble the path + filename. */
      strcpy ( str, path );
      strcat ( str+lpath, file );

   /* Attempt to open the file. */
      if ( ( fp = fopen ( str, "rb" ) ) ) {

      /* Success. */
         *jstat = 0;
         return fp;

      } else {

      /* Fail:  if no path was supplied, try the default. */
         if ( ! lpath ) {

#ifdef _WIN32
            path = getenv("HOMEPATH");
            strcat ( path, "\\" );
#else
            path = getenv("HOME");
            strcat ( path, "/" );
#endif
            more = 1;
         }
      }
   } while ( more );

/* Failure. */
   *jstat = -2;
   return NULL;
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
