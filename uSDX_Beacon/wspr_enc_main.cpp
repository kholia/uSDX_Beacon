/*
   This program tests a prototype of the WSPR encoder "wspr_enc" on PC. Check
   wspr_enc.h and wspr_enc.c for more details.

   The WSPR coding simulation tool "WSPRcode" is required to verify the symbols
   produced by the encoder. It can be built from the WSJT-X source:
        https://sourceforge.net/p/wsjt/wsjtx/ci/master/tree/lib/wsprcode/

   References:
   K1JT: WSPR 2.0 User’s Guide
        https://www.physics.princeton.edu/pulsar/K1JT/WSPR_2.0_User.pdf

   Author: BD1ES

   21 FEB 2020:
       Released to the Gist:
           https://gist.github.com/bd1es/a782e2529b8289288fadd35e407f6440
   31 MAR 2020:
       Removed parity table generation function.
   01 APR 2020:
       Added code auto-comparation using "WSPRcode" built from the source.
       Manual check is no longer available.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "wspr_enc.h"

static void str_to_upper(char *dist, const char *str)
{
  while (*str) *dist++ = toupper(*str++);
  *dist = 0;
}

// Modified "main"
int wspr_encode(char *call, char *grid, char *dBm, uint8_t *symbols)
{
  // static const char *PROG_NAME = "wspr_enc_test";
  char call_u[11], grid_u[7];

  // Call
  if ((strlen(call) < 2) || (strlen(call) > 10)) {
    printf("The length of callsign %s is invalid.\n", call);
    return -1;
  }
  str_to_upper(call_u, call);
  // for Swaziland 3DA0YZ, 3D0YZ will be used for proper decoding
  if (strncmp(call_u, "3DA0", 4) == 0) {
    size_t i;
    for (i = 3; i < strlen(call); i++) {
      call[i - 1] = call[i];
    }
    call[i - 1] = 0;
  }
  str_to_upper(call_u, call);

  // Grid
  if ((strlen(grid) != 4) && (strlen(grid) != 6)) {
    printf("The length of grid locator %s is invalid.\n", grid);
    return -1;
  }
  str_to_upper(grid_u, grid);

  printf("WSPR Message: %s %s %s\n", call, grid, dBm);

  // invoke encoder function to generate symbols
  // uint8_t msgtype = wspr_enc(call, grid, dBm, symbols);
  wspr_enc(call, grid, dBm, symbols);

  /* printf("Channel symbols: (message type %d)", msgtype);
    int i;
    for (i = 0; i < 162; i++) {
      if (i%30 == 0) {
          printf("\n      %d", symbols[i]);
      } else {
          printf(" %d", symbols[i]);
      }
    }
    printf("\n"); */

  return 0;
}
