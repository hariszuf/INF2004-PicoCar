#!/bin/sh
# Host test for the barcode39 draft. Needs only gcc.
set -e
cd "$(dirname "$0")"
gcc -std=c11 -Wall -Wextra -Werror -O1 -o /tmp/test_barcode39 barcode39.c test_barcode39.c
/tmp/test_barcode39
