#!/bin/sh
# Host tests for the barcode decoder and scanner. Needs only gcc.
# Run from app_program/buddy3:  sh test/host/run_barcode_tests.sh
set -e
cd "$(dirname "$0")/../.."
gcc -std=c11 -Wall -Wextra -Werror -O1 -I. -o /tmp/barcode39_host_test \
    barcode39.c test/host/test_barcode39.c
gcc -std=c11 -Wall -Wextra -Werror -O1 -I. -o /tmp/barcode_scan_host_test \
    barcode39.c barcode_scan.c test/host/test_barcode_scan.c
/tmp/barcode39_host_test
/tmp/barcode_scan_host_test
