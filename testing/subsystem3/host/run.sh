#!/bin/sh
# PC tests for subsystem 3's wide-bar barcode decoder (S3-H01..H05).
# Run from the repo root in WSL:  sh testing/subsystem3/host/run.sh
# C39_DEBUG=1 also prints every decode attempt.
set -e
gcc -std=c99 -O1 -Wall -Wextra -g -fsanitize=address,undefined \
    testing/subsystem3/host/test_code39_wide.c app_program/subsystem3/code39_wide.c -lm \
    -o /tmp/s3_host_code39_wide
/tmp/s3_host_code39_wide
