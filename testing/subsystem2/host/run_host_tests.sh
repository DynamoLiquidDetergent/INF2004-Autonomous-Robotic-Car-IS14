#!/bin/sh
# Build and run subsystem 2's PC unit tests (no Pico needed).
# Usage, from the repository root:  sh testing/subsystem2/host/run_host_tests.sh
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
OUT="${TMPDIR:-/tmp}/s2_host_tests"
cc -std=c99 -Wall -Wextra -DS2_HOST_TEST -I"$HERE" \
   "$HERE/test_motion_math.c" "$HERE/../../../app_program/subsystem2/motion_math.c" -o "$OUT"
"$OUT"
