#!/bin/sh
# Build and run the host-side tests for the obstacle maths. Run from the repo root.
set -e
cd "$(dirname "$0")"
gcc -std=c99 -Wall -Wextra -Wpedantic -O1 -fsanitize=address,undefined \
    test_profile.c ../../../app_program/subsystem5/obstacle_profile.c -o /tmp/s5_host_test
/tmp/s5_host_test
