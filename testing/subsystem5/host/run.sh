#!/bin/sh
# Build and run the host-side tests for the obstacle maths and the Safety Monitor logic.
# Run from the repo root.
set -e
cd "$(dirname "$0")"
gcc -std=c99 -Wall -Wextra -Wpedantic -O1 -fsanitize=address,undefined \
    test_profile.c ../../../app_program/subsystem5/obstacle_profile.c -lm -o /tmp/s5_host_test
/tmp/s5_host_test
gcc -std=c99 -Wall -Wextra -Wpedantic -O1 -fsanitize=address,undefined \
    test_safety.c ../../../app_program/subsystem5/safety_logic.c -o /tmp/s5_host_safety
/tmp/s5_host_safety
gcc -std=c99 -Wall -Wextra -Wpedantic -O1 -fsanitize=address,undefined \
    test_vc.c ../../../app_program/subsystem5/vc_logic.c -lm -o /tmp/s5_host_vc
/tmp/s5_host_vc
