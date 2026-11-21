#!/usr/bin/env bash
# Build and run Vyom unit tests on the host with sanitizers, without PX4's full test build.
# CI also runs them through `make tests` (real googletest). Usage: Tools/vyom/run_host_tests.sh
set -euo pipefail
cd "$(dirname "$0")/../.."
OUT=build/vyom_host_tests
mkdir -p "$OUT"
FLAGS=(-std=c++17 -Wall -Wextra -Werror -Wshadow -Wconversion -g -O1 -fsanitize=address,undefined -fno-sanitize-recover=all
       -I Tools/vyom/mini_gtest)
fail=0
build_run() {  # name, sources...
  local name=$1; shift
  echo "== $name"
  g++ "${FLAGS[@]}" "$@" -o "$OUT/$name" && "$OUT/$name" || fail=1
}
build_run SocLinkCoreTest -I src/drivers/vyom_soc_link src/drivers/vyom_soc_link/SocLinkCore.cpp src/drivers/vyom_soc_link/SocLinkCoreTest.cpp
build_run WdtPolicyTest -I src/drivers/vyom_wdt src/drivers/vyom_wdt/WdtPolicyTest.cpp
build_run GnssGuardTest -I src/modules/vyom_gnss_guard src/modules/vyom_gnss_guard/GnssGuard.cpp src/modules/vyom_gnss_guard/GnssGuardTest.cpp
exit $fail
