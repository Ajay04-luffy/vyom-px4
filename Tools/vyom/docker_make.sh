#!/usr/bin/env bash
# Build PX4 targets inside the same px4-dev image CI uses (has the arm-none-eabi toolchain).
# Usage: Tools/vyom/docker_make.sh vyom_core-v1_default [more make args]
set -euo pipefail
cd "$(dirname "$0")/../.."
IMAGE="${PX4_DEV_IMAGE:-px4io/px4-dev:v1.16.0-rc1-258-g0369abd556}"
EXTRA=()
# Pass a corporate/proxy CA through if one is configured (e.g. GIT_SSL_CAINFO=/path/ca.crt).
if [ -n "${GIT_SSL_CAINFO:-}" ]; then
  EXTRA+=(-v "$GIT_SSL_CAINFO:/ca.crt:ro" -e GIT_SSL_CAINFO=/ca.crt -e SSL_CERT_FILE=/ca.crt)
fi
exec docker run --rm --network host -e HTTPS_PROXY -e HTTP_PROXY -e NO_PROXY "${EXTRA[@]}" \
  -v "$PWD:/src" -w /src "$IMAGE" bash -c 'git config --global --add safe.directory "*"; make "$@"' make "$@"
