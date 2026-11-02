#!/usr/bin/env bash
# Builds and runs the DrillCore unit tests with the host compiler (no PlatformIO
# needed). Usage: scripts/run_host_tests.sh [path-to-unity-src]
# The PlatformIO equivalent is:  pio test -e native
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
UNITY="${1:-${UNITY_SRC:-}}"
BUILD="$ROOT/.pio/hosttests"
mkdir -p "$BUILD"
if [ -z "$UNITY" ]; then
  UNITY="$BUILD/unity"
  [ -d "$UNITY" ] || git clone -q --depth 1 -b v2.6.0 https://github.com/ThrowTheSwitch/Unity.git "$UNITY"
  UNITY="$UNITY/src"
fi
CORE=("$ROOT"/lib/DrillCore/src/*.cpp)
fail=0
for t in "$ROOT"/test/test_*/; do
  name=$(basename "$t")
  g++ -std=c++17 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined \
      -I"$ROOT/lib/DrillCore/src" -I"$UNITY" "$t"/*.cpp "${CORE[@]}" "$UNITY/unity.c" \
      -o "$BUILD/$name" -lm
  echo "=== $name"
  if ! "$BUILD/$name"; then fail=1; fi
done
exit $fail
