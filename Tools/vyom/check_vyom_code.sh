#!/usr/bin/env bash
# Static checks for Vyom-owned code only (src/modules/vyom_*, src/drivers/vyom_*, boards/vyom).
#  1. No dynamic allocation in flight-critical Vyom modules (CLAUDE.md rule 3). Allocation is
#     allowed only inside a function whose name ends in "init" / "Init" / constructor, and the
#     line must carry the marker "// vyom-alloc-ok: <reason>".
#  2. clang-tidy over Vyom C/C++ files, using the compile_commands.json of a build dir.
# Usage: Tools/vyom/check_vyom_code.sh [build_dir]   (default build/px4_sitl_default)
set -euo pipefail
cd "$(dirname "$0")/../.."
BUILD_DIR="${1:-build/px4_sitl_default}"

mapfile -t FILES < <(find src/modules -path '*/vyom_*' \( -name '*.c' -o -name '*.cpp' -o -name '*.h' -o -name '*.hpp' \) 2>/dev/null; \
                     find src/drivers -path '*/vyom_*' \( -name '*.c' -o -name '*.cpp' -o -name '*.h' -o -name '*.hpp' \) 2>/dev/null; \
                     find boards/vyom \( -name '*.c' -o -name '*.cpp' \) 2>/dev/null)
echo "vyom code files: ${#FILES[@]}"
if [ "${#FILES[@]}" -eq 0 ]; then
  echo "no Vyom code yet: nothing to check"; exit 0
fi

fail=0
# 1. dynamic allocation
if grep -nE '\b(malloc|calloc|realloc|free)\s*\(|\bnew\s+[A-Za-z_]|\bdelete\b|std::(vector|string|map|list|deque|unique_ptr|shared_ptr)\b' "${FILES[@]}" \
     | grep -v 'vyom-alloc-ok:'; then
  echo "ERROR: dynamic allocation in Vyom code (mark init-time allocations with '// vyom-alloc-ok: <reason>')"
  fail=1
fi
# 2. clang-tidy (only .c/.cpp with a compile command)
if command -v clang-tidy >/dev/null && [ -f "$BUILD_DIR/compile_commands.json" ]; then
  SRCS=(); for f in "${FILES[@]}"; do case "$f" in *.c|*.cpp) SRCS+=("$f");; esac; done
  if [ "${#SRCS[@]}" -gt 0 ]; then
    clang-tidy -p "$BUILD_DIR" --quiet --warnings-as-errors='*' "${SRCS[@]}" || fail=1
  fi
else
  echo "WARN: clang-tidy or $BUILD_DIR/compile_commands.json missing; skipped clang-tidy"
fi
exit $fail
