#!/bin/sh
# Host test for the two-stage trigger output rewrite.
# Slices the REAL apply_trigger_output() out of src/main.cpp and the REAL
# Config_body out of src/config.h, so the maths under test is the maths that
# ships. A copy of either would pass forever while the firmware drifted.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
SRC="$HERE/../../src"
python3 - "$SRC" "$HERE" <<'PY'
import sys
src, here = sys.argv[1], sys.argv[2]
s = open(f"{src}/main.cpp").read()
i = s.index('// Stage-2 output button -> report bit.')
j = s.index('void __not_in_flash_func(interrupt_loop)')
open(f"{here}/t2.inc", "w").write(s[i:j])
c = open(f"{src}/config.h").read()
a = c.index('struct __attribute__((packed)) Config_body')
b = c.index('void config_default();')
open(f"{here}/cfg.inc", "w").write(c[a:b])
PY
g++ -O1 -I"$HERE" -o "$HERE/t2-tests" "$HERE/t2-tests.cpp"
"$HERE/t2-tests"
