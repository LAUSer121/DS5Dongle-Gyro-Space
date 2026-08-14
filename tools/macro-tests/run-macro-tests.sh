#!/usr/bin/env bash
# Host-side regression tests for src/macro.cpp. Run after ANY macro engine edit.
# macro.cpp is copied into stubs/ unmodified so its quoted includes resolve to
# the fakes; the copy is verified identical before compiling.
set -e
cd "$(dirname "$0")"
SRC=../../src
cp "$SRC/macro.cpp" stubs/macro.cpp
cmp -s "$SRC/macro.cpp" stubs/macro.cpp || { echo "copy differs from source"; exit 1; }
g++ -std=c++17 -Wall -DENABLE_WAKE_HID -I"$SRC" -Istubs -o /tmp/macro-tests macro-tests.cpp stubs/macro.cpp
/tmp/macro-tests
