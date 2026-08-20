#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
g++ -std=c++17 -Wall -o /tmp/flick-tests flick-tests.cpp -lm
/tmp/flick-tests
