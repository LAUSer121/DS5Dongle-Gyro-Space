#!/usr/bin/env bash
# Convenience build script for the Waveshare RP2350B-Plus-W target.
#
# NOTE: this script assumes you ALREADY have a correctly pinned SDK checkout and
# a toolchain. If you don't, the one-command builders in tools/ are far easier -
# they fetch the SDK and TinyUSB at the right versions for you:
#
#     Windows:  powershell -ExecutionPolicy Bypass -File tools\build-windows.ps1 -Variant waveshare
#     macOS:    ./tools/build-macos.sh --waveshare
#
# Requires:
#   - ARM toolchain (arm-none-eabi-gcc)
#   - Ninja
#   - A pico-sdk checkout pinned to 2.2.0 with TinyUSB checked out to 0.20.0.
#     If you don't already have one, see the README for setup. By default this
#     script uses PICO_SDK_PATH from the environment.
#
# Usage:
#   ./boards/build_waveshare_rp2350b_plus_w.sh [Release|Debug]   # default: Release

set -euo pipefail

BUILD_TYPE="${1:-Release}"
BUILD_DIR="build/waveshare"
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

if [[ -z "${PICO_SDK_PATH:-}" ]]; then
    echo "PICO_SDK_PATH is not set. Set it to a pico-sdk checkout pinned to 2.2.0+TinyUSB 0.20.0." >&2
    exit 1
fi

# Verify the SDK VERSION, not just that the variable is set. This target compiles
# happily against SDK 2.1.1 - the version the main README's setup recipe produces
# for the Pico 2 W build - but this board's RM2 wireless needs 2.2.0, so a 2.1.1
# checkout yields a binary that builds cleanly and then misbehaves on hardware.
# Failing loudly here beats shipping someone a silently wrong build.
SDK_DESC="$(git -C "${PICO_SDK_PATH}" describe --tags 2>/dev/null || echo '')"
case "${SDK_DESC}" in
    2.2.*|2.[3-9]*|[3-9].*) : ;;
    '')
        echo "WARNING: cannot determine the Pico SDK version at ${PICO_SDK_PATH}." >&2
        echo "         This board needs SDK 2.2.0 or newer; 2.1.1 compiles but does not work correctly." >&2
        ;;
    *)
        echo "ERROR: Pico SDK at ${PICO_SDK_PATH} is ${SDK_DESC}, but this board needs 2.2.0 or newer." >&2
        echo "       SDK 2.1.1 (used for the Pico 2 W build) COMPILES for this board but the" >&2
        echo "       resulting firmware does not work correctly - the RM2 wireless needs 2.2.0." >&2
        echo "" >&2
        echo "       Easiest fix - use a one-command builder, which fetches the right SDK:" >&2
        echo "         Windows: powershell -ExecutionPolicy Bypass -File tools/build-windows.ps1 -Variant waveshare" >&2
        echo "         macOS:   ./tools/build-macos.sh --waveshare" >&2
        exit 1
        ;;
esac

cd "${PROJECT_ROOT}"
cmake -S . -B "${BUILD_DIR}" -G Ninja \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DWAVESHARE_RP2350B_PLUS_W_BUILD=ON \
    -DPICO_SDK_PATH="${PICO_SDK_PATH}"
cmake --build "${BUILD_DIR}" --target ds5-bridge

echo
echo "Build complete. UF2 at:"
echo "  ${PROJECT_ROOT}/${BUILD_DIR}/ds5-bridge.uf2"
