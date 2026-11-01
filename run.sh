#!/usr/bin/env bash
# Build (if needed) and start the MotionBricks tactics proof of concept.
# Usage: ./run.sh [model_dir]   (default: ~/Documents/models/motionbricks_soma_diffusers)
set -euo pipefail
cd "$(dirname "$0")"
if [ ! -f build/build.ninja ]; then
    cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
fi
cmake --build build --target tactics_poc
exec ./build/tactics_poc "$@"
