#!/usr/bin/env bash
# Builds dist/TacticsPoC-x86_64.AppImage in an Ubuntu 24.04 container, so it runs on any x86_64 Linux with glibc 2.38+.
# Usage: tools/build_appimage.sh [model_dir]   (default: ~/Documents/models/motionbricks_soma_diffusers)
# The sources are laid out as for a local build: motion/ and gta2_clone/third_party/WickedEngine/ next to tactics_poc/.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
parent="$(dirname "$root")"
models="$(realpath "${1:-$HOME/Documents/models/motionbricks_soma_diffusers}")"
wicked="$parent/gta2_clone/third_party/WickedEngine"

for dir in "$parent/motion" "$wicked" "$models"; do
    [[ -d "$dir" ]] || { echo "Missing $dir" >&2; exit 1; }
done

docker build -t tactics-poc-appimage "$root/tools/appimage"
docker run --rm --user "$(id -u):$(id -g)" \
    -v "$root:/src/tactics_poc" \
    -v "$parent/motion:/src/motion:ro" \
    -v "$wicked:/src/gta2_clone/third_party/WickedEngine:ro" \
    -v "$models:/models:ro" \
    tactics-poc-appimage bash /src/tactics_poc/tools/appimage/build_in_container.sh
