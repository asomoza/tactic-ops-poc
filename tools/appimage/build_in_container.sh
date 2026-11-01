#!/usr/bin/env bash
# Runs inside the tools/appimage container: builds the game, lays out the AppDir from `cmake --install` and packs it.
# Expects /src/tactics_poc, /src/motion, /src/gta2_clone/third_party/WickedEngine and the weights at /models.
set -euo pipefail

src=/src/tactics_poc
build="$src/build-appimage"
appdir="$build/AppDir"
game="$appdir/usr/lib/tactics_poc"
out="$src/dist/TacticsPoC-x86_64.AppImage"

cmake -S "$src" -B "$build" -G Ninja -DCMAKE_BUILD_TYPE=Release -DTACTICS_MODEL_DIR=/models
cmake --build "$build" --target tactics_poc

rm -rf "$appdir"
cmake --install "$build" --prefix "$game"

# Bundle the shared libraries a desktop may lack. The host's own are used for glibc, the C++ runtime, and everything
# that talks to the GPU driver, display server or session (X11, xcb, Wayland, drm, gbm, ALSA, dbus, systemd), which
# must match the host's drivers; the Vulkan loader and drivers are loaded at runtime and never bundled.
mkdir -p "$game/lib"
host='^(linux-vdso|ld-linux|libc|libm|libdl|librt|libpthread|libstdc\+\+|libgcc_s|libz|libdrm|libgbm|libX11|libX11-xcb|libxcb|libXau|libXdmcp|libXext|libXrender|libwayland-[a-z]+|libasound|libexpat|libffi|libdbus-1|libsystemd|libcap|libgcrypt|libgpg-error|liblzma|liblz4|libzstd|libbsd|libmd)\.so'
ldd "$game/tactics_poc" | awk '$3 ~ /^\// {print $1, $3}' | while read -r name path; do
    if ! [[ "$name" =~ $host ]]; then
        echo "bundling $name"
        cp -L "$path" "$game/lib/$name"
        patchelf --set-rpath '$ORIGIN' "$game/lib/$name"
    fi
done
patchelf --set-rpath '$ORIGIN/lib' "$game/tactics_poc"

install -m 755 "$src/tools/appimage/AppRun" "$appdir/AppRun"
install -m 644 "$src/tools/appimage/tactics_poc.desktop" "$appdir/tactics_poc.desktop"
install -m 644 "$src/tools/appimage/tactics_poc.svg" "$appdir/tactics_poc.svg"
ln -sf tactics_poc.svg "$appdir/.DirIcon"

mkdir -p "$src/dist"
ARCH=x86_64 appimagetool --no-appstream "$appdir" "$out"
echo "Built $out"
