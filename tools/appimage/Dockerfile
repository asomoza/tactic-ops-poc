# Ubuntu 24.04 build environment for the AppImage: glibc 2.39, the floor set by Wicked's prebuilt libdxcompiler.so.
FROM ubuntu:24.04

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        build-essential cmake ninja-build pkg-config libsdl2-dev zlib1g-dev \
        patchelf file curl ca-certificates squashfs-tools \
    && rm -rf /var/lib/apt/lists/*

ARG APPIMAGETOOL_VERSION=1.9.1
RUN curl -fsSL -o /usr/local/bin/appimagetool \
        "https://github.com/AppImage/appimagetool/releases/download/${APPIMAGETOOL_VERSION}/appimagetool-x86_64.AppImage" \
    && chmod +x /usr/local/bin/appimagetool

ENV APPIMAGE_EXTRACT_AND_RUN=1
