#!/usr/bin/env bash
set -euo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
image=seajackal/hydrolib:latest

docker pull "$image"
docker run --rm \
    --user "$(id -u):$(id -g)" \
    --mount "type=bind,source=$repo_dir,target=/workspace" \
    --workdir /workspace \
    "$image" bash -ec '
        cmake -S . -B build/docker -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
        cmake --build build/docker --target pixhawk_simulator
    '
