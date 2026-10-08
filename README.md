# PixhawkSimulator

A C++17 simulator with vehicle dynamics, battery state, controller modes, UDP
commands and telemetry, and configurable network delays and packet loss.

## Build from the host using Docker

Run these commands in a host terminal, outside the development container.
You need Bash and a running Docker daemon accessible to your user. On Windows,
use a WSL2 terminal with Docker integration enabled. No host C++ toolchain is
required. Internet access is needed to pull the image and download Boost on the
first CMake configure.

From the repository root:

```bash
bash build.sh
```

## Run directly on Linux

The Docker build produces a Linux executable for the build container's
architecture. Running it directly requires compatible system libraries,
including the C++ runtime. Boost is built statically by default. The executable
does not run natively on macOS or Windows. Running in the image above provides
the matching libraries.

```bash
./build/docker/pixhawk_simulator --config simulation_config/example.json
```

Use json config to configure simulation and inject errors - network shutdown or delay, GNSS error.

In another terminal, with Python 3 installed (runs simple client, which produces commands for simulator and prints telemetry):

```bash
python3 client.py
```
