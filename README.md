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
./build/docker/pixhawk_simulator
```

In another terminal, with Python 3 installed:

```bash
python3 client.py
```

## Configuration

The default UDP endpoints are:

| Purpose | Address |
| --- | --- |
| Automatic commands | `127.0.0.1:14560` |
| Manual commands | `127.0.0.1:14561` |
| Telemetry destination | `127.0.0.1:14550` |

No configuration file is required. To load the supplied example or inspect
command-line options:

```bash
./build/docker/pixhawk_simulator --config simulation_config/example.json
./build/docker/pixhawk_simulator --help
```

When running in Docker, append these same arguments after
`./build/docker/pixhawk_simulator` in the `docker run` command.

[`simulation_config/example.json`](simulation_config/example.json) demonstrates
model and battery settings, readiness timing, GNSS outages, UDP endpoints,
optional syslog output, and network delay/drop filters. It enables simulated
faults and syslog output; running without it uses the defaults.
Command-line options override configuration-file values, for example:

```bash
./build/docker/pixhawk_simulator \
  --bind-port 14660 --peer-port 14650 --outbound-latency-ms 50
python3 client.py --peer-port 14660 --bind-port 14650
```

Run the simulator and client commands in separate terminals. Stop the simulator
before starting another instance on the same listening ports.

## Build without Docker

For a native Ubuntu 24.04 build, install the dependencies and use a separate
build directory:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake ninja-build ca-certificates python3
cmake -S . -B build/native -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake --build build/native --parallel 2
./build/native/pixhawk_simulator
```

Other environments need a C++17 compiler, CMake 3.20 or newer, internet access
for the initial Boost download, and GoogleTest when tests are enabled. The UDP
transport uses POSIX sockets, so use Linux for the documented build and run flow.
