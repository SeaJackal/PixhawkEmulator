Install a C++17 compiler, CMake 3.20 or newer, and GoogleTest (for example,
`libgtest-dev` on Debian/Ubuntu). From the repository root, run:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
```

This builds all modules and runs all project tests. Add
`-R '^DynamicModelTest\.'` to the CTest command to run only dynamic model tests.
Configure with `-DBUILD_TESTING=OFF` to build the libraries without tests.
