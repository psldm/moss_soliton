# Build and Compilation Guide

## Overview

This guide provides comprehensive instructions for building the Moss Stability SDK from source on various platforms. The SDK supports multiple build systems and optimization levels for different deployment scenarios.

## Quick Build

### Basic Build

```bash
cd Moss_Stability_SDK_C
make all
```

This produces:
- `libmoss_stability.a` - Static library
- `moss_sim` - Flight simulation demo
- `test_ternary_core` - Ternary core tests
- `test_reason_solver` - Reason solver tests

### Run Tests

```bash
make test
```

### Clean Build Artifacts

```bash
make clean
```

## Prerequisites

### System Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **C Compiler** | GCC 7+ / Clang 6+ / MSVC 2019+ | Latest stable version |
| **Operating System** | Linux, macOS, Windows | Same as minimum |
| **Architecture** | x86_64, ARM64, ARMv7 | x86_64 with SIMD |
| **Memory** | 100 MB | 1 GB |
| **Disk Space** | 50 MB | 200 MB |

### Platform-Specific Requirements

#### Linux
```bash
# Ubuntu/Debian
sudo apt update
sudo apt install build-essential cmake

# CentOS/RHEL/Fedora
sudo yum install gcc cmake
# or
sudo dnf install gcc cmake
```

#### macOS
```bash
# Install Xcode Command Line Tools
xcode-select --install

# Install Homebrew (optional)
brew install cmake
```

#### Windows
- Visual Studio 2019 or later
- Windows SDK 10 or later
- CMake 3.15 or later

## Build Configuration

### Make Targets

| Target | Description | Output |
|--------|-------------|--------|
| `all` | Build everything | Library, demos, tests |
| `lib` | Build static library only | `libmoss_stability.a` |
| `demo` | Build demo applications | `moss_sim` |
| `test` | Build and run tests | Test executables |
| `clean` | Remove build artifacts | Clean directory |
| `install` | Install to system | `/usr/local/lib`, `/usr/local/include` |
| `uninstall` | Remove installed files | Clean system |

### Compiler Flags

The Makefile uses the following compiler configurations:

```makefile
CC = gcc
CFLAGS = -Wall -Wextra -O3 -I./include
LDFLAGS = -lm
```

**Flag Explanations**:
- `-Wall -Wextra`: Enable comprehensive warnings
- `-O3`: Maximum optimization level
- `-I./include`: Include path for headers
- `-lm`: Link against math library

### Build Variants

#### Debug Build
```bash
make clean
make CFLAGS="-g -O0 -DDEBUG -fsanitize=address" all
```

#### Release Build
```bash
make clean
make CFLAGS="-O3 -DNDEBUG -flto -march=native" all
```

#### Minimal Build
```bash
make clean
make CFLAGS="-Os -flto" all
```

#### SIMD-Optimized Build
```bash
make clean
make CFLAGS="-O3 -march=native -mtune=native -funroll-loops" all
```

## Cross-Compilation

### ARM64 (AArch64)
```bash
make clean
make CC="aarch64-linux-gnu-gcc" \
     CFLAGS="-O3 -march=native -I./include" \
     LDFLAGS="-lm" \
     all
```

### ARMv7 (32-bit)
```bash
make clean
make CC="arm-linux-gnueabihf-gcc" \
     CFLAGS="-O3 -march=armv7-a -mfpu=neon -I./include" \
     LDFLAGS="-lm" \
     all
```

### Raspberry Pi
```bash
# For Raspberry Pi 4
make clean
make CC="gcc" \
     CFLAGS="-O3 -march=armv8-a+fp+simd -mfpu=neon -I./include" \
     LDFLAGS="-lm" \
     all
```

## Build Scripts

### Automated Build Script

Create `build.sh`:

```bash
#!/bin/bash
set -e

echo "Building Moss Stability SDK..."

# Parse command line arguments
BUILD_TYPE="release"
CROSS_COMPILE=""
CLEAN_BUILD=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --debug)
            BUILD_TYPE="debug"
            shift
            ;;
        --release)
            BUILD_TYPE="release"
            shift
            ;;
        --clean)
            CLEAN_BUILD="yes"
            shift
            ;;
        --cross-compile)
            CROSS_COMPILE="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [--debug|--release] [--clean] [--cross-compile PREFIX]"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Set compiler flags based on build type
case $BUILD_TYPE in
    debug)
        CFLAGS="-g -O0 -DDEBUG -fsanitize=address -Wall -Wextra"
        ;;
    release)
        CFLAGS="-O3 -DNDEBUG -flto -march=native -Wall -Wextra"
        ;;
esac

# Clean build if requested
if [ "$CLEAN_BUILD" = "yes" ]; then
    echo "Cleaning build artifacts..."
    make clean
fi

# Configure cross-compilation if requested
if [ -n "$CROSS_COMPILE" ]; then
    CC="${CROSS_COMPILE}gcc"
    CFLAGS="$CFLAGS -I./include"
    LDFLAGS="-lm"
    echo "Cross-compiling with: $CC"
else
    CC=gcc
    CFLAGS="$CFLAGS -I./include"
    LDFLAGS="-lm"
fi

# Build
echo "Building with flags: $CFLAGS"
make CC="$CC" CFLAGS="$CFLAGS" LDFLAGS="$LDFLAGS" all

# Run tests
echo "Running tests..."
make test

echo "Build completed successfully!"
```

Usage:
```bash
chmod +x build.sh
./build.sh --release           # Release build
./build.sh --debug             # Debug build
./build.sh --clean --release   # Clean release build
./build.sh --cross-compile aarch64-linux-gnu- # ARM64 cross-compile
```

### Platform-Specific Scripts

#### Linux Build Script (`build-linux.sh`)
```bash
#!/bin/bash
# Optimized for Linux development

# Check for required tools
command -v gcc >/dev/null 2>&1 || { echo "gcc is required but not installed. Aborting." >&2; exit 1; }

# Enable maximum optimizations for Linux
export CFLAGS="-O3 -march=native -mtune=native -flto -Wall -Wextra -I./include"
export LDFLAGS="-lm -flto"

make all
make test
```

#### macOS Build Script (`build-macos.sh`)
```bash
#!/bin/bash
# Optimized for macOS development

# Check for Xcode Command Line Tools
if ! xcode-select -p >/dev/null 2>&1; then
    echo "Xcode Command Line Tools required. Install with: xcode-select --install"
    exit 1
fi

# Apple Clang optimization
export CC=clang
export CFLAGS="-O3 -march=native -Wall -Wextra -I./include"
export LDFLAGS="-lm"

make all
make test
```

#### Windows Build Script (`build-windows.bat`)
```bat
@echo off
REM Windows batch file for building Moss Stability SDK

echo Building Moss Stability SDK on Windows...

REM Check for Visual Studio
where cl >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo Visual Studio C compiler not found. Please run from Visual Studio Developer Command Prompt.
    exit /b 1
)

REM Set compilation flags for Windows
set CFLAGS=/O2 /W4 /I./include /DMOSS_DLL_EXPORTS
set LDFLAGS=libm.lib

REM Build library
cl /c src\reason_solver.c %CFLAGS%
cl /c src\ternary_core.c %CFLAGS%
lib /out:libmoss_stability.lib reason_solver.obj ternary_core.obj

REM Build demo
cl examples\flight_loop_sim.c /Fe:moss_sim.exe /link libmoss_stability.lib %LDFLAGS%

REM Build tests
cl tests\test_reason_solver.c /Fe:test_reason_solver.exe /link libmoss_stability.lib %LDFLAGS%
cl tests\test_ternary_core.c /Fe:test_ternary_core.exe /link libmoss_stability.lib %LDFLAGS%

echo Build completed!
```

## CMake Integration

### Basic CMakeLists.txt

The project includes a basic CMakeLists.txt for integration with larger projects:

```cmake
cmake_minimum_required(VERSION 3.15)
project(MossStabilitySDK VERSION 1.0.0 LANGUAGES C)

# Set C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -O3")
set(CMAKE_C_FLAGS_DEBUG "-g -O0 -DDEBUG -fsanitize=address")
set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG -flto")

# Find math library
find_library(MATH_LIBRARY m)

# Source files
set(SOURCES
    src/reason_solver.c
    src/ternary_core.c
)

# Header files
set(HEADERS
    include/reason_solver.h
    include/ternary_core.h
)

# Create static library
add_library(moss_stability STATIC ${SOURCES} ${HEADERS})

# Include directories
target_include_directories(moss_stability PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link math library
target_link_libraries(moss_stability ${MATH_LIBRARY})

# Install targets
install(TARGETS moss_stability
    EXPORT MossStabilityTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)

install(FILES ${HEADERS}
    DESTINATION include
)

# Testing
enable_testing()
add_executable(test_ternary_core tests/test_ternary_core.c)
target_link_libraries(test_ternary_core moss_stability ${MATH_LIBRARY})

add_executable(test_reason_solver tests/test_reason_solver.c)
target_link_libraries(test_reason_solver moss_stability ${MATH_LIBRARY})

add_test(NAME TernaryCoreTests COMMAND test_ternary_core)
add_test(NAME ReasonSolverTests COMMAND test_reason_solver)
```

### CMake Usage

```bash
# Configure and build
mkdir build
cd build
cmake ..
make

# Run tests
ctest

# Install
sudo make install
```

## IDE Integration

### Visual Studio Code

Create `.vscode/c_cpp_properties.json`:

```json
{
    "configurations": [
        {
            "name": "Moss Stability SDK",
            "includePath": [
                "${workspaceFolder}/include",
                "${workspaceFolder}/src"
            ],
            "defines": [],
            "compilerPath": "gcc",
            "cStandard": "c99",
            "cppStandard": "c++11",
            "intelliSenseMode": "gcc-x64",
            "compileCommands": "${workspaceFolder}/compile_commands.json"
        }
    ],
    "version": 4
}
```

Create `.vscode/tasks.json`:

```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "Build",
            "type": "shell",
            "command": "make",
            "args": ["all"],
            "group": {
                "kind": "build",
                "isDefault": true
            },
            "presentation": {
                "echo": true,
                "reveal": "always",
                "focus": false,
                "panel": "shared"
            },
            "problemMatcher": "$gcc"
        },
        {
            "label": "Test",
            "type": "shell",
            "command": "make",
            "args": ["test"],
            "group": "test",
            "presentation": {
                "echo": true,
                "reveal": "always",
                "focus": false,
                "panel": "shared"
            },
            "problemMatcher": "$gcc"
        },
        {
            "label": "Clean",
            "type": "shell",
            "command": "make",
            "args": ["clean"],
            "group": "build",
            "presentation": {
                "echo": true,
                "reveal": "always",
                "focus": false,
                "panel": "shared"
            },
            "problemMatcher": "$gcc"
        }
    ]
}
```

### CLion Integration

Create `CMakeLists.txt` at project root:

```cmake
cmake_minimum_required(VERSION 3.15)
project(moss_stability_sdk)

set(CMAKE_C_STANDARD 99)

# Build configuration
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -O0 -DDEBUG -fsanitize=address")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -DNDEBUG -flto")

# Library sources
set(SOURCES
    src/reason_solver.c
    src/ternary_core.c
)

set(HEADERS
    include/reason_solver.h
    include/ternary_core.h
)

# Create library
add_library(moss_stability STATIC ${SOURCES} ${HEADERS})

# Include directories
target_include_directories(moss_stability PUBLIC include)

# Link math library
target_link_libraries(moss_stability m)

# Demo application
add_executable(moss_sim examples/flight_loop_sim.c)
target_link_libraries(moss_sim moss_stability)

# Tests
add_executable(test_ternary_core tests/test_ternary_core.c)
target_link_libraries(test_ternary_core moss_stability)

add_executable(test_reason_solver tests/test_reason_solver.c)
target_link_libraries(test_reason_solver moss_stability)
```

## Package Management

### Static Library Distribution

Create distribution package:

```bash
#!/bin/bash
# create_package.sh

VERSION="1.0.0"
PACKAGE_NAME="moss-stability-sdk-${VERSION}"

# Create package directory
mkdir -p "${PACKAGE_NAME}"/{lib,include,examples,docs,tests}

# Copy library
cp libmoss_stability.a "${PACKAGE_NAME}/lib/"

# Copy headers
cp include/*.h "${PACKAGE_NAME}/include/"

# Copy examples
cp examples/* "${PACKAGE_NAME}/examples/"

# Copy tests
cp tests/* "${PACKAGE_NAME}/tests/"

# Copy documentation
cp docs/*.md "${PACKAGE_NAME}/docs/" 2>/dev/null || true
cp README.md "${PACKAGE_NAME}/docs/"

# Create makefile for distribution
cat > "${PACKAGE_NAME}/Makefile" << EOF
CC = gcc
CFLAGS = -Wall -Wextra -O3 -I./include
LDFLAGS = -lm

TARGET_LIB = lib/moss_stability.a
SRC = src/reason_solver.c src/ternary_core.c
OBJ = \$(SRC:.c=.o)

all: \$(TARGET_LIB)

\$(TARGET_LIB): \$(OBJ)
\tmkdir -p lib
\tar rcs \$@ \$^

%.o: %.c
\t\$(CC) \$(CFLAGS) -c \$< -o \$@

clean:
\trm -f *.o src/*.o lib/*.a

.PHONY: all clean
EOF

# Create source files
cp src/*.c "${PACKAGE_NAME}/src/"

# Create tarball
tar czf "${PACKAGE_NAME}.tar.gz" "${PACKAGE_NAME}/"

echo "Package created: ${PACKAGE_NAME}.tar.gz"
```

### Conan Integration

Create `conanfile.txt`:

```
[requires]
# Add any external dependencies here

[generators]
cmake

[options]
# Build options
moss_stability:shared=False
moss_stability:simd=True
moss_stability:optimization=Release

[imports]
bin, *.exe -> ./bin # Copies exe to bin
lib, *.a -> ./lib   # Copies static libraries to lib
```

### vcpkg Integration

Create `vcpkg.json`:

```json
{
  "name": "moss-stability-sdk",
  "version": "1.0.0",
  "description": "REASON algorithm implementation for autonomous system stability control",
  "dependencies": []
}
```

## Troubleshooting

### Common Build Issues

#### Missing Math Library
```bash
# Error: undefined reference to 'sin', 'cos', etc.
# Solution: Link against math library
make LDFLAGS="-lm"
```

#### Missing Headers
```bash
# Error: fatal error: reason_solver.h: No such file or directory
# Solution: Include path
make CFLAGS="-I./include"
```

#### SIMD Not Available
```bash
# Warning: SSE2/NEON instructions not available
# Solution: Check CPU support or disable SIMD
make CFLAGS="-O3 -march=native"
```

#### Cross-Compilation Issues
```bash
# Error: incompatible compiler
# Solution: Specify correct cross-compiler
make CC="aarch64-linux-gnu-gcc"
```

### Platform-Specific Issues

#### macOS: Xcode Command Line Tools
```bash
xcode-select: error: unable to get active developer directory
# Solution:
sudo xcode-select --reset
xcode-select --install
```

#### Windows: Visual Studio
```cmd
cl: command not found
# Solution: Use "Developer Command Prompt for VS"
```

#### Linux: Missing Dependencies
```bash
# Error: /usr/bin/ld: cannot find -lm
# Solution:
sudo apt install build-essential  # Ubuntu/Debian
sudo yum install gcc glibc-devel  # CentOS/RHEL
```

### Performance Issues

#### Slow Compilation
```bash
# Use parallel compilation
make -j$(nproc) all

# Or limit parallel jobs
make -j4 all
```

#### Large Binary Size
```bash
# Enable link-time optimization
make CFLAGS="-O3 -flto -Wl,--gc-sections" all

# Strip debug symbols from release build
strip --strip-all libmoss_stability.a
```

### Testing Issues

#### Tests Fail
```bash
# Run with verbose output
./test_reason_solver --verbose

# Check for memory leaks
valgrind --leak-check=full ./test_reason_solver
```

## Continuous Integration

### GitHub Actions

Create `.github/workflows/build.yml`:

```yaml
name: Build and Test

on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main ]

jobs:
  build:
    runs-on: ${{ matrix.os }}
    strategy:
      matrix:
        os: [ubuntu-latest, macos-latest, windows-latest]
        compiler: [gcc, clang]
        include:
          - os: ubuntu-latest
            compiler: gcc
            cc: gcc
          - os: ubuntu-latest
            compiler: clang
            cc: clang
          - os: macos-latest
            compiler: clang
            cc: clang
          - os: windows-latest
            compiler: msvc
            cc: cl

    steps:
    - uses: actions/checkout@v3
    
    - name: Install dependencies (Ubuntu)
      if: matrix.os == 'ubuntu-latest'
      run: |
        sudo apt update
        sudo apt install build-essential cmake
    
    - name: Install dependencies (macOS)
      if: matrix.os == 'macos-latest'
      run: |
        brew install cmake
    
    - name: Configure
      run: |
        mkdir build && cd build
        cmake .. -DCMAKE_BUILD_TYPE=Release
    
    - name: Build
      run: |
        cd build && cmake --build .
    
    - name: Test
      run: |
        cd build && ctest --output-on-failure
```

### Docker Build

Create `Dockerfile`:

```dockerfile
FROM ubuntu:20.04

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . .

RUN make all
RUN make test

CMD ["./test_reason_solver"]
```

Build and run:
```bash
docker build -t moss-stability-sdk .
docker run --rm moss-stability-sdk
```

This comprehensive build guide covers all aspects of building the Moss Stability SDK across different platforms and configurations.