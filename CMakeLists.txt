cmake_minimum_required(VERSION 3.20)
project(interception VERSION 0.4.0 LANGUAGES CXX)

# Reproducibility is a feature of this simulator, so the build must not
# trade it away. -ffast-math permits reassociation and flush-to-zero, which
# would make results depend on optimisation level and target architecture
# and break parity with the Python reference. It is never enabled here.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(INTERCEPTION_BUILD_TESTS "Build the C++ unit tests" ON)

add_subdirectory(core)

if(INTERCEPTION_BUILD_TESTS AND NOT EMSCRIPTEN)
    enable_testing()
    add_subdirectory(core/tests)
endif()
