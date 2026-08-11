# The engine. No third-party runtime dependencies, so the same sources
# build natively, into a Python extension, and into WebAssembly.
add_library(interception_core
    src/rng.cpp
    src/params.cpp
    src/obstacle.cpp
    src/collision.cpp
    src/actor.cpp
    src/guidance.cpp
    src/agent.cpp
    src/target.cpp
    src/state.cpp
    src/simulation.cpp
    src/telemetry.cpp
    src/analysis.cpp
)
add_library(interception::core ALIAS interception_core)

target_include_directories(interception_core
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(interception_core PUBLIC cxx_std_17)

set_target_properties(interception_core PROPERTIES
    POSITION_INDEPENDENT_CODE ON   # required for the Python extension
)

if(MSVC)
    target_compile_options(interception_core PRIVATE /W4)
else()
    target_compile_options(interception_core PRIVATE -Wall -Wextra -Wpedantic)
endif()
