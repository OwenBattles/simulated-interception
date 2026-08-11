# The WebAssembly target. Only defined under an Emscripten toolchain:
#   emcmake cmake -S . -B build-wasm -DINTERCEPTION_BUILD_TESTS=OFF
#   cmake --build build-wasm
# which emits web/public/interception.{js,wasm}.
if(NOT EMSCRIPTEN)
    return()
endif()

add_executable(interception_wasm src/bindings.cpp)
target_link_libraries(interception_wasm PRIVATE interception::core)

set_target_properties(interception_wasm PROPERTIES
    OUTPUT_NAME interception
    SUFFIX ".js"
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/web/public
)

target_link_options(interception_wasm PRIVATE
    -lembind
    -O3
    # A factory function rather than a global, so the page controls when
    # the module loads and there is nothing to collide with.
    "SHELL:-s MODULARIZE=1"
    "SHELL:-s EXPORT_NAME=createInterceptionModule"
    "SHELL:-s ENVIRONMENT=web"
    "SHELL:-s ALLOW_MEMORY_GROWTH=1"
    # Fail loudly instead of corrupting memory if a binding is misused.
    "SHELL:-s ASSERTIONS=1"
)
