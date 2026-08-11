# The Python extension. Off by default so a plain native build needs no
# pybind11; scikit-build-core turns it on when building the wheel.
if(NOT INTERCEPTION_BUILD_PYTHON)
    return()
endif()

find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_core src/bindings.cpp)
target_link_libraries(_core PRIVATE interception::core)

install(TARGETS _core DESTINATION interception)
