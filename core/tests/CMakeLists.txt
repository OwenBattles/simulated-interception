include(FetchContent)

# doctest's own CMakeLists declares a cmake_minimum_required below 3.5,
# which CMake 4 refuses outright. This tells CMake to treat older
# declarations as 3.5 for the dependency; it does not affect this project's
# own policy settings.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

FetchContent_Declare(
    doctest
    GIT_REPOSITORY https://github.com/doctest/doctest.git
    GIT_TAG v2.4.12
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(doctest)

add_executable(interception_tests
    main.cpp
    test_vector.cpp
    test_rng.cpp
    test_collision.cpp
    test_dynamics.cpp
    test_guidance.cpp
    test_evader.cpp
    test_simulation.cpp
)

target_link_libraries(interception_tests PRIVATE interception::core doctest::doctest)
target_include_directories(interception_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_test(NAME interception_tests COMMAND interception_tests)
