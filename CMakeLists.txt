cmake_minimum_required(VERSION 3.15)
project(control_systems C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(CS_BUILD_TESTS "Build control systems tests" ON)

add_compile_definitions(_GNU_SOURCE)

set(ATTITUDE_MATH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/attitudeMathLibrary")

if(EXISTS "${ATTITUDE_MATH_DIR}/CMakeLists.txt")
    add_subdirectory(${ATTITUDE_MATH_DIR} ${CMAKE_BINARY_DIR}/attitudeMathLibrary)
    set(HAVE_ATTITUDE 1)
else()
    set(HAVE_ATTITUDE 0)
endif()

include_directories(include)

set(CS_SOURCES
    src/pid.c
    src/lqr.c
    src/mpc.c
    src/robust_control.c
    src/utils.c
    src/mixer.c
)

add_library(control_systems STATIC ${CS_SOURCES})

if(HAVE_ATTITUDE)
    target_link_libraries(control_systems PUBLIC attitude)
    target_include_directories(control_systems PUBLIC ${ATTITUDE_MATH_DIR}/include)
endif()

target_link_libraries(control_systems PUBLIC m)

add_executable(attitude_hold_example examples/attitude_hold/main.c)
target_link_libraries(attitude_hold_example PRIVATE control_systems m)

if(CS_BUILD_TESTS)
    enable_testing()
    add_executable(test_pid tests/test_pid.c)
    target_link_libraries(test_pid PRIVATE control_systems m)
    add_test(NAME test_pid COMMAND test_pid)

    add_executable(test_mixer tests/test_mixer.c)
    target_link_libraries(test_mixer PRIVATE control_systems m)
    add_test(NAME test_mixer COMMAND test_mixer)
endif()
