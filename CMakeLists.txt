cmake_minimum_required(VERSION 3.28)
project(Koala25Manager VERSION 1.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
# GoogleTest is C++: only the tests are.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# clang-tidy reads this: without it the FetchContent headers are not found.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_TESTING "Build tests" ON)

function(warn_as_errors target)
    if(NOT MSVC)
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
endfunction()

# Only the part of the Koala library that needs no robot. The rest of src/ includes termio.h and opens
# /dev/ttyS1, so it builds on the robot's Linux with src/Makefile and nowhere else.
add_library(koala_parse STATIC src/koala_parse.c)
target_include_directories(koala_parse PUBLIC src)
warn_as_errors(koala_parse)

if(BUILD_TESTING)
    enable_testing()

    # An archive and its hash, not a git clone: nothing here can ask for a credential on the runner.
    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.13.0.tar.gz
        URL_HASH SHA256=ad7fdba11ea011c1d925b3289cf4af2c66a352e18d4c7264392fead75e919363
        SYSTEM
    )
    set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    add_executable(koala_parse_tests tests/koala_parse_test.cpp)
    target_link_libraries(koala_parse_tests PRIVATE koala_parse GTest::gtest_main)
    warn_as_errors(koala_parse_tests)

    add_test(NAME koala_parse_tests COMMAND koala_parse_tests)
    # The manual list is judged where every other file is judged (section 9).
    add_test(
        NAME manual_list
        COMMAND python -m pytest tests/test_manual_list.py -q -p no:cacheprovider
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
endif()
