# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32-poolmon)

# Ignore clang pragmas that keep CLion happy
target_compile_options(main PRIVATE -Wno-unknown-pragmas)

# Ignore false warnings about `struct foo = { 0 }`
target_compile_options(main PRIVATE -Wno-missing-braces -Wmissing-field-initializers)

execute_process(COMMAND "date" "+%Y%m%d-%H%M%S"
    RESULT_VARIABLE date_result
    OUTPUT_VARIABLE date_output
    OUTPUT_STRIP_TRAILING_WHITESPACE)

# GIT_COMMIT := $(shell git describe --match=NeVeRmAtCh --always --abbrev=14 --dirty)
execute_process(COMMAND "git" "describe" "--match=NeVeRmAtCh" "--always" "--abbrev=14" "--dirty"
    RESULT_VARIABLE git_result
    OUTPUT_VARIABLE git_output
    OUTPUT_STRIP_TRAILING_WHITESPACE)

message(STATUS "BUILD_TIMESTAMP [${date_result}]: ${date_output}")
message(STATUS "GIT_COMMIT [${git_result}]: ${git_output}")

target_compile_definitions(main PUBLIC BUILD_TIMESTAMP="${date_output}")
target_compile_definitions(main PUBLIC GIT_COMMIT="${git_output}")
