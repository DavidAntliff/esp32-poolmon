# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32-poolmon)

# Ignore clang pragmas that keep CLion happy
target_compile_options(${IDF_PROJECT_EXECUTABLE} PRIVATE -Wno-unknown-pragmas)

# Ignore false warnings about `struct foo = { 0 }`
target_compile_options(${IDF_PROJECT_EXECUTABLE} PRIVATE -Wno-missing-braces -Wmissing-field-initializers)
