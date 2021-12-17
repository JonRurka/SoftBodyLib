cmake_minimum_required(VERSION 3.17)
project(SoftBodyLib)

set(CMAKE_CXX_STANDARD 17)
set(CONAN_DISABLE_CHECK_COMPILER true)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS 1)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

if (UNIX)
    add_compile_options(-pthread -fPIC)
endif ()

add_subdirectory(source)
