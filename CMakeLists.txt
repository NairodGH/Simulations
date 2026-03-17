cmake_minimum_required(VERSION 4.0)
project(Simulations LANGUAGES CXX VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(MSVC)
    add_compile_options(/O2 /arch:AVX2)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    add_compile_options(-O2 -mavx2)
endif()

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

FetchContent_Declare(
    raylib
    URL https://github.com/raysan5/raylib/archive/refs/tags/5.5.tar.gz)
FetchContent_MakeAvailable(raylib)

FetchContent_Declare(
    Eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 5.0.0
    GIT_PROGRESS TRUE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(Eigen)

set(LIBS raylib Eigen3::Eigen)

set(TARGETS toolpath kinematic soup cad ppi)

foreach(T IN LISTS TARGETS)
    add_executable(${T} src/${T}.cpp)
    set_target_properties(${T} PROPERTIES EXCLUDE_FROM_ALL TRUE)
    target_include_directories(${T} PRIVATE include)
    target_link_libraries(${T} PRIVATE ${LIBS})
endforeach()

# Share a single PCH compilation across all targets
list(GET TARGETS 0 FIRST_TARGET)
target_precompile_headers(${FIRST_TARGET} PRIVATE include/main.hpp)
foreach(T IN LISTS TARGETS)
    if(NOT T STREQUAL FIRST_TARGET)
        target_precompile_headers(${T} REUSE_FROM ${FIRST_TARGET})
    endif()
endforeach()