cmake_minimum_required(VERSION 4.0.0)
project(Simulations LANGUAGES CXX VERSION 1.0)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

FetchContent_Declare(
    raylib
    URL https://github.com/raysan5/raylib/archive/refs/tags/5.5.tar.gz
    GIT_PROGRESS TRUE)
FetchContent_MakeAvailable(raylib)

FetchContent_Declare(
    Eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 5.0.0
    GIT_PROGRESS TRUE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(Eigen)

set(CMAKE_BUILD_TYPE Release CACHE STRING "Default to Release for O2" FORCE)
if(MSVC)
    add_compile_options(/O2 /arch:AVX2)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    add_compile_options(-O2 -mavx2)
endif()

include_directories(include)

set(LIBS raylib Eigen3::Eigen)

add_executable(toolpath src/toolpath.cpp)
target_link_libraries(toolpath ${LIBS})

add_executable(kinematic src/kinematic.cpp)
target_link_libraries(kinematic ${LIBS})

add_executable(soup src/soup.cpp)
target_link_libraries(soup ${LIBS})