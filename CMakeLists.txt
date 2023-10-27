cmake_minimum_required(VERSION 2.8)
project(world_only_line)

add_subdirectory(third_party/pybind11)
pybind11_add_module(world_only_line ./world_only_line.cpp)

find_package(Eigen3 REQUIRED)
if((NOT TARGET Eigen3::Eigen) AND (DEFINED EIGEN3_INCLUDE_DIR))
    add_library(AliasEigen3 INTERFACE)
    target_include_directories(AliasEigen3 INTERFACE ${EIGEN3_INCLUDE_DIR})
    add_library(Eigen3::Eigen ALIAS AliasEigen3)
endif()

target_include_directories(world_only_line PRIVATE ./inc)

target_compile_definitions(world_only_line
                           PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})