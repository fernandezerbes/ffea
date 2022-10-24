cmake_minimum_required(VERSION 3.10)

project(ffea VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_subdirectory(framework)
add_subdirectory(applications)
add_subdirectory(examples)
add_subdirectory(external)
add_subdirectory(test)
