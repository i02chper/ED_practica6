cmake_minimum_required (VERSION 3.1)
project (wgraph_paths VERSION 1.0 LANGUAGES CXX)

enable_language(CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb3")
include_directories(".")

add_subdirectory(wgraph)
include_directories("wgraph")

add_executable(test_dijkstra test_dijkstra.cpp dijkstra_algorithm.hpp dijkstra_algorithm_imp.hpp)
target_link_libraries(test_dijkstra wgraph)
target_compile_definitions(test_dijkstra PRIVATE -D__VERBOSE__=1)

add_executable(test_floyd test_floyd.cpp floyd_algorithm.hpp floyd_algorithm_imp.hpp)
target_link_libraries(test_floyd wgraph)
target_compile_definitions(test_floyd PRIVATE -D__VERBOSE__=1)

