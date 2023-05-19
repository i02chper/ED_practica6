cmake_minimum_required (VERSION 3.1)
project (wgraph VERSION 1.0 LANGUAGES CXX)

enable_language(CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb3")
include_directories(".")

add_library(wgraph STATIC matrix.hpp matrix.cpp city.hpp city.cpp)
link_libraries(wgraph)

add_executable(test_wgraph test_wgraph.cpp wgraph.hpp wgraph_imp.hpp item.hpp city.hpp matrix.hpp)


add_executable(test_create_graph test_create_graph.cpp wgraph.hpp wgraph_imp.hpp item.hpp city.hpp matrix.hpp)

