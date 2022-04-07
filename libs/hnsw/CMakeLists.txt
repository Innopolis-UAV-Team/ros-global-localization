cmake_minimum_required (VERSION 3.5)
project (hnsw_lib)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
else()
    message(FATAL_ERROR "no OpenMP supprot")
endif()

add_library(HNSW_graph_search INTERFACE)
target_include_directories(HNSW_graph_search INTERFACE hnswlib)


if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Ofast -DNDEBUG -std=c++14 -DHAVE_CXX0X -openmp -#march=native -fpic -ftree-vectorize")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Ofast -lrt -DNDEBUG -std=c++14 -DHAVE_CXX0X -#march=native -fpic -w -fopenmp -ftree-vectorize -ftree-vectorizer-verbose=0" )
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  set( CMAKE_CXX_FLAGS  "-Ofast -lrt -DNDEBUG -std=c++14 -DHAVE_CXX0X -openmp -march=native -fpic -w -fopenmp -ftree-vectorize" )
endif()

