cmake_minimum_required(VERSION 3.10)
project(pmc VERSION 1.0.1)

set(PMC_SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/pmc_heu.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/pmc_maxclique.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/pmcx_maxclique.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/pmcx_maxclique_basic.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/pmc_cores.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/pmc_utils.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/pmc_graph.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/pmc_clique_utils.cpp
        )

add_library(pmc STATIC ${PMC_SOURCE_FILES})

target_include_directories(pmc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

find_package(OpenMP REQUIRED)
target_link_libraries(pmc OpenMP::OpenMP_CXX)

