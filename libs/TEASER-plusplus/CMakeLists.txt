cmake_minimum_required(VERSION 3.10)
project(teaserpp VERSION 1.0.1)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})

# Check build types
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE
            STRING "Choose the type of build." FORCE)
endif ()

# Options
option(BUILD_TEASER_FPFH "Build TEASER++ wrappers for PCL FPFH estimation." ON)
option(BUILD_WITH_MARCH_NATIVE "Build with flag march=native" OFF)
option(ENABLE_DIAGNOSTIC_PRINT "Enable printing of diagnostic messages" ON)

if (ENABLE_DIAGNOSTIC_PRINT)
    message(STATUS "Enable printing of diagnostic messages.")
    add_definitions(-DTEASER_DIAG_PRINT)
endif ()

# Find dependencies
# Eigen3
find_package(Eigen3 3.2 QUIET REQUIRED NO_MODULE)

if (BUILD_TEASER_FPFH)
    # Boost
    find_package(Boost 1.58 QUIET REQUIRED)

    # PCL
    find_package(PCL 1.8 QUIET REQUIRED COMPONENTS common io features kdtree)
endif ()


add_subdirectory(teaser)
