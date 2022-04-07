cmake_minimum_required(VERSION 3.0.0)
project(Hybrid_Approach_Point_Cloud_Registration)

set (CMAKE_CXX_STANDARD 14)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE
            STRING "Choose the type of build." FORCE)
endif ()

find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)

add_subdirectory(libs/pmc)
add_subdirectory(libs/TEASER-plusplus)
add_subdirectory(libs/WHI-3D-Local-Descriptor)
add_subdirectory(libs/hnsw)
add_subdirectory(libs/Fast-Global-Registration)

add_library(PCRegistrationLib STATIC ./src/PCRegistration.cpp ./src/app_exception.cpp)

target_link_libraries(PCRegistrationLib 
	PUBLIC ${PCL_LIBRARIES} 
	PUBLIC Eigen3::Eigen
	PRIVATE fast_global_registration
	PRIVATE WHI_feature
	PRIVATE teaser_registration
	PRIVATE HNSW_graph_search
)

target_include_directories(PCRegistrationLib PUBLIC ./include)
	
add_executable(HAPCR ./test/main.cpp)

target_link_libraries(HAPCR PCRegistrationLib)


