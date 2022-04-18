cmake_minimum_required(VERSION 3.0.0)
project(hapcl_registration)

set (CMAKE_CXX_STANDARD 14)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE
            STRING "Choose the type of build." FORCE)
endif ()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    sensor_msgs
    geometry_msgs
    std_srvs
    tf2_ros
)

catkin_package(
    CATKIN_DEPENDS sensor_msgs geometry_msgs std_srvs
)

find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########
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

# Build node
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
add_executable(node src/node.cpp)
add_dependencies(node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(node
    PUBLIC ${catkin_LIBRARIES}
    PUBLIC ${PCL_LIBRARIES}
    PUBLIC Eigen3::Eigen
    PRIVATE fast_global_registration
    PRIVATE WHI_feature
    PRIVATE teaser_registration
    PRIVATE HNSW_graph_search
)
