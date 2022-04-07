cmake_minimum_required(VERSION 3.10)
project(FastGlobalRegistation VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 14)

#find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

add_library(fast_global_registration STATIC src/app.cpp)

#target_link_libraries(fast_global_registration PUBLIC Eigen3::Eigen)
target_link_libraries(fast_global_registration PRIVATE ${PCL_LIBRARIES})

target_include_directories(fast_global_registration PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include)
