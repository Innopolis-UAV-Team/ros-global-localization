cmake_minimum_required(VERSION 3.10)
project(WHI VERSION 0.0.0)

add_library(WHI_feature STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/WHI.cpp)
target_include_directories(WHI_feature PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)			        
target_link_libraries(WHI_feature PUBLIC ${PCL_LIBRARIES} 
			                      PUBLIC ${OpenCV_LIBRARIES})
