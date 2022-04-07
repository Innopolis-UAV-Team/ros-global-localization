project(teaser_source)

# teaser_registration library
add_library(teaser_registration STATIC
        src/registration.cc
        src/graph.cc
        )
target_link_libraries(teaser_registration
        PUBLIC Eigen3::Eigen
        PRIVATE pmc
        )
target_include_directories(teaser_registration PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(teaser_registration PRIVATE OpenMP::OpenMP_CXX)
endif()

# teaser_features library
if (BUILD_TEASER_FPFH)
    add_library(teaser_features STATIC
            src/fpfh.cc
            src/matcher.cc
            )
    target_link_libraries(teaser_features
            PUBLIC ${PCL_LIBRARIES}
            PRIVATE Eigen3::Eigen
            )
    target_include_directories(teaser_features PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif ()

# march=native flag
if (BUILD_WITH_MARCH_NATIVE)
    message(STATUS "-march=native flag enabled.")
    target_compile_options(teaser_registration PUBLIC -march=native)
endif ()
