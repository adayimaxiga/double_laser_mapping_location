cmake_minimum_required(VERSION 2.8.3)
project(loop_closer_check)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_msgs
    tf2_ros
    eigen_conversions
    geometry_msgs
    message_runtime
    nav_msgs
    pcl_conversions
    roscpp
    roslib
    tf2_eigen

)


set(PACKAGE_DEPENDENCIES
        cartographer_ros_msgs
        eigen_conversions
        geometry_msgs
        message_runtime
        nav_msgs
        pcl_conversions
        roscpp
        roslib
        sensor_msgs
        std_msgs
        tf2
        tf2_eigen
        tf2_ros
        )

find_package(cartographer REQUIRED)

include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
set(BUILD_SHARED_LIBS OFF)


set(GTEST_FOUND TRUE)
set(GTEST_INCLUDE_DIRS ${GMOCK_INCLUDE_DIRS})
set(GTEST_LIBRARIES ${GMOCK_LIBRARIES})

find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams)


catkin_package(
        CATKIN_DEPENDS
        ${PACKAGE_DEPENDENCIES}
        DEPENDS
        # TODO(damonkohler): This should be here but causes Catkin to abort because
        # protobuf specifies a library '-lpthread' instead of just 'pthread'.
        # CARTOGRAPHER
        PCL
        EIGEN3
        Boost
        INCLUDE_DIRS "."
        LIBRARIES ${PROJECT_NAME}
)

file(GLOB_RECURSE ALL_SRCS "*.cc" "*.h")
file(GLOB_RECURSE ALL_TESTS "*_test.cc")
file(GLOB_RECURSE ALL_EXECUTABLES "*_main.cc")

add_library(${PROJECT_NAME} ${ALL_SRCS})

target_link_libraries(${PROJECT_NAME} PUBLIC cartographer)

# PCL
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
set(BLACKLISTED_PCL_DEFINITIONS " -march=native -msse4.2 -mfpmath=sse ")
foreach(DEFINITION ${PCL_DEFINITIONS})
    list (FIND BLACKLISTED_PCL_DEFINITIONS "${DEFINITION}" DEFINITIONS_INDEX)
    if (${DEFINITIONS_INDEX} GREATER -1)
        continue()
    endif()
    set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${DEFINITION}")
endforeach()

# Eigen
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
        "${EIGEN3_INCLUDE_DIR}")

# Boost
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
        "${Boost_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})

# Catkin
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
        include/loop_closer_check
        )

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES
        COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})


install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )


install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

# Install source headers.
file(GLOB_RECURSE HDRS "*.h")
foreach(HDR ${HDRS})
    file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
    get_filename_component(INSTALL_DIR ${REL_FIL} DIRECTORY)
    install(
            FILES
            ${HDR}
            DESTINATION
            include/${INSTALL_DIR}
    )
endforeach()

google_binary(loop_closure_node
        SRCS
        src/node_main.cpp
        src/ceres_scan_matcher_2d.cc
        src/correlative_scan_matcher_2d.cc
        src/fast_correlative_scan_matcher_2d.cc
        src/real_time_correlative_scan_matcher_2d.cc
        )

install(TARGETS loop_closure_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )