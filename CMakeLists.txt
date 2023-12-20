cmake_minimum_required(VERSION 3.10.0)
file(STRINGS VERSION CURRENT_VERSION)
project(data_logger VERSION ${CURRENT_VERSION})
set(CMAKE_POLICY_DEFAULT_CMP0048 NEW) # Suppress GTest warnings.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INSTALL_DEFAULT_COMPONENT_NAME ${PROJECT_NAME})

find_package(Eigen3 REQUIRED)
find_package(PCL 1.12.2 REQUIRED EXACT)
find_package(
  catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
)

add_executable(cloud_logger_node ${CMAKE_CURRENT_SOURCE_DIR}/src/cloud_logger_node.cpp)
target_include_directories(cloud_logger_node
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)
target_link_libraries(cloud_logger_node
    PRIVATE
        Eigen3::Eigen
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)