cmake_minimum_required(VERSION 3.5)
project(emc_system)

set(CMAKE_CXX_STANDARD 14)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# find_package(Boost REQUIRED COMPONENTS system program_options)
# find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_library(emc_system
    #include/emc/engine.h
    include/emc/communication.h
    include/emc/data.h
    include/emc/odom.h
    include/emc/io.h
    include/emc/rate.h
    include/emc/bumper.h
    include/emc/ros2subscriber.h
    include/emc/ros2publisher.h

    #src/engine.cpp
    src/communication.cpp
    #src/data.cpp
    src/io.cpp
    src/rate.cpp
)
ament_target_dependencies(emc_system geometry_msgs sensor_msgs visualization_msgs nav_msgs rclcpp tf2 tf2_ros)

# ------------------------------------------------------------------------------------------------
#                                              TOOLS
# ------------------------------------------------------------------------------------------------

add_executable(emc_viz tools/visualize.cpp)
target_link_libraries(emc_viz emc_system ${OpenCV_LIBRARIES})

# ------------------------------------------------------------------------------------------------
#                                               IO
# ------------------------------------------------------------------------------------------------

add_executable(emc_test_io examples/test_io.cpp)
target_link_libraries(emc_test_io emc_system)

add_executable(emc_test_speech examples/testspeech.cpp)
target_link_libraries(emc_test_speech emc_system)

# ------------------------------------------------------------------------------------------------
#                                             EXAMPLES
# ------------------------------------------------------------------------------------------------

add_executable(emc_example1 examples/example01.cpp)
target_link_libraries(emc_example1 emc_system)

add_executable(emc_test_subscriber examples/test_subscriber.cpp)
target_link_libraries(emc_test_subscriber emc_system)

add_executable(emc_test_publisher examples/test_publisher.cpp)
target_link_libraries(emc_test_publisher emc_system)

#add_executable(emc_example2 examples/example02.cpp)
#target_link_libraries(emc_example2 emc_system)

# ------------------------------------------------------------------------------------------------
#                                             UNIT TESTS
# ------------------------------------------------------------------------------------------------

#if(BUILD_TESTING)
#    find_package(rostest REQUIRED)
#    add_rostest_gtest(tests_io test/io.test test/test_io.cpp)
#    target_link_libraries(tests_io emc_system)
#    ament_target_dependencies(emc_system geometry_msgs sensor_msgs nav_msgs roscpp tf2 tf2_ros rostest)
#endif()

#############
## Install ##
#############
install(TARGETS emc_example1 emc_test_io emc_test_speech emc_test_subscriber emc_test_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
