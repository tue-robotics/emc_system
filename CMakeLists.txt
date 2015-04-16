cmake_minimum_required(VERSION 2.8.3)
project(emc_system)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    sensor_msgs
    roscpp
)

# find_package(Boost REQUIRED COMPONENTS system program_options)
# find_package(PCL REQUIRED)
# find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate messages
# add_message_files(
#    FILES
#    message1.msg
#    ...
# )

# Generate services
# add_service_files(
#    FILES
#    service1.srv
#    ...
# )

# Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES
#    geometry_msgs
#    ...
# )

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bla
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(emc_system
    include/emc/engine.h
    include/emc/communication.h
    include/emc/data.h

    src/engine.cpp
    src/communication.cpp
    src/data.cpp
)
target_link_libraries(emc_system ${catkin_LIBRARIES})

# ------------------------------------------------------------------------------------------------
#                                             EXAMPLES
# ------------------------------------------------------------------------------------------------

add_executable(emc_example1 examples/example1.cpp)
target_link_libraries(emc_example1 emc_system)


