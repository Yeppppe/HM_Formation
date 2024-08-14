cmake_minimum_required(VERSION 3.0.2)
project(b2_formation_control)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++11 -O3 -fopenmp -pthread")


find_package(catkin REQUIRED COMPONENTS
    gazebo_msgs
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf
)
find_package(Matplot++ REQUIRED)

include_directories( 
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)

catkin_package()
set(CONTROL_SRC 
    src/goal.cpp
    src/b2_util.cpp
    src/formation.cpp
    src/avoidance.cpp
    src/move_robot.cpp
    src/formation_controller.cpp)

add_executable(formation_move 
    src/formation_move.cpp ${CONTROL_SRC})
target_link_libraries(formation_move ${catkin_LIBRARIES} Matplot++::matplot)

add_executable(through_corridor
    src/through_corridor.cpp ${CONTROL_SRC})
target_link_libraries(through_corridor ${catkin_LIBRARIES})

add_executable(keyboard src/keyboard.cpp)
target_link_libraries(keyboard ${catkin_LIBRARIES})

