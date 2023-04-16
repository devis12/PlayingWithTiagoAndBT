cmake_minimum_required(VERSION 3.0.2)
project(tiago_basic_bt)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_compile_options(-g)

# Dependencies
find_package(catkin REQUIRED COMPONENTS
    roscpp
    actionlib
    geometry_msgs
    nav_msgs
    actionlib_msgs
    play_motion_msgs

    behaviortree_cpp_v3
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

# create a library target for the utility code in this project
add_library(${PROJECT_NAME}_lib 
  src/bt_actions/tiago_move_circle.cpp
  src/bt_actions/tiago_play_motion.cpp
)
target_include_directories(${PROJECT_NAME}_lib PRIVATE include)
target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(${PROJECT_NAME}_node src/tiago_bt_demo.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
