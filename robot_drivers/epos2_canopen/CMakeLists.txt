cmake_minimum_required(VERSION 3.0.2)
project(epos2_canopen)
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
    roscpp
)
find_package(Boost REQUIRED COMPONENTS thread chrono)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LELY REQUIRED 
    liblely-can 
    liblely-co 
    liblely-coapp 
    liblely-ev 
    liblely-io2 
    liblely-libc 
    liblely-tap 
    liblely-util
)


catkin_package(
 INCLUDE_DIRS include ${LELY_INCLUDE_DIRS}
 LIBRARIES epos2_canopen ${LELY_LIBRARIES}
 CATKIN_DEPENDS roscpp
#  DEPENDS PkgConfig
)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${LELY_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
    src/master.cpp
    src/master_ros.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${LELY_LIBRARIES} ${catkin_LIBRARIES})


add_executable(${PROJECT_NAME}_node src/main.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROJECT_NAME}
)
