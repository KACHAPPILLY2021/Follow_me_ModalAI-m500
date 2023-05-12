#cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 3.0)

project(human_tracking)


## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
  message_runtime
  image_transport
  cv_bridge
  #vision_opencv
)

############### J #################
add_message_files(
  DIRECTORY 
  msg
  FILES
  AiDetection.msg
)

generate_messages()

####################################

find_package(cmake_modules REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Boost REQUIRED)
find_package(Eigen 3.1.2 REQUIRED)

#INCLUDE_DIRECTORIES ("$ENV{EIGEN3_INCLUDE_DIR}")
include_directories(${Eigen_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS roscpp
)


find_library (GeographicLib_LIBRARIES Geographic
  PATHS "${CMAKE_INSTALL_PREFIX}/../GeographicLib/lib")

if (GeographicLib_LIBRARIES)
  get_filename_component (GeographicLib_LIBRARY_DIRS
    "${GeographicLib_LIBRARIES}" PATH)

  get_filename_component (_ROOT_DIR "${GeographicLib_LIBRARY_DIRS}" PATH)
  set (GeographicLib_INCLUDE_DIRS "${_ROOT_DIR}/../include")
  set (GeographicLib_BINARY_DIRS "${_ROOT_DIR}/../bin")

  # Required for Debian-alike systems
  find_library (GeographicLib_LIBRARIES_DEBIAN libGeographic.so
    PATHS "/usr/lib/x86_64-linux-gnu")
  if (GeographicLib_LIBRARIES_DEBIAN)
    set (GeographicLib_LIBRARIES ${GeographicLib_LIBRARIES_DEBIAN})
    get_filename_component (GeographicLib_LIBRARY_DIRS
      "${GeographicLib_LIBRARIES}" PATH)
  endif ()
  endif ()

catkin_package(
  DEPENDS Boost Eigen roscpp
  INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs 
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/follower.cpp )

add_executable(view src/view.cpp )



####### J
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
######

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )

####### J
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(${Boost_INCLUDE_DIRS})

 target_link_libraries(view
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${Boost_LIBRARIES}
 ) 




