# cmake_minimum_required(VERSION 2.8.3)
# project(fast_lio)

# SET(CMAKE_BUILD_TYPE "Debug")

# ADD_COMPILE_OPTIONS(-std=c++14 )
# ADD_COMPILE_OPTIONS(-std=c++14 )
# set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

# add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")


cmake_minimum_required(VERSION 2.8.3)
project(fast_lio)

# SET(CMAKE_BUILD_TYPE "Release")

# ADD_COMPILE_OPTIONS(-std=c++17 )
# ADD_COMPILE_OPTIONS(-std=c++14 )
# set( CMAKE_CXX_FLAGS "-std=c++17 -O3" ) 

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")


# CONSIDERATIONS IF USING SERIAL BAG MODE
# Multithreading has two parts that will not be compatible
# because messages are fed in really fast. 
# So you have to make sure to disable them. 
# a) We disable OpenMP and set processor number=1
# b) in ikd tree, there is a DMulti_Thread_Rebuild_Point_Num
# parameter, that has to be set to a really high number. 
# So you would do 
# catkin build  --cmake-args -DMP_PROC_NUM_OVERRIDE=1 -DMULTI_THREAD_REBUILD_POINT_NUM=100000000
if(NOT DEFINED MP_PROC_NUM_OVERRIDE)
  message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
    include(ProcessorCount)
    ProcessorCount(N)
    message("Processer number:  ${N}")
    if(N GREATER 4)
      add_definitions(-DMP_EN)
      add_definitions(-DMP_PROC_NUM=3)
      message("core for MP: 3")
    elseif(N GREATER 3)
      add_definitions(-DMP_EN)
      add_definitions(-DMP_PROC_NUM=2)
      message("core for MP: 2")
    else()
      add_definitions(-DMP_PROC_NUM=1)
    endif()
  else()
    add_definitions(-DMP_PROC_NUM=1)
endif()
else()
  if(MP_PROC_NUM_OVERRIDE GREATER 1)
    add_definitions(-DMP_EN)
  endif()
  add_definitions(-DMP_PROC_NUM=${MP_PROC_NUM_OVERRIDE})
endif()

if (NOT DEFINED MULTI_THREAD_REBUILD_POINT_NUM)
set(MULTI_THREAD_REBUILD_POINT_NUM 1500)
endif()
add_definitions(-DMulti_Thread_Rebuild_Point_Num=${MULTI_THREAD_REBUILD_POINT_NUM})



# add_definitions(-DMP_PROC_NUM=1) 
# add_definitions(-UMP_EN)

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver
  message_generation
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include)

add_message_files(
  FILES
  Pose6D.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

add_executable(fastlio_mapping src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)
target_link_libraries(fastlio_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(fastlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})